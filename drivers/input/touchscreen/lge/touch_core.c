/*
 * touch_core.c
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the¬
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/input/lge_touch_notify.h>
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#include <soc/qcom/lge/board_lge.h>
#endif
#include "touch_core.h"

u32 touch_debug_mask = BASE_INFO;
/* Debug mask value
 * usage: echo [debug_mask] > /sys/module/touch_core/parameters/debug_mask
 */
module_param_named(debug_mask, touch_debug_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);

#define TOUCH_UEVENT_KNOCK		0
#define TOUCH_UEVENT_PASSWD		1
#define TOUCH_UEVENT_SWIPE_RIGHT	2
#define TOUCH_UEVENT_SWIPE_LEFT		3

#define TOUCH_UEVENT_SIZE		4

static void touch_send_uevent(struct touch_core_data *ts, int type);
static void touch_suspend(struct device *dev);
static void touch_resume(struct device *dev);

void touch_msleep(unsigned int msecs)
{
	if (msecs >= 20)
		msleep(msecs);
	else
		usleep_range(msecs * 1000, msecs * 1000);
}

void touch_interrupt_control(struct device *dev, int on_off)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (on_off) {
		if (atomic_cmpxchg(&ts->state.irq_enable, 0, 1) == 0) {
			touch_enable_irq(ts->irq);

			if (ts->role.use_lpwg)
				touch_enable_irq_wake(ts->irq);
		}
	} else {
		if (atomic_cmpxchg(&ts->state.irq_enable, 1, 0) == 1) {
			if (ts->role.use_lpwg)
				touch_disable_irq_wake(ts->irq);

			touch_disable_irq(ts->irq);
		}
	}
}

static void touch_report_palm_event(struct touch_core_data *ts)
{
	u16 old_mask = ts->old_mask;
	int i = 0;

	for (i = 0; i < MAX_FINGER; i++) {
		if (old_mask & (1 << i)) {
			input_mt_slot(ts->input, i);
			input_report_abs(ts->input, ABS_MT_PRESSURE,
					255);
			TOUCH_I("finger canceled:<%d>(%4d,%4d,%4d)\n",
					i,
					ts->tdata[i].x,
					ts->tdata[i].y,
					ts->tdata[i].pressure);
		}
	}

	input_sync(ts->input);
}

static void touch_report_event(struct touch_core_data *ts)
{
	u16 old_mask = ts->old_mask;
	u16 new_mask = ts->new_mask;
	u16 press_mask = 0;
	u16 release_mask = 0;
	u16 change_mask = 0;
	int i;

	TOUCH_TRACE();

	change_mask = old_mask ^ new_mask;
	press_mask = new_mask & change_mask;
	release_mask = old_mask & change_mask;

	TOUCH_D(ABS, "mask [new: %04x, old: %04x]\n",
			new_mask, old_mask);
	TOUCH_D(ABS, "mask [change: %04x, press: %04x, release: %04x]\n",
			change_mask, press_mask, release_mask);

	/* Palm state - Report Pressure value 255 */
	if (ts->is_palm) {
		touch_report_palm_event(ts);
		ts->is_palm = 0;
	}

	for (i = 0; i < MAX_FINGER; i++) {
		if (new_mask & (1 << i)) {
			input_mt_slot(ts->input, i);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID,
					ts->tdata[i].id);
			input_report_abs(ts->input, ABS_MT_POSITION_X,
					ts->tdata[i].x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
					ts->tdata[i].y);
			input_report_abs(ts->input, ABS_MT_PRESSURE,
					ts->tdata[i].pressure);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
					ts->tdata[i].width_major);
			input_report_abs(ts->input, ABS_MT_WIDTH_MINOR,
					ts->tdata[i].width_minor);
			input_report_abs(ts->input, ABS_MT_ORIENTATION,
					ts->tdata[i].orientation);

			if (press_mask & (1 << i)) {
				TOUCH_I("%d finger press:<%d>(%4d,%4d,%4d)\n",
						ts->tcount,
						i,
						ts->tdata[i].x,
						ts->tdata[i].y,
						ts->tdata[i].pressure);
			}
		} else if (release_mask & (1 << i)) {
			input_mt_slot(ts->input, i);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			TOUCH_I("finger release:<%d>(%4d,%4d,%4d)\n",
					i,
					ts->tdata[i].x,
					ts->tdata[i].y,
					ts->tdata[i].pressure);
		}
	}

	ts->old_mask = new_mask;
	input_sync(ts->input);
}

static void touch_report_all_event(struct touch_core_data *ts)
{
	if (ts->old_mask) {
		ts->new_mask = 0;
		touch_report_event(ts);
		ts->tcount = 0;
		memset(ts->tdata, 0, sizeof(struct touch_data) * MAX_FINGER);
	}
	ts->is_palm = 0;
}

static void touch_get_max_delta(struct touch_core_data *ts)
{
	struct asc_info *asc = &(ts->asc);

	if (asc->use_delta_chk == DELTA_CHK_OFF) {
		TOUCH_I("%s : DELTA_CHK is OFf\n", __func__);
		return;
	}

	if (atomic_read(&ts->state.fb) != FB_RESUME) {
		TOUCH_I("%s : fb state is not FB_RESUME\n", __func__);
		return;
	}

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		TOUCH_I("%s : core state is not CORE_NORMAL\n", __func__);
		return;
	}

	mutex_lock(&ts->lock);
	asc->delta = ts->driver->asc(ts->dev, ASC_READ_MAX_DELTA, 0);
	mutex_unlock(&ts->lock);

	asc->delta_updated = true;

	TOUCH_I("%s : delta = %d\n", __func__, asc->delta);

	return;
}

static char asc_str[3][8] = {"NORMAL", "ACUTE", "OBTUSE"};

void touch_change_sensitivity(struct touch_core_data *ts,
		int next_sensitivity)
{
	struct asc_info *asc = &(ts->asc);

	if (atomic_read(&ts->state.fb) != FB_RESUME) {
		TOUCH_I("%s : fb state is not FB_RESUME\n", __func__);
		return;
	}

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		TOUCH_I("%s : core state is not CORE_NORMAL\n", __func__);
		return;
	}

	if (asc->curr_sensitivity == next_sensitivity)
		return;

	TOUCH_I("%s : sensitity(curr->next) = (%s->%s)\n", __func__,
			asc_str[asc->curr_sensitivity],
			asc_str[next_sensitivity]);

	mutex_lock(&ts->lock);
	ts->driver->asc(ts->dev, ASC_WRITE_SENSITIVITY,
			next_sensitivity);
	mutex_unlock(&ts->lock);

	asc->curr_sensitivity = next_sensitivity;

	return;
}

static void touch_update_sensitivity(struct touch_core_data *ts)
{
	struct asc_info *asc = &(ts->asc);
	int next_sensitivity = NORMAL_SENSITIVITY;

	if (asc->use_delta_chk == DELTA_CHK_OFF) {
		TOUCH_I("%s : DELTA_CHK is OFf\n", __func__);
		return;
	}

	if (asc->delta_updated == false) {
		TOUCH_I("%s : delta is not updated.\n", __func__);
		return;
	}

	if (asc->delta < asc->low_delta_thres) {
		next_sensitivity = ACUTE_SENSITIVITY;
	} else if (asc->delta > asc->high_delta_thres) {
		next_sensitivity = OBTUSE_SENSITIVITY;
	} else {
		next_sensitivity = NORMAL_SENSITIVITY;
	}

	asc->delta_updated = false;
	touch_change_sensitivity(ts, next_sensitivity);

	return;
}

static void touch_core_initialize(struct touch_core_data *ts)
{
	/* lockscreen */
	touch_report_all_event(ts);
}

irqreturn_t touch_irq_handler(int irq, void *dev_id)
{
	struct touch_core_data *ts = (struct touch_core_data *) dev_id;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.pm) >= DEV_PM_SUSPEND) {
		TOUCH_I("interrupt in suspend[%d]\n",
				atomic_read(&ts->state.pm));
		atomic_set(&ts->state.pm, DEV_PM_SUSPEND_IRQ);
		wake_lock_timeout(&ts->lpwg_wake_lock, msecs_to_jiffies(1000));
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

irqreturn_t touch_irq_thread(int irq, void *dev_id)
{
	struct touch_core_data *ts = (struct touch_core_data *) dev_id;
	int ret = 0;

	TOUCH_TRACE();
	mutex_lock(&ts->lock);

	ts->intr_status = 0;
	ret = ts->driver->irq_handler(ts->dev); // [e2sh] lg4894_irq_handler

	if (ret >= 0) {
		if (ts->intr_status & TOUCH_IRQ_FINGER) {
			touch_report_event(ts);

			if (ts->asc.use_delta_chk == DELTA_CHK_ON) {
				queue_delayed_work(ts->wq,
						&ts->finger_input_work,
						msecs_to_jiffies(0));
			}
		}

		if (ts->intr_status & TOUCH_IRQ_KNOCK)
			touch_send_uevent(ts, TOUCH_UEVENT_KNOCK);

		if (ts->intr_status & TOUCH_IRQ_PASSWD)
			touch_send_uevent(ts, TOUCH_UEVENT_PASSWD);

		if (ts->intr_status & TOUCH_IRQ_SWIPE_RIGHT)
			touch_send_uevent(ts, TOUCH_UEVENT_SWIPE_RIGHT);

		if (ts->intr_status & TOUCH_IRQ_SWIPE_LEFT)
			touch_send_uevent(ts, TOUCH_UEVENT_SWIPE_LEFT);
	}

	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}

static void touch_init_work_func(struct work_struct *init_work)
{
	struct touch_core_data *ts =
		container_of(to_delayed_work(init_work),
				struct touch_core_data, init_work);

	TOUCH_TRACE();

	TOUCH_I("touch_ic_init Start\n");
	mutex_lock(&ts->lock);
	touch_core_initialize(ts);
	ts->driver->init(ts->dev); // [e2sh] lg4894_init
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
if(0) {
	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		queue_delayed_work(ts->wq, &ts->upgrade_work, 0);
		return;
	}
}
	if (ts->asc.use_asc == ASC_ON) {
		if (atomic_read(&ts->state.core) == CORE_UPGRADE) {
			mutex_lock(&ts->lock);
			ts->driver->asc(ts->dev, ASC_GET_FW_SENSITIVITY, 0);
			mutex_unlock(&ts->lock);
		}

		queue_delayed_work(ts->wq, &ts->toggle_delta_work,
				msecs_to_jiffies(0));
	}

	atomic_set(&ts->state.core, CORE_NORMAL);
	TOUCH_I("touch_ic_init End\n");
}


static void touch_upgrade_work_func(struct work_struct *upgrade_work)
{
	struct touch_core_data *ts =
		container_of(to_delayed_work(upgrade_work),
				struct touch_core_data, upgrade_work);
	int ret;

	TOUCH_TRACE();

	atomic_set(&ts->state.core, CORE_UPGRADE);
	mutex_lock(&ts->lock);
	ret = ts->driver->upgrade(ts->dev);
	mutex_unlock(&ts->lock);

	/* init force_upgrade */
	ts->force_fwup = 0;
	ts->test_fwpath[0] = '\0';

	if (ret < 0) {
		TOUCH_E("failed to upgrade (ret: %d)\n", ret);
		return;
	}

	ts->driver->power(ts->dev, POWER_OFF);
	ts->driver->power(ts->dev, POWER_ON);
	queue_delayed_work(ts->wq, &ts->init_work,
			msecs_to_jiffies(ts->caps.hw_reset_delay));
}

static void touch_fb_work_func(struct work_struct *fb_work)
{
	struct touch_core_data *ts =
			container_of(to_delayed_work(fb_work),
				struct touch_core_data, fb_work);

	if(atomic_read(&ts->state.fb) == FB_SUSPEND)
		touch_suspend(ts->dev);
	else if(atomic_read(&ts->state.fb) == FB_RESUME)
		touch_resume(ts->dev);
}

static void touch_toggle_delta_check_work_func(
		struct work_struct *toggle_delta_work)
{
	struct touch_core_data *ts =
			container_of(to_delayed_work(toggle_delta_work),
				struct touch_core_data, toggle_delta_work);
	struct asc_info *asc = &(ts->asc);
	int connect_status = atomic_read(&ts->state.connect);
	int wireless_status = atomic_read(&ts->state.wireless);
	int call_status = atomic_read(&ts->state.incoming_call);
	int onhand_status = atomic_read(&ts->state.onhand);

	if (asc->use_asc == ASC_OFF) {
		TOUCH_I("%s : ASC is OFF\n", __func__);
		return;
	}

	TOUCH_I("%s : connect = %d, wireless = %d, call = %d, onhand = %d\n",
			__func__, connect_status, wireless_status,
			call_status, onhand_status);

	if ((connect_status != CONNECT_INVALID) ||
			(wireless_status != 0) ||
			(call_status != INCOMING_CALL_IDLE)) {
		asc->use_delta_chk = DELTA_CHK_OFF;
		touch_change_sensitivity(ts, NORMAL_SENSITIVITY);
	} else {
		if (onhand_status == IN_HAND_ATTN) {
			asc->use_delta_chk = DELTA_CHK_ON;
			touch_change_sensitivity(ts, NORMAL_SENSITIVITY);
		} else if (onhand_status == IN_HAND_NO_ATTN) {
			asc->use_delta_chk = DELTA_CHK_ON;
			touch_change_sensitivity(ts, NORMAL_SENSITIVITY);
		} else if (onhand_status == NOT_IN_HAND) {
			asc->use_delta_chk = DELTA_CHK_ON;
			touch_change_sensitivity(ts, ACUTE_SENSITIVITY);
		} else {
			TOUCH_I("%s : unknown onhand_status\n", __func__);
		}
	}

	TOUCH_I("%s : curr_sensitivity = %s, use_delta_chk = %d\n", __func__,
			asc_str[asc->curr_sensitivity], asc->use_delta_chk);

	return;
}

static void touch_finger_input_check_work_func(
		struct work_struct *finger_input_work)
{
	struct touch_core_data *ts =
		container_of(to_delayed_work(finger_input_work),
				struct touch_core_data, finger_input_work);

	if ((ts->tcount == 1) && (!ts->asc.delta_updated))
		touch_get_max_delta(ts);

	if ((ts->tcount == 0) && (ts->asc.delta_updated))
		touch_update_sensitivity(ts);

	return;
}

static int touch_check_driver_function(struct touch_core_data *ts)
{
	if (ts->driver->probe &&
			ts->driver->remove &&
			ts->driver->resume &&
			ts->driver->suspend &&
			ts->driver->init &&
			ts->driver->irq_handler &&
			ts->driver->upgrade &&
			ts->driver->power)
		return 0;

	return -EPERM;
}

static int touch_init_platform_data(struct touch_core_data *ts)
{
	int ret;

	ret = touch_check_driver_function(ts);

	if (ret) {
		TOUCH_E("failed to check functions\n");
		return ret;
	}

	if (ts->dev->of_node)
		ret = touch_get_dts(ts);
	else
		ret = touch_get_platform_data(ts);

	if (ret)
		TOUCH_E("failed to get platform data\n");

	return ret;
}

static void touch_init_locks(struct touch_core_data *ts)
{
	mutex_init(&ts->lock);
	wake_lock_init(&ts->lpwg_wake_lock, WAKE_LOCK_SUSPEND, "touch_lpwg");
}

static int touch_init_input(struct touch_core_data *ts)
{
	struct input_dev *input;
	int ret;

	input = input_allocate_device();

	if (!input) {
		TOUCH_E("failed to allocate memory for input\n");
		return -ENOMEM;
	}

	input->name = "touch_dev";

	TOUCH_I("%s %d-%d-%d-%d-%d-%d-%d\n", __func__,
			ts->caps.max_x,
			ts->caps.max_y,
			ts->caps.max_pressure,
			ts->caps.max_width,
			ts->caps.max_width,
			ts->caps.max_orientation,
			ts->caps.max_id);
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
			ts->caps.max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
			ts->caps.max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			ts->caps.max_pressure, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0,
			ts->caps.max_width, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MINOR, 0,
			ts->caps.max_width, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION, 0,
			ts->caps.max_orientation, 0, 0);

	ret = input_mt_init_slots(input, ts->caps.max_id, 0);

	if (ret < 0) {
		TOUCH_E("failed to initialize input device(ret:%d)\n", ret);
		goto error;
	}

	ret = input_register_device(input);

	if (ret < 0) {
		TOUCH_E("failed to register input device(ret:%d)\n", ret);
		goto error_register;
	}

	input_set_drvdata(input, ts);
	ts->input = input;

	return 0;

error_register:
	input_mt_destroy_slots(input);

error:
	input_free_device(input);

	return ret;
}

static void touch_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	TOUCH_I("%s Start\n", __func__);
	cancel_delayed_work_sync(&ts->init_work);
	cancel_delayed_work_sync(&ts->upgrade_work);
	atomic_set(&ts->state.uevent, UEVENT_IDLE);
	mutex_lock(&ts->lock);
	touch_report_all_event(ts);
	atomic_set(&ts->state.fb, FB_SUSPEND);
	/* if need skip, return value is not 0 in pre_suspend */
	ret = ts->driver->suspend(dev); // [e2sh] lg4894_suspend
	mutex_unlock(&ts->lock);
	TOUCH_I("%s End\n", __func__);

	if (ret == 1)
		mod_delayed_work(ts->wq, &ts->init_work, 0);
}

static void touch_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	TOUCH_I("%s Start\n", __func__);
	mutex_lock(&ts->lock);
	atomic_set(&ts->state.fb, FB_RESUME);
	/* if need skip, return value is not 0 in pre_resume */
	ret = ts->driver->resume(dev); // [e2sh] lg4894_resume
	mutex_unlock(&ts->lock);
	TOUCH_I("%s End\n", __func__);

	if (ret == 0)
		mod_delayed_work(ts->wq, &ts->init_work, 0);
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void touch_early_suspend(struct early_suspend *h)
{
	struct touch_core_data *ts =
		container_of(h, struct touch_core_data, early_suspend);
	TOUCH_TRACE();
	touch_suspend(ts->dev);
}

static void touch_early_resume(struct early_suspend *h)
{
	struct touch_core_data *ts =
		container_of(h, struct touch_core_data, early_suspend);
	TOUCH_TRACE();
	touch_resume(ts->dev);
}

static int touch_init_pm(struct touch_core_data *ts)
{
	TOUCH_TRACE();
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = touch_early_suspend;
	ts->early_suspend.resume = touch_early_resume;
	register_early_suspend(&ts->early_suspend);
	return 0;
}
#elif defined(CONFIG_FB)
static int touch_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(self, struct touch_core_data, fb_notif);
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			touch_resume(ts->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			touch_suspend(ts->dev);
	}

	return 0;
}

static int touch_init_pm(struct touch_core_data *ts)
{
	TOUCH_TRACE();

	ts->fb_notif.notifier_call = touch_fb_notifier_callback;
	return fb_register_client(&ts->fb_notif);
}
#endif

static struct bus_type touch_uevent_subsys = {
	.name = LGE_TOUCH_NAME,
	.dev_name = LGE_TOUCH_NAME,
};

static struct device device_uevent_touch = {
	.id    = 0,
	.bus   = &touch_uevent_subsys,
};

static int touch_init_uevent(struct touch_core_data *ts)
{
	int ret = 0;

	ret = subsys_system_register(&touch_uevent_subsys, NULL);
	if (ret < 0)
		TOUCH_E("%s, bus is not registered, ret : %d\n",
				__func__, ret);
	ret = device_register(&device_uevent_touch);
	if (ret < 0)
		TOUCH_E("%s, device is not registered, ret : %d\n",
				__func__, ret);

	return ret;
}

char *uevent_str[TOUCH_UEVENT_SIZE][2] = {
	{"TOUCH_GESTURE_WAKEUP=WAKEUP", NULL},
	{"TOUCH_GESTURE_WAKEUP=PASSWORD", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWIPE_RIGHT", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWIPE_LEFT", NULL}
};

static void touch_send_uevent(struct touch_core_data *ts, int type)
{
	TOUCH_TRACE();
	if (atomic_read(&ts->state.uevent) == UEVENT_IDLE) {
		wake_lock_timeout(&ts->lpwg_wake_lock, msecs_to_jiffies(3000));
		atomic_set(&ts->state.uevent, UEVENT_BUSY);
		kobject_uevent_env(&device_uevent_touch.kobj,
				KOBJ_CHANGE, uevent_str[type]);
		TOUCH_I("%s\n",  uevent_str[type][0]);
		touch_report_all_event(ts);
	}
}

void touch_notify_connect(u32 type)
{
	touch_atomic_notifier_call(NOTIFY_CONNECTION, &type);
}
EXPORT_SYMBOL(touch_notify_connect);

void touch_notify_wireless(u32 type)
{
	touch_atomic_notifier_call(NOTIFY_WIRELEES, &type);
}

EXPORT_SYMBOL(touch_notify_wireless);

void touch_notify_earjack(u32 type)
{
	touch_atomic_notifier_call(NOTIFY_EARJACK, &type);
}

EXPORT_SYMBOL(touch_notify_earjack);

static int touch_notify(struct touch_core_data *ts,
				   unsigned long event, void *data)
{
	int ret = 0;

	if (touch_boot_mode() == TOUCH_CHARGER_MODE)
		return 0;

	if (ts->driver->notify) {
		mutex_lock(&ts->lock);
		switch (event) {
		case NOTIFY_TOUCH_RESET:
			ret = ts->driver->notify(ts->dev, event, data);
			if (ret != NOTIFY_STOP) {
				touch_interrupt_control(ts->dev,
				INTERRUPT_DISABLE);
			}
			break;

		case NOTIFY_CONNECTION:
			if (atomic_read(&ts->state.connect) != *(int *)data) {
				atomic_set(&ts->state.connect, *(int *)data);
				ret = ts->driver->notify(ts->dev, event, data);

				if (ts->asc.use_asc == ASC_ON) {
					queue_delayed_work(ts->wq,
							&ts->toggle_delta_work,
							msecs_to_jiffies(0));
				}
			}
			break;

		case NOTIFY_WIRELEES:
			atomic_set(&ts->state.wireless, *(int *)data);
			ret = ts->driver->notify(ts->dev, event, data);

			if (ts->asc.use_asc == ASC_ON) {
				queue_delayed_work(ts->wq,
						&ts->toggle_delta_work,
						msecs_to_jiffies(0));
			}

			break;

		case NOTIFY_FB:
			atomic_set(&ts->state.fb, *(int *)data);
			queue_delayed_work(ts->wq, &ts->fb_work, 0);
			break;

		case NOTIFY_EARJACK:
			atomic_set(&ts->state.earjack, *(int *)data);
			ret = ts->driver->notify(ts->dev, event, data);
			break;

		default:
			ret = ts->driver->notify(ts->dev, event, data);
			break;
		}
		mutex_unlock(&ts->lock);
	}

	return ret;
}

static int touch_blocking_notifier_callback(struct notifier_block *this,
				   unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(this, struct touch_core_data, blocking_notif);
	return touch_notify(ts, event, data);
}

static void touch_atomic_notifer_work_func(struct work_struct *notify_work)
{
	struct touch_core_data *ts =
		container_of(to_delayed_work(notify_work),
				struct touch_core_data, notify_work);

	touch_notify(ts, ts->notify_event, &ts->notify_data);
}

static int touch_atomic_notifier_callback(struct notifier_block *this,
				   unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(this, struct touch_core_data, atomic_notif);

	ts->notify_event = event;
	ts->notify_data = *(int *)data;
	queue_delayed_work(ts->wq, &ts->notify_work, 0);

	return 0;
}

static int touch_init_notify(struct touch_core_data *ts)
{
	int ret;

	ts->blocking_notif.notifier_call = touch_blocking_notifier_callback;
	ret = touch_blocking_notifier_register(&ts->blocking_notif);

	if (ret < 0) {
		TOUCH_E("failed to regiseter touch blocking_notify callback\n");
	}

	ts->atomic_notif.notifier_call = touch_atomic_notifier_callback;
	ret = touch_atomic_notifier_register(&ts->atomic_notif);

	if (ret < 0) {
		TOUCH_E("failed to regiseter touch atomic_notify callback\n");
	}

	return ret;
}

static int touch_init_works(struct touch_core_data *ts)
{
	ts->wq = create_singlethread_workqueue("touch_wq");

	if (!ts->wq) {
		TOUCH_E("failed to create workqueue\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&ts->init_work, touch_init_work_func);
	INIT_DELAYED_WORK(&ts->upgrade_work, touch_upgrade_work_func);
	INIT_DELAYED_WORK(&ts->notify_work, touch_atomic_notifer_work_func);
	INIT_DELAYED_WORK(&ts->fb_work, touch_fb_work_func);
	INIT_DELAYED_WORK(&ts->toggle_delta_work,
			touch_toggle_delta_check_work_func);
	INIT_DELAYED_WORK(&ts->finger_input_work,
			touch_finger_input_check_work_func);

	return 0;
}


static int touch_core_probe_normal(struct platform_device *pdev)
{
	struct touch_core_data *ts;
	int ret;

	TOUCH_TRACE();

	ts = (struct touch_core_data *) pdev->dev.platform_data;

	atomic_set(&ts->state.core, CORE_PROBE);
	ret = touch_init_platform_data(ts);
	if (ret) {
		TOUCH_E("failed to initialize platform_data\n");
		return -EINVAL;
	}

	ts->driver->probe(ts->dev); // [e2sh] lg4894_probe

	/* set defalut lpwg value because of AAT */
	ts->lpwg.screen = 1;
	ts->lpwg.sensor = PROX_FAR;

	ts->driver->power(ts->dev, POWER_OFF); // [e2sh] lg4894_power
	ts->driver->power(ts->dev, POWER_ON);

	touch_init_locks(ts);
	ret = touch_init_works(ts);
	if (ret) {
		TOUCH_E("failed to initialize works\n");
		goto error_init_work;
	}

	ret = touch_init_input(ts);
	if (ret) {
		TOUCH_E("failed to register input device(ret:%d)\n", ret);
		goto error_init_input;
	}

	ret = touch_request_irq(ts->irq, touch_irq_handler,
			touch_irq_thread, ts->irqflags | IRQF_ONESHOT,
			LGE_TOUCH_NAME, ts);
	if (ret) {
		TOUCH_E("failed to request_thread_irq(irq:%d, ret:%d)\n",
				ts->irq, ret);
		goto error_request_irq;
	}

	touch_disable_irq(ts->irq);
	touch_init_pm(ts);

	touch_init_notify(ts);
	touch_blocking_notifier_call(LCD_EVENT_TOUCH_DRIVER_REGISTERED, NULL);

	ret = touch_init_uevent(ts);
	ret = touch_init_sysfs(ts);

	TOUCH_I("hw_reset_delay : %d ms\n", ts->caps.hw_reset_delay);
	queue_delayed_work(ts->wq, &ts->init_work,
			msecs_to_jiffies(ts->caps.hw_reset_delay));

	return 0;

error_request_irq:
error_init_input:
error_init_work:
	return ret;
}

static int touch_core_probe_charger(struct platform_device *pdev)
{
	struct touch_core_data *ts;
	int ret = 0;

	TOUCH_TRACE();

	ts = (struct touch_core_data *) pdev->dev.platform_data;

	ret = touch_init_platform_data(ts);
	if (ret) {
		TOUCH_E("failed to initialize platform_data\n");
		return -EINVAL;
	}

	ts->driver->probe(ts->dev);

	touch_init_locks(ts);
	ret = touch_init_works(ts);
	if (ret) {
		TOUCH_E("failed to initialize works\n");
		return ret;
	}

	touch_init_pm(ts);
	touch_init_notify(ts);
	touch_blocking_notifier_call(LCD_EVENT_TOUCH_DRIVER_REGISTERED, NULL);

	return ret;
}

static int touch_core_probe(struct platform_device *pdev)
{
	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("CHARGER MODE\n");
		return touch_core_probe_charger(pdev);
	}

	return touch_core_probe_normal(pdev);
}

static int touch_core_remove(struct platform_device *pdev)
{
	TOUCH_TRACE();
	return 0;
}

static struct platform_driver touch_core_driver = {
	.driver = {
		.name = LGE_TOUCH_DRIVER_NAME,
		.owner = THIS_MODULE,
/*
		.of_match_table = touch_match_ids,
*/
	},
	.probe = touch_core_probe,
	.remove = touch_core_remove,
};

static void touch_core_async_init(void *data, async_cookie_t cookie)
{
	int ret = platform_driver_register(&touch_core_driver);

	TOUCH_TRACE();

	if (ret)
		TOUCH_E("async_init failed");
}

static int __init touch_core_init(void)
{
	TOUCH_TRACE();
	async_schedule(touch_core_async_init, NULL);
	return 0;
}

static void __exit touch_core_exit(void)
{
	TOUCH_TRACE();
	platform_driver_unregister(&touch_core_driver);
}

#if defined(CONFIG_LGE_MODULE_DETECT)
extern int get_display_id(void);
#endif /* CONFIG_LGE_MODULE_DETECT */
enum touch_device_type touch_get_device_type(void)
{
	enum touch_device_type ret = TYPE_LG4945;
#if defined(CONFIG_LGE_MODULE_DETECT)
	ret = get_display_id();
	if (ret != TYPE_LG4894) {
		TOUCH_I("This panel is not LG4894, ret = %d\n", ret);
		return ret;
	}
#else /* CONFIG_LGE_MODULE_DETECT */
//	ret = lge_get_panel_maker_id();
#endif

	TOUCH_I("%s = [%d]\n", __func__, ret);

	return ret;
}

module_init(touch_core_init);
module_exit(touch_core_exit);

MODULE_AUTHOR("hoyeon.jang@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
