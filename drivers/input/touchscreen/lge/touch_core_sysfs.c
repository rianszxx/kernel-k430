/* touch_core_sysfs.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "touch_core.h"
#include "touch_hwif.h"

#define TOUCH_SHOW(ret, buf, fmt, args...) \
	(ret += snprintf(buf + ret, PAGE_SIZE - ret, fmt, ##args))

static char ime_str[3][8] = {"OFF", "ON", "SWYPE"};
static char incoming_call_str[3][8] = {"IDLE", "RINGING", "OFFHOOK"};
static char mfts_str[4][8] = {"NONE", "FOLDER", "FLAT", "CURVED"};
static char onhand_str[3][20] = {"IN_HAND_ATTN", "NOT_IN_HAND", "IN_HAND_NO_ATTN"};
int mfts_lpwg;

static ssize_t show_platform_data(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int i;
	int ret = 0;

	TOUCH_SHOW(ret, buf, "=== Platform Data ===\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "reset_pin", ts->reset_pin);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "int_pin", ts->int_pin);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "maker_id_pin", ts->maker_id_pin);

	TOUCH_SHOW(ret, buf, "caps:\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_x", ts->caps.max_x);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_y", ts->caps.max_y);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_pressure",
		   ts->caps.max_pressure);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_width", ts->caps.max_width);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_orientation",
		   ts->caps.max_orientation);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_id", ts->caps.max_id);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "hw_reset_delay",
		   ts->caps.hw_reset_delay);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "sw_reset_delay",
		   ts->caps.sw_reset_delay);

	TOUCH_SHOW(ret, buf, "role:\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "use_lpwg", ts->role.use_lpwg);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "use_firmware",
		   ts->role.use_firmware);

	TOUCH_SHOW(ret, buf, "power:\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "vdd-gpio", ts->vdd_pin);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "vio-gpio", ts->vio_pin);

	TOUCH_SHOW(ret, buf, "firmware:\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "def_fwcnt", ts->def_fwcnt);
	for (i = 0; i < ts->def_fwcnt; i++)
		TOUCH_SHOW(ret, buf, "\t%25s [%d:%s]\n", "def_fwpath",
			   i, ts->def_fwpath[i]);

	return ret;
}

static ssize_t store_upgrade(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (sscanf(buf, "%255s", &ts->test_fwpath[0]) <= 0)
		return count;

	ts->force_fwup = 1;
	queue_delayed_work(ts->wq, &ts->upgrade_work, 0);

	return count;
}

static ssize_t show_upgrade(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->test_fwpath[0] = '\0';
	ts->force_fwup = 1;

	queue_delayed_work(ts->wq, &ts->upgrade_work, 0);

	return 0;
}

static ssize_t show_lpwg_data(struct device *dev, char *buf)
{
	int i = 0, ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	if (!ts->driver->lpwg)
		return ret;

	mutex_lock(&ts->lock);
	for (i = 0; i < MAX_LPWG_CODE; i++) {
		if (ts->lpwg.code[i].x == -1 && ts->lpwg.code[i].y == -1)
			break;
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	memset(ts->lpwg.code, 0, sizeof(struct point) * MAX_LPWG_CODE);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t store_lpwg_data(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int reply = 0;

	if (sscanf(buf, "%d", &reply) <= 0)
		return count;

	TOUCH_I("%s : reply = %d\n", __func__, reply);

	atomic_set(&ts->state.uevent, UEVENT_IDLE);
	wake_unlock(&ts->lpwg_wake_lock);

	return count;
}

static ssize_t store_lpwg_notify(struct device *dev,
		const char *buf, size_t count)
{

	struct touch_core_data *ts = to_touch_core(dev);
	int code = 0;
	int param[4] = {0, };
	int mfts_mode = 0;

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg)
		return count;

	if (sscanf(buf, "%d %d %d %d %d",
			&code, &param[0], &param[1], &param[2], &param[3]) <= 0)
		return count;

	/* only below code notify
		3 active_area
		4 knockcode tap count
		8 knockcode double tap check
		9 update_all
	*/
	if (code == 1 || code == 2 || code == 5 ||
		code == 6 || code == 7)
		return count;

	if (ts->driver->lpwg) {
		mutex_lock(&ts->lock);
		ts->driver->lpwg(ts->dev, code, param);
		mutex_unlock(&ts->lock);
	}

	return count;
}

static ssize_t show_lockscreen_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.lockscreen);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
		value ? "LOCK" : "UNLOCK", value);

	return ret;
}

static ssize_t store_lockscreen_state(struct device *dev,
		const char *buf, size_t count)
{
	int value = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value == LOCKSCREEN_UNLOCK || value == LOCKSCREEN_LOCK) {
		atomic_set(&ts->state.lockscreen, value);
		TOUCH_I("%s : %s(%d)\n", __func__,
				value ? "LOCK" : "UNLOCK", value);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_ime_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.ime);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
			ime_str[value], value);

	return ret;
}

static ssize_t store_ime_state(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value >= IME_OFF && value <= IME_SWYPE) {
		if (atomic_read(&ts->state.ime) == value)
			return count;

		atomic_set(&ts->state.ime, value);
		ret = touch_blocking_notifier_call(NOTIFY_IME_STATE,
			&ts->state.ime);
		TOUCH_I("%s : %s(%d), ret = %d\n",
			__func__, ime_str[value], value, ret);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_quick_cover_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.quick_cover);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
		value ? "CLOSE" : "OPEN", value);

	return ret;
}

static ssize_t store_quick_cover_state(struct device *dev,
		const char *buf, size_t count)
{
	int value = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value == QUICKCOVER_CLOSE || value == QUICKCOVER_OPEN) {
		atomic_set(&ts->state.quick_cover, value);
		TOUCH_I("%s : %s(%d)\n", __func__,
			value ? "CLOSE" : "OPEN", value);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_incoming_call_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.incoming_call);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
		incoming_call_str[value], value);

	return ret;
}

static ssize_t store_incoming_call_state(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value >= INCOMING_CALL_IDLE && value <= INCOMING_CALL_OFFHOOK) {
		if (atomic_read(&ts->state.incoming_call) == value)
			return count;

		atomic_set(&ts->state.incoming_call, value);

		ret = touch_blocking_notifier_call(NOTIFY_CALL_STATE,
					&ts->state.incoming_call);

		if (ts->asc.use_asc == ASC_ON)
			queue_delayed_work(ts->wq, &ts->toggle_delta_work,
					msecs_to_jiffies(0));

		TOUCH_I("%s : %s(%d)\n", __func__,
				incoming_call_str[value], value);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_version_info(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	ret = ts->driver->get(dev, CMD_VERSION, NULL, buf);

	return ret;
}

static ssize_t show_atcmd_version_info(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	ret = ts->driver->get(dev, CMD_ATCMD_VERSION, NULL, buf);

	return ret;
}

static ssize_t show_mfts_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.mfts);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
			mfts_str[value], value);

	return ret;
}

static ssize_t store_mfts_state(struct device *dev,
		const char *buf, size_t count)
{
	int value = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value >= MFTS_NONE && value <= MFTS_CURVED) {
		atomic_set(&ts->state.mfts, value);
		TOUCH_I("%s : %s(%d)\n", __func__, mfts_str[value], value);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_mfts_lpwg(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "%d\n", ts->role.use_lpwg_test);

	return ret;
}

static ssize_t store_mfts_lpwg(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	ts->role.mfts_lpwg = value;
	mfts_lpwg = value;
	TOUCH_I("mfts_lpwg:%d\n", ts->role.mfts_lpwg);

	return count;
}


static ssize_t show_sp_link_touch_off(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "sp link touch status %d\n",
			atomic_read(&ts->state.sp_link));

	return ret;
}

static ssize_t store_sp_link_touch_off(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		TOUCH_I("Invalid Value\n");
		return count;
	}

	atomic_set(&ts->state.sp_link, value);

	if (atomic_read(&ts->state.sp_link) == SP_CONNECT) {
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		TOUCH_I("SP Mirroring Connected\n");
	} else if(atomic_read(&ts->state.sp_link) == SP_DISCONNECT) {
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		TOUCH_I("SP Mirroring Disconnected\n");
	}

	return count;
}

static ssize_t show_debug_tool_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.debug_tool);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", value);

	return ret;
}

static ssize_t store_debug_tool_state(struct device *dev,
		const char *buf, size_t count)
{
	int data = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	if (sscanf(buf, "%d", &data) <= 0)
		return count;

	if (data >= DEBUG_TOOL_DISABLE && data <= DEBUG_TOOL_ENABLE) {
		atomic_set(&ts->state.debug_tool, data);
		ts->notify_event = NOTIFY_DEBUG_TOOL;
		ts->notify_data = data;
		ts->driver->notify(ts->dev, ts->notify_event, (void *)&ts->notify_data);
		TOUCH_I("%s : %s\n", __func__,
		(data == DEBUG_TOOL_ENABLE) ?
		"Debug Tool Enabled" : "Debug Tool Disabled");
	} else {
		TOUCH_I("%s : Unknown debug tool set value %d\n", __func__, data);
	}

	return count;
}

static ssize_t show_debug_option_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.debug_option_mask);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", value);

	return ret;
}

static ssize_t store_debug_option_state(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int new_mask = 0;

	if (sscanf(buf, "%d", &new_mask) <= 0)
		return count;

	if (new_mask >= DEBUG_OPTION_DISABLE
		&& new_mask <= DEBUG_OPTION_1_2_3) {
		atomic_set(&ts->state.debug_option_mask, new_mask);
		TOUCH_I("%s : Input masking value = %d",
			__func__, new_mask);
	} else {
		TOUCH_I("%s : Unknown debug option set value %d\n", __func__, new_mask);
	}

	return count;
}

static ssize_t show_asc_param(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct asc_info *asc = &(ts->asc);
	int ret = 0;

	TOUCH_SHOW(ret, buf, "use_asc = %d\n", asc->use_asc);
	TOUCH_SHOW(ret, buf, "low_delta_thres = %d\n", asc->low_delta_thres);
	TOUCH_SHOW(ret, buf, "high_delta_thres = %d\n", asc->high_delta_thres);

	return ret;
}

static ssize_t store_asc_param(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct asc_info *asc = &(ts->asc);
	unsigned char string[30] = {0, };
	u32 value = 0;

	if (sscanf(buf, "%s %d", string, &value) <= 0)
		return count;

	if (!strcmp(string, "use_asc")) {
		if (value == ASC_OFF) {
			asc->use_delta_chk = DELTA_CHK_OFF;
			touch_change_sensitivity(ts, NORMAL_SENSITIVITY);
			asc->use_asc = ASC_OFF;
		} else if (value == ASC_ON) {
			asc->use_asc = ASC_ON;
			mutex_lock(&ts->lock);
			ts->driver->asc(ts->dev, ASC_GET_FW_SENSITIVITY, 0);
			mutex_unlock(&ts->lock);
			queue_delayed_work(ts->wq, &ts->toggle_delta_work,
					msecs_to_jiffies(0));
		} else {
			TOUCH_I("%s : unknown value(%d)\n", __func__, value);
		}

	} else if (!strcmp(string, "low_delta_thres")) {
		asc->low_delta_thres = value;
	} else if (!strcmp(string, "high_delta_thres")) {
		asc->high_delta_thres = value;
	}

	return count;
}

static ssize_t show_onhand(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.onhand);
	TOUCH_I("%s : %s(%d)\n", __func__, onhand_str[value], value);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", value);

	return ret;
}

static ssize_t store_onhand(struct device *dev,
		const char *buf, size_t count)
{
	int value = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value >= IN_HAND_ATTN && value <= IN_HAND_NO_ATTN) {
		atomic_set(&ts->state.onhand, value);

		if (ts->asc.use_asc == ASC_ON)
			queue_delayed_work(ts->wq, &ts->toggle_delta_work,
					msecs_to_jiffies(0));


		TOUCH_I("%s : %s(%d)\n", __func__, onhand_str[value], value);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

#if defined(CONFIG_LGE_MODULE_DETECT)
static ssize_t show_module_id(struct device *dev, char *buf)
{
	int ret = 0;
	TOUCH_I("Module id is LG4894\n");
	ret = snprintf(buf, PAGE_SIZE, "%d\n", TYPE_LG4894);

	return ret;
}
#endif /* CONFIG_LGE_MODULE_DETECT */

static TOUCH_ATTR(platform_data, show_platform_data, NULL);
static TOUCH_ATTR(fw_upgrade, show_upgrade, store_upgrade);
static TOUCH_ATTR(lpwg_data, show_lpwg_data, store_lpwg_data);
static TOUCH_ATTR(lpwg_notify, NULL, store_lpwg_notify);
static TOUCH_ATTR(keyguard,
	show_lockscreen_state, store_lockscreen_state);
static TOUCH_ATTR(ime_status, show_ime_state, store_ime_state);
static TOUCH_ATTR(quick_cover_status,
	show_quick_cover_state, store_quick_cover_state);
static TOUCH_ATTR(incoming_call,
	show_incoming_call_state, store_incoming_call_state);
static TOUCH_ATTR(firmware, show_version_info, NULL);
static TOUCH_ATTR(version, show_version_info, NULL);
static TOUCH_ATTR(testmode_ver, show_atcmd_version_info, NULL);
static TOUCH_ATTR(mfts, show_mfts_state, store_mfts_state);
static TOUCH_ATTR(mfts_lpwg, show_mfts_lpwg, store_mfts_lpwg);
static TOUCH_ATTR(sp_link_touch_off,
	show_sp_link_touch_off, store_sp_link_touch_off);
static TOUCH_ATTR(debug_tool, show_debug_tool_state, store_debug_tool_state);
static TOUCH_ATTR(debug_option, show_debug_option_state, store_debug_option_state);
static TOUCH_ATTR(asc, show_asc_param, store_asc_param);
static TOUCH_ATTR(onhand, show_onhand, store_onhand);
#if defined(CONFIG_LGE_MODULE_DETECT)
static TOUCH_ATTR(module_id, show_module_id, NULL);
#endif /* CONFIG_LGE_MODULE_DETECT */

static struct attribute *touch_attribute_list[] = {
	&touch_attr_platform_data.attr,
	&touch_attr_fw_upgrade.attr,
	&touch_attr_lpwg_data.attr,
	&touch_attr_lpwg_notify.attr,
	&touch_attr_keyguard.attr,
	&touch_attr_ime_status.attr,
	&touch_attr_quick_cover_status.attr,
	&touch_attr_incoming_call.attr,
	&touch_attr_firmware.attr,
	&touch_attr_version.attr,
	&touch_attr_testmode_ver.attr,
	&touch_attr_mfts.attr,
	&touch_attr_mfts_lpwg.attr,
	&touch_attr_sp_link_touch_off.attr,
	&touch_attr_debug_tool.attr,
	&touch_attr_debug_option.attr,
	&touch_attr_asc.attr,
	&touch_attr_onhand.attr,
#if defined(CONFIG_LGE_MODULE_DETECT)
	&touch_attr_module_id.attr,
#endif /* CONFIG_LGE_MODULE_DETECT */

	NULL,
};

static const struct attribute_group touch_attribute_group = {
	.attrs = touch_attribute_list,
};

static ssize_t touch_attr_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct touch_core_data *ts =
		container_of(kobj, struct touch_core_data, kobj);
	struct touch_attribute *priv =
		container_of(attr, struct touch_attribute, attr);
	ssize_t ret = 0;

	if (priv->show)
		ret = priv->show(ts->dev, buf);

	return ret;
}

static ssize_t touch_attr_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct touch_core_data *ts =
		container_of(kobj, struct touch_core_data, kobj);
	struct touch_attribute *priv =
		container_of(attr, struct touch_attribute, attr);
	ssize_t ret = 0;

	if (priv->store)
		ret = priv->store(ts->dev, buf, count);

	return ret;
}

static const struct sysfs_ops touch_sysfs_ops = {
	.show	= touch_attr_show,
	.store	= touch_attr_store,
};

static struct kobj_type touch_kobj_type = {
	.sysfs_ops = &touch_sysfs_ops,
};

int touch_init_sysfs(struct touch_core_data *ts)
{
	struct device *dev = &ts->input->dev;
	int ret;

	ret = kobject_init_and_add(&ts->kobj, &touch_kobj_type,
			dev->kobj.parent, "%s", LGE_TOUCH_NAME);

	ret = sysfs_create_group(&ts->kobj, &touch_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	if (ts->driver->register_sysfs)
		ret = ts->driver->register_sysfs(dev);

	return ret;
}

