#$(call all-subdir-src-or-makefile)
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
#subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/hibcore
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(srctree)/drivers/mmc/host/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(MTK_CUSTOM_PATH)/cam_cal/inc

obj-$(CONFIG_MTK_BASE_POWER) += base/
obj-$(CONFIG_MTK_POWER_GS) += power_gs/
obj-$(CONFIG_MEDIATEK_SOLUTION)	+= sched/
obj-$(CONFIG_MEDIATEK_SOLUTION) += mach/
obj-$(CONFIG_MTK_GPT_COMMON) += gpt/
obj-$(CONFIG_MTK_SERIAL)	+= uart/
obj-$(CONFIG_MTK_IRQ) += irq/
obj-$(CONFIG_MTK_SYS_CIRQ) += cirq/
obj-$(CONFIG_MTK_GIC_EXT) += ext_gic/
obj-$(CONFIG_MTK_ICE_DEUBG) +=ice_debug/
obj-$(CONFIG_MTK_NFC) += nfc/

obj-$(CONFIG_MMPROFILE) += mmp/
obj-$(CONFIG_MTK_AUXADC)	+= auxadc/
obj-$(CONFIG_MTK_M4U)	+= m4u/
obj-$(CONFIG_MTK_GPT_SCHEME_SUPPORT)	+= partition/
obj-$(CONFIG_MTK_SYSENV)	+= sysenv/
obj-$(CONFIG_MTK_PWM)	    += pwm/
obj-$(CONFIG_MTK_HALLIC)	+= hallic/
ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
obj-$(CONFIG_CUSTOM_KERNEL_SENSORHUB)	+= sensorHub/
obj-$(CONFIG_MTK_HWMON)	    += hwmon/
obj-$(CONFIG_CUSTOM_KERNEL_ALS) += alsps/
obj-$(CONFIG_CUSTOM_KERNEL_PS) += alsps/
obj-$(CONFIG_CUSTOM_KERNEL_ALSPS) += alsps/
obj-$(CONFIG_CUSTOM_KERNEL_ACCELEROMETER) += accelerometer/
obj-$(CONFIG_CUSTOM_KERNEL_GYROSCOPE) += gyroscope/
obj-$(CONFIG_CUSTOM_KERNEL_MAGNETOMETER) += magnetometer/
obj-$(CONFIG_CUSTOM_KERNEL_BAROMETER) += barometer/
obj-$(CONFIG_CUSTOM_KERNEL_HUMIDITY) += humidity/
obj-$(CONFIG_CUSTOM_KERNEL_STEP_COUNTER)	+= step_counter/
obj-$(CONFIG_CUSTOM_KERNEL_SIGNIFICANT_MOTION_SENSOR)	+= step_counter/
obj-$(CONFIG_CUSTOM_KERNEL_ACTIVITY_SENSOR)	+= activity_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_PICK_UP_SENSOR)	+= pick_up_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_SHAKE_SENSOR)	+= shake_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_FACE_DOWN_SENSOR)	+= face_down_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_IN_POCKET_SENSOR)	+= in_pocket_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_PEDOMETER)	+= pedometer/
obj-$(CONFIG_CUSTOM_KERNEL_HEART)	+= heart_rate_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_TILT_DETECTOR_SENSOR)	+= tilt_detector_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_WAKE_GESTURE_SENSOR)	+= wake_gesture_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_GLANCE_GESTURE_SENSOR)	+= glance_gesture_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_GRAVITY_SENSOR)	+= gravity/
obj-$(CONFIG_CUSTOM_KERNEL_RV_SENSOR)	+= rotationvector/
obj-$(CONFIG_CUSTOM_KERNEL_LINEARACCEL_SENSOR)	+= linearacceleration/
obj-$(CONFIG_CUSTOM_KERNEL_GRV_SENSOR)	+= gamerotationvector/
obj-$(CONFIG_CUSTOM_KERNEL_GMRV_SENSOR)	+= geomagneticrotationvector/
obj-$(CONFIG_CUSTOM_KERNEL_PDR_SENSOR)	+= pdr_sensor/
obj-$(CONFIG_CUSTOM_KERNEL_FREEFALL_SENSOR)	+= freefall_sensor/
endif
obj-$(CONFIG_MTPROF)	+= mtprof/
ifeq ($(CONFIG_MTK_I2C_EXTENSION),y)
obj-$(CONFIG_MTK_I2C)  += i2c/
endif
obj-$(CONFIG_MTK_LEDS)	+= leds/
obj-$(CONFIG_MTK_VIBRATOR)	+= vibrator/
obj-$(CONFIG_MTK_ACCDET)	+= accdet/
obj-$(CONFIG_MTK_ECCCI_C2K) += c2k_usb/
obj-$(CONFIG_MTK_IRTX_SUPPORT) += irtx/
obj-$(CONFIG_MTK_VIDEOCODEC_DRIVER) += videocodec/
obj-$(CONFIG_MTK_JPEG)	+= jpeg/
obj-$(CONFIG_MTK_RTC)	+= rtc/
obj-$(CONFIG_MTK_PMIC)	+= power/
obj-$(CONFIG_MTK_PMIC_WRAP) += pmic_wrap/
obj-$(CONFIG_USB_MTK_HDRC) += usb20/
obj-$(CONFIG_MTK_USBFSH) += usb11/
obj-$(CONFIG_MU3_PHY)	+= mu3phy/
obj-$(CONFIG_USB_MU3D_DRV)	+= mu3d/
obj-$(CONFIG_SSUSB_DRV) += ssusb/
obj-$(CONFIG_USB_XHCI_MTK)	+= xhci/
obj-$(CONFIG_USB_C_SWITCH)	+= usb_c/
obj-$(CONFIG_MTK_GPIO) += gpio/
obj-$(CONFIG_SECURITY_SELINUX)	+= selinux_warning/
obj-$(CONFIG_MTK_MEM) += mem/
obj-$(CONFIG_MTK_MEMORY_LOWPOWER) += memory-lowpower/
obj-$(CONFIG_MTK_CMDQ) += cmdq/
obj-$(CONFIG_MTK_BOOT) += boot/
obj-$(CONFIG_MTK_BOOT_REASON) += boot_reason/
obj-$(CONFIG_MTK_CHIP) += chip/
obj-$(CONFIG_MTK_DEVAPC_DRIVER) += devapc/
obj-$(CONFIG_MTK_DEVINFO_DRIVER) += devinfo/
obj-$(CONFIG_MTK_LCM) += lcm/
obj-y += ext_disp/
obj-$(CONFIG_MTK_HDMI_SUPPORT) += hdmi/
obj-$(CONFIG_MTK_FB) += video/
obj-$(CONFIG_MTK_GPU_SUPPORT) += gpu/
ifneq ($(CONFIG_MTK_PLATFORM), "mt2701")
obj-$(CONFIG_MTK_SMI_EXT)	+= smi/
endif
obj-$(CONFIG_MTK_CQDMA) += cqdma/
obj-$(CONFIG_MTK_DRAMC) += dramc/
obj-$(CONFIG_MTK_SYNC) += sync/
obj-$(CONFIG_MTK_FREQ_HOPPING) += freqhopping/
obj-$(CONFIG_MTK_SECURITY_SW_SUPPORT) +=masp/
obj-$(CONFIG_MEDIATEK_SOLUTION)  += sched/
obj-y += cameraisp/src/
obj-$(CONFIG_MTK_IMGSENSOR) += imgsensor/src/
obj-$(CONFIG_MTK_LENS)   += lens/
obj-$(CONFIG_MTK_FLASHLIGHT)	+= flashlight/src/
obj-$(CONFIG_MTK_CAM_CAL)	+= cam_cal/src/
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/met/),)
obj-$(CONFIG_MTK_MET) += met/
endif
obj-$(CONFIG_MTK_CCCI_DEVICES)	+=  ccci_util/
obj-$(CONFIG_MTK_NET_CCMNI) +=  ccmni/
obj-$(CONFIG_MTK_CCCI_EXT)     +=  dual_ccci/
obj-$(CONFIG_MTK_C2K_SUPPORT) += c2k_ccci/
obj-$(CONFIG_MTK_ECCCI_DRIVER) +=  eccci/

obj-$(CONFIG_MTK_HIBERNATION) += hibcore/

ifdef CONFIG_CUSTOM_KERNEL_SSW
ifneq ($(strip $(CONFIG_CUSTOM_KERNEL_SSW)),"")
obj-y	+= ssw/
endif
endif

obj-$(CONFIG_MTK_COMBO) += connectivity/
obj-$(CONFIG_MTK_BTIF) += btif/
obj-$(CONFIG_MTK_CONN_MD) += conn_md/

obj-$(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT) += gcpu/
obj-$(CONFIG_TRUSTONIC_TEE_SUPPORT) += gcpu/
obj-$(CONFIG_TRUSTONIC_TEE_SUPPORT) += gud/
obj-$(CONFIG_TRUSTONIC_TEE_SUPPORT) += secmem/
obj-$(CONFIG_MTK_DEBUG_LATCH)	+= debug_latch/
obj-$(CONFIG_MTK_WATCHPOINT) += watchpoint/
obj-$(CONFIG_MTK_SYSTRACKER)   += systracker/
obj-$(CONFIG_MTK_RAM_CONSOLE)	+= ram_console/
obj-$(CONFIG_MTK_EXTMEM) += exm/

obj-y += performance/

# gator driver is platform-independent
ifneq ($(wildcard $(srctree)/$(src)/gator-driver/),)
obj-$(CONFIG_GATOR_DRIVER) += gator-driver/
endif # gator-driver folder exist

obj-$(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT) += trustzone/
ifeq ($(CONFIG_HAVE_MTK_IN_HOUSE_TEE_SUPPORT),y)
obj-$(CONFIG_TRUSTY) += trustzone/
endif

ifeq (,$(filter $(CONFIG_ARCH_MT8127), y))
obj-$(CONFIG_THERMAL)   += thermal/
endif

obj-$(CONFIG_CUSTOM_KERNEL_CHARGEPUMP) += chargepump/

obj-$(CONFIG_MTK_MULTIBRIDGE_SUPPORT) += multibridge/
obj-$(CONFIG_MEDIATEK_SOLUTION)	+= aee/
obj-$(CONFIG_MTK_FMRADIO) += fmradio/
obj-$(CONFIG_MTK_ATF_LOGGER)	+= atf_log/
obj-$(CONFIG_MTK_MLOG) += mlog/
obj-$(CONFIG_MTK_EMI_BWL)	+= emi_bwl/
obj-$(CONFIG_MTK_EMI_MPU)	+= emi_mpu/
obj-$(CONFIG_MTK_MJC_DRIVER)	+= mjc/
obj-$(CONFIG_MD32_SUPPORT)	+= md32/
obj-$(CONFIG_HAVE_XLOG_SUPPORT) += xlog/
obj-$(CONFIG_MTK_EMI_MBW) += emi_mbw/
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/tc1_interface/),)
obj-$(CONFIG_MTK_TC1_FEATURE)   += tc1_interface/
endif
obj-$(CONFIG_MTK_BTCVSD) += btcvsd/
obj-$(CONFIG_MTK_MTD_NAND) += nand/
obj-$(CONFIG_MTK_DUM_CHAR) += dum-char/
obj-$(CONFIG_MTK_DYNAMIC_BOOST) += dynamic_boost/

obj-$(CONFIG_MTK_RRC)	+= rrc/

# Crypto performance tools (disable by default)
obj-n += crypto/

#widevine drm
obj-$(CONFIG_TRUSTONIC_TEE_SUPPORT) += secwidevine/
obj-$(CONFIG_MTK_POWER_GS)      += power_gs/
obj-$(CONFIG_MTK_TINYSYS_SCP_SUPPORT) += scp/
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/cpu_stress/),)
obj-$(CONFIG_MTK_CPU_STRESS)	+= cpu_stress/
endif

# TDMB
obj-$(CONFIG_LGE_BROADCAST_TDMB) += broadcast/

# LATIN DTV
obj-$(CONFIG_LGE_BROADCAST_SBTVD_LATIN) += broadcast/
