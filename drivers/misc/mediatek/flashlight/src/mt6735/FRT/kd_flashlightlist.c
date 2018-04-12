/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef WIN32
#include "win_test.h"
#include "stdio.h"
#include "kd_flashlight.h"
#else
#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <mach/upmu_sw.h>
#endif
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
#include <mach/mt_pbm.h>
#include "mode.h"

/* device tree */
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
/* E1MM add proc sys use */
#include <linux/proc_fs.h>   //proc file use
#include <linux/seq_file.h>

/* GPIO Pin control*/
struct platform_device *flash_plt_dev = NULL;
struct pinctrl *flashctrl = NULL;
struct pinctrl_state *flash_en_h = NULL;
struct pinctrl_state *flash_en_l = NULL;
struct pinctrl_state *flash_pwm_h = NULL;
struct pinctrl_state *flash_pwm_l = NULL;

int mtkflash_gpio_init(struct platform_device *pdev)
{
	int ret = 0;
	flashctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flashctrl)) {
		dev_err(&pdev->dev, "Cannot find camera flash pinctrl!");
		ret = PTR_ERR(flashctrl);
	}

	/*flash0 GPIO Ping initialization */
	flash_en_h = pinctrl_lookup_state(flashctrl, "flash0_state1");
	if (IS_ERR(flash_en_h)) {
		ret = PTR_ERR(flash_en_h);
		pr_debug("%s : pinctrl err, flash_en_h\n", __func__);
	}

	flash_en_l = pinctrl_lookup_state(flashctrl, "flash0_state0");
	if (IS_ERR(flash_en_l)) {
		ret = PTR_ERR(flash_en_l);
		pr_debug("%s : pinctrl err, flash_en_l\n", __func__);
	}

	/*flash1 GPIO Ping initialization */
	flash_pwm_h = pinctrl_lookup_state(flashctrl, "flash1_state1");
	if (IS_ERR(flash_pwm_h)) {
		ret = PTR_ERR(flash_pwm_h);
		pr_debug("%s : pinctrl err, flash1_h\n", __func__);
	}

	flash_pwm_l = pinctrl_lookup_state(flashctrl, "flash1_state0");
	if (IS_ERR(flash_pwm_l)) {
		ret = PTR_ERR(flash_pwm_l);
		pr_debug("%s : pinctrl err, flash1_l\n", __func__);
	}

	return ret;
}

int mtkflash_gpio_set(int pin, int mod)
{
	int ret = 0;

	if (pin == FLASH_EN){
		if (mod == PIN_HIGH)
			pinctrl_select_state(flashctrl, flash_en_h);
		else if (mod == PIN_LOW)
			pinctrl_select_state(flashctrl, flash_en_l);
	} else {
		if (mod == PIN_HIGH)
			pinctrl_select_state(flashctrl, flash_pwm_h);
		else if (mod == PIN_LOW)
			pinctrl_select_state(flashctrl, flash_pwm_l);
	}

	pr_debug("pin(%d) mod(%d)\n", pin, mod);

	return ret;
}

/******************************************************************************
 * Definition
******************************************************************************/

/* device name and major number */
#define FLASHLIGHT_DEVNAME            "kd_camera_flashlight"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#ifdef WIN32
#define logI(fmt, ...)    {printf(fmt, __VA_ARGS__); printf("\n"); }
#else
#define PFX "[KD_CAMERA_FLASHLIGHT]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(PFX "%s: " fmt, __func__ , ##arg)

/*#define DEBUG_KD_STROBE*/
#ifdef DEBUG_KD_STROBE
#define logI PK_DBG_FUNC
#else
#define logI(a, ...)
#endif
#endif
/* ============================== */
/* variables */
/* ============================== */
static FLASHLIGHT_FUNCTION_STRUCT
	*g_pFlashInitFunc[e_Max_Sensor_Dev_Num][e_Max_Strobe_Num_Per_Dev][e_Max_Part_Num_Per_Dev];
static int gLowBatDuty[e_Max_Sensor_Dev_Num][e_Max_Strobe_Num_Per_Dev];
static int g_strobePartId[e_Max_Sensor_Dev_Num][e_Max_Strobe_Num_Per_Dev];

/* ============================== */
/* functions */
/* ============================== */
int globalInit(void)
{
	int i;
	int j;
	int k;

	logI("globalInit");
	for (i = 0; i < e_Max_Sensor_Dev_Num; i++)
		for (j = 0; j < e_Max_Strobe_Num_Per_Dev; j++) {
			gLowBatDuty[i][j] = -1;
			g_strobePartId[i][j] = 1;
			for (k = 0; k < e_Max_Part_Num_Per_Dev; k++)
				g_pFlashInitFunc[i][j][k] = 0;
		}
	return 0;
}

int checkAndRelease(void)
{
	int i;
	int j;
	int k;

	for (i = 0; i < e_Max_Sensor_Dev_Num; i++)
		for (j = 0; j < e_Max_Strobe_Num_Per_Dev; j++)
			for (k = 0; k < e_Max_Part_Num_Per_Dev; k++) {
				if (g_pFlashInitFunc[i][j][k] != 0) {
					logI("checkAndRelease %d %d %d", i, j, k);
					g_pFlashInitFunc[i][j][k]->flashlight_release(0);
					g_pFlashInitFunc[i][j][k] = 0;
				}
			}
	return 0;
}

int getSensorDevIndex(int sensorDev)
{
	if (sensorDev == e_CAMERA_MAIN_SENSOR)
		return 0;
	else if (sensorDev == e_CAMERA_SUB_SENSOR)
		return 1;
	else if (sensorDev == e_CAMERA_MAIN_2_SENSOR)
		return 2;
	/* else */
	logI("sensorDev=%d is wrong", sensorDev);
	return -1;
}

int getStrobeIndex(int strobeId)
{
	if (strobeId < 1 || strobeId > 2) {
		logI("strobeId=%d is wrong", strobeId);
		return -1;
	}
	return strobeId - 1;
}

int getPartIndex(int partId)
{
	if (partId < 1 || partId > 2) {
		logI("partId=%d is wrong", partId);
		return -1;
	}
	return partId - 1;
}

MINT32 default_flashlight_open(void *pArg)
{
	logI("[default_flashlight_open] E ~");
	return 0;
}

MINT32 default_flashlight_release(void *pArg)
{
	logI("[default_flashlight_release] E ~");
	return 0;
}

MINT32 default_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int iFlashType = (int)FLASHLIGHT_NONE;
	kdStrobeDrvArg kdArg;
	unsigned long copyRet;

	copyRet = copy_from_user(&kdArg, (void *)arg, sizeof(kdStrobeDrvArg));

	switch (cmd) {
	case FLASHLIGHTIOC_G_FLASHTYPE:
		iFlashType = FLASHLIGHT_NONE;
		kdArg.arg = iFlashType;
		if (copy_to_user((void __user *)arg, (void *)&kdArg, sizeof(kdStrobeDrvArg))) {
			logI("[FLASHLIGHTIOC_G_FLASHTYPE] ioctl copy to user failed ~");
			return -EFAULT;
		}
		break;
	default:
		logI("[default_flashlight_ioctl] ~");
		break;
	}
	return i4RetValue;
}

FLASHLIGHT_FUNCTION_STRUCT defaultFlashlightFunc = {
	default_flashlight_open,
	default_flashlight_release,
	default_flashlight_ioctl,
};

UINT32 strobeInit_dummy(FLASHLIGHT_FUNCTION_STRUCT **pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &defaultFlashlightFunc;
	return 0;
}

/* ======================================================================== */
static int setFlashDrv(int sensorDev, int strobeId)
{
	int partId;
	int sensorDevIndex;
	int strobeIndex;
	int partIndex;
	FLASHLIGHT_FUNCTION_STRUCT **ppF = 0;

	sensorDevIndex = getSensorDevIndex(sensorDev);
	strobeIndex = getStrobeIndex(strobeId);
	if (sensorDevIndex < 0 || strobeIndex < 0)
		return -1;
	partId = g_strobePartId[sensorDevIndex][strobeIndex];
	partIndex = getPartIndex(partId);
	if (partIndex < 0)
		return -1;

	logI("setFlashDrv sensorDev=%d, strobeId=%d, partId=%d ~", sensorDev, strobeId, partId);

	ppF = &g_pFlashInitFunc[sensorDevIndex][strobeIndex][partIndex];
	if (sensorDev == e_CAMERA_MAIN_SENSOR) {
#if defined(DUMMY_FLASHLIGHT)
		strobeInit_dummy(ppF);
#else
		if (strobeId == 1) {
			if (partId == 1)
				constantFlashlightInit(ppF);
			else if (partId == 2)
				strobeInit_main_sid1_part2(ppF);
		} else if (strobeId == 2) {
			if (partId == 1)
				strobeInit_main_sid2_part1(ppF);

			else if (partId == 2)
				strobeInit_main_sid2_part2(ppF);
		}
#endif
	} else if (sensorDev == e_CAMERA_SUB_SENSOR) {
		if (strobeId == 1) {
			if (partId == 1)
				subStrobeInit(ppF);
			else if (partId == 2)
				strobeInit_sub_sid1_part2(ppF);
		} else if (strobeId == 2) {
			if (partId == 1)
				strobeInit_sub_sid2_part1(ppF);
			else if (partId == 2)
				strobeInit_sub_sid2_part2(ppF);
		}
	}

	if ((*ppF) != 0) {
		(*ppF)->flashlight_open(0);
		logI("setFlashDrv ok %d", __LINE__);
	} else {
		logI("set function pointer not found!!");
		return -1;
	}
	return 0;
}

/*
static int decFlash(void)
{
	int i;
	int j;
	int k;
	int duty;
	for (i = 0; i < e_Max_Sensor_Dev_Num; i++)
		for (j = 0; j < e_Max_Strobe_Num_Per_Dev; j++)
			for (k = 0; k < e_Max_Part_Num_Per_Dev; k++) {
				if (g_pFlashInitFunc[i][j][k] != 0) {
					if (gLowBatDuty[i][j] != -1) {
						duty = gLowBatDuty[i][j];
						logI("decFlash i,j,k,duty %d %d %d %d", i, j, k,
						     duty);
						g_pFlashInitFunc[i][j][k]->flashlight_ioctl
						    (FLASH_IOC_SET_DUTY, duty);
					}
				}
			}
	return 0;
}
*/
/*
static int closeFlash(void)
{
	int i;
	int j;
	int k;

	logI("closeFlash ln=%d", __LINE__);
	for (i = 0; i < e_Max_Sensor_Dev_Num; i++) {
		// logI("closeFlash ln=%d %d",__LINE__,i);
		for (j = 0; j < e_Max_Strobe_Num_Per_Dev; j++) {
		//	 logI("closeFlash ln=%d %d",__LINE__,j);
			for (k = 0; k < e_Max_Part_Num_Per_Dev; k++) {
				// logI("closeFlash ln=%d %d %d",__LINE__,k, (int)g_pFlashInitFunc[i][j][k]);
				if (g_pFlashInitFunc[i][j][k] != 0) {
					logI("closeFlash i,j,k %d %d %d", i, j, k);
					g_pFlashInitFunc[i][j][k]->flashlight_ioctl
						(FLASH_IOC_SET_ONOFF, 0);
				}
			}
		}
	}
	return 0;
}
*/
/* @@{ */


#define LOW_BATTERY_LEVEL_0 0
/*
#define LOW_BATTERY_LEVEL_1 1
#define LOW_BATTERY_LEVEL_2 2
#define BATTERY_PERCENT_LEVEL_0 0
#define BATTERY_PERCENT_LEVEL_1 1
*/

/* /}@@ */
static int gLowPowerVbat = LOW_BATTERY_LEVEL_0;

/*
static void Lbat_protection_powerlimit_flash(LOW_BATTERY_LEVEL level)
{
	logI("Lbat_protection_powerlimit_flash %d (%d %d %d %d)\n", level, LOW_BATTERY_LEVEL_0,
	     LOW_BATTERY_LEVEL_1, LOW_BATTERY_LEVEL_2, __LINE__);
	logI("Lbat_protection_powerlimit_flash %d (%d %d %d %d)\n", level, LOW_BATTERY_LEVEL_0,
	     LOW_BATTERY_LEVEL_1, LOW_BATTERY_LEVEL_2, __LINE__);
	if (level == LOW_BATTERY_LEVEL_0) {
		gLowPowerVbat = LOW_BATTERY_LEVEL_0;
	} else if (level == LOW_BATTERY_LEVEL_1) {
		closeFlash();
		gLowPowerVbat = LOW_BATTERY_LEVEL_1;

	} else if (level == LOW_BATTERY_LEVEL_2) {
		closeFlash();
		gLowPowerVbat = LOW_BATTERY_LEVEL_2;
	} else {
		// unlimit cpu and gpu
	}
}
*/
static int gLowPowerPer = BATTERY_PERCENT_LEVEL_0;

/*
static void bat_per_protection_powerlimit_flashlight(BATTERY_PERCENT_LEVEL level)
{
	logI("bat_per_protection_powerlimit_flashlight %d (%d %d %d)\n", level,
	     BATTERY_PERCENT_LEVEL_0, BATTERY_PERCENT_LEVEL_1, __LINE__);
	logI("bat_per_protection_powerlimit_flashlight %d (%d %d %d)\n", level,
	     BATTERY_PERCENT_LEVEL_0, BATTERY_PERCENT_LEVEL_1, __LINE__);
	if (level == BATTERY_PERCENT_LEVEL_0) {
		gLowPowerPer = BATTERY_PERCENT_LEVEL_0;
	} else if (level == BATTERY_PERCENT_LEVEL_1) {
		closeFlash();
		gLowPowerPer = BATTERY_PERCENT_LEVEL_1;
	} else {

	//	unlimit cpu and gpu
	}
}
*/
/*
static int gLowPowerOc=BATTERY_OC_LEVEL_0;

void bat_oc_protection_powerlimit(BATTERY_OC_LEVEL level)
{
    logI("bat_oc_protection_powerlimit %d (%d %d %d)\n", level, BATTERY_OC_LEVEL_0, BATTERY_OC_LEVEL_1,__LINE__);
    logI("bat_oc_protection_powerlimit %d (%d %d %d)\n", level, BATTERY_OC_LEVEL_0, BATTERY_OC_LEVEL_1,__LINE__);
    if (level == BATTERY_OC_LEVEL_1){
	// battery OC trigger CPU Limit to under 4 X 0.8G
	closeFlash();
	gLowPowerOc=BATTERY_OC_LEVEL_1;
    }
    else{
	//unlimit cpu and gpu
	gLowPowerOc=BATTERY_OC_LEVEL_0;
    }
}
*/

/* ======================================================================== */

static long flashlight_ioctl_core(struct file *file, unsigned int cmd, unsigned long arg)
{
	int partId;
	int sensorDevIndex;
	int strobeIndex;
	int partIndex;
	int i4RetValue = 0;
	kdStrobeDrvArg kdArg;
	unsigned long copyRet;

	copyRet = copy_from_user(&kdArg, (void *)arg, sizeof(kdStrobeDrvArg));
	logI("flashlight_ioctl cmd=0x%x(nr=%d), senorDev=0x%x ledId=0x%x arg=0x%lx", cmd,
			_IOC_NR(cmd), kdArg.sensorDev, kdArg.strobeId, (unsigned long)kdArg.arg);
	sensorDevIndex = getSensorDevIndex(kdArg.sensorDev);
	strobeIndex = getStrobeIndex(kdArg.strobeId);
	if (sensorDevIndex < 0 || strobeIndex < 0)
		return -1;
	partId = g_strobePartId[sensorDevIndex][strobeIndex];
	partIndex = getPartIndex(partId);
	if (partIndex < 0)
		return -1;

	switch (cmd) {
		case FLASH_IOC_GET_PROTOCOL_VERSION:
			i4RetValue = 1;
			break;
		case FLASH_IOC_IS_LOW_POWER:
			logI("FLASH_IOC_IS_LOW_POWER");
			{
				int isLow = 0;

				if (gLowPowerPer != BATTERY_PERCENT_LEVEL_0
						|| gLowPowerVbat != LOW_BATTERY_LEVEL_0)
					isLow = 1;
				logI("FLASH_IOC_IS_LOW_POWER %d %d %d", gLowPowerPer, gLowPowerVbat, isLow);
				kdArg.arg = isLow;
				if (copy_to_user
						((void __user *)arg, (void *)&kdArg, sizeof(kdStrobeDrvArg))) {
					logI("[FLASH_IOC_IS_LOW_POWER] ioctl copy to user failed ~");
					return -EFAULT;
				}
			}
			break;

		case FLASH_IOC_LOW_POWER_DETECT_START:
			logI("FLASH_IOC_LOW_POWER_DETECT_START");
			gLowBatDuty[sensorDevIndex][strobeIndex] = kdArg.arg;
			break;

		case FLASH_IOC_LOW_POWER_DETECT_END:
			logI("FLASH_IOC_LOW_POWER_DETECT_END");
			gLowBatDuty[sensorDevIndex][strobeIndex] = -1;
			break;
		case FLASHLIGHTIOC_X_SET_DRIVER:
			i4RetValue = setFlashDrv(kdArg.sensorDev, kdArg.strobeId);
			break;
		case FLASH_IOC_GET_PART_ID:
		case FLASH_IOC_GET_MAIN_PART_ID:
		case FLASH_IOC_GET_SUB_PART_ID:
		case FLASH_IOC_GET_MAIN2_PART_ID:
			{
				int partId;

				partId = strobe_getPartId(kdArg.sensorDev, kdArg.strobeId);
				g_strobePartId[sensorDevIndex][strobeIndex] = partId;
				kdArg.arg = partId;
				if (copy_to_user
						((void __user *)arg, (void *)&kdArg, sizeof(kdStrobeDrvArg))) {
					logI("[FLASH_IOC_GET_PART_ID] ioctl copy to user failed ~");
					return -EFAULT;
				}
				logI("FLASH_IOC_GET_PART_ID line=%d partId=%d", __LINE__, partId);
			}
			break;
		case FLASH_IOC_SET_ONOFF:
			{
				FLASHLIGHT_FUNCTION_STRUCT *pF;

				pF = g_pFlashInitFunc[sensorDevIndex][strobeIndex][partIndex];
				if (pF != 0) {
					kicker_pbm_by_flash(kdArg.arg);
					i4RetValue = pF->flashlight_ioctl(cmd, kdArg.arg);

				} else {
					logI("[FLASH_IOC_SET_ONOFF] function pointer is wrong -");
				}
			}
			break;
		case FLASH_IOC_UNINIT:
			{
				FLASHLIGHT_FUNCTION_STRUCT *pF;

				pF = g_pFlashInitFunc[sensorDevIndex][strobeIndex][partIndex];
				if (pF != 0) {
					i4RetValue = pF->flashlight_release((void *)0);
					pF = 0;

				} else {
					logI("[FLASH_IOC_UNINIT] function pointer is wrong ~");
				}
			}
			break;
		default:
			{
				FLASHLIGHT_FUNCTION_STRUCT *pF;

				pF = g_pFlashInitFunc[sensorDevIndex][strobeIndex][partIndex];
				if (pF != 0)
					i4RetValue = pF->flashlight_ioctl(cmd, kdArg.arg);
				else
					logI("[default] function pointer is wrong ~");
			}
			break;
	}
	return i4RetValue;
}

static long flashlight_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err;
/* int dir; */
	err = flashlight_ioctl_core(file, cmd, arg);
	/* dir  = _IOC_DIR(cmd); */
	/* if(dir &_IOC_READ) */
	{
		/* copy_to_user */
	}
	return err;
}

#ifdef CONFIG_COMPAT

/*
static int compat_arg_struct_user32_to_kernel(
			struct StrobeDrvArg __user *data32,
			struct compat_StrobeDrvArg __user *data)
{
	compat_int_t i;
	int err=0;

	err |= get_user(i, &data32->sensorDev);
	err |= put_user(i, &data->sensorDev);

	err |= get_user(i, &data32->arg);
	err |= put_user(i, &data->arg);

	return err;
}

static int compat_arg_struct_kernel_to_user32(
			struct StrobeDrvArg __user *data32,
			struct compat_StrobeDrvArg __user *data)

{
	compat_int_t i;
	int err=0;

    err |= get_user(i, &data->sensorDev);
	err |= put_user(i, &data32->sensorDev);
	err |= get_user(i, &data->arg);
	err |= put_user(i, &data32->arg);

	return err;
}*/

static long my_ioctl_compat(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int err;
	/* int copyRet; */
	kdStrobeDrvArg *pUObj;

	logI("flash my_ioctl_compat2 line=%d cmd=%d arg=%ld\n", __LINE__, cmd, arg);
	pUObj = compat_ptr(arg);

	/*
	   kdStrobeDrvArg* pUObj;
	   pUObj = compat_ptr(arg);
	   kdStrobeDrvArg obj;
	   copyRet = copy_from_user(&obj , (void *)pUObj , sizeof(kdStrobeDrvArg));
	   logI("strobe arg %d %d %d\n", obj.sensorDev, obj.strobeId, obj.arg);
	   obj.arg = 23411;
	   copy_to_user((void __user *) arg , (void*)&obj , sizeof(kdStrobeDrvArg));
	 */

	/* data = compat_alloc_user_space(sizeof(*data)); */
	/* if (sys_data == NULL) */
	/* return -EFAULT; */
	/* err = compat_arg_struct_user32_to_kernel(data32, data); */
	/* arg2 = (unsigned long)data32; */
	err = flashlight_ioctl_core(filep, cmd, (unsigned long)pUObj);

	return err;

}
#endif

static int flashlight_open(struct inode *inode, struct file *file)
{
	int i4RetValue = 0;
	static int bInited;

	if (bInited == 0) {
		globalInit();
		bInited = 1;
	}
	logI("[flashlight_open] E ~");
	return i4RetValue;
}

static int flashlight_release(struct inode *inode, struct file *file)
{
	logI("[flashlight_release] E ~");

	checkAndRelease();

	return 0;
}

#ifdef WIN32
int fl_open(struct inode *inode, struct file *file)
{
	return flashlight_open(inode, file);
}

int fl_release(struct inode *inode, struct file *file)
{
	return flashlight_release(inode, file);
}

long fl_ioctrl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return flashlight_ioctl(file, cmd, arg);
}

#else
/* ======================================================================== */
/* ======================================================================== */
/* ======================================================================== */
/* Kernel interface */
static const struct file_operations flashlight_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = flashlight_ioctl,
	.open = flashlight_open,
	.release = flashlight_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = my_ioctl_compat,
#endif
};

/* ======================================================================== */
/* Driver interface */
/* ======================================================================== */
struct flashlight_data {
	spinlock_t lock;
	wait_queue_head_t read_wait;
	struct semaphore sem;
};
static struct class *flashlight_class;
static struct device *flashlight_device;
static struct flashlight_data flashlight_private;
static dev_t flashlight_devno;
static struct cdev flashlight_cdev;
/* ======================================================================== */
#define ALLOC_DEVNO
static int flashlight_probe(struct platform_device *dev)
{
	int ret = 0, err = 0;
	mtkflash_gpio_init(dev);
	logI("[flashlight_probe] start ~");

#ifdef ALLOC_DEVNO
	ret = alloc_chrdev_region(&flashlight_devno, 0, 1, FLASHLIGHT_DEVNAME);
	if (ret) {
		logI("[flashlight_probe] alloc_chrdev_region fail: %d ~", ret);
		goto flashlight_probe_error;
	} else {
		logI("[flashlight_probe] major: %d, minor: %d ~", MAJOR(flashlight_devno),
		     MINOR(flashlight_devno));
	}
	cdev_init(&flashlight_cdev, &flashlight_fops);
	flashlight_cdev.owner = THIS_MODULE;
	err = cdev_add(&flashlight_cdev, flashlight_devno, 1);
	if (err) {
		logI("[flashlight_probe] cdev_add fail: %d ~", err);
		goto flashlight_probe_error;
	}
#else
#define FLASHLIGHT_MAJOR 242
	ret = register_chrdev(FLASHLIGHT_MAJOR, FLASHLIGHT_DEVNAME, &flashlight_fops);
	if (ret != 0) {
		logI("[flashlight_probe] Unable to register chardev on major=%d (%d) ~",
		     FLASHLIGHT_MAJOR, ret);
		return ret;
	}
	flashlight_devno = MKDEV(FLASHLIGHT_MAJOR, 0);
#endif

	flashlight_class = class_create(THIS_MODULE, "flashlightdrv");
	if (IS_ERR(flashlight_class)) {
		logI("[flashlight_probe] Unable to create class, err = %d ~",
		     (int)PTR_ERR(flashlight_class));
		goto flashlight_probe_error;
	}

	flashlight_device =
	    device_create(flashlight_class, NULL, flashlight_devno, NULL, FLASHLIGHT_DEVNAME);
	if (NULL == flashlight_device) {
		logI("[flashlight_probe] device_create fail ~");
		goto flashlight_probe_error;
	}

	/* initialize members */
	spin_lock_init(&flashlight_private.lock);
	init_waitqueue_head(&flashlight_private.read_wait);
	/* init_MUTEX(&flashlight_private.sem); */
	sema_init(&flashlight_private.sem, 1);

	logI("[flashlight_probe] Done ~");
	return 0;

flashlight_probe_error:
#ifdef ALLOC_DEVNO
	if (err == 0)
		cdev_del(&flashlight_cdev);
	if (ret == 0)
		unregister_chrdev_region(flashlight_devno, 1);
#else
	if (ret == 0)
		unregister_chrdev(MAJOR(flashlight_devno), FLASHLIGHT_DEVNAME);
#endif
	return -1;
}

static int flashlight_remove(struct platform_device *dev)
{

	logI("[flashlight_probe] start\n");

#ifdef ALLOC_DEVNO
	cdev_del(&flashlight_cdev);
	unregister_chrdev_region(flashlight_devno, 1);
#else
	unregister_chrdev(MAJOR(flashlight_devno), FLASHLIGHT_DEVNAME);
#endif
	device_destroy(flashlight_class, flashlight_devno);
	class_destroy(flashlight_class);

	logI("[flashlight_probe] Done ~");
	return 0;
}

static void flashlight_shutdown(struct platform_device *dev)
{

	logI("[flashlight_shutdown] start\n");
	checkAndRelease();
	logI("[flashlight_shutdown] Done ~");
}

static const struct of_device_id flashlight_of_match[] = {
	{.compatible = "mediatek,camera_flashlight",},
	{},
};

static struct platform_driver flashlight_platform_driver = {
	.probe = flashlight_probe,
	.remove = flashlight_remove,
	.shutdown = flashlight_shutdown,
	.driver = {
		   .name = FLASHLIGHT_DEVNAME,
		   .owner = THIS_MODULE,
	       .of_match_table = flashlight_of_match
		   },
};
#ifndef CONFIG_OF
static struct platform_device flashlight_platform_device = {
	.name = FLASHLIGHT_DEVNAME,
	.id = 0,
	.dev = {
		}
};
#endif

extern int FL_Init(void);
extern int FL_Uninit(void);
extern int FL_Enable(void);
extern int FL_Disable(void);
extern int FL_dim_duty(kal_uint32 duty);
static ssize_t  flash_proc_write(struct file *file, const char *buffer, size_t count,
		loff_t *data)
{
	int  ret = 0;
	int  val = 0;
	char regBuf[2] = {'\0'};
	u32  u4CopyBufSize = (count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);

	if (copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	sscanf(regBuf, "%d\n", &val);
	logI("%s: Input = %d.\n", __func__, val);

	if(val <0 || val>2)
		logI("%s: val number is wrong!\n", __func__);

	if((int)val)
	{
		ret = FL_Init();
		ret |= FL_dim_duty(0);
		ret |= FL_Enable();
	}
	else
	{
		ret |= FL_Disable();
		ret |= FL_Uninit();
	}

	if (ret)
		return -EFAULT;

	return count;
}

static ssize_t  flash_proc_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
	return 0;
}
static  struct file_operations flash_proc_fops = {
	.read = flash_proc_read,
	.write = flash_proc_write,
};

static int __init flashlight_init(void)
{
	int ret = 0;

	logI("[flashlight_probe] start ~");
	proc_create("driver/flashlight", 0, NULL, &flash_proc_fops);

#ifndef CONFIG_OF
	ret = platform_device_register(&flashlight_platform_device);
	if (ret) {
		logI("[flashlight_probe] platform_device_register fail ~");
		return ret;
	}
#endif
	ret = platform_driver_register(&flashlight_platform_driver);
	if (ret) {
		logI("[flashlight_probe] platform_driver_register fail ~");
		return ret;
	}

//	register_low_battery_notify(&Lbat_protection_powerlimit_flash, LOW_BATTERY_PRIO_FLASHLIGHT);
//	register_battery_percent_notify(&bat_per_protection_powerlimit_flashlight,
//					BATTERY_PERCENT_PRIO_FLASHLIGHT);
/* @@    register_battery_oc_notify(&bat_oc_protection_powerlimit, BATTERY_OC_PRIO_FLASHLIGHT); */

	logI("[flashlight_probe] done! ~");
	return ret;
}

static void __exit flashlight_exit(void)
{
	logI("[flashlight_probe] start ~");
	platform_driver_unregister(&flashlight_platform_driver);
	/* to flush work queue */
	/* flush_scheduled_work(); */
	logI("[flashlight_probe] done! ~");
}

/* ======================================================== */
module_init(flashlight_init);
module_exit(flashlight_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jackie Su <jackie.su@mediatek.com>");
MODULE_DESCRIPTION("Flashlight control Driver");

#endif
