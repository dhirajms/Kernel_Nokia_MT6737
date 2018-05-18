/* drivers/input/touchscreen/nt11206/NVTtouch_206.h
 *
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 *
 * $Revision: 3459 $
 * $Date: 2016-02-18 17:38:19 +0800 (週四, 18 二月 2016) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
//#include <linux/io.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
//#include <linux/rtpm_prio.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#endif
#ifdef MT6577
#include <mach/mt6577_pm_ldo.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_boot.h>
#endif

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <cust_eint.h>
#endif

#include <linux/jiffies.h>

#define SWITCH_OFF                  0
#define SWITCH_ON                   1

//---GPIO number---
#define I2C_DMA_SUPPORT      1
#define I2C_MASTER_CLOCK   300

#define DMA_MAX_TRANSACTION_LENGTH        255   // for DMA mode
#define DMA_MAX_I2C_TRANSFER_SIZE        (DMA_MAX_TRANSACTION_LENGTH - 1)
#define MAX_TRANSACTION_LENGTH            8
#define MAX_I2C_TRANSFER_SIZE            (MAX_TRANSACTION_LENGTH - 1)


//---INT trigger mode---
#define EINTF_TRIGGER_RISING       0x00000001 
//#define EINTF_TRIGGER_FALLING    0x00000002 
//#define EINTF_TRIGGER_HIGH       0x00000004 
//#define EINTF_TRIGGER_LOW        0x00000008 
#define INT_TRIGGER_TYPE EINTF_TRIGGER_RISING


//---I2C driver info.---
#define NVT_I2C_NAME "NVT-ts"
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62


//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"


//---Touch info.---
#define TOUCH_MAX_WIDTH 720
#define TOUCH_MAX_HEIGHT 1280
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 3
#if TOUCH_KEY_NUM > 0
//extern const uint16_t touch_key_array_1[TOUCH_KEY_NUM];
#endif


//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define MT_PROTOCOL_B 1
#define WAKEUP_GESTURE 0
#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif


#define BOOT_UPDATE_FIRMWARE 1

#define BOOT_UPDATE_HeaderFile 1
#if (BOOT_UPDATE_HeaderFile)			//[20161213,jx]For FIH_E1
#else
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"
#endif//#if(BOOT_UPDATE_HeaderFile)
#define NVT_INFO(fmt, arg...)       pr_info("<<NVT-INF>>[%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define NVT_ERROR(fmt, arg...)      pr_err("<<NVT-ERR>>[%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define NVT_DEBUG(fmt,arg...)       do{\
										if(NVT_DEBUG_ON)\
                                        printk("<<-NVT-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
									}while(0)

struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct nvt_work;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t x_num;
	uint8_t y_num;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
	struct i2c_client *client;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN	// normal run
} RST_COMPLETE_STATE;

#endif /* _LINUX_NVT_TOUCH_H */
