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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_pwm.h>
#include <mach/mt_pwm_hal.h>

#include "mode.h"
//#include <mt-plat/mt6735/include/mach/mt_pwm_hal.h>




/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#ifdef _LOG_USER_
#define PK_DBG_FUNC(fmt, arg...)
#else
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#endif
#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

extern int mtkflash_gpio_set(int pin, int mod);

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);


#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;


/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

#ifndef CONFIG_MTK_PWM
s32 pwm_set_spec_config(struct pwm_spec_config *conf)
{
    return 0;
}

void mt_pwm_disable(u32 pwm_no, u8 pmic_pad)
{
}
#endif


int FL_cur(int duty)
{
    struct pwm_spec_config pwmconfig;
    PK_DBG("yjy FL_cur Start!");

#if 0
    if(mt_set_gpio_mode(Flash_PWM,GPIO_MODE_02)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(Flash_PWM,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(Flash_PWM, GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio out failed!! \n");}
#endif
    mtkflash_gpio_set(FLASH_CONCRL,PIN_PWM);
#if 1
    pwmconfig.pwm_no=PWM3;
    pwmconfig.mode=PWM_MODE_OLD;
    pwmconfig.clk_src=PWM_CLK_OLD_MODE_BLOCK;
    pwmconfig.clk_div=CLK_DIV4;
    if(duty == 0){
        pwmconfig.PWM_MODE_OLD_REGS.THRESH= 0;

    }else{
	    pwmconfig.PWM_MODE_OLD_REGS.THRESH= (duty+1)*15;
    }
    pwmconfig.PWM_MODE_OLD_REGS.DATA_WIDTH=255;
	pwmconfig.PWM_MODE_OLD_REGS.IDLE_VALUE=0;
    pwmconfig.PWM_MODE_OLD_REGS.GUARD_VALUE=0;
    pwmconfig.PWM_MODE_OLD_REGS.WAVE_NUM=0;
    pwmconfig.pmic_pad=0;
    pwmconfig.PWM_MODE_OLD_REGS.GDURATION = 0;

    if(0!=pwm_set_spec_config(&pwmconfig))
        PK_DBG("pwm_set_spec_config(&pwmconfig)) Fail!!");
#endif

return 0;

}

int FL_Enable(void)
{

    PK_DBG("FL_enable g_duty=%d\n",g_duty);
    //mtkflash_gpio_set(FLASH_CONCRL,PIN_HIGH);

	if(g_duty <= 4 )//torch
    {
		mtkflash_gpio_set(FLASH_EN,PIN_LOW);
        mtkflash_gpio_set(FLASH_CONCRL,PIN_HIGH);
		//msleep(6);
		//FL_cur((g_duty+1)*3);
    }
    else       //flash	
    {
		FL_cur(g_duty);
        mtkflash_gpio_set(FLASH_EN,PIN_HIGH);
		
        return 0;

    }
    return 0;
}



int FL_Disable(void)
{
     PK_DBG("yjy FL_Disable Start!");
#if 0
    if(mt_set_gpio_mode(Flash_PWM,GPIO_MODE_02)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(Flash_PWM,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(Flash_PWM, GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio out failed!! \n");}
    msleep(2);
    if(mt_set_gpio_mode(GPIO_Flash_EN,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_Flash_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_Flash_EN, GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio out failed!! \n");}
#endif
    mtkflash_gpio_set(FLASH_CONCRL,PIN_LOW);
    msleep(2);
    mtkflash_gpio_set(FLASH_EN,PIN_LOW);
    return 0;

}



int FL_dim_duty(kal_uint32 duty)
{
    PK_DBG("yjy FL_dim_duty Start!");

	g_duty=duty;
    if( g_duty < 0 )
    {
        g_duty = 0;
    }
    else if( g_duty >= 16 )
    {
        g_duty = 16;
    }
	PK_DBG("FL_dim_duty\n");
    return 0;
}


int FL_Init(void)
{
    PK_DBG("yjy FL_Init Start!");
    mtkflash_gpio_set(FLASH_CONCRL,PIN_PWM);
    PK_DBG("yjy FL_Init Now Set GPIO to PWM mode!!\n");
    mtkflash_gpio_set(FLASH_CONCRL,PIN_LOW);
    msleep(2);
    mtkflash_gpio_set(FLASH_EN,PIN_LOW);
    return 0;


    PK_DBG(" FL_Init Done !\n");

    return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("LM3642 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
