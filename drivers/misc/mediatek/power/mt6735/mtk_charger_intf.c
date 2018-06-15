/*
 * Copyright (C) 2016 MediaTek Inc.
 * ShuFanLee <shufan_lee@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/mt_gpio.h>
#include <mt-plat/battery_meter.h>
#include <mach/upmu_hw.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>

#include <linux/delay.h>
#include <linux/reboot.h>

#include "mtk_charger_intf.h"


/* Necessary functions for integrating with MTK */
/* All of them are copied from original source code of MTK */


kal_bool charging_type_det_done = KAL_TRUE;

const u32 VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#ifndef CUST_GPIO_VIN_SEL
#define CUST_GPIO_VIN_SEL 18
#endif

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#define SW_POLLING_PERIOD 100	/* 100 ms */
#define MSEC_TO_NSEC(x)		(x * 1000000UL)

static DEFINE_MUTEX(diso_polling_mutex);
static DECLARE_WAIT_QUEUE_HEAD(diso_polling_thread_wq);
static struct hrtimer diso_kthread_timer;
static kal_bool diso_thread_timeout = KAL_FALSE;
static struct delayed_work diso_polling_work;
static void diso_polling_handler(struct work_struct *work);
static DISO_Polling_Data DISO_Polling;
#endif /* MTK_AUXADC_IRQ_SUPPORT */

int g_diso_state = 0;
int vin_sel_gpio_number = (CUST_GPIO_VIN_SEL | 0x80000000);

static char *DISO_state_s[8] = {
	"IDLE",
	"OTG_ONLY",
	"USB_ONLY",
	"USB_WITH_OTG",
	"DC_ONLY",
	"DC_WITH_OTG",
	"DC_WITH_USB",
	"DC_USB_OTG",
};
#endif /*CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT */


u32 charging_value_to_parameter(const u32 *parameter, const u32 array_size,
				       const u32 val)
{
	if (val < array_size)
		return parameter[val];

	battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
	return parameter[0];
}

u32 charging_parameter_to_value(const u32 *parameter, const u32 array_size,
				       const u32 val)
{
	u32 i;

	battery_log(BAT_LOG_FULL, "array_size = %d \r\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	battery_log(BAT_LOG_CRTI, "NO register value match \r\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}

static u32 bmt_find_closest_level(const u32 *pList, u32 number, u32 level)
{
	u32 i;
	u32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				battery_log(2, "zzf_%d<=%d     i=%d\n", pList[i], level, i);
				return pList[i];
			}
		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

/* The following functions are for chr_control_interface */

int mtk_charger_sw_init(struct mtk_charger_info *info, void *data)
{

	return 0;
}


int mtk_charger_set_hv_threshold(struct mtk_charger_info *info, void *data)
{
	int ret = 0;
	u32 set_hv_voltage;
	u32 array_size;
	u16 register_value;
	u32 voltage = *((u32 *)data);

	array_size = ARRAY_SIZE(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size, set_hv_voltage);
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);

	return ret;
}

int mtk_charger_get_hv_status(struct mtk_charger_info *info, void *data)
{
	int ret = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;
	pr_notice("[charging_get_hv_status] charger ok for bring up.\n");
#else
	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#endif

	return ret;
}

int mtk_charger_get_battery_status(struct mtk_charger_info *info, void *data)
{
	int ret = 0;
	unsigned int val = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;	/* battery exist */
	battery_log(BAT_LOG_CRTI, "[charging_get_battery_status] battery exist for bring up.\n");
#else
	val = pmic_get_register_value(PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#endif

	return ret;
}

int mtk_charger_get_charger_det_status(struct mtk_charger_info *info, void *data)
{
	int ret = 0;
	unsigned int val = 0;

#if defined(CONFIG_MTK_FPGA)
	val = 1;
	battery_log(BAT_LOG_CRTI, "[charging_get_charger_det_status] chr exist for fpga.\n");
#else
#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	val = pmic_get_register_value_nolock(PMIC_RGS_CHRDET);
#else
	if (((g_diso_state >> 1) & 0x3) != 0x0 || pmic_get_register_value_nolock(PMIC_RGS_CHRDET))
		val = KAL_TRUE;
	else
		val = KAL_FALSE;
#endif
#endif

	*(kal_bool *) (data) = val;

	return ret;
}

int mtk_charger_get_charger_type(struct mtk_charger_info *info, void *data)
{
	int ret = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
#endif

	return ret;
}

int mtk_charger_get_is_pcm_timer_trigger(struct mtk_charger_info *info, void *data)
{
	int ret = 0;
	/*
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = KAL_FALSE;
#else

	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;

	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());

#endif
*/
	return ret;
}


int mtk_charger_set_platform_reset(struct mtk_charger_info *info, void *data)
{
	int ret = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");

	kernel_restart("battery service reboot system");
	/* arch_reset(0,NULL); */
#endif
	return ret;
}


int mtk_charger_get_platform_boot_mode(struct mtk_charger_info *info, void *data)
{
	int ret = 0;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(unsigned int *) (data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif
	return ret;
}

int mtk_charger_set_power_off(struct mtk_charger_info *info, void *data)
{
	int ret = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
	kernel_power_off();
#endif

	return ret;
}

int mtk_charger_get_power_source(struct mtk_charger_info *info, void *data)
{
	int ret = 0;

#if 0				/* #if defined(MTK_POWER_EXT_DETECT) */
	if (MT_BOARD_PHONE == mt_get_board_type())
		*(kal_bool *) data = KAL_FALSE;
	else
		*(kal_bool *) data = KAL_TRUE;
#else
	*(kal_bool *) data = KAL_FALSE;
#endif

	return ret;
}

int mtk_charger_get_csdac_full_flag(struct mtk_charger_info *info, void *data)
{
	return -ENOTSUPP;
}

int mtk_charger_diso_init(struct mtk_charger_info *info, void *data)
{
	int ret = 0;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *) data;

	/* Initialization DISO Struct */
	pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;

	pDISO_data->diso_state.pre_otg_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vdc_state = DISO_OFFLINE;

	pDISO_data->chr_get_diso_state = KAL_FALSE;
	pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

	hrtimer_init(&diso_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	diso_kthread_timer.function = diso_kthread_hrtimer_func;
	INIT_DELAYED_WORK(&diso_polling_work, diso_polling_handler);

	kthread_run(diso_thread_kthread, NULL, "diso_thread_kthread");
	battery_log(BAT_LOG_CRTI, "[%s] done\n", __func__);

#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
	battery_log(BAT_LOG_CRTI, "[diso_eint]vdc eint irq registitation\n");
	mt_eint_set_hw_debounce(CUST_EINT_VDC_NUM, CUST_EINT_VDC_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_VDC_NUM, CUST_EINTF_TRIGGER_LOW, vdc_eint_handler, 0);
	mt_eint_mask(CUST_EINT_VDC_NUM);
#endif
#endif

	return ret;
}

int mtk_charger_get_diso_state(struct mtk_charger_info *info, void *data)
{
	int ret = 0;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	int diso_state = 0x0;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *) data;

	_get_diso_interrupt_state();
	diso_state = g_diso_state;
	battery_log(BAT_LOG_CRTI, "[do_chrdet_int_task] current diso state is %s!\n", DISO_state_s[diso_state]);
	if (((diso_state >> 1) & 0x3) != 0x0) {
		switch (diso_state) {
		case USB_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
#ifdef MTK_DISCRETE_SWITCH
#ifdef MTK_DSC_USE_EINT
			mt_eint_unmask(CUST_EINT_VDC_NUM);
#else
			set_vdc_auxadc_irq(DISO_IRQ_ENABLE, 1);
#endif
#endif
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_RISING);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_USB:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_OTG:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			break;
		default:	/* OTG only also can trigger vcdt IRQ */
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			battery_log(BAT_LOG_CRTI, " switch load vcdt irq triggerd by OTG Boost!\n");
			break;	/* OTG plugin no need battery sync action */
		}
	}

	if (DISO_ONLINE == pDISO_data->diso_state.cur_vdc_state)
		pDISO_data->hv_voltage = VDC_MAX_VOLTAGE;
	else
		pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;
#endif

	return ret;
}

int mtk_charger_set_vbus_ovp_en(struct mtk_charger_info *info, void *data)
{
	return -ENOTSUPP;
}

int mtk_charger_set_chrind_ck_pdn(struct mtk_charger_info *info, void *data)
{
	return -ENOTSUPP;
}

int mtk_charger_get_bif_vbat(struct mtk_charger_info *info, void *data)
{
	return -ENOTSUPP;
}

int mtk_charger_get_bif_tbat(struct mtk_charger_info *info, void *data)
{
	return -ENOTSUPP;
}

int mtk_charger_set_dp(struct mtk_charger_info *info, void *data)
{
	return -ENOTSUPP;
}

int mtk_charger_get_bif_is_exist(struct mtk_charger_info *info, void *data)
{
	return -ENOTSUPP;
}


extern kal_bool chargin_hw_init_done; /* Used by MTK battery driver */
static struct mtk_charger_info *g_mchr_info;
static struct mtk_charger_info *g_slave_mchr_info;

DEFINE_MUTEX(mtk_info_access_lock);
void mtk_charger_set_info(struct mtk_charger_info *mchr_info)
{
	mutex_lock(&mtk_info_access_lock);
	if (strcmp(mchr_info->name, "slave_charger") == 0)
		g_slave_mchr_info = mchr_info;
	else
		g_mchr_info = mchr_info;
	mutex_unlock(&mtk_info_access_lock);
}

static int mtk_chg_ctrl_intf(const struct mtk_charger_info *mchr_info,
	CHARGING_CTRL_CMD cmd, void *data)
{
	int ret = 0;
	const mtk_charger_intf *mchr_intf = NULL;


	if (!mchr_info) {
		battery_log(BAT_LOG_CRTI, "%s: no charger is ready\n",
			__func__);
		return -ENODEV;
	}

	if (!mchr_info->mchr_intf) {
		battery_log(BAT_LOG_CRTI, "%s: no ctrl intf is ready\n",
			__func__);
		return -ENOTSUPP;
	}

	mchr_intf = mchr_info->mchr_intf;

	if (cmd < CHARGING_CMD_NUMBER && mchr_intf[cmd])
		ret = mchr_intf[cmd](g_mchr_info, data);
	else
		ret = -ENOTSUPP;

	if (ret == -ENOTSUPP)
		battery_log(BAT_LOG_CRTI, "%s: function %d is not support\n",
			__func__, cmd);
	else if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: function %d failed, ret = %d\n",
			__func__, cmd, ret);

	return ret;

}

signed int rt9458_chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	return mtk_chg_ctrl_intf(g_mchr_info, cmd, data);
}

#if 0 /* Uncomment this if you need dual charger */
int slave_chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	return mtk_chg_ctrl_intf(g_slave_mchr_info, cmd, data);
}
#endif
