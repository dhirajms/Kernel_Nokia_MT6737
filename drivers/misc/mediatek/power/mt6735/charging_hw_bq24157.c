#include <linux/types.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/battery_common.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include "bq24157.h"
//#include <mt-plat/mt_gpio.h>

/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK	0
#define STATUS_FAIL	1
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


/* ============================================================ // */
/* Global variable */
/* ============================================================ // */

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0
int wireless_charger_gpio_number = (168 | 0x80000000);
#endif

/*
#if 1
#include <mach/gpio_const.h>
//modify by Jason. E1 GPIO_SWCHARGER_EN_PIN:gpio_80.
int bq24157_gpio_number   = ( GPIO80 | 0x80000000);  //GPIO_SWCHARGER_EN_PIN;
int bq24157_gpio_off_mode = GPIO_MODE_GPIO;
int bq24157_gpio_on_mode  = GPIO_MODE_GPIO;

#else
int bq24157_gpio_number   = (19 | 0x80000000);
int bq24157_gpio_off_mode = 0;
int bq24157_gpio_on_mode  = 0;
#endif
int bq24157_gpio_off_dir  = GPIO_DIR_OUT;
int bq24157_gpio_off_out  = GPIO_OUT_ONE;
int bq24157_gpio_on_dir   = GPIO_DIR_OUT;
int bq24157_gpio_on_out   = GPIO_OUT_ZERO;
*/

/*
 Charge voltage range is 3.5V to 4.44V with the offset of 3.5V,and
 steps of 20mV(default 3.54V).using register R2,bits B2-B7.
*/
const u32 bq24157_VBAT_CV_VTH[] = {
	BATTERY_VOLT_03_500000_V, BATTERY_VOLT_03_520000_V, BATTERY_VOLT_03_540000_V,
	    BATTERY_VOLT_03_560000_V,
	BATTERY_VOLT_03_580000_V, BATTERY_VOLT_03_600000_V, BATTERY_VOLT_03_620000_V,
	    BATTERY_VOLT_03_640000_V,
	BATTERY_VOLT_03_660000_V, BATTERY_VOLT_03_680000_V, BATTERY_VOLT_03_700000_V,
	    BATTERY_VOLT_03_720000_V,
	BATTERY_VOLT_03_740000_V, BATTERY_VOLT_03_760000_V, BATTERY_VOLT_03_780000_V,
	    BATTERY_VOLT_03_800000_V,
	BATTERY_VOLT_03_820000_V, BATTERY_VOLT_03_840000_V, BATTERY_VOLT_03_860000_V,
	    BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V, BATTERY_VOLT_03_920000_V, BATTERY_VOLT_03_940000_V,
	    BATTERY_VOLT_03_960000_V,
	BATTERY_VOLT_03_980000_V, BATTERY_VOLT_04_000000_V, BATTERY_VOLT_04_020000_V,
	    BATTERY_VOLT_04_040000_V,
	BATTERY_VOLT_04_060000_V, BATTERY_VOLT_04_080000_V, BATTERY_VOLT_04_100000_V,
	    BATTERY_VOLT_04_120000_V,
	BATTERY_VOLT_04_140000_V, BATTERY_VOLT_04_160000_V, BATTERY_VOLT_04_180000_V,
	    BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_220000_V, BATTERY_VOLT_04_240000_V, BATTERY_VOLT_04_260000_V,
	    BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V, BATTERY_VOLT_04_320000_V, BATTERY_VOLT_04_340000_V,
	    BATTERY_VOLT_04_360000_V,
	BATTERY_VOLT_04_380000_V, BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_420000_V,
	    BATTERY_VOLT_04_440000_V
};

const u32 bq24157_CS_VTH[] = {
	CHARGE_CURRENT_550_00_MA, CHARGE_CURRENT_650_00_MA, CHARGE_CURRENT_750_00_MA,
	    CHARGE_CURRENT_850_00_MA,
	CHARGE_CURRENT_950_00_MA, CHARGE_CURRENT_1050_00_MA, CHARGE_CURRENT_1150_00_MA,
	    CHARGE_CURRENT_1250_00_MA
};

/*
add for volue of Rsense is 56mR.
	667.8mA		789.2mA		911.6mA 	1032mA
	1153.5mA	1274.8mA	1396.2mA	1517.6mA

*/
const u32 bq24157_CS_VTH_E1[] = {
	CHARGE_CURRENT_650_00_MA, CHARGE_CURRENT_775_00_MA, CHARGE_CURRENT_900_00_MA,
	    CHARGE_CURRENT_1000_00_MA,
	CHARGE_CURRENT_1150_00_MA, CHARGE_CURRENT_1250_00_MA, CHARGE_CURRENT_1375_00_MA,
	    CHARGE_CURRENT_1500_00_MA
};



const u32 bq24157_INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_500_00_MA, CHARGE_CURRENT_800_00_MA,
	    CHARGE_CURRENT_MAX
};

const u32 bq24157_VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

// add for DPM at 20150604
const unsigned int bq24157_VBAT_SCV_VTH[]=
{
    BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_280000_V, BATTERY_VOLT_04_360000_V, BATTERY_VOLT_04_440000_V,
    BATTERY_VOLT_04_520000_V, BATTERY_VOLT_04_600000_V, BATTERY_VOLT_04_680000_V, BATTERY_VOLT_04_760000_V
};

u32 bq24157_charging_value_to_parameter(const u32 *parameter, const u32 array_size, const u32 val)
{
	if (val < array_size)
		return parameter[val];
	battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
	return parameter[0];
}

u32 bq24157_charging_parameter_to_value(const u32 *parameter, const u32 array_size, const u32 val)
{
	u32 i;

	for (i = 0; i < array_size; i++)
		if (val == *(parameter + i))
			return i;

	battery_log(BAT_LOG_CRTI, "NO register value match \r\n");

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
		for (i = (number - 1); i != 0; i--)	/* max value in the last element */
			if (pList[i] <= level)
				return pList[i];

		battery_log(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++)	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];

		battery_log(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

static u32 charging_hw_init(void *data)
{
	u32 status = STATUS_OK;
	static bool charging_init_flag = KAL_FALSE;

    /*
    mt_set_gpio_mode(bq24157_gpio_number,bq24157_gpio_on_mode);
    mt_set_gpio_dir(bq24157_gpio_number,bq24157_gpio_on_dir);
    mt_set_gpio_out(bq24157_gpio_number,bq24157_gpio_on_out);
	*/
	charger_pin_cd_config(KAL_FALSE);

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
	mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
#endif

	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	//bq24157_reg_config_interface(0x06,0x79); // ISAFE = 1250mA, VSAFE = 4.38V
	bq24157_reg_config_interface(0x06,0x7b); // ISAFE = 1518mA, VSAFE = 4.42V
	#else
	bq24157_reg_config_interface(0x06,0x70);
	#endif
	bq24157_reg_config_interface(0x00, 0xC0);	/* kick chip watch dog */
	bq24157_reg_config_interface(0x01, 0xb8);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	//bq24157_reg_config_interface(0x05,0x02); //VDPM=4.36V
	bq24157_reg_config_interface(0x05,0x03); //VDPM=4.44V
	if (!charging_init_flag) {
		bq24157_reg_config_interface(0x04, 0x1A);	/* termination current 121mA */
		charging_init_flag = KAL_TRUE;
	}
	return status;
}


static u32 charging_dump_register(void *data)
{
	u32 status = STATUS_OK;

	bq24157_dump_register();

	return status;
}


static u32 charging_enable(void *data)
{
	u32 status = STATUS_OK;
	u32 enable = *(u32 *) (data);

	if (KAL_TRUE == enable) {
		bq24157_set_ce(0);
		bq24157_set_hz_mode(0);
		bq24157_set_opa_mode(0);
	} else {

#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if (mt_usb_is_device())
#endif
    	{
            /*
	        mt_set_gpio_mode(bq24157_gpio_number,bq24157_gpio_off_mode);
	        mt_set_gpio_dir(bq24157_gpio_number,bq24157_gpio_off_dir);
	        mt_set_gpio_out(bq24157_gpio_number,bq24157_gpio_off_out);
			*/
			charger_pin_cd_config(KAL_TRUE);

	        //bq24157_set_ce(1);
    	}
	}

	return status;
}


static u32 charging_set_cv_voltage(void *data)
{
	u32 status = STATUS_OK;
	u16 register_value;
// not use,Just wait.
#if 1
    u32 cv_value = *(u32 *) (data);

    if (cv_value == BATTERY_VOLT_04_200000_V) {
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
    //highest of voltage will be 4.3V, because powerpath limitatior
    cv_value = 4304000;
#else
    cv_value = 4208000;
#endif
    }
#endif

	register_value = bq24157_charging_parameter_to_value(bq24157_VBAT_CV_VTH, GETARRAYNUM(bq24157_VBAT_CV_VTH), *(u32 *) (data));
	bq24157_set_oreg(register_value);

	return status;
}


static u32 charging_get_current(void *data)
{
	u32 status = STATUS_OK;
	u32 array_size;
	u8 reg_value;

	/* Get current level */
	array_size = GETARRAYNUM(bq24157_CS_VTH);
	bq24157_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
	*(u32 *) data = bq24157_charging_value_to_parameter(bq24157_CS_VTH, array_size, reg_value);

	return status;
}



static u32 charging_set_current(void *data)
{
	u32 status = STATUS_OK;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;
	u32 current_value = *(u32 *) data;

	//if (current_value <= CHARGE_CURRENT_350_00_MA) {
	if (current_value <= CHARGE_CURRENT_400_00_MA) {//Jason.
		bq24157_set_io_level(1);
	} else {
		bq24157_set_io_level(0);
	//modify by Jason for E1 volue of Rsense is 56mR
	/*	array_size = GETARRAYNUM(bq24157_CS_VTH);
		set_chr_current = bmt_find_closest_level(bq24157_CS_VTH, array_size, current_value);
		register_value = bq24157_charging_parameter_to_value(bq24157_CS_VTH, array_size, set_chr_current);
	*/
		array_size = GETARRAYNUM(bq24157_CS_VTH_E1);
		set_chr_current = bmt_find_closest_level(bq24157_CS_VTH_E1, array_size, current_value);
		register_value = bq24157_charging_parameter_to_value(bq24157_CS_VTH_E1, array_size, set_chr_current);

		bq24157_set_iocharge(register_value);
	}
	return status;
}


static u32 charging_set_input_current(void *data)
{
	u32 status = STATUS_OK;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;

	//if (*(u32 *) data > CHARGE_CURRENT_500_00_MA) {
	if (*(u32 *) data > CHARGE_CURRENT_800_00_MA) {
		register_value = 0x3;
	} else {
		array_size = GETARRAYNUM(bq24157_INPUT_CS_VTH);
		set_chr_current = bmt_find_closest_level(bq24157_INPUT_CS_VTH, array_size, *(u32 *) data);
		register_value =
		    bq24157_charging_parameter_to_value(bq24157_INPUT_CS_VTH, array_size, set_chr_current);
	}

	bq24157_set_input_charging_current(register_value);

	return status;
}


static u32 charging_get_charging_status(void *data)
{
	u32 status = STATUS_OK;
	u32 ret_val;

	ret_val = bq24157_get_chip_status();

	if (ret_val == 0x2)
		*(u32 *) data = KAL_TRUE;
	else
		*(u32 *) data = KAL_FALSE;

	return status;
}

 static unsigned int charging_get_enable(void *data)
 {
	 unsigned int status = STATUS_OK;
	 unsigned int ret_val;

	 ret_val = bq24157_get_chip_status();

	 if((ret_val == 0x1) || (ret_val == 0x2)) //0x2: charge done;	0x1: charge in progress
		 *(unsigned int *)data = KAL_TRUE;
	 else
		 *(unsigned int *)data = KAL_FALSE;

	 return status;
 }

static u32 charging_reset_watch_dog_timer(void *data)
{
	u32 status = STATUS_OK;

	bq24157_set_tmr_rst(1);

	return status;
}


static u32 charging_set_hv_threshold(void *data)
{
	u32 status = STATUS_OK;

	u32 set_hv_voltage;
	u32 array_size;
	u16 register_value;
	u32 voltage = *(u32 *) (data);

	array_size = GETARRAYNUM(bq24157_VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(bq24157_VCDT_HV_VTH, array_size, voltage);
	register_value = bq24157_charging_parameter_to_value(bq24157_VCDT_HV_VTH, array_size, set_hv_voltage);
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);// PMIC register setting, while not charger IC.
    pmic_set_register_value(PMIC_RG_VCDT_LV_VTH,0);//change VBUS threadhold to 4.2V
	return status;
}


static u32 charging_get_hv_status(void *data)
{
	u32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;
#else
	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#endif
	return status;
}


static u32 charging_get_battery_status(void *data)
{
	unsigned int status = STATUS_OK;

#if 1 //defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;	/* battery exist */
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] battery exist for bring up.\n");
#else
	unsigned int val = 0;

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

	return status;
}


static u32 charging_get_charger_det_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int val = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	val = 1;
	battery_log(BAT_LOG_CRTI, "[charging_get_charger_det_status] chr exist for fpga.\n");
#else
	val = pmic_get_register_value(PMIC_RGS_CHRDET);
#endif

	*(kal_bool *) (data) = val;

	return status;
}

static u32 charging_get_charger_type(void *data)
{
	u32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
#endif

	return status;
}

static u32 charging_get_is_pcm_timer_trigger(void *data)
{
	u32 status = STATUS_OK;
/* M migration
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;
	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
*/
	*(kal_bool *)(data) = KAL_FALSE;
	return status;
}

static u32 charging_set_platform_reset(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");

	kernel_restart("battery service reboot system");
	/* arch_reset(0,NULL); */
#endif

	return status;
}

static u32 charging_get_platform_boot_mode(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(unsigned int *) (data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif

	return status;
}

static u32 charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
	kernel_power_off();
#endif

	return status;
}

static u32 charging_get_power_source(void *data)
{
	u32 status = STATUS_UNSUPPORTED;

	return status;
}

static u32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static u32 charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;
}

static u32 charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

//add for DPM
static unsigned int charging_set_scv(void *data)
{
    unsigned int status = STATUS_OK;

    unsigned int set_sc_voltage;
    unsigned int array_size;
    unsigned short register_value;
    unsigned int voltage = *(unsigned int*)(data);

    array_size = GETARRAYNUM(bq24157_VBAT_SCV_VTH);
    set_sc_voltage = bmt_find_closest_level(bq24157_VBAT_SCV_VTH, array_size, voltage);
    register_value = bq24157_charging_parameter_to_value(bq24157_VBAT_SCV_VTH, array_size ,set_sc_voltage);
    bq24157_set_vsp(register_value);

    return status;
}

 static unsigned int charging_get_v_safe(void *data)
 {
     unsigned int status = STATUS_OK;

     *(unsigned int *)data = bq24157_get_v_safe();

     return status;
 }
static u32(*const charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
	 charging_hw_init
	,charging_dump_register
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platform_boot_mode
	,charging_set_power_off
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
    ,charging_set_scv
    ,charging_get_v_safe
    ,charging_get_enable
 };

s32 bq24157_chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	s32 status;

#if 0
	if (cmd < CHARGING_CMD_NUMBER)
		status = charging_func[cmd] (data);
	else
		return STATUS_UNSUPPORTED;
#endif
	if (cmd <= CHARGING_CMD_GET_CHARGING_ENABLE) {
		status = charging_func[cmd] (data);
	} else {
		battery_log(BAT_LOG_CRTI, "%s: hw bq24157 interface Error(%d)\n", __FUNCTION__, cmd);
		return STATUS_UNSUPPORTED;
	}

	return status;
}
