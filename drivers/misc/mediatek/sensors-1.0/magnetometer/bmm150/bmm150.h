/*!
 * @section LICENSE
 * (C) Copyright 2011~2016 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 */

/*
 * Definitions for bmm150 magnetic sensor chip.
 */
#ifndef __BMM150_H__
#define __BMM150_H__

#include <linux/ioctl.h>

#define BMM_DRIVER_VERSION "V1.3"

#define CALIBRATION_DATA_SIZE	12

/* 7-bit addr:
*	0x12 (SDO connected to GND)
*	0x13 (SDO connected to VDDIO)
*/
#define BMM150_I2C_ADDR		0x13

/* conversion of magnetic data (for bmm150) to uT units
conversion of magnetic data to uT units
32768 = 1Guass = 100 uT
100 / 32768 = 25 / 8096
65536 = 360Degree
360 / 65536 = 45 / 8192
*/

#define CONVERT_M			1
#define CONVERT_M_DIV		4
#define CONVERT_O			1
/*(C_PI_F32X * AXIS_RESOLUTION_FACTOR / 180)*/
#define CONVERT_O_DIV		71
#define CONVERT_G			1
#define CONVERT_G_DIV		938
#define CONVERT_VRV			1
#define CONVERT_VRV_DIV	16384
#define CONVERT_VLA_DIV	16384
#define CONVERT_VG_DIV 16384

#define BMM150_U16 unsigned short
#define BMM150_S16 signed short
#define BMM150_S32 signed int


#define BMM150_BUS_WR_RETURN_TYPE char
#define BMM150_BUS_WR_PARAM_TYPES\
	unsigned char, unsigned char, unsigned char *, unsigned char
#define BMM150_BUS_WR_PARAM_ORDER\
	(device_addr, register_addr, register_data, wr_len)
#define BMM150_BUS_WRITE_FUNC(\
		device_addr, register_addr, register_data, wr_len)\
	bus_write(device_addr, register_addr, register_data, wr_len)

#define BMM150_BUS_RD_RETURN_TYPE char

#define BMM150_BUS_RD_PARAM_TYPES\
	unsigned char, unsigned char, unsigned char *, unsigned char

#define BMM150_BUS_RD_PARAM_ORDER (device_addr, register_addr, register_data)

#define BMM150_BUS_READ_FUNC(device_addr, register_addr, register_data, rd_len)\
	bus_read(device_addr, register_addr, register_data, rd_len)


#define BMM150_DELAY_RETURN_TYPE void

#define BMM150_DELAY_PARAM_TYPES unsigned int

#define BMM150_DELAY_FUNC(delay_in_msec)\
	delay_func(delay_in_msec)

#define BMM150_DELAY_POWEROFF_SUSPEND      1
//zack modify
#define BMM150_DELAY_SUSPEND_SLEEP         5//2
//end zack
#define BMM150_DELAY_SLEEP_ACTIVE          1
#define BMM150_DELAY_ACTIVE_SLEEP          1
#define BMM150_DELAY_SLEEP_SUSPEND         1
#define BMM150_DELAY_ACTIVE_SUSPEND        1
#define BMM150_DELAY_SLEEP_POWEROFF        1
#define BMM150_DELAY_ACTIVE_POWEROFF       1
//zack modify
#define BMM150_DELAY_SETTLING_TIME         5//2
//end zack


#define BMM150_I2C_ADDRESS                 0x13

/*General Info datas*/
#define BMM150_SOFT_RESET7_ON              1
#define BMM150_SOFT_RESET1_ON              1
#define BMM150_SOFT_RESET7_OFF             0
#define BMM150_SOFT_RESET1_OFF             0
#define BMM150_DELAY_SOFTRESET             1

/* Fixed Data Registers */
#define BMM150_CHIP_ID                     0x40

/* Data Registers */
#define BMM150_DATAX_LSB                   0x42
#define BMM150_DATAX_MSB                   0x43
#define BMM150_DATAY_LSB                   0x44
#define BMM150_DATAY_MSB                   0x45
#define BMM150_DATAZ_LSB                   0x46
#define BMM150_DATAZ_MSB                   0x47
#define BMM150_R_LSB                       0x48
#define BMM150_R_MSB                       0x49

/* Status Registers */
#define BMM150_INT_STAT                    0x4A

/* Control Registers */
#define BMM150_POWER_CNTL                  0x4B
#define BMM150_CONTROL                     0x4C
#define BMM150_INT_CNTL                    0x4D
#define BMM150_SENS_CNTL                   0x4E
#define BMM150_LOW_THRES                   0x4F
#define BMM150_HIGH_THRES                  0x50
#define BMM150_NO_REPETITIONS_XY           0x51
#define BMM150_NO_REPETITIONS_Z            0x52

/* Trim Extended Registers */
#define BMM150_DIG_X1                      0x5D
#define BMM150_DIG_Y1                      0x5E
#define BMM150_DIG_Z4_LSB                  0x62
#define BMM150_DIG_Z4_MSB                  0x63
#define BMM150_DIG_X2                      0x64
#define BMM150_DIG_Y2                      0x65
#define BMM150_DIG_Z2_LSB                  0x68
#define BMM150_DIG_Z2_MSB                  0x69
#define BMM150_DIG_Z1_LSB                  0x6A
#define BMM150_DIG_Z1_MSB                  0x6B
#define BMM150_DIG_XYZ1_LSB                0x6C
#define BMM150_DIG_XYZ1_MSB                0x6D
#define BMM150_DIG_Z3_LSB                  0x6E
#define BMM150_DIG_Z3_MSB                  0x6F
#define BMM150_DIG_XY2                     0x70
#define BMM150_DIG_XY1                     0x71


/* Data X LSB Regsiter */
#define BMM150_DATAX_LSB_VALUEX__POS        3
#define BMM150_DATAX_LSB_VALUEX__LEN        5
#define BMM150_DATAX_LSB_VALUEX__MSK        0xF8
#define BMM150_DATAX_LSB_VALUEX__REG        BMM150_DATAX_LSB

#define BMM150_DATAX_LSB_TESTX__POS         0
#define BMM150_DATAX_LSB_TESTX__LEN         1
#define BMM150_DATAX_LSB_TESTX__MSK         0x01
#define BMM150_DATAX_LSB_TESTX__REG         BMM150_DATAX_LSB

/* Data Y LSB Regsiter */
#define BMM150_DATAY_LSB_VALUEY__POS        3
#define BMM150_DATAY_LSB_VALUEY__LEN        5
#define BMM150_DATAY_LSB_VALUEY__MSK        0xF8
#define BMM150_DATAY_LSB_VALUEY__REG        BMM150_DATAY_LSB

#define BMM150_DATAY_LSB_TESTY__POS         0
#define BMM150_DATAY_LSB_TESTY__LEN         1
#define BMM150_DATAY_LSB_TESTY__MSK         0x01
#define BMM150_DATAY_LSB_TESTY__REG         BMM150_DATAY_LSB

/* Data Z LSB Regsiter */
#define BMM150_DATAZ_LSB_VALUEZ__POS        1
#define BMM150_DATAZ_LSB_VALUEZ__LEN        7
#define BMM150_DATAZ_LSB_VALUEZ__MSK        0xFE
#define BMM150_DATAZ_LSB_VALUEZ__REG        BMM150_DATAZ_LSB

#define BMM150_DATAZ_LSB_TESTZ__POS         0
#define BMM150_DATAZ_LSB_TESTZ__LEN         1
#define BMM150_DATAZ_LSB_TESTZ__MSK         0x01
#define BMM150_DATAZ_LSB_TESTZ__REG         BMM150_DATAZ_LSB

/* Hall Resistance LSB Regsiter */
#define BMM150_R_LSB_VALUE__POS             2
#define BMM150_R_LSB_VALUE__LEN             6
#define BMM150_R_LSB_VALUE__MSK             0xFC
#define BMM150_R_LSB_VALUE__REG             BMM150_R_LSB

#define BMM150_DATA_RDYSTAT__POS            0
#define BMM150_DATA_RDYSTAT__LEN            1
#define BMM150_DATA_RDYSTAT__MSK            0x01
#define BMM150_DATA_RDYSTAT__REG            BMM150_R_LSB

/* Interupt Status Register */
#define BMM150_INT_STAT_DOR__POS            7
#define BMM150_INT_STAT_DOR__LEN            1
#define BMM150_INT_STAT_DOR__MSK            0x80
#define BMM150_INT_STAT_DOR__REG            BMM150_INT_STAT

#define BMM150_INT_STAT_OVRFLOW__POS        6
#define BMM150_INT_STAT_OVRFLOW__LEN        1
#define BMM150_INT_STAT_OVRFLOW__MSK        0x40
#define BMM150_INT_STAT_OVRFLOW__REG        BMM150_INT_STAT

#define BMM150_INT_STAT_HIGH_THZ__POS       5
#define BMM150_INT_STAT_HIGH_THZ__LEN       1
#define BMM150_INT_STAT_HIGH_THZ__MSK       0x20
#define BMM150_INT_STAT_HIGH_THZ__REG       BMM150_INT_STAT

#define BMM150_INT_STAT_HIGH_THY__POS       4
#define BMM150_INT_STAT_HIGH_THY__LEN       1
#define BMM150_INT_STAT_HIGH_THY__MSK       0x10
#define BMM150_INT_STAT_HIGH_THY__REG       BMM150_INT_STAT

#define BMM150_INT_STAT_HIGH_THX__POS       3
#define BMM150_INT_STAT_HIGH_THX__LEN       1
#define BMM150_INT_STAT_HIGH_THX__MSK       0x08
#define BMM150_INT_STAT_HIGH_THX__REG       BMM150_INT_STAT

#define BMM150_INT_STAT_LOW_THZ__POS        2
#define BMM150_INT_STAT_LOW_THZ__LEN        1
#define BMM150_INT_STAT_LOW_THZ__MSK        0x04
#define BMM150_INT_STAT_LOW_THZ__REG        BMM150_INT_STAT

#define BMM150_INT_STAT_LOW_THY__POS        1
#define BMM150_INT_STAT_LOW_THY__LEN        1
#define BMM150_INT_STAT_LOW_THY__MSK        0x02
#define BMM150_INT_STAT_LOW_THY__REG        BMM150_INT_STAT

#define BMM150_INT_STAT_LOW_THX__POS        0
#define BMM150_INT_STAT_LOW_THX__LEN        1
#define BMM150_INT_STAT_LOW_THX__MSK        0x01
#define BMM150_INT_STAT_LOW_THX__REG        BMM150_INT_STAT

/* Power Control Register */
#define BMM150_POWER_CNTL_SRST7__POS       7
#define BMM150_POWER_CNTL_SRST7__LEN       1
#define BMM150_POWER_CNTL_SRST7__MSK       0x80
#define BMM150_POWER_CNTL_SRST7__REG       BMM150_POWER_CNTL

#define BMM150_POWER_CNTL_SPI3_EN__POS     2
#define BMM150_POWER_CNTL_SPI3_EN__LEN     1
#define BMM150_POWER_CNTL_SPI3_EN__MSK     0x04
#define BMM150_POWER_CNTL_SPI3_EN__REG     BMM150_POWER_CNTL

#define BMM150_POWER_CNTL_SRST1__POS       1
#define BMM150_POWER_CNTL_SRST1__LEN       1
#define BMM150_POWER_CNTL_SRST1__MSK       0x02
#define BMM150_POWER_CNTL_SRST1__REG       BMM150_POWER_CNTL

#define BMM150_POWER_CNTL_PCB__POS         0
#define BMM150_POWER_CNTL_PCB__LEN         1
#define BMM150_POWER_CNTL_PCB__MSK         0x01
#define BMM150_POWER_CNTL_PCB__REG         BMM150_POWER_CNTL

/* Control Register */
#define BMM150_CNTL_ADV_ST__POS            6
#define BMM150_CNTL_ADV_ST__LEN            2
#define BMM150_CNTL_ADV_ST__MSK            0xC0
#define BMM150_CNTL_ADV_ST__REG            BMM150_CONTROL

#define BMM150_CNTL_DR__POS                3
#define BMM150_CNTL_DR__LEN                3
#define BMM150_CNTL_DR__MSK                0x38
#define BMM150_CNTL_DR__REG                BMM150_CONTROL

#define BMM150_CNTL_OPMODE__POS            1
#define BMM150_CNTL_OPMODE__LEN            2
#define BMM150_CNTL_OPMODE__MSK            0x06
#define BMM150_CNTL_OPMODE__REG            BMM150_CONTROL

#define BMM150_CNTL_S_TEST__POS            0
#define BMM150_CNTL_S_TEST__LEN            1
#define BMM150_CNTL_S_TEST__MSK            0x01
#define BMM150_CNTL_S_TEST__REG            BMM150_CONTROL

/* Interupt Control Register */
#define BMM150_INT_CNTL_DOR_EN__POS            7
#define BMM150_INT_CNTL_DOR_EN__LEN            1
#define BMM150_INT_CNTL_DOR_EN__MSK            0x80
#define BMM150_INT_CNTL_DOR_EN__REG            BMM150_INT_CNTL

#define BMM150_INT_CNTL_OVRFLOW_EN__POS        6
#define BMM150_INT_CNTL_OVRFLOW_EN__LEN        1
#define BMM150_INT_CNTL_OVRFLOW_EN__MSK        0x40
#define BMM150_INT_CNTL_OVRFLOW_EN__REG        BMM150_INT_CNTL

#define BMM150_INT_CNTL_HIGH_THZ_EN__POS       5
#define BMM150_INT_CNTL_HIGH_THZ_EN__LEN       1
#define BMM150_INT_CNTL_HIGH_THZ_EN__MSK       0x20
#define BMM150_INT_CNTL_HIGH_THZ_EN__REG       BMM150_INT_CNTL

#define BMM150_INT_CNTL_HIGH_THY_EN__POS       4
#define BMM150_INT_CNTL_HIGH_THY_EN__LEN       1
#define BMM150_INT_CNTL_HIGH_THY_EN__MSK       0x10
#define BMM150_INT_CNTL_HIGH_THY_EN__REG       BMM150_INT_CNTL

#define BMM150_INT_CNTL_HIGH_THX_EN__POS       3
#define BMM150_INT_CNTL_HIGH_THX_EN__LEN       1
#define BMM150_INT_CNTL_HIGH_THX_EN__MSK       0x08
#define BMM150_INT_CNTL_HIGH_THX_EN__REG       BMM150_INT_CNTL

#define BMM150_INT_CNTL_LOW_THZ_EN__POS        2
#define BMM150_INT_CNTL_LOW_THZ_EN__LEN        1
#define BMM150_INT_CNTL_LOW_THZ_EN__MSK        0x04
#define BMM150_INT_CNTL_LOW_THZ_EN__REG        BMM150_INT_CNTL

#define BMM150_INT_CNTL_LOW_THY_EN__POS        1
#define BMM150_INT_CNTL_LOW_THY_EN__LEN        1
#define BMM150_INT_CNTL_LOW_THY_EN__MSK        0x02
#define BMM150_INT_CNTL_LOW_THY_EN__REG        BMM150_INT_CNTL

#define BMM150_INT_CNTL_LOW_THX_EN__POS        0
#define BMM150_INT_CNTL_LOW_THX_EN__LEN        1
#define BMM150_INT_CNTL_LOW_THX_EN__MSK        0x01
#define BMM150_INT_CNTL_LOW_THX_EN__REG        BMM150_INT_CNTL

/* Sensor Control Register */
#define BMM150_SENS_CNTL_DRDY_EN__POS          7
#define BMM150_SENS_CNTL_DRDY_EN__LEN          1
#define BMM150_SENS_CNTL_DRDY_EN__MSK          0x80
#define BMM150_SENS_CNTL_DRDY_EN__REG          BMM150_SENS_CNTL

#define BMM150_SENS_CNTL_IE__POS               6
#define BMM150_SENS_CNTL_IE__LEN               1
#define BMM150_SENS_CNTL_IE__MSK               0x40
#define BMM150_SENS_CNTL_IE__REG               BMM150_SENS_CNTL

#define BMM150_SENS_CNTL_CHANNELZ__POS         5
#define BMM150_SENS_CNTL_CHANNELZ__LEN         1
#define BMM150_SENS_CNTL_CHANNELZ__MSK         0x20
#define BMM150_SENS_CNTL_CHANNELZ__REG         BMM150_SENS_CNTL

#define BMM150_SENS_CNTL_CHANNELY__POS         4
#define BMM150_SENS_CNTL_CHANNELY__LEN         1
#define BMM150_SENS_CNTL_CHANNELY__MSK         0x10
#define BMM150_SENS_CNTL_CHANNELY__REG         BMM150_SENS_CNTL

#define BMM150_SENS_CNTL_CHANNELX__POS         3
#define BMM150_SENS_CNTL_CHANNELX__LEN         1
#define BMM150_SENS_CNTL_CHANNELX__MSK         0x08
#define BMM150_SENS_CNTL_CHANNELX__REG         BMM150_SENS_CNTL

#define BMM150_SENS_CNTL_DR_POLARITY__POS      2
#define BMM150_SENS_CNTL_DR_POLARITY__LEN      1
#define BMM150_SENS_CNTL_DR_POLARITY__MSK      0x04
#define BMM150_SENS_CNTL_DR_POLARITY__REG      BMM150_SENS_CNTL

#define BMM150_SENS_CNTL_INTERRUPT_LATCH__POS            1
#define BMM150_SENS_CNTL_INTERRUPT_LATCH__LEN            1
#define BMM150_SENS_CNTL_INTERRUPT_LATCH__MSK            0x02
#define BMM150_SENS_CNTL_INTERRUPT_LATCH__REG            BMM150_SENS_CNTL

#define BMM150_SENS_CNTL_INTERRUPT_POLARITY__POS         0
#define BMM150_SENS_CNTL_INTERRUPT_POLARITY__LEN         1
#define BMM150_SENS_CNTL_INTERRUPT_POLARITY__MSK         0x01
#define BMM150_SENS_CNTL_INTERRUPT_POLARITY__REG         BMM150_SENS_CNTL

/* Register 6D */
#define BMM150_DIG_XYZ1_MSB__POS         0
#define BMM150_DIG_XYZ1_MSB__LEN         7
#define BMM150_DIG_XYZ1_MSB__MSK         0x7F
#define BMM150_DIG_XYZ1_MSB__REG         BMM150_DIG_XYZ1_MSB


#define BMM150_X_AXIS               0
#define BMM150_Y_AXIS               1
#define BMM150_Z_AXIS               2
#define BMM150_RESISTANCE           3
#define BMM150_X                    1
#define BMM150_Y                    2
#define BMM150_Z                    4
#define BMM150_XYZ                  7

/* Constants */
#define BMM150_NULL                             0
#define BMM150_DISABLE                          0
#define BMM150_ENABLE                           1
#define BMM150_CHANNEL_DISABLE                  1
#define BMM150_CHANNEL_ENABLE                   0
#define BMM150_INTPIN_LATCH_ENABLE              1
#define BMM150_INTPIN_LATCH_DISABLE             0
#define BMM150_OFF                              0
#define BMM150_ON                               1

#define BMM150_NORMAL_MODE                      0x00
#define BMM150_FORCED_MODE                      0x01
#define BMM150_SUSPEND_MODE                     0x02
#define BMM150_SLEEP_MODE                       0x03

#define BMM150_ADVANCED_SELFTEST_OFF            0
#define BMM150_ADVANCED_SELFTEST_NEGATIVE       2
#define BMM150_ADVANCED_SELFTEST_POSITIVE       3

#define BMM150_NEGATIVE_SATURATION_Z            -32767
#define BMM150_POSITIVE_SATURATION_Z            32767

#define BMM150_SPI_RD_MASK                      0x80
#define BMM150_READ_SET                         0x01

#define E_BMM150_NULL_PTR                       ((char)-127)
#define E_BMM150_COMM_RES                       ((char)-1)
#define E_BMM150_OUT_OF_RANGE                   ((char)-2)
#define E_BMM150_UNDEFINED_MODE                 0

#define BMM150_WR_FUNC_PTR\
	char (*bus_write)(unsigned char, unsigned char,\
			unsigned char *, unsigned char)

#define BMM150_RD_FUNC_PTR\
	char (*bus_read)(unsigned char, unsigned char,\
			unsigned char *, unsigned char)
#define BMM150_MDELAY_DATA_TYPE unsigned int
#define BMM150_RETURN_FUNCTION_TYPE        char
/*Shifting Constants*/
#define SHIFT_RIGHT_1_POSITION                  1
#define SHIFT_RIGHT_2_POSITION                  2
#define SHIFT_RIGHT_3_POSITION                  3
#define SHIFT_RIGHT_4_POSITION                  4
#define SHIFT_RIGHT_5_POSITION                  5
#define SHIFT_RIGHT_6_POSITION                  6
#define SHIFT_RIGHT_7_POSITION                  7
#define SHIFT_RIGHT_8_POSITION                  8

#define SHIFT_LEFT_1_POSITION                   1
#define SHIFT_LEFT_2_POSITION                   2
#define SHIFT_LEFT_3_POSITION                   3
#define SHIFT_LEFT_4_POSITION                   4
#define SHIFT_LEFT_5_POSITION                   5
#define SHIFT_LEFT_6_POSITION                   6
#define SHIFT_LEFT_7_POSITION                   7
#define SHIFT_LEFT_8_POSITION                   8

/* Conversion factors*/
#define BMM150_CONVFACTOR_LSB_UT                6

/* get bit slice  */
#define BMM150_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

/* Set bit slice */
#define BMM150_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/* compensated output value returned if sensor had overflow */
#define BMM150_OVERFLOW_OUTPUT       -32768
#define BMM150_OVERFLOW_OUTPUT_S32   ((BMM150_S32)(-2147483647-1))
#define BMM150_OVERFLOW_OUTPUT_FLOAT  (0.0f)
#define BMM150_FLIP_OVERFLOW_ADCVAL  -4096
#define BMM150_HALL_OVERFLOW_ADCVAL  -16384


#define BMM150_PRESETMODE_LOWPOWER                  1
#define BMM150_PRESETMODE_REGULAR                   2
#define BMM150_PRESETMODE_HIGHACCURACY              3
#define BMM150_PRESETMODE_ENHANCED                  4

/* PRESET MODES - DATA RATES */
#define BMM150_LOWPOWER_DR                       BMM150_DR_25HZ
#define BMM150_REGULAR_DR                        BMM150_DR_25HZ
#define BMM150_HIGHACCURACY_DR                   BMM150_DR_25HZ
#define BMM150_ENHANCED_DR                       BMM150_DR_25HZ

/* PRESET MODES - REPETITIONS-XY RATES */
#define BMM150_LOWPOWER_REPXY                     1
#define BMM150_REGULAR_REPXY                      4
#define BMM150_HIGHACCURACY_REPXY                23
#define BMM150_ENHANCED_REPXY                     7

/* PRESET MODES - REPETITIONS-Z RATES */
#define BMM150_LOWPOWER_REPZ                      2
#define BMM150_REGULAR_REPZ                      15
#define BMM150_HIGHACCURACY_REPZ                 82
#define BMM150_ENHANCED_REPZ                     26

#define BMM_ERROR_BUF                        1
#define BMM_ERROR_PTR                        2
/* Data Rates */

#define BMM150_DR_10HZ                     0
#define BMM150_DR_02HZ                     1
#define BMM150_DR_06HZ                     2
#define BMM150_DR_08HZ                     3
#define BMM150_DR_15HZ                     4
#define BMM150_DR_20HZ                     5
#define BMM150_DR_25HZ                     6
#define BMM150_DR_30HZ                     7

/*user defined Structures*/
struct bmm150api_mdata {
	BMM150_S16 datax;
	BMM150_S16 datay;
	BMM150_S16 dataz;
	BMM150_U16 resistance;
};
struct bmm150api_mdata_s32 {
	BMM150_S32 datax;
	BMM150_S32 datay;
	BMM150_S32 dataz;
	BMM150_U16 resistance;
};
struct bmm150api_mdata_float {
	float datax;
	float datay;
	float  dataz;
	BMM150_U16 resistance;
};

struct bmm150api {
	unsigned char company_id;
	unsigned char dev_addr;

	BMM150_WR_FUNC_PTR;
	BMM150_RD_FUNC_PTR;
	void(*delay_msec)(BMM150_MDELAY_DATA_TYPE);

	signed char dig_x1;
	signed char dig_y1;

	signed char dig_x2;
	signed char dig_y2;

	BMM150_U16 dig_z1;
	BMM150_S16 dig_z2;
	BMM150_S16 dig_z3;
	BMM150_S16 dig_z4;

	unsigned char dig_xy1;
	signed char dig_xy2;

	BMM150_U16 dig_xyz1;
};


BMM150_RETURN_FUNCTION_TYPE bmm150api_init(struct bmm150api *p_bmm150);
BMM150_RETURN_FUNCTION_TYPE bmm150api_read_mdataXYZ(
	struct bmm150api_mdata *mdata);
BMM150_RETURN_FUNCTION_TYPE bmm150api_read_mdataXYZ_s32(
	struct bmm150api_mdata_s32 *mdata);
#ifdef ENABLE_FLOAT
BMM150_RETURN_FUNCTION_TYPE bmm150api_read_mdataXYZ_float(
	struct bmm150api_mdata_float *mdata);
#endif
BMM150_RETURN_FUNCTION_TYPE bmm150api_read_register(
	unsigned char addr, unsigned char *data, unsigned char len);
BMM150_RETURN_FUNCTION_TYPE bmm150api_write_register(
	unsigned char addr, unsigned char *data, unsigned char len);
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_self_test_XYZ(
	unsigned char *self_testxyz);
BMM150_S16 bmm150api_compensate_X(
	BMM150_S16 mdata_x, BMM150_U16 data_R);
BMM150_S32 bmm150api_compensate_X_s32(
	BMM150_S16 mdata_x, BMM150_U16 data_R);
#ifdef ENABLE_FLOAT
float bmm150api_compensate_X_float(
	BMM150_S16 mdata_x, BMM150_U16 data_R);
#endif
BMM150_S16 bmm150api_compensate_Y(
	BMM150_S16 mdata_y, BMM150_U16 data_R);
BMM150_S32 bmm150api_compensate_Y_s32(
	BMM150_S16 mdata_y, BMM150_U16 data_R);
#ifdef ENABLE_FLOAT
float bmm150api_compensate_Y_float(
	BMM150_S16 mdata_y, BMM150_U16 data_R);
#endif
BMM150_S16 bmm150api_compensate_Z(
	BMM150_S16 mdata_z, BMM150_U16 data_R);
BMM150_S32 bmm150api_compensate_Z_s32(
	BMM150_S16 mdata_z, BMM150_U16 data_R);
#ifdef ENABLE_FLOAT
float bmm150api_compensate_Z_float(
	BMM150_S16 mdata_z, BMM150_U16 data_R);
#endif
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_raw_xyz(
	struct bmm150api_mdata *mdata);
BMM150_RETURN_FUNCTION_TYPE bmm150api_init_trim_registers(void);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_spi3(
	unsigned char value);
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_powermode(
	unsigned char *mode);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_powermode(
	unsigned char mode);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_adv_selftest(
	unsigned char adv_selftest);
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_adv_selftest(
	unsigned char *adv_selftest);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_datarate(
	unsigned char data_rate);
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_datarate(
	unsigned char *data_rate);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_functional_state(
	unsigned char functional_state);
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_functional_state(
	unsigned char *functional_state);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_selftest(
	unsigned char selftest);
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_selftest(
	unsigned char *selftest);
BMM150_RETURN_FUNCTION_TYPE bmm150api_perform_advanced_selftest(
	BMM150_S16 *diff_z);
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_repetitions_XY(
	unsigned char *no_repetitions_xy);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_repetitions_XY(
	unsigned char no_repetitions_xy);
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_repetitions_Z(
	unsigned char *no_repetitions_z);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_repetitions_Z(
	unsigned char no_repetitions_z);
BMM150_RETURN_FUNCTION_TYPE bmm150api_get_presetmode(unsigned char *mode);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_presetmode(unsigned char mode);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_control_measurement_x(
	unsigned char enable_disable);
BMM150_RETURN_FUNCTION_TYPE bmm150api_set_control_measurement_y(
	unsigned char enable_disable);
BMM150_RETURN_FUNCTION_TYPE bmm150api_soft_reset(void);
#endif /* __BMM150_H__ */

