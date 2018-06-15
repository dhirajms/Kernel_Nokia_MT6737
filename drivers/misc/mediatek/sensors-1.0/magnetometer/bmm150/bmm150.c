/*!
 * @section LICENSE
 * (C) Copyright 2011~2016 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * VERSION: V1.2
 * History:	V1.0 --- Driver creation
 *          V1.1 --- Add share I2C address solution
 *          V1.2 --- Fix bug that daemon can't get
 *                   delay command.
 *          V1.3 --- [2017.03.08]Add I2C retry in initializition
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/module.h>

#include <mag.h>

#include <cust_mag.h>
#include "bmm150.h"

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*   BMM150 API Section	*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
#define DEBUG 1
#define BMM150_DEV_NAME         "bmm150"
/*----------------------------------------------------------------------------*/

#define SENSOR_CHIP_ID_BMM	(0x32)

#define BMM150_DEFAULT_DELAY	100
#define BMM150_BUFSIZE  0x20

#define MSE_TAG	"[Msensor] "
#define MSE_FUN(f)	printk(KERN_INFO MSE_TAG"%s\n", __func__)
#define MSE_ERR(fmt, args...)	printk(KERN_ERR MSE_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)	printk(KERN_INFO MSE_TAG fmt, ##args)


static struct bmm150api *p_bmm150;

BMM150_RETURN_FUNCTION_TYPE bmm150api_init(struct bmm150api *bmm150)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[2];
	p_bmm150 = bmm150;

	p_bmm150->dev_addr = BMM150_I2C_ADDRESS;

	/* set device from suspend into sleep mode */
	bmm150api_set_powermode(BMM150_ON);

	/* wait two millisecond for bmc to settle */
	p_bmm150->delay_msec(BMM150_DELAY_SETTLING_TIME);

	/*Read CHIP_ID and REv. info */
	comres = p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_CHIP_ID, a_data_u8r, 1);
	p_bmm150->company_id = a_data_u8r[0];

	/* Function to initialise trim values */
	bmm150api_init_trim_registers();
	bmm150api_set_presetmode(BMM150_PRESETMODE_REGULAR);
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_presetmode(unsigned char mode)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	switch (mode) {
	case BMM150_PRESETMODE_LOWPOWER:
		/* Set the data rate for Low Power mode */
		comres = bmm150api_set_datarate(BMM150_LOWPOWER_DR);
		/* Set the XY-repetitions number for Low Power mode */
		comres |= bmm150api_set_repetitions_XY(BMM150_LOWPOWER_REPXY);
		/* Set the Z-repetitions number  for Low Power mode */
		comres |= bmm150api_set_repetitions_Z(BMM150_LOWPOWER_REPZ);
		break;
	case BMM150_PRESETMODE_REGULAR:
		/* Set the data rate for Regular mode */
		comres = bmm150api_set_datarate(BMM150_REGULAR_DR);
		/* Set the XY-repetitions number for Regular mode */
		comres |= bmm150api_set_repetitions_XY(BMM150_REGULAR_REPXY);
		/* Set the Z-repetitions number  for Regular mode */
		comres |= bmm150api_set_repetitions_Z(BMM150_REGULAR_REPZ);
		break;
	case BMM150_PRESETMODE_HIGHACCURACY:
		/* Set the data rate for High Accuracy mode */
		comres = bmm150api_set_datarate(BMM150_HIGHACCURACY_DR);
		/* Set the XY-repetitions number for High Accuracy mode */
		comres |=
		 bmm150api_set_repetitions_XY(BMM150_HIGHACCURACY_REPXY);
		/* Set the Z-repetitions number  for High Accuracyr mode */
		comres |= bmm150api_set_repetitions_Z(BMM150_HIGHACCURACY_REPZ);
		break;
	case BMM150_PRESETMODE_ENHANCED:
		/* Set the data rate for Enhanced Accuracy mode */
		comres = bmm150api_set_datarate(BMM150_ENHANCED_DR);
		/* Set the XY-repetitions number for High Enhanced mode */
		comres |= bmm150api_set_repetitions_XY(BMM150_ENHANCED_REPXY);
		/* Set the Z-repetitions number  for High Enhanced mode */
		comres |= bmm150api_set_repetitions_Z(BMM150_ENHANCED_REPZ);
		break;
	default:
		comres = E_BMM150_OUT_OF_RANGE;
		break;
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_functional_state(
		unsigned char functional_state)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		switch (functional_state) {
		case BMM150_NORMAL_MODE:
			comres = bmm150api_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMM150_OFF) {
				comres |= bmm150api_set_powermode(BMM150_ON);
				p_bmm150->delay_msec(
						BMM150_DELAY_SUSPEND_SLEEP);
			}
			{
				comres |= p_bmm150->BMM150_BUS_READ_FUNC(
						p_bmm150->dev_addr,
						BMM150_CNTL_OPMODE__REG,
						&v_data1_u8r, 1);
				v_data1_u8r = BMM150_SET_BITSLICE(
						v_data1_u8r,
						BMM150_CNTL_OPMODE,
						BMM150_NORMAL_MODE);
				comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
						p_bmm150->dev_addr,
						BMM150_CNTL_OPMODE__REG,
						&v_data1_u8r, 1);
			}
			break;
		case BMM150_SUSPEND_MODE:
			comres = bmm150api_set_powermode(BMM150_OFF);
			break;
		case BMM150_FORCED_MODE:
			comres = bmm150api_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMM150_OFF) {
				comres = bmm150api_set_powermode(BMM150_ON);
				p_bmm150->delay_msec(
						BMM150_DELAY_SUSPEND_SLEEP);
			}
			comres |= p_bmm150->BMM150_BUS_READ_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM150_SET_BITSLICE(
					v_data1_u8r,
					BMM150_CNTL_OPMODE, BMM150_ON);
			comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			break;
		case BMM150_SLEEP_MODE:
			bmm150api_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMM150_OFF) {
				comres = bmm150api_set_powermode(BMM150_ON);
				p_bmm150->delay_msec(
						BMM150_DELAY_SUSPEND_SLEEP);
			}
			comres |= p_bmm150->BMM150_BUS_READ_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM150_SET_BITSLICE(
					v_data1_u8r,
					BMM150_CNTL_OPMODE,
					BMM150_SLEEP_MODE);
			comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			break;
		default:
			comres = E_BMM150_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_get_functional_state(
		unsigned char *functional_state)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_CNTL_OPMODE__REG,
				&v_data_u8r, 1);
		*functional_state = BMM150_GET_BITSLICE(
				v_data_u8r, BMM150_CNTL_OPMODE);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_read_mdataXYZ(
	struct bmm150api_mdata *mdata)
{
	BMM150_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM150_S16 raw_dataX;
		BMM150_S16 raw_dataY;
		BMM150_S16 raw_dataZ;
		BMM150_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
				BMM150_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM150_GET_BITSLICE(a_data_u8r[0],
				BMM150_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[1])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM150_GET_BITSLICE(a_data_u8r[2],
				BMM150_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[3])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM150_GET_BITSLICE(a_data_u8r[4],
				BMM150_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[5])) <<
					SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM150_GET_BITSLICE(a_data_u8r[6],
				BMM150_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM150_U16)((((BMM150_U16)
						a_data_u8r[7]) <<
					SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm150api_compensate_X(raw_dataXYZ.raw_dataX,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm150api_compensate_Y(raw_dataXYZ.raw_dataY,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm150api_compensate_Z(raw_dataXYZ.raw_dataZ,
				raw_dataXYZ.raw_dataR);

	    /* Output raw resistance value */
	    mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_read_mdataXYZ_s32(
	struct bmm150api_mdata_s32 *mdata)
{
	BMM150_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM150_S16 raw_dataX;
		BMM150_S16 raw_dataY;
		BMM150_S16 raw_dataZ;
		BMM150_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
				BMM150_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM150_GET_BITSLICE(a_data_u8r[0],
				BMM150_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[1])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM150_GET_BITSLICE(a_data_u8r[2],
				BMM150_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[3])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM150_GET_BITSLICE(a_data_u8r[4],
				BMM150_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[5])) <<
					SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM150_GET_BITSLICE(a_data_u8r[6],
				BMM150_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM150_U16)((((BMM150_U16)
						a_data_u8r[7]) <<
					SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm150api_compensate_X_s32(raw_dataXYZ.raw_dataX,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm150api_compensate_Y_s32(raw_dataXYZ.raw_dataY,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm150api_compensate_Z_s32(raw_dataXYZ.raw_dataZ,
				raw_dataXYZ.raw_dataR);

	    /* Output raw resistance value */
	    mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}

#ifdef ENABLE_FLOAT
BMM150_RETURN_FUNCTION_TYPE bmm150api_read_mdataXYZ_float(
	struct bmm150api_mdata_float *mdata)
{
	BMM150_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM150_S16 raw_dataX;
		BMM150_S16 raw_dataY;
		BMM150_S16 raw_dataZ;
		BMM150_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
				BMM150_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM150_GET_BITSLICE(a_data_u8r[0],
				BMM150_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[1])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM150_GET_BITSLICE(a_data_u8r[2],
				BMM150_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[3])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM150_GET_BITSLICE(a_data_u8r[4],
				BMM150_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[5])) <<
					SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM150_GET_BITSLICE(a_data_u8r[6],
				BMM150_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM150_U16)((((BMM150_U16)
						a_data_u8r[7]) <<
					SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm150api_compensate_X_float(
		 raw_dataXYZ.raw_dataX,
		 raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm150api_compensate_Y_float(
		 raw_dataXYZ.raw_dataY,
		 raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm150api_compensate_Z_float(
		 raw_dataXYZ.raw_dataZ,
		 raw_dataXYZ.raw_dataR);

	    /* Output raw resistance value */
	    mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}
#endif

BMM150_RETURN_FUNCTION_TYPE bmm150api_read_register(unsigned char addr,
		unsigned char *data, unsigned char len)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres += p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			addr, data, len);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_write_register(unsigned char addr,
	    unsigned char *data, unsigned char len)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_WRITE_FUNC(p_bmm150->dev_addr,
			addr, data, len);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_selftest(unsigned char selftest)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr, BMM150_CNTL_S_TEST__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMM150_SET_BITSLICE(
				v_data1_u8r, BMM150_CNTL_S_TEST, selftest);
		comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr, BMM150_CNTL_S_TEST__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_get_self_test_XYZ(
		unsigned char *self_testxyz)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[5], v_result_u8r = 0x00;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr, BMM150_DATAX_LSB_TESTX__REG,
				a_data_u8r, 5);

		v_result_u8r = BMM150_GET_BITSLICE(a_data_u8r[4],
				BMM150_DATAZ_LSB_TESTZ);

		v_result_u8r = (v_result_u8r << 1);
		v_result_u8r = (v_result_u8r | BMM150_GET_BITSLICE(
					a_data_u8r[2], BMM150_DATAY_LSB_TESTY));

		v_result_u8r = (v_result_u8r << 1);
		v_result_u8r = (v_result_u8r | BMM150_GET_BITSLICE(
					a_data_u8r[0], BMM150_DATAX_LSB_TESTX));

		*self_testxyz = v_result_u8r;
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_spi3(unsigned char value)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_POWER_CNTL_SPI3_EN__REG, &v_data1_u8r, 1);
		v_data1_u8r = BMM150_SET_BITSLICE(v_data1_u8r,
			BMM150_POWER_CNTL_SPI3_EN, value);
		comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(p_bmm150->dev_addr,
		    BMM150_POWER_CNTL_SPI3_EN__REG, &v_data1_u8r, 1);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_datarate(unsigned char data_rate)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_CNTL_DR__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMM150_SET_BITSLICE(v_data1_u8r,
				BMM150_CNTL_DR, data_rate);
		comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_CNTL_DR__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_get_datarate(unsigned char *data_rate)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_CNTL_DR__REG,
				&v_data_u8r, 1);
		*data_rate = BMM150_GET_BITSLICE(v_data_u8r,
				BMM150_CNTL_DR);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_perform_advanced_selftest(
		BMM150_S16 *diff_z)
{
	BMM150_RETURN_FUNCTION_TYPE comres;
	BMM150_S16 result_positive, result_negative;
	struct bmm150api_mdata_s32 mdata;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		/* set sleep mode to prepare for forced measurement.
		 * If sensor is off, this will turn it on
		 * and respect needed delays. */
		comres = bmm150api_set_functional_state(BMM150_SLEEP_MODE);

		/* set normal accuracy mode */
		comres |= bmm150api_set_repetitions_Z(BMM150_LOWPOWER_REPZ);
		/* 14 repetitions Z in normal accuracy mode */

		/* disable X, Y channel */
		comres |= bmm150api_set_control_measurement_x(
				BMM150_CHANNEL_DISABLE);
		comres |= bmm150api_set_control_measurement_y(
				BMM150_CHANNEL_DISABLE);

		/* enable positive current and force a
		 * measurement with positive field */
		comres |= bmm150api_set_adv_selftest(
				BMM150_ADVANCED_SELFTEST_POSITIVE);
		comres |= bmm150api_set_functional_state(BMM150_FORCED_MODE);
		/* wait for measurement to complete */
		p_bmm150->delay_msec(4);

		/* read result from positive field measurement */
		comres |= bmm150api_read_mdataXYZ_s32(&mdata);
		result_positive = mdata.dataz;

		/* enable negative current and force a
		 * measurement with negative field */
		comres |= bmm150api_set_adv_selftest(
				BMM150_ADVANCED_SELFTEST_NEGATIVE);
		comres |= bmm150api_set_functional_state(BMM150_FORCED_MODE);
		p_bmm150->delay_msec(4); /* wait for measurement to complete */

		/* read result from negative field measurement */
		comres |= bmm150api_read_mdataXYZ_s32(&mdata);
		result_negative = mdata.dataz;

		/* turn off self test current */
		comres |= bmm150api_set_adv_selftest(
				BMM150_ADVANCED_SELFTEST_OFF);

		/* enable X, Y channel */
		comres |= bmm150api_set_control_measurement_x(
				BMM150_CHANNEL_ENABLE);
		comres |= bmm150api_set_control_measurement_y(
				BMM150_CHANNEL_ENABLE);

		/* write out difference in positive and negative field.
		 * This should be ~ 200 mT = 3200 LSB */
		*diff_z = (result_positive - result_negative);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_init_trim_registers(void)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[2];
	comres = p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_X1, (unsigned char *)&p_bmm150->dig_x1, 1);
	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_Y1, (unsigned char *)&p_bmm150->dig_y1, 1);
	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_X2, (unsigned char *)&p_bmm150->dig_x2, 1);
	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_Y2, (unsigned char *)&p_bmm150->dig_y2, 1);
	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_XY1, (unsigned char *)&p_bmm150->dig_xy1, 1);
	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_XY2, (unsigned char *)&p_bmm150->dig_xy2, 1);

	/* shorts can not be recasted into (unsigned char*)
	 * due to possible mixup between trim data
	 * arrangement and memory arrangement */

	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_Z1_LSB, a_data_u8r, 2);
	p_bmm150->dig_z1 = (BMM150_U16)((((BMM150_U16)((unsigned char)
						a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_Z2_LSB, a_data_u8r, 2);
	p_bmm150->dig_z2 = (BMM150_S16)((((BMM150_S16)(
						(signed char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_Z3_LSB, a_data_u8r, 2);
	p_bmm150->dig_z3 = (BMM150_S16)((((BMM150_S16)(
						(signed char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_Z4_LSB, a_data_u8r, 2);
	p_bmm150->dig_z4 = (BMM150_S16)((((BMM150_S16)(
						(signed char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_DIG_XYZ1_LSB, a_data_u8r, 2);
	a_data_u8r[1] = BMM150_GET_BITSLICE(a_data_u8r[1], BMM150_DIG_XYZ1_MSB);
	p_bmm150->dig_xyz1 = (BMM150_U16)((((BMM150_U16)
					((unsigned char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_adv_selftest(
	unsigned char adv_selftest)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		switch (adv_selftest) {
		case BMM150_ADVANCED_SELFTEST_OFF:
			comres = p_bmm150->BMM150_BUS_READ_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM150_SET_BITSLICE(
					v_data1_u8r,
					BMM150_CNTL_ADV_ST,
					BMM150_ADVANCED_SELFTEST_OFF);
			comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			break;
		case BMM150_ADVANCED_SELFTEST_POSITIVE:
			comres = p_bmm150->BMM150_BUS_READ_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM150_SET_BITSLICE(
					v_data1_u8r,
					BMM150_CNTL_ADV_ST,
					BMM150_ADVANCED_SELFTEST_POSITIVE);
			comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			break;
		case BMM150_ADVANCED_SELFTEST_NEGATIVE:
			comres = p_bmm150->BMM150_BUS_READ_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM150_SET_BITSLICE(
					v_data1_u8r,
					BMM150_CNTL_ADV_ST,
					BMM150_ADVANCED_SELFTEST_NEGATIVE);
			comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
					p_bmm150->dev_addr,
					BMM150_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			break;
		default:
			break;
		}
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_get_adv_selftest(
	unsigned char *adv_selftest)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
			BMM150_CNTL_ADV_ST__REG, &v_data_u8r, 1);
		*adv_selftest = BMM150_GET_BITSLICE(v_data_u8r,
			BMM150_CNTL_ADV_ST);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_get_presetmode(
	unsigned char *mode)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char data_rate = 0;
	unsigned char repetitionsxy = 0;
	unsigned char repetitionsz = 0;

	/* Get the current data rate */
	comres = bmm150api_get_datarate(&data_rate);
	/* Get the preset number of XY Repetitions */
	comres |= bmm150api_get_repetitions_XY(&repetitionsxy);
	/* Get the preset number of Z Repetitions */
	comres |= bmm150api_get_repetitions_Z(&repetitionsz);
	if ((data_rate == BMM150_LOWPOWER_DR) && (
		repetitionsxy == BMM150_LOWPOWER_REPXY) && (
		repetitionsz == BMM150_LOWPOWER_REPZ)) {
		*mode = BMM150_PRESETMODE_LOWPOWER;
	} else {
		if ((data_rate == BMM150_REGULAR_DR) && (
			repetitionsxy == BMM150_REGULAR_REPXY) && (
			repetitionsz == BMM150_REGULAR_REPZ)) {
			*mode = BMM150_PRESETMODE_REGULAR;
		} else {
			if ((data_rate == BMM150_HIGHACCURACY_DR) && (
				repetitionsxy == BMM150_HIGHACCURACY_REPXY) && (
				repetitionsz == BMM150_HIGHACCURACY_REPZ)) {
					*mode = BMM150_PRESETMODE_HIGHACCURACY;
			} else {
				if ((data_rate == BMM150_ENHANCED_DR) && (
				repetitionsxy == BMM150_ENHANCED_REPXY) && (
				repetitionsz == BMM150_ENHANCED_REPZ)) {
					*mode = BMM150_PRESETMODE_ENHANCED;
				} else {
					*mode = E_BMM150_UNDEFINED_MODE;
				}
			}
		}
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_get_powermode(unsigned char *mode)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
		*mode = BMM150_GET_BITSLICE(v_data_u8r,
				BMM150_POWER_CNTL_PCB);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_powermode(unsigned char mode)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	//zack add
	unsigned char loop = 0;
	unsigned char retry_times = 3; //set retry times
	//end zack
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
		v_data_u8r = BMM150_SET_BITSLICE(v_data_u8r,
				BMM150_POWER_CNTL_PCB, mode);
		comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
	}
	//zack add	
	if(mode == BMM150_ON)
	{
	  while(loop < retry_times)
	  {
		p_bmm150->delay_msec(BMM150_DELAY_SUSPEND_SLEEP);
	    p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);		
		p_bmm150->delay_msec(1);
		if((v_data_u8r & 0x1) == 0) //check if power control bit is set to 1
		{
			v_data_u8r=0x1; //set reg0x4B = 0x1
			p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
		}
		else
		{
			break;
		}
		loop++;
	   }
	   if(loop == retry_times)
	   MSE_ERR("power mode bit has been set %d times\n", retry_times);
       else
	   MSE_ERR("power mode bit has been set successfully\n");	   
	}
	
	//end zack
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_get_repetitions_XY(
		unsigned char *no_repetitions_xy)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_NO_REPETITIONS_XY,
				&v_data_u8r, 1);
		*no_repetitions_xy = v_data_u8r;
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_repetitions_XY(
		unsigned char no_repetitions_xy)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		v_data_u8r = no_repetitions_xy;
		comres = p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_NO_REPETITIONS_XY,
				&v_data_u8r, 1);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_get_repetitions_Z(
		unsigned char *no_repetitions_z)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_NO_REPETITIONS_Z,
				&v_data_u8r, 1);
		*no_repetitions_z = v_data_u8r;
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_repetitions_Z(
		unsigned char no_repetitions_z)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		v_data_u8r = no_repetitions_z;
		comres = p_bmm150->BMM150_BUS_WRITE_FUNC(p_bmm150->dev_addr,
				BMM150_NO_REPETITIONS_Z, &v_data_u8r, 1);
	}
	return comres;
}

BMM150_S16 bmm150api_compensate_X(BMM150_S16 mdata_x, BMM150_U16 data_R)
{
	BMM150_S16 inter_retval;
	if (mdata_x != BMM150_FLIP_OVERFLOW_ADCVAL  /* no overflow */
	   ) {
		inter_retval = ((BMM150_S16)(((BMM150_U16)
				((((BMM150_S32)p_bmm150->dig_xyz1) << 14) /
				 (data_R != 0 ? data_R : p_bmm150->dig_xyz1))) -
				((BMM150_U16)0x4000)));
		inter_retval = ((BMM150_S16)((((BMM150_S32)mdata_x) *
				((((((((BMM150_S32)p_bmm150->dig_xy2) *
			      ((((BMM150_S32)inter_retval) *
				((BMM150_S32)inter_retval)) >> 7)) +
			     (((BMM150_S32)inter_retval) *
			      ((BMM150_S32)(((BMM150_S16)p_bmm150->dig_xy1)
			      << 7)))) >> 9) +
			   ((BMM150_S32)0x100000)) *
			  ((BMM150_S32)(((BMM150_S16)p_bmm150->dig_x2) +
			  ((BMM150_S16)0xA0)))) >> 12)) >> 13)) +
			(((BMM150_S16)p_bmm150->dig_x1) << 3);
	} else {
		/* overflow */
		inter_retval = BMM150_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMM150_S32 bmm150api_compensate_X_s32 (BMM150_S16 mdata_x, BMM150_U16 data_R)
{
	BMM150_S32 retval;

	retval = bmm150api_compensate_X(mdata_x, data_R);
	if (retval == (BMM150_S32)BMM150_OVERFLOW_OUTPUT)
		retval = BMM150_OVERFLOW_OUTPUT_S32;
	return retval;
}

#ifdef ENABLE_FLOAT
float bmm150api_compensate_X_float (BMM150_S16 mdata_x, BMM150_U16 data_R)
{
	float inter_retval;
	if (mdata_x != BMM150_FLIP_OVERFLOW_ADCVAL	/* no overflow */
	   ) {
		if (data_R != 0) {
			inter_retval = ((((float)p_bmm150->dig_xyz1)*16384.0f
				/data_R)-16384.0f);
		} else {
			inter_retval = 0;
		}
		inter_retval = (((mdata_x * ((((((float)p_bmm150->dig_xy2) *
			(inter_retval*inter_retval / 268435456.0f) +
			inter_retval*((float)p_bmm150->dig_xy1)/16384.0f))
			+ 256.0f) *	(((float)p_bmm150->dig_x2) + 160.0f)))
			/ 8192.0f) + (((float)p_bmm150->dig_x1) * 8.0f))/16.0f;
	} else {
		inter_retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}
#endif

BMM150_S16 bmm150api_compensate_Y(BMM150_S16 mdata_y, BMM150_U16 data_R)
{
	BMM150_S16 inter_retval;
	if (mdata_y != BMM150_FLIP_OVERFLOW_ADCVAL  /* no overflow */
	   ) {
		inter_retval = ((BMM150_S16)(((BMM150_U16)(((
			(BMM150_S32)p_bmm150->dig_xyz1) << 14) /
			(data_R != 0 ?
			 data_R : p_bmm150->dig_xyz1))) -
			((BMM150_U16)0x4000)));
		inter_retval = ((BMM150_S16)((((BMM150_S32)mdata_y) *
				((((((((BMM150_S32)
				       p_bmm150->dig_xy2) *
				      ((((BMM150_S32) inter_retval) *
					((BMM150_S32)inter_retval)) >> 7)) +
				     (((BMM150_S32)inter_retval) *
				      ((BMM150_S32)(((BMM150_S16)
				      p_bmm150->dig_xy1) << 7)))) >> 9) +
				   ((BMM150_S32)0x100000)) *
				  ((BMM150_S32)(((BMM150_S16)p_bmm150->dig_y2)
					  + ((BMM150_S16)0xA0))))
				 >> 12)) >> 13)) +
			(((BMM150_S16)p_bmm150->dig_y1) << 3);
	} else {
		/* overflow */
		inter_retval = BMM150_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMM150_S32 bmm150api_compensate_Y_s32 (BMM150_S16 mdata_y, BMM150_U16 data_R)
{
	BMM150_S32 retval;

	retval = bmm150api_compensate_Y(mdata_y, data_R);
	if (retval == BMM150_OVERFLOW_OUTPUT)
		retval = BMM150_OVERFLOW_OUTPUT_S32;
	return retval;
}

#ifdef ENABLE_FLOAT
float bmm150api_compensate_Y_float(BMM150_S16 mdata_y, BMM150_U16 data_R)
{
	float inter_retval;
	if (mdata_y != BMM150_FLIP_OVERFLOW_ADCVAL /* no overflow */
	   ) {
		if (data_R != 0) {
			inter_retval = ((((float)p_bmm150->dig_xyz1)*16384.0f
			/data_R)-16384.0f);
		} else {
			inter_retval = 0;
		}
		inter_retval = (((mdata_y * ((((((float)p_bmm150->dig_xy2) *
			(inter_retval*inter_retval / 268435456.0f) +
			inter_retval * ((float)p_bmm150->dig_xy1)/16384.0f)) +
			256.0f) * (((float)p_bmm150->dig_y2) + 160.0f)))
			/ 8192.0f) + (((float)p_bmm150->dig_y1) * 8.0f))/16.0f;
	} else {
		/* overflow, set output to 0.0f */
		inter_retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}
#endif

BMM150_S16 bmm150api_compensate_Z(BMM150_S16 mdata_z, BMM150_U16 data_R)
{
	BMM150_S32 retval;
	if ((mdata_z != BMM150_HALL_OVERFLOW_ADCVAL)	/* no overflow */
	   ) {
		retval = (((((BMM150_S32)(mdata_z - p_bmm150->dig_z4)) << 15) -
					((((BMM150_S32)p_bmm150->dig_z3) *
					  ((BMM150_S32)(((BMM150_S16)data_R) -
						  ((BMM150_S16)
						   p_bmm150->dig_xyz1))))>>2)) /
				(p_bmm150->dig_z2 +
				 ((BMM150_S16)(((((BMM150_S32)
					 p_bmm150->dig_z1) *
					 ((((BMM150_S16)data_R) << 1)))+
						 (1<<15))>>16))));
		/* saturate result to +/- 2 mT */
		if (retval > BMM150_POSITIVE_SATURATION_Z) {
			retval =  BMM150_POSITIVE_SATURATION_Z;
		} else {
			if (retval < BMM150_NEGATIVE_SATURATION_Z)
				retval = BMM150_NEGATIVE_SATURATION_Z;
		}
	} else {
		/* overflow */
		retval = BMM150_OVERFLOW_OUTPUT;
	}
	return (BMM150_S16)retval;
}

BMM150_S32 bmm150api_compensate_Z_s32(BMM150_S16 mdata_z, BMM150_U16 data_R)
{
	BMM150_S32 retval;
	if (mdata_z != BMM150_HALL_OVERFLOW_ADCVAL) {
		retval = (((((BMM150_S32)(mdata_z - p_bmm150->dig_z4)) << 15) -
			((((BMM150_S32)p_bmm150->dig_z3) *
			((BMM150_S32)(((BMM150_S16)data_R) -
			((BMM150_S16)p_bmm150->dig_xyz1))))>>2)) /
			(p_bmm150->dig_z2 +
			((BMM150_S16)(((((BMM150_S32)p_bmm150->dig_z1) *
			((((BMM150_S16)data_R) << 1)))+(1<<15))>>16))));
	} else {
		retval = BMM150_OVERFLOW_OUTPUT_S32;
	}
	return retval;
}

#ifdef ENABLE_FLOAT
float bmm150api_compensate_Z_float (BMM150_S16 mdata_z, BMM150_U16 data_R)
{
	float inter_retval;
	if (mdata_z != BMM150_HALL_OVERFLOW_ADCVAL /* no overflow */
	   ) {
		inter_retval = ((((((float)mdata_z)-((float)p_bmm150->dig_z4))*
		131072.0f)-(((float)p_bmm150->dig_z3)*(((float)data_R)-
		((float)p_bmm150->dig_xyz1))))/((((float)p_bmm150->dig_z2)+
		((float)p_bmm150->dig_z1)*((float)data_R)/32768.0)*4.0))/16.0;
	} else {
		/* overflow, set output to 0.0f */
		inter_retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}
#endif

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_control_measurement_x(
		unsigned char enable_disable)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_SENS_CNTL_CHANNELX__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMM150_SET_BITSLICE(v_data1_u8r,
				BMM150_SENS_CNTL_CHANNELX,
				enable_disable);
		comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_SENS_CNTL_CHANNELX__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_set_control_measurement_y(
		unsigned char enable_disable)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_SENS_CNTL_CHANNELY__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMM150_SET_BITSLICE(
				v_data1_u8r,
				BMM150_SENS_CNTL_CHANNELY,
				enable_disable);
		comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_SENS_CNTL_CHANNELY__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_soft_reset(void)
{
	BMM150_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		v_data_u8r = BMM150_ON;

		comres = p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_POWER_CNTL_SRST7__REG,
				&v_data_u8r, 1);
		v_data_u8r = BMM150_SET_BITSLICE(v_data_u8r,
				BMM150_POWER_CNTL_SRST7,
				BMM150_SOFT_RESET7_ON);
		comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_POWER_CNTL_SRST7__REG, &v_data_u8r, 1);

		comres |= p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_POWER_CNTL_SRST1__REG,
				&v_data_u8r, 1);
		v_data_u8r = BMM150_SET_BITSLICE(v_data_u8r,
				BMM150_POWER_CNTL_SRST1,
				BMM150_SOFT_RESET1_ON);
		comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_POWER_CNTL_SRST1__REG,
				&v_data_u8r, 1);

		p_bmm150->delay_msec(BMM150_DELAY_SOFTRESET);
	}
	return comres;
}

BMM150_RETURN_FUNCTION_TYPE bmm150api_get_raw_xyz(struct bmm150api_mdata *mdata)
{
	BMM150_RETURN_FUNCTION_TYPE comres;
	unsigned char a_data_u8r[6];
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
	} else {
		comres = p_bmm150->BMM150_BUS_READ_FUNC(p_bmm150->dev_addr,
				BMM150_DATAX_LSB, a_data_u8r, 6);

		a_data_u8r[0] = BMM150_GET_BITSLICE(a_data_u8r[0],
				BMM150_DATAX_LSB_VALUEX);
		mdata->datax = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[1]))
					<< SHIFT_LEFT_5_POSITION)
				| a_data_u8r[0]);

		a_data_u8r[2] = BMM150_GET_BITSLICE(a_data_u8r[2],
				BMM150_DATAY_LSB_VALUEY);
		mdata->datay = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[3]))
					<< SHIFT_LEFT_5_POSITION)
				| a_data_u8r[2]);

		a_data_u8r[4] = BMM150_GET_BITSLICE(a_data_u8r[4],
				BMM150_DATAZ_LSB_VALUEZ);
		mdata->dataz = (BMM150_S16)((((BMM150_S16)
						((signed char)a_data_u8r[5]))
					<< SHIFT_LEFT_7_POSITION)
				| a_data_u8r[4]);
	}
	return comres;
}

/*----------------------------------------------------------------------------*/
/* End of API Section */
/*----------------------------------------------------------------------------*/


static struct i2c_client *this_client;

/*calibration msensor and orientation data*/
static int sensor_data[CALIBRATION_DATA_SIZE];

static struct mutex sensor_data_mutex;
static DECLARE_WAIT_QUEUE_HEAD(uplink_event_flag_wq);

//static int bmm150d_delay = BMM150_DEFAULT_DELAY;

//static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);
static struct mutex uplink_event_flag_mutex;
/* uplink event flag */
/*static volatile u32 uplink_event_flag;*/

/* Maintain  cust info here */
struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id bmm150_i2c_id[] = {{BMM150_DEV_NAME, 0}, {} };
//static struct i2c_board_info __initdata bmm150_i2c_info = {I2C_BOARD_INFO(BMM150_DEV_NAME, BMM150_I2C_ADDR)};

/* 0<==>OK -1 <==> fail*/
static int bmm150_init_flag = -1;
static struct mag_init_info bmm150_init_info;

/*----------------------------------------------------------------------------*/

enum MMC_TRC {
	MMC_FUN_DEBUG  = 0x01,
	MMC_DATA_DEBUG = 0X02,
	MMC_HWM_DEBUG  = 0X04,
	MMC_CTR_DEBUG  = 0X08,
	MMC_I2C_DEBUG  = 0x10,
};

/*----------------------------------------------------------------------------*/
struct bmm150_i2c_data {
	struct i2c_client *client;
	struct mag_hw *hw;
	atomic_t layout;
	atomic_t trace;
	struct hwmsen_convert   cvt;

	struct bmm150api device;

	u8 op_mode;
	u8 odr;
	u8 rept_xy;
	u8 rept_z;
	s16 result_test;

#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
static void bmm150_restore_hw_cfg(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
static void bmm150_power(struct mag_hw *hw, unsigned int on)
{
}

/*Daemon application save the data*/
#if 0
static int ECS_SaveData(int buf[CALIBRATION_DATA_SIZE])
{
#if DEBUG
	struct bmm150_i2c_data *data = i2c_get_clientdata(this_client);
#endif

	mutex_lock(&sensor_data_mutex);
	switch (buf[0]) {
	case 2:	/* SENSOR_HANDLE_MAGNETIC_FIELD */
		memcpy(sensor_data+4, buf+1, 4*sizeof(int));
		break;
	case 3:	/* SENSOR_HANDLE_ORIENTATION */
		memcpy(sensor_data+8, buf+1, 4*sizeof(int));
		break;
#ifdef BMC150_M4G
	case 4:	/* SENSOR_HANDLE_GYROSCOPE */
		memcpy(m4g_data, buf+1, 4*sizeof(int));
		break;
#endif
#ifdef BMC150_VRV
	case 11:	/* SENSOR_HANDLE_ROTATION_VECTOR */
		memcpy(m4g_data+4, buf+1, 4*sizeof(int));
		break;
#endif
#ifdef BMC150_VLA
	case 10: /* SENSOR_HANDLE_LINEAR_ACCELERATION */
		memcpy(vla_data, buf+1, 4*sizeof(int));
		break;
#endif
#ifdef BMC150_VG
	case 9: /* SENSOR_HANDLE_GRAVITY */
		memcpy(vg_data, buf+1, 4*sizeof(int));
		break;
#endif
	default:
		break;
	}
	mutex_unlock(&sensor_data_mutex);

#if DEBUG
	if (atomic_read(&data->trace) & MMC_HWM_DEBUG) {
		MSE_LOG(
		"Get daemon data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
		sensor_data[0], sensor_data[1],
		sensor_data[2], sensor_data[3],
		sensor_data[4], sensor_data[5],
		sensor_data[6], sensor_data[7],
		sensor_data[8], sensor_data[9],
		sensor_data[10], sensor_data[11]);
	}
#endif

	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int ECS_GetRawData(int data[3])
{
	struct bmm150api_mdata_s32 mdata;
	struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);

	u8 databuf[2] = {BMM150_CONTROL, 0x02};

	bmm150api_read_mdataXYZ_s32(&mdata);
	/*data in uT*/
	data[0] = mdata.datax/16;
	data[1] = mdata.datay/16;
	data[2] = mdata.dataz/16;

	/* measure magnetic field for next sample */
	if (obj->op_mode == BMM150_SUSPEND_MODE) {
		/* power on firstly */
		bmm150api_set_powermode(BMM150_ON);
	}
	/* special treat of forced mode
	 * for optimization */
	i2c_master_send(this_client, databuf, 2);
	obj->op_mode = BMM150_SLEEP_MODE;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int bmm150_ReadChipInfo(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= BMM150_BUFSIZE - 1))
		return -BMM_ERROR_BUF;

	if (!this_client) {
		*buf = 0;
		return -BMM_ERROR_PTR;
	}

	snprintf(buf, BMM150_BUFSIZE, "BMM150 Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static void bmm150_SetPowerMode(struct i2c_client *client, bool enable)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(client);

	if (enable == false) {
		if (bmm150api_set_functional_state(BMM150_SUSPEND_MODE) != 0) {
			MSE_ERR("fail to suspend sensor");
			return;
		}
		obj->op_mode = BMM150_SUSPEND_MODE;
	} else {
		if (obj->op_mode == BMM150_SUSPEND_MODE)
		{	
	     //zack modify	
		 obj->op_mode = BMM150_NORMAL_MODE/*BMM150_SLEEP_MODE*/;
		 MSE_ERR("bmm150_SetPowerMode to normal mode");
		//end zack
		}
		bmm150_restore_hw_cfg(client);
	}
}

/*----------------------------------------------------------------------------*/
/* Driver Attributes Functions Section */
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[BMM150_BUFSIZE];
	bmm150_ReadChipInfo(strbuf, BMM150_BUFSIZE);
	return snprintf(buf, BMM150_BUFSIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int sensordata[3];
	char strbuf[BMM150_BUFSIZE];

	ECS_GetRawData(sensordata);
	snprintf(strbuf, BMM150_BUFSIZE, "%d %d %d\n",
	 sensordata[0], sensordata[1], sensordata[2]);
	return snprintf(buf, BMM150_BUFSIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[BMM150_BUFSIZE];
	tmp[0] = sensor_data[0] * CONVERT_O / CONVERT_O_DIV;
	tmp[1] = sensor_data[1] * CONVERT_O / CONVERT_O_DIV;
	tmp[2] = sensor_data[2] * CONVERT_O / CONVERT_O_DIV;
	snprintf(strbuf, BMM150_BUFSIZE,
	"%d, %d, %d\n", tmp[0], tmp[1], tmp[2]);

	return snprintf(buf, BMM150_BUFSIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct bmm150_i2c_data *data = i2c_get_clientdata(this_client);

	return snprintf(buf, BMM150_BUFSIZE,
	"(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
	data->hw->direction, atomic_read(&data->layout),
	data->cvt.sign[0], data->cvt.sign[1],
	data->cvt.sign[2], data->cvt.map[0],
	data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct bmm150_i2c_data *data = i2c_get_clientdata(this_client);

	int layout = 0;

	if (1 == sscanf(buf, "%d", &layout)) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt)) {
			MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(data->hw->direction, &data->cvt))
			MSE_ERR("invalid layout: %d, restore to %d\n",
			layout, data->hw->direction);
		else {
			MSE_ERR("invalid layout: (%d, %d)\n", layout,
			data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else
		MSE_ERR("invalid format = '%s'\n", buf);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct bmm150_i2c_data *data = i2c_get_clientdata(this_client);

	ssize_t len = 0;

	if (data->hw)
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
		data->hw->i2c_num, data->hw->direction,
		data->hw->power_id, data->hw->power_vol);
	else
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

	len += snprintf(buf + len, PAGE_SIZE - len, "i2c addr:%#x,ver:%s\n",
		data->client->addr, BMM_DRIVER_VERSION);

	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);

	if (NULL == obj) {
		MSE_ERR("bmm150_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);

	int trace;
	if (NULL == obj) {
		MSE_ERR("bmm150_i2c_data is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		MSE_ERR("invalid content: '%s', length = %d\n",
		buf, (int)count);

	return count;
}
/*----------------------------------------------------------------------------*/
#define BMM150_AXIS_X          0
#define BMM150_AXIS_Y          1
#define BMM150_AXIS_Z          2
static ssize_t show_cpsdata_value(struct device_driver *ddri, char *buf)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);

	struct bmm150api_mdata_s32 mdata;
	s32 mag[3];

	bmm150api_read_mdataXYZ_s32(&mdata);

	/*remap coordinate*/
	mag[obj->cvt.map[BMM150_AXIS_X]] =
	obj->cvt.sign[BMM150_AXIS_X]*mdata.datax;
	mag[obj->cvt.map[BMM150_AXIS_Y]] =
	obj->cvt.sign[BMM150_AXIS_Y]*mdata.datay;
	mag[obj->cvt.map[BMM150_AXIS_Z]] =
	obj->cvt.sign[BMM150_AXIS_Z]*mdata.dataz;

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n",
	mag[BMM150_AXIS_X], mag[BMM150_AXIS_Y], mag[BMM150_AXIS_Z]);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cpsopmode_value(struct device_driver *ddri, char *buf)
{
	u8 op_mode = 0xff;
	u8 power_mode;

	bmm150api_get_powermode(&power_mode);
	if (power_mode)
		bmm150api_get_functional_state(&op_mode);
	else
		op_mode = BMM150_SUSPEND_MODE;

	MSE_LOG("op_mode: %d", op_mode);
	return snprintf(buf, PAGE_SIZE, "%d\n", op_mode);
}

/*----------------------------------------------------------------------------*/
static ssize_t store_cpsopmode_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);
	long op_mode = -1;
	int err;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	if ((unsigned char)op_mode > 3)
		return -EINVAL;

	/* don't return error here */
	if (op_mode == obj->op_mode)
		return count;


	if (BMM150_FORCED_MODE == op_mode) {
		u8 databuf[2] = {BMM150_CONTROL, 0x02};
		if (obj->op_mode == BMM150_SUSPEND_MODE)
			bmm150api_set_powermode(BMM150_ON);
		/* special treat of forced mode
		 * for optimization */
		i2c_master_send(this_client, databuf, 2);
		obj->op_mode = BMM150_SLEEP_MODE;
	} else {
		bmm150api_set_functional_state((unsigned char)op_mode);
		obj->op_mode = op_mode;
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_cpsreptxy_value(struct device_driver *ddri, char *buf)
{
	unsigned char data = 0;
	u8 power_mode;
	int err;

	bmm150api_get_powermode(&power_mode);
	if (power_mode)
		err = bmm150api_get_repetitions_XY(&data);
	else
		err = -EIO;

	if (err)
		return err;

	return snprintf(buf, PAGE_SIZE, "%d\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cpsreptxy_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);

	unsigned long tmp = 0;
	int err;
	u8 data;
	u8 power_mode;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	bmm150api_get_powermode(&power_mode);
	if (power_mode) {
		err = bmm150api_set_repetitions_XY(data);
		if (!err)
			obj->rept_xy = data;
	} else {
		err = -EIO;
	}

	if (err)
		return err;

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_cpsreptz_value(struct device_driver *ddri, char *buf)
{
	unsigned char data = 0;
	u8 power_mode;
	int err;

	bmm150api_get_powermode(&power_mode);
	if (power_mode)
		err = bmm150api_get_repetitions_Z(&data);
	else
		err = -EIO;

	if (err)
		return err;

	return snprintf(buf, PAGE_SIZE, "%d\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cpsreptz_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);
	unsigned long tmp = 0;
	int err;
	u8 data;
	u8 power_mode;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	bmm150api_get_powermode(&power_mode);
	if (power_mode) {
		err = bmm150api_set_repetitions_Z(data);
		if (!err)
			obj->rept_z = data;
	} else {
		err = -EIO;
	}

	if (err)
		return err;

	return count;
}

static ssize_t show_test_value(struct device_driver *ddri, char *buf)
{
	struct bmm150_i2c_data *client_data = i2c_get_clientdata(this_client);
	int err;

	err = snprintf(buf, PAGE_SIZE, "%d\n", client_data->result_test);
	return err;
}

static ssize_t store_test_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct bmm150_i2c_data *client_data = i2c_get_clientdata(this_client);
	u8 dummy;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	/* the following code assumes the work thread is not running */
	if (1 == data) {
		/* self test */
		err = bmm150api_set_functional_state(BMM150_SLEEP_MODE);
		udelay(3000);
		err = bmm150api_set_selftest(1);
		udelay(3000);
		err = bmm150api_get_self_test_XYZ(&dummy);
		client_data->result_test = dummy;
	} else if (2 == data) {
		/* advanced self test */
		err =
		bmm150api_perform_advanced_selftest(&client_data->result_test);
	} else {
		err = -EINVAL;
	}

	if (!err) {
		bmm150api_soft_reset();
		udelay(1000);
		bmm150_restore_hw_cfg(this_client);
	}

	if (err)
		count = -1;

	return count;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value,
	store_layout_value);
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value,
	store_trace_value);
static DRIVER_ATTR(cpsdata, S_IWUSR | S_IRUGO, show_cpsdata_value, NULL);
static DRIVER_ATTR(cpsopmode,      S_IRUGO | S_IWUSR, show_cpsopmode_value,
	store_cpsopmode_value);
static DRIVER_ATTR(cpsreptxy,      S_IRUGO | S_IWUSR, show_cpsreptxy_value,
	store_cpsreptxy_value);
static DRIVER_ATTR(cpsreptz,      S_IRUGO | S_IWUSR, show_cpsreptz_value,
	store_cpsreptz_value);
static DRIVER_ATTR(test,      S_IRUGO | S_IWUSR, show_test_value,
	store_test_value);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *bmm150_attr_list[] = {
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_posturedata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
	&driver_attr_cpsdata,
	&driver_attr_cpsopmode,
	&driver_attr_cpsreptxy,
	&driver_attr_cpsreptz,
	&driver_attr_test,
};
/*----------------------------------------------------------------------------*/
static int bmm150_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(bmm150_attr_list)/sizeof(bmm150_attr_list[0]));
	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, bmm150_attr_list[idx]);
		if (err) {
			MSE_ERR("driver_create_file (%s) = %d\n",
			 bmm150_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int bmm150_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(bmm150_attr_list)/sizeof(bmm150_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, bmm150_attr_list[idx]);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int bmm150_open(struct inode *inode, struct file *file)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);
	int ret = -1;

	if (atomic_read(&obj->trace) & MMC_CTR_DEBUG)
		MSE_LOG("Open device node:bmm150\n");

	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int bmm150_release(struct inode *inode, struct file *file)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);

	if (atomic_read(&obj->trace) & MMC_CTR_DEBUG)
		MSE_LOG("Release device node:bmm150\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define BMM_IOC_GET_EVENT_FLAG	ECOMPASS_IOC_GET_OPEN_STATUS

static long bmm150_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char buff[BMM150_BUFSIZE];/* for chip information */

	int vec[3] = {0};
	struct hwm_sensor_data *osensor_data;
	uint32_t enable;

	switch (cmd) {
	case MSENSOR_IOCTL_READ_CHIPINFO:
		if (argp == NULL) {
			MSE_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}
		bmm150_ReadChipInfo(buff, BMM150_BUFSIZE);
		if (copy_to_user(argp, buff, strlen(buff)+1))
			return -EFAULT;
		break;

		/*used by MTK ftm or engineering mode*/
	case MSENSOR_IOCTL_READ_SENSORDATA:
		if (argp == NULL) {
			MSE_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}
		ECS_GetRawData(vec);
		snprintf(buff, BMM150_BUFSIZE,
		"%x %x %x", vec[0], vec[1], vec[2]);
		if (copy_to_user(argp, buff, strlen(buff)+1))
			return -EFAULT;
		break;
	/*used by MTK ftm*/
	case MSENSOR_IOCTL_SENSOR_ENABLE:
		if (argp == NULL) {
			MSE_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}
		if (copy_from_user(&enable, argp, sizeof(enable))) {
			MSE_ERR("copy_from_user failed.");
			return -EFAULT;
		} else {
			MSE_LOG(
			"MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\r\n",
			 enable);
			if (1 == enable)
				atomic_set(&o_flag, 1);
			else
				atomic_set(&o_flag, 0);

			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
		}
		break;

	/*used by MTK ftm*/
	case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
		if (argp == NULL) {
			MSE_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}

		osensor_data = (struct hwm_sensor_data *)buff;
		mutex_lock(&sensor_data_mutex);

		osensor_data->values[0] = sensor_data[8];
		osensor_data->values[1] = sensor_data[9];
		osensor_data->values[2] = sensor_data[10];
		osensor_data->status = sensor_data[11];
		osensor_data->value_divide = CONVERT_O_DIV;

		mutex_unlock(&sensor_data_mutex);

		snprintf(buff, BMM150_BUFSIZE, "%x %x %x %x %x",
		osensor_data->values[0], osensor_data->values[1],
		osensor_data->values[2], osensor_data->status,
		 osensor_data->value_divide);
		if (copy_to_user(argp, buff, strlen(buff)+1))
			return -EFAULT;
		break;

	default:
		MSE_ERR("%s not supported = 0x%04x", __func__, cmd);
		return -ENOIOCTLCMD;
		break;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static const struct file_operations bmm150_fops = {
	.owner = THIS_MODULE,
	.open = bmm150_open,
	.release = bmm150_release,
	.unlocked_ioctl = bmm150_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice bmm150_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msensor",
	.fops = &bmm150_fops,
};

/*----------------------------------------------------------------------------*/
static void bmm150_restore_hw_cfg(struct i2c_client *client)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(client);

	if (obj->op_mode > 3)
		obj->op_mode = BMM150_SLEEP_MODE;
	bmm150api_set_functional_state(obj->op_mode);

	bmm150api_set_datarate(obj->odr);

	bmm150api_set_repetitions_XY(obj->rept_xy);

	bmm150api_set_repetitions_Z(obj->rept_z);
}

/*----------------------------------------------------------------------------*/
#ifndef	CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int bmm150_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct bmm150_i2c_data *data = i2c_get_clientdata(client);

	if (msg.event == PM_EVENT_SUSPEND) {
		bmm150_SetPowerMode(data->client, false);
		bmm150_power(data->hw, 0);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bmm150_resume(struct i2c_client *client)
{
	struct bmm150_i2c_data *data = i2c_get_clientdata(client);

	bmm150_power(data->hw, 1);
	bmm150_SetPowerMode(data->client, true);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void bmm150_early_suspend(struct early_suspend *h)
{
	struct bmm150_i2c_data *obj =
	container_of(h, struct bmm150_i2c_data, early_drv);

	if (NULL == obj) {
		MSE_ERR("null pointer!!\n");
		return;
	}

	bmm150_SetPowerMode(obj->client, false);
	bmm150_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void bmm150_late_resume(struct early_suspend *h)
{
	struct bmm150_i2c_data *obj =
	container_of(h, struct bmm150_i2c_data, early_drv);

	if (NULL == obj) {
		MSE_ERR("null pointer!!\n");
		return;
	}

	bmm150_power(obj->hw, 1);
	bmm150_SetPowerMode(obj->client, true);
}
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
#define BMM_MAX_RETRY_WAKEUP (5)
#define BMM_I2C_WRITE_DELAY_TIME (1)
static int bmm150_wakeup(struct i2c_client *client)
{
	int err = 0;
	int try_times = BMM_MAX_RETRY_WAKEUP;
	u8 data[2] = {BMM150_POWER_CNTL, 0x01};
	u8 dummy;
	int count = 0;
	MSE_LOG("waking up the chip...");

	while (try_times) {

		err = i2c_master_send(client, data, 2);
		count++;
		udelay(BMM_I2C_WRITE_DELAY_TIME * 1000);
		while ((count < 3) && (err < 0)) {
			err = i2c_master_send(client, data, 2);
			count++;
			udelay(BMM_I2C_WRITE_DELAY_TIME * 1000);
	}
		dummy = 0;
		err += hwmsen_read_block(client, BMM150_POWER_CNTL, &dummy, 1);
		if (data[1] == dummy)
			break;
		try_times--;
	}

	MSE_LOG("wake up result: %s, tried times: %d",
			(try_times > 0) ? "succeed" : "fail",
			BMM_MAX_RETRY_WAKEUP - try_times + 1);

	err = (try_times > 0) ? 0 : -1;

	return err;
}
/*----------------------------------------------------------------------------*/
static int bmm150_checkchipid(struct i2c_client *client)
{
	u8 chip_id = 0;

	hwmsen_read_block(client, BMM150_CHIP_ID, &chip_id, 1);
	MSE_LOG("read chip id result: %#x", chip_id);

	if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMM)
		return -BMM_ERROR_BUF;
	else
		return 0;
}
/*----------------------------------------------------------------------------*/
static char bmm150_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	return hwmsen_read_block(this_client, reg_addr, data, len);
}
/*----------------------------------------------------------------------------*/
static char bmm150_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	u8 buff[BMM150_BUFSIZE + 1];

	if (len > BMM150_BUFSIZE)
		return -BMM_ERROR_BUF;

	buff[0] = reg_addr;
	memcpy(buff+1, data, len);
	/* I2C transfer error */
	if (i2c_master_send(this_client, buff, len+1) != (len+1))
		return -EIO;
	else
		return 0;
}
/*----------------------------------------------------------------------------*/
static void bmm150_delay(u32 msec)
{
	udelay(msec*1000);
}
/*----------------------------------------------------------------------------*/
static int bmm150_init_client(struct i2c_client *client)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	res = bmm150_wakeup(client);
	if (res < 0)
		return res;
	res = bmm150_checkchipid(client);
	if (res < 0)
		return res;
	MSE_LOG("check chip ID ok");

	/*bmm150 api init*/
	obj->device.bus_read = bmm150_i2c_read_wrapper;
	obj->device.bus_write = bmm150_i2c_write_wrapper;
	obj->device.delay_msec = bmm150_delay;
	bmm150api_init(&obj->device);

	/* now it's power on which is considered as resuming from suspend */
	obj->op_mode = BMM150_SUSPEND_MODE;
	obj->odr = BMM150_REGULAR_DR;
	obj->rept_xy = BMM150_REGULAR_REPXY;
	obj->rept_z = BMM150_REGULAR_REPZ;

	res = bmm150api_set_functional_state(BMM150_SUSPEND_MODE);
	if (res)
		return -EIO;

	return 0;
}
/*----------------------------------------------------------------------------*/

int __attribute__((weak)) bsx_algo_m_enable(int en)
{
	char comres = 0;
	unsigned char v_data1_u8r;
	comres = bmm150api_get_powermode(&v_data1_u8r);
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
		return comres;
	}
	if ((v_data1_u8r == BMM150_OFF) && (en == true)) {
		comres |= bmm150api_set_powermode(BMM150_ON);
		p_bmm150->delay_msec(
			BMM150_DELAY_SUSPEND_SLEEP);
		{
			comres |= p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_CNTL_OPMODE__REG,
				&v_data1_u8r, 1);
			v_data1_u8r = BMM150_SET_BITSLICE(
				v_data1_u8r,
				BMM150_CNTL_OPMODE,
				BMM150_NORMAL_MODE);
			comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_CNTL_OPMODE__REG,
				&v_data1_u8r, 1);
		}
	} else if ((v_data1_u8r == BMM150_ON) && (en == false))
		comres = bmm150api_set_powermode(BMM150_OFF);


	return comres;
}

int __attribute__((weak)) bsx_algo_m_set_delay(u64 ns)
{
	return 0;
}
int __attribute__((weak)) bsx_algo_m_open_report_data(int open)
{
	return 0;
}

int __attribute__((weak)) bsx_algo_o_enable(int en)
{
	char comres = 0;
	unsigned char v_data1_u8r;
	comres = bmm150api_get_powermode(&v_data1_u8r);
	if (p_bmm150 == BMM150_NULL) {
		comres = E_BMM150_NULL_PTR;
		return comres;
	}
	if ((v_data1_u8r == BMM150_OFF) && (en == true)) {
		//zack add
		MSE_ERR("bsx_algo_o_enable set power mode bit\n");
        //end zack
		comres |= bmm150api_set_powermode(BMM150_ON);
		p_bmm150->delay_msec(
			BMM150_DELAY_SUSPEND_SLEEP);
		{
			comres |= p_bmm150->BMM150_BUS_READ_FUNC(
				p_bmm150->dev_addr,
				BMM150_CNTL_OPMODE__REG,
				&v_data1_u8r, 1);
			v_data1_u8r = BMM150_SET_BITSLICE(
				v_data1_u8r,
				BMM150_CNTL_OPMODE,
				BMM150_NORMAL_MODE);
			comres |= p_bmm150->BMM150_BUS_WRITE_FUNC(
				p_bmm150->dev_addr,
				BMM150_CNTL_OPMODE__REG,
				&v_data1_u8r, 1);
		}
	} else if ((v_data1_u8r == BMM150_ON) && (en == false))
		comres = bmm150api_set_powermode(BMM150_OFF);


	return comres;
}

int __attribute__((weak)) bsx_algo_o_set_delay(u64 ns)
{
	return 0;
}

int __attribute__((weak)) bsx_algo_o_open_report_data(int open)
{
	return 0;
}

int __attribute__((weak)) bsx_algo_o_get_data(
	int *x, int *y, int *z, int *status)
{
struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);

	struct bmm150api_mdata_s32 mdata;
	s32 mag[3];

	bmm150api_read_mdataXYZ_s32(&mdata);

	/*remap coordinate*/
	mag[obj->cvt.map[BMM150_AXIS_X]] =
	obj->cvt.sign[BMM150_AXIS_X]*mdata.datax;
	mag[obj->cvt.map[BMM150_AXIS_Y]] =
	obj->cvt.sign[BMM150_AXIS_Y]*mdata.datay;
	mag[obj->cvt.map[BMM150_AXIS_Z]] =
	obj->cvt.sign[BMM150_AXIS_Z]*mdata.dataz;

	(*x) = mag[BMM150_AXIS_X] / 16;
	(*y) = mag[BMM150_AXIS_Y] / 16;
	(*z) = mag[BMM150_AXIS_Z] / 16;
	(*status) = 3;
	//zack add
	MSE_ERR("bsx_algo_o_get_data *x=%d, *y=%d, *z=%d\n", *x, *y, *z);
	//end zack
	return 0;
}

int __attribute__((weak)) bsx_algo_m_get_data(
	int *x, int *y, int *z, int *status)
{
struct bmm150_i2c_data *obj = i2c_get_clientdata(this_client);
struct bmm150api_mdata_s32 mdata;
	s32 mag[3];

	bmm150api_read_mdataXYZ_s32(&mdata);

	/*remap coordinate*/
	mag[obj->cvt.map[BMM150_AXIS_X]] =
	obj->cvt.sign[BMM150_AXIS_X]*mdata.datax;
	mag[obj->cvt.map[BMM150_AXIS_Y]] =
	obj->cvt.sign[BMM150_AXIS_Y]*mdata.datay;
	mag[obj->cvt.map[BMM150_AXIS_Z]] =
	obj->cvt.sign[BMM150_AXIS_Z]*mdata.dataz;

	(*x) = mag[BMM150_AXIS_X];
	(*y) = mag[BMM150_AXIS_Y];
	(*z) = mag[BMM150_AXIS_Z];
	(*status) = 3;
	return 0;
}

static int bmm150_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
    return bsx_algo_m_set_delay((u64)samplingPeriodNs);
}

static int bmm150_flush(void)
{
    return mag_flush_report();
}

/*----------------------------------------------------------------------------*/
int bmm150_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	struct i2c_client *new_client;
	struct bmm150_i2c_data *data;
	struct mag_control_path ctl = {0};
	struct mag_data_path mag_data = {0};

	MSE_FUN();

	data = kmalloc(sizeof(struct bmm150_i2c_data), GFP_KERNEL);
	if (!(data))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct bmm150_i2c_data));

	data->hw = hw;
	err = hwmsen_get_convert(data->hw->direction, &data->cvt);
	if (err)
	{
		MSE_ERR("invalid direction: %d\n", data->hw->direction);
		goto exit;
	}
	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);

	mutex_init(&sensor_data_mutex);
	mutex_init(&uplink_event_flag_mutex);

	init_waitqueue_head(&uplink_event_flag_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	this_client = new_client;

	/*initial client*/
	err = bmm150_init_client(this_client);
	if (err)
	{
		MSE_ERR("fail to initialize client");
		goto exit_client_failed;
	}

	/* Register sysfs attribute */
	err = bmm150_create_attr(&bmm150_init_info.platform_diver_addr->driver);
	if (err)
	{
		MSE_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = misc_register(&bmm150_device);
	if (err)
	{
		MSE_ERR("bmm150_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	ctl.is_use_common_factory = false;
	ctl.enable = bsx_algo_o_enable;
	ctl.set_delay  = bsx_algo_o_set_delay;
	ctl.open_report_data = bsx_algo_o_open_report_data;
	ctl.batch = bmm150_batch;
	ctl.flush = bmm150_flush;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = data->hw->is_batch_supported;
	strlcpy(ctl.libinfo.libname, "bmm", sizeof(ctl.libinfo.libname));

	err = mag_register_control_path(&ctl);
	if (err)
	{
		MSE_ERR("register mag control path err\n");
		goto exit_kfree;
	}

	//zack modify
	mag_data.div = 4;
	//end zack	
	mag_data.get_data = bsx_algo_o_get_data;

	err = mag_register_data_path(&mag_data);
	if (err)
	{
		MSE_ERR("register data control path err\n");
		goto exit_kfree;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	data->early_drv.suspend  = bmm150_early_suspend,
	data->early_drv.resume   = bmm150_late_resume,
	register_early_suspend(&data->early_drv);
#endif

	bmm150_init_flag = 1;

	MSE_ERR("bmm150_i2c_probe OK\n");

	return 0;

exit_client_failed:
exit_sysfs_create_group_failed:
exit_misc_device_register_failed:
exit_kfree:
	kfree(data);

exit:
	bmm150_init_flag = -1;
	MSE_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int bmm150_i2c_remove(struct i2c_client *client)
{
	struct bmm150_i2c_data *obj = i2c_get_clientdata(client);

	if (bmm150_delete_attr(&bmm150_init_info.platform_diver_addr->driver))
		MSE_ERR("bmm150_delete_attr fail");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&obj->early_drv);
#endif
	bmm150api_set_functional_state(BMM150_SUSPEND_MODE);
	this_client = NULL;
	i2c_unregister_device(client);

	kfree(obj);
	misc_deregister(&bmm150_device);
	return 0;
}
/*----------------------------------------------------------------------------*/

#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,msensor"},
	{},
};
#endif

static struct i2c_driver bmm150_i2c_driver = {
	.driver = {
		.name  = BMM150_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = mag_of_match,
#endif
	},
	.probe      = bmm150_i2c_probe,
	.remove     = bmm150_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend    = bmm150_suspend,
	.resume     = bmm150_resume,
#endif
	.id_table = bmm150_i2c_id,
};

static int bmm150_remove(void)
{
	MSE_FUN();
	bmm150_power(hw, 0);
	i2c_del_driver(&bmm150_i2c_driver);
	return 0;
}

static int bmm150_local_init(void)
{
	MSE_LOG("fwq loccal init+++\n");

	bmm150_power(hw, 1);

	if (i2c_add_driver(&bmm150_i2c_driver))
	{
		MSE_ERR("add driver error\n");
		return -BMM_ERROR_BUF;
	}

	if (-1 == bmm150_init_flag)
	{
		MSE_FUN();
		return -BMM_ERROR_BUF;
	}

	MSE_LOG("fwq loccal init---\n");
	return 0;
}

static struct mag_init_info bmm150_init_info = {
	.name = "bmm150",
	.init = bmm150_local_init,
	.uninit = bmm150_remove,
};

/*----------------------------------------------------------------------------*/
static int __init bmm150_init(void)
{
	const char *name = "mediatek,bmm150";

	MSE_FUN();

	//hw = get_mag_dts_func(name, hw);
	hw = f_get_mag_dts_func(name, hw);

	//i2c_register_board_info(hw->i2c_num, &bmm150_i2c_info, 1);

	mag_driver_add(&bmm150_init_info);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit bmm150_exit(void)
{
	MSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(bmm150_init);
module_exit(bmm150_exit);

MODULE_AUTHOR("contact@bosch-sensortec.com");
MODULE_DESCRIPTION("BMM150 COMPASS SENSOR DRIVER");
MODULE_LICENSE("GPL v2");
