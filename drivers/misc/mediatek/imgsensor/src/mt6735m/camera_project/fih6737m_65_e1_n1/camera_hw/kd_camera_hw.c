#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/gpio.h>
#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#ifdef _LOG_USER_
#define PK_DBG_FUNC(fmt, args...)
#else
#define PK_DBG_FUNC(fmt, args...)    pr_debug(PFX  fmt, ##args)
#endif
#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#ifdef _LOG_USER_
#define PK_ERR(fmt, arg...)
#else
#define PK_ERR(fmt, arg...) pr_err(fmt, ##arg)
#endif
#define PK_XLOG_INFO(fmt, args...)  pr_debug(PFX  fmt, ##args)

#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif

#define GPIO_CAMERA_ID0_PIN 99
#define GPIO_CAMERA_ID1_PIN 98


int camIdPin = -1;
int cam1IdPin = -1;

/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;
struct pinctrl_state *cam0_pnd_l = NULL;

struct pinctrl_state *cam1_pnd_h = NULL;
struct pinctrl_state *cam1_pnd_l = NULL;

struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;

struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;

struct pinctrl_state *cam_ldo0_h = NULL;
struct pinctrl_state *cam_ldo0_l = NULL;

struct pinctrl_state *cam0_avdd_h = NULL;
struct pinctrl_state *cam0_avdd_l = NULL;

struct pinctrl_state *cam1_avdd_h = NULL;
struct pinctrl_state *cam1_avdd_l = NULL;

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/*Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		PK_DBG_FUNC("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		PK_DBG_FUNC("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}


	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		PK_DBG_FUNC("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		PK_DBG_FUNC("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

    /*Cam1 Power/Rst Ping initialization*/
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		PK_DBG_FUNC("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l)) {
		ret = PTR_ERR(cam1_pnd_l);
		PK_DBG_FUNC("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		PK_DBG_FUNC("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		PK_DBG_FUNC("%s : pinctrl err, cam1_rst_l\n", __func__);
	}
	
	/*externel LDO enable */
	cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
	if (IS_ERR(cam_ldo0_h)) {
		ret = PTR_ERR(cam_ldo0_h);
		PK_DBG_FUNC("%s : pinctrl err, cam_ldo0_h\n", __func__);
	}


	cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
	if (IS_ERR(cam_ldo0_l)) {
		ret = PTR_ERR(cam_ldo0_l);
		PK_DBG_FUNC("%s : pinctrl err, cam_ldo0_l\n", __func__);
	}
	
	
	/*AVDD enable*/
	cam0_avdd_h = pinctrl_lookup_state(camctrl, "cam0_avdd1");
	if (IS_ERR(cam0_avdd_h)) {
		ret = PTR_ERR(cam0_avdd_h);
		PK_DBG_FUNC("%s : pinctrl err, cam0_avdd_h\n", __func__);
	}


	cam0_avdd_l = pinctrl_lookup_state(camctrl, "cam0_avdd0");
	if (IS_ERR(cam0_avdd_l)) {
		ret = PTR_ERR(cam0_avdd_l);
		PK_DBG_FUNC("%s : pinctrl err, cam0_avdd_l\n", __func__);
	}
		
	cam1_avdd_h = pinctrl_lookup_state(camctrl, "cam1_avdd1");
	if (IS_ERR(cam1_avdd_h)) {
		ret = PTR_ERR(cam1_avdd_h);
		PK_DBG_FUNC("%s : pinctrl err, cam1_avdd_h\n", __func__);
	}


	cam1_avdd_l = pinctrl_lookup_state(camctrl, "cam1_avdd0");
	if (IS_ERR(cam1_avdd_l)) {
		ret = PTR_ERR(cam1_avdd_l);
		PK_DBG_FUNC("%s : pinctrl err, cam1_avdd_l\n", __func__);
	}
	
	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;
	switch (PwrType) {
	
	case CAMRST:

		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		break;
		
	case CAMPDN:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else
				pinctrl_select_state(camctrl, cam1_pnd_h);
		}

		break;
	case CAMLDO:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
		break;
	
	case CAMAVDD:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_avdd_l);
			else
				pinctrl_select_state(camctrl, cam0_avdd_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_avdd_l);
			else
				pinctrl_select_state(camctrl, cam1_avdd_h);
		}
		break;
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}




int cntVCAMD = 0;
int cntVCAMA = 0;
int cntVCAMIO = 0;
int cntVCAMAF = 0;
int cntVCAMD_SUB = 0;

static DEFINE_SPINLOCK(kdsensor_pw_cnt_lock);


bool _hwPowerOnCnt(KD_REGULATOR_TYPE_T powerId, int powerVolt, char *mode_name)
{

	if (_hwPowerOn(powerId, powerVolt)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD += 1;
		else if (powerId == VCAMA)
			cntVCAMA += 1;
		else if (powerId == VCAMIO)
			cntVCAMIO += 1;
		else if (powerId == VCAMAF)
			cntVCAMAF += 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

bool _hwPowerDownCnt(KD_REGULATOR_TYPE_T powerId, char *mode_name)
{

	if (_hwPowerDown(powerId)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD -= 1;
		else if (powerId == VCAMA)
			cntVCAMA -= 1;
		else if (powerId == VCAMIO)
			cntVCAMIO -= 1;
		else if (powerId == VCAMAF)
			cntVCAMAF -= 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

void checkPowerBeforClose(char *mode_name)
{

	int i = 0;

	PK_DBG
	    ("[checkPowerBeforClose]cntVCAMD:%d, cntVCAMA:%d,cntVCAMIO:%d, cntVCAMAF:%d, cntVCAMD_SUB:%d,\n",
	     cntVCAMD, cntVCAMA, cntVCAMIO, cntVCAMAF, cntVCAMD_SUB);


	for (i = 0; i < cntVCAMD; i++)
		_hwPowerDown(VCAMD);
	for (i = 0; i < cntVCAMA; i++)
		_hwPowerDown(VCAMA);
	for (i = 0; i < cntVCAMIO; i++)
		_hwPowerDown(VCAMIO);
	for (i = 0; i < cntVCAMAF; i++)
		_hwPowerDown(VCAMAF);

	cntVCAMD = 0;
	cntVCAMA = 0;
	cntVCAMIO = 0;
	cntVCAMAF = 0;
	cntVCAMD_SUB = 0;

}



int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On,
		       char *mode_name)
{

	u32 pinSetIdx = 0;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000


	u32 pinSet[3][8] = {
		/* for main sensor */
		{CAMERA_CMRST_PIN,
		 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
			GPIO_OUT_ONE,              /* ON state */
			GPIO_OUT_ZERO,             /* OFF state */
			CAMERA_CMPDN_PIN,
			CAMERA_CMPDN_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		/* for sub sensor */
		{CAMERA_CMRST1_PIN,
		 CAMERA_CMRST1_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
			CAMERA_CMPDN1_PIN,
			CAMERA_CMPDN1_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		{GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,	/* mode */
			GPIO_OUT_ONE,               /* ON state */
			GPIO_OUT_ZERO,              /* OFF state */
			GPIO_CAMERA_INVALID,
			GPIO_CAMERA_INVALID,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		}
	};

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
		pinSetIdx = 0;
	 else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
		pinSetIdx = 1;
	 else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
		pinSetIdx = 2;

	if (On) {

		PK_DBG("[PowerON]pinSetIdx:%d, currSensorName: %s\n", pinSetIdx, currSensorName);

	//if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4H8_MIPI_RAW, currSensorName)) && (0 == pinSetIdx)) {
	if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4H8_MIPI_RAW, currSensorName))) || (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OVS5K4H8_MIPI_RAW, currSensorName))) ) {

			ISP_MCLK1_EN(1);
 			//mdelay(5);

			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);

			mdelay(1);

			/* VCAM_D */
			if (TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAM_D),power id = %d\n",
				     VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);

			/* VCAM_A */
			mtkcam_gpio_set(pinSetIdx, CAMAVDD, 1);

			mdelay(1);

			/* VCAM_IO */
			if (TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO),power id = %d\n",
				     VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			/* AF_VCC */
			if (TRUE != _hwPowerOnCnt(VCAMAF, VOL_2800, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF),power id = %d\n",
				     VCAMAF);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);

			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);

			mdelay(1);

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
			}
			mdelay(3);

			camIdPin = gpio_get_value(GPIO_CAMERA_ID0_PIN);

			//ISP_MCLK1_EN(1);
		}
#if 1
    //else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4H8SUB_MIPI_RAW, currSensorName)) && (1 == pinSetIdx))
	else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4H8SUB_MIPI_RAW, currSensorName))) 
		|| (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OVS5K4H8SUB_MIPI_RAW, currSensorName))))
		{
			ISP_MCLK1_EN(1);
 			//mdelay(5);

			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);

			mdelay(1);

			mtkcam_gpio_set(pinSetIdx, CAMAVDD, 1);

			mdelay(1);

			/* VCAM_D */
			if (TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAM_D),power id = %d\n",
				     VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);

			/* VCAM_IO */
			if (TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO),power id = %d\n",
				     VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			/* AF_VCC */
			if (TRUE != _hwPowerOnCnt(VCAMAF, VOL_2800, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF),power id = %d\n",
				     VCAMAF);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);

			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);

			mdelay(1);

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
			}
			mdelay(3);

			cam1IdPin = gpio_get_value(GPIO_CAMERA_ID1_PIN);
        }
#endif
    else {
			/* First Power Pin low and Reset Pin Low */
				PK_DBG("sensor is not found\n");
				return -1;
		}

	} else {		/* power OFF */

		PK_DBG("[PowerOFF]pinSetIdx:%d\n", pinSetIdx);


		//if (currSensorName &&(0 == strcmp(SENSOR_DRVNAME_S5K4H8_MIPI_RAW, currSensorName)) && (0 == pinSetIdx)) {
		if ((currSensorName &&(0 == strcmp(SENSOR_DRVNAME_S5K4H8_MIPI_RAW, currSensorName)))
			|| (currSensorName &&(0 == strcmp(SENSOR_DRVNAME_OVS5K4H8_MIPI_RAW, currSensorName))) ) {

            ISP_MCLK1_EN(0);

			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
			}			
			
			/* Set Reset Pin Low */

    		mdelay(3);

			/* AF_VCC */
			if (TRUE != _hwPowerDownCnt(VCAMAF, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d\n",
				     VCAMAF);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);


			if (TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d\n",
				     VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(2);

			/* VCAM_A */
			mtkcam_gpio_set(pinSetIdx, CAMAVDD, 0);
			
			mdelay(1);

            /* VCAM_IO */
            if (TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
                PK_DBG
                    ("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n",
                     VCAMIO);
                goto _kdCISModulePowerOn_exit_;
            }
			mdelay(1);

  }
        #if 1
        //else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4H8SUB_MIPI_RAW, currSensorName)) && (1 == pinSetIdx)) {
			else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4H8SUB_MIPI_RAW, currSensorName))) || (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OVS5K4H8SUB_MIPI_RAW, currSensorName)))  ) {

               ISP_MCLK1_EN(0);

			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);			
			}

			/* Set Reset Pin Low */
			mdelay(2);

			/* AF_VCC */
			if (TRUE != _hwPowerDownCnt(VCAMAF, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d\n",
				     VCAMAF);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(2);

			if (TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d\n",
				     VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(2);

			/* VCAM_A */
			mtkcam_gpio_set(pinSetIdx, CAMAVDD, 0);

			mdelay(1);

			/* VCAM_IO */

			if (TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n",
				     VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);
		}
        #endif
        else {

			PK_DBG("[PowerOFF] err\n");
			return -1;
		}

			//ISP_MCLK1_EN(0);
	}


	return 0;


_kdCISModulePowerOn_exit_:
	return -EIO;

}

EXPORT_SYMBOL(kdCISModulePowerOn);

/* !-- */
/*  */
