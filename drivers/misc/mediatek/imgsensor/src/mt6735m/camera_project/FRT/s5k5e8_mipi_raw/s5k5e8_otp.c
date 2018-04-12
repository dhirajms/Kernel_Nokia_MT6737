/*
* Copyright (C) 2016 MediaTek Inc.
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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k5e8mipiraw_Sensor.h"

/*#include <linux/xlog.h>*/
#define PFX "s5k5e8_otp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_ERR(format, args...)    pr_err(PFX "[%s] " format, __FUNCTION__, ##args)


//extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
//extern  BYTE imx135_byteread_cmos_sensor(kal_uint32 addr);
/*
extern kal_uint16 read_cmos_sensor(kal_uint32 addr);
extern void write_cmos_sensor(kal_uint32 addr, kal_uint32 para);
extern void write_cmos_sensor_16(kal_uint16 addr,kal_uint16 para);
extern void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
*/
#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#if 0
#define VALID_OTP          0x40

#define GAIN_DEFAULT       0x0100
#define GAIN_GREEN1_ADDR_H   0x020E
#define GAIN_GREEN1_ADDR_L   0x020F

#define GAIN_BLUE_ADDR_H     0x0212
#define GAIN_BLUE_ADDR_L     0x0213

#define GAIN_RED_ADDR_H      0x0210
#define GAIN_RED_ADDR_L      0x0211

#define GAIN_GREEN2_ADDR_H   0x0214
#define GAIN_GREEN2_ADDR_L   0x0214

USHORT golden_r;
USHORT golden_gr;
USHORT golden_gb;
USHORT golden_b;

USHORT current_r;
USHORT current_gr;
USHORT current_gb;
USHORT current_b;
#endif

//kal_uint32 r_ratio;
//kal_uint32 b_ratio;

kal_uint32    golden_r = 0, golden_gr = 0, golden_gb = 0, golden_b = 0;
kal_uint32    current_r = 0, current_gr = 0, current_gb = 0, current_b = 0;
/*************************************************************************************************
* Function    :  start_read_otp
* Description :  before read otp , set the reading block setting
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  0, reading block setting err
                 1, reading block setting ok
**************************************************************************************************/
#if 0
typedef struct product_time{
    kal_uint32 year;
    kal_uint32 month;
    kal_uint32 day;
    kal_uint32 hour;
    kal_uint32 minute;
}product_time;

typedef struct module_info{
    kal_uint32 supplied_id;
    kal_uint32 sensor_id;
    kal_uint32 len_id;
    kal_uint32 IR_BR_id;
    kal_uint32 color_temp_id;
    kal_uint32 product_id;
    struct product_time time;
}module_info;
struct Darling_S5K5E8_otp_struct {
    kal_uint32 flag;
    kal_uint32 MID;
    kal_uint32 LID;
    kal_uint32 PID;
    kal_uint32 group;
} Darling_S5K5E8_OTP;
struct Darling_S5K5E8_otp_struct Darling_S5K5E8_OTP_data =
{
    .flag = 0,
    .MID = 0,
    .LID = 0,
    .PID = 0,
    .group = 0,
};
#endif
static void otp_read_enable(BYTE zone)
{
    write_cmos_sensor_8(0x0a00,0x04);
    write_cmos_sensor_8(0x0a02,zone);
    write_cmos_sensor_8(0x0a00,0x01);
    mdelay(5);
}
static void otp_read_disable(void)
{
    write_cmos_sensor_8(0x0a00,0x04);
    write_cmos_sensor_8(0x0a00,0x00);
}

/* ************************************************************************************************
 * * Function    :  get_otp_module_id
 * * Description :  get otp MID value
 * * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
 * * Return      :  [BYTE] 0 : OTP data fail
 *                  other value : module ID data , TRULY ID is 0x0001
 * **************************************************************************************************/
BYTE get_otp_module_id(BYTE zone)
{
    BYTE module_id = 0;
    otp_read_enable(zone);
    module_id = read_cmos_sensor(0x0a05);
    LOG_INF("OTP_Module ID: 0x%02x.\n",module_id);
    return module_id;
}

bool get_module_info(BYTE zone)
{
    BYTE group_id;
    kal_uint8 i;
    BYTE data[11];
    BYTE temph;
    BYTE templ;
    kal_uint16 checksum = 0;
    kal_uint16 datasum = 0;
    otp_read_enable(zone);
    group_id = read_cmos_sensor(0x0a04);
    if ((group_id & 0xc0) == 0x40)
    {
	kal_uint16 addr[11] ={0x0a05,0x0a06,0x0a07,0x0a08,0x0a09,0x0a0a,0x0a0b,0x0a0c,0x0a0d,0x0a0e,0x0a0f};
        for(i=0;i<11;i++)
        {
            data[i] = read_cmos_sensor(addr[i]);
	    datasum += data[i];
        }

        datasum += group_id;
	temph = read_cmos_sensor(0x0a30);
        templ = read_cmos_sensor(0x0a31);
        checksum = templ + (temph << 8);

    }else if ((group_id & 0x30) == 0x10)
    {
	kal_uint16 addr[11] ={0x0a12,0x0a13,0x0a14,0x0a15,0x0a16,0x0a17,0x0a18,0x0a19,0x0a1a,0x0a1b,0x0a1c};
	for(i=0;i<11;i++)
	{
	    data[i] = read_cmos_sensor(addr[i]);
	    datasum += data[i];
	}
	datasum += group_id;
        temph = read_cmos_sensor(0x0a32);
        templ = read_cmos_sensor(0x0a33);
        checksum = templ + (temph << 8);
    }else{
        LOG_ERR("get module info group is null \n");
        return 0;
    }
    otp_read_disable();

    if (checksum == datasum){
        LOG_ERR("checksum success\n");
        return 1;
    }
    else{
        LOG_ERR("checksum failed\n");
        return 0;
    }
   /*
    md_info->supplied_id = data[0];
    md_info->sensor_id = data[1];
    md_info->len_id = data[2];
    md_info->IR_BR_id = data[3];
    md_info->color_temp_id = data[4];
    md_info->product_id = data[5];
    md_info->time.year = data[6];
    md_info->time.month = data[7];
    md_info->time.day = data[8];
    md_info->time.hour = data[9];
    md_info->time.minute = data[10];
    */
}

#if 0
/*************************************************************************************************
* Function    :  get_otp_date
* Description :  get otp date value
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
**************************************************************************************************/
bool get_otp_date(BYTE zone)
{
    BYTE year  = 0;
    BYTE month = 0;
    BYTE day   = 0;
    if(!start_read_otp(zone))
    {
        LOG_INF("Start read Page %d Fail!\n", zone);
        return 0;
    }
    year  = read_cmos_sensor(0x3B06);
    month = read_cmos_sensor(0x3B07);
    day   = read_cmos_sensor(0x3B08);
    LOG_INF("OTP date=%02d.%02d.%02d", year,month,day);
    return 1;
}


/*************************************************************************************************
* Function    :  get_otp_lens_id
* Description :  get otp LENS_ID value
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail
                 other value : LENS ID data
**************************************************************************************************/
BYTE get_otp_lens_id(BYTE zone)
{
    BYTE lens_id = 0;
    if(!start_read_otp(zone))
    {
        LOG_INF("Start read Page %d Fail!\n", zone);
        return 0;
    }
    lens_id = read_cmos_sensor(0x3B09);
    LOG_INF("OTP_Lens ID: 0x%02x.\n",lens_id);
    return lens_id;
}
#endif

/*************************************************************************************************
* Function    :  get_otp_wb
* Description :  Get WB data
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
**************************************************************************************************/
static bool get_otp_wb(BYTE zone)
{
    BYTE temph = 0;
    BYTE templ = 0;
    BYTE group_id = 0;
    kal_uint16 checksum = 0;
    kal_uint16 datasum = 0;
    golden_r = 0, golden_gr = 0, golden_gb = 0, golden_b = 0;
    current_r = 0, current_gr = 0, current_gb = 0, current_b = 0;

    otp_read_enable(zone);

    group_id = read_cmos_sensor(0x0a04);
    if ((group_id & 0xc0) == 0x40)
    {
	    temph = read_cmos_sensor(0x0a05);
	    templ = read_cmos_sensor(0x0a06);
	    current_r  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a07);
	    templ = read_cmos_sensor(0x0a08);
	    current_b  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a09);
	    templ = read_cmos_sensor(0x0a0a);
	    current_gr  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a0b);
	    templ = read_cmos_sensor(0x0a0c);
	    current_gb  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;


	    temph = read_cmos_sensor(0x0a0d);
	    templ = read_cmos_sensor(0x0a0e);
	    golden_r  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a0f);
	    templ = read_cmos_sensor(0x0a10);
	    golden_b  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a11);
	    templ = read_cmos_sensor(0x0a12);
	    golden_gr  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a13);
	    templ = read_cmos_sensor(0x0a14);
	    golden_gb  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;
	    datasum +=group_id;
            temph = read_cmos_sensor(0x0a40);
            templ = read_cmos_sensor(0x0a41);
            checksum = templ + (temph << 8);


    }else if ((group_id & 0x30) == 0x10)
    {
	    temph = read_cmos_sensor(0x0a17);
	    templ = read_cmos_sensor(0x0a18);
	    current_r  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);

	    temph = read_cmos_sensor(0x0a19);
	    templ = read_cmos_sensor(0x0a1a);
	    current_b  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a1b);
	    templ = read_cmos_sensor(0x0a1c);
	    current_gr  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a1d);
	    templ = read_cmos_sensor(0x0a1e);
	    current_gb  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a1f);
            templ = read_cmos_sensor(0x0a20);
	    golden_r  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a21);
	    templ = read_cmos_sensor(0x0a22);
	    golden_b  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a23);
	    templ = read_cmos_sensor(0x0a24);
	    golden_gr  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;

	    temph = read_cmos_sensor(0x0a25);
	    templ = read_cmos_sensor(0x0a26);
	    golden_gb  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
	    datasum +=temph;
	    datasum +=templ;
	    datasum +=group_id;
            temph = read_cmos_sensor(0x0a40);
            templ = read_cmos_sensor(0x0a41);
            checksum = templ + (temph << 8);
    }else{
        LOG_ERR("otp is null\n");
	return 0;
    }
    otp_read_disable();
    if (checksum == datasum){
        LOG_ERR("checksum success\n");
        return 1;
    }
    else{
        LOG_ERR("checksum failed\n");
        return 0;
    }
}


/*************************************************************************************************
* Function    :  otp_wb_update
* Description :  Update WB correction
* Return      :  [bool] 0 : OTP data fail
                        1 : otp_WB update success
**************************************************************************************************/
static bool otp_wb_update(BYTE zone)
{
    kal_uint32 R_gain,B_gain,Gb_gain,Gr_gain,Base_gain;

    if(!get_otp_wb(0x0e))  // get wb data from otp
    {
        LOG_ERR("Get OTP WB data Err!\n");
        return 0;
    }

    R_gain = ((golden_r * current_gr)*1000) /( golden_gr * current_r );
    B_gain = ((golden_b * current_gb)*1000) /( golden_gb * current_b );
    Gb_gain = ((golden_gb * current_gr)*1000) /( golden_gr * current_gb);
    Gr_gain = 1000;
    Base_gain = R_gain;
    if(Base_gain>B_gain) Base_gain=B_gain;
    if(Base_gain>Gb_gain) Base_gain=Gb_gain;
    if(Base_gain>Gr_gain) Base_gain=Gr_gain;
    R_gain = 0x100 * R_gain /Base_gain;
    B_gain = 0x100 * B_gain /Base_gain;
    Gb_gain = 0x100 * Gb_gain /Base_gain;
    Gr_gain = 0x100 * Gr_gain /Base_gain;
    LOG_ERR("R_gain = %d, B_gain = %d,Gb_gain = %d,Gr_gain = %d",R_gain,B_gain,Gb_gain,Gr_gain);
    /*
    //get the original data
    original_dat[0] = read_cmos_sensor(0x020e);
    original_dat[1] = read_cmos_sensor(0x020f);
    original_dat[2] = read_cmos_sensor(0x0210);
    original_dat[3] = read_cmos_sensor(0x0211);
    original_dat[4] = read_cmos_sensor(0x0212);
    original_dat[5] = read_cmos_sensor(0x0213);
    original_dat[6] = read_cmos_sensor(0x0214);
    original_dat[7] = read_cmos_sensor(0x0215);
    */
    if(Gr_gain>0x100){
        write_cmos_sensor_8(0x020e,Gr_gain>>8);
        write_cmos_sensor_8(0x020f,Gr_gain&0xff);
    }
    if(R_gain>0x100){
        write_cmos_sensor_8(0x0210,R_gain>>8);
        write_cmos_sensor_8(0x0211,R_gain&0xff);
    }
    if(B_gain>0x100){
        write_cmos_sensor_8(0x0212,B_gain>>8);
	write_cmos_sensor_8(0x0213,B_gain&0xff);
     }
    if(Gb_gain>0x100){
	write_cmos_sensor_8(0x0214,Gb_gain>>8);
	write_cmos_sensor_8(0x0215,Gb_gain&0xff);
     }
    //wb_gain_set();

    LOG_INF("OTP WB update finished! \n");

    return 1;
}

/*************************************************************************************************
* Function    :  otp_update()
* Description :  update otp data from otp , it otp data is valid,
                 it include get ID and WB update function
* Return      :  [bool] 0 : update fail
                        1 : update success
**************************************************************************************************/
bool otp_update(void)
{
    bool flag;

    flag = get_module_info(0x04);
    if (!flag){
        LOG_ERR("get module info failed \n");
	return 0;
    }
    get_otp_module_id(0x04);
/*
    MID =     get_otp_module_id(zone);
#ifdef DEBUG_IMX135_OTP
    get_otp_lens_id(zone);
    get_otp_vcm_id(zone);
#endif

    if(MID != TRULY_ID) //Select
    {
        LOG_INF("No Truly Module !!!!\n");
        return 0;
    }
*/
    flag = otp_wb_update(0x0e);
    if (!flag){
        LOG_ERR("update otp failed \n");
	return 0;
    }
    write_cmos_sensor(0x3400, 0x00);//enable lsc
    return 1;
}
