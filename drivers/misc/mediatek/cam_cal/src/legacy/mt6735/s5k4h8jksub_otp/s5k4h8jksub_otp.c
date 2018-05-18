/*
 * Driver for CAM_CAL
 *
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "s5k4h8jksub_otp.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define PFX "S5K4H8JKSUB_OTP_FMT"


//#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#ifdef _LOG_USER_
#define CAM_CALINF(format, args...)
#define CAM_CALDB(format, args...)
#define CAM_CALERR(format, args...)
#else
#define CAM_CALINF(format, args...)    pr_info(PFX "[%s] " format, __func__, ##args)
#define CAM_CALDB(format, args...)     pr_debug(PFX "[%s] " format, __func__, ##args)
#define CAM_CALERR(format, args...)    printk(KERN_ERR format, ##args)
#endif
#else
#define CAM_CALINF(x,...)
#define CAM_CALDB(x,...)
#define CAM_CALERR(x, ...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV_S5K4H8JKSUB"
#define CAM_CAL_I2C_GROUP_ID 0
#define IMGSENSOR_DRVNAME "S5K4H8JKSUB"
/*******************************************************************************
*
********************************************************************************/
//static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, S5K4H8JKSUB_OTP_DEVICE_ID>>1)};

//static struct i2c_client * g_pstI2Cclient = NULL;

static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;


static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;

#define S5K4H8JKSUB_OTP_INFOFLAG_ADDR           0x0A04
#define S5K4H8JKSUB_OTP_INFO_ADDR               0x0A05
#define S5K4H8JKSUB_OTP_VCMFLAG_ADDR            0x0A38
//#define S5K4H8JK_OTP_VCM_ADDR                0x0A39

#define MAX_OTP_SIZE 16
static int s5k4h8jksub_otp_read = 0;
typedef unsigned short kal_uint16;
typedef struct {
    u8   infoflag;//valid info&awb group
    u8   module_integrator_id;
    u8   R_G_H;
    u8   R_G_L;
    u8   B_G_H;
    u8   B_G_L;
    u8   Gb_Gr_H;
    u8   Gb_Gr_L;
	u8   lens_vcm_id;
	u8   vcmflag;
    u8   vcm_inf_H;
	u8   vcm_inf_L;
    u8   vcm_mac_H;
	u8   vcm_mac_L;
}S5K4H8JKSUB_OTP_DATA;

S5K4H8JKSUB_OTP_DATA s5k4h8jksub_otp_data = {0,};


extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
//extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iWriteReg_otp(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId , char * sensorname );

//#define write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, S5K4H8JKSUB_OTP_DEVICE_ID)//add by hhl
#define write_cmos_sensor(addr, para) iWriteReg_otp((u16) addr , (u32) para , 1, S5K4H8JKSUB_OTP_DEVICE_ID, IMGSENSOR_DRVNAME)//add by yjy

static u8 read_cmos_sensor(kal_uint16 addr)
{
    u8  data = 0;
	//char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	if( iReadReg(addr, &data, S5K4H8JKSUB_OTP_DEVICE_ID) )
    {
        CAM_CALDB("[s5k4h8jksub_otp]read_cmos_sensor i2c read error\n");
        return 0;
    }//0 for good
    CAM_CALDB("[s5k4h8jksub_otp]read_cmos_sensor addr=0x%x, data=0x%x\n", addr, data);
    return data;
}

int read_s5k4h8jksub_otp_data(void){
	int i = 0;
	int offset = 0;
	//int val = 0;

    CAM_CALDB("[s5k4h8jksub_otp]read_s5k4h8jksub_otp_data start!!!\n");
    memset(&s5k4h8jksub_otp_data,0,sizeof(S5K4H8JKSUB_OTP_DATA));
    write_cmos_sensor(0x0100,0x01);
    mdelay(10);

    for (i=0;i<64;i++)
    {
        write_cmos_sensor(0x0A04+i,0x00);
    }

    write_cmos_sensor(0x0A02,0x0F); //set page 15
    write_cmos_sensor(0x0A00,0x01); 
    mdelay(20);

    s5k4h8jksub_otp_data.infoflag = read_cmos_sensor(S5K4H8JKSUB_OTP_INFOFLAG_ADDR);
    CAM_CALERR("[s5k4h8jksub_otp]read_s5k4h8jksub_otp infoflag=0x%x\n", s5k4h8jksub_otp_data.infoflag);
    if( (s5k4h8jksub_otp_data.infoflag>>4 & 0x0C) == 0x04 )
    {
        offset = 0;
    }
    else if( (s5k4h8jksub_otp_data.infoflag>>4 & 0x03) == 0x01 )
    {
        offset = 0x020;
    }
    else
    {
        CAM_CALERR("[s5k4h8jksub_otp]read_s5k4h8jksub_otp_data fail empty!!!\n");
        return -1;
    }
    s5k4h8jksub_otp_data.module_integrator_id = read_cmos_sensor(S5K4H8JKSUB_OTP_INFO_ADDR+offset);
    s5k4h8jksub_otp_data.lens_vcm_id = read_cmos_sensor(S5K4H8JKSUB_OTP_INFO_ADDR+offset+0x4);

    s5k4h8jksub_otp_data.R_G_H = read_cmos_sensor(S5K4H8JKSUB_OTP_INFO_ADDR+offset+0x7);
    s5k4h8jksub_otp_data.R_G_L = read_cmos_sensor(S5K4H8JKSUB_OTP_INFO_ADDR+offset+0x8);
    s5k4h8jksub_otp_data.B_G_H = read_cmos_sensor(S5K4H8JKSUB_OTP_INFO_ADDR+offset+0x9);
    s5k4h8jksub_otp_data.B_G_L = read_cmos_sensor(S5K4H8JKSUB_OTP_INFO_ADDR+offset+0xa);
    s5k4h8jksub_otp_data.Gb_Gr_H = read_cmos_sensor(S5K4H8JKSUB_OTP_INFO_ADDR+offset+0xb);
    s5k4h8jksub_otp_data.Gb_Gr_L = read_cmos_sensor(S5K4H8JKSUB_OTP_INFO_ADDR+offset+0xc);
#if 1
    s5k4h8jksub_otp_data.vcmflag = read_cmos_sensor(S5K4H8JKSUB_OTP_VCMFLAG_ADDR);
    CAM_CALERR("[s5k4h8jksub_otp]read_s5k4h8jksub_otp vcmflag=0x%x\n", s5k4h8jksub_otp_data.vcmflag);
    if( (s5k4h8jksub_otp_data.vcmflag & 0xc0) == 0x40 )
    {
        offset = 0x01;
    }
    else if( (s5k4h8jksub_otp_data.vcmflag & 0x30) == 0x10 )
    {
        offset = 0x05;
    }
    else
    {
        CAM_CALERR("[s5k4h8jksub_otp]read_s5k4h8jksub_otp_data fail vcm empty!!!\n");
        return -1;
    }

	
	s5k4h8jksub_otp_data.vcm_inf_H = read_cmos_sensor(S5K4H8JKSUB_OTP_VCMFLAG_ADDR+offset+0);
	s5k4h8jksub_otp_data.vcm_inf_L = read_cmos_sensor(S5K4H8JKSUB_OTP_VCMFLAG_ADDR+offset+1);
    s5k4h8jksub_otp_data.vcm_mac_H = read_cmos_sensor(S5K4H8JKSUB_OTP_VCMFLAG_ADDR+offset+2);
	s5k4h8jksub_otp_data.vcm_mac_L = read_cmos_sensor(S5K4H8JKSUB_OTP_VCMFLAG_ADDR+offset+3);
	//CAM_CALERR("[s5k4h8jk_otp]read_s5k4h8jk_otp vcmflag=0x%x\n", s5k4h8jk_otp_data.vcmflag);
	//	CAM_CALERR("\n[s5k4h8jksubotp]%s: %s: %d,vcm_inf_H=0x%x \n",__FILE__,__func__,__LINE__, s5k4h8jksub_otp_data.vcm_inf_H);
	//	CAM_CALERR("\n[s5k4h8jksubotp]%s: %s: %d,vcm_inf_L=0x%x \n",__FILE__,__func__,__LINE__,  s5k4h8jksub_otp_data.vcm_inf_L );
	//	CAM_CALERR("\n[s5k4h8jksubotp]%s: %s: %d,vcm_mac_H=0x%x\n",__FILE__,__func__,__LINE__, s5k4h8jksub_otp_data.vcm_mac_H );
	//	CAM_CALERR("\n[s5k4h8jksubotp]%s: %s: %d,vcm_mac_L=0x%x \n",__FILE__,__func__,__LINE__, s5k4h8jksub_otp_data.vcm_mac_L );
	
	#endif
    write_cmos_sensor(0x0A00,0x00); 


    spin_lock(&g_CAM_CALLock);
    s5k4h8jksub_otp_read = 1;
    spin_unlock(&g_CAM_CALLock);

    CAM_CALDB("[s5k4h8jksub_otp]read_s5k4h8jksub_otp_data end!!!\n");
	return 0;
}

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
	err |= get_user(p, (compat_uptr_t *)&data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long s5k4h8jksubotp_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	CAM_CALDB("[s5k4h8jksub_otp] s5k4h8jksub_otp_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("[s5k4h8jksub_otp] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}


#endif


static int selective_read_region(u32 offset, BYTE* data, u32 size)
{
    if( !s5k4h8jksub_otp_read )
    {
        CAM_CALDB("[s5k4h8jksub_otp]selective_read_region first read data!!!\n");
        if( 0 != read_s5k4h8jksub_otp_data() )
        {
            CAM_CALDB("[s5k4h8jksub_otp]selective_read_region failed!!!\n");
            return -1;
        }
    }
    CAM_CALDB("[s5k4h8jksub_otp]selective_read_region offset =%x size %d \n", offset,size);
    if( offset == S5K4H8JKSUB_OTP_INFO_OFFSET+1 && size == 1 )
    {
        CAM_CALDB("[s5k4h8jksub_otp]read module_integrator_id=%d \n", s5k4h8jksub_otp_data.module_integrator_id);
        *data = s5k4h8jksub_otp_data.module_integrator_id;
    }
	else if( offset == S5K4H8JKSUB_OTP_VCM_OFFSET && size == 1 )
	{
	        *data = 0xD9;
	}
    //else if( offset == S5K4H8_OTP_VCM_OFFSET && size == S5K4H8_OTP_VCM_SIZE )
    else if( offset == S5K4H8JKSUB_OTP_VCM_OFFSET && size == 4 )
	{
        CAM_CALDB("[s5k4h8jk_otp]selective_read_region vcm_inf_H=%d, vcm_inf_L=%d, vcm_mac_H=%d, vcm_mac_L=%d \n", s5k4h8jksub_otp_data.vcm_inf_H, s5k4h8jksub_otp_data.vcm_inf_L, s5k4h8jksub_otp_data.vcm_mac_H, s5k4h8jksub_otp_data.vcm_mac_L );
        /*
		memcpy(data, &s5k4h8jk_otp_data.vcm_inf_H, 1);
        memcpy(data+1, &s5k4h8jk_otp_data.vcm_inf_L, 1);
		memcpy(data+2, &s5k4h8jk_otp_data.vcm_mac_H, 1);
        memcpy(data+3, &s5k4h8jk_otp_data.vcm_mac_L, 1);
		*/
        data[0] = s5k4h8jksub_otp_data.vcm_mac_L;
        data[1] = s5k4h8jksub_otp_data.vcm_mac_H;
        data[2] = s5k4h8jksub_otp_data.vcm_inf_L;
        data[3] = s5k4h8jksub_otp_data.vcm_inf_H;
    }
    else if( offset == S5K4H8JKSUB_OTP_INFO_OFFSET && size == sizeof(S5K4H8JKSUB_OTP_DATA))
    {
        CAM_CALDB("[s5k4h8jksub_otp]read otp info\n");
        memcpy(data, &s5k4h8jksub_otp_data, size);
    }
    else if(size > sizeof(S5K4H8JKSUB_OTP_DATA))
    {
        CAM_CALDB("[s5k4h8jksub_otp]selective_read_region read size 0x%x is too big! change to max size:0x%x\n", 
                  size, (u32)sizeof(S5K4H8JKSUB_OTP_DATA));
        size = sizeof(S5K4H8JKSUB_OTP_DATA);
        memcpy(data, &s5k4h8jksub_otp_data, size);
    }
    else
    {
        CAM_CALDB("[s5k4h8jksub_otp]selective_read_region no offset and size maching!!!\n");
        return -1;
    }

    return 0;
}

int s5k4h8jksub_read_otp(u16 offset, u8* data, u32 size)
{
    return selective_read_region(offset, data, size);
}

/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pu1Params = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALERR("[s5k4h8jksub_otp]ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALERR("[s5k4h8jksub_otp]ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALERR("[s5k4h8jksub_otp]ioctl allocate mem failed\n");
        return -ENOMEM;
    }


    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALERR("[s5k4h8jksub_otp]ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            CAM_CALDB("[s5k4h8jksub_otp]Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = 0;//iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("[s5k4h8jksub_otp]Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[s5k4h8jksub_otp]Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, ptempbuf->u4Length);

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("[s5k4h8jksub_otp]Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif

            break;
        default :
      	     CAM_CALINF("[CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALERR("[s5k4h8jksub_otp]ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[s5k4h8jksub_otp]CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("[s5k4h8jksub_otp]Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);
    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
    .compat_ioctl = s5k4h8jksubotp_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
//#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1

inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;
    CAM_CALDB("[s5k4h8jksub_otp]RegisterCAM_CALCharDrv\n");
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALERR("[s5k4h8jksub_otp]Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALERR("[s5k4h8jksub_otp]Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALERR("[s5k4h8jksub_otp]Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR("[s5k4h8jksub_otp]Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv_S5K4H8JKSUB");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("[s5k4h8jksub_otp]Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{

    return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    //i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_init(void)
{
    int i4RetValue = 0;
    CAM_CALDB("[s5k4h8jksub_otp]CAM_CAL_i2C_init\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   CAM_CALDB("[s5k4h8jksub_otp]register char device failed!\n");
	   return i4RetValue;
	}
	CAM_CALDB("[s5k4h8jksub_otp]Attached!! \n");

  //  i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALERR("[s5k4h8jksub_otp]failed to register s5k4h8jksub_otp driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALERR("[s5k4h8jksub_otp]failed to register s5k4h8jksub_otp driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit CAM_CAL_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_init);
module_exit(CAM_CAL_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


