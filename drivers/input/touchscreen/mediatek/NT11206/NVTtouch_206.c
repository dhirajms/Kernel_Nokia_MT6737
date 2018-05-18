/* drivers/input/touchscreen/nt11206/NVTtouch_206.c
 *
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 *
 * $Revision: 4541 $
 * $Date: 2016-05-10 14:56:50 +0800 (¶g¤G, 10 ¤­¤ë 2016) $
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
#ifndef CONFIG_MTK_LEGACY
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#include "tpd.h"
#include "NVTtouch_206.h"
#include <linux/input/mt.h>
#include <linux/wakelock.h>
#include <linux/version.h>
#ifdef CONFIG_MTK_LEGACY
#ifndef TPD_NO_GPIO
#include "cust_gpio_usage.h" 
#endif
#endif

#if I2C_DMA_SUPPORT
#include <linux/dma-mapping.h>
#endif

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
#endif

unsigned int touch_irq = 0;
struct nvt_ts_data *ts;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif



#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array_1[TOUCH_KEY_NUM] = {
    KEY_BACK, 
	KEY_HOMEPAGE,
	KEY_APPSELECT,
	//KEY_SEND
};
#endif

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
    KEY_POWER,  //GESTURE_WORD_C
    KEY_POWER,  //GESTURE_WORD_W
    KEY_POWER,  //GESTURE_WORD_V
    KEY_POWER,  //GESTURE_DOUBLE_CLICK
    KEY_POWER,  //GESTURE_WORD_Z
    KEY_POWER,  //GESTURE_WORD_M
    KEY_POWER,  //GESTURE_WORD_O
    KEY_POWER,  //GESTURE_WORD_e
    KEY_POWER,  //GESTURE_WORD_S
    KEY_POWER,  //GESTURE_SLIDE_UP
    KEY_POWER,  //GESTURE_SLIDE_DOWN
    KEY_POWER,  //GESTURE_SLIDE_LEFT
    KEY_POWER,  //GESTURE_SLIDE_RIGHT
};
#endif

static int tpd_flag = 0; 
int tpd_halt_1 = 0;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
extern struct tpd_device *tpd;

#if I2C_DMA_SUPPORT
s32 i2c_dma_write_1(struct i2c_client *client, u16 addr,uint8_t offset, uint8_t *rxbuf, uint16_t len);
s32 i2c_dma_read_1(struct i2c_client *client, u16 addr,uint8_t offset, uint8_t *rxbuf, uint16_t len);

static uint8_t *gpDMABuf_va = NULL;
//static uint32_t  *gpDMABuf_pa = 0;
static dma_addr_t  gpDMABuf_pa;	
#endif

static irqreturn_t tpd_eint_handler(int irq, void *dev_id);
static int touch_event_handler(void *unused);

static int nvt_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int nvt_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int nvt_i2c_remove(struct i2c_client *client);

extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);

#ifdef CONFIG_MTK_LEGACY
#ifndef MT6572
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif
#endif
//extern s32 nvt_test_sysfs_init(void);
extern void nvt_test_sysfs_deinit(void);

struct i2c_client *i2c_client_point_1 = NULL;



static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

static unsigned short forces[] = {
	0, 
	I2C_HW_Address, 
	I2C_CLIENT_END, 
	I2C_CLIENT_END 
};

static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};

static struct i2c_driver nvt_i2c_driver =
{
    .probe = nvt_i2c_probe,
    .remove = nvt_i2c_remove,
    .detect = nvt_i2c_detect,
    .driver.name = NVT_I2C_NAME,
	.driver = {
		   .name = NVT_I2C_NAME,
		   .of_match_table = tpd_of_match,
		   },
    .id_table = nvt_ts_id,
	.address_list = (const unsigned short *)forces,
};



#if I2C_DMA_SUPPORT
s32 i2c_dma_read_1(struct i2c_client *client, uint16_t addr, uint8_t offset, uint8_t *rxbuf, uint16_t len)
{
    int ret;
    uint8_t retry = 0;
	uint8_t buf[2] = {offset,0};

    struct i2c_msg msg[2] =
    {
        {
            .addr = (addr & I2C_MASK_FLAG),
            .flags = 0,
            .buf = buf,
            .len = 1,
            .timing = I2C_MASTER_CLOCK
        },
        {
            .addr = (addr & I2C_MASK_FLAG),
            .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            .buf = (uint8_t*)gpDMABuf_pa,     
            .len = len,
            .timing = I2C_MASTER_CLOCK
        },
    };
    
    if (rxbuf == NULL)
        return -1;

    //dev_info(&ts->client->dev,"dma i2c read: 0x%04X, %d bytes(s)", addr, len);
    for (retry = 0; retry < 20; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg[0], 2);
        if (ret < 0)
        {
            continue;
        }
        memcpy(rxbuf, gpDMABuf_va, len);
        return 0;
    }
	
    dev_err(&ts->client->dev,"Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
    return ret;
}

s32 i2c_dma_write_1(struct i2c_client *client, uint16_t addr, uint8_t offset, uint8_t *txbuf, uint16_t len)
{
    int ret;
    s32 retry = 0;
    uint8_t *wr_buf = gpDMABuf_va;
    
    struct i2c_msg msg =
    {
        .addr = (addr & I2C_MASK_FLAG),
        .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
        .flags = 0,
        .buf = (uint8_t*)gpDMABuf_pa,
        .len = 1 + len,
        .timing = I2C_MASTER_CLOCK
    };
    
    wr_buf[0] = offset;

    if (txbuf == NULL)
        return -1;
    
    //dev_info(&client->dev,"dma i2c write: 0x%04X, %d bytes(s)", addr, len);
    memcpy(wr_buf+1, txbuf, len);
    for (retry = 0; retry < 20; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret < 0)
        {
            continue;
        }
        return 0;
    }
    dev_err(&client->dev,"Dma I2C Write Error: 0x%04X, %d byte(s), err-code: %d", offset, len, ret);
    return ret;
}

s32 i2c_read_bytes_dma_1(struct i2c_client *client, u16 addr, uint8_t offset, uint8_t *rxbuf, uint16_t len)
{
    uint16_t left = len;
    uint16_t read_len = 0;
    uint8_t *rd_buf = rxbuf;
    s32 ret = 0;    
    
    //dev_info(&client->dev,"Read bytes dma: 0x%04X, %d byte(s)", offset, len);
    while (left > 0)
    {
        if (left > DMA_MAX_TRANSACTION_LENGTH)
        {
            read_len = DMA_MAX_TRANSACTION_LENGTH;
        }
        else
        {
            read_len = left;
        }
        ret = i2c_dma_read_1(client, addr, offset, rd_buf, read_len);
        if (ret < 0)
        {
            dev_err(&client->dev,"dma read failed");
            return -1;
        }
        
        left -= read_len;
        offset += read_len;
        rd_buf += read_len;
    }
    return 0;
}

s32 i2c_write_bytes_dma_1(struct i2c_client *client, u16 addr, uint8_t offset, uint8_t *txbuf, uint16_t len)
{
    s32 ret = 0;
    s32 write_len = 0;
    s32 left = len;
    uint8_t *wr_buf = txbuf;
    
    //dev_info(&client->dev,"Write bytes dma: 0x%04X, %d byte(s)", offset, len);
    while (left > 0)
    {
        if (left > DMA_MAX_I2C_TRANSFER_SIZE)
        {
            write_len = DMA_MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            write_len = left;
        }
        ret = i2c_dma_write_1(client, addr, offset, wr_buf, write_len);
        
        if (ret < 0)
        {
            dev_err(&client->dev,"dma i2c write failed!");
            return -1;
        }
        
        left -= write_len;
        offset += write_len;
        wr_buf += write_len;
    }
    return 0;
}
#endif

int i2c_read_bytes_non_dma_1(struct i2c_client *client, u16 addr,uint8_t offset, uint8_t *rxbuf, uint16_t len)
{
    uint16_t left = len;
	uint16_t index = 0;
	
	uint8_t buf[2] = {0};
    uint8_t retry;
    struct i2c_msg msg[2] =
    {
        {
            .addr = ((addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            /* .addr = ((addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)), */
            .flags = 0,
            .buf = buf,
            .len = 1,
            .timing = I2C_MASTER_CLOCK
        },
        {
            .addr = ((addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            /* .addr = ((addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)), */
            .flags = I2C_M_RD,
            .timing = I2C_MASTER_CLOCK
        },
    };

    if (rxbuf == NULL)
        return -1;

    //dev_info(&client->dev,"i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
		buf[0] = offset + index;
        msg[1].buf = &rxbuf[index];

        if (left > MAX_TRANSACTION_LENGTH)
        {
            msg[1].len = MAX_TRANSACTION_LENGTH;
            left -= MAX_TRANSACTION_LENGTH;
            index += MAX_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;

        while (i2c_transfer(client->adapter, &msg[0], 2) != 2)
        {
            retry++;

            if (retry == 20)
            {             
                dev_err(&client->dev,"I2C read 0x%X length=%d failed\n", offset + index, len);
                return -1;
            }
        }
    }

    return 0;
}

int i2c_write_bytes_non_dma_1(struct i2c_client *client, u16 addr,uint8_t offset, uint8_t *txbuf, uint16_t len)
{
    uint8_t buf[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 index = 0;
    uint8_t retry = 0;
    struct i2c_msg msg =
    {
        .addr = ((addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
        /* .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)), */
        .flags = 0,
        .buf = buf,
        .timing = I2C_MASTER_CLOCK,
    };
    if (txbuf == NULL)
    	{
        return -1;
		}

    //dev_info(&client->dev,"i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        retry = 0;

		buf[0] = (offset + index) & 0xFF;

        if (left > MAX_I2C_TRANSFER_SIZE)
        {
            memcpy(&buf[1], &txbuf[index], MAX_I2C_TRANSFER_SIZE);
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            index += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy(&buf[1], &txbuf[index], left);
            msg.len = left + 1;
            left = 0;
        }

        //dev_info(&client->dev,"byte left %d index %d\n", left, index);

        while (i2c_transfer(client->adapter, &msg, 1) != 1)
        {
            retry++;
            if (retry == 20)
            {
               dev_err(&client->dev,"I2C write 0x%X length=%d failed\n", offset, len);
               return -1;
            }
        }
    }

    return 0;
}

/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
#if I2C_DMA_SUPPORT
    return i2c_read_bytes_dma_1(client, address, buf[0], &buf[1], len-1);
#else
    return i2c_read_bytes_non_dma_1(client, address, buf[0], &buf[1], len-1);
#endif
}

/*******************************************************
Description:
	Novatek touchscreen i2c dummy read function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
#if 0
int32_t CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address)
{
	struct i2c_msg msg;
	uint8_t buf[8] = {0};
	int32_t ret = -1;
	int32_t retries = 0;

	msg.flags = I2C_M_RD;
	msg.addr  = address;
	msg.len   = 1;
	msg.buf   = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		dev_err(&client->dev, "%s: error, ret=%d\n", __func__, ret);
		ret = -EIO;
	}

	return ret;
}
#endif

int32_t CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address)
{
	uint8_t buf[8] = {0};
	int32_t ret = -1;
	ret = CTP_I2C_READ(client, address, buf, 2);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
#if I2C_DMA_SUPPORT
    return i2c_write_bytes_dma_1(client, address, buf[0], &buf[1], len-1);
#else
    return i2c_write_bytes_non_dma_1(client, address, buf[0], &buf[1], len-1);
#endif

}
/*******************************************************
Description:
	Novatek touchscreen IC hardware reset function.

return:
	n.a.
*******************************************************/
void nvt_hw_reset(void)
{
	//---trigger rst-pin to reset---
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(20);
	tpd_gpio_output(GTP_RST_PORT, 0);
	msleep(5);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(5);
}

/*******************************************************
Description:
	Novatek touchscreen set i2c debounce function.

return:
	n.a.
*******************************************************/
void nvt_set_i2c_debounce(void)
{
	uint8_t buf[8] = {0};
	uint8_t reg1_val = 0;
	uint8_t reg2_val = 0;
	uint32_t retry = 0;

	do {
		msleep(10);

		// set xdata index to 0x1F000
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF0;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		// set i2c debounce 34ns
		buf[0] = 0x15;
		buf[1] = 0x17;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		buf[0] = 0x15;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
		reg1_val = buf[1];

		// set schmitt trigger enable
		buf[0] = 0x3E;
		buf[1] = 0x07;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		buf[0] = 0x3E;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
		reg2_val = buf[1];
	} while (((reg1_val != 0x17) || (reg2_val != 0x07)) && (retry++ < 20));

	if(retry == 20) {
		dev_err(&ts->client->dev,"%s: set i2c debounce failed, reg1_val=0x%02X, reg2_val=0x%02X\n", __func__, reg1_val, reg2_val);
	}
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	uint8_t buf[4]={0};

	//---write i2c cmds to reset idle---
	buf[0]=0x00;
	buf[1]=0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(10);

	nvt_set_i2c_debounce();
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = {0};

	//---write i2c cmds to reset---
	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	// need 35ms delay after bootloader reset
	msleep(35);
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 10;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	for (i = 0; i < retry; i++) {
		//---set xdata index to 0x14700---
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0x47;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---clear fw status---
		buf[0] = 0x51;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		//---read fw status---
		buf[0] = 0x51;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry)
		return -1;
	else
		return 0;
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 10;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	for (i = 0; i < retry; i++) {
		//---set xdata index to 0x14700---
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0x47;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---read fw status---
		buf[0] = 0x51;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry)
		return -1;
	else
		return 0;
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		msleep(10);

		//---read reset state---
		buf[0] = 0x60;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] >= check_reset_state) && (buf[1] < 0xFF)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > 100)) {
			ret = -1;
			dev_err(&ts->client->dev,"%s: error, retry=%d, buf[1]=0x%02X\n", __func__, retry, buf[1]);
			break;
		}
	}

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	//struct i2c_msg msgs[2];
	uint8_t str[64] = {0};
	int32_t ret = -1;
	//int32_t retries = 0;
	int8_t i2c_wr = 0;

	if (count > sizeof(str))
		return -EFAULT;

	if (copy_from_user(str, buff, count))
		return -EFAULT;

	i2c_wr = str[0] >> 7;

	if (i2c_wr == 0) {	//I2C write
	   #if 0
		msgs[0].flags = !I2C_M_RD;
		msgs[0].addr  = str[0] & 0x7F;
		msgs[0].len   = str[1];
		msgs[0].buf   = &str[2];

		while (retries < 20) {
			ret = i2c_transfer(ts->client->adapter, msgs, 1);
			if (ret == 1)
				break;
			else
				dev_err(&ts->client->dev,"%s: error, retries=%d, ret=%d\n", __func__, retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			dev_err(&ts->client->dev, "%s: error, ret = %d\n", __func__, ret);
			return -EIO;
		}

		return ret;
	#endif

    #if I2C_DMA_SUPPORT
		  ret=i2c_write_bytes_dma_1(ts->client,(str[0] & 0x7F),str[2],&str[3],(str[1]-1));
    #else
		  ret=i2c_write_bytes_non_dma_1(ts->client,(str[0] & 0x7F),str[2],&str[3],(str[1]-1));
	#endif
		return ret;
	} else if (i2c_wr == 1) {	//I2C read
     #if 0
		msgs[0].flags = !I2C_M_RD;
		msgs[0].addr  = str[0] & 0x7F;
		msgs[0].len   = 1;
		msgs[0].buf   = &str[2];

		msgs[1].flags = I2C_M_RD;
		msgs[1].addr  = str[0] & 0x7F;
		msgs[1].len   = str[1]-1;
		msgs[1].buf   = &str[3];

		while (retries < 20) {
			ret = i2c_transfer(ts->client->adapter, msgs, 2);
			if (ret == 2)
				break;
			else
				dev_err(&ts->client->dev,"%s: error, retries=%d, ret=%d\n", __func__, retries, ret);

			retries++;
		}

		// copy buff to user if i2c transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count))
				return -EFAULT;
		}

		if (unlikely(retries == 20)) {
			dev_err(&ts->client->dev, "%s: error, ret = %d\n", __func__, ret);
			return -EIO;
		}
    #endif
	    #if I2C_DMA_SUPPORT
		  ret=i2c_read_bytes_dma_1(ts->client,(str[0] & 0x7F),str[2],&str[3],(str[1]-1));
    #else
		  ret=i2c_read_bytes_non_dma_1(ts->client,(str[0] & 0x7F),str[2],&str[3],(str[1]-1));
	#endif
	  if(!ret){
          if(copy_to_user(buff,str,count))
		  return -EFAULT;
	    }
		return ret;
	} else {
		dev_err(&ts->client->dev,"%s: Call error, str[0]=%d\n", __func__, str[0]);
		return -EFAULT;
	}
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&ts->client->dev,"%s: Failed to allocate memory for nvt flash data\n", __func__);
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL,&nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		dev_err(&ts->client->dev,"%s: Failed!\n", __func__);
		return -ENOMEM;
	} else {
		dev_info(&ts->client->dev,"%s: Succeeded!\n", __func__);
		NVT_INFO("%s: Succeeded!\n\n",  __func__);
	}

	dev_info(&ts->client->dev,"============================================================\n");
	dev_info(&ts->client->dev,"Create /proc/NVTflash\n");
	dev_info(&ts->client->dev,"============================================================\n");

	return 0;
}
#endif

#if WAKEUP_GESTURE
#define GESTURE_WORD_C			12
#define GESTURE_WORD_W			13
#define GESTURE_WORD_V			14
#define GESTURE_DOUBLE_CLICK	15
#define GESTURE_WORD_Z			16
#define GESTURE_WORD_M			17
#define GESTURE_WORD_O			18
#define GESTURE_WORD_e			19
#define GESTURE_WORD_S			20
#define GESTURE_SLIDE_UP		21
#define GESTURE_SLIDE_DOWN		22
#define GESTURE_SLIDE_LEFT		23
#define GESTURE_SLIDE_RIGHT		24

static struct wake_lock gestrue_wakelock;
static uint8_t bTouchIsAwake = 1;
static uint8_t bWakeupByGesture = 0;
//static uint8_t bTouchInGesture = 0;

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id)
{
	struct i2c_client *client = ts->client;
	uint32_t keycode = 0;

	dev_info(&client->dev, "gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			dev_info(&client->dev, "Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			dev_info(&client->dev, "Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			dev_info(&client->dev, "Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			dev_info(&client->dev, "Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			dev_info(&client->dev, "Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			dev_info(&client->dev, "Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			dev_info(&client->dev, "Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			dev_info(&client->dev, "Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			dev_info(&client->dev, "Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			dev_info(&client->dev, "Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			dev_info(&client->dev, "Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			dev_info(&client->dev, "Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			dev_info(&client->dev, "Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);

		bWakeupByGesture = 1;
	//	bTouchInGesture = 0;
	}
	msleep(250);
}
#endif

/*******************************************************
Description:
	Novatek touchscreen print chip version function.

return:
	Executive outcomes. 2---succeed. -5---failed.
*******************************************************/
static int32_t nvt_ts_chip_version(void)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	uint8_t cut_number = 0;
	uint8_t cut_version = 0;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	//write i2c index to 0x1F000
	buf[0] = 0xFF;
	buf[1] = 0x01;
	buf[2] = 0xF0;
	ret = CTP_I2C_WRITE(ts->client, 0x01, buf, 3);
	if (ret < 0) {
		dev_err(&ts->client->dev, "%s: write i2c index error!!(%d)\n", __func__, ret);
		return ret;
	}

	//read chip version
	buf[0] = 0x01;
	buf[1] = 0x00;
	ret = CTP_I2C_READ(ts->client, 0x01, buf, 3);
	if (ret < 0) {
		dev_err(&ts->client->dev, "%s: read chip version error!!(%d)\n", __func__, ret);
		return ret;
	}

	// [4:0]: Cut Number
	cut_number = buf[1] & 0x1F;
	// [7:5]: Cut Version
	cut_version = (buf[1] & 0xE0) >> 5;

	dev_info(&ts->client->dev, "chip version: cut number=%d, cut version=%d\n", cut_number, cut_version);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen read chip id function.

return:
	Executive outcomes. 0x26---succeed.
*******************************************************/
static uint8_t nvt_ts_read_chipid(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
    
	NVT_INFO("nvt_ts_read_chipid");
	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_HW_Address);

	// reset idle to keep default addr 0x01 to read chipid
	buf[0] = 0x00;
	buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(10);

	//---Check NT11206 for 5 times---
	for (retry = 5; retry > 0; retry--) {
		//write i2c index to 0x1F000
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF0;
		CTP_I2C_WRITE(ts->client, 0x01, buf, 3);

		//read hw chip id
		buf[0] = 0x00;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, 0x01, buf, 3);
        NVT_INFO("chipid=0x%x",buf[1]);
		if (buf[1] == 0x26)
			break;
	}

	return buf[1];
}


int tpd_power_switch(s32 state)
{
	switch(state)
	{
		case SWITCH_ON:
			NVT_INFO("Power switch on!");
			if (regulator_enable(tpd->reg)){ /*enable regulator*/
				NVT_ERROR("regulator_enable() failed!\n");
				return -1;
			}
			if (regulator_set_voltage(tpd->reg, 3300000, 3300000)){/*set 3.3v*/
				NVT_ERROR("regulator_set_voltage() failed!\n");
				return -1;
			}
			break;
		case SWITCH_OFF:
			NVT_INFO("Power switch off!");
			if (regulator_disable(tpd->reg)){ /*disable regulator*/
				NVT_ERROR("regulator_disable() failed!\n");
				return -1;
			}
			break;
		default:
			NVT_ERROR("Invalid power switch command!");
			return -1;
		break;
	}
	
	return 0;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;

	NVT_INFO("Device Tree tpd_irq_registration!");

	node = of_find_matching_node(node, touch_of_match);

	if (node) {
		/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		touch_irq = irq_of_parse_and_map(node, 0);
		NVT_INFO("touch_irq number %d\n", touch_irq);

		ret = request_irq(touch_irq, tpd_eint_handler, IRQF_TRIGGER_FALLING,
					TPD_DEVICE, NULL);
		if (ret > 0){
			ret = -1;
			NVT_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
		}
	} else {
		NVT_INFO("tpd request_irq can not find touch eint device node!.");
	}

	return ret;
}



static int nvt_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
	int32_t err = 0;
	
	
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif
	
	NVT_INFO("tpd_i2c_probe start...");
	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev, "%s: failed to allocated memory for nvt ts data\n", __func__);
		return -ENOMEM;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);
	
	if (tpd_power_switch(SWITCH_ON) < 0)
	{
		NVT_ERROR("tpd_power_switch error !!!\n");
	}

	#if 0
	// Power and RST Request and Control
	
    i2c_client_point_1 = client;
    ret = tpd_power_on(client);

    if (ret < 0)
    {
        dev_err(&client->dev,"I2C communication ERROR!");
    }
	#endif
            
	//nvt_test_sysfs_init();
    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread))
    {
        err = PTR_ERR(thread);
		NVT_INFO(TPD_DEVICE "failed to create kernel thread: %d\n", err);
    }
    
    //---request INT-pin---
	tpd_gpio_as_int(GTP_INT_PORT);
    msleep(50);
   
	nvt_hw_reset();//
	//msleep(500);

	//---check i2c func.---
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip id---
	ret = nvt_ts_read_chipid();
	if (ret != 0x26) {
		NVT_INFO("nvt_ts_read_chipid is not 0x26. ret=0x%02X\n", ret);
		dev_err(&client->dev, "nvt_ts_read_chipid is not 0x26. ret=0x%02X\n", ret);
		ret = -EINVAL;
		goto err_chipid_failed;
	}
	

	//---check chip version---
	ret = nvt_ts_chip_version();

	mutex_init(&ts->lock);

	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		dev_err(&client->dev, "%s: allocate input device failed\n", __func__);
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if (!nvt_fwu_wq) {
		dev_err(&client->dev, "%s: nvt_fwu_wq create workqueue failed\n", __func__);
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(4000));
#endif

	ts->abs_x_max = TOUCH_MAX_WIDTH;
	ts->abs_y_max = TOUCH_MAX_HEIGHT;
	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#else
	input_mt_init_slots(ts->input_dev, ts->max_touch_num);
#endif
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);    //pressure = 255

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif
#endif

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < TOUCH_KEY_NUM; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array_1[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < (sizeof(gesture_key_array) / sizeof(gesture_key_array[0])); retry++) {
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	}
	wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;


	//---register input device---
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev, "register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}
	
	#if 0
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, INT_TRIGGER_TYPE, tpd_eint_handler, 1);
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	#endif 
	
	ret = tpd_irq_registration();
	if(ret != 0){
		NVT_ERROR("tpd register irq failed!\n");
		goto err_int_request_failed;
	}
	
	mutex_lock(&ts->lock);
	nvt_hw_reset();
	msleep(5);
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	mutex_unlock(&ts->lock);
	
	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		dev_err(&client->dev, "nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		dev_err(&client->dev, "nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		dev_err(&client->dev, "nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

	dev_info(&client->dev, "%s: finished\n", __func__);
	NVT_INFO("%s: finished\n",	__func__);

    tpd_load_status = 1;
	return 0;

err_init_NVT_ts:
	free_irq(touch_irq,ts);
err_int_request_failed:
err_input_register_device_failed:
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	mutex_destroy(&ts->lock);
err_chipid_failed:
err_check_functionality_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;	
}

static int nvt_i2c_remove(struct i2c_client *client)
{
	mutex_destroy(&ts->lock);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

    return 0;
}

static int nvt_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strncpy(info->type, "mtk-tpd", sizeof(info->type));
	return 0;
}

static irqreturn_t tpd_eint_handler(int irq, void *dev_id)
{
    TPD_DEBUG_PRINT_INT;
    
    tpd_flag = 1;
    
    wake_up_interruptible(&waiter);
	
	return IRQ_HANDLED;
}


static int touch_event_handler(void *unused)
{
    struct sched_param param = { .sched_priority = 4 };	/*RTPM_PRIO_TPD*/
	struct i2c_client *client = ts->client;

	int32_t ret = -1;
	uint8_t point_data[64] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;

	uint8_t point_data1[64] = {0};
	
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};

	int32_t i = 0;
	int32_t finger_cnt = 0;

    sched_setscheduler(current, SCHED_RR, &param);
    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
        
        while (tpd_halt_1)
        {
        #if WAKEUP_GESTURE
            //if (bTouchInGesture == 1)
            if (bTouchIsAwake== 0)
            {
                break;
            }
        #endif
            tpd_flag = 0;
            msleep(20);
        }

        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        TPD_DEBUG_SET_TIME;
        set_current_state(TASK_RUNNING);

		mutex_lock(&ts->lock);
		memset(point_data,0,64);
		ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, 62 + 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s: CTP_I2C_READ failed.(%d)\n", __func__, ret);
			goto XFER_ERROR;
		}

		#if 0
		//--- dump I2C buf ---
		for (i = 0; i < 10; i++) {
			NVT_INFO("%02X %02X %02X %02X %02X %02X  ", point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
		}
		NVT_INFO("\n");
		#endif
		
		finger_cnt = 0;
		input_id = (uint8_t)(point_data[1] >> 3);

		memset(press_id,0,10);//0826

		#if WAKEUP_GESTURE
			//if (bTouchInGesture == 1)
			if (bTouchIsAwake== 0){
				nvt_ts_wakeup_gesture_report(input_id);
				//enable_irq(ts->client->irq);
				mutex_unlock(&ts->lock);
				continue;
			}
		#endif

		#if MT_PROTOCOL_B
			for (i = 0; i < ts->max_touch_num; i++) {
				position = 1 + 6 * i;
				input_id = (uint8_t)(point_data[position + 0] >> 3);
				if (input_id > TOUCH_MAX_FINGER_NUM)
					continue;

				if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
					input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
					input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
					input_w = (uint32_t)(point_data[position + 4]) + 10;
					if (input_w > 255)
						input_w = 255;

					if ((input_x < 0) || (input_y < 0))
						continue;
					if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
						continue;

					press_id[input_id - 1] = 1;
					input_mt_slot(ts->input_dev, input_id - 1);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);

					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_w);
				//	NVT_INFO("report abs:x=%d, y=%d\n",input_x,input_y);
					finger_cnt++;
				}
			}

			for (i = 0; i < ts->max_touch_num; i++) {
				if (press_id[i] != 1) {
					input_mt_slot(ts->input_dev, i);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
				}
			}

			input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));

		#else

			for (i = 0; i < ts->max_touch_num; i++) {
				position = 1 + 6 * i;
				input_id = (uint8_t)(point_data[position + 0] >> 3);

				if ((point_data[position] & 0x07) == 0x03) {	// finger up (break)
					continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
				} else if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
					input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
					input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
					input_w = (uint32_t)(point_data[position + 4]) + 10;
					if (input_w > 255)
						input_w = 255;

					if ((input_x < 0) || (input_y < 0))
						continue;
					if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
						continue;

					press_id[input_id - 1] = 1;
					input_report_key(ts->input_dev, BTN_TOUCH, 1);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_w);
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
	            //	NVT_INFO("report abs:x=%d, y=%d\n",input_x,input_y);
					input_mt_sync(ts->input_dev);

					finger_cnt++;
				}
			}
			if (finger_cnt == 0) {
				input_report_key(ts->input_dev, BTN_TOUCH, 0);

				input_mt_sync(ts->input_dev);
			}
		#endif


		#if TOUCH_KEY_NUM > 0
		  //  NVT_INFO("point_data[61]=0x%x, point_data[62]=0x%x\n",point_data[61],point_data[62]);
			if (point_data[61] == 0xF8) {

                if(point_data1[62] == point_data[62])
				{

				}
				else
				{
					point_data1[61] = point_data[61];
					point_data1[62] = point_data[62];
					NVT_INFO("point_data[61]=0x%x, point_data[62]=0x%x\n",point_data[61],point_data[62]);
				}  
				
				for (i = 0; i < TOUCH_KEY_NUM; i++) {
					input_report_key(ts->input_dev, touch_key_array_1[i], ((point_data[62] >> i) & 0x01));
				}
			} else {
				for (i = 0; i < TOUCH_KEY_NUM; i++) {
					input_report_key(ts->input_dev, touch_key_array_1[i], 0);
				}
			}
		#endif

		input_sync(ts->input_dev);

XFER_ERROR:
		mutex_unlock(&ts->lock);
	
    } while (!kthread_should_stop());

    return 0;
}

static int nvt_local_init(void)
{
   
  // ts->input_dev=tpd->dev;
   NVT_INFO("nvt_local_init");
   NVT_INFO("Device Tree get regulator!");
   tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
#if I2C_DMA_SUPPORT
	
	//ts->input_dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

    gpDMABuf_va = (uint8_t *)dma_alloc_coherent(&tpd->dev->dev, DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
    if(!gpDMABuf_va){
        NVT_ERROR("Allocate DMA I2C Buffer failed!\n");
		
	//	GTP_DEBUG("[Error] Allocate DMA I2C Buffer failed!\n");
		NVT_INFO("ate DMA I2C Buffer failed!\n");
    }
    memset(gpDMABuf_va, 0, DMA_MAX_TRANSACTION_LENGTH);
#endif

    if (i2c_add_driver(&nvt_i2c_driver) != 0)
    {
        NVT_ERROR("unable to add i2c driver.\n");
        return -1;
    }

    if (tpd_load_status == 0)
    {
        NVT_ERROR("add error touch panel driver.\n");
        i2c_del_driver(&nvt_i2c_driver);
        return -1;
    }

	NVT_INFO("%s: finished\n", __func__);
	
    tpd_type_cap = 1;

    return 0;
}

static void nvt_ts_suspend(struct device *h)
{
	uint8_t buf[4] = {0};

	mutex_lock(&ts->lock);

	dev_info(&ts->client->dev, "%s: begin...\n", __func__);

    tpd_halt_1 = 1;
    
#if WAKEUP_GESTURE
	bTouchIsAwake = 0;
//	bTouchInGesture = 1;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	//---write i2c command to enter "wakeup gesture mode"---
	buf[0] = 0x50;
	buf[1] = 0x13;
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);

	//enable_irq_wake(ts->client->irq);

	dev_info(&ts->client->dev, "%s: Enabled touch wakeup gesture\n", __func__);

#else
	//disable_irq(ts->client->irq);
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	disable_irq(touch_irq);
	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	//---write i2c command to enter "deep sleep mode"---
	buf[0] = 0x50;
	buf[1] = 0x12;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif

	msleep(50);

	mutex_unlock(&ts->lock);

	
	tpd_power_switch(SWITCH_OFF);
	
	dev_info(&ts->client->dev, "%s: end\n", __func__);
	
	return ;
}

static void nvt_ts_resume(struct device *h)
{
#if WAKEUP_GESTURE
	uint8_t buf[4] = {0};
#endif
    tpd_halt_1 = 0;
	
	#if 0
	tpd_power_switch(SWITCH_ON);
	
	#endif
	
	mutex_lock(&ts->lock);

	dev_info(&ts->client->dev, "%s: begin...\n", __func__);

#if WAKEUP_GESTURE
	if (bWakeupByGesture == 1) {
		bWakeupByGesture = 0;
		//---write i2c command to leave "wakeup gesture mode"---
		buf[0] = 0x50;
		buf[1] = 0x14;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
	} else {
		nvt_hw_reset();
		nvt_bootloader_reset();
		nvt_check_fw_reset_state(RESET_STATE_INIT);
	}

	bTouchIsAwake = 1;
	//bTouchInGesture = 0;
#else
   //	nvt_hw_reset();

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	//enable_irq(ts->client->irq);
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	enable_irq(touch_irq);
#endif

	mutex_unlock(&ts->lock);

	dev_info(&ts->client->dev, "%s: end\n", __func__);

	return ;
}

static struct tpd_driver_t nvt_device_driver =
{
    .tpd_device_name = NVT_I2C_NAME,
    .tpd_local_init = nvt_local_init,
    .suspend = nvt_ts_suspend,
    .resume = nvt_ts_resume,
};

static int __init nvt_driver_init(void)
{
	int32_t ret = 0;
	
	NVT_INFO("NVT touch panel driver init!\n");
	tpd_get_dts_info();
	ret = tpd_driver_add(&nvt_device_driver);
    if ( ret < 0){
		NVT_ERROR("add generic driver failed\n");
		goto err_driver;
	}
	NVT_INFO("nvt_driver_init finished\n");
	
err_driver:
	return ret;
}

static void __exit nvt_driver_exit(void)
{
	NVT_INFO("NVT touch panel driver exit!\n");
	nvt_test_sysfs_deinit();
    tpd_driver_remove(&nvt_device_driver);
}

module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");

