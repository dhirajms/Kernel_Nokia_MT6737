/* BEGIN PN: , Added by h84013687, 2013.08.13*/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/disp_drv_platform.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
	#include <mach/gpio_const.h>
    #include <mt-plat/mt_gpio.h>
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt, args...)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt, args...)  pr_debug("[KERNEL/LCM]"fmt, ##args)
#endif

#define I2C_I2C_LCD_BIAS_CHANNEL 	1
#define GPIO_LCD_BIAS_ENP_PIN		(GPIO102 | 0x80000000)
#define GPIO_LCD_BIAS_ENN_PIN		(GPIO101 | 0x80000000)
#define GPIO_DISP_LRSTB_PIN			(GPIO146 | 0x80000000)
#define GPIO_LCDBL_EN_PIN			(GPIO82  | 0x80000000)
#define GTP_RST_PORT			    (GPIO42  | 0x80000000)


extern int tpd_power_switch(s32 state);


/*********************************************************
 * Gate Driver
 *********************************************************/

 /*****************************************************************************
 * Define
 *****************************************************************************/
/*for I2C channel 1*/
#define TPS_I2C_BUSNUM	I2C_I2C_LCD_BIAS_CHANNEL
#define TPS_ADDR		0x3E
#define I2C_ID_NAME		"tps65132"

 #ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
#include <linux/uaccess.h>
/* #include <linux/delay.h> */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info tps65132_board_info __initdata = { I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR) };

static struct i2c_client *tps65132_i2c_client = NULL;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);

/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct tps65132_dev {
	struct i2c_client *client;

};

static const struct i2c_device_id tps65132_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver tps65132_iic_driver = {
	.id_table = tps65132_id,
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "tps65132",
		   },

};

/*****************************************************************************
 * Extern Area
 *****************************************************************************/
 
 
 
/*****************************************************************************
 * Function
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("tps65132_iic_probe\n");
	pr_info("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
	tps65132_i2c_client = client;
	return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	printk("tps65132_remove\n");
	tps65132_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	unsigned char write_data[2] = { 0 };
	write_data[0] = addr;
	write_data[1] = value;

	if (client == NULL)
	{
       return ret;
	}
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		printk("tps65132 write data fail !!\n");
	return ret;
}

/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{
	printk("tps65132_iic_init\n");
	i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
	printk("tps65132_iic_init2\n");
	i2c_add_driver(&tps65132_iic_driver);
	printk("tps65132_iic_init success\n");
	return 0;
}

static void __exit tps65132_iic_exit(void)
{
	printk("tps65132_iic_exit\n");
	i2c_del_driver(&tps65132_iic_driver);
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");

#else
#include <platform/mt_i2c.h> 
static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct mt_i2c_t tps65132_i2c;
	char write_data[2] = { 0 };
	
	write_data[0] = addr;
	write_data[1] = value;
	
	tps65132_i2c.id    = TPS_I2C_BUSNUM;
	tps65132_i2c.addr  = TPS_ADDR;
	tps65132_i2c.mode  = ST_MODE;
	tps65132_i2c.speed = 100;
	
	ret = i2c_write(&tps65132_i2c, write_data, 2);
	if (ret < 0)
		printf("tps65132 write data fail !!\n");
	return ret;
}
#endif


//const static unsigned char LCD_MODULE_ID = 0x09;	//ID0->1;ID1->X
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

const static unsigned int BL_MIN_LEVEL =20;
static LCM_UTIL_FUNCS lcm_util = {0};

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define MDELAY(n) 											(lcm_util.mdelay(n))
#define UDELAY(n)											(lcm_util.udelay(n))

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[128];
};


//update initial param for IC boe_nt35521 0.01
static struct LCM_setting_table lcm_initialization_setting_3lane[] = {

#if 0
	{0xF0,	5,  	{0x55,0xAA,0x52,0x08,00}},   //test mode
	{0xEE,	4,  	{0x87,0x78,0x02,0x40}},
#endif

    {0xFF,	4,	{0xAA,0x55,0x25,0x01}},   // 4lane change for 3lane
	{0x6F,	1,	{0x16}},
	{0xF7,	1,	{0x10}},

    {0xF0,	5,	{0x55,0xAA,0x52,0x08,0x00}}, 
    {0xBD,	5,	{0x01,0xC1,0x10,0x10,0x00}},
    {0xF0,	5,	{0x55,0xAA,0x52,0x08,0x04}},
    {0xC3,	1,	{0x83}},
    


	{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 70, {}},
	
	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};


#if 0
static struct LCM_setting_table lcm_initialization_setting_4lane[] = {

	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x00}}, 
    {0xBD,	5,	{0x01,0xC1,0x10,0x10,0x00}},
    {0xF0,	5,	{0x55,0xAA,0x52,0x08,0x04}},
    {0xC3,	1,	{0x83}},

	{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 70, {}},
	
	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};

#endif

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 40, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 100, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;


    params->physical_width=62;
    params->physical_height=111;


#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */

	params->dsi.LANE_NUM						= 3;	// 4
    params->dsi.PLL_CLOCK 						= 399; 		//3lane
	
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active			= 4;
    params->dsi.vertical_backporch				= 23;
    params->dsi.vertical_frontporch				= 18;
    params->dsi.vertical_active_line			= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active			= 4;
    params->dsi.horizontal_backporch			= 220;
    params->dsi.horizontal_frontporch			= 230;
    params->dsi.horizontal_active_pixel			= FRAME_WIDTH;

    //improve clk quality
   // params->dsi.PLL_CLOCK 						= 390; 		  //3lane

	
   // params->dsi.PLL_CLOCK 						= 257;      //257  4lane
    params->dsi.compatibility_for_nvk 			= 1;
    params->dsi.ssc_disable 					= 1;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

	
}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	
	int ret;
	int i = 0; 
	//enable VSP & VSN
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(5);	
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
    MDELAY(5);	

  
  
 	
	cmd  = 0x00;
	data = 0x0F;	//5.4V
	
	ret = tps65132_write_bytes(cmd,data);
	
	if(ret)
		LCD_DEBUG("[LK]nt35521_hd720_dsi_vdo_innolux cmd=%0x--i2c write error----\n",cmd);    	
	else
		LCD_DEBUG("[LK]nt35521_hd720_dsi_vdo_innolux cmd=%0x--i2c write success----\n",cmd);
  	
	cmd  = 0x01;
	data = 0x0F;	//-5.4V
	
	ret = tps65132_write_bytes(cmd,data);
	if(ret)
		LCD_DEBUG("[LK]nt35521_hd720_dsi_vdo_innolux cmd=%0x--i2c write error----\n",cmd);    	
	else
		LCD_DEBUG("[LK]nt35521_hd720_dsi_vdo_innolux cmd=%0x--i2c write success----\n",cmd);
	MDELAY(10);
	
	//reset high to low to high
    mt_set_gpio_mode(GPIO_DISP_LRSTB_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_DISP_LRSTB_PIN, GPIO_DIR_OUT);

    
   	mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
	 MDELAY(5);   
	mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
	 MDELAY(5);   
	mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    MDELAY(120);


   
   // if (1)
   // {

    push_table(lcm_initialization_setting_3lane, sizeof(lcm_initialization_setting_3lane) / sizeof(struct LCM_setting_table), 1);  

	//}
	//else
//	{

     
	// push_table(lcm_initialization_setting_4lane, sizeof(lcm_initialization_setting_4lane) / sizeof(struct LCM_setting_table), 1);	

//	}
   
//   lcm_util.set_gpio_out(GPIO_LCDBL_EN_PIN, GPIO_OUT_ONE);
    
	   for (i = 0; i < 5; i++) {
		   mt_set_gpio_out(GPIO_LCDBL_EN_PIN, GPIO_OUT_ONE);
		   UDELAY(2);
		   mt_set_gpio_out(GPIO_LCDBL_EN_PIN, GPIO_OUT_ZERO);
		   UDELAY(2);
		    LCD_DEBUG("kernel:GPIO_LCDBL_EN_PIN\n");
	   }

	  mt_set_gpio_out(GPIO_LCDBL_EN_PIN, GPIO_OUT_ONE); 
	   UDELAY(10);


    LCD_DEBUG("innolux_nt35521_lcm_init\n");
}

static void lcm_suspend(void)
{
  
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

	mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
    MDELAY(5);

	mt_set_gpio_out(GPIO_LCDBL_EN_PIN, GPIO_OUT_ZERO);
	MDELAY(5);  
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
    MDELAY(5);	

 
    LCD_DEBUG("kernel:boe_nt35521_lcm_suspend\n");
}

static void lcm_resume(void)
{

   int i = 0; 


    tpd_power_switch(1);
  
    mt_set_gpio_mode(GTP_RST_PORT, GPIO_MODE_00);     // add TP reset in lcm
    mt_set_gpio_dir(GTP_RST_PORT, GPIO_DIR_OUT);
    mt_set_gpio_out(GTP_RST_PORT, GPIO_OUT_ONE);
   


	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(5);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	MDELAY(5);
	 
	mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
	 MDELAY(5);   
	mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
	 MDELAY(5);   
	mt_set_gpio_out(GTP_RST_PORT, GPIO_OUT_ZERO);    // add TP reset in lcm
	mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    MDELAY(20);

    mt_set_gpio_out(GTP_RST_PORT, GPIO_OUT_ONE);  // add TP reset in lcm

  //  if (1)
  // 	{
    push_table(lcm_initialization_setting_3lane, sizeof(lcm_initialization_setting_3lane) / sizeof(struct LCM_setting_table), 1);
  // 	}
  // else
  // 	{
	//push_table(lcm_initialization_setting_4lane, sizeof(lcm_initialization_setting_4lane) / sizeof(struct LCM_setting_table), 1);

 //  }


	   for (i = 0; i < 5; i++) {
		   mt_set_gpio_out(GPIO_LCDBL_EN_PIN, GPIO_OUT_ONE);
		   UDELAY(2);
		   mt_set_gpio_out(GPIO_LCDBL_EN_PIN, GPIO_OUT_ZERO);
		   UDELAY(2);
		    LCD_DEBUG("kernel:GPIO_LCDBL_EN_PIN\n");
	   }

	  mt_set_gpio_out(GPIO_LCDBL_EN_PIN, GPIO_OUT_ONE); 
	   UDELAY(10);
   
    LCD_DEBUG("kernel:innolux_nt35521_lcm_resume\n");
}


LCM_DRIVER nt35521_hd720_dsi_vdo_innolux_lcm_drv =
{
    .name           	= "nt35521_hd720_dsi_vdo_innolux",
    .set_util_funcs 	= lcm_set_util_funcs,
    .get_params     	= lcm_get_params,
    .init           	= lcm_init,
    .suspend        	= lcm_suspend,
    .resume         	= lcm_resume,
    //.compare_id     	= lcm_compare_id,
};
