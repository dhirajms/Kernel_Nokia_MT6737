#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/wait.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	    #include <mt-plat/mt_gpio.h>
		#include <mach/gpio_const.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define PHYSICAL_WIDTH                                      (56)
#define PHYSICAL_HIGHT                                      (100)


#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x00   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_RST (GPIO146 | 0x80000000)

#if 1
    #ifndef TRUE
        #define   TRUE     1
    #endif
 
    #ifndef FALSE
        #define   FALSE    0
    #endif
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

//static kal_bool IsFirstBoot = KAL_TRUE;

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[120];
};


static struct LCM_setting_table lcm_initialization_setting[] = {

	{0xFF,4,{0xAA,0x55,0xA5,0x80}},
	{0x6F,1,{0x0E}},
	{0xF4,1,{0x3A}},
	{0xFF,4,{0xAA,0x55,0xA5,0x00}},
	/*************end***************************/

	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}}, //#LV2 Page 1 enable
	{0xB0,3,{0x0A,0x0A,0x0A}},          //#AVDD Set AVDD 5.5V
	{0xB6,3,{0x34,0x34,0x34}},          //#AVDD ratio
	{0xB1,3,{0x0A,0x0A,0x0A}},         //#AVEE  -5.5Vd
	{0xB7,3,{0x45,0x24,0x24}},	       //24  //#AVEE ratio
	{0xB2,3,{0x01,0x01,0x01}},       //#VCL  -3V
	{0xB8,3,{0x34,0x34,0x34}},       //#VCL ratio
	{0xBF,1,{0x01}},                 //#VGH 15V  
	{0xB3,3,{0x08,0x08,0x08}},
	{0xB9,3,{0x34,0x34,0x34}},  //#VGH ratio
	{0xBA,3,{0x14,0x14,0x14}},  //#VGLX ratio
	{0xBC,3,{0x00,0xA0,0x00}},		//a0  //#VGMP/VGSP 5V/0V
	{0xBD,3,{0x00,0xA0,0x00}},	    //a0  //#VGMN/VGSN -5V/0V
	{0xBE,2,{0x00,0x8D}},			 //   //#VCOM 
	//{0xC3,1,{0x05}},			   //06

	{0xD1,52,{0x00,0x78,0x00,0x7A,0x00,0x92,0x00,0xB4,0x00,0xCE,0x00,0xFE,0x01,0x20,0x01,0x55,0x01,0x7A,0x01,0xB1,0x01,0xD8,0x02,0x12,0x02,0x3E,0x02,0x3F,0x02,0x65,0x02,0x8A,0x02,0x9E,0x02,0xB6,0x02,0xC4,0x02,0xD6,0x02,0xE2,0x02,0xF1,0x02,0xFC,0x03,0x0E,0x03,0x3C,0x03,0xFF}},
	{0xD2,52,{0x00,0x78,0x00,0x7A,0x00,0x92,0x00,0xB4,0x00,0xCE,0x00,0xFE,0x01,0x20,0x01,0x55,0x01,0x7A,0x01,0xB1,0x01,0xD8,0x02,0x12,0x02,0x3E,0x02,0x3F,0x02,0x65,0x02,0x8A,0x02,0x9E,0x02,0xB6,0x02,0xC4,0x02,0xD6,0x02,0xE2,0x02,0xF1,0x02,0xFC,0x03,0x0E,0x03,0x3C,0x03,0xFF}},
	{0xD3,52,{0x00,0x78,0x00,0x7A,0x00,0x92,0x00,0xB4,0x00,0xCE,0x00,0xFE,0x01,0x20,0x01,0x55,0x01,0x7A,0x01,0xB1,0x01,0xD8,0x02,0x12,0x02,0x3E,0x02,0x3F,0x02,0x65,0x02,0x8A,0x02,0x9E,0x02,0xB6,0x02,0xC4,0x02,0xD6,0x02,0xE2,0x02,0xF1,0x02,0xFC,0x03,0x0E,0x03,0x3C,0x03,0xFF}},
	{0xD4,52,{0x00,0x78,0x00,0x7A,0x00,0x92,0x00,0xB4,0x00,0xCE,0x00,0xFE,0x01,0x20,0x01,0x55,0x01,0x7A,0x01,0xB1,0x01,0xD8,0x02,0x12,0x02,0x3E,0x02,0x3F,0x02,0x65,0x02,0x8A,0x02,0x9E,0x02,0xB6,0x02,0xC4,0x02,0xD6,0x02,0xE2,0x02,0xF1,0x02,0xFC,0x03,0x0E,0x03,0x3C,0x03,0xFF}},
	{0xD5,52,{0x00,0x78,0x00,0x7A,0x00,0x92,0x00,0xB4,0x00,0xCE,0x00,0xFE,0x01,0x20,0x01,0x55,0x01,0x7A,0x01,0xB1,0x01,0xD8,0x02,0x12,0x02,0x3E,0x02,0x3F,0x02,0x65,0x02,0x8A,0x02,0x9E,0x02,0xB6,0x02,0xC4,0x02,0xD6,0x02,0xE2,0x02,0xF1,0x02,0xFC,0x03,0x0E,0x03,0x3C,0x03,0xFF}},
	{0xD6,52,{0x00,0x78,0x00,0x7A,0x00,0x92,0x00,0xB4,0x00,0xCE,0x00,0xFE,0x01,0x20,0x01,0x55,0x01,0x7A,0x01,0xB1,0x01,0xD8,0x02,0x12,0x02,0x3E,0x02,0x3F,0x02,0x65,0x02,0x8A,0x02,0x9E,0x02,0xB6,0x02,0xC4,0x02,0xD6,0x02,0xE2,0x02,0xF1,0x02,0xFC,0x03,0x0E,0x03,0x3C,0x03,0xFF}},


	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
	{0xB1,2,{0xFC,0x00}}, //#Display control
	{0xB2,1,{0x00}},
	{0xB5,1,{0x6B}},   //#480x854								  
	{0xB6,1,{0x03}},	//05  //#Source hold time
	{0xB7,2,{0x70,0x70}},   //#Gate EQ control
	{0xB8,4,{0x01,0x05,0x05,0x05}},		// 01,05,05,05 //#Source EQ control (Mode 2)
	{0xBC,3,{0x00,0x00,0x00}},      //#Inversion mode  (Column)
	{0xBD,4,{0x01,0x90,0x1C,0x1C}},
	{0xCB,10,{0x02,0x0B,0xDC,0x01,0x12,0x33,0x33,0x11,0x11,0x0C}}, //#NT35512 Timing control 

	//{0x35,1,{0x00}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 130, {}},   //delay 130ms
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				//MDELAY(10);//soso add or it will fail to send register
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

		params->physical_width=PHYSICAL_WIDTH;
        params->physical_height=PHYSICAL_HIGHT;


		params->dsi.mode   =  SYNC_EVENT_VDO_MODE;   //BURST_VDO_MODE;  //SYNC_PULSE_VDO_MODE;  //SYNC_EVENT_VDO_MODE;
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 5;//16
		params->dsi.vertical_frontporch					= 12;//6
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;
		params->dsi.horizontal_backporch				= 30;  //30
		params->dsi.horizontal_frontporch				= 30;  //30
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

#ifndef CONFIG_FPGA_EARLY_PORTING
	    params->dsi.PLL_CLOCK = 190; //190     //Fmipi=(xres+left_proch+right_proch+hsync_len)*(yres+upper_proch+lower_proch+vsync_len)*bpp* 
                                                //refresh / mipi_lanes
#else
    	// Bit rate calculation
		params->dsi.pll_div1=30;//32		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		params->dsi.pll_div2=1; 
		// div2=0~15: fout=fvo/(2*div2)
#endif

#if 0
		/* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response. 
		params->dsi.lcm_int_te_monitor = TRUE;   //FALSE
		params->dsi.lcm_int_te_period = 1; // Unit : frames 
 

		// Need longer FP for more opportunity to do int. TE monitor applicably. 
		//if(params->dsi.lcm_int_te_monitor) 
			//params->dsi.vertical_frontporch *= 2; 
 
		// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
		params->dsi.lcm_ext_te_monitor = FALSE;   //FALSE
		// Non-continuous clock 
		params->dsi.noncont_clock = TRUE; 
		params->dsi.noncont_clock_period = 2; // Unit : frames	
#endif

	#if 1
      params->dsi.esd_check_enable = 1;
      params->dsi.customization_esd_check_enable = 1; //switch for  esd check with register
      params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
      params->dsi.lcm_esd_check_table[0].count = 1;
      params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
    #endif
}

static void lcm_init(void)
{
#ifdef BUILD_LK
    lcm_util.set_gpio_mode(LCM_RST, 0);
	lcm_util.set_gpio_dir(LCM_RST, 1);
	lcm_util.set_gpio_out(LCM_RST, 1);
	MDELAY(1);
	lcm_util.set_gpio_out(LCM_RST, 0);
	MDELAY(10);
	lcm_util.set_gpio_out(LCM_RST, 1);
	MDELAY(125); 
#else
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
#endif	
	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	
	//init_lcm_registers();

}


static void lcm_suspend(void)
{
#if 0
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20);			//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(150);		//Must > 120ms
#endif
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	//lcm_compare_id();

	lcm_init();
	
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

#if 0
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];
	unsigned int array[16];
	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x04, buffer, 3);
	id = buffer[1]; //we only need ID
#if defined(BUILD_UBOOT)
	/*The Default Value should be 0x00,0x80,0x00*/
	//printf("\n\n\n\n[soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);
#endif
    //return (id == 0x80)?1:0;
    return 1;
}


#ifndef BUILD_LK
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif

static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];
	int ret = 0;
	
	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

    /*
	read_reg_v2(0x0F, buffer, 1);

	if(buffer[0] != 0xc0)
	{
		printk("[LCM ERROR] [0x0F]=0x%02x\n", buffer[0]);
		ret++;
	}

	read_reg_v2(0x05, buffer, 1);
	if(buffer[0] != 0x00)
	{
		printk("[LCM ERROR] [0x05]=0x%02x\n", buffer[0]);
		ret++;
	}
	*/
	read_reg_v2(0x0A, buffer, 1);
	if(buffer[0]!=0x9C)
	{
		printk("[LCM ERROR] [0x0A]=0x%02x\n", buffer[0]);
		ret++;
	}

	// return TRUE: need recovery
	// return FALSE: No need recovery
	if(ret)
	{
		return TRUE;
	}
	else
	{			 
		return FALSE;
	}
#else
	return FALSE;
 #endif
}


static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	lcm_resume();

	return TRUE;
}
#endif

LCM_DRIVER nt35512_fwvga_dsi_vdo_zgd_lcm_drv = 
{
    .name			= "nt35512_fwvga_dsi_vdo_zgd_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.compare_id    = lcm_compare_id,
	//.esd_check   	= lcm_esd_check,
    //.esd_recover	= lcm_esd_recover,
};

