#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/charging.h>
#include "bq24157.h"
#include "mach/mt_charging.h"

#define bq24157_SLAVE_ADDR_WRITE   0xD4
#define bq24157_SLAVE_ADDR_READ    0xD5

static struct i2c_client *new_client;
static const struct i2c_device_id bq24157_i2c_id[] = { {"bq24157", 0}, {} };

extern kal_bool chargin_hw_init_done;
static int bq24157_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

//add for charger IC GPIO_CD.
#ifdef CONFIG_OF
struct pinctrl *pinctrll;
//struct pinctrl_state *charger_default;
struct pinctrl_state *charger_cd_high;
struct pinctrl_state *charger_cd_low;
#endif

#ifdef CONFIG_OF
static const struct of_device_id bq24157_of_match[] = {
	{.compatible = "mediatek,swithing_charger",},
	{},
};

MODULE_DEVICE_TABLE(of, bq24157_of_match);

struct of_device_id charger_of_match[] = {
    {.compatible = "mediatek,charger_bq24157"},
	{},
};
MODULE_DEVICE_TABLE(of, charger_of_match);
#endif

static struct i2c_driver bq24157_driver = {
	.driver = {
		   .name = "bq24157",
#ifdef CONFIG_OF
		   .of_match_table = bq24157_of_match,
#endif
	},
	.probe = bq24157_driver_probe,
	.id_table = bq24157_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char bq24157_reg[bq24157_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq24157_i2c_access);
static kal_bool bq24157_init_success = KAL_FALSE; //add for charging patch
int g_bq24157_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write bq24157]
  *
  *********************************************************/
int bq24157_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&bq24157_i2c_access);

	if (new_client == NULL) {
		battery_log(BAT_LOG_CRTI, "[bq24157_read_byte]:failed, new_client is NULL\n");

		mutex_unlock(&bq24157_i2c_access);
		return 0;
	}
	
	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		new_client->ext_flag = 0;

		mutex_unlock(&bq24157_i2c_access);
		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	new_client->ext_flag = 0;

	mutex_unlock(&bq24157_i2c_access);
	return 1;
}

int bq24157_write_byte_base(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&bq24157_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	if (new_client == NULL) {
		battery_log(BAT_LOG_CRTI, "[bq24157_write_byte_base]:failed, new_client is NULL\n");

		mutex_unlock(&bq24157_i2c_access);
		return 0;
	}

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&bq24157_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&bq24157_i2c_access);
	return 1;
}

unsigned int bq24157_write_byte(unsigned char addr, unsigned char value)
{
    unsigned int ret_code = 0;
    if(KAL_TRUE == bq24157_init_success) {
        ret_code = bq24157_write_byte_base(addr, value);

        return ret_code;
    }

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
    ret_code = bq24157_write_byte_base(6, 0x79);

#else
    ret_code = bq24157_write_byte_base(6, 0x70);
#endif
    if(1 == ret_code) {

        bq24157_init_success = KAL_TRUE;
        ret_code = bq24157_write_byte_base(addr, value);

    }
    return ret_code;
}
/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq24157_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK, unsigned char SHIFT)
{
	unsigned char bq24157_reg = 0;
	int ret = 0;

	ret = bq24157_read_byte(RegNum, &bq24157_reg);

	battery_log(BAT_LOG_FULL, "[bq24157_read_interface] Reg[%x]=0x%x\n", RegNum, bq24157_reg);

	bq24157_reg &= (MASK << SHIFT);
	*val = (bq24157_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[bq24157_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int bq24157_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT)
{
	unsigned char bq24157_reg = 0;
	int ret = 0;

	ret = bq24157_read_byte(RegNum, &bq24157_reg);
	battery_log(BAT_LOG_FULL, "[bq24157_config_interface] Reg[%x]=0x%x\n", RegNum, bq24157_reg);

	bq24157_reg &= ~(MASK << SHIFT);
	bq24157_reg |= (val << SHIFT);

	if (RegNum == bq24157_CON4 && val == 1 && MASK == CON4_RESET_MASK
	    && SHIFT == CON4_RESET_SHIFT) {
		/* RESET bit */
	} else if (RegNum == bq24157_CON4) {
		bq24157_reg &= ~0x80;	/* RESET bit read returs 1, so clear it */
	}

	ret = bq24157_write_byte(RegNum, bq24157_reg);
	battery_log(BAT_LOG_FULL, "[bq24157_config_interface] write Reg[%x]=0x%x\n", RegNum,
		    bq24157_reg);

	return ret;
}

/* write one register directly */
unsigned int bq24157_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	int ret = 0;

	ret = bq24157_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0 */

void bq24157_set_tmr_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_TMR_RST_MASK),
				       (unsigned char) (CON0_TMR_RST_SHIFT)
	    );
}

unsigned int bq24157_get_otg_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface((unsigned char) (bq24157_CON0),
				     (&val), (unsigned char) (CON0_OTG_MASK),
				     (unsigned char) (CON0_OTG_SHIFT)
	    );
	return val;
}

void bq24157_set_en_stat(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_STAT_MASK),
				       (unsigned char) (CON0_EN_STAT_SHIFT)
	    );
}

unsigned int bq24157_get_chip_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface((unsigned char) (bq24157_CON0),
				     (&val), (unsigned char) (CON0_STAT_MASK),
				     (unsigned char) (CON0_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq24157_get_boost_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface((unsigned char) (bq24157_CON0),
				     (&val), (unsigned char) (CON0_BOOST_MASK),
				     (unsigned char) (CON0_BOOST_SHIFT)
	    );
	return val;
}

unsigned int bq24157_get_fault_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface((unsigned char) (bq24157_CON0),
				     (&val), (unsigned char) (CON0_FAULT_MASK),
				     (unsigned char) (CON0_FAULT_SHIFT)
	    );
	return val;
}

/* CON1 */

void bq24157_set_input_charging_current(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LIN_LIMIT_MASK),
				       (unsigned char) (CON1_LIN_LIMIT_SHIFT)
	    );
}

void bq24157_set_v_low(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LOW_V_MASK),
				       (unsigned char) (CON1_LOW_V_SHIFT)
	    );
}

void bq24157_set_te(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_TE_MASK),
				       (unsigned char) (CON1_TE_SHIFT)
	    );
}

void bq24157_set_ce(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CE_MASK),
				       (unsigned char) (CON1_CE_SHIFT)
	    );
}

void bq24157_set_hz_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_HZ_MODE_MASK),
				       (unsigned char) (CON1_HZ_MODE_SHIFT)
	    );
}

void bq24157_set_opa_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OPA_MODE_MASK),
				       (unsigned char) (CON1_OPA_MODE_SHIFT)
	    );
}

/* CON2 */

void bq24157_set_oreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OREG_MASK),
				       (unsigned char) (CON2_OREG_SHIFT)
	    );
}

void bq24157_set_otg_pl(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_PL_MASK),
				       (unsigned char) (CON2_OTG_PL_SHIFT)
	    );
}

void bq24157_set_otg_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_EN_MASK),
				       (unsigned char) (CON2_OTG_EN_SHIFT)
	    );
}

/* CON3 */

unsigned int bq24157_get_vender_code(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface((unsigned char) (bq24157_CON3),
				     (&val), (unsigned char) (CON3_VENDER_CODE_MASK),
				     (unsigned char) (CON3_VENDER_CODE_SHIFT)
	    );
	return val;
}

unsigned int bq24157_get_pn(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface((unsigned char) (bq24157_CON3),
				     (&val), (unsigned char) (CON3_PIN_MASK),
				     (unsigned char) (CON3_PIN_SHIFT)
	    );
	return val;
}

unsigned int bq24157_get_revision(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface((unsigned char) (bq24157_CON3),
				     (&val), (unsigned char) (CON3_REVISION_MASK),
				     (unsigned char) (CON3_REVISION_SHIFT)
	    );
	return val;
}

/* CON4 */

void bq24157_set_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_RESET_MASK),
				       (unsigned char) (CON4_RESET_SHIFT)
	    );
}

void bq24157_set_iocharge(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_CHR_MASK),
				       (unsigned char) (CON4_I_CHR_SHIFT)
	    );
}

void bq24157_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_TERM_MASK),
				       (unsigned char) (CON4_I_TERM_SHIFT)
	    );
}

/* CON5 */

void bq24157_set_dis_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_DIS_VREG_MASK),
				       (unsigned char) (CON5_DIS_VREG_SHIFT)
	    );
}

void bq24157_set_io_level(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IO_LEVEL_MASK),
				       (unsigned char) (CON5_IO_LEVEL_SHIFT)
	    );
}

unsigned int bq24157_get_sp_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface((unsigned char) (bq24157_CON5),
				     (&val), (unsigned char) (CON5_SP_STATUS_MASK),
				     (unsigned char) (CON5_SP_STATUS_SHIFT)
	    );
	return val;
}

unsigned int bq24157_get_en_level(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface((unsigned char) (bq24157_CON5),
				     (&val), (unsigned char) (CON5_EN_LEVEL_MASK),
				     (unsigned char) (CON5_EN_LEVEL_SHIFT)
	    );
	return val;
}

void bq24157_set_vsp(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_VSP_MASK),
				       (unsigned char) (CON5_VSP_SHIFT)
	    );
}

/* CON6 */

void bq24157_set_i_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_ISAFE_MASK),
				       (unsigned char) (CON6_ISAFE_SHIFT)
	    );
}

void bq24157_set_v_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24157_config_interface((unsigned char) (bq24157_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VSAFE_MASK),
				       (unsigned char) (CON6_VSAFE_SHIFT)
	    );
}

unsigned int bq24157_get_v_safe(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq24157_read_interface(     (unsigned char)(bq24157_CON6),
                                    (&val),
                                    (unsigned char)(CON6_VSAFE_MASK),
                                    (unsigned char)(CON6_VSAFE_SHIFT)
                                    );
    return val;
}
/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void bq24157_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24157_read_interface(0x03, &val, 0xFF, 0x0);

	if (val == 0)
		g_bq24157_hw_exist = 0;
	else
		g_bq24157_hw_exist = 1;

	battery_log(BAT_LOG_CRTI, "[bq24157_hw_component_detect] exist=%d, Reg[03]=0x%x\n", g_bq24157_hw_exist, val);
}

int is_bq24157_exist(void)
{
	battery_log(BAT_LOG_CRTI, "[is_bq24157_exist] g_bq24157_hw_exist=%d\n", g_bq24157_hw_exist);

	return g_bq24157_hw_exist;
}

void bq24157_dump_register(void)
{
	int i = 0;

	battery_log(BAT_LOG_CRTI, "[bq24157] ");
	for (i = 0; i < bq24157_REG_NUM; i++) {
		bq24157_read_byte(i, &bq24157_reg[i]);
		printk("[0x%x]=0x%x ", i, bq24157_reg[i]);
	}
    printk("\n");
}

static int bq24157_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	new_client = client;

	bq24157_hw_component_detect();
	bq24157_dump_register();
	chargin_hw_init_done = KAL_TRUE;

	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_bq24157 = 0;


/**********************************************************
  *
  *   [bq24157 gpio_cd function]
  *
  *********************************************************/
void charger_pin_cd_config(kal_bool status)
{
#ifdef CONFIG_OF

    if (KAL_TRUE == status)  { // cd pin out put High
        pinctrl_select_state(pinctrll, charger_cd_high);
		//battery_log(BAT_LOG_CRTI, "%s for test: cd out put High!\n", __func__);
    } else {
        pinctrl_select_state(pinctrll, charger_cd_low);
		//battery_log(BAT_LOG_CRTI, "%s for test: cd out put Low!\n", __func__);
    }
#endif //CONFIG_OF
}
int charger_get_gpio_info(struct platform_device *pdev)
{
    int ret;
    battery_log(BAT_LOG_CRTI, "[%s %d] mt_charger_pincrrl+++++++++++++++++\n",
        __func__, pdev->id);
    pinctrll = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrll)) {
        ret = PTR_ERR(pinctrll);
		battery_log(BAT_LOG_CRTI, "[%s]Cannot find charger_bq pinctrl, err=%d\n",
			__func__, (int)PTR_ERR(pinctrll));
		return ret;
	}
	/*
    charger_default = pinctrl_lookup_state(pinctrll, "default");
	if (IS_ERR(charger_default)) {
		ret = PTR_ERR(charger_default);
		dev_err(&pdev->dev, "charger_get_gpio_info Cannot find charger default charger_default!\n");
		return ret;
	}
	*/
    charger_cd_high = pinctrl_lookup_state(pinctrll, "cd_high");
	if (IS_ERR(charger_cd_high)) {
		ret = PTR_ERR(charger_cd_high);
		dev_err(&pdev->dev, "charger_get_gpio_info Cannot find charger pinctrl charger_cd_high!\n");
		return ret;
	}
    charger_cd_low = pinctrl_lookup_state(pinctrll, "cd_low");
	if (IS_ERR(charger_cd_low)) {
		ret = PTR_ERR(charger_cd_low);
		dev_err(&pdev->dev, "charger_get_gpio_info Cannot find charger pinctrl charger_cd_low!\n");
		return ret;
	}
    battery_log(BAT_LOG_CRTI, "[%s %d] mt_charger_pincrrl-----------------------\n",
        __func__, pdev->id);
    return 0;
}


struct platform_device bq24157_user_space_device = {
	.name = "bq24157-user",
	.id = -1,
};

static int __init bq24157_init(void)
{
	return 0;
}

static void __exit bq24157_exit(void)
{
	i2c_del_driver(&bq24157_driver);
}

subsys_initcall(bq24157_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq24157 Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");
