/*
 * Copyright (C) 2016 MediaTek Inc.
 * ShuFanLee <shufan_lee@richtek.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/mt_boot_common.h>
#include <mach/mt_pe.h>
#include "rt9458.h"
#include "mtk_charger_intf.h"

#ifdef CONFIG_RT_REGMAP
#include <mt-plat/rt-regmap.h>
#endif

#define RT9458_DRV_VERSION	"1.0.6_MTK"

#define I2C_ACCESS_MAX_RETRY	5

/* ================= */
/* internal variable */
/* ================= */

struct rt9458_desc {
	u32 ichg;	/* 10uA */
	u32 aicr;	/* 10uA */
	u32 mivr;	/* mV */
	u32 cv;		/* uV */
	u32 ieoc;	/* uA */
	bool enable_te;
	bool enable_wdt;
	int regmap_represent_slave_addr;
	const char *regmap_name;
};

/* These default values will be applied if there's no property in dts */
static struct rt9458_desc rt9458_default_desc = {
	.ichg = 50000,	/* 10uA */
	.aicr = 50000,	/* 10uA */
	.mivr = 4400,	/* mV */
	.cv = 4350000,	/* uA */
	.ieoc = 150000,	/* uA */
	.enable_te = true,
	.enable_wdt = true,
	.regmap_represent_slave_addr = RT9458_SLAVE_ADDR,
	.regmap_name = "rt9458",
};

struct rt9458_info {
	struct mtk_charger_info mchr_info;
	struct i2c_client *i2c;
	struct mutex i2c_access_lock;
	struct mutex aicr_access_lock;
	struct rt9458_desc *desc;
	bool err_state;
	int irq;
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *regmap_dev;
	struct rt_regmap_properties *regmap_prop;
#endif
};

static const unsigned char rt9458_reg_addr[] = {
	RT9458_REG_CTRL1,
	RT9458_REG_CTRL2,
	RT9458_REG_CTRL3,
	RT9458_REG_DEVID,
	RT9458_REG_CTRL4,
	RT9458_REG_CTRL5,
	RT9458_REG_CTRL6,
	RT9458_REG_CTRL7,
	RT9458_REG_IRQ1,
	RT9458_REG_IRQ2,
	RT9458_REG_IRQ3,
	RT9458_REG_MASK1,
	RT9458_REG_MASK2,
	RT9458_REG_MASK3,
	RT9458_REG_CTRL8,
};

enum rt9458_charging_status {
	RT9458_CHG_STATUS_READY = 0,
	RT9458_CHG_STATUS_PROGRESS,
	RT9458_CHG_STATUS_DONE,
	RT9458_CHG_STATUS_FAULT,
	RT9458_CHG_STATUS_MAX,
};

/* Charging status name */
static const char *rt9458_chg_status_name[RT9458_CHG_STATUS_MAX] = {
	"ready", "progress", "done", "fault",
};

/* uV, step is 20000uA except 4300000->4330000 */
static const u32 rt9458_battery_voreg[] = {
	3500000, 3520000, 3540000, 3560000, 3580000, 3600000, 3620000, 3640000,
	3660000, 3680000, 3700000, 3720000, 3740000, 3760000, 3780000, 3800000,
	3820000, 3840000, 3860000, 3880000, 3900000, 3920000, 3940000, 3960000,
	3980000, 4000000, 4020000, 4040000, 4060000, 4080000, 4100000, 4120000,
	4140000, 4160000, 4180000, 4200000, 4220000, 4240000, 4260000, 4280000,
	4300000, 4330000, 4350000, 4370000, 4390000, 4410000, 4430000, 4450000,
	4450000, 4450000, 4450000, 4450000, 4450000, 4450000, 4450000, 4450000,
	4450000, 4450000, 4450000, 4450000, 4450000, 4450000, 4450000, 4450000,
};

/* uV, step is 20000uA except 4300000->4330000 */
static const u32 rt9458_battery_vmreg[] = {
	4200000, 4220000, 4240000, 4260000, 4280000, 4300000, 4320000, 4340000,
	4360000, 4380000, 4400000, 4430000, 4450000, 4450000, 4450000, 4450000,
};

#ifdef CONFIG_RT_REGMAP
RT_REG_DECL(RT9458_REG_CTRL1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_DEVID, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL4, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL5, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL6, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL7, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_IRQ1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_IRQ2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_IRQ3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_MASK1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_MASK2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_MASK3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL8, 1, RT_VOLATILE, {});

static rt_register_map_t rt9458_regmap_map[] = {
	RT_REG(RT9458_REG_CTRL1),
	RT_REG(RT9458_REG_CTRL2),
	RT_REG(RT9458_REG_CTRL3),
	RT_REG(RT9458_REG_DEVID),
	RT_REG(RT9458_REG_CTRL4),
	RT_REG(RT9458_REG_CTRL5),
	RT_REG(RT9458_REG_CTRL6),
	RT_REG(RT9458_REG_CTRL7),
	RT_REG(RT9458_REG_IRQ1),
	RT_REG(RT9458_REG_IRQ2),
	RT_REG(RT9458_REG_IRQ3),
	RT_REG(RT9458_REG_MASK1),
	RT_REG(RT9458_REG_MASK2),
	RT_REG(RT9458_REG_MASK3),
	RT_REG(RT9458_REG_CTRL8),
};
#endif /* CONFIG_RT_REGMAP */

/* ========================= */
/* I2C operations */
/* ========================= */

static int rt9458_device_read(void *client, u32 addr, int leng, void *dst)
{
	int ret = 0;
	struct i2c_client *i2c = NULL;

	i2c = (struct i2c_client *)client;
	ret = i2c_smbus_read_i2c_block_data(i2c, addr, leng, dst);

	return ret;
}

static int rt9458_device_write(void *client, u32 addr, int leng,
	const void *src)
{
	int ret = 0;
	struct i2c_client *i2c = NULL;

	i2c = (struct i2c_client *)client;
	ret = i2c_smbus_write_i2c_block_data(i2c, addr, leng, src);

	return ret;
}

#ifdef CONFIG_RT_REGMAP
static struct rt_regmap_fops rt9458_regmap_fops = {
	.read_device = rt9458_device_read,
	.write_device = rt9458_device_write,
};

static int rt9458_register_rt_regmap(struct rt9458_info *info)
{
	int ret = 0;
	struct i2c_client *i2c = info->i2c;
	struct rt_regmap_properties *prop = NULL;

	pr_info("%s\n", __func__);

	prop = devm_kzalloc(&i2c->dev, sizeof(struct rt_regmap_properties),
		GFP_KERNEL);
	if (!prop) {
		pr_err("%s: no enough memory\n", __func__);
		return -ENOMEM;
	}

	prop->name = info->desc->regmap_name;
	prop->aliases = info->desc->regmap_name;
	prop->register_num = ARRAY_SIZE(rt9458_regmap_map);
	prop->rm = rt9458_regmap_map;
	prop->rt_regmap_mode = RT_SINGLE_BYTE | RT_CACHE_DISABLE | RT_IO_PASS_THROUGH;
	prop->io_log_en = 0;

	info->regmap_prop = prop;
	info->regmap_dev = rt_regmap_device_register_ex(
		info->regmap_prop,
		&rt9458_regmap_fops,
		&i2c->dev,
		i2c,
		info->desc->regmap_represent_slave_addr,
		info
	);

	if (!info->regmap_dev) {
		pr_err("%s: register regmap device failed\n", __func__);
		return -EIO;
	}

	return ret;
}
#endif /* CONFIG_RT_REGMAP */

static inline int _rt9458_i2c_write_byte(struct rt9458_info *info, u8 cmd,
	u8 data)
{
	int ret = 0, retry = 0;

	do {
#ifdef CONFIG_RT_REGMAP
		ret = rt_regmap_block_write(info->regmap_dev, cmd, 1, &data);
#else
		ret = rt9458_device_write(info->i2c, cmd, 1, &data);
#endif
		retry++;
		if (ret < 0)
			mdelay(20);
	} while (ret < 0 && retry < I2C_ACCESS_MAX_RETRY);

	if (ret < 0)
		pr_err("%s: I2CW[0x%02X] = 0x%02X failed, ret = %d\n",
			__func__, cmd, data, ret);
	else
		pr_debug("%s: I2CW[0x%02X] = 0x%02X\n", __func__, cmd, data);

	return ret;
}

#if 0 /* Uncomment this block if you need these functions */
static int rt9458_i2c_write_byte(struct rt9458_info *info, u8 cmd, u8 data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9458_i2c_write_byte(info, cmd, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}
#endif

static inline int _rt9458_i2c_read_byte(struct rt9458_info *info, u8 cmd)
{
	int ret = 0, ret_val = 0, retry = 0;

	do {
#ifdef CONFIG_RT_REGMAP
		ret = rt_regmap_block_read(info->regmap_dev, cmd, 1, &ret_val);
#else
		ret = rt9458_device_read(info->i2c, cmd, 1, &ret_val);
#endif
		retry++;
		if (ret < 0)
			msleep(20);
	} while (ret < 0 && retry < I2C_ACCESS_MAX_RETRY);

	if (ret < 0) {
		pr_err("%s: I2CR[0x%02X] failed, ret = %d\n",
			__func__, cmd, ret);
		return ret;
	}

	ret_val = ret_val & 0xFF;

	pr_debug("%s: I2CR[0x%02X] = 0x%02X\n", __func__, cmd, ret_val);

	return ret_val;
}

static int rt9458_i2c_read_byte(struct rt9458_info *info, u8 cmd)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9458_i2c_read_byte(info, cmd);
	mutex_unlock(&info->i2c_access_lock);

	if (ret < 0)
		return ret;

	return (ret & 0xFF);
}

#if 0 /* Uncomment this block if you need these functions */
static inline int _rt9458_i2c_block_write(struct rt9458_info *info, u8 cmd,
	u32 leng, const u8 *data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(info->regmap_dev, cmd, leng, data);
#else
	ret = rt9458_device_write(info->i2c, cmd, leng, data);
#endif

	return ret;
}


static int rt9458_i2c_block_write(struct rt9458_info *info, u8 cmd, u32 leng,
	const u8 *data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9458_i2c_block_write(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int _rt9458_i2c_block_read(struct rt9458_info *info, u8 cmd,
	u32 leng, u8 *data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(info->regmap_dev, cmd, leng, data);
#else
	ret = rt9458_device_read(info->i2c, cmd, leng, data);
#endif

	return ret;
}


static int rt9458_i2c_block_read(struct rt9458_info *info, u8 cmd, u32 leng,
	u8 *data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9458_i2c_block_read(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}
#endif

static int rt9458_i2c_test_bit(struct rt9458_info *info, u8 cmd, u8 shift,
	bool *is_one)
{
	int ret = 0;
	u8 data = 0;

	ret = rt9458_i2c_read_byte(info, cmd);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	data = ret & (1 << shift);
	*is_one = (data == 0 ? false : true);

	return ret;
}

static int rt9458_i2c_update_bits(struct rt9458_info *info, u8 cmd, u8 data,
	u8 mask)
{
	int ret = 0;
	u8 reg_data = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9458_i2c_read_byte(info, cmd);
	if (ret < 0) {
		mutex_unlock(&info->i2c_access_lock);
		return ret;
	}

	reg_data = ret & 0xFF;
	reg_data &= ~mask;
	reg_data |= (data & mask);

	ret = _rt9458_i2c_write_byte(info, cmd, reg_data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int rt9458_set_bit(struct rt9458_info *info, u8 reg, u8 mask)
{
	return rt9458_i2c_update_bits(info, reg, mask, mask);
}

static inline int rt9458_clr_bit(struct rt9458_info *info, u8 reg, u8 mask)
{
	return rt9458_i2c_update_bits(info, reg, 0x00, mask);
}

/* ================== */
/* internal functions */
/* ================== */
static int rt_charger_set_aicr(struct mtk_charger_info *mchr_info, void *data);

static u8 rt9458_find_closest_reg_value(u32 min, u32 max, u32 step, u32 num,
	u32 target)
{
	u32 i = 0, cur_val = 0, next_val = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target < min)
		return 0;

	for (i = 0; i < num - 1; i++) {
		cur_val = min + i * step;
		next_val = cur_val + step;

		if (cur_val > max)
			cur_val = max;

		if (next_val > max)
			next_val = max;

		if (target >= cur_val && target < next_val)
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return num - 1;
}

static u8 rt9458_find_closest_reg_value_via_table(const u32 *value_table,
	u32 table_size, u32 target_value)
{
	u32 i = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target_value < value_table[0])
		return 0;

	for (i = 0; i < table_size - 1; i++) {
		if (target_value >= value_table[i] &&
		    target_value < value_table[i + 1])
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return table_size - 1;
}

static u32 rt9458_find_closest_real_value(u32 min, u32 max, u32 step,
	u8 reg_val)
{
	u32 ret_val = 0;

	ret_val = min + reg_val * step;
	if (ret_val > max)
		ret_val = max;

	return ret_val;
}

/*
 * internal: use AICR setting in CTRL2
 * otherwise, AICR is decided by OTG_PinP
 */
static int rt9458_set_aicr_int(struct rt9458_info *info, bool internal)
{
	int ret = 0;

	pr_info("%s: internal = %d\n", __func__, internal);

	ret = (internal ? rt9458_set_bit : rt9458_clr_bit)
		(info, RT9458_REG_CTRL2, RT9458_MASK_AICR_INT);

	return ret;
}

static int rt9458_set_aicr_sel(struct rt9458_info *info, bool sel)
{
	int ret = 0;

	pr_info("%s: sel = %d\n", __func__, sel);

	ret = (sel ? rt9458_set_bit : rt9458_clr_bit)
		(info, RT9458_REG_CTRL6, RT9458_MASK_AICR_SEL);

	return ret;
}

static bool rt9458_is_hw_exist(struct rt9458_info *info)
{
	int ret = 0;
	u8 vendor_id = 0, chip_rev = 0;

	ret = i2c_smbus_read_byte_data(info->i2c, RT9458_REG_DEVID);
	if (ret < 0)
		return 0;

	vendor_id = ret & 0xF0;
	chip_rev = ret & 0x0F;
	if (vendor_id != RT9458_VENDOR_ID) {
		pr_err("%s: vendor id is incorrect (0x%02X)\n",
			__func__, vendor_id);
		return false;
	}
	pr_info("%s: chip rev (E%d, 0x%02X)\n", __func__, chip_rev, chip_rev);

	info->mchr_info.device_id = chip_rev;
	return true;
}

static int rt9458_set_battery_vmreg(struct rt9458_info *info, u32 vmreg)
{
	int ret = 0;
	u8 reg_vmreg = 0;

	reg_vmreg = rt9458_find_closest_reg_value_via_table(
		rt9458_battery_vmreg,
		ARRAY_SIZE(rt9458_battery_vmreg),
		vmreg
	);

	pr_info("%s: vmreg = %d(0x%02X)\n", __func__, vmreg, reg_vmreg);

	ret = rt9458_i2c_update_bits(
		info,
		RT9458_REG_CTRL7,
		reg_vmreg << RT9458_SHIFT_VMREG,
		RT9458_MASK_VMREG
	);

	return ret;
}

static int rt9458_get_mivr(struct rt9458_info *info, u32 *mivr)
{
	int ret = 0;
	u8 reg_mivr = 0;

	ret = rt9458_i2c_read_byte(info, RT9458_REG_CTRL8);
	if (ret < 0)
		return ret;

	reg_mivr = ((ret & RT9458_MASK_MIVR) >> RT9458_SHIFT_MIVR) & 0xFF;
	*mivr = rt9458_find_closest_real_value(RT9458_MIVR_MIN, RT9458_MIVR_MAX,
		RT9458_MIVR_STEP, reg_mivr);

	return ret;
}

static int rt9458_parse_dt(struct rt9458_info *info, struct device *dev)
{
	struct rt9458_desc *desc = NULL;

	//struct device_node *np = dev->of_node;

	/* check customer setting */
	struct device_node *np; 
	np = of_find_compatible_node(NULL, NULL, "richtek,rt9458");

	pr_info("%s\n", __func__);

	if (!np) {
		pr_err("%s: no device node\n", __func__);
		return -EINVAL;
	}

	info->desc = &rt9458_default_desc;

	desc = devm_kzalloc(dev, sizeof(struct rt9458_desc), GFP_KERNEL);
	if (!desc) {
		pr_err("%s: no enough memory\n", __func__);
		return -ENOMEM;
	}
	memcpy(desc, &rt9458_default_desc, sizeof(struct rt9458_desc));

	if (of_property_read_string(np, "charger_name",
		&(info->mchr_info.name)) < 0) {
		battery_log(BAT_LOG_CRTI, "%s: no charger name\n", __func__);
		info->mchr_info.name = "primary_charger";
	}

	if (of_property_read_u32(np, "regmap_represent_slave_addr",
		&desc->regmap_represent_slave_addr) < 0)
		battery_log(BAT_LOG_CRTI, "%s: no regmap slave addr\n",
			__func__);

	if (of_property_read_string(np, "regmap_name",
		&(desc->regmap_name)) < 0)
		battery_log(BAT_LOG_CRTI, "%s: no regmap name\n", __func__);

	if (of_property_read_u32(np, "ichg", &desc->ichg) < 0)
		battery_log(BAT_LOG_CRTI, "%s: no ichg\n", __func__);

	if (of_property_read_u32(np, "aicr", &desc->aicr) < 0)
		battery_log(BAT_LOG_CRTI, "%s: no aicr\n", __func__);

	if (of_property_read_u32(np, "mivr", &desc->mivr) < 0)
		battery_log(BAT_LOG_CRTI, "%s: no mivr\n", __func__);

	if (of_property_read_u32(np, "cv", &desc->cv) < 0)
		battery_log(BAT_LOG_CRTI, "%s: no cv\n", __func__);

	if (of_property_read_u32(np, "ieoc", &desc->ieoc) < 0)
		battery_log(BAT_LOG_CRTI, "%s: no ieoc\n", __func__);

	desc->enable_te = of_property_read_bool(np, "enable_te");
	desc->enable_wdt = of_property_read_bool(np, "enable_wdt");

	info->desc = desc;

	return 0;
}

static int rt9458_get_charging_status(struct rt9458_info *info,
	enum rt9458_charging_status *chg_stat)
{
	int ret = 0;

	ret = rt9458_i2c_read_byte(info, RT9458_REG_CTRL1);
	if (ret < 0)
		return ret;

	*chg_stat = (ret & RT9458_MASK_CHG_STAT) >> RT9458_SHIFT_CHG_STAT;

	return ret;
}

static int rt9458_enable_te(struct rt9458_info *info, bool enable)
{
	int ret = 0;

	pr_info("%s: enable = %d\n", __func__, enable);

	ret = (enable ? rt9458_set_bit : rt9458_clr_bit)
		(info, RT9458_REG_CTRL2, RT9458_MASK_TE_EN);

	return ret;
}

static int rt9458_enable_te_shutdown(struct rt9458_info *info, bool enable)
{
	int ret = 0;

	pr_info("%s: enable = %d\n", __func__, enable);

	ret = (enable ? rt9458_set_bit : rt9458_clr_bit)
		(info, RT9458_REG_CTRL2, RT9458_MASK_TE_SHTDM_EN);

	return ret;
}

static int rt9458_enable_wdt(struct rt9458_info *info, bool enable)
{
	int ret = 0;

	pr_info("%s: enable = %d\n", __func__, enable);

	ret = (enable ? rt9458_set_bit : rt9458_clr_bit)
		(info, RT9458_REG_CTRL5, RT9458_MASK_TMR_EN);

	return ret;
}

#if 0 /* Uncomment this block if you need these functions */
static int rt9458_reset_chip(struct rt9458_info *info)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = rt9458_set_bit(info, RT9458_REG_CTRL4, RT9458_MASK_RST);

	return ret;
}
#endif

static int rt9458_set_ieoc(struct rt9458_info *info, u32 ieoc)
{
	int ret = 0;

	/* Find corresponding reg value */
	u8 reg_ieoc = rt9458_find_closest_reg_value(RT9458_IEOC_MIN,
		RT9458_IEOC_MAX, RT9458_IEOC_STEP, RT9458_IEOC_NUM, ieoc);

	pr_info("%s: ieoc = %d(0x%02X)\n", __func__, ieoc, reg_ieoc);

	ret = rt9458_i2c_update_bits(
		info,
		RT9458_REG_CTRL5,
		reg_ieoc << RT9458_SHIFT_IEOC,
		RT9458_MASK_IEOC
	);

	return ret;
}

static int rt9458_get_ieoc(struct rt9458_info *info, u32 *ieoc)
{
	int ret = 0;
	u8 reg_ieoc = 0;

	ret = rt9458_i2c_read_byte(info, RT9458_REG_CTRL5);
	if (ret < 0)
		return ret;

	reg_ieoc = (ret & RT9458_MASK_IEOC) >> RT9458_SHIFT_IEOC;
	*ieoc = rt9458_find_closest_real_value(RT9458_IEOC_MIN, RT9458_IEOC_MAX,
		RT9458_IEOC_STEP, reg_ieoc);

	return ret;
}

static int rt9458_is_charging_enable(struct rt9458_info *info, bool *enable)
{
	int ret = 0;

	ret = rt9458_i2c_test_bit(info, RT9458_REG_CTRL7, RT9458_SHIFT_CHG_EN,
		enable);

	return ret;
}

/* ================== */
/* released functions */
/* ================== */

static int rt_charger_enable_charging(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	bool enable = *((bool *)data);
	u32 aicr = 50000; /* 10uA */
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	pr_info("%s: enable = %d\n", __func__, enable);

	ret = (enable ? rt9458_clr_bit : rt9458_set_bit)
		(info, RT9458_REG_CTRL2, RT9458_MASK_HZ_EN);

	if (ret == 0 && enable == false)
		ret = rt_charger_set_aicr(mchr_info, &aicr);

	return ret;
}

static int rt_charger_enable_hz(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	bool enable = *((bool *)data);
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	pr_info("%s: enable = %d\n", __func__, enable);

	ret = (enable ? rt9458_set_bit : rt9458_clr_bit)
		(info, RT9458_REG_CTRL2, RT9458_MASK_HZ_EN);

	return ret;
}

static int rt_charger_enable_otg(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	bool enable = *((bool *)data);
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	pr_info("%s: enable = %d\n", __func__, enable);

	if (enable) {
		rt9458_clr_bit(info, RT9458_REG_CTRL2, RT9458_MASK_HZ_EN);
		rt9458_clr_bit(info, RT9458_REG_CTRL7, RT9458_MASK_CHG_EN);
	} else {
		rt9458_set_bit(info, RT9458_REG_CTRL2, RT9458_MASK_HZ_EN);
	}

	ret = (enable ? rt9458_set_bit : rt9458_clr_bit)
		(info, RT9458_REG_CTRL2, RT9458_MASK_OPA_MODE);

	return ret;
}

static int rt_charger_set_ichg(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u32 ichg = *((u32 *)data);
	u8 reg_ichg = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	/* MTK's current unit : 10uA */
	/* Our current unit : uA */
	ichg *= 10;

	/* Find corresponding reg value */
	reg_ichg = rt9458_find_closest_reg_value(RT9458_ICHG_MIN,
		RT9458_ICHG_MAX, RT9458_ICHG_STEP, RT9458_ICHG_NUM, ichg);

	pr_info("%s: ichg = %d(0x%02X)\n", __func__, ichg, reg_ichg);

	ret = rt9458_i2c_update_bits(
		info,
		RT9458_REG_CTRL6,
		reg_ichg << RT9458_SHIFT_ICHG,
		RT9458_MASK_ICHG
	);

	return ret;
}

static int rt_charger_set_aicr(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u32 aicr = *((u32 *)data);
	u8 reg_aicr = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	mutex_lock(&info->aicr_access_lock);

	/* MTK's current unit : 10uA */
	/* Our current unit : uA */
	aicr *= 10;

	if (aicr < 500000) /* AICR = 100 mA */
		reg_aicr = 0x00;
	else if (aicr < 700000) { /* AICR = 500 mA */
		ret = rt9458_set_aicr_sel(info, false);
		if (ret < 0)
			goto out;
		reg_aicr = 0x01;
	} else if (aicr < 1000000) { /* AICR = 700 mA */
		ret = rt9458_set_aicr_sel(info, true);
		if (ret < 0)
			goto out;
		reg_aicr = 0x01;
	} else if (aicr == 1000000) { /* AICR = 1000mA */
		ret = rt9458_set_aicr_sel(info, false);
		if (ret < 0)
			goto out;
		reg_aicr = 0x02;
	} else /* AICR = no limit */
		reg_aicr = 0x03;

	pr_info("%s: aicr = %d(0x%02X)\n", __func__, aicr, reg_aicr);

	ret = rt9458_i2c_update_bits(
		info,
		RT9458_REG_CTRL2,
		reg_aicr << RT9458_SHIFT_AICR,
		RT9458_MASK_AICR
	);

out:
	mutex_unlock(&info->aicr_access_lock);
	return ret;
}


static int rt_charger_set_mivr(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u32 mivr = *((u32 *)data);
	u8 reg_mivr = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	/* MTK's MIVR unit : mV */
	/* Our MIVR unit : uV */
	mivr *= 1000;

	reg_mivr = rt9458_find_closest_reg_value(RT9458_MIVR_MIN,
		RT9458_MIVR_MAX, RT9458_MIVR_STEP, RT9458_MIVR_NUM, mivr);

	pr_info("%s: mivr = %d(0x%02X)\n", __func__, mivr, reg_mivr);

	ret = rt9458_i2c_update_bits(
		info,
		RT9458_REG_CTRL8,
		reg_mivr << RT9458_SHIFT_MIVR,
		RT9458_MASK_MIVR
	);

	return ret;
}

static int rt_charger_set_battery_voreg(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u32 voreg = *((u32 *)data);
	u8 reg_voreg = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	reg_voreg = rt9458_find_closest_reg_value_via_table(
		rt9458_battery_voreg,
		ARRAY_SIZE(rt9458_battery_voreg),
		voreg
	);

	pr_info("%s: voreg = %d(0x%02X)\n", __func__, voreg, reg_voreg);

	ret = rt9458_i2c_update_bits(
		info,
		RT9458_REG_CTRL3,
		reg_voreg << RT9458_SHIFT_VOREG,
		RT9458_MASK_VOREG
	);

	return ret;
}

static int rt_charger_get_ichg(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u8 reg_ichg = 0;
	u32 ichg = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	ret = rt9458_i2c_read_byte(info, RT9458_REG_CTRL6);
	if (ret < 0)
		return ret;

	reg_ichg = (ret & RT9458_MASK_ICHG) >> RT9458_SHIFT_ICHG;
	ichg = rt9458_find_closest_real_value(RT9458_ICHG_MIN, RT9458_ICHG_MAX,
		RT9458_ICHG_STEP, reg_ichg);

	/* MTK's current unit : 10uA */
	/* Our current unit : uA */
	*((u32 *)data) = ichg / 10;

	return ret;
}

/* 0 stands for unlimited */
static int rt_charger_get_aicr(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	bool aicr_sel = false;
	u8 reg_aicr = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	mutex_lock(&info->aicr_access_lock);

	/* Read AICR sel */
	ret = rt9458_i2c_test_bit(info, RT9458_REG_CTRL6, RT9458_SHIFT_AICR_SEL,
		&aicr_sel);
	if (ret < 0)
		goto out;

	/* Read AICR */
	ret = rt9458_i2c_read_byte(info, RT9458_REG_CTRL2);
	if (ret < 0)
		goto out;
	reg_aicr = ((ret & RT9458_MASK_AICR) >> RT9458_SHIFT_AICR) & 0xFF;


	/* MTK's current unit : 10uA */
	switch (reg_aicr) {
	case 0:
		*((u32 *)data) = 10000;
		break;
	case 1:
		*((u32 *)data) = aicr_sel ? 70000 : 50000;
		break;
	case 2:
		*((u32 *)data) = aicr_sel ? 70000 : 100000;
		break;
	case 3:
		*((u32 *)data) = 0;
		break;
	}

out:
	mutex_unlock(&info->aicr_access_lock);
	return ret;
}


static int rt_charger_kick_wdt(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;
	enum rt9458_charging_status chg_status = RT9458_CHG_STATUS_READY;

	/* Any I2C communication kicks wdt */
	ret = rt9458_get_charging_status(info, &chg_status);

	return ret;
}

static int rt_charger_dump_register(struct mtk_charger_info *mchr_info,
	void *data)
{
	int i = 0, ret = 0;
	u32 ichg = 0, aicr = 0, mivr = 0, ieoc = 0;
	bool chg_enable = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;
	enum rt9458_charging_status chg_status = RT9458_CHG_STATUS_READY;

	ret = rt9458_get_charging_status(info, &chg_status);
	for (i = 0; i < ARRAY_SIZE(rt9458_reg_addr); i++)
		ret = rt9458_i2c_read_byte(info, rt9458_reg_addr[i]);

	ret = rt_charger_get_ichg(&info->mchr_info, &ichg);
	ret = rt9458_get_mivr(info, &mivr);
	ret = rt_charger_get_aicr(&info->mchr_info, &aicr);
	ret = rt9458_get_ieoc(info, &ieoc);
	ret = rt9458_is_charging_enable(info, &chg_enable);

	pr_info("%s: ICHG = %dmA, AICR = %d%smA, MIVR = %dmV, IEOC = %dmA\n",
		__func__, ichg / 100, aicr / 100,
		(aicr == 0 ? "(unlimited)" : ""), mivr / 1000, ieoc / 1000);

	pr_info("%s: CHG_EN = %d, CHG_STATUS = %s\n",
		__func__, chg_enable, rt9458_chg_status_name[chg_status]);

	return ret;
}
int mtk_charger_sw_init1(struct mtk_charger_info *mchr_info, void *data)
{
		int i = 0, ret = 0;
		u32 ichg = 0, aicr = 0, mivr = 0, ieoc = 0;
		bool chg_enable = 0;
		bool chg_force_disable = 0;
		struct rt9458_info *info = (struct rt9458_info *)mchr_info;
		enum rt9458_charging_status chg_status = RT9458_CHG_STATUS_READY;

		ret = rt9458_set_aicr_int(info, true);
		ret = rt9458_set_battery_vmreg(info, 4450000);
		
		ret = rt9458_get_charging_status(info, &chg_status);
		if (chg_status == RT9458_CHG_STATUS_FAULT) {
			for (i = 0; i < ARRAY_SIZE(rt9458_reg_addr); i++)
				ret = rt9458_i2c_read_byte(info, rt9458_reg_addr[i]);
		}


	
		ret = rt_charger_get_ichg(&info->mchr_info, &ichg);
		      rt_charger_set_ichg(&info->mchr_info, &ichg);
		
		ret = rt9458_get_mivr(info, &mivr);
		      rt_charger_set_mivr(&info->mchr_info, &mivr);
		
		ret = rt_charger_get_aicr(&info->mchr_info, &aicr);
		      rt_charger_set_aicr(&info->mchr_info, &aicr);
			  
		ret = rt9458_get_ieoc(info, &ieoc);
		      rt9458_set_ieoc(info, ieoc);
		
		ret = rt9458_is_charging_enable(info, &chg_enable);
		rt_charger_enable_hz(&info->mchr_info,&chg_force_disable);
		rt_charger_enable_otg(&info->mchr_info,&chg_force_disable);
		//rt_charger_enable_charging(&info->mchr_info,&chg_force_enable);
	
		pr_info("%s: sunjie ICHG = %dmA, AICR = %d%smA, MIVR = %dmV, IEOC = %dmA\n",
			__func__, ichg / 100, aicr / 100,
			(aicr == 0 ? "(unlimited)" : ""), mivr / 1000, ieoc / 1000);
	
		pr_info("%s: sunjie CHG_EN = %d, CHG_STATUS = %s\n",
			__func__, chg_enable, rt9458_chg_status_name[chg_status]);
	


	return 0;
}

static int rt9458_init_setting(struct rt9458_info *info)
{
	int ret = 0;
	struct rt9458_desc *desc = info->desc;

	pr_info("%s\n", __func__);

	/* Set AICR to internal */
	ret = rt9458_set_aicr_int(info, true);
	if (ret < 0)
		pr_info("%s: set AICR int failed\n", __func__);

	/* Set VMREG to maximum, uV */
	ret = rt9458_set_battery_vmreg(info, 4450000);
	if (ret < 0)
		pr_info("%s: set vmreg failed\n", __func__);

	ret = rt_charger_set_battery_voreg(&info->mchr_info, &desc->cv);
	if (ret < 0)
		pr_info("%s: set cv failed\n", __func__);

	ret = rt_charger_set_ichg(&info->mchr_info, &desc->ichg);
	if (ret < 0)
		pr_info("%s: set ichg failed\n", __func__);

	ret = rt_charger_set_aicr(&info->mchr_info, &desc->aicr);
	if (ret < 0)
		pr_info("%s: set aicr failed\n", __func__);

	ret = rt_charger_set_mivr(&info->mchr_info, &desc->mivr);
	if (ret < 0)
		pr_info("%s: set mivr failed\n", __func__);

	ret = rt9458_set_ieoc(info, desc->ieoc);
	if (ret < 0)
		pr_info("%s: set ieoc failed\n", __func__);

	ret = rt9458_enable_te(info, desc->enable_te);
	if (ret < 0)
		pr_info("%s: set te failed\n", __func__);

	ret = rt9458_enable_te_shutdown(info, desc->enable_te);
	if (ret < 0)
		pr_info("%s: set te shutdown failed\n", __func__);

	ret = rt9458_enable_wdt(info, desc->enable_wdt);
	if (ret < 0)
		pr_info("%s: set wdt failed\n", __func__);

	return ret;
}

static int rt_charger_is_charging_done(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	enum rt9458_charging_status chg_stat = RT9458_CHG_STATUS_READY;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	ret = rt9458_get_charging_status(info, &chg_stat);

	/* Return is charging done or not */
	switch (chg_stat) {
		case RT9458_CHG_STATUS_READY:
			*((u32 *)data) = 2;
			break;
		case RT9458_CHG_STATUS_PROGRESS:
			*((u32 *)data) = 2;
			break;
		case RT9458_CHG_STATUS_FAULT:
			*((u32 *)data) = 0;
			break;
		case RT9458_CHG_STATUS_DONE:
			*((u32 *)data) = 1;
			break;
		default:
			*((u32 *)data) = 0;
			break;
	}

	return ret;
}

static int rt_charger_run_aicl(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	bool mivri = false, mivr_stat = false;
	u32 aicr = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	pr_info("%s\n", __func__);

	/* Check MIVR IRQ */
	ret = rt9458_i2c_test_bit(info, RT9458_REG_IRQ2, RT9458_SHIFT_CHMIVRI,
		&mivri);
	if (ret < 0)
		return ret;

	pr_info("%s: mivri = %d\n", __func__, mivri);
	if (!mivri)
		return ret;

	/* Check MIVR stat */
	ret = rt9458_i2c_test_bit(info, RT9458_REG_IRQSTATUS,
		RT9458_SHIFT_MIVR_STAT, &mivr_stat);
	if (ret < 0)
		return ret;

	pr_info("%s: mivr_stat = %d\n", __func__, mivr_stat);
	if (!mivr_stat)
		return ret;

	ret = rt_charger_get_aicr(mchr_info, &aicr);
	if (ret < 0)
		return ret;

	if (aicr == 0) /* from unlimited to 1000mA */
		aicr = 100000;
	else if (aicr == 100000) /* from 1000mA to 700mA */
		aicr = 70000;
	else if (aicr == 70000) /* from 700mA to 500mA */
		aicr = 50000;
//	else if (aicr == 50000) /* from 500mA to 100mA */
//		aicr = 10000;
	else { /* already the smallest value */
		pr_err("%s: AICR is %dmA\n", __func__, aicr / 100);
		return ret;
	}

	pr_info("%s: set aicr to %dmA\n", __func__, aicr / 100);
	ret = rt_charger_set_aicr(mchr_info, &aicr);
	if (ret < 0)
		return ret;

	//*((u32 *)data) = aicr;
	/* This API's unit is mA */
	*((u32 *)data) = aicr / 100;

	return ret;
}

static int rt_charger_set_error_state(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	struct rt9458_info *info = (struct rt9458_info *)mchr_info;

	info->err_state = *((bool *)data);
	ret = rt_charger_enable_hz(mchr_info, &info->err_state);

	return ret;
}

static const mtk_charger_intf rt9458_mchr_intf[CHARGING_CMD_NUMBER] = {
	[CHARGING_CMD_DUMP_REGISTER] = rt_charger_dump_register,
	[CHARGING_CMD_ENABLE] = rt_charger_enable_charging,
	[CHARGING_CMD_SET_HIZ_SWCHR] = rt_charger_enable_hz,
	[CHARGING_CMD_ENABLE_OTG] = rt_charger_enable_otg,
	[CHARGING_CMD_SET_CURRENT] = rt_charger_set_ichg,
	[CHARGING_CMD_SET_INPUT_CURRENT] = rt_charger_set_aicr,
	[CHARGING_CMD_SET_VINDPM] = rt_charger_set_mivr,
	[CHARGING_CMD_SET_CV_VOLTAGE] = rt_charger_set_battery_voreg,
	[CHARGING_CMD_SET_ERROR_STATE] = rt_charger_set_error_state,
	[CHARGING_CMD_GET_CURRENT] = rt_charger_get_ichg,
	[CHARGING_CMD_GET_INPUT_CURRENT] = rt_charger_get_aicr,
	[CHARGING_CMD_GET_CHARGING_STATUS] = rt_charger_is_charging_done,
	[CHARGING_CMD_RESET_WATCH_DOG_TIMER] = rt_charger_kick_wdt,
	[CHARGING_CMD_RUN_AICL] = rt_charger_run_aicl,

	/*
	 * The following interfaces are not related to charger
	 * Define in mtk_charger_intf.c
	 */
	[CHARGING_CMD_SW_INIT] = mtk_charger_sw_init1,
	[CHARGING_CMD_SET_HV_THRESHOLD] = mtk_charger_set_hv_threshold,
	[CHARGING_CMD_GET_HV_STATUS] = mtk_charger_get_hv_status,
	[CHARGING_CMD_GET_BATTERY_STATUS] = mtk_charger_get_battery_status,
	[CHARGING_CMD_GET_CHARGER_DET_STATUS] = mtk_charger_get_charger_det_status,
	[CHARGING_CMD_GET_CHARGER_TYPE] = mtk_charger_get_charger_type,
	[CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER] = mtk_charger_get_is_pcm_timer_trigger,
	[CHARGING_CMD_SET_PLATFORM_RESET] = mtk_charger_set_platform_reset,
	[CHARGING_CMD_GET_PLATFORM_BOOT_MODE] = mtk_charger_get_platform_boot_mode,
	[CHARGING_CMD_SET_POWER_OFF] = mtk_charger_set_power_off,
	[CHARGING_CMD_GET_POWER_SOURCE] = mtk_charger_get_power_source,
	[CHARGING_CMD_GET_CSDAC_FALL_FLAG] = mtk_charger_get_csdac_full_flag,
	[CHARGING_CMD_DISO_INIT] = mtk_charger_diso_init,
	[CHARGING_CMD_GET_DISO_STATE] = mtk_charger_get_diso_state,
	[CHARGING_CMD_SET_VBUS_OVP_EN] = mtk_charger_set_vbus_ovp_en,
	[CHARGING_CMD_GET_BIF_VBAT] = mtk_charger_get_bif_vbat,
	[CHARGING_CMD_SET_CHRIND_CK_PDN] = mtk_charger_set_chrind_ck_pdn,
	[CHARGING_CMD_GET_BIF_TBAT] = mtk_charger_get_bif_tbat,
	[CHARGING_CMD_SET_DP] = mtk_charger_set_dp,
	[CHARGING_CMD_GET_BIF_IS_EXIST] = mtk_charger_get_bif_is_exist,
};


/* ========================= */
/* i2c driver function */
/* ========================= */
static int rt9458_probe(struct i2c_client *client,
	const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct rt9458_info *info = NULL;

	pr_info("%s(%s)\n", __func__, RT9458_DRV_VERSION);

	info = devm_kzalloc(&client->dev, sizeof(struct rt9458_info),
		GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->i2c = client;
	mutex_init(&info->i2c_access_lock);
	mutex_init(&info->aicr_access_lock);

	/* Is HW exist */
	if (!rt9458_is_hw_exist(info)) {
		pr_info("%s: no rt9458 exists\n", __func__);
		ret = -ENODEV;
		goto err_nodev;
	}
	i2c_set_clientdata(client, info);

	/* Parse default configuration from dts */
	ret = rt9458_parse_dt(info, &client->dev);
	if (ret < 0)
		goto err_parse_dt;

#ifdef CONFIG_RT_REGMAP
	ret = rt9458_register_rt_regmap(info);
	if (ret < 0)
		goto err_register_regmap;
#endif

	ret = rt9458_init_setting(info);
	if (ret < 0) {
		pr_info("%s: init setting failed\n", __func__);
		goto err_init_setting;
	}

	rt_charger_dump_register(&info->mchr_info, NULL);

	/* Hook chr_control_interface with battery's interface */
	info->mchr_info.mchr_intf = rt9458_mchr_intf;
	mtk_charger_set_info(&info->mchr_info);
		
	if (battery_charging_control == NULL) {
		battery_charging_control = rt9458_chr_control_interface;
	}		
		
	chargin_hw_init_done = KAL_TRUE;

	pr_info("%s: ends\n", __func__);

	return ret;

err_init_setting:
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(info->regmap_dev);
err_register_regmap:
#endif
err_nodev:
err_parse_dt:
	mutex_destroy(&info->i2c_access_lock);
	mutex_destroy(&info->aicr_access_lock);
	return ret;
}


static int rt9458_remove(struct i2c_client *client)
{
	int ret = 0;
	struct rt9458_info *info = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);

	if (info) {
#ifdef CONFIG_RT_REGMAP
		rt_regmap_device_unregister(info->regmap_dev);
#endif
		mutex_destroy(&info->i2c_access_lock);
		mutex_destroy(&info->aicr_access_lock);
	}

	return ret;
}

static const struct i2c_device_id rt9458_i2c_id[] = {
	{"rt9458", 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id rt9458_of_match[] = {
	{ .compatible = "mediatek,swithing_charger", },
	{},
};
MODULE_DEVICE_TABLE(of, rt9458_of_match);
#else

#define RT9458_BUSNUM 1

static struct i2c_board_info rt9458_i2c_board_info __initdata = {
	I2C_BOARD_INFO("rt9458", RT9458_SALVE_ADDR)
};
#endif /* CONFIG_OF */


static struct i2c_driver rt9458_i2c_driver = {
	.driver = {
		.name = "rt9458",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = rt9458_of_match,
#endif
	},
	.probe = rt9458_probe,
	.remove = rt9458_remove,
	.id_table = rt9458_i2c_id,
};

static int __init rt9458_init(void)
{
	int ret = 0;

#ifdef CONFIG_OF
	pr_info("%s: with dts\n", __func__);
#else
	pr_info("%s: without dts\n", __func__);
	i2c_register_board_info(RT9458_BUSNUM, &rt9458_i2c_board_info, 1);
#endif

	ret = i2c_add_driver(&rt9458_i2c_driver);
	if (ret < 0)
		pr_info("%s: register i2c driver failed\n", __func__);

	return ret;
}
subsys_initcall(rt9458_init);


static void __exit rt9458_exit(void)
{
	i2c_del_driver(&rt9458_i2c_driver);
	return;
}
module_exit(rt9458_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("ShuFanLee <shufan_lee@richtek.com>");
MODULE_DESCRIPTION("RT9458 Charger Driver");
MODULE_VERSION(RT9458_DRV_VERSION);

/*
 * Revision Note
 * 1.0.6
 * (1) Check MIVR stat after checking MIVR IRQ
 *
 * 1.0.5
 * (1) Fix rt_charger_enable_hz, modify mask of CHG_EN to HZ_EN
 * (2) Use HZ to disable charger
 *
 * 1.0.4
 * (1) For aicl_run, modify unit to mA and the lowest aicr to 500mA
 * (2) Fix checkpatch error
 *
 * 1.0.3
 * (1) Refactoring
 */
