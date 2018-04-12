/*
 * Copyright (C) 2015 MediaTek Inc.
 * Copyright (C) 2017 FIH Mobile Limited.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * NOTE: This file has been modified by FIH Mobile Limited.
 * Modifications are licensed under the License.
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif

#define LDO_GEN1(_supply,min_voltage,max_voltage)	\
	{	\
		.supply_name= #_supply,	\
		.min_uV= (min_voltage),	\
		.max_uV= (max_voltage),	\
	}

struct lcm_device {
	struct regulator *reg;
	char *supply_name;
	int min_uV;
	int max_uV;
};

static struct lcm_device lcm_ldos[] = {
#if 0
	{NULL,"vdd28",2800000,2800000},
	{NULL,"vddi18",2800000,2800000},
#else
	LDO_GEN1(vdd28,2800000,2800000),
	LDO_GEN1(vddi18,1800000,1800000),
#endif
};

/* get(vgp6) LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int i;
	int ret = 0;

	pr_debug("LCM: lcm_get_vgp_supply is going\n");

	for (i = 0; i < ARRAY_SIZE(lcm_ldos); i++)
	{
		pr_debug("LCM: lcm_get_vgp_supply is going lcm_ldos[%d]->supply_name=%s\n",i,lcm_ldos[i].supply_name);
		//lcm_ldos[i].reg = devm_regulator_get(dev, lcm_ldos[i].supply_name);
#if 1
		lcm_ldos[i].reg = regulator_get(dev, lcm_ldos[i].supply_name);
#endif
		if (NULL == lcm_ldos[i].reg || IS_ERR(lcm_ldos[i].reg)) {
			ret = PTR_ERR(lcm_ldos[i].reg);
			pr_debug("failed to get LCM LDO=%s, err=%d\n",lcm_ldos[i].supply_name, ret);
			continue;
		}

		pr_debug("LCM: lcm get supply ok.\n");

		/* get current voltage settings */
		ret = regulator_get_voltage(lcm_ldos[i].reg);
		pr_debug("lcm LDO voltage = %d in LK stage\n", ret);
	}

	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int i;
	int ret = 0;
	unsigned int volt;

	pr_debug("LCM: lcm_vgp_supply_enable\n");

	for (i = 0; i < ARRAY_SIZE(lcm_ldos); i++)
	{
		if (NULL == lcm_ldos[i].reg || IS_ERR(lcm_ldos[i].reg) )
		{
			pr_debug("LCM: lcm_ldos[%d]->supply_name=%s is going wrong \n",i,lcm_ldos[i].supply_name);
			continue;
		}
		pr_debug("LCM: lcm_ldos[%d]->supply_name=%s is going right \n",i,lcm_ldos[i].supply_name);
		pr_debug("LCM: set regulator voltage lcm_vgp voltage to 2.8V\n");
		/* set(vgp1) voltage to 2.8V */
		ret = regulator_set_voltage(lcm_ldos[i].reg, lcm_ldos[i].min_uV, lcm_ldos[i].max_uV);
		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
			continue;
		}

		/* get(vgp1) voltage settings again */
		volt = regulator_get_voltage(lcm_ldos[i].reg);
		if (volt == lcm_ldos[i].min_uV)
			pr_debug("LCM: check regulator voltage=%d pass!\n",lcm_ldos[i].min_uV);
		else
			pr_err("LCM: check regulator voltage=%d fail! (voltage: %d)\n",lcm_ldos[i].min_uV, volt);

		ret = regulator_enable(lcm_ldos[i].reg);
		if (ret != 0) {
			pr_err("LCM: Failed to enable %s: %d\n",lcm_ldos[i].supply_name, ret);
			continue;
		}
	}

	return ret;
}

int lcm_vgp_supply_disable(void)
{
	int i;
	int ret = 0;
	unsigned int isenable;

	pr_debug("LCM: lcm_vgp_supply_disable\n");

	for (i = 0; i < ARRAY_SIZE(lcm_ldos); i++)
	{
		if (NULL == lcm_ldos[i].reg || IS_ERR(lcm_ldos[i].reg) )
		{
			pr_debug("LCM: lcm_ldos[%d]->supply_name=%s is going wrong \n",i,lcm_ldos[i].supply_name);
			continue;
		}
		pr_debug("LCM: lcm_ldos[%d]->supply_name=%s is going right \n",i,lcm_ldos[i].supply_name);

		/* disable regulator */
		isenable = regulator_is_enabled(lcm_ldos[i].reg);

		pr_debug("LCM: lcm query regulator enable status[0x%d]\n", isenable);

		if (isenable) {
			ret = regulator_disable(lcm_ldos[i].reg);
			if (ret != 0) {
				pr_err("LCM: lcm failed to disable %s: %d\n", lcm_ldos[i].supply_name, ret);
				continue;
			}
			/* verify */
			isenable = regulator_is_enabled(lcm_ldos[i].reg);
			if (!isenable)
				pr_debug("LCM: lcm regulator disable pass\n");
		}
	}

	return ret;
}

/*
void lcm_request_gpio_control(void)
{
	gpio_request(GPIO_LCD_PWR_EN, "GPIO_LCD_PWR_EN");
	pr_debug("[KE/LCM] GPIO_LCD_PWR_EN = 0x%x\n", GPIO_LCD_PWR_EN);

	gpio_request(GPIO_LCD_PWR2_EN, "GPIO_LCD_PWR2_EN");
	pr_debug("[KE/LCM] GPIO_LCD_PWR2_EN = 0x%x\n", GPIO_LCD_PWR2_EN);

	gpio_request(GPIO_LCD_RST_EN, "GPIO_LCD_RST_EN");
	pr_debug("[KE/LCM] GPIO_LCD_RST_EN = 0x%x\n", GPIO_LCD_RST_EN);

	gpio_request(GPIO_LCD_STB_EN, "GPIO_LCD_STB_EN");
	pr_debug("[KE/LCM] GPIO_LCD_STB_EN =  0x%x\n", GPIO_LCD_STB_EN);

	gpio_request(GPIO_LCD_BRIDGE_EN, "GPIO_LCD_BRIDGE_EN");
	pr_debug("[KE/LCM] GPIO_LCD_BRIDGE_EN = 0x%x\n", GPIO_LCD_BRIDGE_EN);
}*/

static int lcm_probe(struct device *dev)
{
	lcm_get_vgp_supply(dev);
	lcm_vgp_supply_enable();
	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,lcm",},
	{}
};

static struct platform_driver lcm_driver = {
//	.probe = lcm_probe,
	.driver = {
		   .name = "mtk_lcm",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_debug("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}


static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_debug("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
