/*
 * Driver for IRPS5401 chip
 *
 * Copyright (C) 2018 Xilinx, Inc. All rights reserved.
 *
 *
 * Description:
 * This driver is to program IRPS5401 chip through I2C.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define AVTT_SLAVE_REG_OFFSET       0x20
#define DAC_AVTT_FAULT_LIMIT_OFFSET 0x40
#define DAV_AVTT_MAX_OFFSET         0x24
#define DAC_AVTT_SEL_OFFSET         0x21
#define VTT_CONFIG_REG			    0x00

/**
 * struct pwr_irps5401
 * @regmap:	Device's regmap
 * @i2c_client:	I2C client pointer
 */
struct pwr_irps5401 {
	struct regmap *regmap;
	unsigned long volt;
	struct i2c_client *i2c_client;
};

static int iic_write(struct regmap *regmap, u8 reg_addr, u8 *data, u8 size)
{
	u32 ret;

	ret = regmap_bulk_write(regmap, reg_addr, data, size);
	if (ret)
		pr_err("i2c write failed for register offset : 0x%x", reg_addr);

	return ret;
}

static int irps5401_set_32v(struct pwr_irps5401 *data)
{
	u8 tx_array[5];
	u32 ret;

	tx_array[0] = (u8) 0x18 & (0xff);
	ret = iic_write(data->regmap, AVTT_SLAVE_REG_OFFSET, tx_array, 0x1);
	if (ret)
		goto err;

	tx_array[0] = (u8) 0x80 & (0xff);
	tx_array[1] = (u8) 0x03 & (0xff);
	ret = iic_write(data->regmap, DAC_AVTT_FAULT_LIMIT_OFFSET, tx_array,
			0x2);
	if (ret)
		goto err;

	tx_array[0] = (u8) 0x33 & (0xff);
	tx_array[1] = (u8) 0x03 & (0xff);
	ret = iic_write(data->regmap, DAV_AVTT_MAX_OFFSET, tx_array, 0x2);
	if (ret)
		goto err;

	tx_array[0] = (u8) 0x00 & (0xff);
	tx_array[1] = (u8) 0x03 & (0xff);
	ret = iic_write(data->regmap, DAC_AVTT_SEL_OFFSET, tx_array, 0x2);
	if (ret)
		goto err;

	return ret;
err:
	pr_err("%s() failed\n", __func__);
	return ret;
}

static int irps5401_set_25v(struct pwr_irps5401 *data)
{
	u8 tx_array[5];
	u32 ret;

	tx_array[0] = (u8) 0x80 & (0xff);
	tx_array[1] = (u8) 0x02 & (0xff);
	ret = iic_write(data->regmap, DAC_AVTT_SEL_OFFSET, tx_array, 0x2);
	if (ret)
		goto err;

	tx_array[0] = (u8) 0xcd & (0xff);
	tx_array[1] = (u8) 0x02 & (0xff);
	ret = iic_write(data->regmap, DAC_AVTT_FAULT_LIMIT_OFFSET, tx_array,
			0x2);
	if (ret)
		goto err;

	tx_array[0] = (u8) 0xb3 & (0xff);
	tx_array[1] = (u8) 0x02 & (0xff);
	ret = iic_write(data->regmap, DAV_AVTT_MAX_OFFSET, tx_array, 0x2);
	if (ret)
		goto err;
	return ret;
err:
	pr_err("%s() failed\n", __func__);
	return ret;
}

/*
 * irps5401_update_voltage() - PMIC output voltage
 * @data:	Driver data structure
 */
static int irps5401_update_voltage(struct pwr_irps5401 *data)
{
	u8 tx_array[5];
	u32 val;
	int ret = 0;

	tx_array[0] = (u8) 0x01 & (0xFF);
	iic_write(data->regmap, VTT_CONFIG_REG, tx_array, 0x1);
	if (ret)
		goto err;

	if (data->volt == 25) {
		ret = irps5401_set_25v(data);
	} else if (data->volt == 30) {
		ret = irps5401_set_32v(data);
	} else {
		pr_err("0x%x value is not supported\n", data->volt);
		ret = 1;
	}

	return ret ? (-1) : 1;
err:
	pr_err("%s() failed\n", __func__);
	return -1;
}

static const struct regmap_config irps5401_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0x41,
};

static ssize_t voltage_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long volt;
	ssize_t status;
	struct i2c_client *client = to_i2c_client(dev);
	struct pwr_irps5401 *data = i2c_get_clientdata(client);

	status = kstrtoul(buf, 16, &volt);
	data->volt = volt;

	status = irps5401_update_voltage(data);
	if (status < 0)
		return status;
	return size;
}
static DEVICE_ATTR_WO(voltage);

static const struct attribute *irps_attrs[] = {
	&dev_attr_voltage.attr,
	NULL,
};

static const struct attribute_group irps_attributes = {
	.attrs = (struct attribute **)irps_attrs,
};
static int irps5401_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct pwr_irps5401 *data;
	struct clk_init_data init;
	u32 initial_fout;
	int err;
	struct device *dev = &client->dev;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->i2c_client = client;

	data->regmap = devm_regmap_init_i2c(client, &irps5401_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(data->regmap);
	}

	i2c_set_clientdata(client, data);

	/*
	 * Create sysfs file entries for the device
	 */
	err = sysfs_create_group(&dev->kobj, &irps_attributes);
	if (err < 0)
		return err;
	return 0;
}

static int irps5401_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &irps_attributes);
	return 0;
}

enum clk_lmx2594_variant {
	irps5401
};

static const struct i2c_device_id irps5401_id[] = {
	{ "irps5401", irps5401 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, irsp5401_id);

static const struct of_device_id irps5401_of_match[] = {
	{ .compatible = "infineon,irps5401" },
	{ },
};
MODULE_DEVICE_TABLE(of, irps5401_of_match);

static struct i2c_driver irps5401_driver = {
	.driver = {
		.name = "irps5401",
		.of_match_table = irps5401_of_match,
	},
	.probe		= irps5401_probe,
	.remove		= irps5401_remove,
	.id_table	= irps5401_id,
};
module_i2c_driver(irps5401_driver);

MODULE_AUTHOR("Bhargav Shah <bhargavs@xilinx.com>");
MODULE_DESCRIPTION("irps5401 driver");
MODULE_LICENSE("GPL");
