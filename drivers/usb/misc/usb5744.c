// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Microchip USB5744 4-port hub.
 *
 * Copyright (c) 2021 Xilinx, Inc.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/byteorder/generic.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>

/* chip address - usb5744 */
#define I2C_ADDR		0x2d
/* chip type and driver name */
#define DRIVER_NAME		"usb5744"

struct usb5744 {
	struct gpio_desc *reset_gpio;
};

static int usb5744_init_hw(struct device *dev, struct usb5744 *data)
{
	data = devm_kzalloc(dev, sizeof(struct usb5744), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(data->reset_gpio)) {
		return dev_err_probe(dev, PTR_ERR(data->reset_gpio),
				     "Failed to request reset GPIO\n");
	}

	/* Toggle RESET_N to reset the hub. */
	gpiod_set_value_cansleep(data->reset_gpio, 1);
	/* Delay - Sleep for an approximate time 5 to 20 usecs */
	usleep_range(5, 20);
	gpiod_set_value_cansleep(data->reset_gpio, 0);
	msleep(5);

	return 0;
}

static int usb5744_i2c_dev_init(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct usb5744 *data = NULL;
	int ret = 0;
	/*
	 *  Prevent the MCU from the putting the HUB in suspend mode through register write.
	 *  The BYPASS_UDC_SUSPEND bit (Bit 3) of the RuntimeFlags2 register at address
	 *  0x411D controls this aspect of the hub.
	 *  Format to write to hub registers via SMBus- 00 00 05 00 01 41 1D 08
	 *  Byte 1: Command / Memory address 00
	 *  Byte 2: Memory address 00
	 *  Byte 3: Number of bytes to write to memory
	 *  Byte 4: Write configuration register (00)
	 *  Byte 5: Write the number of data bytes (01- 1 data byte)
	 *  Byte 6: LSB of register address 0x41
	 *  Byte 7: MSB of register address 0x1D
	 *  Byte 8: value to be written to the register
	 */
	char wr_buf[7] = {0x00, 0x05, 0x00, 0x01, 0x41, 0x1D, 0x08};

	i2c_set_clientdata(client, data);

	/* Trigger gpio reset to the hub. */
	ret = usb5744_init_hw(dev, data);
	if (ret)
		return ret;

	ret = i2c_smbus_write_block_data(client, 0, sizeof(wr_buf), wr_buf);
	if (ret)
		return dev_err_probe(dev, ret, "BYPASS_UDC_SUSPEND bit configuration failed\n");

	ret = i2c_smbus_write_word_data(client, 0x99, htons(0x3700));
	if (ret)
		return dev_err_probe(dev, ret, "Configuration Register Access Command failed\n");

	/* Send SMBus command to boot hub. */
	ret = i2c_smbus_write_word_data(client, 0xAA, htons(0x5600));
	if (ret < 0)
		return dev_err_probe(dev, ret, "USB Attach with SMBus command failed\n");

	return ret;
}

static int usb5744_i2c_probe(struct i2c_client *client)
{
	/* I2C device init and gpio reset to the hub. */
	return usb5744_i2c_dev_init(client);
}

static int usb5744_platform_probe(struct platform_device *pdev)
{
	struct usb5744 *data = NULL;
	struct device_node *i2c_node;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info = {
		.type = DRIVER_NAME,
		.addr = I2C_ADDR,
	};

	i2c_node = of_parse_phandle(pdev->dev.of_node, "i2c-bus", 0);
	if (i2c_node) {
		adapter = of_find_i2c_adapter_by_node(i2c_node);
		of_node_put(i2c_node);

		if (!adapter)
			return -EPROBE_DEFER;

		client = i2c_new_client_device(adapter, &info);
		return usb5744_i2c_dev_init(client);
	}

	/* Trigger gpio reset to the hub. */
	return usb5744_init_hw(&pdev->dev, data);
}

static const struct i2c_device_id usb5744_id[] = {
	{ DRIVER_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, usb5744_id);

static struct i2c_driver usb5744_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = usb5744_i2c_probe,
	.id_table = usb5744_id,
};

static const struct of_device_id usb5744_platform_id[] = {
	{ .compatible = "microchip,usb5744", },
	{ }
};

static struct platform_driver usb5744_platform_driver = {
	.driver = {
		.name = "microchip,usb5744",
		.of_match_table	= usb5744_platform_id,
	},
	.probe = usb5744_platform_probe,
};

static int __init usb5744_init(void)
{
	int err;

	err = i2c_add_driver(&usb5744_i2c_driver);
	if (err != 0)
		pr_err("usb5744: Failed to register I2C driver: %d\n", err);

	err = platform_driver_register(&usb5744_platform_driver);
	if (err != 0)
		pr_err("usb5744: Failed to register platform driver: %d\n",
		       err);
	return 0;
}
module_init(usb5744_init);

static void __exit usb5744_exit(void)
{
	platform_driver_unregister(&usb5744_platform_driver);
	i2c_del_driver(&usb5744_i2c_driver);
}
module_exit(usb5744_exit);

MODULE_AUTHOR("Piyush Mehta <piyush.mehta@xilinx.com>");
MODULE_DESCRIPTION("USB5744 Hub");
MODULE_LICENSE("GPL");
