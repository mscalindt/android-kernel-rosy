/*
 * Copyright (C) 2016 AWINIC Technology CO., LTD
 * Copyright (C) 2020 Dimitar Yurukov <mscalindt@protonmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)		"aw87319: " fmt

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#define AW87319_I2C_NAME	"AW87319_PA"
#define AW87319_I2C_BUS		0
#define AW87319_I2C_ADDR	0x58

#define AW87319_VER_MAJOR	0
#define AW87319_VER_MINOR	9

#define CHIP_ID_READ_RETRIES	5
#define CHIP_ID_RETRY_DELAY	10

int aw87319_rst;
static unsigned char chip_id;

unsigned char AW87319_Audio_Speaker(void);
unsigned char AW87319_Audio_OFF(void);

struct i2c_client *aw87319_pa_client;

unsigned char I2C_write_reg(unsigned char addr, unsigned char reg_data)
{
	u8 wdbuf[512] = {0};
	char ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= aw87319_pa_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= wdbuf,
		},
	};

	wdbuf[0] = addr;
	wdbuf[1] = reg_data;

	if (aw87319_pa_client == NULL) {
		pr_err("aw87319_pa_client is NULL\n");
		return -EPERM;
	}

	ret = i2c_transfer(aw87319_pa_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("i2c read error: %d\n", ret);

	return ret;
}

unsigned char I2C_read_reg(unsigned char addr)
{
	u8 rdbuf[512] = {0};
	unsigned char ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= aw87319_pa_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= aw87319_pa_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	rdbuf[0] = addr;

	if (aw87319_pa_client == NULL) {
		pr_err("aw87319_pa_client is NULL\n");
		return -EPERM;
	}

	ret = i2c_transfer(aw87319_pa_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("i2c read error: %d\n", ret);

	return rdbuf[0];
}

static void aw87319_pa_pwron(void)
{
	gpio_direction_output(aw87319_rst, true);
	msleep(10);
	I2C_write_reg(0x64, 0x2C);
}

static void aw87319_pa_pwroff(void)
{
	I2C_write_reg(0x01, 0x00);
	gpio_direction_output(aw87319_rst, false);
	msleep(1);
}

unsigned char AW87319_Audio_Speaker(void)
{
	aw87319_pa_pwron();

	I2C_write_reg(0x02, 0x28);
	I2C_write_reg(0x03, 0x05);
	I2C_write_reg(0x04, 0x04);
	I2C_write_reg(0x05, 0x0D);
	I2C_write_reg(0x06, 0x05);
	I2C_write_reg(0x07, 0x52);
	I2C_write_reg(0x08, 0x28);
	I2C_write_reg(0x09, 0x02);

	I2C_write_reg(0x01, 0x03);
	I2C_write_reg(0x01, 0x07);

	return 0;
}

unsigned char AW87319_Audio_OFF(void)
{
	aw87319_pa_pwroff();

	return 0;
}

static int aw87319_read_chip_id(void)
{
	int cnt = 0;
	int err = 0;

	aw87319_pa_pwron();

	while (cnt < CHIP_ID_READ_RETRIES) {
		chip_id = I2C_read_reg(0x00);
		if (chip_id == 0x9B)
			break;
		cnt++;
		msleep(CHIP_ID_RETRY_DELAY);
	}

	if (cnt == CHIP_ID_READ_RETRIES)
		err = -ENODEV;

	aw87319_pa_pwroff();

	return err;
}

static int aw87319_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;

	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!err)
		goto err_i2c_check;

	aw87319_rst = of_get_named_gpio(client->dev.of_node,
					"qcom,ext_pa_spk_aw87319_rst", 0);
	if (aw87319_rst < 0) {
		err = -ENODEV;
		goto err_gpio_get;
	}

	err = gpio_request_one(aw87319_rst, GPIOF_DIR_OUT , "spk_enable");
	if (err)
		goto err_gpio_req;

	aw87319_pa_client = client;

	err = aw87319_read_chip_id();
	if (err)
		goto err_chip;

	pr_info("version %d.%d\n", AW87319_VER_MAJOR, AW87319_VER_MINOR);
	pr_info("chip ID = 0x%x\n", chip_id);

	return 0;

err_chip:
	aw87319_pa_client = NULL;
err_gpio_req:
	gpio_free(aw87319_rst);
err_gpio_get:
err_i2c_check:
	return err;
}

static int aw87319_i2c_remove(struct i2c_client *client)
{
	aw87319_pa_client = NULL;

	return 0;
}

static const struct i2c_device_id aw87319_i2c_id[] = {
	{AW87319_I2C_NAME, 0},
	{}
};

static const struct of_device_id extpa_of_match[] = {
	{.compatible = "awinic,aw87319_pa"},
	{},
};

static struct i2c_driver aw87319_i2c_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name	= AW87319_I2C_NAME,
		.of_match_table = extpa_of_match,
	},
	.probe		= aw87319_i2c_probe,
	.remove		= aw87319_i2c_remove,
	.id_table	= aw87319_i2c_id,
};

static int __init aw87319_pa_init(void)
{
	int ret;

	ret = i2c_add_driver(&aw87319_i2c_driver);
	if (ret)
		pr_err("%s: Failed (ret=%d)\n", __func__, ret);

	return ret;
}

static void __exit aw87319_pa_exit(void)
{
	i2c_del_driver(&aw87319_i2c_driver);
}

module_init(aw87319_pa_init);
module_exit(aw87319_pa_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW87319 PA driver");
MODULE_LICENSE("GPL");
