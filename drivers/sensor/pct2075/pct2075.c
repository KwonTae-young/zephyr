/*
 * Copyright (c) 2020 Kwon Tae-young <tykwon@m2i.co.kr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "pct2075.h"

#define SIGN_BIT_MASK 0x7F

LOG_MODULE_REGISTER(pct2075, CONFIG_SENSOR_LOG_LEVEL);

struct pct2075_data {
	struct device *i2c_dev;
	u16_t temperature;
};

static int pct2075_reg_read(struct pct2075_data *drv_data, u8_t reg, u8_t *val)
{
	struct i2c_msg msgs[2] = {
		{
			.buf = &reg,
			.len = 1,
			.flags = I2C_MSG_WRITE,
		},
		{
			.buf = val,
			.flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP,
		},
	};

	switch (reg) {
	case PCT2075_REG_CONF:
	case PCT2075_REG_TIDLE:
		msgs[1].len = 1;
		break;
	case PCT2075_REG_TEMP:
	case PCT2075_REG_THYST:
	case PCT2075_REG_TOS:
		msgs[1].len = 2;
		break;
	default:
		LOG_ERR("not defined register read request: reg=0x%x", reg);
		return -ENOTSUP;
	}

	if (i2c_transfer(drv_data->i2c_dev, msgs, 2,
			DT_INST_0_NXP_PCT2075_BASE_ADDRESS) != 0) {
		return -EIO;
	}

	return 0;
}

static int pct2075_reg_write(struct pct2075_data *drv_data, u8_t reg,
			      u8_t val)
{
	return -ENOTSUP;
}

static int pct2075_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	return -ENOTSUP;
}

static int pct2075_channel_get(struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	return -ENOTSUP;
}

static const struct sensor_driver_api pct2075_api = {
	.sample_fetch = &pct2075_sample_fetch,
	.channel_get = &pct2075_channel_get,
};

static int pct2075_init(struct device *dev)
{
	struct pct2075_data *drv_data = dev->driver_data;
	u8_t val;
	u8_t val1[2];

	drv_data->i2c_dev = device_get_binding(DT_INST_0_NXP_PCT2075_BUS_NAME);

	if (!drv_data->i2c_dev) {
		LOG_ERR("Failed to get pointer to %s device!",
			DT_INST_0_NXP_PCT2075_BUS_NAME);
		return -EINVAL;
	}



	if (pct2075_reg_read(drv_data, PCT2075_REG_CONF, &val)) {
		return -EIO;
	} else {
		LOG_INF("PCT2075_REG_CONF=0x%x", val);
	}

	if (pct2075_reg_read(drv_data, PCT2075_REG_TEMP, val1)) {
		return -EIO;
	} else {
		LOG_INF("PCT2075_REG_TEMP=0x%x%x", val1[0], val1[1]);
	}

	if (pct2075_reg_read(drv_data, PCT2075_REG_TOS, val1)) {
		return -EIO;
	} else {
		LOG_INF("PCT2075_REG_TOS=0x%x%x", val1[0], val1[1]);
	}

	if (pct2075_reg_read(drv_data, PCT2075_REG_THYST, val1)) {
		return -EIO;
	} else {
		LOG_INF("PCT2075_REG_THYST=0x%x%x", val1[0], val1[1]);
	}

	if (pct2075_reg_read(drv_data, PCT2075_REG_TIDLE, &val)) {
		return -EIO;
	} else {
		LOG_INF("PCT2075_REG_TIDLE=0x%x", val);
	}

	LOG_INF("%s() Success", __func__)

	return 0;
}

static struct pct2075_data pct2075_data;

DEVICE_AND_API_INIT(pct2075, DT_INST_0_NXP_PCT2075_LABEL,
		    pct2075_init, &pct2075_data, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &pct2075_api);
