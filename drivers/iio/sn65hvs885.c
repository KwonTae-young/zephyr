/*
 * Copyright (c) 2019 Kwon Tae-young <tykwon@m2i.co.kr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <spi.h>
#include <drivers/gpio.h>
#include <drivers/iio.h>

#define LOG_LEVEL CONFIG_LORA_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(sn65hvs885);

#define GPIO_LOAD_PIN		DT_INST_0_TI_SN65HVS885_LOAD_GPIOS_PIN
#define GPIO_MISO_SWITCH_PIN	DT_INST_0_TI_SN65HVS885_MISO_SWITCH_GPIOS_PIN
#define GPIO_HOT_PIN		DT_INST_0_TI_SN65HVS885_HOT_GPIOS_PIN

#define MISO_SWITCH_DELAY	10

struct sn65hvs885_data {
	struct device *spi;
	struct spi_config spi_cfg;
	struct device *load_pulse;
	struct device *miso_switch;
	struct device *hot;
} sn65hvs885_dev_data;

static int read_value(struct device *spi, struct spi_config *spi_cfg,
			struct device *load_pulse, struct device *miso_switch,
			void *data, u8_t size)
{
	struct spi_buf bufs[] = {
		{
			.buf = data,
			.len = 1
		},
	};
	struct spi_buf_set rx = {
		.buffers = bufs,
		.count = 1
	};
	int ret;

	gpio_pin_write(miso_switch, GPIO_MISO_SWITCH_PIN, 1);
	k_usleep(MISO_SWITCH_DELAY);

	gpio_pin_write(load_pulse, GPIO_LOAD_PIN, 0);
	gpio_pin_write(load_pulse, GPIO_LOAD_PIN, 1);

	ret = spi_read(spi, spi_cfg, &rx);
	if (ret == 0) {
		((uint8_t*) data)[0] = ((uint8_t*) data)[0] >> 1;
	}

	gpio_pin_write(miso_switch, GPIO_MISO_SWITCH_PIN, 0);

	return ret;
}


static int sn65hvs885_iio_read(struct device *dev, u8_t *data, u8_t size)
{
	int ret;

	ret = read_value(sn65hvs885_dev_data.spi, &sn65hvs885_dev_data.spi_cfg, sn65hvs885_dev_data.load_pulse,
			sn65hvs885_dev_data.miso_switch, data, size);
	if (ret) {
		LOG_ERR("Error during value read");
		return ret;
	}

	return 0;
}

static int sn65hvs885_iio_status(struct device *dev, u8_t *data, u8_t size)
{
	int ret, value;

	ret = gpio_pin_read(sn65hvs885_dev_data.miso_switch, GPIO_HOT_PIN, &value);
	value = !value;

	memcpy(data, &value, size);

	return ret;
}

static int sn65hvs885_iio_init(struct device *dev)
{
	static struct spi_cs_control spi_cs;

	sn65hvs885_dev_data.spi = device_get_binding(DT_INST_0_TI_SN65HVS885_BUS_NAME);
	if (!sn65hvs885_dev_data.spi) {
		LOG_ERR("Cannot get pointer to %s device",
			    DT_INST_0_TI_SN65HVS885_BUS_NAME);
		return -EINVAL;
	}

	/* SPI Setting */
	sn65hvs885_dev_data.spi_cfg.operation = SPI_WORD_SET(8) | SPI_MODE_CPHA;
	sn65hvs885_dev_data.spi_cfg.frequency = DT_INST_0_TI_SN65HVS885_SPI_MAX_FREQUENCY;
	sn65hvs885_dev_data.spi_cfg.slave = DT_INST_0_TI_SN65HVS885_BASE_ADDRESS;

	/* SPI Chip Select */
	spi_cs.gpio_dev = device_get_binding(
				DT_INST_0_TI_SN65HVS885_CS_GPIOS_CONTROLLER);
	spi_cs.gpio_pin = DT_INST_0_TI_SN65HVS885_CS_GPIOS_PIN;
	spi_cs.delay = 0;
	if (spi_cs.gpio_dev) {
		sn65hvs885_dev_data.spi_cfg.cs = &spi_cs;
	} else {
		LOG_ERR("Cannot get pointer to %s device",
		       DT_INST_0_SEMTECH_SX1276_CS_GPIOS_CONTROLLER);
		return -EIO;
	}

	/* Load Pulse Input */
	sn65hvs885_dev_data.load_pulse = device_get_binding(
				DT_INST_0_TI_SN65HVS885_LOAD_GPIOS_CONTROLLER);
	if (sn65hvs885_dev_data.load_pulse != NULL) {
		gpio_pin_configure(sn65hvs885_dev_data.load_pulse, GPIO_LOAD_PIN,
								GPIO_DIR_OUT);
		gpio_pin_write(sn65hvs885_dev_data.load_pulse, GPIO_LOAD_PIN, 1);
	} else {
		LOG_ERR("Failed to get pointer to %s device!",
				DT_INST_0_TI_SN65HVS885_LOAD_GPIOS_CONTROLLER);
		return -EIO;
	}

	/* Hot */
	sn65hvs885_dev_data.hot = device_get_binding(
				DT_INST_0_TI_SN65HVS885_HOT_GPIOS_CONTROLLER);
	if (sn65hvs885_dev_data.hot != NULL) {
		gpio_pin_configure(sn65hvs885_dev_data.load_pulse, GPIO_HOT_PIN,
								GPIO_DIR_IN);
	} else {
		LOG_ERR("Failed to get pointer to %s device!",
				DT_INST_0_TI_SN65HVS885_HOT_GPIOS_CONTROLLER);
		return -EIO;
	}

	/* SPI MISO Switch GPIO */
	sn65hvs885_dev_data.miso_switch = device_get_binding(
			DT_INST_0_TI_SN65HVS885_MISO_SWITCH_GPIOS_CONTROLLER);
	if (sn65hvs885_dev_data.miso_switch != NULL) {
		gpio_pin_configure(sn65hvs885_dev_data.load_pulse, GPIO_MISO_SWITCH_PIN,
					GPIO_DIR_OUT);
		gpio_pin_write(sn65hvs885_dev_data.load_pulse, GPIO_MISO_SWITCH_PIN, 0);
	} else {
		LOG_ERR("Failed to get pointer to %s device!",
			DT_INST_0_TI_SN65HVS885_MISO_SWITCH_GPIOS_CONTROLLER);
		return -EIO;
	}

	return 0;
}

static const struct iio_driver_api sn65hvs885_iio_api = {
	.read = sn65hvs885_iio_read,
	.status = sn65hvs885_iio_status,
};

DEVICE_AND_API_INIT(sn65hvs885_iio, DT_INST_0_TI_SN65HVS885_LABEL,
		    &sn65hvs885_iio_init, NULL,
		    NULL, POST_KERNEL, CONFIG_IIO_INIT_PRIORITY,
		    &sn65hvs885_iio_api);
