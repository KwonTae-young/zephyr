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
LOG_MODULE_REGISTER(drv8804);

#define GPIO_RESET_PIN		DT_INST_0_TI_DRV8804_RESET_GPIOS_PIN
#define GPIO_NFAULT_PIN	DT_INST_0_TI_DRV8804_NFAULT_GPIOS_PIN

struct drv8804_data {
	struct device *spi;
	struct spi_config spi_cfg;
	struct device *reset;
	struct device *nfault;
} drv8804_dev;

static int write_value(struct device *spi, struct spi_config *spi_cfg,
			void *data, size_t size)
{
	struct spi_buf bufs[] = {
		{
			.buf = data,
			.len = size
		},
	};
	struct spi_buf_set tx = {
		.buffers = bufs,
		.count = 1
	};
	
	return spi_write(spi, spi_cfg, &tx);
}

static int drv8804_iio_write(struct device *dev, u8_t *data, u8_t size)
{
	int ret;

	ret = write_value(drv8804_dev.spi, &drv8804_dev.spi_cfg, data, size);
	if (ret) {
		LOG_ERR("Error during value write");
		return ret;
	}

	return 0;
}

static int drv8804_iio_status(struct device *dev, u8_t *data, u8_t size)
{
	int ret, value;

	ret = gpio_pin_read(drv8804_dev.nfault, GPIO_NFAULT_PIN, &value);
	value = !value;

	memcpy(data, &value, size);

	return value;
}

static int drv8804_iio_init(struct device *dev)
{
	static struct spi_cs_control spi_cs;

	drv8804_dev.spi = device_get_binding(DT_INST_0_TI_DRV8804_BUS_NAME);
	if (!drv8804_dev.spi) {
		LOG_ERR("Cannot get pointer to %s device",
			    DT_INST_0_TI_DRV8804_BUS_NAME);
		return -EINVAL;
	}

	/* SPI Setting */
	drv8804_dev.spi_cfg.operation = SPI_WORD_SET(8);
	drv8804_dev.spi_cfg.frequency = DT_INST_0_TI_DRV8804_SPI_MAX_FREQUENCY;
	drv8804_dev.spi_cfg.slave = DT_INST_0_TI_DRV8804_BASE_ADDRESS;

	/* SPI Chip Select */
	spi_cs.gpio_dev = device_get_binding(
				DT_INST_0_TI_DRV8804_CS_GPIOS_CONTROLLER);
	spi_cs.gpio_pin = DT_INST_0_TI_DRV8804_CS_GPIOS_PIN;
	spi_cs.delay = 0;
	if (spi_cs.gpio_dev) {
		drv8804_dev.spi_cfg.cs = &spi_cs;
	} else {
		LOG_ERR("Cannot get pointer to %s device",
		       DT_INST_0_TI_DRV8804_CS_GPIOS_CONTROLLER);
		return -EIO;
	}

	/* Reset */
	drv8804_dev.reset = device_get_binding(
				DT_INST_0_TI_DRV8804_RESET_GPIOS_CONTROLLER);
	if (drv8804_dev.reset != NULL) {
		gpio_pin_configure(drv8804_dev.reset, GPIO_RESET_PIN,
								GPIO_DIR_OUT);
		gpio_pin_write(drv8804_dev.reset, GPIO_RESET_PIN, 0);
		gpio_pin_write(drv8804_dev.reset, GPIO_RESET_PIN, 1);
		k_usleep(20);
		gpio_pin_write(drv8804_dev.reset, GPIO_RESET_PIN, 0);

	} else {
		LOG_ERR("Failed to get pointer to %s device!",
				DT_INST_0_TI_DRV8804_RESET_GPIOS_CONTROLLER);
		return -EIO;
	}

	/* nFAULT Input */
	drv8804_dev.nfault = device_get_binding(
				DT_INST_0_TI_DRV8804_NFAULT_GPIOS_CONTROLLER);
	if (drv8804_dev.nfault != NULL) {
		gpio_pin_configure(drv8804_dev.nfault, GPIO_NFAULT_PIN, GPIO_DIR_IN);
	} else {
		printk("Failed to get pointer to %s device!",
				DT_INST_0_TI_DRV8804_NFAULT_GPIOS_CONTROLLER);
		return -EIO;
	}

	return 0;
}

static const struct iio_driver_api drv8804_iio_api = {
	.write = drv8804_iio_write,
	.status = drv8804_iio_status,
};

DEVICE_AND_API_INIT(drv8804_iio, DT_INST_0_TI_DRV8804_LABEL,
		    &drv8804_iio_init, NULL,
		    NULL, POST_KERNEL, CONFIG_IIO_INIT_PRIORITY,
		    &drv8804_iio_api);
