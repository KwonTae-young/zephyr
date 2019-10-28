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
LOG_MODULE_REGISTER(adc121s051);

struct adc121s051_data {
	struct device *spi;
	struct spi_config spi_cfg;
	struct device *reset;
	struct device *nfault;
} adc121s051_dev;

static int read_value(struct device *spi, struct spi_config *spi_cfg,
			void *data, size_t size)
{
	struct spi_buf bufs[] = {
		{
			.buf = data,
			.len = size
		},
	};
	struct spi_buf_set rx = {
		.buffers = bufs,
		.count = 1
	};
	int ret;
	
	ret = spi_read(spi, spi_cfg, &rx);
	if (ret == 0) {
		((uint16_t*) data)[0] = (((uint16_t*) data)[0] << 8) + (((uint16_t*) data)[0] >> 8);
	}

	return ret;
}


static int adc121s051_iio_read(struct device *dev, u8_t *data, u8_t size)
{
	int ret;

	ret = read_value(adc121s051_dev.spi, &adc121s051_dev.spi_cfg,
									data, size);
	if (ret) {
		LOG_ERR("Error during value read");
		return ret;
	}

	return 0;
}


static int adc121s051_iio_init(struct device *dev)
{
	static struct spi_cs_control spi_cs;

	adc121s051_dev.spi = device_get_binding(DT_INST_0_TI_ADC121S051_BUS_NAME);
	if (!adc121s051_dev.spi) {
		LOG_ERR("Cannot get pointer to %s device",
			    DT_INST_0_TI_ADC121S051_BUS_NAME);
		return -EINVAL;
	}

	/* SPI Setting */
	adc121s051_dev.spi_cfg.operation = SPI_WORD_SET(8);
	adc121s051_dev.spi_cfg.frequency = DT_INST_0_TI_ADC121S051_SPI_MAX_FREQUENCY;
	adc121s051_dev.spi_cfg.slave = DT_INST_0_TI_ADC121S051_BASE_ADDRESS;

	/* SPI Chip Select */
	spi_cs.gpio_dev = device_get_binding(
				DT_INST_0_TI_ADC121S051_CS_GPIOS_CONTROLLER);
	spi_cs.gpio_pin = DT_INST_0_TI_ADC121S051_CS_GPIOS_PIN;
	spi_cs.delay = 0;
	if (spi_cs.gpio_dev) {
		adc121s051_dev.spi_cfg.cs = &spi_cs;
	} else {
		LOG_ERR("Cannot get pointer to %s device",
		       DT_INST_0_TI_ADC121S051_CS_GPIOS_CONTROLLER);
		return -EIO;
	}

	return 0;
}

static const struct iio_driver_api adc121s051_iio_api = {
	.read = adc121s051_iio_read,
};

DEVICE_AND_API_INIT(adc121s051_iio, DT_INST_0_TI_ADC121S051_LABEL,
		    &adc121s051_iio_init, NULL,
		    NULL, POST_KERNEL, CONFIG_IIO_INIT_PRIORITY,
		    &adc121s051_iio_api);
