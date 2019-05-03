/*
 * Copyright (c) 2019, Kwon Tae-young <tykwon@m2i.co.kr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *AT86RF233 Datasheet:
 *	http://ww1.microchip.com/downloads/en/devicedoc/
 *	atmel-8351-mcu_wireless-at86rf233_datasheet.pdf
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <errno.h>
#include <gpio.h>
#include <spi.h>

/* 6.3 SPI Protocol */
#define AT86RF2XX_ACCESS_READ	0x00
#define AT86RF2XX_ACCESS_WRITE	0x40
#define AT86RF2XX_ACCESS_REG	0x80

/* 6.5 Radio Transceiver Identification */
#define AT86RF2XX_REG_PART_NUM		0x1C
#define AT86RF2XX_REG_VERSION_NUM	0x1D
#define AT86RF2XX_REG_MAIN_ID_0		0x1E
#define AT86RF2XX_REG_MAIN_ID_1		0x1F

#define AT86RF2XX_PART_NUMBER		0x0B
#define AT86RF2XX_REVISION_A		0x01
#define AT86RF2XX_REVISION_B		0x02
#define AT86RF2XX_MAIN_ID		0x001F


#define AT86RF2XX_RESET_PULSE_WIDTH	1
#define AT86RF2XX_RESET_DELAY		1

static int at86rf2xx_access(struct device *spi, struct spi_config *spi_cfg,
			    u8_t cmd, u16_t addr, void *data, size_t len)
{
	u8_t access[1];
	struct spi_buf bufs[] = {
		{
			.buf = access,
		},
		{
			.buf = data,
			.len = len
		}
	};
	struct spi_buf_set tx = {
		.buffers = bufs
	};
	
	access[0] = AT86RF2XX_ACCESS_REG | cmd | addr;

	bufs[0].len = 1;
	tx.count = 2;

	if (cmd == AT86RF2XX_ACCESS_READ) {
		struct spi_buf_set rx = {
			.buffers = bufs,
			.count = 2
		};

		return spi_transceive(spi, spi_cfg, &tx, &rx);
	}

	return spi_write(spi, spi_cfg, &tx);
}

static int read_reg(struct device *spi, struct spi_config *spi_cfg,
		      u16_t addr, u8_t *data, u32_t num_bytes)
{
	int err;

	err = at86rf2xx_access(spi, spi_cfg,
			       AT86RF2XX_ACCESS_READ, addr, data, num_bytes);
	if (err) {
		printk("Error during SPI read\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_part_number(struct device *spi, struct spi_config *spi_cfg)
{
	u8_t id[1];
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG_PART_NUM, id, 1);
	if (err) {
		printk("Error during part number read\n");
		return -EIO;
	}

	printk("PART_NUM:0x%x\n", id[0]);

	if (id[0] != AT86RF2XX_PART_NUMBER) {
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_version_number(struct device *spi, struct spi_config *spi_cfg)
{
	u8_t version[1];
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG_VERSION_NUM, version, 1);
	if (err) {
		printk("Error during version number read\n");
		return -EIO;
	}

	printk("VERSION_NUM:0x%x\n", version[0]);

	if ((version[0] != AT86RF2XX_REVISION_A) &&
		(version[0] != AT86RF2XX_REVISION_B)){
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_main_id(struct device *spi, struct spi_config *spi_cfg)
{
	u8_t id[2];
	u16_t main_id;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG_MAIN_ID_0, id, 1);
	if (err) {
		printk("Error during part number read\n");
		return -EIO;
	}

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG_MAIN_ID_1, id + 1, 1);
	if (err) {
		printk("Error during part number read\n");
		return -EIO;
	}

	main_id = (id[1] << 8) | id[0];
	printk("MAIN_ID:0x%x\n", main_id);

	if (main_id != AT86RF2XX_MAIN_ID) {
		return -EIO;
	}

	return 0;
}

void main(void)
{
	struct device *spi;
	struct device *reset;
	struct spi_cs_control spi_cs;
	struct spi_config spi_cfg;
	int err;

	printk("AT86RF2XX(2.4GHz Transceiver) example application\n");
	printk("\n");

	spi = device_get_binding(DT_ATMEL_AT86RF2XX_0_BUS_NAME);
	if (!spi) {
		printk("Could not find AT86RF2XX driver\n");
		return;
	}

	spi_cfg.operation = SPI_WORD_SET(8);
	spi_cfg.frequency = DT_ATMEL_AT86RF2XX_0_SPI_MAX_FREQUENCY;

	spi_cs.gpio_dev = device_get_binding(DT_ATMEL_AT86RF2XX_0_CS_GPIO_CONTROLLER);
	spi_cs.gpio_pin = DT_ATMEL_AT86RF2XX_0_CS_GPIO_PIN;
	spi_cs.delay = 0;
	spi_cfg.cs = &spi_cs;

#ifdef DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_CONTROLLER
	reset = device_get_binding(DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_CONTROLLER);
	if (reset == NULL) {
		printk("Failed to get pointer to %s device!",
			    DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_CONTROLLER);
		return;
	}

 	gpio_pin_configure(reset, DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_PIN, GPIO_DIR_OUT);
	gpio_pin_write(reset, DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_PIN, 0);
	k_sleep(AT86RF2XX_RESET_PULSE_WIDTH);
	gpio_pin_write(reset, DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_PIN, 1);
	k_sleep(AT86RF2XX_RESET_DELAY);
#endif

	err = at86rf2xx_part_number(spi, &spi_cfg);
	if (err) {
		printk("Could not verify AT86RF2XX part number\n");
		return;
	}

	err = at86rf2xx_version_number(spi, &spi_cfg);
	if (err) {
		printk("Could not verify AT86RF2XX version number\n");
		return;
	}

	err = at86rf2xx_main_id(spi, &spi_cfg);
	if (err) {
		printk("Could not verify AT86RF2XX main id\n");
		return;
	}
}
