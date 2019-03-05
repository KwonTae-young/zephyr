/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <uart.h>
#include <device.h>
#include <console.h>
#include <tty.h>
#include <string.h>

#define BARCODE_UART_PORT	"UART_1"

static struct device *uart_dev;
static struct tty_serial console_serial;

static u8_t console_rxbuf[CONFIG_CONSOLE_GETCHAR_BUFSIZE];
static u8_t barcode_buf[64];
static u8_t length = 0;

int barcode_init()
{
	struct uart_config uart_cfg;
	int ret;

	uart_dev = device_get_binding(BARCODE_UART_PORT);
	if (uart_dev == NULL) {
		printf("Failed to get %s\n", BARCODE_UART_PORT);
		return -1;
	}

	ret = uart_config_get(uart_dev, &uart_cfg);
	if (!ret) {
		printf("\n======== [%s] ========\n", BARCODE_UART_PORT);
		printf("[%s] uart_config.baudrate=%d\n", BARCODE_UART_PORT, uart_cfg.baudrate);
		printf("[%s] uart_config.parity=%d\n", BARCODE_UART_PORT, uart_cfg.parity);
		printf("[%s] uart_config.stop_bits=%d\n", BARCODE_UART_PORT, uart_cfg.stop_bits);
		printf("[%s] uart_config.data_bits=%d\n", BARCODE_UART_PORT, uart_cfg.data_bits);
		printf("[%s] uart_config.flow_ctrl=%d\n", BARCODE_UART_PORT, uart_cfg.flow_ctrl);
	} else {
		printf("uart_config_get() error\n");
		return -1;
	}

	tty_init(&console_serial, uart_dev);
	tty_set_rx_buf(&console_serial, console_rxbuf, sizeof(console_rxbuf));

	return 0;
}


int get_barcode()
{
	int size = 0;

	size = tty_read(&console_serial, console_rxbuf, 1);
	if (size > 0) {
		barcode_buf[length++] = console_rxbuf[0];
		if (console_rxbuf[0] == 0x0a) {
			return 0;
		}
	}

	return -1;
}

void main(void)
{
	int i, ret;

	printf("tty API sample app running on(for barcode): %s\n", CONFIG_BOARD);

	ret = barcode_init();
	if (ret) {
		printf("barcode_init() error\n");
		return;
	}

	while (1) {
		if (!get_barcode()) {
			printf("barcode data: ");
			for (i = 0; i < length; i++) {
				printf("%02x ", barcode_buf[i]);
			}
			printf("\n");

			length = 0;
			memset(console_rxbuf, 0x00, sizeof(console_rxbuf));
			memset(barcode_buf, 0x00, sizeof(barcode_buf));
		}
	}

	return;
}
