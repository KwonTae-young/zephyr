/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <gpio.h>
#include <uart.h>
#include <device.h>
#include <console/tty.h>
#include <string.h>

#define TTY_UART_PORT	"UART_1"
#define TEST_GPIO_PORT	"GPIOB"
#define TEST_GPIO_PIN	5

void main(void)
{
	struct device *tty_dev, *gpio;
	struct uart_config uart_cfg;
	struct tty_serial tty;
	int ret;
	u8_t tty_txbuf[32];

	tty_dev = device_get_binding(TTY_UART_PORT);
	if (tty_dev == NULL) {
		printf("Failed to get %s\n", TTY_UART_PORT);
		return;
	}

	gpio = device_get_binding(TEST_GPIO_PORT);
	if (tty_dev == NULL) {
		printf("Failed to get %s\n", TTY_UART_PORT);
		return;
	}
	gpio_pin_configure(gpio, TEST_GPIO_PIN, GPIO_DIR_OUT);


	ret = uart_config_get(tty_dev, &uart_cfg);
	if (!ret) {
		printf("\n======== [%s] ========\n", TTY_UART_PORT);
		printf("[%s] uart_cfg.baudrate=%d\n", TTY_UART_PORT, uart_cfg.baudrate);
		printf("[%s] uart_cfg.parity=%d\n", TTY_UART_PORT, uart_cfg.parity);
		printf("[%s] uart_cfg.stop_bits=%d\n", TTY_UART_PORT, uart_cfg.stop_bits);
		printf("[%s] uart_cfg.data_bits=%d\n", TTY_UART_PORT, uart_cfg.data_bits);
		printf("[%s] uart_cfg.flow_ctrl=%d\n", TTY_UART_PORT, uart_cfg.flow_ctrl);
	} else {
		printf("uart_config_get() error\n");
		return;
	}

	tty_init(&tty, tty_dev);
	tty_set_tx_buf(&tty, tty_txbuf, sizeof(tty_txbuf));
	memset(tty_txbuf, 0x12, sizeof(tty_txbuf));

	while (1) {
		tty_write(&tty, tty_txbuf, sizeof(tty_txbuf));
		gpio_pin_write(gpio, TEST_GPIO_PIN, true);
		k_sleep(1);
		gpio_pin_write(gpio, TEST_GPIO_PIN, false);

		k_sleep(1000);
	}
}
