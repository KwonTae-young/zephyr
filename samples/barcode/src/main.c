/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <zephyr/types.h>
#include <string.h>
#include <uart.h>
#include <misc/byteorder.h>

#define BUF_MAXSIZE	256
#define SLEEP_TIME	500

static struct device *uart_dev;
static u8_t rx_buf[BUF_MAXSIZE];

static void msg_dump(const char *s, u8_t *data, unsigned len)
{
	unsigned i;

	printf("%s: ", s);
	for (i = 0U; i < len; i++) {
		printf("%02x ", data[i]);
	}
	printf("(%u bytes)\n", len);
}

static void uart_isr(struct device *x)
{
	int len = uart_fifo_read(uart_dev, rx_buf, BUF_MAXSIZE);

	ARG_UNUSED(x);
	msg_dump(__func__, rx_buf, len);
}

static void uart_init(void)
{
	uart_dev = device_get_binding("UART_1");

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);

	printf("%s() done\n", __func__);
}

void main(void)
{
	struct uart_config uart_cfg;
	int ret;

	printf("Sample app running on: %s\n", CONFIG_BOARD);

	uart_init();

	ret = uart_config_get(uart_dev, &uart_cfg);
	if (ret == 0) {
		printf("uart_config.baudrate=%d\n", uart_cfg.baudrate);
		printf("uart_config.parity=%d\n", uart_cfg.parity);
		printf("uart_config.stop_bits=%d\n", uart_cfg.stop_bits);
		printf("uart_config.data_bits=%d\n", uart_cfg.data_bits);
		printf("uart_config.flow_ctrl=%d\n", uart_cfg.flow_ctrl);
	} else {
		printf("uart_config_get() error\n");
	}

	while (1) {
		k_sleep(SLEEP_TIME);
	}
}
