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

static struct device *uart_dev;
static struct tty_serial console_serial;

void main(void)
{
	printf("Sample app running on: %s\n", CONFIG_BOARD);

	uart_dev = device_get_binding("UART_1");
	tty_init(&console_serial, uart_dev);
}
