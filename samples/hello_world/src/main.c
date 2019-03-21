/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>

#if defined(CONFIG_BOARD_NRF52840_PCA10056)
/* [success] SRAM: 99.60% */
#define SIZE		64000
#elif defined(CONFIG_BOARD_STM32F4_DISCO)
/* [fail] SRAM: 83.59% */
#define SIZE		40000

/* [success] SRAM: 63.25% */
//#define SIZE		30000
#else
#error untested board.
#endif

static int test[SIZE];

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	printk("%p\n", test);
	printk("sizeof(test) = %d\n", sizeof(test));
}
