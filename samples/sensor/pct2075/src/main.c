/*
 * Copyright (c) 2020 Kwon Tae-young <tykwon@m2i.co.kr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <sys/printk.h>

void main(void)
{
	struct device *dev;
	struct sensor_value temp_value;
	int ret;

	dev = device_get_binding(DT_INST_0_NXP_PCT2075_LABEL);
	if (dev == NULL) {
		printk("Failed to get PCT2075 device binding\n");
		return;
	}

	printk("Device %s - %p is ready\n", dev->config->name, dev);
	/*
	while (1) {
		ret = sensor_sample_fetch(dev);
		if (ret) {
			printk("Failed to fetch measurements (%d)\n", ret);
			return;
		}

		ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
					 &temp_value);
		if (ret) {
			printk("Failed to get measurements (%d)\n", ret);
			return;
		}

		printk("temp is %d.%d oC\n", temp_value.val1, temp_value.val2);

		k_sleep(K_MSEC(1000));
	}
	*/
}
