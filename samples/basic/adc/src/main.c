/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <adc.h>

uint16_t temp_data[8];

static const struct adc_channel_cfg channel_cfg_0 = {
        .channel_id       = 0,
};

const struct adc_sequence sequence = {
        .channels    = BIT(0),
        .buffer      = temp_data,
        .buffer_size = sizeof(temp_data),
        .resolution  = 12,
};

void main(void)
{
        int ret;
        struct device *adc_dev = device_get_binding(DT_ADC_0_NAME);
        adc_channel_setup(adc_dev, &channel_cfg_0);

        while (1) {
                ret = adc_read(adc_dev, &sequence);
                if (ret != 0) {
                        printk("adc_read() failed with code %d", ret);
                } else {
                        printk("adc_read() value %d\n", temp_data[0]);
                }
                k_sleep(100);
        }
}
