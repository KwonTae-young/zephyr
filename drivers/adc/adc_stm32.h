/*
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ADC_STM32_H
#define __ADC_STM32_H

#include <soc.h>

extern void adc_stm32_init_soc(ADC_TypeDef *adc);
extern void adc_stm32_start_conversion_soc(ADC_TypeDef *adc);
extern void adc_stm32_enable_eoc_soc(ADC_TypeDef *adc);
extern void adc_stm32_setup_channels_soc(ADC_TypeDef *adc,
			u8_t channel_index, uint8_t rank_index);
extern void adc_stm32_setup_resolution_soc(ADC_TypeDef *adc,
					u8_t resolution);
extern void adc_stm32_setup_sequence_length_soc(ADC_TypeDef *adc,
						u8_t count);

#endif
