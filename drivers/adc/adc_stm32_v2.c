/*
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * For F1/F2/F4/F7 series stm32 support.
 */

#include "adc_stm32.h"

void adc_stm32_init_soc(ADC_TypeDef *adc)
{
	adc->CR2 |= ADC_CR2_ADON;
}

void adc_stm32_start_conversion_soc(ADC_TypeDef *adc)
{
	adc->CR2 |= ADC_CR2_SWSTART;
}

void adc_stm32_enable_eoc_soc(ADC_TypeDef *adc)
{
	adc->CR1 |= ADC_CR1_EOCIE;
	adc->CR2 |= ADC_CR2_EOCS;
	adc->CR1 |= ADC_CR1_SCAN;
}

void adc_stm32_setup_channels_soc(ADC_TypeDef *adc,
			u8_t channel_index, u8_t rank_index)
{
	if (rank_index > 11) {
		adc->SQR1 |= channel_index << ((rank_index - 12) * 5);
	} else if (rank_index > 5) {
		adc->SQR2 |= channel_index << ((rank_index - 6) * 5);
	} else {
		adc->SQR3 |= channel_index << (rank_index * 5);
	}
}

void adc_stm32_setup_sequence_length_soc(ADC_TypeDef *adc, u8_t count)
{
	adc->SQR1 &= ~ADC_SQR1_L_Msk;
	adc->SQR1 |= (count - 1) << ADC_SQR1_L_Pos;
}

void adc_stm32_setup_resolution_soc(ADC_TypeDef *adc, u8_t resolution)
{
	adc->CR1 &= ~ADC_CR1_RES_Msk;
	adc->CR1 |= (3 - (resolution - 6) / 2) << ADC_CR1_RES_Pos;
}
