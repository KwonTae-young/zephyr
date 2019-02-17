/*
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * For F3/L4 series stm32 support.
 */

#include "adc_stm32.h"

void adc_stm32_init_soc(ADC_TypeDef *adc)
{
	adc->CR |= ADC_CR_ADEN;
}

void adc_stm32_start_conversion_soc(ADC_TypeDef *adc)
{
	adc->CR2 |= ADC_CR2_SWSTART;
}

void adc_stm32_enable_eoc_soc(ADC_TypeDef *adc)
{
	adc->CR1 |= ADC_CR1_EOCIE;
}

void adc_stm32_setup_channels_soc(ADC_TypeDef *adc,
			u8_t channel_index, u8_t rank_index)
{
	if (rank_index < 4) {
		adc->SQR1 |= channel_index << ((rank_index + 1) * 6);
	} else if (rank_index < 9) {
		adc->SQR2 |= channel_index << ((rank_index - 4) * 6);
	} else if (rank_index < 14) {
		adc->SQR3 |= channel_index << ((rank_index - 9) * 6);
	} else {
		adc->SQR4 |= channel_index << ((rank_index - 14) * 6);
	}
}

void adc_stm32_setup_sequence_length_soc(ADC_TypeDef *adc, u8_t count)
{
	adc->SQR1 &= ~ADC_SQR1_L_Msk;
	adc->SQR1 |= count - 1;
}

void adc_stm32_setup_resolution_soc(ADC_TypeDef *adc, u8_t resolution);
{
	adc->CFGR &= ~ADC_CFGR_RES_Msk;
	adc->CFGR |= (3 - (resolution - 6) / 2) << ADC_CFGR_RES_Pos;
}
