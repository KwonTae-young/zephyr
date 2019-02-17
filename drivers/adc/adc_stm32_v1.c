/*
 * Copyright (c) 2019 Endre Karlson
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * For F0/L0 series stm32 support.
 */

#include "adc_stm32.h"

void adc_stm32_init_soc(ADC_TypeDef *adc)
{
	adc->CFGR1 = 0;
	adc->CFGR2 = 0;

	ADC->CCR = ADC_CCR_VREFEN | ADC_CCR_TSEN | ADC_CCR_PRESC_1;

	adc->SMPR |= ADC_SMPR_SMP;

	adc->CR |= ADC_CR_ADEN;
}

void adc_stm32_start_conversion_soc(ADC_TypeDef *adc)
{
	adc->CR |= ADC_CR_ADSTART;
}

void adc_stm32_enable_eoc_soc(ADC_TypeDef *adc)
{
	adc->IER |= ADC_IER_EOCIE;
}

void adc_stm32_setup_channels_soc(ADC_TypeDef *adc,
			u8_t channel_index, u8_t rank_index)
{
	ARG_UNUSED(rank_index);

	adc->CHSELR = 1 << channel_index;
}

void adc_stm32_setup_sequence_length_soc(ADC_TypeDef *adc, u8_t count)
{
}

void adc_stm32_setup_resolution_soc(ADC_TypeDef *adc, u8_t resolution)
{
	adc->CFGR1 &= ~ADC_CFGR1_RES;
	adc->CFGR1 |= (3 - (resolution - 6) / 2) << ADC_CFGR1_RES_Pos;
}
