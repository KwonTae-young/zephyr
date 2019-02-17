/*
 * Copyright (c) 2018 Kokoon Technology Limited
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 * Copyright (c) 2019 Endre Karlson
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <adc.h>
#include <device.h>
#include <kernel.h>
#include <init.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_stm32);

#include <clock_control/stm32_clock_control.h>

#include "adc_stm32.h"

struct adc_stm32_data {
	struct adc_context ctx;
	struct device *dev;
	u16_t *buffer;
	u16_t *repeat_buffer;

	u8_t seq_size;
	u8_t resolution;

	u8_t channel_count;
};

struct adc_stm32_cfg {
	ADC_TypeDef *base;

	void (*irq_cfg_func)(void);

	struct stm32_pclken pclken;
	struct device *p_dev;
};

static int check_buffer_size(const struct adc_sequence *sequence,
			     u8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(u16_t);

	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}
	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Provided buffer is too small (%u/%u)",
				sequence->buffer_size, needed_buffer_size);
		return -ENOMEM;
	}
	return 0;
}

static void adc_stm32_start_conversion(struct device *dev)
{
	const struct adc_stm32_cfg *config = dev->config->config_info;

	LOG_INF("Starting conversion");

	adc_stm32_start_conversion_soc(config->base);

	LOG_INF("Conversion done!");
}

static int start_read(struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_stm32_data *data = dev->driver_data;
	const struct adc_stm32_cfg *cfg = dev->config->config_info;

	int err;

	switch (sequence->resolution) {
	case 6:
		break;
	case 8:
		break;
	case 10:
		break;
	case 12:
		break;
	default:
		LOG_ERR("Invalid resolution");
		return -EINVAL;
	}

	u32_t channels = sequence->channels;

	data->buffer = sequence->buffer;
	u8_t index;

	index = find_lsb_set(channels) - 1;
	adc_stm32_setup_channels_soc(cfg->base, index, data->channel_count);
	adc_stm32_setup_sequence_length_soc(cfg->base, 1);
	data->channel_count = 1;

	err = check_buffer_size(sequence, data->channel_count);
	if (err) {
		return err;
	}

	adc_stm32_setup_resolution_soc(cfg->base, sequence->resolution);
	adc_stm32_enable_eoc_soc(cfg->base);

	adc_context_start_read(&data->ctx, sequence);
	err = adc_context_wait_for_completion(&data->ctx);
	adc_context_release(&data->ctx, err);

	return err;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_stm32_data *data =
		CONTAINER_OF(ctx, struct adc_stm32_data, ctx);

	data->repeat_buffer = data->buffer;

	adc_stm32_start_conversion(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct adc_stm32_data *data =
		CONTAINER_OF(ctx, struct adc_stm32_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_stm32_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct adc_stm32_data *data = (struct adc_stm32_data *)dev->driver_data;
	struct adc_stm32_cfg *config =
		(struct adc_stm32_cfg *)dev->config->config_info;
	ADC_TypeDef *adc = config->base;

	*data->buffer = adc->DR;
	adc_context_on_sampling_done(&data->ctx, dev);

	LOG_INF("ISR triggered.");
}

static int adc_stm32_read(struct device *dev,
			  const struct adc_sequence *sequence)
{
	struct adc_stm32_data *data = dev->driver_data;

	adc_context_lock(&data->ctx, false, NULL);

	return start_read(dev, sequence);
}

#ifdef CONFIG_ADC_ASYNC
static int adc_stm32_read_async(struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct adc_stm32_data *data = dev->driver_data;

	adc_context_lock(&data->ctx, true, async);

	return start_read(dev, sequence);
}
#endif

int adc_stm32_channel_setup(struct device *dev,
			    const struct adc_channel_cfg *channel_cfg)
{

	if (channel_cfg->channel_id > 18) {
		LOG_ERR("Channel %d is not valid", channel_cfg->channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Invalid channel acquisition time");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Invalid channel gain");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Invalid channel reference");
		return -EINVAL;
	}

	LOG_INF("Channel setup succeeded!");

	return 0;
}

static int adc_stm32_init(struct device *dev)
{
	struct adc_stm32_data *data = dev->driver_data;
	const struct adc_stm32_cfg *cfg = dev->config->config_info;
	struct device *clk =
		device_get_binding(STM32_CLOCK_CONTROL_NAME);

	LOG_DBG("Initializing ADC of STM32");

	data->dev = dev;

	if (clock_control_on(clk,
		(clock_control_subsys_t *) &cfg->pclken) != 0) {
		return -EIO;
	}

	adc_stm32_init_soc(cfg->base);

	cfg->irq_cfg_func();

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static struct adc_driver_api api_stm32_driver_api = {
	.channel_setup = adc_stm32_channel_setup,
	.read = adc_stm32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_stm32_read_async,
#endif
};

#define STM32_ADC_INIT(index)						\
									\
static void adc_stm32_cfg_func_##index(void);				\
									\
static const struct adc_stm32_cfg adc_stm32_cfg_##index = {		\
	.base = (ADC_TypeDef *)DT_ADC_##index##_BASE_ADDRESS,		\
	.irq_cfg_func = adc_stm32_cfg_func_##index,			\
	.pclken = {							\
		.enr = DT_ADC_##index##_CLOCK_BITS,			\
		.bus = DT_ADC_##index##_CLOCK_BUS,			\
	},								\
};									\
static struct adc_stm32_data adc_stm32_data_##index = {			\
	ADC_CONTEXT_INIT_TIMER(adc_stm32_data_##index, ctx),		\
	ADC_CONTEXT_INIT_LOCK(adc_stm32_data_##index, ctx),		\
	ADC_CONTEXT_INIT_SYNC(adc_stm32_data_##index, ctx),		\
};									\
									\
DEVICE_AND_API_INIT(adc_##index, DT_ADC_##index##_NAME, &adc_stm32_init,\
		    &adc_stm32_data_##index, &adc_stm32_cfg_##index,	\
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
		    &api_stm32_driver_api);				\
									\
static void adc_stm32_cfg_func_##index(void)				\
{									\
	IRQ_CONNECT(DT_ADC_##index##_IRQ, DT_ADC_##index##_IRQ_PRI,	\
		    adc_stm32_isr, DEVICE_GET(adc_##index), 0);		\
	irq_enable(DT_ADC_##index##_IRQ);				\
}

#ifdef CONFIG_ADC_0
STM32_ADC_INIT(0)
#endif /* CONFIG_ADC_0 */

#ifdef CONFIG_ADC_1
STM32_ADC_INIT(1)
#endif /* CONFIG_ADC_1 */

#ifdef CONFIG_ADC_2
STM32_ADC_INIT(2)
#endif /* CONFIG_ADC_2 */
