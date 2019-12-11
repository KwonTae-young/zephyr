/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <errno.h>
#include <lorawan.h>
#include <misc/util.h>
#include <zephyr.h>

/* If commented, it operates in ABP mode */
//#define OTTA

#define LORAWAN_DEV_EUI			{ 0xDD, 0xEE, 0xAA, 0xDD, 0xBB, 0xEE,\
					  0xEE, 0xFF }
#define LORAWAN_JOIN_EUI		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00 }
#define LORAWAN_APP_EUI			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00 }
#define LORAWAN_APP_KEY         	{ 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE,\
					  0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88,\
					  0x09, 0xCF, 0x4F, 0x3C }
#define LORAWAN_DEFAULT_DATARATE	LORAWAN_DR_0

#ifndef OTTA
#define LORAWAN_NET_ID			0
#define LORAWAN_DEV_ADDR		0
#define LORAWAN_F_NWK_S_INT_KEY		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00, 0x00, 0x00 }
#define LORAWAN_S_NWK_S_INT_KEY		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00, 0x00, 0x00 }
#define LORAWAN_NWK_S_ENC_KEY		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00, 0x00, 0x00 }
#define LORAWAN_APP_S_KEY		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00, 0x00, 0x00 }
#endif


#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lora_receive);

void main(void)
{
	struct device *lora_dev;
	struct lorawan_mib_config mib_config;
	enum lorawan_act_type act_type;
	int ret;
	u8_t dev_eui[] = LORAWAN_DEV_EUI;
	u8_t join_eui[] = LORAWAN_JOIN_EUI;
	u8_t app_eui[] = LORAWAN_APP_EUI;
	u8_t app_key[] = LORAWAN_APP_KEY;
#ifndef OTTA
	u32_t net_id = LORAWAN_NET_ID;
	u32_t dev_addr = LORAWAN_DEV_ADDR;
	u8_t s_nwk_s_int_key[] = LORAWAN_S_NWK_S_INT_KEY;
	u8_t f_nwk_s_int_key[] = LORAWAN_F_NWK_S_INT_KEY;
	u8_t nwk_s_enc_key[] = LORAWAN_NWK_S_ENC_KEY;
	u8_t app_s_key[] = LORAWAN_APP_S_KEY;
#endif
//	u8_t data[MAX_DATA_LEN] = {0};

#ifdef OTTA
	act_type = LORAWAN_ACT_OTAA;
#else
	act_type = LORAWAN_ACT_ABP;
#endif

	lora_dev = device_get_binding(DT_INST_0_SEMTECH_SX1276_LABEL);
	if (!lora_dev) {
		LOG_ERR("%s Device not found", DT_INST_0_SEMTECH_SX1276_LABEL);
		return;
	}

	mib_config.lw_class = LORAWAN_CLASS_A;
	mib_config.dev_eui = dev_eui;
	mib_config.join_eui = join_eui;
	mib_config.app_eui = app_eui;
	mib_config.app_key = app_key;
	mib_config.nwk_key = app_key;
	if (act_type == LORAWAN_ACT_ABP) {
		mib_config.net_id = net_id;
		mib_config.dev_addr = dev_addr;
		mib_config.f_nwk_s_int_key = f_nwk_s_int_key;
		mib_config.s_nwk_s_int_key = s_nwk_s_int_key;
		mib_config.nwk_s_enc_key = nwk_s_enc_key;
		mib_config.app_s_key = app_s_key;
	}
	mib_config.rx2_datarate = LORAWAN_DEFAULT_DATARATE;
	mib_config.rx2_freq = 865100000;
	mib_config.adr_enable = true;

	LOG_INF("Configuring MIB");
	ret = lorawan_config(&mib_config, act_type);
	if (ret < 0) {
		return;
	}

	LOG_INF("Joining network over %s",
			(act_type == LORAWAN_ACT_OTAA) ? "OTTA" : "ABP");
	ret = lorawan_join_network(LORAWAN_DEFAULT_DATARATE, act_type);
	if (ret < 0) {
		return;
	}

	LOG_INF("Network join request sent!");
}
