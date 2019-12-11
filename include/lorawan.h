/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_LORAWAN_H_
#define ZEPHYR_INCLUDE_LORAWAN_H_

/**
 * @file
 * @brief Public LoRaWAN APIs
 */

#include <zephyr/types.h>
#include <device.h>

enum lorawan_class {
	LORAWAN_CLASS_A = 0x00,
	LORAWAN_CLASS_B = 0x01,
	LORAWAN_CLASS_C = 0x02,
};

enum lorawan_act_type {
	LORAWAN_ACT_OTAA = 0,
	LORAWAN_ACT_ABP,
};

enum lorawan_datarate {
	LORAWAN_DR_0 = 0,
	LORAWAN_DR_1,
	LORAWAN_DR_2,
	LORAWAN_DR_3,
	LORAWAN_DR_4,
	LORAWAN_DR_5,
	LORAWAN_DR_6,
	LORAWAN_DR_7,
	LORAWAN_DR_8,
	LORAWAN_DR_9,
	LORAWAN_DR_10,
	LORAWAN_DR_11,
	LORAWAN_DR_12,
	LORAWAN_DR_13,
	LORAWAN_DR_14,
	LORAWAN_DR_15,
};

struct lorawan_mib_config {
	enum lorawan_class lw_class;
	enum lorawan_datarate rx2_datarate;
	u32_t rx2_freq;
	u32_t net_id;
	u32_t dev_addr;
	u8_t *dev_eui;
	u8_t *join_eui;
	u8_t *app_eui;
	u8_t *app_key;
	u8_t *nwk_key;
	u8_t *f_nwk_s_int_key;
	u8_t *s_nwk_s_int_key;
	u8_t *nwk_s_enc_key;
	u8_t *app_s_key;
	bool adr_enable;
};

extern int lorawan_config(struct lorawan_mib_config *mib_config,
				enum lorawan_act_type mode);
extern int lorawan_join_network(enum lorawan_datarate datarate,
				enum lorawan_act_type mode);

#endif	/* ZEPHYR_INCLUDE_LORAWAN_H_ */
