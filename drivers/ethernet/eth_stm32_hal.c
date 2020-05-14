/*
 * Copyright (c) 2017 Erwin Rol <erwin@erwinrol.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME eth_stm32_hal
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <kernel.h>
#include <device.h>
#include <sys/__assert.h>
#include <sys/util.h>
#include <errno.h>
#include <stdbool.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>
#include <soc.h>
#include <sys/printk.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/stm32_clock_control.h>

#include "eth_stm32_hal_priv.h"

#if defined(CONFIG_ETH_STM32_HAL_USE_DTCM_FOR_DMA_BUFFER) && \
	    !DT_HAS_NODE(DT_CHOSEN(zephyr_dtcm))
#error DTCM for DMA buffer is activated but zephyr,dtcm is not present in dts
#endif

#if defined(CONFIG_SOC_SERIES_STM32H7X)
#define ETH_MULTICASTFRAMESFILTER_PERFECTHASHTABLE    ((uint32_t)0x00000404U)
#define ETH_MULTICASTFRAMESFILTER_HASHTABLE           ((uint32_t)0x00000004U)
#define ETH_MULTICASTFRAMESFILTER_PERFECT             ((uint32_t)0x00000000U)
#define ETH_MULTICASTFRAMESFILTER_NONE                ((uint32_t)0x00000010U)
#define ETH_RXBUFNB 4
#define ETH_TXBUFNB 4
#define PHY_BSR                         ((uint16_t)0x01U)
#define PHY_LINKED_STATUS               ((uint16_t)0x0004U)
#endif

#if defined(CONFIG_ETH_STM32_HAL_USE_DTCM_FOR_DMA_BUFFER) && \
	    DT_HAS_NODE(DT_CHOSEN(zephyr_dtcm))
static ETH_DMADescTypeDef dma_rx_desc_tab[ETH_RXBUFNB] __dtcm_noinit_section;
static ETH_DMADescTypeDef dma_tx_desc_tab[ETH_TXBUFNB] __dtcm_noinit_section;
static u8_t dma_rx_buffer[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __dtcm_noinit_section;
static u8_t dma_tx_buffer[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __dtcm_noinit_section;
#else
static ETH_DMADescTypeDef dma_rx_desc_tab[ETH_RXBUFNB] __aligned(4);
static ETH_DMADescTypeDef dma_tx_desc_tab[ETH_TXBUFNB] __aligned(4);
static u8_t dma_rx_buffer[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __aligned(4);
static u8_t dma_tx_buffer[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __aligned(4);
#endif /* CONFIG_ETH_STM32_HAL_USE_DTCM_FOR_DMA_BUFFER */

#if defined(CONFIG_NET_L2_CANBUS_ETH_TRANSLATOR)
#include <net/can.h>

static void set_mac_to_translator_addr(u8_t *mac_addr)
{
	/* Set the last 14 bit to the translator  link layer address to avoid
	 * address collissions with the 6LoCAN address range
	 */
	mac_addr[4] = (mac_addr[4] & 0xC0) | (NET_CAN_ETH_TRANSLATOR_ADDR >> 8);
	mac_addr[5] = NET_CAN_ETH_TRANSLATOR_ADDR & 0xFF;
}

static void enable_canbus_eth_translator_filter(ETH_HandleTypeDef *heth,
						u8_t *mac_addr)
{
	heth->Instance->MACA1LR = (mac_addr[3] << 24U) | (mac_addr[2] << 16U) |
				  (mac_addr[1] << 8U) | mac_addr[0];
	/*enable filter 1 and ignore byte 5 and 6 for filtering*/
	heth->Instance->MACA1HR = ETH_MACA1HR_AE |  ETH_MACA1HR_MBC_HBits15_8 |
				  ETH_MACA1HR_MBC_HBits7_0;
}
#endif /*CONFIG_NET_L2_CANBUS_ETH_TRANSLATOR*/

static inline void disable_mcast_filter(ETH_HandleTypeDef *heth)
{
	__ASSERT_NO_MSG(heth != NULL);

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	ETH_MACFilterConfigTypeDef mac_filter_conf;
	mac_filter_conf.PromiscuousMode = ENABLE;
	mac_filter_conf.HashUnicast = DISABLE;
	mac_filter_conf.HashMulticast = DISABLE;
	mac_filter_conf.DestAddrInverseFiltering = DISABLE;
	mac_filter_conf.PassAllMulticast = ENABLE;
	mac_filter_conf.BroadcastFilter = ENABLE;
	mac_filter_conf.ControlPacketsFilter = DISABLE;
	mac_filter_conf.SrcAddrInverseFiltering = DISABLE;
	mac_filter_conf.SrcAddrFiltering = DISABLE;
	mac_filter_conf.HachOrPerfectFilter = DISABLE;
	mac_filter_conf.ReceiveAllMode = ENABLE;
	HAL_ETH_SetMACFilterConfig(heth, &mac_filter_conf);
#else
	u32_t tmp = heth->Instance->MACFFR;

	/* disable multicast filtering */
	tmp &= ~(ETH_MULTICASTFRAMESFILTER_PERFECTHASHTABLE |
		 ETH_MULTICASTFRAMESFILTER_HASHTABLE |
		 ETH_MULTICASTFRAMESFILTER_PERFECT);

	/* enable receiving all multicast frames */
	tmp |= ETH_MULTICASTFRAMESFILTER_NONE;

	heth->Instance->MACFFR = tmp;

	/* Wait until the write operation will be taken into account:
	 * at least four TX_CLK/RX_CLK clock cycles
	 */
	tmp = heth->Instance->MACFFR;
	k_sleep(K_MSEC(1));
	heth->Instance->MACFFR = tmp;
#endif
}

static int eth_tx(struct device *dev, struct net_pkt *pkt)
{
	struct eth_stm32_hal_dev_data *dev_data = DEV_DATA(dev);
	ETH_HandleTypeDef *heth;
	u8_t *dma_buffer;
	int res;
	u16_t total_len;
	__IO ETH_DMADescTypeDef *dma_tx_desc;

	LOG_DBG("%s() %dLine", __func__, __LINE__);

	__ASSERT_NO_MSG(pkt != NULL);
	__ASSERT_NO_MSG(pkt->frags != NULL);
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(dev_data != NULL);

	heth = &dev_data->heth;

	k_mutex_lock(&dev_data->tx_mutex, K_FOREVER);

	total_len = net_pkt_get_len(pkt);
	if (total_len > ETH_TX_BUF_SIZE) {
		LOG_ERR("PKT to big");
		res = -EIO;
		goto error;
	}

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	dma_tx_desc = heth->Init.TxDesc;
#else
	dma_tx_desc = heth->TxDesc;
#endif
	while ((dma_tx_desc->DESC3 & ETH_DMATXNDESCRF_OWN) != (u32_t)RESET) {
		k_yield();
	}
#if defined(CONFIG_SOC_SERIES_STM32H7X)
	dma_buffer = (u8_t *)(dma_tx_desc->DESC0);
#else
	dma_buffer = (u8_t *)(dma_tx_desc->Buffer1Addr);
#endif

	if (net_pkt_read(pkt, dma_buffer, total_len)) {
		res = -EIO;
		goto error;
	}

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	ETH_TxPacketConfig txconfig;

	memset(&txconfig, 0, sizeof(ETH_TxPacketConfig));
	txconfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM |
					ETH_TX_PACKETS_FEATURES_CRCPAD;
	txconfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
	txconfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
	txconfig.Length = total_len;

	if (HAL_ETH_Transmit(heth, &txconfig, 20) != HAL_OK) {
#else
	if (HAL_ETH_TransmitFrame(heth, total_len) != HAL_OK) {
#endif
		LOG_ERR("HAL_ETH_TransmitFrame failed");
		res = -EIO;
		goto error;
	}

#if !defined(CONFIG_SOC_SERIES_STM32H7X)
	/* When Transmit Underflow flag is set, clear it and issue a
	 * Transmit Poll Demand to resume transmission.
	 */
	if ((heth->Instance->DMASR & ETH_DMASR_TUS) != (u32_t)RESET) {
		/* Clear TUS ETHERNET DMA flag */
		heth->Instance->DMASR = ETH_DMASR_TUS;
		/* Resume DMA transmission*/
		heth->Instance->DMATPDR = 0;
		res = -EIO;
		goto error;
	}
#endif

	res = 0;
error:
	k_mutex_unlock(&dev_data->tx_mutex);

	return res;
}

static struct net_if *get_iface(struct eth_stm32_hal_dev_data *ctx,
				u16_t vlan_tag)
{
#if defined(CONFIG_NET_VLAN)
	struct net_if *iface;

	iface = net_eth_get_vlan_iface(ctx->iface, vlan_tag);
	if (!iface) {
		return ctx->iface;
	}

	return iface;
#else
	ARG_UNUSED(vlan_tag);

	return ctx->iface;
#endif
}

static struct net_pkt *eth_rx(struct device *dev, u16_t *vlan_tag)
{
	struct eth_stm32_hal_dev_data *dev_data;
	ETH_HandleTypeDef *heth;
	__IO ETH_DMADescTypeDef *dma_rx_desc;
	struct net_pkt *pkt;
	u16_t total_len;
	u8_t *dma_buffer;
	int i;

	LOG_DBG("%s() %dLine", __func__, __LINE__);

	__ASSERT_NO_MSG(dev != NULL);

	dev_data = DEV_DATA(dev);

	__ASSERT_NO_MSG(dev_data != NULL);

	heth = &dev_data->heth;


#if defined(CONFIG_SOC_SERIES_STM32H7X)
	if (HAL_ETH_IsRxDataAvailable(heth) != true) {
		/* no frame available */
		return NULL;
	}

	ETH_BufferTypeDef rxbuff;
	u32_t framelength;

	HAL_ETH_GetRxDataBuffer(heth, &rxbuff);
	HAL_ETH_GetRxDataLength(heth, &framelength);
	total_len = framelength;
#else
	if (HAL_ETH_GetReceivedFrame_IT(heth) != HAL_OK) {
		/* no frame available */
		return NULL;
	}

	total_len = heth->RxFrameInfos.length;
	dma_buffer = (u8_t *)heth->RxFrameInfos.buffer;
#endif

	pkt = net_pkt_rx_alloc_with_buffer(get_iface(dev_data, *vlan_tag),
					   total_len, AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		LOG_ERR("Failed to obtain RX buffer");
		goto release_desc;
	}

	if (net_pkt_write(pkt, dma_buffer, total_len)) {
		LOG_ERR("Failed to append RX buffer to context buffer");
		net_pkt_unref(pkt);
		pkt = NULL;
		goto release_desc;
	}

release_desc:
#if !defined(CONFIG_SOC_SERIES_STM32H7X)
	/* Release descriptors to DMA */
	/* Point to first descriptor */
	dma_rx_desc = heth->RxFrameInfos.FSRxDesc;
	/* Set Own bit in Rx descriptors: gives the buffers back to DMA */
	for (i = 0; i < heth->RxFrameInfos.SegCount; i++) {
		dma_rx_desc->Status |= ETH_DMARXDESC_OWN;
		dma_rx_desc = (ETH_DMADescTypeDef *)
			(dma_rx_desc->Buffer2NextDescAddr);
	}

	/* Clear Segment_Count */
	heth->RxFrameInfos.SegCount = 0;

	/* When Rx Buffer unavailable flag is set: clear it
	 * and resume reception.
	 */
	if ((heth->Instance->DMASR & ETH_DMASR_RBUS) != (u32_t)RESET) {
		/* Clear RBUS ETHERNET DMA flag */
		heth->Instance->DMASR = ETH_DMASR_RBUS;
		/* Resume DMA reception */
		heth->Instance->DMARPDR = 0;
	}
#endif

#if defined(CONFIG_NET_VLAN)
	struct net_eth_hdr *hdr = NET_ETH_HDR(pkt);

	if (ntohs(hdr->type) == NET_ETH_PTYPE_VLAN) {
		struct net_eth_vlan_hdr *hdr_vlan =
			(struct net_eth_vlan_hdr *)NET_ETH_HDR(pkt);

		net_pkt_set_vlan_tci(pkt, ntohs(hdr_vlan->vlan.tci));
		*vlan_tag = net_pkt_vlan_tag(pkt);

#if CONFIG_NET_TC_RX_COUNT > 1
		enum net_priority prio;

		prio = net_vlan2priority(net_pkt_vlan_priority(pkt));
		net_pkt_set_priority(pkt, prio);
#endif
	} else {
		net_pkt_set_iface(pkt, dev_data->iface);
	}
#endif /* CONFIG_NET_VLAN */

	if (!pkt) {
		eth_stats_update_errors_rx(get_iface(dev_data, *vlan_tag));
	}

	return pkt;
}

static void rx_thread(void *arg1, void *unused1, void *unused2)
{
	u16_t vlan_tag = NET_VLAN_TAG_UNSPEC;
	struct device *dev;
	struct eth_stm32_hal_dev_data *dev_data;
	struct net_pkt *pkt;
	int res;
	u32_t status;

	LOG_DBG("%s() %dLine", __func__, __LINE__);

	__ASSERT_NO_MSG(arg1 != NULL);
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	dev = (struct device *)arg1;
	dev_data = DEV_DATA(dev);

	__ASSERT_NO_MSG(dev_data != NULL);

	while (1) {
		LOG_DBG("ETH_DMACSR: 0x%x", dev_data->heth.Instance->DMACSR);
		LOG_DBG("ETH_MTLQICSR: 0x%x", dev_data->heth.Instance->MTLQICSR);
		LOG_DBG("ETH_MACPCSR: 0x%x", dev_data->heth.Instance->MACPCSR);
		LOG_DBG("ETH_MACLCSR: 0x%x", dev_data->heth.Instance->MACLCSR);
		LOG_DBG("");

		res = k_sem_take(&dev_data->rx_int_sem,
			K_MSEC(CONFIG_ETH_STM32_CARRIER_CHECK_RX_IDLE_TIMEOUT_MS));
		if (res == 0) {
			LOG_DBG("%s() %dLine", __func__, __LINE__);
			/* semaphore taken, update link status and receive packets */
			if (dev_data->link_up != true) {
				dev_data->link_up = true;
				net_eth_carrier_on(get_iface(dev_data,
							     vlan_tag));
			}
			while ((pkt = eth_rx(dev, &vlan_tag)) != NULL) {
				res = net_recv_data(net_pkt_iface(pkt), pkt);
				if (res < 0) {
					eth_stats_update_errors_rx(
							net_pkt_iface(pkt));
					LOG_ERR("Failed to enqueue frame "
						"into RX queue: %d", res);
					net_pkt_unref(pkt);
				}
			}
		} else if (res == -EAGAIN) {
			/* semaphore timeout period expired, check link status */
#if defined(CONFIG_SOC_SERIES_STM32H7X)
			if (HAL_ETH_ReadPHYRegister(&dev_data->heth,
				CONFIG_ETH_STM32_HAL_PHY_ADDRESS, PHY_BSR,
				(uint32_t *) &status) == HAL_OK) {
#else
			if (HAL_ETH_ReadPHYRegister(&dev_data->heth, PHY_BSR,
				(uint32_t *) &status) == HAL_OK) {
#endif
				if ((status & PHY_LINKED_STATUS) == PHY_LINKED_STATUS) {
					if (dev_data->link_up != true) {
						LOG_DBG("Link UP!");
						dev_data->link_up = true;
						net_eth_carrier_on(
							get_iface(dev_data,
								  vlan_tag));
					}
				} else {
					if (dev_data->link_up != false) {
						LOG_DBG("Link DOWN!");
						dev_data->link_up = false;
						net_eth_carrier_off(
							get_iface(dev_data,
								  vlan_tag));
					}
				}
			}
		}
	}
}

static void eth_isr(void *arg)
{
	struct device *dev;
	struct eth_stm32_hal_dev_data *dev_data;
	ETH_HandleTypeDef *heth;

	LOG_ERR("%s() %dLine!!", __func__, __LINE__);

	__ASSERT_NO_MSG(arg != NULL);

	dev = (struct device *)arg;
	dev_data = DEV_DATA(dev);

	__ASSERT_NO_MSG(dev_data != NULL);

	heth = &dev_data->heth;

	__ASSERT_NO_MSG(heth != NULL);

	HAL_ETH_IRQHandler(heth);
}


void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth_handle)
{
	LOG_DBG("%s() %dLine!!", __func__, __LINE__);

	__ASSERT_NO_MSG(heth_handle != NULL);

	struct eth_stm32_hal_dev_data *dev_data =
		CONTAINER_OF(heth_handle, struct eth_stm32_hal_dev_data, heth);

	__ASSERT_NO_MSG(dev_data != NULL);

	k_sem_give(&dev_data->rx_int_sem);
}

#if defined(CONFIG_ETH_STM32_HAL_RANDOM_MAC)
static void generate_mac(u8_t *mac_addr)
{
	u32_t entropy;

	entropy = sys_rand32_get();

	mac_addr[0] |= 0x02; /* force LAA bit */

	mac_addr[3] = entropy >> 16;
	mac_addr[4] = entropy >> 8;
	mac_addr[5] = entropy >> 0;
}
#endif

static int eth_initialize(struct device *dev)
{
	struct eth_stm32_hal_dev_data *dev_data;
	struct eth_stm32_hal_dev_cfg *cfg;
	ETH_HandleTypeDef *heth;
	u8_t hal_ret;
	int ret = 0;

	__ASSERT_NO_MSG(dev != NULL);

	dev_data = DEV_DATA(dev);
	cfg = DEV_CFG(dev);

	__ASSERT_NO_MSG(dev_data != NULL);
	__ASSERT_NO_MSG(cfg != NULL);

	dev_data->clock = device_get_binding(STM32_CLOCK_CONTROL_NAME);
	__ASSERT_NO_MSG(dev_data->clock != NULL);

	/* enable clock */
	ret = clock_control_on(dev_data->clock,
		(clock_control_subsys_t *)&cfg->pclken);
	ret |= clock_control_on(dev_data->clock,
		(clock_control_subsys_t *)&cfg->pclken_tx);
	ret |= clock_control_on(dev_data->clock,
		(clock_control_subsys_t *)&cfg->pclken_rx);
#if !defined(CONFIG_SOC_SERIES_STM32H7X)
	ret |= clock_control_on(dev_data->clock,
		(clock_control_subsys_t *)&cfg->pclken_ptp);
#endif

	if (ret) {
		LOG_ERR("Failed to enable ethernet clock");
		return -EIO;
	}

	__ASSERT_NO_MSG(cfg->config_func != NULL);

	cfg->config_func();

	heth = &dev_data->heth;

#if defined(CONFIG_ETH_STM32_HAL_RANDOM_MAC)
	generate_mac(dev_data->mac_addr);
#endif
#if defined(CONFIG_NET_L2_CANBUS_ETH_TRANSLATOR)
	set_mac_to_translator_addr(dev_data->mac_addr);
#endif

	heth->Init.MACAddr = dev_data->mac_addr;
#if defined(CONFIG_SOC_SERIES_STM32H7X)
	heth->Init.TxDesc = dma_tx_desc_tab;
	heth->Init.RxDesc = dma_rx_desc_tab;
	heth->Init.RxBuffLen = ETH_RX_BUF_SIZE;
#endif
	hal_ret = HAL_ETH_Init(heth);
	if (hal_ret == HAL_TIMEOUT) {
		/* HAL Init time out. This could be linked to */
		/* a recoverable error. Log the issue and continue */
		/* driver initialisation */
		LOG_ERR("HAL_ETH_Init Timed out");
	} else if (hal_ret != HAL_OK) {
		LOG_ERR("HAL_ETH_Init failed: %d", hal_ret);
		return -EINVAL;
	}



	dev_data->link_up = false;

	/* Initialize semaphores */
	k_mutex_init(&dev_data->tx_mutex);
	k_sem_init(&dev_data->rx_int_sem, 0, UINT_MAX);

	/* Start interruption-poll thread */
	k_thread_create(&dev_data->rx_thread, dev_data->rx_thread_stack,
			K_THREAD_STACK_SIZEOF(dev_data->rx_thread_stack),
			rx_thread, (void *) dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_STM32_HAL_RX_THREAD_PRIO),
			0, K_NO_WAIT);

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	for (int idx = 0; idx < ETH_RXBUFNB; idx ++)	{
		HAL_ETH_DescAssignMemory(heth, idx, dma_rx_buffer[idx], NULL);
	}
#else
	HAL_ETH_DMATxDescListInit(heth, dma_tx_desc_tab,
		&dma_tx_buffer[0][0], ETH_TXBUFNB);
	HAL_ETH_DMARxDescListInit(heth, dma_rx_desc_tab,
		&dma_rx_buffer[0][0], ETH_RXBUFNB);
#endif

	/*
	 * LAN8742(PHY Chip) Register Read
	 *
	 * Basic Control Register: 0x3000
	 * Basic Status Register: 0x7809
	 * PHY Identifier 1 Register: 0x7
	 * PHY Identifier 2 Register: 0xc131
	 */
	u32_t status;

	HAL_ETH_ReadPHYRegister(&dev_data->heth,
					CONFIG_ETH_STM32_HAL_PHY_ADDRESS,
					0x00, (uint32_t *) &status);
	LOG_DBG("Basic Control Register: 0x%x", status);

	HAL_ETH_ReadPHYRegister(&dev_data->heth,
					CONFIG_ETH_STM32_HAL_PHY_ADDRESS,
					0x01, (uint32_t *) &status);
	LOG_DBG("Basic Status Register: 0x%x", status);

	HAL_ETH_ReadPHYRegister(&dev_data->heth,
					CONFIG_ETH_STM32_HAL_PHY_ADDRESS,
					0x02, (uint32_t *) &status);
	LOG_DBG("PHY Identifier 1 Register: 0x%x", status);

	HAL_ETH_ReadPHYRegister(&dev_data->heth,
					CONFIG_ETH_STM32_HAL_PHY_ADDRESS,
					0x03, (uint32_t *) &status);
	LOG_DBG("PHY Identifier 2 Register: 0x%x", status);

	ETH_MACConfigTypeDef mac_conf;
	HAL_ETH_GetMACConfig(heth, &mac_conf);
	mac_conf.DuplexMode = ETH_FULLDUPLEX_MODE;
	mac_conf.Speed = ETH_SPEED_100M;
	HAL_ETH_SetMACConfig(heth, &mac_conf);

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	HAL_ETH_Start_IT(heth);
#else
	HAL_ETH_Start(heth);
#endif

	disable_mcast_filter(heth);

#if defined(CONFIG_NET_L2_CANBUS_ETH_TRANSLATOR)
	enable_canbus_eth_translator_filter(heth, dev_data->mac_addr);
#endif

	LOG_DBG("MAC %02x:%02x:%02x:%02x:%02x:%02x",
		dev_data->mac_addr[0], dev_data->mac_addr[1],
		dev_data->mac_addr[2], dev_data->mac_addr[3],
		dev_data->mac_addr[4], dev_data->mac_addr[5]);

	LOG_DBG("ETH_DMACSR: 0x%x", heth->Instance->DMACSR);
	LOG_DBG("ETH_MTLISR: 0x%x", heth->Instance->MTLISR);
	LOG_DBG("ETH_MACISR: 0x%x", heth->Instance->MACISR);

	return 0;
}

static void eth_iface_init(struct net_if *iface)
{
	struct device *dev;
	struct eth_stm32_hal_dev_data *dev_data;

	LOG_DBG("%s() %dLine", __func__, __LINE__);

	__ASSERT_NO_MSG(iface != NULL);

	dev = net_if_get_device(iface);
	__ASSERT_NO_MSG(dev != NULL);

	dev_data = DEV_DATA(dev);
	__ASSERT_NO_MSG(dev_data != NULL);

	/* For VLAN, this value is only used to get the correct L2 driver.
	 * The iface pointer in context should contain the main interface
	 * if the VLANs are enabled.
	 */
	if (dev_data->iface == NULL) {
		dev_data->iface = iface;
	}

	/* Register Ethernet MAC Address with the upper layer */
	net_if_set_link_addr(iface, dev_data->mac_addr,
			     sizeof(dev_data->mac_addr),
			     NET_LINK_ETHERNET);

	ethernet_init(iface);

	net_if_flag_set(iface, NET_IF_NO_AUTO_START);
}

static enum ethernet_hw_caps eth_stm32_hal_get_capabilities(struct device *dev)
{
	LOG_DBG("%s() %dLine", __func__, __LINE__);

	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T
#if defined(CONFIG_NET_VLAN)
		| ETHERNET_HW_VLAN
#endif
		;
}

static int eth_stm32_hal_set_config(struct device *dev,
				    enum ethernet_config_type type,
				    const struct ethernet_config *config)
{
	struct eth_stm32_hal_dev_data *dev_data;
	ETH_HandleTypeDef *heth;

	LOG_DBG("%s() %dLine", __func__, __LINE__);

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		dev_data = DEV_DATA(dev);
		heth = &dev_data->heth;

		memcpy(dev_data->mac_addr, config->mac_address.addr, 6);
		heth->Instance->MACA0HR = (dev_data->mac_addr[5] << 8) |
			dev_data->mac_addr[4];
		heth->Instance->MACA0LR = (dev_data->mac_addr[3] << 24) |
			(dev_data->mac_addr[2] << 16) |
			(dev_data->mac_addr[1] << 8) |
			dev_data->mac_addr[0];
		return 0;
	default:
		break;
	}

	return -ENOTSUP;
}

static const struct ethernet_api eth_api = {
	.iface_api.init = eth_iface_init,

	.get_capabilities = eth_stm32_hal_get_capabilities,
	.set_config = eth_stm32_hal_set_config,
	.send = eth_tx,
};

static struct device DEVICE_NAME_GET(eth0_stm32_hal);

static void eth0_irq_config(void)
{
	LOG_DBG("%s() ETH_IRQn=%d", __func__, ETH_IRQn);

	IRQ_CONNECT(ETH_IRQn, CONFIG_ETH_STM32_HAL_IRQ_PRI, eth_isr,
		    DEVICE_GET(eth0_stm32_hal), 0);
	irq_enable(ETH_IRQn);
}

static const struct eth_stm32_hal_dev_cfg eth0_config = {
	.config_func = eth0_irq_config,
#if defined(CONFIG_SOC_SERIES_STM32H7X)
	.pclken   =   { .bus = STM32_CLOCK_BUS_AHB1,
			.enr = LL_AHB1_GRP1_PERIPH_ETH1MAC },
	.pclken_tx =  { .bus = STM32_CLOCK_BUS_AHB1,
			.enr = LL_AHB1_GRP1_PERIPH_ETH1TX },
	.pclken_rx =  { .bus = STM32_CLOCK_BUS_AHB1,
			.enr = LL_AHB1_GRP1_PERIPH_ETH1RX },
#else
	.pclken   =   { .bus = STM32_CLOCK_BUS_AHB1,
			.enr = LL_AHB1_GRP1_PERIPH_ETHMAC },
	.pclken_tx =  { .bus = STM32_CLOCK_BUS_AHB1,
			.enr = LL_AHB1_GRP1_PERIPH_ETHMACTX },
	.pclken_rx =  { .bus = STM32_CLOCK_BUS_AHB1,
			.enr = LL_AHB1_GRP1_PERIPH_ETHMACRX },
	.pclken_ptp = { .bus = STM32_CLOCK_BUS_AHB1,
			.enr = LL_AHB1_GRP1_PERIPH_ETHMACPTP },
#endif
};

static struct eth_stm32_hal_dev_data eth0_data = {
	.heth = {
		.Instance = ETH,
		.Init = {
#if defined(CONFIG_SOC_SERIES_STM32H7X)
#if defined(CONFIG_ETH_STM32_HAL_MII)
			.MediaInterface = HAL_ETH_MII_MODE,
#else
			.MediaInterface = HAL_ETH_RMII_MODE,
#endif /* CONFIG_ETH_STM32_HAL_MII */
#else
			.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE,
			.PhyAddress = CONFIG_ETH_STM32_HAL_PHY_ADDRESS,
			.RxMode = ETH_RXINTERRUPT_MODE,
			.ChecksumMode = ETH_CHECKSUM_BY_SOFTWARE,
#if defined(CONFIG_ETH_STM32_HAL_MII)
			.MediaInterface = ETH_MEDIA_INTERFACE_MII,
#else
			.MediaInterface = ETH_MEDIA_INTERFACE_RMII,
#endif /* CONFIG_ETH_STM32_HAL_MII */
#endif /* CONFIG_SOC_SERIES_STM32H7X */
		},
	},
	.mac_addr = {
		ST_OUI_B0,
		ST_OUI_B1,
		ST_OUI_B2,
#if !defined(CONFIG_ETH_STM32_HAL_RANDOM_MAC)
		CONFIG_ETH_STM32_HAL_MAC3,
		CONFIG_ETH_STM32_HAL_MAC4,
		CONFIG_ETH_STM32_HAL_MAC5
#endif
	},
};

ETH_NET_DEVICE_INIT(eth0_stm32_hal, CONFIG_ETH_STM32_HAL_NAME, eth_initialize,
		    device_pm_control_nop, &eth0_data, &eth0_config,
		    CONFIG_ETH_INIT_PRIORITY, &eth_api, ETH_STM32_HAL_MTU);
