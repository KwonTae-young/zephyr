/*
 * reference: https://github.com/msolters/arduino-at86rf233
 *
 *AT86RF233 Datasheet:
 *	http://ww1.microchip.com/downloads/en/devicedoc/
 *	atmel-8351-mcu_wireless-at86rf233_datasheet.pdf
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <gpio.h>
#include <spi.h>
#include <string.h>
#include <errno.h>

//#define IEEE802154_USE

/*
 * WRITE(Tx) Mode: #define WRITE
 * READ(Rx) Mode: //#define WRITE
 */
#define WRITE

#define AT86RF2XX_REG__TRX_STATUS	0x01
#define AT86RF2XX_REG__TRX_STATE	0x02
#define AT86RF2XX_REG__TRX_CTRL_0	0x03
#define AT86RF2XX_REG__TRX_CTRL_1	0x04
#define AT86RF2XX_REG__PHY_TX_PWR	0x05
#define AT86RF2XX_REG__PHY_RSSI		0x06
#define AT86RF2XX_REG__PHY_ED_LEVEL	0x07
#define AT86RF2XX_REG__PHY_CC_CCA	0x08
#define AT86RF2XX_REG__CCA_THRES	0x09
#define AT86RF2XX_REG__RX_CTRL		0x0A
#define AT86RF2XX_REG__SFD_VALUE	0x0B
#define AT86RF2XX_REG__TRX_CTRL_2	0x0C
#define AT86RF2XX_REG__ANT_DIV		0x0D
#define AT86RF2XX_REG__IRQ_MASK		0x0E
#define AT86RF2XX_REG__IRQ_STATUS	0x0F
#define AT86RF2XX_REG__VREG_CTRL	0x10
#define AT86RF2XX_REG__BATMON		0x11
#define AT86RF2XX_REG__XOSC_CTRL	0x12
#define AT86RF2XX_REG__CC_CTRL_1	0x14
#define AT86RF2XX_REG__RX_SYN		0x15
#define AT86RF2XX_REG__RF_CTRL_0	0x16
#define AT86RF2XX_REG__XAH_CTRL_1	0x17
#define AT86RF2XX_REG__FTN_CTRL		0x18
#define AT86RF2XX_REG__PLL_CF		0x1A
#define AT86RF2XX_REG__PLL_DCU		0x1B
#define AT86RF2XX_REG__PART_NUM		0x1C
#define AT86RF2XX_REG__VERSION_NUM	0x1D
#define AT86RF2XX_REG__MAN_ID_0		0x1E
#define AT86RF2XX_REG__MAN_ID_1		0x1F
#define AT86RF2XX_REG__SHORT_ADDR_0	0x20
#define AT86RF2XX_REG__SHORT_ADDR_1	0x21
#define AT86RF2XX_REG__PAN_ID_0		0x22
#define AT86RF2XX_REG__PAN_ID_1		0x23
#define AT86RF2XX_REG__IEEE_ADDR_0	0x24
#define AT86RF2XX_REG__IEEE_ADDR_1	0x25
#define AT86RF2XX_REG__IEEE_ADDR_2	0x26
#define AT86RF2XX_REG__IEEE_ADDR_3	0x27
#define AT86RF2XX_REG__IEEE_ADDR_4	0x28
#define AT86RF2XX_REG__IEEE_ADDR_5	0x29
#define AT86RF2XX_REG__IEEE_ADDR_6	0x2A
#define AT86RF2XX_REG__IEEE_ADDR_7	0x2B
#define AT86RF2XX_REG__XAH_CTRL_0	0x2C
#define AT86RF2XX_REG__CSMA_SEED_0	0x2D
#define AT86RF2XX_REG__CSMA_SEED_1	0x2E
#define AT86RF2XX_REG__CSMA_BE		0x2F
#define AT86RF2XX_REG__TST_CTRL_DIGI	0x36


/* 6.3 SPI Protocol */
#define AT86RF2XX_ACCESS_READ	0x00
#define AT86RF2XX_ACCESS_FB	0x20
#define AT86RF2XX_ACCESS_WRITE	0x40
#define AT86RF2XX_ACCESS_REG	0x80

#define AT86RF2XX_PART_NUMBER		0x0B
#define AT86RF2XX_REVISION_A		0x01
#define AT86RF2XX_REVISION_B		0x02
#define AT86RF2XX_MAN_ID		0x001F

#define AT86RF2XX_IRQ_STATUS_MASK__RX_START	0x04

#define AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE		0x02


#define AT86RF2XX_DEFAULT_SHORT_ADDR	0x1234
#define AT86RF2XX_DEFAULT_PAN_ID	0x5678
#define AT86RF2XX_DEFAULT_IEEE_ADDR	0x123456789ABCDEF0


#define AT86RF2XX_PHY_CC_CCA__CHANNEL	0x1F
#define AT86RF2XX_DEFAULT_CHANNEL	12


#define AT86RF2XX_PHY_TX_PWR_MASK	0xF
#define AT86RF2XX_DEFAULT_TXPOWER	0

#define AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES	0x0E
#define AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK		0x10
#define AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1		0x07


#define AT86RF2XX_OPT_AUTOACK		0x0001      /**< auto ACKs active */
#define AT86RF2XX_OPT_CSMA		0x0002       /**< CSMA active */
#define AT86RF2XX_OPT_PROMISCUOUS	0x0004       /**< promiscuous mode active */
//#define AT86RF2XX_OPT_PRELOADING	0x0008       /**< preloading enabled */
//#define AT86RF2XX_OPT_TELL_TX_START	0x0010       /**< notify MAC layer on TX start */
//#define AT86RF2XX_OPT_TELL_TX_END	0x0020       /**< notify MAC layer on TX finished */
#define AT86RF2XX_OPT_TELL_RX_START	0x0040       /**< notify MAC layer on RX start */
#define AT86RF2XX_OPT_TELL_RX_END	0x0080       /**< notify MAC layer on RX finished */
//#define AT86RF2XX_OPT_RAWDUMP		0x0100       /**< pass RAW frame data to upper layer */
//#define AT86RF2XX_OPT_SRC_ADDR_LONG	0x0200       /**< send data using long source address */
//#define AT86RF2XX_OPT_USE_SRC_PAN	0x0400       /**< do not compress source PAN ID */

#define AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE	0x80
#define AT86RF2XX_TRX_CTRL_2_MASK__OQPSK_DATA_RATE	0x03


#define AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL		0x07
#define AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL	0x08

#define AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF		0x00

#define AT86RF2XX_IRQ_STATUS_MASK__TRX_END		0x08



#define AT86RF2XX_STATE_TRX_OFF		0x08	/**< idle */
#define AT86RF2XX_STATE_PLL_ON		0x09	/**< ready to transmit */
#define AT86RF2XX_STATE_SLEEP		0x0f
#define AT86RF2XX_STATE_BUSY_RX_AACK	0x11	/**< busy receiving data */
#define AT86RF2XX_STATE_BUSY_TX_ARET	0x12	/**< busy transmitting data */
#define AT86RF2XX_STATE_RX_AACK_ON	0x16     /**< wait for incoming data */
#define AT86RF2XX_STATE_TX_ARET_ON	0x19	/**< ready for sending data */
#define AT86RF2XX_STATE_IN_PROGRESS	0x1f	/**< ongoing state conversion */

#define AT86RF2XX_TRX_STATUS_MASK__TRX_STATUS	0x1F

#define AT86RF2XX_TRX_STATE__TX_START			0x02
#define AT86RF2XX_TRX_STATE__FORCE_TRX_OFF		0x03


#define AT86RF2XX_RESET_PULSE_WIDTH	100
#define AT86RF2XX_RESET_DELAY		50

#define AT86RF2XX_WAKEUP_DELAY		1	/* 300us... */

#define AT86RF2XX_MAX_PKT_LENGTH	127

#define AT86RF2XX_ACCESS_SRAM		0x00
#define AT86RF2XX_ACCESS_READ		0x00
#define AT86RF2XX_ACCESS_WRITE		0x40


static const int8_t tx_power_to_dbm[] = {
	4, 4, 3, 3, 2, 2, 1, 0, -1, -2, -3, -4, -6, -8 -12, -17
};

uint16_t options;
uint64_t ieee_addr_double;
uint8_t ieee_addr_char[8];
uint8_t state = AT86RF2XX_STATE_SLEEP;
uint8_t idle_state = AT86RF2XX_STATE_TRX_OFF;
uint8_t frame_len;

#define IRQ_FLAGS		(GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_PUD_PULL_UP | GPIO_INT_ACTIVE_HIGH)

static struct k_work data_read_work;

struct device *spi;
struct spi_config spi_cfg;
struct device *sleep;

enum data_rate_type {
	TWO_FIVE_ZERO_KBPS,
	FIVE_ZERO_ZERO_KBPS,
	ONE_MBPS,
	TWO_MBPS,
};

static int at86rf2xx_access(struct device *spi, struct spi_config *spi_cfg,
			    uint8_t cmd, uint16_t addr, void *data, size_t len)
{
	uint8_t access[1];
	struct spi_buf bufs[] = {
		{
			.buf = access,
		},
		{
			.buf = data,
			.len = len
		}
	};
	struct spi_buf_set tx = {
		.buffers = bufs
	};
	
	access[0] = AT86RF2XX_ACCESS_REG | cmd | addr;

	bufs[0].len = 1;
	tx.count = 2;

	if (cmd == AT86RF2XX_ACCESS_READ) {
		struct spi_buf_set rx = {
			.buffers = bufs,
			.count = 2
		};

		return spi_transceive(spi, spi_cfg, &tx, &rx);
	}

	return spi_write(spi, spi_cfg, &tx);
}

static int read_reg(struct device *spi, struct spi_config *spi_cfg,
		      uint16_t addr, uint8_t *data, u32_t num_bytes)
{
	int err;

	err = at86rf2xx_access(spi, spi_cfg,
			       AT86RF2XX_ACCESS_READ, addr, data, num_bytes);
	if (err) {
		printk("Error during SPI read\n");
		return -EIO;
	}

	return 0;
}

static int write_reg(struct device *spi, struct spi_config *spi_cfg,
		      uint16_t addr, uint8_t *data, u32_t num_bytes)
{
	int err;

	err = at86rf2xx_access(spi, spi_cfg,
			       AT86RF2XX_ACCESS_WRITE, addr, data, num_bytes);
	if (err) {
		printk("Error during SPI write\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_sram_access(struct device *spi, struct spi_config *spi_cfg,
			    uint8_t cmd, uint8_t addr, void *data, size_t len)
{
	uint8_t access[2];
	struct spi_buf bufs[] = {
		{
			.buf = access,
		},
		{
			.buf = data,
			.len = len
		}
	};
	struct spi_buf_set tx = {
		.buffers = bufs
	};

	access[0] = cmd;
	access[1] = addr;

	bufs[0].len = 2;
	tx.count = 2;

	if (cmd == AT86RF2XX_ACCESS_READ) {
		struct spi_buf_set rx = {
			.buffers = bufs,
			.count = 2
		};

		return spi_transceive(spi, spi_cfg, &tx, &rx);
	}

	return spi_write(spi, spi_cfg, &tx);
}

static int at86rf2xx_read_sram(struct device *spi, struct spi_config *spi_cfg,
		      uint8_t addr, uint8_t *data, u32_t num_bytes)
{
	int err;

	err = at86rf2xx_sram_access(spi, spi_cfg,
			       AT86RF2XX_ACCESS_READ, addr, data, num_bytes);
	if (err) {
		printk("Error during SPI read\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_write_sram(struct device *spi, struct spi_config *spi_cfg,
		      uint8_t addr, uint8_t *data, u32_t num_bytes)
{
	int err;

	err = at86rf2xx_sram_access(spi, spi_cfg,
			       AT86RF2XX_ACCESS_WRITE, addr, data, num_bytes);
	if (err) {
		printk("Error during SPI write\n");
		return -EIO;
	}

	return 0;
}

/*
static int at86rf2xx_fb_access(struct device *spi, struct spi_config *spi_cfg,
				void *data, const size_t len)
{
	uint8_t access[1];
	struct spi_buf bufs[] = {
		{
			.buf = access,
		}
	};
	struct spi_buf_set tx = {
		.buffers = bufs
	};

	access[0] = AT86RF2XX_ACCESS_FB;
	bufs[0].len = 1;
	tx.count = 1;

	struct spi_buf_set rx = {
		.buffers = bufs,
		.count = 1
	};

	return spi_transceive(spi, spi_cfg, &tx, &rx);
}

static int at86rf2xx_read_fb(struct device *spi, struct spi_config *spi_cfg,
		      uint8_t *data, u32_t num_bytes)
{
	int err;

	err = at86rf2xx_fb_access(spi, spi_cfg, data, num_bytes);
	if (err) {
		printk("Error during SPI read\n");
		return -EIO;
	}

	return 0;
}
*/

static int at86rf2xx_part_number(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t id;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__PART_NUM, &id, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__PART_NUM read\n");
		return -EIO;
	}

	printk("PART_NUM: 0x%x\n", id);

	if (id != AT86RF2XX_PART_NUMBER)
		return -EIO;


	return 0;
}

static int at86rf2xx_version_number(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t version;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__VERSION_NUM, &version, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__VERSION_NUM read\n");
		return -EIO;
	}

	printk("VERSION_NUM: 0x%x\n", version);

	if ((version != AT86RF2XX_REVISION_A) && (version != AT86RF2XX_REVISION_B))
		return -EIO;


	return 0;
}

static int at86rf2xx_man_id(struct device *spi, struct spi_config *spi_cfg)
{
	uint16_t man_id;
	uint8_t id[2];
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__MAN_ID_0, id, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__MAN_ID_0 read\n");
		return -EIO;
	}

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__MAN_ID_1, id + 1, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__MAN_ID_1 read\n");
		return -EIO;
	}

	man_id = (id[1] << 8) | id[0];
	printk("MAN_ID: 0x%x\n", man_id);

	if (man_id != AT86RF2XX_MAN_ID)
		return -EIO;

	return 0;
}

static uint16_t at86rf2xx_get_short_addr(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t addr[2];
	int i, err;

	for (i = 0; i < 2; i++) {
		err = read_reg(spi, spi_cfg, AT86RF2XX_REG__SHORT_ADDR_0 + i, addr + i, 1);
		if (err) {
			printk("Error during AT86RF2XX_REG__SHORT_ADDR_0 + %d read\n", i);
			return -EIO;
		}
	}

	return (addr[0] << 8) | addr[1];
}

static int at86rf2xx_set_short_addr(struct device *spi, struct spi_config *spi_cfg,
					uint16_t short_addr)
{
	uint8_t addr[2];
	int i, err;

	addr[0] = (short_addr >> 8) & 0xFF;
	addr[1] = short_addr & 0xFF;

	for (i = 0; i < 2; i++) {
		err = write_reg(spi, spi_cfg, AT86RF2XX_REG__SHORT_ADDR_0 + i, addr + i, 1);
		if (err) {
			printk("Error during AT86RF2XX_REG__SHORT_ADDR_0 + %d write\n", i);
			return -EIO;
		}
	}

	return 0;
}

static uint16_t at86rf2xx_get_pan_id(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t id[2];
	int i, err;

	for (i = 0; i < 2; i++) {
		err = read_reg(spi, spi_cfg, AT86RF2XX_REG__PAN_ID_0 + i, id + i, 1);
		if (err) {
			printk("Error during AT86RF2XX_REG__PAN_ID_0 + %d read\n", i);
			return -EIO;
		}
	}

	return (id[0] << 8) | id[1];
}

static int at86rf2xx_set_pan_id(struct device *spi, struct spi_config *spi_cfg,
					uint16_t pan_id)
{
	uint8_t id[2];
	int i, err;

	id[0] = (pan_id >> 8) & 0xFF;
	id[1] = pan_id & 0xFF;

	for (i = 0; i < 2; i++) {
		err = write_reg(spi, spi_cfg, AT86RF2XX_REG__PAN_ID_0 + i, id + i, 1);
		if (err) {
			printk("Error during AT86RF2XX_REG__PAN_ID_0 + %d write\n", i);
			return -EIO;
		}
	}

	return 0;
}

static uint64_t at86rf2xx_get_ieee_addr(struct device *spi, struct spi_config *spi_cfg)
{
	uint64_t ieee_addr = 0;
	uint8_t addr[8];
	int i, err;

	for (i = 0; i < 8; i++) {
		err = read_reg(spi, spi_cfg, AT86RF2XX_REG__IEEE_ADDR_0 + i, addr + i, 1);
		if (err) {
			printk("Error during AT86RF2XX_REG__IEEE_ADDR_0 + %d read\n", i);
			return -EIO;
		}

		ieee_addr = (ieee_addr << 8) | addr[i];
	}

	ieee_addr_double = ieee_addr;
	memcpy(ieee_addr_char, addr, sizeof(addr));

	return ieee_addr;
}

static int at86rf2xx_set_ieee_addr(struct device *spi, struct spi_config *spi_cfg,
					uint64_t ieee_addr)
{
	uint8_t addr[8];
	int i, err;

	addr[0] = (ieee_addr >> 56) & 0xFF;
	addr[1] = (ieee_addr >> 48) & 0xFF;
	addr[2] = (ieee_addr >> 40) & 0xFF;
	addr[3] = (ieee_addr >> 32) & 0xFF;
	addr[4] = (ieee_addr >> 24) & 0xFF;
	addr[5] = (ieee_addr >> 16) & 0xFF;
	addr[6] = (ieee_addr >> 8) & 0xFF;
	addr[7] = ieee_addr & 0xFF;

	for (i = 0; i < 8; i++) {
		err = write_reg(spi, spi_cfg, AT86RF2XX_REG__IEEE_ADDR_0 + i, addr + i, 1);
		if (err) {
			printk("Error during AT86RF2XX_REG__IEEE_ADDR_0 + %d write\n", i);
			return -EIO;
		}
	}

	ieee_addr_double = ieee_addr;
	memcpy(ieee_addr_char, addr, sizeof(addr));

	return 0;
}

static uint8_t at86rf2xx_get_channel(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__PHY_CC_CCA, &val , 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__PHY_CC_CCA read\n");
		return -EIO;
	}

	return val & AT86RF2XX_PHY_CC_CCA__CHANNEL;
}

static int at86rf2xx_set_channel(struct device *spi, struct spi_config *spi_cfg,
					uint8_t channel)
{
	uint8_t val;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__PHY_CC_CCA, &val , 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__PHY_CC_CCA read\n");
		return -EIO;
	}

	val &= ~(AT86RF2XX_PHY_CC_CCA__CHANNEL);
	val |= (channel & AT86RF2XX_PHY_CC_CCA__CHANNEL);

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__PHY_CC_CCA, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__PHY_CC_CCA write\n");
		return -EIO;
	}

	return 0;
}

static int8_t at86rf2xx_get_tx_power(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__PHY_TX_PWR, &val , 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__PHY_TX_PWR read\n");
		return -128;
	}

	return tx_power_to_dbm[val & AT86RF2XX_PHY_TX_PWR_MASK];
}

static int at86rf2xx_set_tx_power(struct device *spi, struct spi_config *spi_cfg,
					int8_t tx_power)
{
	int err;

	if ((tx_power < 0) || (tx_power > 15)) {
		printk("tx_power range over\n");
		return -EIO;
	}

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__PHY_TX_PWR, &tx_power , 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__PHY_TX_PWR write\n");
		return -EIO;
	}

	return 0;
}

static int8_t at86rf2xx_get_sfd(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__SFD_VALUE, &val , 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__SFD_VALUE read\n");
		return -EIO;
	}

	return val;
}

static int at86rf2xx_set_sfd(struct device *spi, struct spi_config *spi_cfg,
					uint8_t sfd)
{
	int err;

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__SFD_VALUE, &sfd , 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__SFD_VALUE write\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_set_csma_seed(struct device *spi, struct spi_config *spi_cfg,
					uint8_t entropy[2])
{
	int err;
	uint8_t val;

	if(entropy == NULL)
		return -EIO;

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_0, entropy, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__CSMA_SEED_0 write\n");
		return -EIO;
	}

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__CSMA_SEED_1 read\n");
		return -EIO;
	}
	val &= ~(AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1);
	val |= entropy[1] & AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1;
	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__CSMA_SEED_1 write\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_set_csma_max_retries(struct device *spi, struct spi_config *spi_cfg,
						int8_t retries)
{
	uint8_t val;
	int err;

	retries = (retries > 5) ? 5 : retries;
	retries = (retries < 0) ? 7 : retries;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__XAH_CTRL_0, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__XAH_CTRL_0 read\n");
		return -EIO;
	}
	val &= ~(AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES);
	val |= (retries << 1);
	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__XAH_CTRL_0, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__XAH_CTRL_0 write\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_set_csma_backoff_exp(struct device *spi, struct spi_config *spi_cfg,
						uint8_t min, uint8_t max)
{
	uint8_t val;
	int err;

	max = (max > 8) ? 8 : max;
	min = (min > max) ? max : min;
	val = (max << 4) | min;

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_BE, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__CSMA_BE write\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_set_option(struct device *spi, struct spi_config *spi_cfg,
					uint16_t option, bool state)
{
	uint8_t val;
	int err;

	if (state) {
		options |= option;
		switch (option) {
		case AT86RF2XX_OPT_CSMA:
			err = at86rf2xx_set_csma_seed(spi, spi_cfg, ieee_addr_char);
			if (err) {
				printk("Error during at86rf2xx_set_csma_seed()\n");
				return -EIO;
			}

			err = at86rf2xx_set_csma_max_retries(spi, spi_cfg, 4);
			if (err) {
				printk("Error during at86rf2xx_set_csma_max_retries()\n");
				return -EIO;
			}

			err = at86rf2xx_set_csma_backoff_exp(spi, spi_cfg, 3, 5);
			if (err) {
				printk("Error during at86rf2xx_set_csma_backoff_exp()\n");
				return -EIO;
			}
			break;
		case AT86RF2XX_OPT_PROMISCUOUS:
			err = read_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG___CSMA_SEED_1 read\n");
				return -EIO;
			}
			val |= AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK;
			err = write_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG__CSMA_SEED_1 write\n");
				return -EIO;
			}

			err = read_reg(spi, spi_cfg, AT86RF2XX_REG__XAH_CTRL_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG___CSMA_SEED_1 read\n");
				return -EIO;
			}
			val |= AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE;
			err = write_reg(spi, spi_cfg, AT86RF2XX_REG__XAH_CTRL_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG__XAH_CTRL_1 write\n");
				return -EIO;
			}
			break;
		case AT86RF2XX_OPT_AUTOACK:
			err = read_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG___CSMA_SEED_1 read\n");
				return -EIO;
			}
			val &= ~(AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK);
			err = write_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG__CSMA_SEED_1 write\n");
				return -EIO;
			}
			break;
		case AT86RF2XX_OPT_TELL_RX_START:
			err = read_reg(spi, spi_cfg, AT86RF2XX_REG__IRQ_MASK, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG__IRQ_MASK read\n");
				return -EIO;
			}
			val |= AT86RF2XX_IRQ_STATUS_MASK__RX_START;
			err = write_reg(spi, spi_cfg, AT86RF2XX_REG__IRQ_MASK, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG__IRQ_MASK write\n");
				return -EIO;
			}
			break;
		default:
			/* do nothing */
			return -EIO;
	        }
	} else {
		options &= ~(option);
		switch (option) {
		case AT86RF2XX_OPT_CSMA:
			err = at86rf2xx_set_csma_max_retries(spi, spi_cfg, -1);
			if (err) {
				printk("Error during at86rf2xx_set_csma_max_retries()\n");
				return -EIO;
			}
			break;
		case AT86RF2XX_OPT_PROMISCUOUS:
			err = read_reg(spi, spi_cfg, AT86RF2XX_REG__XAH_CTRL_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG___XAH_CTRL_1 read\n");
				return -EIO;
			}
			val &= ~(AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE);
			err = write_reg(spi, spi_cfg, AT86RF2XX_REG__XAH_CTRL_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG___XAH_CTRL_1 write\n");
				return -EIO;
			}
			if (options & AT86RF2XX_OPT_AUTOACK) {
				err = read_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
				if (err) {
					printk("Error during AT86RF2XX_REG___CSMA_SEED_1 read\n");
					return -EIO;
				}
				val &= ~(AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK);
				err = write_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
				if (err) {
					printk("Error during AT86RF2XX_REG__CSMA_SEED_1 write\n");
					return -EIO;
				}
			}
			break;
		case AT86RF2XX_OPT_AUTOACK:
			err = read_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG___CSMA_SEED_1 read\n");
				return -EIO;
			}
			val |= AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK;
			err = write_reg(spi, spi_cfg, AT86RF2XX_REG__CSMA_SEED_1, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG__CSMA_SEED_1 write\n");
				return -EIO;
			}
			break;
		case AT86RF2XX_OPT_TELL_RX_START:
			err = read_reg(spi, spi_cfg, AT86RF2XX_REG__IRQ_MASK, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG__IRQ_MASK read\n");
				return -EIO;
			}
			val &= ~AT86RF2XX_IRQ_STATUS_MASK__RX_START;
			err = write_reg(spi, spi_cfg, AT86RF2XX_REG__IRQ_MASK, &val, 1);
			if (err) {
				printk("Error during AT86RF2XX_REG__IRQ_MASK write\n");
				return -EIO;
			}
			break;
		default:
			/* do nothing */
			return -EIO;
		}
	}

	return 0;
}

static int at86rf2xx_enable_rx_safe_mode(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_CTRL_2, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_CTRL_2 write\n");
		return -EIO;
	}
	val |= AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE;

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_CTRL_2, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_CTRL_2 write\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_set_data_rate(struct device *spi, struct spi_config *spi_cfg,
					enum data_rate_type data_rate)
{
	uint8_t val;
	int err;

	if ((data_rate < TWO_FIVE_ZERO_KBPS) || (data_rate > TWO_MBPS)) {
		printk("data rate range over\n");
		return -EIO;
	}

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_CTRL_2, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_CTRL_2 write\n");
		return -EIO;
	}
	val |= data_rate;

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_CTRL_2, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_CTRL_2 write\n");
		return -EIO;
	}

	return 0;
}

static enum data_rate_type at86rf2xx_get_data_rate(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_CTRL_2, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_CTRL_2 write\n");
		return -EIO;
	}

	val &= AT86RF2XX_TRX_CTRL_2_MASK__OQPSK_DATA_RATE;

	return val;
}

static int at86rf2xx_disable_clock_output(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val;
	int err;

	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_CTRL_0, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_CTRL_0 read\n");
		return -EIO;
	}
	val &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL);
	val &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
	val |= (AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF);

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_CTRL_0, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_CTRL_0 write\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_enable_irq(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val = AT86RF2XX_IRQ_STATUS_MASK__TRX_END;
	int err;

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__IRQ_MASK, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__IRQ_MASK write\n");
		return -EIO;
	}

	/* clear interrupt flags */
	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__IRQ_STATUS, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__IRQ_STATUS read\n");
		return -EIO;
	}

	return 0;
}

static uint8_t at86rf2xx_get_status(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val;
	int err;

	/* if sleeping immediately return state */
	if (state == AT86RF2XX_STATE_SLEEP)
		return state;

	/* clear interrupt flags */
	err = read_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_STATUS, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_STATUS read\n");
		//return -EIO;
	}

	return val & AT86RF2XX_TRX_STATUS_MASK__TRX_STATUS;
}

static int at86rf2xx_reg_set_state(struct device *spi, struct spi_config *spi_cfg,
					uint8_t state_)
{
	int err;

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_STATE, &state_, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_STATE write\n");
		return -EIO;
	}

	while (at86rf2xx_get_status(spi, spi_cfg) != state_);
	state = state_;

	return 0;
}

static int at86rf2xx_reg_set_state_a(struct device *spi, struct spi_config *spi_cfg,
					uint8_t state_)
{
	int err;

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_STATE, &state_, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_STATE write\n");
		return -EIO;
	}

	return 0;
}

static int at86rf2xx_assert_awake(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val;
	int err;

	if (at86rf2xx_get_status(spi, spi_cfg) == AT86RF2XX_STATE_SLEEP) {
		gpio_pin_write(sleep, DT_ATMEL_AT86RF2XX_0_SLEEP_GPIOS_PIN, 0);
		k_sleep(AT86RF2XX_WAKEUP_DELAY);

		err = read_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_STATUS, &val, 1);
		if (err) {
			printk("Error during AT86RF2XX_REG__TRX_STATUS read\n");
			return -EIO;
		}
		state = val & AT86RF2XX_TRX_STATUS_MASK__TRX_STATUS;
	}

	return 0;
}

static int at86rf2xx_force_trx_off(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val = AT86RF2XX_TRX_STATE__FORCE_TRX_OFF;
	int err;

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_STATE, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_STATE write\n");
		return -EIO;
	}

	while (at86rf2xx_get_status(spi, spi_cfg) != AT86RF2XX_STATE_TRX_OFF);

	return 0;
}

static int at86rf2xx_set_state(struct device *spi, struct spi_config *spi_cfg,
					uint8_t state_)
{
	uint8_t old_state = at86rf2xx_get_status(spi, spi_cfg);
	uint8_t val;
	int err;

	if (state_ == old_state)
		return 0;

	while (old_state == AT86RF2XX_STATE_BUSY_RX_AACK ||
			old_state == AT86RF2XX_STATE_BUSY_TX_ARET ||
			old_state == AT86RF2XX_STATE_IN_PROGRESS)
		old_state = at86rf2xx_get_status(spi, spi_cfg);

	if ((old_state == AT86RF2XX_STATE_RX_AACK_ON &&
			state_ == AT86RF2XX_STATE_TX_ARET_ON) ||
			(old_state == AT86RF2XX_STATE_TX_ARET_ON &&
			state_ == AT86RF2XX_STATE_RX_AACK_ON))
		at86rf2xx_reg_set_state(spi, spi_cfg, AT86RF2XX_STATE_PLL_ON);
	else if (old_state == AT86RF2XX_STATE_SLEEP)
		at86rf2xx_assert_awake(spi, spi_cfg);

	if (state_ == AT86RF2XX_STATE_SLEEP) {
		at86rf2xx_force_trx_off(spi, spi_cfg);

		err = read_reg(spi, spi_cfg, AT86RF2XX_REG__IRQ_STATUS, &val, 1);
		if (err) {
			printk("Error during AT86RF2XX_REG__IRQ_STATUS read\n");
			return -EIO;
		}

		gpio_pin_write(sleep, DT_ATMEL_AT86RF2XX_0_SLEEP_GPIOS_PIN, 1);
		state = state_;
	} else {
		at86rf2xx_reg_set_state(spi, spi_cfg, state_);
	}

	return 0;
}

static int at86rf2xx_tx_prepare(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t state;

	do {
		state = at86rf2xx_get_status(spi, spi_cfg);
	}
	while (state == AT86RF2XX_STATE_BUSY_TX_ARET);

	if(state == AT86RF2XX_STATE_BUSY_RX_AACK) {
		at86rf2xx_force_trx_off(spi, spi_cfg);
		idle_state = AT86RF2XX_STATE_RX_AACK_ON;
	} else if (state != AT86RF2XX_STATE_TX_ARET_ON) {
		idle_state = state;
	}

	at86rf2xx_set_state(spi, spi_cfg, AT86RF2XX_STATE_TX_ARET_ON);
	frame_len = 2;

	return 0;
}

static size_t at86rf2xx_tx_load(struct device *spi, struct spi_config *spi_cfg,
					uint8_t addr, uint8_t *data, size_t len)
{
	frame_len += (uint8_t)len;

	at86rf2xx_write_sram(spi, spi_cfg, addr+1, data, len);

	return addr + len;
}

static int at86rf2xx_tx_exec(struct device *spi, struct spi_config *spi_cfg)
{
	uint8_t val = AT86RF2XX_TRX_STATE__TX_START;
	int err;

	at86rf2xx_write_sram(spi, spi_cfg, 0, &(frame_len), 1);

	err = write_reg(spi, spi_cfg, AT86RF2XX_REG__TRX_STATE, &val, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__TRX_STATE write\n");
		return -EIO;
	}

	return 0;
}

static size_t at86rf2xx_send(struct device *spi, struct spi_config *spi_cfg,
					uint8_t *data, size_t len)
{
	if (len > AT86RF2XX_MAX_PKT_LENGTH) {
		printk("[at86rf2xx] Error: Data to send exceeds max packet size.\n");
		return 0;
	}

	at86rf2xx_tx_prepare(spi, spi_cfg);
	at86rf2xx_tx_load(spi, spi_cfg, 0x00, data, len);
	at86rf2xx_tx_exec(spi, spi_cfg);

	return len;
}

void irq_handler(struct device *gpioc, struct gpio_callback *cb, uint32_t pins)
{
	k_work_submit(&data_read_work);
}

size_t at86rf2xx_rx_len()
{
	uint8_t phr;

	at86rf2xx_read_sram(spi, &spi_cfg, 0x00, &phr, 1);

	return (size_t)((phr & 0x7f) - 2);
}

void at86rf2xx_receive_data(void)
{
	size_t pkt_len;
	uint8_t sram_data[128];
	int i;

	pkt_len = at86rf2xx_rx_len();
	printf("read %dbyte\n", pkt_len);

	at86rf2xx_read_sram(spi, &spi_cfg, 0x01, sram_data, pkt_len);
	printk("data[%d]: ", pkt_len);
	for (i = 0; i < pkt_len; i++)
		printf("0x%x ", sram_data[i]);
	printk("\n\n");
}

static void data_read_work_handler(struct k_work *work)
{
	uint8_t state, irq_mask;
	int err;

	state = at86rf2xx_get_status(spi, &spi_cfg);
	if (state == AT86RF2XX_STATE_SLEEP)
		return;

	err = read_reg(spi, &spi_cfg, AT86RF2XX_REG__IRQ_STATUS, &irq_mask, 1);
	if (err) {
		printk("Error during AT86RF2XX_REG__IRQ_STATUS write\n");
		return;
	}

	if (irq_mask & AT86RF2XX_IRQ_STATUS_MASK__RX_START)
		printk("[at86rf2xx] EVT - RX_START\n");

	if (irq_mask & AT86RF2XX_IRQ_STATUS_MASK__TRX_END) {
		if(state == AT86RF2XX_STATE_RX_AACK_ON || state == AT86RF2XX_STATE_BUSY_RX_AACK) {
			printk("[at86rf2xx] EVT - RX_END\n");
			at86rf2xx_receive_data();
		}
	}
}


void main(void)
{
	struct device *reset;
	struct device *irq;
	struct gpio_callback irq_cb;
	struct spi_cs_control spi_cs;
	uint64_t ieee_addr;
	uint16_t short_addr, pan_id, channel;
	uint8_t sfd, data_rate, tx_power, tx_test_data[23];
	int err;

	printk("AT86RF2XX(2.4GHz Transceiver) example application\n\n");

	spi = device_get_binding(DT_ATMEL_AT86RF2XX_0_BUS_NAME);
	if (!spi) {
		printk("Could not find AT86RF2XX driver\n");
		return;
	}

	spi_cfg.operation = SPI_WORD_SET(8);
	spi_cfg.frequency = DT_ATMEL_AT86RF2XX_0_SPI_MAX_FREQUENCY;

	spi_cs.gpio_dev = device_get_binding(DT_ATMEL_AT86RF2XX_0_CS_GPIO_CONTROLLER);
	spi_cs.gpio_pin = DT_ATMEL_AT86RF2XX_0_CS_GPIO_PIN;
	spi_cs.delay = 0;
	spi_cfg.cs = &spi_cs;

#ifdef DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_CONTROLLER
	reset = device_get_binding(DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_CONTROLLER);
	if (reset == NULL) {
		printk("Failed to get pointer to %s device!",
			    DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_CONTROLLER);
		return;
	}

 	gpio_pin_configure(reset, DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_PIN, GPIO_DIR_OUT);
	gpio_pin_write(reset, DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_PIN, 0);
	k_sleep(AT86RF2XX_RESET_PULSE_WIDTH);
	gpio_pin_write(reset, DT_ATMEL_AT86RF2XX_0_RESET_GPIOS_PIN, 1);
	k_sleep(AT86RF2XX_RESET_DELAY);
#endif

	sleep = device_get_binding(DT_ATMEL_AT86RF2XX_0_SLEEP_GPIOS_CONTROLLER);
	if (sleep == NULL) {
		printk("Failed to get pointer to %s device!",
			    DT_ATMEL_AT86RF2XX_0_SLEEP_GPIOS_CONTROLLER);
		return;
	}
	gpio_pin_configure(sleep, DT_ATMEL_AT86RF2XX_0_SLEEP_GPIOS_PIN, GPIO_DIR_OUT);

	irq = device_get_binding(DT_ATMEL_AT86RF2XX_0_IRQ_GPIOS_CONTROLLER);
	if (irq == NULL) {
		printk("Failed to get pointer to %s device!",
			    DT_ATMEL_AT86RF2XX_0_IRQ_GPIOS_CONTROLLER);
		return;
	}
	gpio_pin_configure(irq, DT_ATMEL_AT86RF2XX_0_IRQ_GPIOS_PIN, IRQ_FLAGS);
	gpio_init_callback(&irq_cb, irq_handler, BIT(DT_ATMEL_AT86RF2XX_0_IRQ_GPIOS_PIN));
	gpio_add_callback(irq, &irq_cb);
	gpio_pin_enable_callback(irq, DT_ATMEL_AT86RF2XX_0_IRQ_GPIOS_PIN);

	err = at86rf2xx_assert_awake(spi, &spi_cfg);
	if (err) {
		printk("Error during at86rf2xx_assert_awake()\n");
		return;
	}


	/* Part Number */
	err = at86rf2xx_part_number(spi, &spi_cfg);
	if (err) {
		printk("Could not verify AT86RF2XX part number\n");
		return;
	}

	/* Version Number */
	err = at86rf2xx_version_number(spi, &spi_cfg);
	if (err) {
		printk("Could not verify AT86RF2XX version number\n");
		return;
	}

	/* Manufacturer ID */
	err = at86rf2xx_man_id(spi, &spi_cfg);
	if (err) {
		printk("Could not verify AT86RF2XX main id\n");
		return;
	}

#ifdef IEEE802154_USE
	/* SHORT ADDR */
	err = at86rf2xx_set_short_addr(spi, &spi_cfg, AT86RF2XX_DEFAULT_SHORT_ADDR);
	if (err) {
		printk("Could not write short_addr:0x%x\n", AT86RF2XX_DEFAULT_SHORT_ADDR);
		return;
	}

	short_addr = at86rf2xx_get_short_addr(spi, &spi_cfg);
	if (short_addr < 0) {
		printk("Could not read short addr\n");
		return;
	} else {
		printk("SHORT ADDR:0x%x\n", short_addr);
	}

	/* PAN ID */
	err = at86rf2xx_set_pan_id(spi, &spi_cfg, AT86RF2XX_DEFAULT_PAN_ID);
	if (err) {
		printk("Could not write pan id:0x%x\n", AT86RF2XX_DEFAULT_PAN_ID);
		return;
	}

	pan_id = at86rf2xx_get_pan_id(spi, &spi_cfg);
	if (pan_id < 0) {
		printk("Could not read pan id\n");
		return;
	} else {
		printk("PAN ID:0x%x\n", pan_id);
	}

	/* IEEE ADDR */
	err = at86rf2xx_set_ieee_addr(spi, &spi_cfg, AT86RF2XX_DEFAULT_IEEE_ADDR);
	if (err) {
		printk("Could not write ieee addr:0x%llx\n", AT86RF2XX_DEFAULT_IEEE_ADDR);
		return;
	}

	ieee_addr = at86rf2xx_get_ieee_addr(spi, &spi_cfg);
	if (ieee_addr < 0) {
		printk("Could not read ieee addr\n");
		return;
	} else {
		printk("IEEE ADDR:0x%llx\n", ieee_addr);
	}

	/* Channel */
	err = at86rf2xx_set_channel(spi, &spi_cfg, AT86RF2XX_DEFAULT_CHANNEL);
	if (err) {
		printk("Could not write channel:0x%x\n", AT86RF2XX_DEFAULT_CHANNEL);
		return;
	}

	channel = at86rf2xx_get_channel(spi, &spi_cfg);
	if (channel < 0) {
		printk("Could not read channel\n");
		return;
	} else {
		printk("channel:%d\n", channel);
	}

	/* IEEE 802.15.4 Option */
	options = 0;
	err = at86rf2xx_set_option(spi, &spi_cfg, AT86RF2XX_OPT_PROMISCUOUS, true);
	if (err) {
		printk("Could not set option: AT86RF2XX_OPT_PROMISCUOUS");
		return;
	}

	err = at86rf2xx_set_option(spi, &spi_cfg, AT86RF2XX_OPT_AUTOACK, true);
	if (err) {
		printk("Could not set option: AT86RF2XX_OPT_AUTOACK");
		return;
	}

	err = at86rf2xx_set_option(spi, &spi_cfg, AT86RF2XX_OPT_CSMA, true);
	if (err) {
		printk("Could not set option: AT86RF2XX_OPT_CSMA");
		return;
	}

	err = at86rf2xx_set_option(spi, &spi_cfg, AT86RF2XX_OPT_TELL_RX_START, true);
	if (err) {
		printk("Could not set option: AT86RF2XX_OPT_TELL_RX_START");
		return;
	}

	/* Not yet supported. */
	/*
	err = at86rf2xx_set_option(spi, &spi_cfg, AT86RF2XX_OPT_TELL_RX_END, true);
	if (err) {
		printk("Could not set option: AT86RF2XX_OPT_TELL_RX_END");
		return;
	}
	*/
#else
	/* SFD(Start-Of-Frame Delimiter) */
	err = at86rf2xx_set_sfd(spi, &spi_cfg, 0x12);
	if (err) {
		printk("Error during at86rf2xx_set_sfd()\n");
		return;
	}

	sfd = at86rf2xx_get_sfd(spi, &spi_cfg);
	if (err) {
		printk("Error during at86rf2xx_get_sfd()\n");
		return;
	} else {
		printk("sfd: 0x%x\n", sfd);
	}

	/* Extended Operating Mode Option */
	options = 0;
	err = at86rf2xx_set_option(spi, &spi_cfg, AT86RF2XX_OPT_PROMISCUOUS, true);
	if (err) {
		printk("Could not set option: AT86RF2XX_OPT_PROMISCUOUS");
		return;
	}
#endif

	/* Rx Safe Mode */
	err = at86rf2xx_enable_rx_safe_mode(spi, &spi_cfg);
	if (err) {
		printk("Error during at86rf2xx_enable_rx_safe_mode()\n");
		return;
	}

	/* Data Rate */
	err = at86rf2xx_set_data_rate(spi, &spi_cfg, TWO_MBPS);
	if (err) {
		printk("Error during at86rf2xx_set_data_rate()\n");
		return;
	}

	data_rate = at86rf2xx_get_data_rate(spi, &spi_cfg);
	if (data_rate < 0) {
		printk("Error during at86rf2xx_get_data_rate()\n");
		return;
	} else {
		switch (data_rate) {
		case TWO_FIVE_ZERO_KBPS:
			printf("DATA RATE: 250kb/s\n");
			break;
		case FIVE_ZERO_ZERO_KBPS:
			printf("DATA RATE: 500kb/s\n");
			break;
		case ONE_MBPS:
			printf("DATA RATE: 1000kb/s\n");
			break;
		case TWO_MBPS:
			printf("DATA RATE: 2000kb/s\n");
			break;
		default:
			break;
		}
	}

	/* Tx Power */
	err = at86rf2xx_set_tx_power(spi, &spi_cfg, 0);
	if (err) {
		printk("Error during at86rf2xx_set_tx_power()\n");
		return;
	}

	tx_power = at86rf2xx_get_tx_power(spi, &spi_cfg);
	if (tx_power == -128) {
		printk("Error during at86rf2xx_get_tx_power()\n");
		return;
	} else {
		printk("tx_power: %d[dBm]\n", tx_power);
	}

	/* disable clock output to save power */
	err = at86rf2xx_disable_clock_output(spi, &spi_cfg);
	if (err) {
		printk("Error during at86rf2xx_disable_clock_output()\n");
		return;
	}

	/* enable interrupts */
	err = at86rf2xx_enable_irq(spi, &spi_cfg);
	if (err) {
		printk("Error during at86rf2xx_enable_irq()\n");
		return;
	}

	/* XXX: I do not know why I need it. But without it, it does not work. */
	at86rf2xx_reg_set_state_a(spi, &spi_cfg, 0x03);

	/* go into RX state */
	err = at86rf2xx_set_state(spi, &spi_cfg, AT86RF2XX_STATE_RX_AACK_ON);
	if (err) {
		printk("Error during at86rf2xx_set_state()\n");
		return;
	}

	/* Data read workqueue */
	k_work_init(&data_read_work, data_read_work_handler);

	tx_test_data[0] = 0x12;
	tx_test_data[1] = 0x34;
	tx_test_data[2] = 0x56;
	tx_test_data[3] = 0x78;
	tx_test_data[4] = 0x9A;
	tx_test_data[5] = 0xBC;
	tx_test_data[6] = 0xDE;
	tx_test_data[7] = 0xFF;
	tx_test_data[8] = 0x11;
	tx_test_data[9] = 0x22;
	tx_test_data[10] = 0x33;
	tx_test_data[11] = 0x44;
	tx_test_data[12] = 0x55;
	tx_test_data[13] = 0x66;
	tx_test_data[14] = 0x77;
	tx_test_data[15] = 0x88;
	tx_test_data[16] = 0x99;
	tx_test_data[17] = 0xAA;
	tx_test_data[18] = 0xBB;
	tx_test_data[19] = 0xCC;
	tx_test_data[20] = 0xDD;
	tx_test_data[21] = 0xEE;
	tx_test_data[22] = 0xFF;


	printk("\nsetting end\n");

	while (1) {
#ifdef WRITE
		/* Transfer data at 1 second intervals */
		at86rf2xx_send(spi, &spi_cfg, tx_test_data, sizeof(tx_test_data));
#endif
		k_sleep(1000);
	}

	printk("application end\n");

	return;
}
