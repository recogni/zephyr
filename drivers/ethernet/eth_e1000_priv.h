/*
 * Copyright (c) 2018 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ETH_E1000_PRIV_H
#define ETH_E1000_PRIV_H

#ifdef __cplusplus
extern "C" {
#endif

#define CTRL_SLU	(1 << 6) /* Set Link Up */

#define TCTL_EN		(1 << 1)
#define RCTL_EN		(1 << 1)

#define ICR_TXDW	     (1) /* Transmit Descriptor Written Back */
#define ICR_TXQE	(1 << 1) /* Transmit Queue Empty */
#define ICR_RXO		(1 << 6) /* Receiver Overrun */

#define IMS_RXO		(1 << 6) /* Receiver FIFO Overrun */

#define RCTL_MPE	(1 << 4) /* Multicast Promiscuous Enabled */

#define TDESC_EOP	     (1) /* End Of Packet */
#define TDESC_RS	(1 << 3) /* Report Status */

#define RDESC_STA_DD	     (1) /* Descriptor Done */
#define TDESC_STA_DD	     (1) /* Descriptor Done */

#define ETH_ALEN 6	/* TODO: Add a global reusable definition in OS */

enum e1000_reg_t {
	CTRL	= 0x0000,	/* Device Control */
	ICR	= 0x1500,	/* Interrupt Cause Read */
	ICS	= 0x1504,	/* Interrupt Cause Set */
	IMS	= 0x1508,	/* Interrupt Mask Set */
	RCTL	= 0x0100,	/* Receive Control */
	TCTL	= 0x0400,	/* Transmit Control */
	RDBAL	= 0xC000,	/* Rx Descriptor Base Address Low */
	RDBAH	= 0xC004,	/* Rx Descriptor Base Address High */
	RDLEN	= 0xC008,	/* Rx Descriptor Length */
	RDH	= 0xC010,	/* Rx Descriptor Head */
	RDT	= 0xC018,	/* Rx Descriptor Tail */
	TDBAL	= 0xE000,	/* Tx Descriptor Base Address Low */
	TDBAH	= 0xE004,	/* Tx Descriptor Base Address High */
	TDLEN	= 0xE008,	/* Tx Descriptor Length */
	TDH	= 0xE010,	/* Tx Descriptor Head */
	TDT	= 0xE018,	/* Tx Descriptor Tail */
	RAL	= 0x5400,	/* Receive Address Low */
	RAH	= 0x5404,	/* Receive Address High */
	MCR	= 0x0052,	/* Receive Address High */
};

/* Legacy TX Descriptor */
struct e1000_tx {
	uint64_t addr;
	uint16_t len;
	uint8_t  cso;
	uint8_t  cmd;
	uint8_t  sta;
	uint8_t  css;
	uint16_t special;
};

/* Legacy RX Descriptor */
struct e1000_rx {
	uint64_t addr;
	uint16_t len;
	uint16_t csum;
	uint8_t  sta;
	uint8_t  err;
	uint16_t special;
};

struct e1000_dev {
//	volatile struct e1000_tx tx __aligned(16);
//	volatile struct e1000_rx rx __aligned(16);
	struct e1000_tx *ptx;
	struct e1000_rx *prx;

	mm_reg_t address;
	/* If VLAN is enabled, there can be multiple VLAN interfaces related to
	 * this physical device. In that case, this iface pointer value is not
	 * really used for anything.
	 */
	struct net_if *iface;
	uint8_t mac[ETH_ALEN];
	uint8_t *txb;
	uint8_t *rxb;
};

static const char *e1000_reg_to_string(enum e1000_reg_t r)
	__attribute__((unused));


/*
#define iow32(_dev, _reg, _val) do {					\
	LOG_DBG("iow32 %s 0x%08lx", e1000_reg_to_string(_reg), (_val)); 	\
	sys_write32(_val, (_dev)->address + (_reg));			\
} while (0)
*/
#define iow32(_dev, _reg, _val) do {					\
	sys_write32(_val, (_dev)->address + (_reg));			\
} while (0)


/*
#define ior32(_dev, _reg)						\
({									\
	uint32_t val = sys_read32((_dev)->address + (_reg));		\
	LOG_DBG("ior32 %s 0x%08lx", e1000_reg_to_string(_reg), val); 	\
	val;								\
})

*/

#define ior32(_dev, _reg)						\
({									\
	uint32_t val = sys_read32((_dev)->address + (_reg));		\
	val;								\
})


#ifdef __cplusplus
}
#endif

#endif /* ETH_E1000_PRIV_H_ */
