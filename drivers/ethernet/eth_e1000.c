/*
 * Copyright (c) 2018-2019 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_e1000

#define LOG_MODULE_NAME eth_e1000
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <sys/types.h>
#include <zephyr.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>
#include <drivers/pcie/pcie.h>
#include <scorpio_noc.h>

#include "eth_e1000_priv.h"

extern void plda_pcie_isr(const struct device *ddev);

void *pAmem = ((void*) (SCORPIO_NOC_AMEM_DMA_ADDRESS
		| SCORPIO_SCPU_CLUSTER_NOC_ADDRESS));
struct k_heap h;

#if defined(CONFIG_ETH_E1000_VERBOSE_DEBUG)
#define hexdump(_buf, _len, fmt, args...)				\
({									\
	const size_t STR_SIZE = 80;					\
	char _str[STR_SIZE];						\
									\
	snprintk(_str, STR_SIZE, "%s: " fmt, __func__, ## args);	\
									\
	LOG_HEXDUMP_DBG(_buf, _len, log_strdup(_str));			\
})
#else
#define hexdump(args...)
#endif

static const char* e1000_reg_to_string(enum e1000_reg_t r) {
#define _(_x)	case _x: return #_x
	switch (r) {
	_(CTRL)
;		_(ICR);
		_(ICS);
		_(ITR);
		_(IMS);
		_(RCTL);
		_(TCTL);
		_(RDBAL);
		_(RDBAH);
		_(RDLEN);
		_(RDH);
		_(RDT);
		_(RADV);
		_(TDBAL);
		_(TDBAH);
		_(TDLEN);
		_(TDH);
		_(TDT);
		_(RAL);
		_(RAH);
		_(TXDCTL);
	}
#undef _
	LOG_ERR("Unsupported register: 0x%x", r);
	k_oops();
	return NULL;
}

static struct net_if* get_iface(struct e1000_dev *ctx, uint16_t vlan_tag) {
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

static enum ethernet_hw_caps e1000_caps(const struct device *dev) {
	return
#if IS_ENABLED(CONFIG_NET_VLAN)
		ETHERNET_HW_VLAN |
#endif
	ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T | ETHERNET_LINK_1000BASE_T;
}

static int e1000_tx(struct e1000_dev *dev, void *buf, size_t len) {
	hexdump(buf, len, "%zu byte(s)", len);

	memcpy(dev->txb, buf, len);
	uint8_t tdt = dev->tdt;

	dev->ptx[tdt].addr = POINTER_TO_INT(dev->txb) & 0xFFFFFFFFFF;
	dev->ptx[tdt].len = len;
	dev->ptx[tdt].cmd = TDESC_EOP | TDESC_RS | TDESC_IFCS;
	dev->ptx[tdt].sta = 0;
	dev->tdt = (dev->tdt + 1) % dev->tdlen;

	iow32(dev, TDT, dev->tdt);

	while (!(dev->ptx[tdt].sta)) {
		k_yield();
	}

	LOG_DBG("tx.sta: 0x%02hx", dev->ptx[tdt].sta);

	if (dev->ptx[tdt].sta & TDESC_STA_DD) {
		return 0;
	}

	return -EIO;
}

static int e1000_send(const struct device *ddev, struct net_pkt *pkt) {
	struct e1000_dev *dev = ddev->data;
	size_t len = net_pkt_get_len(pkt);

	if (net_pkt_read(pkt, dev->scpu_txb, len)) {
		return -EIO;
	}

	return e1000_tx(dev, dev->scpu_txb, len);
}

static struct net_pkt* e1000_rx(struct e1000_dev *dev) {
	struct net_pkt *pkt = NULL;
	void *buf;
	ssize_t len;

	LOG_DBG("rx.sta: 0x%02hx", dev->prx[dev->rdh].sta);

	if (!(dev->prx[dev->rdh].sta & RDESC_STA_DD)) {
		LOG_ERR("RX descriptor not ready");
		goto out;
	}

	memcpy(dev->scpu_rxb, dev->rxb, dev->prx[dev->rdh].len);
	buf = INT_TO_POINTER(dev->scpu_rxb);
	len = dev->prx[dev->rdh].len - 4;

	if (len <= 0) {
		LOG_ERR("Invalid RX descriptor length: %hu", dev->prx[dev->rdh].len);
		goto out;
	}

	hexdump(buf, len, "%zd byte(s)", len);

	pkt = net_pkt_rx_alloc_with_buffer(dev->iface, len, AF_UNSPEC, 0,
	K_NO_WAIT);
	if (!pkt) {
		LOG_ERR("Out of buffers");
	}

	if (net_pkt_write(pkt, buf, len)) {
		LOG_ERR("Out of memory for received frame");
		net_pkt_unref(pkt);
		pkt = NULL;
	}

	out: return pkt;
}

void e1000_isr(struct e1000_dev *dev) {
	uint32_t icr = ior32(dev, ICR); /* Cleared upon read */
	uint16_t vlan_tag = NET_VLAN_TAG_UNSPEC;

	icr &= ~(ICR_TXDW | ICR_TXQE);

	if (icr & ICR_RXO || icr & ICR_RXT0) {
		struct net_pkt *pkt = e1000_rx(dev);

		icr &= ~(ICR_RXT0 | ICR_RXO);

		if (pkt) {
#if defined(CONFIG_NET_VLAN)
			struct net_eth_hdr *hdr = NET_ETH_HDR(pkt);

			if (ntohs(hdr->type) == NET_ETH_PTYPE_VLAN) {
				struct net_eth_vlan_hdr *hdr_vlan =
					(struct net_eth_vlan_hdr *)
					NET_ETH_HDR(pkt);

				net_pkt_set_vlan_tci(
					pkt, ntohs(hdr_vlan->vlan.tci));
				vlan_tag = net_pkt_vlan_tag(pkt);

#if CONFIG_NET_TC_RX_COUNT > 1
				enum net_priority prio;

				prio = net_vlan2priority(
						net_pkt_vlan_priority(pkt));
				net_pkt_set_priority(pkt, prio);
#endif
			}
#endif /* CONFIG_NET_VLAN */

			net_recv_data(get_iface(dev, vlan_tag), pkt);
			dev->rdh = (dev->rdh + 1) % dev->rdlen;
			dev->prx[dev->rdt].addr = POINTER_TO_INT(dev->rxb) & 0xFFFFFFFFFF;
			dev->prx[dev->rdt].sta = 0;
			dev->prx[dev->rdt].len = 2048;
			dev->rdt = (dev->rdt + 1) % dev->rdlen;
			iow32(dev, RDT, dev->rdt);
		} else {
			eth_stats_update_errors_rx(get_iface(dev, vlan_tag));
		}
	}

	if (icr) {
		LOG_ERR("Unhandled interrupt, ICR: 0x%x", icr);
	}

}

#define PCI_VENDOR_ID_INTEL	0x8086
#define PCI_DEVICE_ID_I82540EM	0x100e

int e1000_probe(const struct device *ddev) {
	const pcie_bdf_t bdf = 0x100;
	struct e1000_dev *dev = ddev->data;
	uint32_t ral, rah;
	struct pcie_mbar mbar;

	pcie_probe(0x100, PCIE_ID(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_I82540EM));

	pcie_get_mbar(bdf, 0, &mbar);
	pcie_set_cmd(bdf, PCIE_CONF_CMDSTAT_MEM |
	PCIE_CONF_CMDSTAT_MASTER, true);

	mbar.phys_addr = 0x10858000000;
	mbar.size = 0x20000;
	device_map(&dev->address, mbar.phys_addr, mbar.size,
	K_MEM_CACHE_NONE);

	/* Setup TX descriptor */

	dev->rdlen = dev->tdlen = 8;
	dev->rdh = 0;
	dev->rdt = (dev->rdh + 1) % dev->rdlen;
	dev->tdh = 0;
	dev->tdt = 0;

	k_heap_init(&h, pAmem, 65536);
	dev->txb = k_heap_aligned_alloc(&h, 512, 2048, K_NO_WAIT);
	dev->rxb = k_heap_aligned_alloc(&h, 512, 2048, K_NO_WAIT);

	/*	dev->rxb = 0x8D0010C00; if multi translation tables used */
	dev->ptx = (struct e1000_tx*) k_heap_aligned_alloc(&h, 128,
			(dev->tdlen * 16),
			K_NO_WAIT);
	dev->prx = (struct e1000_rx*) k_heap_aligned_alloc(&h, 128,
			(dev->rdlen * 16),
			K_NO_WAIT);

	for (int i = 0; i < dev->rdlen; i++) {
		dev->prx[i].addr = POINTER_TO_INT(dev->rxb) & 0xFFFFFFFFFF;
		dev->prx[i].len = 2048;
		dev->prx[i].sta = 0;

	}

	for (int i = 0; i < dev->tdlen; i++) {
		dev->ptx[i].addr = POINTER_TO_INT(dev->txb) & 0xFFFFFFFFFF;
		dev->ptx[i].len = 2048;
		dev->ptx[i].sta = 0;
		dev->ptx[i].cmd = TDESC_EOP | TDESC_RS | TDESC_IFCS;

	}

	iow32(dev, TDBAL, 0xFFFFFFFF & ((uint64_t )dev->ptx));
	iow32(dev, TDBAH, (0xFF & ((uint64_t )(dev->ptx) >> 32)));
	iow32(dev, TDLEN, (dev->tdlen * 16));

	iow32(dev, TCTL, 0); // disable Tx
	iow32(dev, TDH, dev->tdh);
	iow32(dev, TDT, dev->tdt);

	iow32(dev, ITR, 0x1); // ITR minimum value
	iow32(dev, RADV, 0x1); // RADV minimum value

	iow32(dev, TCTL, TCTL_EN | 0x8); // PSP field for padding
	ral = ior32(dev, TXDCTL);
	iow32(dev, TXDCTL, ral);

	iow32(dev, RDBAL, 0xFFFFFFFF & ((uint64_t )dev->prx));
	iow32(dev, RDBAH, (0xFF & ((uint64_t )(dev->prx) >> 32)));
	iow32(dev, RDLEN, (dev->rdlen * 16));

	iow32(dev, RCTL, 0);
	iow32(dev, RDH, dev->rdh);
	iow32(dev, RDT, dev->rdt);

	iow32(dev, IMS, IMS_RXO | IMS_RXT0);

	ral = ior32(dev, RAL);
	rah = ior32(dev, RAH);

	memcpy(dev->mac, &ral, 4);
	memcpy(dev->mac + 4, &rah, 2);

	return 0;
}

static void e1000_iface_init(struct net_if *iface) {
	struct e1000_dev *dev = net_if_get_device(iface)->data;

	/* For VLAN, this value is only used to get the correct L2 driver.
	 * The iface pointer in device context should contain the main
	 * interface if the VLANs are enabled.
	 */
	if (dev->iface == NULL) {
		dev->iface = iface;

		/* Do the phy link up only once */
//       IRQ_CONNECT(DT_INST_IRQN(0),
//                       7,
//					   plda_pcie_isr, NULL,
//                       0);
//
//		irq_enable(DT_INST_IRQN(0));
		iow32(dev, CTRL, CTRL_SLU); /* Set link up */
		iow32(dev, RCTL, RCTL_EN | RCTL_MPE);

	}

	ethernet_init(iface);

	net_if_set_link_addr(iface, dev->mac, sizeof(dev->mac), NET_LINK_ETHERNET);

	LOG_DBG("done");
}

struct e1000_dev e1000_dev;

static const struct ethernet_api e1000_api = { .iface_api.init =
		e1000_iface_init, .get_capabilities = e1000_caps, .send = e1000_send, };

ETH_NET_DEVICE_DT_INST_DEFINE(0, e1000_probe, device_pm_control_nop, &e1000_dev,
		NULL, CONFIG_ETH_INIT_PRIORITY, &e1000_api, NET_ETH_MTU);
