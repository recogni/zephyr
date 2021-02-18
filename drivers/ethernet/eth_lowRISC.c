
/*
 * Copyright (c) Recogni.
 *
 * LowRISC Ethernet driver
 */


#define DT_DRV_COMPAT lowrisc_eth

#define CONFIG_PTP_CLOCK_LOWRISC 1

#define LOG_MODULE_NAME eth_lowRISC
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);
#include <sys/types.h>
#include <zephyr.h>
#include <device.h>
#include <string.h>
#include <errno.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>
#include <assert.h>
#include "eth_lowRISC.h"
#include "mdiobb.h"
#include "rtl8211_phy.h"

#if defined(CONFIG_PTP_CLOCK_LOWRISC)
#include <ptp_clock.h>
#include <net/gptp.h>
#include <sys/util.h>
#endif

struct {
	int ptp;
	int rx_isr;
	int prio;
	int am_ptp;
} eth_stats;

static struct net_if *get_iface(struct net_local_lr *ctx, uint16_t vlan_tag);

#if defined(CONFIG_ETH_LOWRISC_VERBOSE_DEBUG)
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

static void inline eth_copyout(struct net_local_lr *priv, u8 *data, int len)
{
    int i, rnd = ((len - 1) | 7) + 1;
    volatile u64 *eth_base = (volatile u64 *)(priv->ioaddr);
    if (!(((size_t)data) & 7))
    {
        u64 *ptr = (u64 *)data;
        for (i = 0; i < rnd / 8; i++) eth_base[TXBUFF_OFFSET / 8 + i] = ptr[i];
    }
    else  // We can't unfortunately rely on the skb being word aligned
    {
        u64 notptr;
        for (i = 0; i < rnd / 8; i++)
        {
            memcpy(&notptr, data + (i << 3), sizeof(u64));
            eth_base[TXBUFF_OFFSET / 8 + i] = notptr;
        }
    }
}

#if defined(FROM_MCUX)
static bool eth_get_ptp_data(struct net_if *iface, struct net_pkt *pkt)
{
        int eth_hlen;
	if (ntohs(NET_ETH_HDR(pkt)->type) != NET_ETH_PTYPE_PTP) {
		return false;
	}
	eth_hlen = sizeof(struct net_eth_hdr);
        net_pkt_set_priority(pkt, NET_PRIORITY_CA);
        return true;
}
#endif

#if defined(CONFIG_PTP_CLOCK_LOWRISC)
static bool lr_need_timestamping(struct gptp_hdr *hdr)
{
        switch (hdr->message_type) {
        case GPTP_SYNC_MESSAGE:
        case GPTP_PATH_DELAY_RESP_MESSAGE:
                return true;
        default:
                return false;
        }
}

static struct gptp_hdr *lr_check_gptp_msg(struct net_pkt *pkt,
                                       bool is_tx)
{
        uint8_t *msg_start = net_pkt_data(pkt);
        struct gptp_hdr *gptp_hdr;
        int eth_hlen;

	struct net_eth_hdr *hdr;

	hdr = (struct net_eth_hdr *)msg_start;
	if (ntohs(hdr->type) != NET_ETH_PTYPE_PTP) {
		return NULL;
	}

	eth_hlen = sizeof(struct net_eth_hdr);

        /* In TX, the first net_buf contains the Ethernet header
         * and the actual gPTP header is in the second net_buf.
         * In RX, the Ethernet header + other headers are in the
         * first net_buf.
         */
        if (is_tx) {
                if (pkt->frags->frags == NULL) {
                        return false;
                }

                gptp_hdr = (struct gptp_hdr *)pkt->frags->frags->data;
        } else {
                gptp_hdr = (struct gptp_hdr *)(pkt->frags->data + eth_hlen);
        }

        return gptp_hdr;
}

static int lr_eth_clock_gettime(struct net_ptp_time *time)
{
#ifdef NOTYET
	struct timespec tp;
	int ret;

	ret = clock_gettime(CLOCK_MONOTONIC_RAW, &tp);
	if (ret < 0) {
		return -errno;
	}

	time->second = tp.tv_sec;
	time->nanosecond = tp.tv_nsec;
#endif

	return 0;
}

static void lr_update_pkt_priority(struct gptp_hdr *hdr, struct net_pkt *pkt)
{
        if (GPTP_IS_EVENT_MSG(hdr->message_type)) {
                net_pkt_set_priority(pkt, NET_PRIORITY_CA);
		eth_stats.prio++;
        } else {
                net_pkt_set_priority(pkt, NET_PRIORITY_IC);
        }
}

#ifdef NOTUSED
static void lr_update_gptp(struct net_if *iface, struct net_pkt *pkt,
                        bool send)
{
        struct net_ptp_time timestamp;
        struct gptp_hdr *hdr;
        int ret;

        ret = lr_eth_clock_gettime(&timestamp);
        if (ret < 0) {
                return;
        }

        net_pkt_set_timestamp(pkt, &timestamp);

        //hdr = lr_check_gptp_msg(iface, pkt, send);
        hdr = lr_check_gptp_msg(pkt, send);
        if (!hdr) {
                return;
        }

        if (send) {
                ret = lr_need_timestamping(hdr);
                if (ret) {
                        net_if_add_tx_timestamp(pkt);
                }
        } else {
                lr_update_pkt_priority(hdr, pkt);
        }
}
#endif /* UNUSED */



/* Retrieve timestamp (from hw if possible) and write timestamp into packet */
static /* inline */ void lr_timestamp_tx_pkt(struct gptp_hdr *hdr, struct net_pkt *pkt)
{
        struct net_ptp_time timestamp;
        //timestamp = get_current_ts(gmac);
        net_pkt_set_timestamp(pkt, &timestamp);
}

/* Retrieve time (from hw if possible) and write it into packet */
static inline void lr_timestamp_rx_pkt(struct gptp_hdr *hdr,
                                    struct net_pkt *pkt)
{
        struct net_ptp_time timestamp;

	/*
	 From drivers/kscan/kscan_mchp_xec.c
	 uint32_t stop_cycles = k_cycle_get_32();
	 cycles_spent =  stop_cycles - start_cycles;
         microsecs_spent = CLOCK_32K_HW_CYCLES_TO_US(cycles_spent);
	 ***/

        //timestamp = get_current_ts(gmac);
	
	//Call somekind of timeofday thing to get time.
	//Number ticks?
	//Brett

        net_pkt_set_timestamp(pkt, &timestamp);
}

#endif

static int lr_send(const struct device *dev, struct net_pkt *pkt)
{
   struct net_local_lr *priv  = dev->data;

   int rslt;
   size_t len;

//#if defined(CONFIG_PTP_CLOCK_LOWRISC)
        struct gptp_hdr *hdr;

	hdr = lr_check_gptp_msg(pkt, true);
        if (hdr && lr_need_timestamping(hdr)) {
		lr_timestamp_tx_pkt(hdr, pkt);
	}
	/***
        if (hdr && lr_need_timestamping(hdr)) {
                net_if_add_tx_timestamp(pkt);
        }
	****/

//#endif

   rslt = eth_read(priv, TPLR_OFFSET);
   len = net_pkt_get_len(pkt);

   if (net_pkt_read(pkt, priv->txb, len))
   {
      return -EIO;
   }

   if (rslt & TPLR_BUSY_MASK)
   {
	  printk("TX Busy Status = %x, len = %lu, ignoring\n", rslt, len);
   }

   eth_copyout(priv, priv->txb, len);
   eth_write(priv, TPLR_OFFSET, len);

   return 0;
}

static void inline eth_enable_irq(struct net_local_lr *priv)
{
    volatile u64 *eth_base = (volatile u64 *)(priv->ioaddr);

    /*
     * Hackery: Enable Promiscious mode for until we figure out how to recieve
     * multicast packets addressed to PTP Multicast Group Addr 01:1b:19:00:00:00
     */
    eth_base[MACHI_OFFSET >> 3] |= (MACHI_IRQ_EN | MACHI_ALLPKTS_MASK);  //PROMISC
    mmiowb();
}

static void inline eth_disable_irq(struct net_local_lr *priv)
{
    volatile u64 *eth_base = (volatile u64 *)(priv->ioaddr);
    eth_base[MACHI_OFFSET >> 3] &= ~MACHI_IRQ_EN;
    mmiowb();
}

/*
 *  Fetch the rx size of the packet sitting in the `first` buffer.
 */
int lr_eth_recv_size(struct net_local_lr *priv)
{
    volatile const int rsr = eth_read(priv, RSR_OFFSET);
    const int first = rsr & RSR_RECV_FIRST_MASK;
    //const int next = (rsr & RSR_RECV_NEXT_MASK) >> 4;

    // Check if there is Rx Data available ...
    if (rsr & RSR_RECV_DONE_MASK)
    {
        // Read the rx length for the buffer slot we are processing. There are
        // up-to 8 of these. TODO: Verify the << 3.
        const int rx_len = eth_read(priv, RPLR_OFFSET + ((first & 0x7) << 3));
        const int rlen = (rx_len & RPLR_LENGTH_MASK) - 4;  // Discard FCS bytes.
        return rlen;
    }
    return 0;
}

static void inline eth_copyin(struct net_local_lr *priv, u8 *data, int len,
                              int start)
{
    int i, rnd = ((len - 1) | 7) + 1;
    volatile u64 *eth_base = (volatile u64 *)(priv->ioaddr);
    if (!(((size_t)data) & 7))
    {
        u64 *ptr = (u64 *)data;
        for (i = 0; i < rnd / 8; i++) ptr[i] = eth_base[start + i];
    }
    else  // We can't unfortunately rely on the skb being word aligned
    {
        for (i = 0; i < rnd / 8; i++)
        {
            u64 notptr = eth_base[start + i];
            memcpy(data + (i << 3), &notptr, sizeof(u64));
        }
    }
}

/*
 *  Fetch `len` number of bytes from the current `first` buffer.
 *
 *  First, Last == pointers to indicate full or empty
 *  Next  == buffer slot to read data into for the device (only increments)
 *  Full (buffer slot)
 */
int lr_eth_recv(struct net_local_lr *priv, u8 *buf, const int len, struct net_pkt *pkt)
{
#if defined(CONFIG_PTP_CLOCK_LOWRISC)
        struct gptp_hdr *hdr;
#endif
	uint16_t vlan_tag = NET_VLAN_TAG_UNSPEC;

    assert(len > 0);

eth_stats.am_ptp = 0;
eth_stats.rx_isr++;
    const int rsr = eth_read(priv, RSR_OFFSET);
    const int first = rsr & RSR_RECV_FIRST_MASK;

    const int start = RXBUFF_OFFSET / 8 + ((first & 7) << 8);
    eth_copyin(priv, buf, len, start);


    /*
     *  Advance first buffer by 1 slot (that we consumed above).
     */
    eth_write(priv, LR_WR_FIRST_BUFFER_PTR, (first + 1) & 0xF);

#ifndef RELOCATED
	/* Relocated from ISR */
	if (net_pkt_write(pkt, priv->rxb, len)) {
        	LOG_ERR("Out of memory for received frame");
		net_pkt_unref(pkt);
		pkt = NULL;
		return -1;
	}

#if defined(CONFIG_PTP_CLOCK_LOWRISC)
	hdr = lr_check_gptp_msg(pkt, false);
	if (hdr) {
eth_stats.ptp++;
eth_stats.am_ptp = 1;
		lr_timestamp_rx_pkt(hdr, pkt);
		lr_update_pkt_priority(hdr, pkt);
	}
#endif /* CONFIG_PTP_CLOCK_LOWRISC */

	net_recv_data(get_iface(priv, vlan_tag), pkt);
#endif /* RELOCATED from ISR */

    return len;
}

static struct net_if *get_iface(struct net_local_lr *ctx, uint16_t vlan_tag)
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

void lr_eth_enable_irq(struct net_local_lr *priv) { eth_enable_irq(priv); }
void lr_eth_disable_irq(struct net_local_lr *priv) { eth_disable_irq(priv); }

/*
 *  Trigger an IRQ - this is done on driver startup and if the rx buffer is
 *  empty and we did not fetch any packets.
 */
void lr_eth_trigger_irq(struct net_local_lr *priv)
{
    int rsr = eth_read(priv, RSR_OFFSET);
    const int first = rsr & RSR_RECV_FIRST_MASK;
    eth_write(priv, LR_WR_FIRST_BUFFER_PTR, (first + 1) & 0xF);
}

static void lr_isr(struct device *dev)
{
   ssize_t n; // Bytes rx'd
   struct net_local_lr *priv = dev->data;
   struct net_pkt *pkt = NULL;

   lr_eth_disable_irq(priv);

   if ((n = lr_eth_recv_size(priv)) > 0)
   {

	pkt = net_pkt_rx_alloc_with_buffer(priv->iface, n, AF_UNSPEC, 0,
						   K_NO_WAIT);
	if (!pkt) {
	    LOG_ERR("Out of buffers");
	    goto out;
	}

        if (lr_eth_recv(priv, priv->rxb, n, pkt) < 0) {
	    goto out;
	}

#ifdef OLD
        hexdump(priv->rxb, n, "%zd byte(s)", n);

	if (net_pkt_write(pkt, priv->rxb, n)) {
        	LOG_ERR("Out of memory for received frame");
		net_pkt_unref(pkt);
		pkt = NULL;
		goto out;
	}
	net_recv_data(get_iface(priv, vlan_tag), pkt);
#endif
   }
   else
   {
       /*
        *  The read side was not full, re-enable IRQs.
        */
       lr_eth_trigger_irq(priv);
   }

out:
   // The ISR handler disables the ethernet IRQs, since we are done, turn
   // them back on to process the next potential packet.
   lr_eth_enable_irq(priv);

    return;
}

/*
 *  Update the device mac address. The input address is a array of 6 hex bytes.
 */
static void inline eth_update_address(struct net_local_lr *priv, u8 *mac)
{
    u32 macaddr_lo, macaddr_hi;
    u32 flags = 0;
    memcpy(&macaddr_lo, mac + 2, sizeof(u32));
    memcpy(&macaddr_hi, mac + 0, sizeof(u16));
    eth_write(priv, MACLO_OFFSET, htonl(macaddr_lo));
    eth_write(priv, MACHI_OFFSET, flags | htons(macaddr_hi));
}


int lr_probe(const struct device *dev)
{
   struct net_local_lr *priv = dev->data;

   uint8_t mac_addr[6] = DT_INST_PROP(0, local_mac_address);

   memcpy(priv->mac, mac_addr, sizeof(mac_addr));
   eth_update_address(priv, mac_addr);
   priv->ioaddr = (void *) 0x30000000;

   /*
    *  MDIO config
    */
   mdiobb_write(priv, 0, MII_BMCR,
                BMCR_RESET | BMCR_ANRESTART | BMCR_SPEED100);


   /*
    *  RX buffer starting condition:
    *      First : 0 (or `next` which is set to 0 on reset)
    *      Last  : The size of the rotational buffer (s/w set, static)
    *      Next  : (HW managed) should be 0
    *
    *  This will allow the hardware to realize that the buffer is empty and has
    *  8 slots available.
    *
    *  The interrupt is only fired when the `Next` buffer (hardware managed)
    *  does not match the `First` buffer (got a packet or two...).
    *
    *  Additionally, the condtition for getting a new packet (buffer has space)
    *  is gated by the check for buffer full which is true if
    *  `Next` == (`First` + `Last`) & 0xF.
    */
   int rsr = eth_read(priv, RSR_OFFSET);
   const int next = rsr & RSR_RECV_NEXT_MASK >> 4;
   assert(next == 0);
   const int first = 0;
   const int last = 8;
   eth_write(priv, LR_WR_FIRST_BUFFER_PTR, first);
   eth_write(priv, LR_WR_LAST_BUFFER_PTR, last);

   /*
    *  Enable IRQs for the eth driver. On the first go, we blindly bump the
    *  rx slot to get the interrupts going.
    */
   eth_enable_irq(priv);
   return 0;
}

static enum ethernet_hw_caps lr_caps(const struct device *dev)
{
   return ETHERNET_LINK_10BASE_T 
#if defined(CONFIG_PTP_CLOCK_LOWRISC)
	| ETHERNET_PTP 
#endif
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
        | ETHERNET_PROMISC_MODE
#endif

   	| ETHERNET_LINK_100BASE_T;
}


DEVICE_DT_INST_DECLARE(0);

static void lr_iface_init(struct net_if *iface)
{
   struct net_local_lr *dev = net_if_get_device(iface)->data;

   /* For VLAN, this value is only used to get the correct L2 driver.
    * The iface pointer in device context should contain the main
    * interface if the VLANs are enabled.
    */
    if (dev->iface == NULL) {
       dev->iface = iface;
       IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), lr_isr,  DEVICE_DT_INST_GET(0), DT_INST_IRQ(0, sense));
       irq_enable(DT_INST_IRQN(0));
       eth_enable_irq(dev);
    }

    ethernet_init(iface);
    net_if_set_link_addr(iface, dev->mac, sizeof(dev->mac), NET_LINK_ETHERNET);
    return;
}

#if defined(CONFIG_PTP_CLOCK)
static const struct device *lr_get_ptp_clock(const struct device *dev)
{
	struct net_local_lr *priv = dev->data;
        return priv->ptp_clock;
}
#endif

static const struct ethernet_api lr_api = {
        .iface_api.init         = lr_iface_init,
#if defined(CONFIG_PTP_CLOCK)
        .get_ptp_clock          = lr_get_ptp_clock,
#endif
        .get_capabilities       = lr_caps,
        .send                   = lr_send,
};

static struct net_local_lr lr_dev;


ETH_NET_DEVICE_DT_INST_DEFINE(0,
                    lr_probe,
                    device_pm_control_nop,
                    &lr_dev,
                    NULL,
                    CONFIG_ETH_INIT_PRIORITY,
                    &lr_api,
                    NET_ETH_MTU);


#define CONFIG_ETH_LOWRISC_INTERFACE_COUNT 1

#if defined(CONFIG_PTP_CLOCK_LOWRISC)
#if IS_ENABLED(CONFIG_NET_GPTP)
BUILD_ASSERT(								\
	CONFIG_ETH_LOWRISC_INTERFACE_COUNT == CONFIG_NET_GPTP_NUM_PORTS, \
	"Number of network interfaces must match gPTP port count");
#endif

struct ptp_context {
	const struct device *eth_dev;
};

//#define DEFINE_PTP_DEV_DATA(x, _) 
static struct ptp_context ptp_context_0;

static int ptp_clock_set_lowrisc(const struct device *clk,
				      struct net_ptp_time *tm)
{
	ARG_UNUSED(clk);
	ARG_UNUSED(tm);

	/* We cannot set the host device time so this function
	 * does nothing.
	 */

	return 0;
}

static int ptp_clock_get_lowrisc(const struct device *clk,
				      struct net_ptp_time *tm)
{
	ARG_UNUSED(clk);

	return lr_eth_clock_gettime(tm);
}

static int ptp_clock_adjust_lowrisc(const struct device *clk,
					 int increment)
{
	ARG_UNUSED(clk);
	ARG_UNUSED(increment);

	/* We cannot adjust the host device time so this function
	 * does nothing.
	 */

	return 0;
}

static int ptp_clock_rate_adjust_lowrisc(const struct device *clk,
					      float ratio)
{
	ARG_UNUSED(clk);
	ARG_UNUSED(ratio);

	/* We cannot adjust the host device time so this function
	 * does nothing.
	 */

	return 0;
}

static const struct ptp_clock_driver_api ptp_api = {
	.set = ptp_clock_set_lowrisc,
	.get = ptp_clock_get_lowrisc,
	.adjust = ptp_clock_adjust_lowrisc,
	.rate_adjust = ptp_clock_rate_adjust_lowrisc,
};

static int lr_ptp_init(const struct device *port)
{
	const struct device *eth_dev = DEVICE_DT_INST_GET(0);
	struct net_local_lr *dev_data = eth_dev->data;
	struct ptp_context *ptp_context = port->data;

	dev_data->ptp_clock = port;
	ptp_context->eth_dev = eth_dev;

	return 0;
}

DEVICE_DEFINE(eth_lowrisc_ptp_clock_0, PTP_CLOCK_NAME, lr_ptp_init,
		    device_pm_control_nop, &ptp_context_0, NULL, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &ptp_api);

#endif /* CONFIG_PTP_CLOCK_LOWRISC */
