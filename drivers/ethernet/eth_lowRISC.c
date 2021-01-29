
/*
 * Copyright (c) Recogni.
 *
 * LowRISC Ethernet driver
 */

#define DT_DRV_COMPAT lowrisc_eth


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

/******************************************************************************/

static void inline eth_write(struct net_local_lr *priv, size_t addr, int data)
{
    volatile u64 *eth_base = (volatile u64 *)(priv->ioaddr);
    eth_base[addr >> 3] = data;
}

static volatile inline int eth_read(struct net_local_lr *priv, size_t addr)
{
    volatile u64 *eth_base = (volatile u64 *)(priv->ioaddr);
    return eth_base[addr >> 3];
}


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


static int lr_send(const struct device *dev, struct net_pkt *pkt)
{
   struct net_local_lr *priv  = dev->data;

   int rslt = eth_read(priv, TPLR_OFFSET);
   size_t len = net_pkt_get_len(pkt);

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
    eth_base[MACHI_OFFSET >> 3] |= (MACHI_IRQ_EN | MACHI_ALLPKTS_MASK);
    mmiowb();
}

static void inline eth_disable_irq(struct net_local_lr *priv)
{
    volatile u64 *eth_base = (volatile u64 *)(priv->ioaddr);
    eth_base[MACHI_OFFSET >> 3] &= ~MACHI_IRQ_EN;
    mmiowb();
}

// TBD
static void lr_isr(struct device *dev)
{
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

   eth_update_address(priv, mac_addr);

   priv->ioaddr = (void *) 0x30000000;
//
//   /*
//    *  MDIO config
//    */
//   mdiobb_write(priv, 0, MII_BMCR,
//                BMCR_RESET | BMCR_ANRESTART | BMCR_SPEED100);
//

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
   return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T | ETHERNET_LINK_1000BASE_T;
}


static void lr_iface_init(struct net_if *iface)
{
   struct net_local_lr *dev = net_if_get_device(iface)->data;

   /* For VLAN, this value is only used to get the correct L2 driver.
    * The iface pointer in device context should contain the main
    * interface if the VLANs are enabled.
    */
    if (dev->iface == NULL) {
       dev->iface = iface;
//       /* Do the phy link up only once */
       IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
			lr_isr, NULL, DT_INST_IRQ(0, sense));

		irq_enable(DT_INST_IRQN(0));
	}

    ethernet_init(iface);
    net_if_set_link_addr(iface, dev->mac, sizeof(dev->mac), NET_LINK_ETHERNET);
    return;
}

static const struct ethernet_api lr_api = {
        .iface_api.init         = lr_iface_init,
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

