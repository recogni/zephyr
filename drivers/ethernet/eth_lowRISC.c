
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


//static int lr_tx(struct net_local_lr *dev, void *buf, size_t len)
//{
//        return 0;
//}

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


//static void lr_isr(struct device *dev)
//{
//        return;
//}

int lr_probe(const struct device *dev)
{
        return 0;
}

static enum ethernet_hw_caps lr_caps(const struct device *dev)
{
        return ETHERNET_LINK_1000BASE_T;
}


static void lr_iface_init(struct net_if *iface)
{
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
                    NULL,
                    &lr_dev,
                    NULL,
                    CONFIG_ETH_INIT_PRIORITY,
                    &lr_api,
                    NET_ETH_MTU);

