
/*
 * Copyright (c) Recogni.
 *
 * LowRISC Ethernet driver
 */

#define DT_DRV_COMPAT lowrisc_eth

#include <sys/types.h>
#include <zephyr.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>
#include <drivers/pcie/pcie.h>
#include "eth_lowRISC.h"

struct lowrisc_dev {
	struct net_if *iface;
	uint8_t mac[6];
	uint8_t txb[NET_ETH_MTU];
	uint8_t rxb[NET_ETH_MTU];
};


static int lr_tx(struct lowrisc_dev *dev, void *buf, size_t len)
{
        return 0;
}

static int lr_send(const struct device *device, struct net_pkt *pkt)
{
        return 0;
}

static void lr_isr(const struct device *device)
{
        return;
}

//DEVICE_DT_INST_DECLARE(0);

int lr_probe(const struct device *device)
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

static struct lowrisc_dev lr_dev;


ETH_NET_DEVICE_DT_INST_DEFINE(0,
                    lr_probe,
                    NULL,
                    &lr_dev,
                    NULL,
                    CONFIG_ETH_INIT_PRIORITY,
                    &lr_api,
                    NET_ETH_MTU);

