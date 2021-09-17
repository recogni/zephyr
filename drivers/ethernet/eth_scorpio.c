#define DT_DRV_COMPAT scorpio_eth

#define LOG_MODULE_NAME eth_scorpio
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
#include <drivers/i2c.h>
#include <soc.h>

//#include "phy_scorpio_scorpio_eth.h"
//#include "eth_scorpio_scorpio_eth_priv.h"

#include "eth.h"

#define SCORPIO_ETH_MTU 1500

#ifdef CONFIG_SOC_FAMILY_SOC_RISCV64_SCORPIO
#include "scorpio_eth.h"
#endif

struct eth_scorpio_dev_data {
};
typedef struct scorpio_eth_s {
} scorpio_eth;

struct eth_scorpio_cfg {
	int init_done;
};

static const struct eth_scorpio_cfg eth0_config = {};

struct scorpio_eth_queue;

static inline struct net_if *get_iface(struct eth_scorpio_dev_data *ctx, uint16_t vlan_tag)
{
	return NULL;
}

// static void tx_completed(scorpio_eth *scorpio_eth, struct scorpio_eth_queue *queue)
// {
// }

// static int scorpio_eth_init(scorpio_eth *scorpio_eth, uint32_t scorpio_eth_ncfgr_val)
// {
// 	return -1;
// }

// static void link_configure(scorpio_eth *scorpio_eth, uint32_t flags)
// {
// }

// static struct net_pkt *frame_get(struct scorpio_eth_queue *queue)
// {
// 	return NULL;
// }

// static void eth_rx(struct scorpio_eth_queue *queue)
// {
// }

static int eth_tx(const struct device *dev, struct net_pkt *pkt)
{
	return -1;
}

static int eth_initialize(const struct device *dev)
{
	// const struct eth_scorpio_cfg *const cfg = DEV_CFG(dev);

	// cfg->config_func();

	return 0;
}

static void eth0_iface_init(struct net_if *iface)
{
}

static enum ethernet_hw_caps eth_scorpio_scorpio_eth_get_capabilities(const struct device *dev)
{
	return 0;
}

static int eth_scorpio_scorpio_eth_set_config(const struct device *dev,
					      enum ethernet_config_type type,
					      const struct ethernet_config *config)
{
	return -1;
}

static int eth_scorpio_scorpio_eth_get_config(const struct device *dev,
					      enum ethernet_config_type type,
					      struct ethernet_config *config)
{
	return -1;
}

static const struct ethernet_api eth_api = {
	.iface_api.init = eth0_iface_init,
	.get_capabilities = eth_scorpio_scorpio_eth_get_capabilities,
	.set_config = eth_scorpio_scorpio_eth_set_config,
	.get_config = eth_scorpio_scorpio_eth_get_config,
	.send = eth_tx,
};

static struct eth_scorpio_dev_data eth0_data = {};

ETH_NET_DEVICE_DT_INST_DEFINE(0, eth_initialize, NULL, &eth0_data, &eth0_config,
			      CONFIG_ETH_INIT_PRIORITY, &eth_api, SCORPIO_ETH_MTU);
