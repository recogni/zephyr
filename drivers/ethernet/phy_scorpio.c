#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <net/mii.h>

//#include <mdio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(eth_scoprio_phy, CONFIG_ETHERNET_LOG_LEVEL);

__subsystem struct mdio_driver_api {
	/** Enable the MDIO bus device */
	void (*bus_enable)(const struct device *dev);

	/** Disable the MDIO bus device */
	void (*bus_disable)(const struct device *dev);

	/** Read data from MDIO bus */
	int (*read)(const struct device *dev, uint8_t prtad, uint8_t devad, uint16_t *data);

	/** Write data to MDIO bus */
	int (*write)(const struct device *dev, uint8_t prtad, uint8_t devad, uint16_t data);
};

static void mdio_bus_enable(const struct device *eth)
{
}

static void mdio_bus_disable(const struct device *eth)
{
}

static int mdio_read(const struct device *dev, uint8_t prtad, uint8_t devad, uint16_t *data)
{
	return -1;
}

static int mdio_write(const struct device *dev, uint8_t prtad, uint8_t devad, uint16_t data)
{
	return -1;
}

struct mdio_driver_api scorpio_mdio_driver = { .bus_enable = mdio_bus_enable,
					       .bus_disable = mdio_bus_disable,
					       .read = mdio_read,
					       .write = mdio_write };