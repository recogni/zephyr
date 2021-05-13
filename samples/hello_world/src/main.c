/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <pcie_recogni.h>
#include <drivers/pcie/pcie.h>
#include <sys/types.h>
#include <sys/device_mmio.h>
#include <drivers/pcie/pcie.h>

#define EP_DEV_VENDOR_ID  0x10c98086

void main(void)
{
	struct pcie_mbar mbar;
	pcie_bdf_t bdf = 0x100;
	mm_reg_t address;
	printk("Hello World! %s\n", CONFIG_BOARD);

    pcie_probe(0x100, EP_DEV_VENDOR_ID);

	pcie_get_mbar(bdf, 0, &mbar);
	pcie_set_cmd(bdf, PCIE_CONF_CMDSTAT_MEM |
			     PCIE_CONF_CMDSTAT_MASTER, true);

	device_map(&address, mbar.phys_addr, mbar.size, K_MEM_CACHE_NONE);

//	pcie_get_mbar(bdf, 1, &mbar);
//	pcie_get_mbar(bdf, 2, &mbar);
//	pcie_get_mbar(bdf, 3, &mbar);
//	pcie_get_mbar(bdf, 4, &mbar);
//	pcie_get_mbar(bdf, 4, &mbar);

	printk("0x%x\n", *(int *)address);

    printk("Done! %s\n", CONFIG_BOARD);

}
