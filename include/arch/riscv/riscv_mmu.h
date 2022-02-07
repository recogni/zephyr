/*
 * Copyright (c) 2021 Recogni, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_ARCH_RISCV_RISCV_MMU_H_
#define ZEPHYR_INCLUDE_ARCH_RISCV_RISCV_MMU_H_

#include <arch/riscv/csr.h>

#ifndef _ASMLANGUAGE

/* Region definition data structure */
struct riscv_mmu_region {
	/* Region base physical Address */
	uintptr_t base_pa;
	/* Region base virtual Address */
	uintptr_t base_va;
	/* Region size */
	size_t size;
	/* Region atributes */
	uint32_t attrs;
};

/* MMU configuration data structure */
struct riscv_mmu_config {
	/* Number of regions */
	unsigned int num_regions;
	/* Regions */
	const struct riscv_mmu_region *mmu_regions;
};

/*
 * Convenience macros to represent the RISC-V-specific
 * configuration for memory access permission flags
 */
#define MMU_REGION_ENTRY(_name, _base_pa, _base_va, _size, _attrs) \
	{							   \
		.base_pa = _base_pa,				   \
		.base_va = _base_va,				   \
		.size = _size,					   \
		.attrs = _attrs,				   \
	}

#define MMU_REGION_FLAT_ENTRY(name, adr, sz, attrs) \
	MMU_REGION_ENTRY(name, adr, adr, sz, attrs)

#define MMU_PAGE_ALIGN_DOWN(x) ROUND_DOWN(x, CONFIG_MMU_PAGE_SIZE)
#define MMU_PAGE_ALIGN_UP(x)   ROUND_UP(x, CONFIG_MMU_PAGE_SIZE)

/* Reference to the MMU configuration.
 *
 * This struct is defined and populated for each SoC (in the SoC definition),
 * and holds the build-time configuration information for the fixed MMU
 * regions enabled during kernel initialization.
 */
extern const struct riscv_mmu_config mmu_config;

#endif  /* _ASMLANGUAGE */

#endif  /* ZEPHYR_INCLUDE_ARCH_RISCV_RISCV_MMU_H_ */
