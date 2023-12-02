/*
 * Copyright (c) 2021 Recogni, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME mmu

#include <zephyr.h>
#include <init.h>
#include <string.h>
#include <sys/mem_manage.h>
#include <kernel.h>
#include <spinlock.h>
#include <logging/log.h>
#include <arch/riscv/csr.h>
#include <assert.h>
#include <kernel_arch_interface.h>
#include <mmu.h>
#include <arch/riscv/riscv_mmu.h>

#include <linker/linker-defs.h>

#define LEVEL0_PAGE_BITS 12

#define VPN0_BITS 9
#define VPN1_BITS 9
#define VPN2_BITS 9

#define VPNS(va)									      \
	{										      \
		((va) >> LEVEL0_PAGE_BITS) & ((1 << VPN0_BITS) - 1),			      \
		((va) >> (VPN0_BITS + LEVEL0_PAGE_BITS)) & ((1 << VPN1_BITS) - 1),	      \
		((va) >> (VPN1_BITS + VPN0_BITS + LEVEL0_PAGE_BITS)) & ((1 << VPN2_BITS) - 1) \
	}

#define LEVEL_PAGE_SIZE(l) (1 << (((l) == 2 ? (VPN1_BITS + VPN0_BITS) : ((l) == 1 ? VPN0_BITS : 0)) + LEVEL0_PAGE_BITS))

#if CONFIG_MMU_PAGE_SIZE != LEVEL_PAGE_SIZE(1)
#error "Unsupported RISC-V MMU page size (only 2 MB page size is supported currently)"
#endif

#define PPN0_BITS VPN0_BITS
#define PPN1_BITS VPN1_BITS
#define PPN2_BITS (VPN2_BITS + 17)

#define PPNS(pa)									      \
	{										      \
		((pa) >> LEVEL0_PAGE_BITS) & ((1 << PPN0_BITS) - 1),			      \
		((pa) >> (PPN0_BITS + LEVEL0_PAGE_BITS)) & ((1 << PPN1_BITS) - 1),	      \
		((pa) >> (PPN1_BITS + PPN0_BITS + LEVEL0_PAGE_BITS)) & ((1 << PPN2_BITS) - 1) \
	}

#define NUM_OF_L2_PTE								     \
	(ROUND_UP(CONFIG_KERNEL_VM_BASE + CONFIG_KERNEL_VM_SIZE, LEVEL_PAGE_SIZE(2)) \
	 >> (VPN1_BITS + VPN0_BITS + LEVEL0_PAGE_BITS))

#define NUM_LEVELS 3
#define END_LEVEL  0

typedef uint64_t pentry_t;

/*****************************************************************************/

/*
 * Page tables.
 */
static uint64_t __noinit pt_l2[1 << VPN2_BITS] __attribute__((aligned(LEVEL_PAGE_SIZE(0))));
static uint64_t __noinit pt_l1[NUM_OF_L2_PTE][1 << VPN1_BITS] __attribute__((aligned(LEVEL_PAGE_SIZE(0))));
static uint64_t __noinit pt_l0[1][1 << VPN0_BITS] __attribute__((aligned(LEVEL_PAGE_SIZE(0))));

extern void pt_init(void);

/*
 *
 */
static struct k_spinlock riscv_mmu_lock;

/*
 *
 */

static void invalidate_tlb_all(void)
{
	__asm__ volatile ("fence.i\n"
			  "sfence.vma" : : : "memory");
}

/*****************************************************************************/

/*
 *
 */
static ALWAYS_INLINE bool is_leaf(uint64_t pte)
{
	return (pte & (PTE_R | PTE_W | PTE_X)) ? true : false;
}

/*
 *
 */
static ALWAYS_INLINE bool is_valid(uint64_t pte)
{
	return (pte & PTE_V) ? true : false;
}

/*
 *
 */
static void set_mapping(uint64_t *ptables, uintptr_t virt, uint64_t phys, size_t size, uint32_t pte_flags)
{
	pentry_t *table[NUM_LEVELS] = { [NUM_LEVELS - 1] = ptables };
	bool satp_mode_off = (csr_read(satp) & SATP64_MODE) == SATP_MODE_OFF;

	while (size > 0) {
		int vpns[NUM_LEVELS] = VPNS(virt);

		for (int level = NUM_LEVELS - 1; level >= END_LEVEL; level--) {
			__ASSERT(table[level] != NULL, "missing page table level %d when trying to map 0x%lx", level, virt);

			pentry_t *entryp = &table[level][vpns[level]];

			if (level < (NUM_LEVELS - 1) && !satp_mode_off) {
				entryp = (pentry_t *)Z_MEM_VIRT_ADDR((int64_t)entryp);
			}

			__ASSERT(is_valid(*entryp), "Page table entry at level %d not valid for 0x%" PRIxPTR, level, virt);

			if (level == END_LEVEL || is_leaf(*entryp)) {
				size_t level_size = LEVEL_PAGE_SIZE(level);

				__ASSERT((virt & (level_size - 1)) == 0,
					 "Virtual address 0x%" PRIxPTR " misaligned for page table entry at level %d",
					 virt,
					 level);

				__ASSERT((phys & (level_size - 1)) == 0,
					 "Physical address 0x%" PRIxPTR " misaligned for page table entry at level %d",
					 virt,
					 level);

				*entryp = ((phys >> LEVEL0_PAGE_BITS) << PTE_PPN_SHIFT) | pte_flags;

				virt += level_size;
				phys += level_size;
				size -= level_size;

				break;
			} else   {
				table[level - 1] = (pentry_t *)(((*entryp) >> PTE_PPN_SHIFT) << LEVEL0_PAGE_BITS);
			}
		}
	}

	__ASSERT(size == 0, "Cannot find the page entry!!");
}

/*
 *
 */

static void range_map(void *virt, uint64_t phys, size_t size, uint32_t pte_flags)
{
	set_mapping(pt_l2, (uintptr_t)virt, phys, size, pte_flags);
}

static inline void range_map_unlocked(void *virt, uint64_t phys, size_t size, uint32_t pte_flags)
{
	k_spinlock_key_t key;

	key = k_spin_lock(&riscv_mmu_lock);
	range_map(virt, phys, size, pte_flags);
	k_spin_unlock(&riscv_mmu_lock, key);
}

/*
 *
 */

static uint32_t convert_flags(uint32_t flags)
{
	uint32_t pte_flags = PTE_V | PTE_A | PTE_D;

	if ((flags & (K_MEM_PERM_RW | K_MEM_PERM_EXEC | K_MEM_PERM_USER)) == 0) {
		pte_flags |= PTE_W;
		return pte_flags;
	}

	if ((flags & K_MEM_PERM_RW) != 0U) {
		pte_flags |= PTE_R | PTE_W;
	}

	if ((flags & K_MEM_PERM_EXEC) != 0U) {
		pte_flags |= PTE_X;
	}

	if ((flags & K_MEM_PERM_USER) != 0U) {
		pte_flags |= PTE_U;
	}

	return pte_flags;
}

/*****************************************************************************/

/*
 * The function wraps the mmu setup. It includes page table initialization and
 * mmu configure.
 */
__attribute__((optimize("O0,no-omit-frame-pointer"))) void z_riscv_mmu_init(void)
{
	memset((void *)pt_l2, 0, sizeof(pt_l2));
	memset((void *)pt_l1, 0, sizeof(pt_l1));
	memset((void *)pt_l0, 0, sizeof(pt_l0));

	uint64_t virt = 0, phys = 0;
	size_t size = CONFIG_KERNEL_VM_BASE + CONFIG_KERNEL_VM_SIZE;

	/* Set up all intermediate and leaf page table entries with no permissions */
	while (size >= LEVEL_PAGE_SIZE(1)) {
		uint64_t v[3] = VPNS(virt);

		pt_l2[v[2]] = ((((uint64_t)pt_l1[v[2]]) >> LEVEL0_PAGE_BITS) << PTE_PPN_SHIFT) | PTE_V;

		__ASSERT(v[2] < NUM_OF_L2_PTE, "Too many level one page tables");

		/* If not setting up the lowest level one page... */
		if (v[1] != 0 || v[2] != 0) {
			pt_l1[v[2]][v[1]] = ((phys >> LEVEL0_PAGE_BITS) << PTE_PPN_SHIFT) | PTE_W | PTE_V;
		} else   {
			pt_l1[0][0] = ((((uint64_t)pt_l0[0]) >> LEVEL0_PAGE_BITS) << PTE_PPN_SHIFT) | PTE_V;
			for (int i = 0; i < (1 << PPN0_BITS); i++) {
				pt_l0[0][i] = ((phys >> LEVEL0_PAGE_BITS) << PTE_PPN_SHIFT) | PTE_V;
				phys += 1 << LEVEL0_PAGE_BITS;
			}
			phys -= LEVEL_PAGE_SIZE(1);
		}

		virt += LEVEL_PAGE_SIZE(1);
		phys += LEVEL_PAGE_SIZE(1);
		size -= LEVEL_PAGE_SIZE(1);
	}

	/* Set up translation and permissions for Zephyr regions */
	set_mapping(pt_l2,
		    Z_MEM_VIRT_ADDR(MMU_PAGE_ALIGN_DOWN(__rodata_region_start)),
		    MMU_PAGE_ALIGN_DOWN(__rodata_region_start) ^ mmu_config.data_section_pa_mask,
		    MMU_PAGE_ALIGN_UP(__rodata_region_end) - MMU_PAGE_ALIGN_DOWN(__rodata_region_start),
		    PTE_V | PTE_R | PTE_A | PTE_D);
	set_mapping(pt_l2,
		    Z_MEM_VIRT_ADDR(MMU_PAGE_ALIGN_DOWN(_image_ram_start)),
		    MMU_PAGE_ALIGN_DOWN(_image_ram_start) ^ mmu_config.data_section_pa_mask,
		    MMU_PAGE_ALIGN_UP(_image_ram_end) - MMU_PAGE_ALIGN_DOWN(_image_ram_start),
		    PTE_V | PTE_R | PTE_W | PTE_A | PTE_D);
	set_mapping(pt_l2,
		    Z_MEM_VIRT_ADDR(MMU_PAGE_ALIGN_DOWN(__text_region_start)),
		    MMU_PAGE_ALIGN_DOWN(__text_region_start) ^ mmu_config.text_section_pa_mask,
		    MMU_PAGE_ALIGN_UP(__text_region_end) - MMU_PAGE_ALIGN_DOWN(__text_region_start),
		    PTE_V | PTE_R | PTE_X | PTE_A | PTE_D);

	/* Set up translation and permissions for user-provided platform regions */
	for (int index = 0; index < mmu_config.num_regions; index++) {
		const struct riscv_mmu_region *region;
		region = (__typeof__(region))Z_MEM_PHYS_ADDR(POINTER_TO_UINT(&mmu_config.mmu_regions[index]));
		set_mapping(pt_l2, region->base_va, region->base_pa, region->size, PTE_V | region->attrs | PTE_A | PTE_D);
	}

	/* Remove all permissions from the lowest level two page */
	set_mapping(pt_l2, 0x0, 0x0, 4096, PTE_A | PTE_D);

	invalidate_tlb_all();

	/* Set the root level page table address and enable translation in supervisor and user modes */
	csr_write(satp, INSERT_FIELD((uint64_t)pt_l2 >> LEVEL0_PAGE_BITS, SATP64_MODE, SATP_MODE_SV39));

	/* Allow breakpoint exceptions to be delegated to supervisor mode */
	csr_write(medeleg, 1 << 3);

	/* Allow interrupts to be delegated to supervisor mode */
	csr_write(mideleg, MIP_SSIP | MIP_STIP | MIP_SEIP);

	/* Convert the supervisoe mode exception handler to a virtual address */
	csr_write(stvec, csr_read(mtvec) + Z_MEM_VM_OFFSET);

	/* Set the machine mode exception handler */
	extern void __irq_wrapper_machine(void);
	csr_write(mtvec, __irq_wrapper_machine);

	/* Set the previous privilege for exception return to supervisor mode */
	csr_clear(mstatus, MSTATUS_MPP);
	csr_set(mstatus, INSERT_FIELD(MSTATUS_SUM, MSTATUS_MPP, PRV_S));

	/* Adjust the return address of this function on the stack frame to a virtual address */
	((ulong_t *)__builtin_frame_address(0))[-1] += Z_MEM_VM_OFFSET;

	/* Adjust the current stack pointer to a virtual address an return from exception to
	   perform a privilege switch to supervisor mode with virtual address translation enabled */
	__asm__ volatile ("li t1, %0\n"
			  "add sp, sp, t1\n"
			  "la t0, 1f\n"
			  "add t0, t0, t1\n"
			  "csrw mepc, t0\n"
			  "mret\n"
			  "1:\n"
#if defined(CONFIG_RISCV_MMU_DEBUG)
			  "ebreak\n"
#endif
			  :
			  : "i" (Z_MEM_VM_OFFSET));
}

/*
 *
 */

void arch_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	uint32_t pte_flags = convert_flags(flags);

	switch (flags & K_MEM_CACHE_MASK) {
	case K_MEM_CACHE_NONE:
	case K_MEM_CACHE_WT:
		break;
	case K_MEM_CACHE_WB:
		phys ^= mmu_config.data_section_pa_mask;
		break;
	default:
		__ASSERT(false, "bad memory mapping flags 0x%x", flags);
	}

	range_map_unlocked(virt, (uint64_t)phys, size, pte_flags);

	invalidate_tlb_all();
}

/*
 *
 */
void arch_mem_unmap(void *addr, size_t size)
{
	range_map_unlocked((void *)addr, 0, size, convert_flags(0));

	invalidate_tlb_all();
}

/*
 *
 */

int arch_mem_domain_max_partitions_get(void)
{
	return CONFIG_MAX_DOMAIN_PARTITIONS;
}

/*
 *
 */
void arch_mem_domain_partition_add(struct k_mem_domain *domain, uint32_t partition_id)
{
	// TBD
}

/*
 *
 */
void arch_mem_domain_partition_remove(struct k_mem_domain *domain, uint32_t partition_id)
{
	// TBD
}

/*
 *
 */
void arch_mem_domain_thread_add(struct k_thread *thread)
{
}

/*
 *
 */

void arch_mem_domain_thread_remove(struct k_thread *thread)
{
}

#ifdef CONFIG_ARCH_HAS_RESERVED_PAGE_FRAMES

/*
 *
 */
static void mark_addr_page_reserved(uintptr_t addr, size_t len)
{
	uintptr_t pos = ROUND_DOWN(addr, CONFIG_MMU_PAGE_SIZE);
	uintptr_t end = ROUND_UP(addr + len, CONFIG_MMU_PAGE_SIZE);

	for (; pos < end; pos += CONFIG_MMU_PAGE_SIZE) {
		if (!z_is_page_frame(pos)) {
			continue;
		}

		struct z_page_frame *pf = z_phys_to_page_frame(pos);

		pf->flags |= Z_PAGE_FRAME_RESERVED;
	}
}

void arch_reserved_pages_update(void)
{
}

#endif

/*
 *
 */
static bool page_validate(pentry_t *ptables, uint8_t *addr, bool write)
{
	return true;
}

/*
 *
 */
void z_riscv_init_user_accesses(struct k_thread *thread)
{
	return;
}

/*
 *
 */
void z_riscv_configure_user_allowed_stack(struct k_thread *thread)
{
	return;
}

int arch_buffer_validate(void *addr, size_t size, int write)
{
	uint64_t *ptables = pt_l2;

	uint8_t *virt;
	size_t aligned_size;
	int ret = 0;

	/* addr/size arbitrary, fix this up into an aligned region */
	k_mem_region_align((uintptr_t *)&virt, &aligned_size, (uintptr_t)addr, size, CONFIG_MMU_PAGE_SIZE);

	for (size_t offset = 0; offset < aligned_size; offset += CONFIG_MMU_PAGE_SIZE) {
		if (!page_validate(ptables, virt + offset, write)) {
			ret = -1;
			break;
		}
	}

	return ret;
}
