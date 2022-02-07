/*
 * Copyright (c) 2021 Recogni, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <toolchain.h>
#include <arch/cpu.h>
#include <arch/riscv/csr.h>
#include <sys/mem_manage.h>

GTEXT(__irq_wrapper_machine)

SECTION_FUNC(exception.other, __irq_wrapper_machine)

	/* Switch to the machine mode IRQ stack and save a few registers */
	csrrw sp, mscratch, sp
	addi sp, sp, -16
	sd t0, 0(sp)
	sd t1, 8(sp)

	/* Check whether the exception is the result of an interrupt */
	csrr t0, mcause
	li t1, SOC_MCAUSE_IRQ_MASK
	and t1, t0, t1
	beqz t1, not_interrupt

	/*
	 * All interrupts are delegated to supervisor mode, so disable the
	 * pending raw interrupt in machine mode and set a pending interrupt
	 * for supervisor mode
	 */
	csrr t0, mip
	li t1, (MIP_MSIP | MIP_MTIP | MIP_MEIP)
	and t0, t0, t1
	csrc mie, t0
	srli t0, t0, (IRQ_M_SOFT - IRQ_S_SOFT)
	csrs mip, t0

	/*
	 * Return from the machine mode exception and let the supervisor mode
	 * exception handler to be invoked due to the pending delegated interrupt
	 */
	j restore_and_return

not_interrupt:
	/*
	 * Check whether the exception is the result of an ecall instruction
	 * from supervisor mode that requires handling in machine mode
	 */
	li t1, SOC_MCAUSE_SUPERVISOR_ECALL_EXP
	bne t0, t1, delegate_exception
	li t1, SET_MIP_MIE_SYSCALL_ID
	beq a7, t1, do_supervisor_syscall

delegate_exception:
	/*
	 * To delegate all excpetions that get this far, copy the machine mode
	 * exception registers to the corresponding supervisor mode ones
	 */
	csrr t0, mepc
	csrw sepc, t0
	csrr t0, mcause
	csrw scause, t0
	csrr t0, mtval
	csrw stval, t0

	/*
	 * Convert the 2-bit previous privilege mode in machine mode to a
	 * 1-bit previous privilege mode in supervisor mode
	 */
	li t1, INSERT_FIELD(0, MSTATUS_SPP, PRV_S)
	csrrs t0, mstatus, t1
	li t1, MSTATUS_MPP
	and t0, t0, t1
	bnez t0, enter_supervisor
	li t1, INSERT_FIELD(0, MSTATUS_SPP, PRV_S)
	csrc mstatus, t1

enter_supervisor:
	/*
	 * Set the previous privilige mode to supervisor mode and the exception
	 * return address (physical) to the supervisor mode exception handler
	 */
	li t0, MSTATUS_MPP
	csrc mstatus, t0
	li t0, INSERT_FIELD(0, MSTATUS_MPP, PRV_S)
	csrs mstatus, t0
	la t0, __irq_wrapper
	li t1, Z_MEM_VM_OFFSET
	add t0, t0, t1
	csrw mepc, t0
	j restore_and_return

do_supervisor_syscall:
	/*
	 * Currently there's only one machine mode syscall to clear pending
	 * interrupts and enable them in machine mode again
	 */
	csrc mip, a0
	csrs mie, a1
	csrr t1, mepc
	addi t1, t1, 4
	csrw mepc, t1

restore_and_return:
	/* Restore saved registers and switch back to the original stack */
	ld t1, 8(sp)
	ld t0, 0(sp)
	addi sp, sp, 16
	csrrw sp, mscratch, sp

	/*
	 * Return from the machine mode exception to enter the supervisor mode
	 * exception handler and resume normal exception processing there
	 */
	mret


