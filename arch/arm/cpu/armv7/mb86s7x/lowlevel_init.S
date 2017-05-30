/*
 * u-boot/arch/arm/cpu/armv7/mb86s7x/lowlevel_init.S
 *
 * Copyright (C) 2011-2012 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <asm/arch/hardware.h>

/*
 * lowlevel_init
 */
.globl lowlevel_init
lowlevel_init:
	/* STEP1 : CPU1 Sleeped */
	/*
	 * read MultiProcessor ID register(MPIDR)
	 */
	mrc	p15,0,r0,c0,c0,5
	and	r0, r0, #0x3			@ extract CPUID,bits
	cmp	r0, #1					@ 0x0 : processor is CPU0
							@ 0x1 : processor is CPU1
	bleq	CA9_1_SLEEP			@ CPU1 Sleeped

#if 0            /* skip PLL operation, it should be done by Cortex-M3 */
	/* STEP2 : check PLL Locked */
	ldr	r0, =CRG_BASE		@ 0xfff60000
PLL_LOCKED_CHECK:
	ldr	r1, [r0, #CRG_PLLRDY]
	and	r1, r1, #0x7
	cmp	r1, #0x7	@ bit0:MAIN_CRG11 PLLRDY status
				@ bit1:DMC_CRG11 PLLRDY status
				@ bit2:GMAC_CRG11 PLLRDY status
				@  0 -> Unlocked.
				@  1 -> Locked.
	bne	PLL_LOCKED_CHECK
#endif

	/* STEP3 : Initialize Core */
	/*
	 * Setup a temporary stack
	 */
	ldr	sp, =LOW_LEVEL_SDRAM_STACK_0	@ AP Trusted RAM

#if 0
	/* STEP4 : Initialize MEMC */
	push	{ip, lr}
	bl	init_memc			@ Call external function
	pop	{ip, lr}

	/* STEP5 : Initialize Data bahn */
	push	{ip, lr}
	bl	init_ddr3c			@ Call external function
	pop	{ip, lr}

	/* STEP6 : Initialize Peripheral Macro */
	/*
	 *  CRG11
	 *  CRRRS : Deasserting Request for RRESETn
	 */
	ldr	r0, =CRG_BASE		@ 0xfff60000
	ldr	r1, = 0x000000ff
	str	r1, [r0, #CRG_CRRRS]

	/*
	 * CRG11 Frequency Setting
	 */
	ldr	r0, =CRG_BASE		@ 0xfff60000

	/* CLK0 1/1 (500MHz) */
	ldr	r1, =0x00000000
	str	r1, [r0, #CRG_CRDM0]

	/* CLK1,2 1/2 (250MHz) */
	ldr	r1, =0x00000001
	str	r1, [r0, #CRG_CRDM1]
	str	r1, [r0, #CRG_CRDM2]

	/* CLK3 1/4 (125MHz)  */
	ldr	r1, =0x00000003
	str	r1, [r0, #CRG_CRDM3]

	/* CLK4,5,6 1/8 (62.5MHz) */
	ldr	r1, =0x00000007
	str	r1, [r0, #CRG_CRDM4]
	str	r1, [r0, #CRG_CRDM5]
	str	r1, [r0, #CRG_CRDM6]

	/* CLK7,8 1/4 (125MHz) */
	ldr	r1, =0x00000003
	str	r1, [r0, #CRG_CRDM7]
	str	r1, [r0, #CRG_CRDM8]

	/* CLK9,A,B 1/2  (250MHz) */
	ldr	r1, =0x00000001
	str	r1, [r0, #CRG_CRDM9]
	str	r1, [r0, #CRG_CRDMA]
	str	r1, [r0, #CRG_CRDMB]

	/* CLKC 1/4 (125MHz) */
	ldr	r1, =0x00000003
	str	r1, [r0, #CRG_CRDMC]

	/* CLKD 1/8 (62.5MHz) */
	ldr	r1, =0x00000007
	str	r1, [r0, #CRG_CRDMD]

	/* CLKE 1/16 (31.25MHz) */
	ldr	r1, =0x0000000f
	str	r1, [r0, #CRG_CRDME]

	/* CLKF 1/8 (62.5MHz) */
	ldr	r1, =0x00000007
	str	r1, [r0, #CRG_CRDMF]

	/* update clock divider mode */
	ldr	r1, =0x00000001
	str	r1, [r0, #CRG_CRCDC]

CLOCK_DIVIDER_UPDATE_CHECK:
	ldr	r1, [r0, #CRG_CRCDC]
	tst	r1, #0x1	@ bit0:DCHREQ
				@  0 -> The divider mode are updated.
				@  1 -> The divider mode are not updated.
	bne	CLOCK_DIVIDER_UPDATE_CHECK
#endif

	/* STEP7 : Initialize Memory System */
	/*
	 * Invalidate L1/L2 icache
	 */
	push	{ip, lr}
	bl	invalidate_dcache_all       @ Call external function
	pop	{ip, lr}

	mov	pc, lr

CA9_1_SLEEP:
	/*
	 * Setup a temporary stack
	 */
        ldr	sp, =LOW_LEVEL_SDRAM_STACK_1	@ AP Trusted RAM

	/*
	 * Invalidate L1/L2 icache
	 */
	push	{ip, lr}
	bl	invalidate_dcache_all       @ Call external function
	pop	{ip, lr}

#if 0
	ldr	r0, =MRBC_BASE
	ldr	r1, =0x0
	str	r1, [r0, #MRBC_GPREG0]
#endif

	/*
	 * Enable GICC irq
	 */
	ldr	r0, =GIC_IC_PHY_BASE
	ldr	r1, =0xff
	str	r1, [r0, #GICC_PRIMASK]
	ldr	r1, =0x1
	str	r1, [r0, #GICC_CTL]

BACK_TO_SLEEP:
	wfi	@ sleep

	/*
	 * read GICC_IAR
	 */
	ldr	r1, [r0, #GICC_INTACK]

	/*
	 * set EOI
	 */
	str	r1, [r0, #GICC_EOI]
	
	B BACK_TO_SLEEP
