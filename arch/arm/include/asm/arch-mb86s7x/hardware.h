/*
 * u-boot/arch/arm/include/asm/arch-mb86s7x/hardware.h
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#if !(defined(__KERNEL_STRICT_NAMES) || defined(__ASSEMBLY__))
#include <asm/types.h>
#endif /* !(__KERNEL_STRICT_NAMES || __ASSEMBLY__) */

/* stack of lowlevel_init */
/* Use SDRAM for stack since it is initialized before AP bootloader is running */
#define LOW_LEVEL_SDRAM_STACK_0	(0x89000000)  /* stack for CPU0 */
#define LOW_LEVEL_SDRAM_STACK_1	(0x8a000000)  /* Stack for CPU1 */

/*
 * Address of Registers
 */

/* MRBC (Multi-core Remap and Boot Controller) */
#define MRBC_BASE	(0x2A4C0000)

#define MRBC_GPREG0	(0x30)		/* offset address */

/* GIC */
#define GIC_ID_PHY_BASE 0x2C001000     /* Physical Distributor   */
#define GIC_IC_PHY_BASE 0x2C002000     /* Physical CPU interface */

/* Distributor interface registers */
#define GICD_CTL          0x0
#define GICD_CTR          0x4
#define GICD_SEC          0x80
#define GICD_ENABLESET    0x100
#define GICD_ENABLECLEAR  0x180
#define GICD_PENDINGSET   0x200
#define GICD_PENDINGCLEAR 0x280
#define GICD_ACTIVESET    0x300
#define GICD_ACTIVECLEAR  0x380
#define GICD_PRI          0x400
#define GICD_CPUS         0x800
#define GICD_CONFIG       0xC00
#define GICD_SW           0xF00
#define GICD_CPENDSGIR    0xF10
#define GICD_SPENDSGIR    0xF20

/* Physical CPU Interface registers */
#define GICC_CTL         0x0
#define GICC_PRIMASK     0x4
#define GICC_BP          0x8
#define GICC_INTACK      0xC
#define GICC_EOI         0x10
#define GICC_RUNNINGPRI  0x14
#define GICC_HIGHESTPEND 0x18
#define GICC_DEACTIVATE  0x1000
#define GICC_PRIODROP    GICC_EOI

/* FGMAC4 */
#define FGMAC4_BASE	(0x31400000)

/* F_ATIKI */
#define F_TAIKI_BASE (0x31600000)

/* F_SDH30 */
#define F_EMMC_BASE	(0x300C0000)
#define F_SDH30_BASE       (0x36600000)

/* mhu */
#define MB86S7X_MHU_PHYS		0x2b1f0000
#define MB86S7X_SHM_FROM_SCB		0x2e003000

/* USB 2.0 Host controller LAP, only on mb86s73 */
#define F_USB20HO_LAP_BASE 0x34200000
#define F_USB20HO_LAP_EHCI_BASE (F_USB20HO_LAP_BASE + 0x40000)

#define F_SPI_BASE  (0x48000000)
#define F_SPI_IP_BASE (0x30010000)
#define F_SYSOC_SPI (0x37300000)

#endif /* __ASM_ARCH_HARDWARE_H */
