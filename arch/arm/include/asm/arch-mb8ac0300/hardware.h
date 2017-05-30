/*
 * u-boot/arch/arm/include/asm/arch-mb8ac0300/hardware.h
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
#define LOW_LEVEL_SRAM_STACK_0	(0x01008000)  /* Stack for CPU0 */
#define LOW_LEVEL_SRAM_STACK_1	(0x01018000)  /* Stack for CPU1 */


/*
 * Address of Registers
 */

/* MRBC (Multi-core Remap and Boot Controller) */
#define MRBC_BASE	(0xfff68000)

#define MRBC_GPREG0	(0x30)		/* offset address */

/* CRG11 (Clock and Reset Generator) */
#define CRG_BASE	(0xfff60000)

#define CRG_CRPLC	(0x00)		/* offset address */
#define CRG_CRRDY	(0x04)		/* offset address */
#define CRG_CRSTP	(0x08)		/* offset address */
#define CRG_CRIMA	(0x10)		/* offset address */
#define CRG_CRPIC	(0x14)		/* offset address */
#define CRG_CRRSC	(0x20)		/* offset address */
#define CRG_CRSWR	(0x24)		/* offset address */
#define CRG_CRRRS	(0x28)		/* offset address */
#define CRG_CRRSM	(0x2c)		/* offset address */
#define CRG_CRCDC	(0x30)		/* offset address */
#define CRG_CRDM0	(0x100)		/* offset address */
#define CRG_CRLP0	(0x104)		/* offset address */
#define CRG_CRDM1	(0x110)		/* offset address */
#define CRG_CRLP1	(0x114)		/* offset address */
#define CRG_CRDM2	(0x120)		/* offset address */
#define CRG_CRLP2	(0x124)		/* offset address */
#define CRG_CRDM3	(0x130)		/* offset address */
#define CRG_CRLP3	(0x134)		/* offset address */
#define CRG_CRDM4	(0x140)		/* offset address */
#define CRG_CRLP4	(0x144)		/* offset address */
#define CRG_CRDM5	(0x150)		/* offset address */
#define CRG_CRLP5	(0x154)		/* offset address */
#define CRG_CRDM6	(0x160)		/* offset address */
#define CRG_CRLP6	(0x164)		/* offset address */
#define CRG_CRDM7	(0x170)		/* offset address */
#define CRG_CRLP7	(0x174)		/* offset address */
#define CRG_CRDM8	(0x180)		/* offset address */
#define CRG_CRLP8	(0x184)		/* offset address */
#define CRG_CRDM9	(0x190)		/* offset address */
#define CRG_CRLP9	(0x194)		/* offset address */
#define CRG_CRDMA	(0x1a0)		/* offset address */
#define CRG_CRLPA	(0x1a4)		/* offset address */
#define CRG_CRDMB	(0x1b0)		/* offset address */
#define CRG_CRLPB	(0x1b4)		/* offset address */
#define CRG_CRDMC	(0x1c0)		/* offset address */
#define CRG_CRLPC	(0x1c4)		/* offset address */
#define CRG_CRDMD	(0x1d0)		/* offset address */
#define CRG_CRLPD	(0x1d4)		/* offset address */
#define CRG_CRDME	(0x1e0)		/* offset address */
#define CRG_CRLPE	(0x1e4)		/* offset address */
#define CRG_CRDMF	(0x1f0)		/* offset address */
#define CRG_CRLPF	(0x1f4)		/* offset address */
#define CRG_PLLRDY	(0x200)		/* offset address */

/* MEMCS (MEMory Controller S) */
#define MEMC_BASE	(0xfff64000)

#define MEMC_MODE0	(MEMC_BASE + 0x00)
#define MEMC_MODE1	(MEMC_BASE + 0x04)
#define MEMC_MODE2	(MEMC_BASE + 0x08)
#define MEMC_MODE3	(MEMC_BASE + 0x0c)
#define MEMC_MODE4	(MEMC_BASE + 0x10)
#define MEMC_MODE5	(MEMC_BASE + 0x14)
#define MEMC_MODE6	(MEMC_BASE + 0x18)
#define MEMC_MODE7	(MEMC_BASE + 0x1c)
#define MEMC_TIM0	(MEMC_BASE + 0x20)
#define MEMC_TIM1	(MEMC_BASE + 0x24)
#define MEMC_TIM2	(MEMC_BASE + 0x28)
#define MEMC_TIM3	(MEMC_BASE + 0x2c)
#define MEMC_TIM4	(MEMC_BASE + 0x30)
#define MEMC_TIM5	(MEMC_BASE + 0x34)
#define MEMC_TIM6	(MEMC_BASE + 0x38)
#define MEMC_TIM7	(MEMC_BASE + 0x3c)
#define MEMC_AREA0	(MEMC_BASE + 0x40)
#define MEMC_AREA1	(MEMC_BASE + 0x44)
#define MEMC_AREA2	(MEMC_BASE + 0x48)
#define MEMC_AREA3	(MEMC_BASE + 0x4c)
#define MEMC_AREA4	(MEMC_BASE + 0x50)
#define MEMC_AREA5	(MEMC_BASE + 0x54)
#define MEMC_AREA6	(MEMC_BASE + 0x58)
#define MEMC_AREA7	(MEMC_BASE + 0x5c)

/* DDR3C (DDR3 sdram Controller) */
/* Databahn */
#define DDR3C_BASE	(0xfff4a000)

#define GPU_BASE (0xf0100000)
#define GPIO0_BASE (0xfff69000)
#define GPIO1_BASE (0xfff6a000)

/* Cortex-A9 MPCore Private Mem Region */
#define PERIPHBASE_A9	(0xf8100000)

/* GICC */
#define GICC_BASE	(PERIPHBASE_A9 + 0x100)

#define GICC_CTRL	(0x00)		/* offset address */
#define GICC_PMR	(0x04)		/* offset address */
#define GICC_IAR	(0x0c)		/* offset address */
#define GICC_EOIR	(0x10)		/* offset address */

/* FGMAC4 */
#define FGMAC4_BASE	(0xf4000000)

/* F_SDH30 */
#define F_SDH30_BASE	(0xf4002000)

#endif /* __ASM_ARCH_HARDWARE_H */
