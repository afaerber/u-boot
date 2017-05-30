/*
 *  u-boot/include/configs/mb86s71.h
 *
 * Copyright (C) 2015 Socionext Inc.
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
#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_ARMV7 1 /* This is an ARM v7 CPU Core */
#define CONFIG_MB86S7X
#define CONFIG_MB86S71
#define CONFIG_MB86S71_IOCLK (500000000) /* 500MHz */

//#define CONFIG_SYS_DCACHE_OFF
//#define CONFIG_SYS_ICACHE_OFF

#define CONFIG_MB86S7X_MHU

/* UARTx(PCLK) */
#define CONFIG_UART_CLK (7813000)

/* Timers for fasp(TIMCLK) */
#define CONFIG_TIMER_CLK (CONFIG_MB86S71_IOCLK / 16) /*  CLKE 50MHz  */
#define CONFIG_SYS_HZ 1000 /* 1 msec */
#define CONFIG_SYS_TIMERBASE 0x31080000 /* AP Timer 1 (ARM-SP804) */

#define CONFIG_EMMC_CLOCK			(2000000)		/* 2MHz */
#define	CONFIG_SD_CLOCK				(50000000)		/* 50MHz */
#define CONFIG_EMMC_MAX_CLOCK			(200000000)		/* 200MHz */

#undef CONFIG_USE_IRQ /* we don't need IRQ/FIQ stuff */

/*
 * Device Tree Support
 */

#define CONFIG_OF_LIBFDT

#define BOOTM_DIRECT_START_LINUX


/*
 * Hardware drivers support
 */

/* Serial(support)       */
#define CONFIG_SERIAL_MULTI
#define CONFIG_PL011_SERIAL
#define CONFIG_PL011_CLOCK CONFIG_UART_CLK
#define CONFIG_PL01x_PORTS {(void *)(0x31040000), (void *)(0x31050000), (void *)(0x31060000)}

#define CONFIG_BAUDRATE			115200
#define CONFIG_CONS_INDEX 0

/* I2C (Not support)     */
/* USB (Not support)     */

/* SD(support) */
#define CONFIG_SDHCI
#define CONFIG_F_SDH30_SDHCI
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_DOS_PARTITION
#define CONFIG_MMC_SDMA
#define CONFIG_MMC_ADMA
#define CONFIG_SDHCI_AUTO_CMD12
#define CONFIG_SDHCI_AUTO_CMD23

/*
 * BOOTP options(support)
 */
#define CONFIG_BOOTP_BOOTFILESIZE 1
#define CONFIG_BOOTP_BOOTPATH 1
#define CONFIG_BOOTP_GATEWAY 1
#define CONFIG_BOOTP_HOSTNAME 1

/* Ethernet PHY options */
/* Enable these macro if gigabit is supported by hardware */
/* Auto negotiation */
#define CONFIG_PHY_SUPPORT_GIGA_AUTONEG
/* Force media */
/*#define CONFIG_PHY_SUPPORT_GIGA_FORCE */

/*
 * Command line configuration.
 *
 */
#include <config_cmd_default.h>
#undef CONFIG_CMD_FPGA		/* FPGA configuration Support	*/
#undef CONFIG_CMD_SETGETDCR	/* DCR support on 4xx		*/

#define CONFIG_CMD_ELF		/* bootelf, bootvx */
#define CONFIG_CMD_CACHE	/* icache, dcache */
#define CONFIG_CMD_JFFS2	/* JFFS2 Support */
#define CONFIG_CMD_MTDPARTS	/* MTD partition support */
#define CONFIG_CMD_MMC		/* mmc command support */
#define CONFIG_CMD_EXT2		/* EXT2 file system support */
#define CONFIG_CMD_FAT		/* FAT file system support */
#define CONFIG_CMD_ROMFS    /* ROM file system support */
#define CONFIG_CMD_PING		/* ping support */
#define CONFIG_CMD_DHCP		/* dhcp support */

/* f_taiki for ethernet */
#define CONFIG_DRIVER_OGMA
#define CONFIG_DRIVER_OGMA_BUF_START 0x88000000
#define CONFIG_DRIVER_OGMA_BUF_END   0x88200000

/* Network default configurations */
#define CONFIG_ETHADDR  12:34:56:78:9a:bc
#define CONFIG_NETMASK  255.255.255.0
#define CONFIG_IPADDR   192.168.1.105
#define CONFIG_SERVERIP 192.168.1.2
#define CONFIG_GATEWAYIP 192.168.1.1

#define CONFIG_ENV_OVERWRITE  /* ethaddr can be reprogrammed */

#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2 "> "

/* arguments for bootm command */
//#define CONFIG_BOOTARGS  "mem=2048M console=ttyS0,115200 mtdparts=physmap-flash.0:1m(u-boot)ro,2m(kernel),-(rt) root=/dev/mtdblock2 rw rootfstype=jffs2"

#define CONFIG_BOOTCOMMAND "mmc dev 1; mmc rescan ; ext2load mmc 1:1 80008000 boot/Image ; ext2load mmc 1:1 81000000 boot/mb86s71eb.dtb ; bootm 80008000 - 81000000"
//#define CONFIG_BOOTARGS "shm_offset=2048 loglevel=4 console=ttyS0,115200 root=/dev/mmcblk0p1 rootfstype=ext4 rootwait rw"
#define CONFIG_BOOTARGS "shm_offset=2048 loglevel=4 console=ttyAMA0,115200 root=/dev/mmcblk1p1 rootfstype=ext4 rootwait rw"
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0"

/*
 * SDRAM (for initialize)
 */
#define CONFIG_NR_DRAM_BANKS 4 /* we have 1 bank of SDRAM */
#define CONFIG_SYS_SDRAM_BASE (0x80000000)  /* Start address of DDR3 */
#define PHYS_SDRAM_SIZE 0x10000000 /* size of DDR3(256MB) */

/*
 * FLASH and environment organization
 *
 * cf. Flash spec.
 *   Spansion S29GL128P90TFCR2
 *   S29GL-P_00_A102_j.pdf
 */


/*
 * if your U-Boot will run from XCS0, need to define this
 * currently, it kills NOR detect in U-Boot
 */

#define UBOOT_SIZE 0x00100000 /* 1MB */
#define KERNEL_SIZE 0x00400000 /* 4MB */


/* boot path */
#define CONFIG_SYS_FLASH_BASE 0x48000000 /* XCS4 BootROM(32MB) */
#define PHYS_FLASH_SIZE_1 0x01000000 /* 16MB in memory map */
#define CONFIG_BOOTDELAY 2  /* disable autoboot */

//#define CONFIG_SKIP_FLASH_PROBE 1
#define CONFIG_SYS_MAX_FLASH_BANKS 1
#define CONFIG_SYS_MAX_FLASH_SECT (256 * 4)
#define CONFIG_SYS_MONITOR_BASE CONFIG_SYS_FLASH_BASE

#define CONFIG_ENV_IS_IN_SPI_FLASH 1
#define CONFIG_ENV_SIZE (48 * 512) /* 24KB */
#define CONFIG_BOOTPARAM_ADDR (0x2E003ECC)
#define CONFIG_BOOTPARAM_ADDR_MASK (0x00FFFFFF)
#define CONFIG_ENV_AREA_SIZE (0x40000)
#define CONFIG_ENV_OFFSET ((*((u32 *)CONFIG_BOOTPARAM_ADDR))&CONFIG_BOOTPARAM_ADDR_MASK)
#define CONFIG_PROTECTION_TB_OFFSET (CONFIG_ENV_OFFSET + CONFIG_ENV_AREA_SIZE)
#define CONFIG_ENV_SECT_SIZE (0x10000) /* (128*2)KB */
#define CONFIG_SYS_FLASH_PROTECTION
#define CONFIG_FLASH_CMD_FOR_SF

/* Support JFFS2 */
#define CONFIG_JFFS2_DEV "nor0"
#define CONFIG_JFFS2_PART_OFFSET (UBOOT_SIZE + KERNEL_SIZE)
#define CONFIG_JFFS2_PART_SIZE (PHYS_FLASH_SIZE_1 - UBOOT_SIZE - KERNEL_SIZE)

/* Support MTD */
#define CONFIG_MTD_DEVICE 1
#define CONFIG_MTD_PARTITIONS
#define MTDIDS_DEFAULT "nor0=norflash-0"
#define MTDPARTS_DEFAULT "mtdparts=norflash-0:2m@0(uboot),14m(kernel),-(rt)"

//#define CONFIG_FLASH_CFI_MTD 1
/*
 * CFI FLASH driver setup
 */
//#define CONFIG_SYS_FLASH_CFI 1
//#define CONFIG_FLASH_CFI_DRIVER 1
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE 1 /* ~10x faster */

//#define CONFIG_SCB_STORAGE 1

#define CONFIG_SYS_LOAD_ADDR CONFIG_SYS_SDRAM_BASE /* default kernel load address */

#define CONFIG_SYS_MEMTEST_START (CONFIG_SYS_SDRAM_BASE + (512*1024))
#define CONFIG_SYS_MEMTEST_END (CONFIG_SYS_SDRAM_BASE + PHYS_SDRAM_SIZE / 0x10)

#define CONFIG_BAUDRATE 115200
#define CONFIG_SYS_BAUDRATE_TABLE {115200, 19200, 38400, 57600, 9600 }

#define CONFIG_SYS_PROMPT "u-boot> "
#define CONFIG_SYS_CBSIZE 256
#define CONFIG_SYS_MAXARGS 128
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_LONGHELP 1
#define CONFIG_CMDLINE_EDITING 1
/* auto boot */
#define CONFIG_ZERO_BOOTDELAY_CHECK     /* check for keypress on bootdelay==0 */

#define CONFIG_SYS_MALLOC_LEN (0x00400000) /* 4Mbyte size of malloc() */
#define CONFIG_SYS_TEXT_BASE 0x80008000 /* Boot from HSSPI NOR */
#define CONFIG_SYS_INIT_SP_ADDR 0x80004000 /* stack of init proccess */

#define CONFIG_DISPLAY_CPUINFO 1   /* Display CPU information */
#define CONFIG_DISPLAY_BOARDINFO 1 /* Display BOARD information */
#define CONFIG_OF_BOARD_SETUP 1 /* config FDT for the board */

#ifdef CONFIG_USE_IRQ
#error CONFIG_USE_IRQ not supported
#endif

//#define NO_FIX_MEMORY_NODE 1

#define CONFIG_MB86S7X_HS_SPI 1
#define CONFIG_SPI_FLASH 1
#define CONFIG_SPI_FLASH_STMICRO 1
#define CONFIG_QUIRK_N25Q512A 1

#define CONFIG_FLASH_CHIP_SELECT
/* undef the commands we don't use */
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_LOADB
#undef CONFIG_CMD_LOADS

/* needs extra care*/
#undef CONFIG_CMD_LOOP
#undef CONFIG_CMD_IMXTRACT
#undef CONFIG_CMD_BOOTVX

#endif /* __CONFIG_H */
