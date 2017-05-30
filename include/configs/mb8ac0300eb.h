/*
 *  u-boot/include/configs/mb8ac0300eb.h
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
#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_ARMV7 1 /* This is an ARM cortexA9 Core */
#define CONFIG_MB8AC0300
#define CONFIG_MB8AC0300_IOCLK (662500000) /* 662.5MHz */

/* UARTx(PCLK) */
#define CONFIG_UART_CLK (CONFIG_MB8AC0300_IOCLK / 8) /* CLK6 82.8125MHz */

/* Timers for fasp(TIMCLK) */
#define CONFIG_TIMER_CLK (CONFIG_MB8AC0300_IOCLK / 16) /*  CLKE 41.40625MHz  */
#define CONFIG_SYS_HZ 1000 /* 1 msec */
#define CONFIG_SYS_TIMERBASE 0xfff6d000 /* Timer of ARM-SP804 */

#undef CONFIG_USE_IRQ /* we don't need IRQ/FIQ stuff */

/*
 * Device Tree Support
 */

#define CONFIG_OF_LIBFDT



/*
 * Hardware drivers support
 */

/* Serial(support)       */
#define CONFIG_SERIAL_MULTI
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE (-4)
#define CONFIG_SYS_NS16550_CLK CONFIG_UART_CLK
#define CONFIG_SYS_NS16550_COM1 0xfff6b000 /* UART 0 */
#define CONFIG_SYS_NS16550_COM2 0xfff6c000 /* UART 1 */

#define CONFIG_CONS_INDEX 1

/* I2C (Non support)     */
/* USB(Non support)      */

/* SD(support) */
#define CONFIG_SDHCI
#define CONFIG_F_SDH30_SDHCI
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_DOS_PARTITION
#define CONFIG_MMC_SDMA

/*
 * Ethernet(support)
 */
#define CONFIG_NET_MULTI
#define CONFIG_DRIVER_FGMAC4 1
#define CONFIG_FGMAC4_SYS_CLK 125000000 /* 125MHZ */
#define CONFIG_FGMAC4_BUF_SIZE 0x00100000 /* 1MB */

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
#define CONFIG_CMD_PING		/* ping support */
#define CONFIG_CMD_DHCP		/* dhcp support */

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

#define CONFIG_BOOTCOMMAND "mmc rescan ; ext2load mmc 0:1 40000000 uImage ; ext2load mmc 0:1 41000000 mb8ac0300eb.dtb; bootm 40000000 - 41000000"
#define CONFIG_BOOTARGS "mem=2048M loglevel=8 earlycon=ttyS0,115200 earlyprintk=0 console=ttyS0,115200 console=tty0 root=/dev/mmcblk0p2 rootfstype=ext4 rootwait"
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0"

/*
 * SDRAM (for initialize)
 */
#define CONFIG_NR_DRAM_BANKS 1 /* we have 1 bank of SDRAM */
#define CONFIG_SYS_SDRAM_BASE (0x40000000)  /* Start address of DDR3 */
#define PHYS_SDRAM_SIZE 0x80000000 /* size of DDR3(2GB) */

/*
 * FLASH and environment organization
 *
 * cf. Flash spec.
 *   Spansion S29GL128P90TFCR2
 *   S29GL-P_00_A102_j.pdf
 */

#define CONFIG_VIDEO_MB8AC0300

/*
 * if your U-Boot will run from XCS0, need to define this
 * currently, it kills NOR detect in U-Boot
 */

#define CONFIG_MB8AC0300_XCS0_MODE


#define UBOOT_SIZE 0x00100000 /* 1MB */
#define KERNEL_SIZE 0x00200000 /* 2MB */


#ifndef  CONFIG_MB8AC0300_XCS0_MODE

/* normal, XCS4 boot path */

#define CONFIG_SYS_FLASH_BASE 0x10000000 /* XCS4 BootROM(32MB) */
#define PHYS_FLASH_SIZE_1 0x02000000 /* (16+16)MB */
#define CONFIG_BOOTDELAY -1  /* disable autoboot */

#else

/* XCS0 boot path */

#define CONFIG_SYS_FLASH_BASE 0x11000000 /* XCS0 BootROM(16MB) */
#define PHYS_FLASH_SIZE_1 0x01000000 /* 16MB */
#define CONFIG_BOOTDELAY 0  /* enable autoboot */

#endif /*  CONFIG_MB8AC0300_XCS0_MODE  */


#define CONFIG_SYS_MAX_FLASH_BANKS 1
#define CONFIG_SYS_MAX_FLASH_SECT (128)
#define CONFIG_SYS_MONITOR_BASE CONFIG_SYS_FLASH_BASE

#define CONFIG_ENV_IS_IN_FLASH 1 /* create enviromment data in Flash ROM */
#define CONFIG_ENV_SIZE (0x40000) /* (128*2)KB */
#define CONFIG_ENV_ADDR (CONFIG_SYS_FLASH_BASE \
			+ UBOOT_SIZE - CONFIG_ENV_SIZE)
#define CONFIG_ENV_SECT_SIZE (0x40000) /* (128*2)KB */

/* Support JFFS2 */
#define CONFIG_JFFS2_DEV "nor0"
#define CONFIG_JFFS2_PART_OFFSET (UBOOT_SIZE + KERNEL_SIZE)
#define CONFIG_JFFS2_PART_SIZE (PHYS_FLASH_SIZE_1 - UBOOT_SIZE - KERNEL_SIZE)

/* Support MTD */
#define CONFIG_MTD_DEVICE 1
#define CONFIG_MTD_PARTITIONS
#define MTDIDS_DEFAULT "nor0=norflash-0"
#define MTDPARTS_DEFAULT "mtdparts=norflash-0:1m@0(uboot),2m(kernel),-(rt)"

#define CONFIG_FLASH_CFI_MTD 1
/*
 * CFI FLASH driver setup
 */
#define CONFIG_SYS_FLASH_CFI 1
#define CONFIG_FLASH_CFI_DRIVER 1
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE 1 /* ~10x faster */

#define CONFIG_SYS_LOAD_ADDR CONFIG_SYS_SDRAM_BASE /* default kernel load address */

#define CONFIG_SYS_MEMTEST_START (CONFIG_SYS_SDRAM_BASE + (512*1024))
#define CONFIG_SYS_MEMTEST_END (CONFIG_SYS_SDRAM_BASE + PHYS_SDRAM_SIZE)

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
#define CONFIG_SYS_TEXT_BASE 0x00000000 /* Boot from NOR */
#define CONFIG_SYS_INIT_SP_ADDR 0x43000000 /* stack of init proccess */

#define CONFIG_SYS_L2_PL310 1
#define CONFIG_SYS_PL310_BASE 0xf8102000

#define CONFIG_USE_ARCH_MEMCPY 1

#define CONFIG_DISPLAY_CPUINFO 1   /* Display CPU information */
#define CONFIG_DISPLAY_BOARDINFO 1 /* Display BOARD information */
#define CONFIG_SILENT_CONSOLE 1

#ifdef CONFIG_USE_IRQ
#error CONFIG_USE_IRQ not supported
#endif

#endif /* __CONFIG_H */
