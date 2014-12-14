/*
 * Copyright 2014 Andreas Färber
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <config_distro_defaults.h>
#undef CONFIG_CMD_BOOTZ
#undef CONFIG_CMD_DHCP
#undef CONFIG_CMD_ELF
#undef CONFIG_CMD_EXT2
#undef CONFIG_CMD_EXT4
#undef CONFIG_CMD_FAT
#undef CONFIG_CMD_FS_GENERIC
#undef CONFIG_CMD_MII
#undef CONFIG_CMD_NET
#undef CONFIG_CMD_PING
#undef CONFIG_CMD_PXE
#undef CONFIG_SYS_LONGHELP

#define CONFIG_SYS_THUMB_BUILD

#define CONFIG_SYS_NO_FLASH

#define CONFIG_STM32_USART
#define CONFIG_BAUDRATE			115200

/*
 * Command line configuration.
 */
#include <config_cmd_default.h>
#undef CONFIG_CMD_BDI
#undef CONFIG_CMD_BOOTD
#undef CONFIG_CMD_CONSOLE
#undef CONFIG_CMD_ECHO
#undef CONFIG_CMD_EDITENV
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_ITEST
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_LOADB
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_MEMORY
#undef CONFIG_CMD_MISC
#undef CONFIG_CMD_NET
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_SAVEENV
#undef CONFIG_CMD_SETGETDCR
#undef CONFIG_CMD_SOURCE
#undef CONFIG_CMD_XIMG

#define CONFIG_SYS_CBSIZE		256	/* Console I/O Buffer Size */
#define CONFIG_SYS_MAXARGS		16	/* max number of cmd args */

#define CONFIG_SYS_LOAD_ADDR		0x90000000

/*
 * Physical memory map
 */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM_1_SIZE		(8 << 20)
#define CONFIG_SYS_SDRAM_BASE		0x90000000

#define CONFIG_SYS_MALLOC_LEN		(64 * 1024)

/*
 * Environment
 */
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE			1024

#endif
