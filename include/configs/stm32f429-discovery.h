/*
 * Copyright 2014 Andreas Färber
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <config_distro_defaults.h>

#define CONFIG_SYS_THUMB_BUILD

#define CONFIG_SYS_NO_FLASH

#define CONFIG_BAUDRATE			115200

/*
 * Command line configuration.
 */
#include <config_cmd_default.h>

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
