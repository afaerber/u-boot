/*
 * CI20 configuration
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_CI20_H__
#define __CONFIG_CI20_H__

#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_MISC_INIT_R
#define CONFIG_LZO

/* Ingenic JZ4780 clock configuration. */
#define CONFIG_SYS_HZ			1000
#define CONFIG_SYS_MHZ			1200
#define CONFIG_SYS_MIPS_TIMER_FREQ	(CONFIG_SYS_MHZ * 1000000)

/* Memory configuration */
#define CONFIG_SYS_MONITOR_LEN		(512 * 1024)
#define CONFIG_SYS_MALLOC_LEN		(64 * 1024 * 1024)
#define CONFIG_SYS_BOOTPARAMS_LEN	(128 * 1024)

#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_SDRAM_BASE		0x80000000 /* cached (KSEG0) address */
#define CONFIG_SYS_INIT_SP_OFFSET	0x400000
#define CONFIG_SYS_LOAD_ADDR		0x81000000
#define CONFIG_LOADADDR			CONFIG_SYS_LOAD_ADDR
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		0x88000000

#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE

/* NS16550-ish UARTs */
#define CONFIG_SYS_NS16550_CLK		CONFIG_SYS_EXTAL
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_CONSOLE_MUX
#define CONFIG_BAUDRATE			115200

/* Ethernet: davicom DM9000 */
#define CONFIG_DRIVER_DM9000		1
#define CONFIG_DM9000_BASE		0xb6000000
#define DM9000_IO			CONFIG_DM9000_BASE
#define DM9000_DATA			(CONFIG_DM9000_BASE + 2)

/* Environment */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_ENV_SIZE			(32 << 10)
#define CONFIG_ENV_OFFSET		((14 + 512) << 10)
#define CONFIG_ENV_OVERWRITE

/* Command line configuration. */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_CBSIZE	1024		/* Console I/O buffer size */
#define CONFIG_SYS_MAXARGS	32		/* Max number of command args */
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE
						/* Boot argument buffer size */
#define CONFIG_AUTO_COMPLETE			/* Command auto complete */
#define CONFIG_CMDLINE_EDITING			/* Command history etc */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "

/* Miscellaneous configuration options */
#define CONFIG_SYS_BOOTM_LEN		(64 << 20)
#define CONFIG_BOOTARGS			"console=ttyS0,115200 rw rootwait"

/* SPL */
#define CONFIG_SPL_FRAMEWORK

#define CONFIG_SPL_STACK		0xf4008000 /* only max. 2KB spare! */

#define CONFIG_SPL_TEXT_BASE		0xf4000a00
#define CONFIG_SPL_MAX_SIZE		((14 * 1024) - 0xa00)

#define CONFIG_SPL_BSS_START_ADDR	0xf4004000
#define CONFIG_SPL_BSS_MAX_SIZE		0x00002000 /* 512KB, arbitrary */

#define CONFIG_SPL_LDSCRIPT		"arch/mips/mach-jz47xx/jz4780/u-boot-spl.lds"
#define CONFIG_SPL_START_S_PATH		"arch/mips/mach-jz47xx"

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SPL_ABORT_ON_RAW_IMAGE
#define CONFIG_MMC_TINY
#undef CONFIG_DM_MMC
#endif

#endif /* __CONFIG_CI20_H__ */
