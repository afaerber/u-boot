/*
 * Configuration for ODROID-C2
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_MISC_INIT_R

/* Serial setup */
#define CONFIG_CONS_INDEX		0
#define CONFIG_BAUDRATE			115200

#define MESON_FDTFILE_SETTING "fdtfile=amlogic/meson-gxbb-odroidc2.dtb\0"

#ifdef CONFIG_DM_MMC
#define CONFIG_MMC_MESON_SD_PORT	1
#endif

#include <configs/meson-gxbb-common.h>

#endif /* __CONFIG_H */
