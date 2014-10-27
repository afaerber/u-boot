/*
 * (c) Copyright 2014 Andreas Färber <afaerber@suse.de>
 *
 * Configuration settings for the ALlwinner A80 (sun9i) CPU
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * A80 specific configuration
 */
#define CONFIG_SUN9I		/* sun9i SoC generation */
#define CONFIG_SYS_PROMPT	"sun9i# "

/*
 * Include common sunxi configuration where most of the settings are
 */
#include <configs/sunxi-common.h>

#endif /* __CONFIG_H */
