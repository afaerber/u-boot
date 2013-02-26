/*
 * Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

/* keep the debugging on for now since it's a new feature */
#define DEBUG
#include <common.h>
#include <asm/errno.h>
#include <asm/arch/power.h>
#include <asm/arch/s5p-dp.h>
#include <cros/cros_splash.h>
#include <mkbp.h>

DECLARE_GLOBAL_DATA_PTR;

/* Minimum battery charge level before starting the full software */
#define BATTERY_LOW_THRESH_PERCENT 3 /* % of charge */

/* how long we are displaying the battery charging screen before powering off */
#define BATTERY_SCREEN_DURATION_MSECS 10000 /* milliseconds */

/* delay necessary to drain a 64-char UART FIFO at 115 kBds */
#define UART_FIFO_DRAIN_USECS 5000 /* microseconds */

/* Smart battery standard registers */
enum {
	REG_TEMPERATURE     = 0x08,
	REG_VOLTAGE         = 0x09,
	REG_CURRENT         = 0x0a,
	REG_AVERAGE_CURRENT = 0x0b,
	REG_RELATIVE_CHARGE = 0x0d,
	REG_BATTERY_STATUS  = 0x16,
	REG_DESIGN_CAP      = 0x18,
	REG_DESIGN_VOLTAGE  = 0x19,
};

/**
 * Try to reach the lowest power state available with screen on.
 */
static void wait_in_low_power(void)
{
	/* TODO(vpalatin) real power management ! */
	udelay(1000000);
}

/**
 * Display a battery animation
 *
 * Indicates we have not enough power to startup,
 * then shutdowns.
 */
static void charging_screen(void)
{
	ulong t0 = get_timer(0);

	/* complete screen initialization */
	exynos_lcd_check_next_stage(gd->fdt_blob, 1);
	/* display the "battery is charging" screen */
	cros_splash_display(0);

	/* wait for a fixed delay with the splash, then shutdown */
	while (get_timer(t0) < BATTERY_SCREEN_DURATION_MSECS) {
		cros_splash_display(0);
		wait_in_low_power();
		cros_splash_display(1);
		wait_in_low_power();
	}

	debug("now, shutting down...\n");
	/* no flush function, wait for UART FIFO draining */
	udelay(UART_FIFO_DRAIN_USECS);
	power_shutdown();
}

/**
 * Returns whether we have enough energy available to start the OS.
 *
 * @param mkbp_dev	handle to communicate with the EC
 * @return 0 if energy is too low to startup, >0 if the current state is fine
 */
static int energy_good(struct mkbp_dev *mkbp_dev)
{
	int ret;
	uint16_t bat_charge;
	struct ec_response_power_info *info = NULL;
	int battery_retries = 3;

	do {
		ret = mkbp_read_battery_reg(mkbp_dev, REG_RELATIVE_CHARGE,
					    &bat_charge);
	} while (ret < 0 && --battery_retries);
	if (ret < 0) {
		debug("%s: cannot read battery, fallback to normal boot\n",
			__func__);
		return 1;
	}
	if (bat_charge > BATTERY_LOW_THRESH_PERCENT) {
		debug("%s: battery level (%d%%) acceptable, booting...\n",
			__func__, bat_charge);
		return 1;
	}
	ret = mkbp_get_power_info(mkbp_dev, &info);
	if ((ret < 0) || info->usb_dev_type == 0) {
		debug("%s: no power plugged, stopping ...\n", __func__);
		/* no flush function, wait for UART FIFO draining */
		udelay(UART_FIFO_DRAIN_USECS);
		power_shutdown();
	}

	debug("%s: battery level %d%% charge limit %d mA, keep charging...\n",
		 __func__, bat_charge, info->usb_current_limit);
	/* we want the low battery screen */
	return 0;
}

void low_battery_init(void)
{
	struct mkbp_dev *mkbp_dev = board_get_mkbp_dev();
	if (!mkbp_dev) {
		debug("%s: no mkbp device: cannot check power status\n",
			__func__);
		return;
	}

	if (!energy_good(mkbp_dev))
		charging_screen();
}
