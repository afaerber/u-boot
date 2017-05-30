/**
 * f_usb20ho_lap.c - Fujitsu EHCI platform driver
 *
 * Copyright (c) 2013 FUJITSU SEMICONDUCTOR LIMITED
 *		http://jp.fujitsu.com/group/fsl
 *
 * based on bcma-hcd.c
 *
 * Author: FUJITSU SEMICONDUCTOR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
 
#include <common.h>
#include <usb.h>
#include <errno.h>
#include <asm/io.h>
#include "ehci.h"
#include <asm/hardware.h>
#ifdef MB86S7X_MHU_PHYS
#include <mhu.h>
#define DISABLE 0
#define ENABLE 1
#endif

int ehci_hcd_init(int index, struct ehci_hccr **hccr, struct ehci_hcor **hcor)
{
	int ret = 0;
	
	/* assign base address */
	*hccr = (struct ehci_hccr *)F_USB20HO_LAP_EHCI_BASE;
	*hcor = (struct ehci_hcor *)(F_USB20HO_LAP_EHCI_BASE + 0x10);

#ifdef CONFIG_MB86S7X_MHU
	/* enable power domain and usb 2.0 ehci host clock */
	if (!get_power_state(CONFIG_F_USB20HO_POWERDOMAIN))
		ret = set_power_state(CONFIG_F_USB20HO_POWERDOMAIN, ENABLE);

	if (ret)
		return ret;

	set_clk_state(2, 2, 4, ENABLE);  /* main_2_4 */
	set_clk_state(2, 4, 5, ENABLE);  /* main_4_5 */
	set_clk_state(4, 0, 0, ENABLE);  /* usb_0_0 */
#endif

	return ret;
}

int ehci_hcd_stop(int index)
{
#ifdef CONFIG_MB86S7X_MHU
	int ret = 0;

	/* disable PD#9 and PD#10 */
	
	if (get_power_state(CONFIG_F_USB20HO_POWERDOMAIN))
		ret = set_power_state(CONFIG_F_USB20HO_POWERDOMAIN, DISABLE);

	if (ret)
		return ret;
#endif

	return 0;
}
