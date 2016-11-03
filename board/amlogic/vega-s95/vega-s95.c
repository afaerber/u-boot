/*
 * Copyright (c) 2016 Andreas Färber
 *
 * Based on ODROID-C2:
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/gxbb.h>
#include <asm/io.h>

int board_init(void)
{
	return 0;
}

int misc_init_r(void)
{
	/* Set RGMII mode */
	setbits_le32(GXBB_ETH_REG_0, GXBB_ETH_REG_0_PHY_INTF |
				     GXBB_ETH_REG_0_TX_PHASE(1) |
				     GXBB_ETH_REG_0_TX_RATIO(4) |
				     GXBB_ETH_REG_0_PHY_CLK_EN |
				     GXBB_ETH_REG_0_CLK_EN);

	/* Enable power and clock gate */
	setbits_le32(GXBB_GCLK_MPEG_1, GXBB_GCLK_MPEG_1_ETH);
	clrbits_le32(GXBB_MEM_PD_REG_0, GXBB_MEM_PD_REG_0_ETH_MASK);

	/* Reset PHY on GPIOZ_14 */
	clrbits_le32(GXBB_GPIO_EN(3), BIT(14));
	clrbits_le32(GXBB_GPIO_OUT(3), BIT(14));
	mdelay(10);
	setbits_le32(GXBB_GPIO_OUT(3), BIT(14));

	return 0;
}
