/**
 * ogma.h
 *
 *  Copyright (c) 2012 - 2013 Fujitsu Semiconductor Limited.
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *   
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *   
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * 
 */
#ifndef OGMA_H
#define OGMA_H

#include <asm/io.h>
#include <common.h>
#include <net.h>
#include <asm/arch/hardware.h>
#include "ogma_reg.h"
#include "ogma_reg_f_taiki.h"
#include "ogma_reg_f_gmac_4mt.h"
#include "ogma_basic_access.h"
#include "ogma_config.h"
#include "ogma_internal.h"

struct eth_info	{
	int mdc_clk;			/* MDC clock			     */
//	struct fgmac4_phy_info phy_info;/* PHY info			     */

	dma_addr_t ring_dma;		/* base	address	of descriptor memory */
//	struct fgmac4_desc *rx_ring;	/* RX Desc ring's pointer	     */
//	struct fgmac4_desc *tx_ring;	/* TX Desc ring's pointer	     */
	unsigned char* rx_buf_array;	/* RX buffer pointer */
	unsigned char* tx_buf_array;	/* TX buffer pointer */
	u32 rx_ring_num;		/* Current RX desc number	     */
	u32 tx_ring_num;		/* Current TX desc number	     */
	u32 rx_buf_sz;			/* MAX size of socket buffer	     */

	struct eth_device netdev;
};

static __inline void ogma_write_reg (
    unsigned long reg_addr,
    unsigned long value
    )
{
	writel(value, (unsigned long *)(F_TAIKI_BASE + (reg_addr << 2)));
}

static __inline unsigned long ogma_read_reg (
    unsigned long reg_addr
    )
{
    return readl((unsigned long *)(F_TAIKI_BASE + (reg_addr << 2)));
}

#endif
