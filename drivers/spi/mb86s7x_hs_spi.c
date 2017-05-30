/*
 *  u-boot/drivers/spi/mb86s7x_hs_spi.c
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


#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <asm/io.h>

#include <asm/errno.h>
#include "hs_spi_reg.h"
#include "mb8ac0300-hs_spi.h"
#include <asm/arch/hardware.h>


/* HS_SPI all TX interrupts except Slave Select Released Interrupt */
#define	HS_SPI_TXINT_EXCEPT_TSSRC	(TXC_TFMTC_MASK << TXC_TFMTC_OFFSET |\
				TXC_TFLETC_MASK << TXC_TFLETC_OFFSET |\
				TXC_TFUC_MASK << TXC_TFUC_OFFSET |\
				TXC_TFOC_MASK << TXC_TFOC_OFFSET |\
				TXC_TFEC_MASK << TXC_TFEC_OFFSET |\
				TXC_TFFC_MASK << TXC_TFFC_OFFSET)
/* HS_SPI all RX interrupts except Slave Select Released Interrupt */
#define	HS_SPI_RXINT_EXCEPT_RSSRC	(RXC_RFMTC_MASK << RXC_RFMTC_OFFSET |\
				RXC_RFLETC_MASK << RXC_RFLETC_OFFSET |\
				RXC_RFUC_MASK << RXC_RFUC_OFFSET |\
				RXC_RFOC_MASK << RXC_RFOC_OFFSET |\
				RXC_RFEC_MASK << RXC_RFEC_OFFSET |\
				RXC_RFFC_MASK << RXC_RFFC_OFFSET)
/* HS_SPI all TX interrupts */
#define	HS_SPI_TX_ALL_INT	(TXC_TSSRC_MASK << TXC_TSSRC_OFFSET |\
				HS_SPI_TXINT_EXCEPT_TSSRC)
/* HS_SPI all RX interrupts */
#define	HS_SPI_RX_ALL_INT	(RXC_RSSRC_MASK << RXC_RSSRC_OFFSET |\
				HS_SPI_RXINT_EXCEPT_RSSRC)
/* HS_SPI all fault interrupts */
#define	HS_SPI_ALL_FAULT	(FAULTC_DRCBSFC_MASK << FAULTC_DRCBSFC_OFFSET |\
				FAULTC_DWCBSFC_MASK << FAULTC_DWCBSFC_OFFSET |\
				FAULTC_PVFC_MASK << FAULTC_PVFC_OFFSET |\
				FAULTC_WAFC_MASK << FAULTC_WAFC_OFFSET |\
				FAULTC_UMAFC_MASK << FAULTC_UMAFC_OFFSET)
/* HS_SPI mode bits mask value */
#define	HS_SPI_MODE_MASK	(PCC_CPHA_MASK << PCC_CPHA_OFFSET |\
				PCC_CPOL_MASK << PCC_CPOL_OFFSET |\
				PCC_SSPOL_MASK << PCC_SSPOL_OFFSET |\
				PCC_SDIR_MASK << PCC_SDIR_OFFSET)
				

struct hs_spi	*hs;
static int init = 0;

#ifdef HS_SPI_DEBUG
static void dump_regs(struct hs_spi *hs)
{
	printf("reg:HS_SPI_REG_MCTRL, val:%x\n", hs_spi_readl(hs, MCTRL));
	printf("reg:HS_SPI_REG_PCC0, val:%x\n", hs_spi_readl(hs, PCC0));
	printf("reg:HS_SPI_REG_PCC1, val:%x\n", hs_spi_readl(hs, PCC1));
	printf("reg:HS_SPI_REG_PCC2, val:%x\n", hs_spi_readl(hs, PCC2));
	printf("reg:HS_SPI_REG_PCC3, val:%x\n", hs_spi_readl(hs, PCC3));
	printf("reg:HS_SPI_REG_TXF, val:%x\n", hs_spi_readl(hs, TXF));
	printf("reg:HS_SPI_REG_TXE, val:%x\n", hs_spi_readl(hs, TXE));
	printf("reg:HS_SPI_REG_TXC, val:%x\n", hs_spi_readl(hs, TXC));
	printf("reg:HS_SPI_REG_RXF, val:%x\n", hs_spi_readl(hs, RXF));
	printf("reg:HS_SPI_REG_RXE, val:%x\n", hs_spi_readl(hs, RXE));
	printf("reg:HS_SPI_REG_RXC, val:%x\n", hs_spi_readl(hs, RXC));

	printf("reg:HS_SPI_REG_FAULTF, val:%x\n", hs_spi_readl(hs, FAULTF));
	printf("reg:HS_SPI_REG_FAULTC, val:%x\n", hs_spi_readl(hs, FAULTC));
	printf("reg:HS_SPI_REG_DMCFG, val:%x\n", hs_spi_readb(hs, DMCFG));
	printf("reg:HS_SPI_REG_DMDMAEN, val:%x\n", hs_spi_readb(hs, DMDMAEN));
	printf("reg:HS_SPI_REG_DMSTART, val:%x\n", hs_spi_readb(hs, DMSTART));
	printf("reg:HS_SPI_REG_DMSTOP, val:%x\n", hs_spi_readb(hs, DMSTOP));
	printf("reg:HS_SPI_REG_DMPSEL, val:%x\n", hs_spi_readb(hs, DMPSEL));
	printf("reg:HS_SPI_REG_DMTRP, val:%x\n", hs_spi_readb(hs, DMTRP));
	printf("reg:HS_SPI_REG_DMBCC, val:%x\n", hs_spi_readw(hs, DMBCC));
	printf("reg:HS_SPI_REG_DMBCS, val:%x\n", hs_spi_readw(hs, DMBCS));
	printf("reg:HS_SPI_REG_DMSTATUS, val:%x\n", hs_spi_readl(hs, DMSTATUS));
	printf("reg:HS_SPI_REG_TXBITCNT, val:%x\n", hs_spi_readl(hs, TXBITCNT));
	printf("reg:HS_SPI_REG_FIFOCFG, val:%x\n", hs_spi_readl(hs, FIFOCFG));
	printf("reg:HS_SPI_REG_CSCFG, val:%x\n", hs_spi_readl(hs, CSCFG));
	printf("reg:HS_SPI_REG_CSITIME, val:%x\n", hs_spi_readl(hs, CSITIME));
	printf("reg:HS_SPI_REG_CSAEXT, val:%x\n", hs_spi_readl(hs, CSAEXT));

	printf("reg:HS_SPI_REG_RDCSDC0, val:%x\n", hs_spi_readw(hs, RDCSDC0));
	printf("reg:HS_SPI_REG_RDCSDC1, val:%x\n", hs_spi_readw(hs, RDCSDC1));
	printf("reg:HS_SPI_REG_RDCSDC2, val:%x\n", hs_spi_readw(hs, RDCSDC2));
	printf("reg:HS_SPI_REG_RDCSDC3, val:%x\n", hs_spi_readw(hs, RDCSDC3));
	printf("reg:HS_SPI_REG_RDCSDC4, val:%x\n", hs_spi_readw(hs, RDCSDC4));
	printf("reg:HS_SPI_REG_RDCSDC5, val:%x\n", hs_spi_readw(hs, RDCSDC5));
	printf("reg:HS_SPI_REG_RDCSDC6, val:%x\n", hs_spi_readw(hs, RDCSDC6));
	printf("reg:HS_SPI_REG_RDCSDC7, val:%x\n", hs_spi_readw(hs, RDCSDC7));
	printf("reg:HS_SPI_REG_WRCSDC0, val:%x\n", hs_spi_readw(hs, WRCSDC0));
	printf("reg:HS_SPI_REG_WRCSDC1, val:%x\n", hs_spi_readw(hs, WRCSDC1));
	printf("reg:HS_SPI_REG_WRCSDC2, val:%x\n", hs_spi_readw(hs, WRCSDC2));
	printf("reg:HS_SPI_REG_WRCSDC3, val:%x\n", hs_spi_readw(hs, WRCSDC3));
	printf("reg:HS_SPI_REG_WRCSDC4, val:%x\n", hs_spi_readw(hs, WRCSDC4));
	printf("reg:HS_SPI_REG_WRCSDC5, val:%x\n", hs_spi_readw(hs, WRCSDC5));
	printf("reg:HS_SPI_REG_WRCSDC6, val:%x\n", hs_spi_readw(hs, WRCSDC6));
	printf("reg:HS_SPI_REG_WRCSDC7, val:%x\n", hs_spi_readw(hs, WRCSDC7));
	printf("reg:HS_SPI_REG_MID, val:%x\n", hs_spi_readl(hs, MID));

}
#endif

/*
 * hs_spi_read_dummy - read dummy from the receive FIFO at direct mode
 * @hs:		HS SPI device platform data.
 *
 * When transfer protocol is TX_RX,
 * While TX-FIFO is transmitting data, RX-FIFO is also receiving dummy
 * at the same time.
 */
static void hs_spi_read_dummy(struct hs_spi *hs)
{
	unsigned int	rxbytes = HSSPI_BITS_GET(l, DMSTATUS_RXFLEVEL, hs,
			DMSTATUS);
	unsigned int	i;
	unsigned char	rxdata;

	for (i = 0; i < rxbytes; i++) {
		rxdata = hs_spi_fiforead(hs, RXFIFO0);
		debug("dummy data:%x", rxdata);
	}

	barrier(); /* why? */
}


static void hs_spi_chipselect(struct hs_spi *hs)
{	
	HSSPI_BITS_SET(l, DMPSEL_PSEL, hs->chip_select, hs, DMPSEL);
	HSSPI_BITS_SET(l, DMSTOP_STOP, 0, hs, DMSTOP);
}

static void hs_spi_stop_transfer(struct hs_spi *hs)
{	
	HSSPI_BITS_SET(l, DMSTOP_STOP, 1, hs, DMSTOP);
	while (!HSSPI_BITS_GET(l, DMSTOP_STOP, hs, DMSTOP))
		;
	if (hs->rx || ((hs->tx == NULL) && (hs->rx == NULL)))
			hs_spi_read_dummy(hs);
	while (!(HSSPI_BITS_GET(l, TXF_TSSRS, hs, TXF) |
			HSSPI_BITS_GET(l, RXF_RSSRS, hs, RXF)))
		;
}


/*
 * hs_spi_write_tx_fifo - write datas into the transmit FIFO at direct mode
 * @hs:		HS SPI device platform data.
 *
 * No more than 16 byte datas can be write once.
 *
 * Returns write data size
 */
static int hs_spi_write_tx_fifo(struct hs_spi *hs)
{
	unsigned int	txflevel = HSSPI_BITS_GET(l, DMSTATUS_TXFLEVEL, hs,
			DMSTATUS);
	unsigned int	txbytes = min(HS_SPI_FIFO_LEN - txflevel,
			hs->len - hs->tx_cnt);
	unsigned int	i;
	unsigned char	txdata;

	//printf("write tx fifo:");

	for (i = 0; i < txbytes; i++) {
		txdata = hs->tx ? hs->tx[hs->tx_cnt + i] : 0xFF;
		hs_spi_writeb(hs, TXFIFO0, txdata);
		//printf("%02x", txdata);
	}

	//printf("\nwrite done\n");

	return txbytes;
}

/*
 * hs_spi_read_rx_fifo - read datas from the receive FIFO direct mode
 * @hs:		HS SPI device platform data.
 *
 * No more then 16 byte datas can be read once.
 *
 * Returns read data size
 */
static int hs_spi_read_rx_fifo(struct hs_spi *hs)
{
	unsigned int	rxflevel = HSSPI_BITS_GET(l, DMSTATUS_RXFLEVEL, hs,
			DMSTATUS);
	unsigned int	rxbytes = min(rxflevel, hs->len - hs->rx_cnt);
	unsigned int	i;
	unsigned char	rxdata;

	//printf("read rx fifo, rxflevel:%d, data:", rxflevel);

	for (i = 0; i < rxbytes; i++) {
		rxdata = hs_spi_fiforead(hs, RXFIFO0);
		hs->rx[hs->rx_cnt + i] = rxdata;
		//printf("%02x", rxdata);
	}
	barrier(); /* dunno why this is needed */

	//printf("\nread done\n");

	return rxbytes;
}

static int hs_spi_tx(struct hs_spi	*hs)
{
	int			txf;

	while(hs->tx_cnt <= hs->len) {
		txf = hs_spi_readl(hs, TXF) & 0x7F;

		if(!txf)
			continue;
		
		/* clear flags */
		hs_spi_writel(hs, TXC, HS_SPI_TXINT_EXCEPT_TSSRC);

		if (txf & HSSPI_BIT(TXF_TFLETS)) {
			debug("TX-FIFO Fill Level <= Threshold\n");
			if (hs->tx_cnt < hs->len)
				hs->tx_cnt += hs_spi_write_tx_fifo(hs);
		}

		if (txf & HSSPI_BIT(TXF_TFES)) {
			debug("TX-FIFO and Shift Register is Empty\n");
			if (hs->tx_cnt >= hs->len) {
				//hs_spi_writel(hs, TXE, 0x00);
				break;
			}
		}
	}

	return 0;

}

static int hs_spi_rx(struct hs_spi	*hs)
{
	int			rxf;

	while(hs->rx_cnt < hs->len) {
		rxf = hs_spi_readl(hs, RXF) & 0x7F;

		if(!rxf)
			continue;
		
		/* clear flags */
		hs_spi_writel(hs, RXC, HS_SPI_RXINT_EXCEPT_RSSRC);

		if (rxf & HSSPI_BIT(RXF_RFMTS)) {
			debug("RX-FIFO Fill Level is More Than Threshold\n");
			hs->rx_cnt += hs_spi_read_rx_fifo(hs);
			if (hs->rx_cnt >= hs->len) {
				//hs_spi_writel(hs, RXE, 0x00);			
				break;
			}
		}
	}
	
	return 0;
}

static int hs_spi_txrx(struct hs_spi	*hs)
{	
	/* Flush RX and TX FIFO  */
	HSSPI_BITS_SET(l, FIFOCFG_TXFLSH, 1, hs, FIFOCFG);
	HSSPI_BITS_SET(l, FIFOCFG_RXFLSH, 1, hs, FIFOCFG);

	if (hs->tx) {
		/* set tx transfer protocol */
		if (hs->bitwidth == 2) {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_DUAL_TX_ONLY,
				hs, DMTRP);
		} else if (hs->bitwidth == 4) {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_QUAD_TX_ONLY,
				hs, DMTRP);
	} else {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_LEGACY_TX_ONLY,
				hs, DMTRP);
		}
		hs->tx_cnt += hs_spi_write_tx_fifo(hs);
	} else {
		/* set rx transfer protocol */
		if (hs->bitwidth == 2) {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_DUAL_RX_ONLY,
				hs, DMTRP);
		} else if (hs->bitwidth == 4) {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_QUAD_RX_ONLY,
				hs, DMTRP);
		} else {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_LEGACY_RX_ONLY,
				hs, DMTRP);
		}
	}
	hs_spi_writel(hs, TXC, HS_SPI_TX_ALL_INT);
	hs_spi_writel(hs, RXC, HS_SPI_RX_ALL_INT);
	/* start transfer */
	HSSPI_BITS_SET(l, DMSTART_START, 1, hs, DMSTART);
	if (hs->tx) {
		//hs_spi_writel(hs, TXE, TXE_TFLETE_MASK << TXE_TFLETE_OFFSET |
		//			TXE_TFEE_MASK << TXE_TFEE_OFFSET | TXE_TSSRE_OFFSET << TXE_TSSRE_OFFSET);
		hs_spi_tx(hs);
	}
	if (hs->rx) {
		//hs_spi_writel(hs, RXE, RXE_RFMTE_MASK << RXE_RFMTE_OFFSET | 
		//			RXE_RSSRE_MASK << RXE_RSSRE_OFFSET);
		hs_spi_rx(hs);
	}

	if (hs->fault_flag)
		return -EPERM;

	return hs->rx ? hs->rx_cnt : hs->tx_cnt;
}


struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct spi_slave *slave;

	spi_init();

	slave = malloc(sizeof(struct spi_slave));
	if (!slave)
		return NULL;

	slave->bus = bus;
	slave->cs = cs;

	return slave;

}

void spi_free_slave(struct spi_slave *slave)
{
	free(slave);
}

int spi_claim_bus(struct spi_slave *slave)
{
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	return;
}

int  spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
	int ret;
	if(!hs) {
		printf("spi device not exist\n");
		return -ENODEV;
	}

	hs->chip_select = slave->cs;
	hs_spi_chipselect(hs);

//	printf("spi_xfer, bitlen:%d, dout:%x, din:%x, flags:%x\n", bitlen, (unsigned int)dout, 
//		(unsigned int)din, flags);

	hs->tx = dout;
	hs->rx = din;
	hs->fault_flag	= 0;
	hs->tx_cnt	= 0;
	hs->rx_cnt	= 0;
	hs->len		= bitlen / 8; /* length in byte */

	ret = hs_spi_txrx(hs);

	if(flags & SPI_XFER_END) {
		hs_spi_stop_transfer(hs);
	}

	return ret <= 0;
	
}

void spi_set_speed(struct spi_slave *slave, uint hz)
{
	unsigned int		div;
	unsigned long		rate;
	unsigned char		safesync = 0;
	u32 csval = 0x230508;

	rate = 125000000;

	div = DIV_ROUND_UP(rate, hz * 2);
	/*
	 * If the resulting divider doesn't fit into the
	 * register bitfield, we can't satisfy the constraint.
	 */
	if (div > 127) {
		printf("setup: %d Hz too slow, div %u; min %ld Hz\n",
			hz, div, rate / (2 * 127));
		return;
	}

#if 0
	/* safesync bit */
	if (hs->mode == HS_SPI_DIRECT_MODE) {
		/* direct mode */
		if (((spi->rx_bitwidth == 4) ||
		     (spi->tx_bitwidth == 4)) &&
		     (div < 3))
			safesync = 1;
	} else
		/* cs mode */
		if (hs->pdata->clock == HS_SPI_PCLK)
			if (((spi->rx_bitwidth == 4) ||
				 (spi->tx_bitwidth == 4)) &&
			    (div < 3))
				safesync = 1;
#else
	safesync = 0;
#endif

	switch (hs->chip_select) {
	case 0:
		HSSPI_BITS_SET(l, PCC_CDRS, div, hs, PCC0);
		HSSPI_BITS_SET(l, PCC_ESYNC, safesync, hs, PCC0);
		HSSPI_BITS_SET(l, PCC_ACES, 0, hs, PCC0);
		HSSPI_BITS_SET(l, PCC_CPHA, 0, hs, PCC0);
		debug("Spi speed is set to %dHz, div:%u\n",
			hz, HSSPI_BITS_GET(l, PCC_CDRS, hs, PCC0));

		/* for now set them all to something good for mb86s70 */

		writel(csval, hs->reg + 4);
		writel(csval, hs->reg + 8);
		writel(csval, hs->reg + 0xc);
		writel(csval, hs->reg + 0x10);
		break;
	case 1:
		HSSPI_BITS_SET(l, PCC_CDRS, div, hs, PCC1);
		HSSPI_BITS_SET(l, PCC_ESYNC, safesync, hs, PCC1);
		debug("Spi speed is set to %dHz, div:%u\n",
			hz, HSSPI_BITS_GET(l, PCC_CDRS, hs, PCC1));
		/* for now set them all to something good for mb86s70 */

		writel(csval, hs->reg + 4);
		writel(csval, hs->reg + 8);
		writel(csval, hs->reg + 0xc);
		writel(csval, hs->reg + 0x10);
		break;
	case 2:
		HSSPI_BITS_SET(l, PCC_CDRS, div, hs, PCC2);
		HSSPI_BITS_SET(l, PCC_ESYNC, safesync, hs, PCC2);
		debug("Spi speed is set to %dHz, div:%u\n",
			hz, HSSPI_BITS_GET(l, PCC_CDRS, hs, PCC2));
		break;
	case 3:
		HSSPI_BITS_SET(l, PCC_CDRS, div, hs, PCC3);
		HSSPI_BITS_SET(l, PCC_ESYNC, safesync, hs, PCC3);
		debug("Spi speed is set to %dHz, div:%u\n",
			hz, HSSPI_BITS_GET(l, PCC_CDRS, hs, PCC3));
		break;
	default:
		printf("setup: invalid chipselect %u (%u defined)\n",
			hs->chip_select, 2);
		return;
	}
	//cs->speed_hz = hz;

	return;

}

void hs_spi_cleanup(void)
{
	if(!init)
		return;
	
	if(!hs)
		return;

	/* disable all interrupts */
	hs_spi_writel(hs, TXE, 0x00);
	hs_spi_writel(hs, RXE, 0x00);
	
	/* clear all interrupts */
	hs_spi_writel(hs, TXC, 0x7F);
	hs_spi_writel(hs, RXC, 0x7F);
	hs_spi_writel(hs, FAULTC, 0x7F);
	
	/* clean up memory */
	if(hs)
		free(hs);
	init = 0;
}

void spi_init(void)
{
	if(init)
		return;
	
	hs = malloc(sizeof(struct hs_spi));
	if(!hs) {
		return;
	}

	hs->reg = (void *)F_SPI_IP_BASE;
	hs->mode = HS_SPI_DIRECT_MODE;
	hs->clk_source = HS_SPI_HCLK;
	hs->chip_select = 0;
	hs->bitwidth = 8;
	hs->tx = NULL;
	hs->rx = NULL;

	/* change to dm mode */
	HSSPI_BITS_SET(l, MCTRL_CSEN, 0, hs, MCTRL);

	HSSPI_BITS_SET(l, MCTRL_MEN, 0, hs, MCTRL);
	while (HSSPI_BITS_GET(l, MCTRL_MES, hs, MCTRL))
		;

	/* disable interrupt */
	hs_spi_writel(hs, TXE, 0x00);
	hs_spi_writel(hs, RXE, 0x00);
	/* clear interrupt flag */
	hs_spi_writel(hs, TXC, HS_SPI_TXINT_EXCEPT_TSSRC);
	hs_spi_writel(hs, RXC, HS_SPI_RXINT_EXCEPT_RSSRC);

	/* read module ID */
	//printf("HS SPI module ID:%#4x\n", hs_spi_readl(hs, MID));

	/* Clock Division Source Select 0:AHBCLK 1:PCLK*/
	if (hs->clk_source == HS_SPI_HCLK)
		HSSPI_BITS_SET(l, MCTRL_CDSS, 0, hs, MCTRL);
	else
		HSSPI_BITS_SET(l, MCTRL_CDSS, 1, hs, MCTRL);

	if (hs->mode != HS_SPI_COMMAND_SEQUENCER)
		/* set to software flow control mode */
		HSSPI_BITS_SET(l, DMCFG_SSDC, 0, hs, DMCFG);
	else
		/* set to hardware flow control mode */
		HSSPI_BITS_SET(l, DMCFG_SSDC, 1, hs, DMCFG);

	/* configure the FIFO threshold levels and the FIFO width */
	hs_spi_writel(hs, FIFOCFG,
		HSSPI_BITS(FIFOCFG_FWIDTH, HS_SPI_FIFO_WIDTH) |
		HSSPI_BITS(FIFOCFG_TXFTH, HS_SPI_TX_FIFO_LEVEL) |
		HSSPI_BITS(FIFOCFG_RXFTH, HS_SPI_RX_FIFO_LEVEL));

	/* enable module */
	HSSPI_BITS_SET(l, MCTRL_MEN, 1, hs, MCTRL);
	while (!HSSPI_BITS_GET(l, MCTRL_MES, hs, MCTRL))
		;

	/* set max speed, don't need the slave parameter because we use only 1 chip */
	spi_set_speed(NULL, 31250000);

	/* enable all interrupt from tx or rx*/
	hs_spi_writel(hs, TXE, 0x7F);
	hs_spi_writel(hs, RXE, 0x7F);

	init = 1;
}

