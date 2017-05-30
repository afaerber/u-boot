/*
 * u-boot/drivers/net/fgmac4.c
 *
 * Copyright (C) 2010-2012 FUJITSU SEMICONDUCTOR LIMITED
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
#include <command.h>
#include <net.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/system.h>
#include <malloc.h>
#include <asm/arch/hardware.h>
#include <i2c.h>
#include <config.h>

#include <linux/mii.h>
#include <linux/ethtool.h>

#include "fgmac4.h"

#define FGMAC4_CLK CONFIG_FGMAC4_SYS_CLK/1000000 /* SYS_CLK 125MHz *//* H_CLK */

#define READ_REG(r)  readl((volatile u32 *)(FGMAC4_BASE + r))
#define WRITE_REG(val,r) writel(val, (volatile u32 *)(FGMAC4_BASE + r))

#define ETH_ZLEN		60


/* MAX size of TX&RX size */
/* frame(1500)+EthernetHeader(14)+FCS(4)+NET_IP_ALIGN(2) */
#define BUFFER_SIZE		1536 /* 32 * 48 */

#define FGMAC4_TDESC_NUM	10
#define FGMAC4_RDESC_NUM	10

/* RX and TX descriptor number */
#define FGMAC4_DESC_NUM (FGMAC4_TDESC_NUM + FGMAC4_RDESC_NUM)

/* Size of descriptor spaces */
#define FGMAC4_DESC_BYTES  ((sizeof(struct fgmac4_desc) * FGMAC4_DESC_NUM))

/* After reset device, have to wait 2000ns before access the register */
#define DELAY_TIME		2		/* 2us = 2000ns */

/* wait counter */
#define WAIT_COUNT		1000
#define ANEG_WAIT_COUNT		(500*1000)

/* MAX address of 32 PHYs */
#define MAX_PHY_ADR		31

/* we use upper 1MB non-cached region for fgmac4's tx/rx buffers */
#define FGMAC4_BUF_BASE (CONFIG_SYS_SDRAM_BASE + PHYS_SDRAM_SIZE - CONFIG_FGMAC4_BUF_SIZE)

/* Define the RX/TX Descriptor ring */
static u8 *fgmac4_ring = (u8 *)FGMAC4_BUF_BASE;
static u8 *fgmac4_buf = (u8 *)(FGMAC4_BUF_BASE + FGMAC4_DESC_BYTES);

static struct eth_info fgmac4_info;

/* Generic MII registers. */

#ifndef MII_CTRL1000
#define MII_CTRL1000		0x09	/* 1000BASE-T control */
#endif /* MII_CTRL1000 */
#ifndef MII_STAT1000
#define MII_STAT1000		0x0a	/* 1000BASE-T status */
#endif /* MII_STAT1000 */

/* 1000BASE-T Control register */
#ifndef ADVERTISE_1000FULL
#define ADVERTISE_1000FULL	0x0200	/* Advertise 1000BASE-T full duplex */
#endif /* ADVERTISE_1000FULL */
#ifndef ADVERTISE_1000HALF
#define ADVERTISE_1000HALF	0x0100	/* Advertise 1000BASE-T half duplex */
#endif /* ADVERTISE_1000HALF */

/* 1000BASE-T Status register */
#ifndef LPA_1000FULL
#define LPA_1000FULL		0x0800	/* Link partner 1000BASE-T full duplex*/
#endif /* LPA_1000FULL */
#ifndef LPA_1000HALF
#define LPA_1000HALF		0x0400	/* Link partner 1000BASE-T half duplex*/
#endif /* LPA_1000HALF */

/* Enable or disable autonegotiation.  If this is set to enable,
 * the forced link modes above are completely ignored.
 */
#ifndef AUTONEG_DISABLE
#define AUTONEG_DISABLE		0x00
#endif /* AUTONEG_DISABLE */
#ifndef AUTONEG_ENABLE
#define AUTONEG_ENABLE		0x01
#endif /* AUTONEG_ENABLE */

/* Indicates what features are advertised by the interface. */
#ifndef ADVERTISED_10baseT_Half
#define ADVERTISED_10baseT_Half		(1 << 0)
#endif /* ADVERTISED_10baseT_Half */
#ifndef ADVERTISED_10baseT_Full
#define ADVERTISED_10baseT_Full		(1 << 1)
#endif /* ADVERTISED_10baseT_Full */
#ifndef ADVERTISED_100baseT_Half
#define ADVERTISED_100baseT_Half	(1 << 2)
#endif /* ADVERTISED_100baseT_Half */
#ifndef ADVERTISED_100baseT_Full
#define ADVERTISED_100baseT_Full	(1 << 3)
#endif /* ADVERTISED_100baseT_Full */
#ifndef ADVERTISED_1000baseT_Half
#define ADVERTISED_1000baseT_Half	(1 << 4)
#endif /* ADVERTISED_1000baseT_Half */
#ifndef ADVERTISED_1000baseT_Full
#define ADVERTISED_1000baseT_Full	(1 << 5)
#endif /* ADVERTISED_1000baseT_Full */

/* The forced speed, 10Mb, 100Mb, Gigabit. */
#ifndef SPEED_10
#define SPEED_10		10
#endif /* SPEED_10 */
#ifndef SPEED_100
#define SPEED_100		100
#endif /* SPEED_100 */
#ifndef SPEED_1000
#define SPEED_1000		1000
#endif /* SPEED_1000 */

/* Duplex, half or full. */
#ifndef DUPLEX_HALF
#define DUPLEX_HALF		0x00
#endif /* DUPLEX_HALF */
#ifndef DUPLEX_FULL
#define DUPLEX_FULL		0x01
#endif /* DUPLEX_FULL */


/*
 * fgmac4_phy_wait -- confirm that PHY is not busy
 *
 * Description: In this function, the driver will confirm whether PHY is
 * not busy.  If PHY does not become free in time, the driver will send
 * warning message to user and continue.
 */
static int fgmac4_phy_wait(void)
{
	u32 wtime;		/* waiting time */
	u32 val;

	/* confirm that PHY is not busy befor write/read GAR and GDR.
	 * We set a waitingtime, if PHY does not become free in time,
	 * we will send warning message to user and continue.
	 */
	wtime = WAIT_COUNT;
	while (wtime--) {
		val = READ_REG(FGMAC4_REG_GAR);
		if (!(val & FGMAC4_GAR_GB))
			return 0;
		udelay(DELAY_TIME);
	}

	printf("PHY is busy!\n");
	return 1;
}

/*
 * fgmac4_phy_read -- read PHY's register
 * @phy_addr: the id of phy. range:0-31
 * @reg_addr: phy register address
 * @read_val: read value from PHY register
 *
 * Description: read PHY's register by setting FGMAC4's GAR register and
 * GDR register.
 */
static int fgmac4_phy_read(u16 phy_addr, u16 reg_addr, u32 *read_val)
{
	int wval, ret;

	/* Set the GAR register */
	wval = (phy_addr << FGMAC4_GAR_PA_SHIFT)
		| ((reg_addr & FGMAC4_GAR_GR_MASK) << FGMAC4_GAR_GR_SHIFT)
		| FGMAC4_GAR_GW_R
		| ((fgmac4_info.mdc_clk & FGMAC4_GAR_CR_MASK) << FGMAC4_GAR_CR_SHIFT)
		| FGMAC4_GAR_GB;

	/* Wait until GMII/MII is not busy */
	ret = fgmac4_phy_wait();
	if (ret) {
		printf("Read PHY Error!\n");
		return ret;
	}

	WRITE_REG(wval, FGMAC4_REG_GAR);

	/* Wait until GMII/MII is not busy */
	ret = fgmac4_phy_wait();
	if (ret) {
		printf("Read PHY Error!\n");
		return ret;
	}

	*read_val = READ_REG(FGMAC4_REG_GDR);
	return 0;
}

/*
 * fgmac4_phy_write -- write PHY's register
 * @phy_addr: address of phy. range:0-31
 * @reg_addr: phy register address
 * @write_val: written value
 *
 * Description: read PHY's register by setting FGMAC4's GAR register and
 * GDR register.
 */
static int fgmac4_phy_write(u16 phy_addr, u16 reg_addr, u32 write_val)
{
	int wval, ret;

	/* Set the GAR register */
	wval = (phy_addr << FGMAC4_GAR_PA_SHIFT)
		| ((reg_addr & FGMAC4_GAR_GR_MASK) << FGMAC4_GAR_GR_SHIFT)
		| FGMAC4_GAR_GW_W
		| ((fgmac4_info.mdc_clk & FGMAC4_GAR_CR_MASK) << FGMAC4_GAR_CR_SHIFT)
		| FGMAC4_GAR_GB;

	/* Wait until GMII/MII is not busy */
	ret = fgmac4_phy_wait();
	if (ret) {
		printf("Read PHY Error!\n");
		return ret;
	}

	/* Set the value that is want to be written in PHY register */
	WRITE_REG(write_val, FGMAC4_REG_GDR);
	/* Set PHY Register access information */
	WRITE_REG(wval, FGMAC4_REG_GAR);

	/* Wait until GMII/MII is not busy */
	ret = fgmac4_phy_wait();
	if (ret) {
		printf("Read PHY Error!\n");
		return ret;
	}

	return 0;
}

/*
 * fgmac4_get_phyaddr -- find out PHYs on the device
 *
 * Description: Reads the ID registers of the every PHY addr(0-31) on the
 * device(if the corresponding bit is masked, skip it).
 * If the ID<>0x1FFFFFFF then set i to phy_id.
 */

#define PHY_MASK_ADDR 0xffffff01
static int fgmac4_get_phyaddr(void)
{
	u32 phy_id, phy_reg, i, phyaddr_mask = PHY_MASK_ADDR;
	int ret;

	/* initialize PHY installation information */
	fgmac4_info.phy_info.phy_addr = ~0;

	for (i = 0; i <= MAX_PHY_ADR; i++) {
		/* Do not scan the masked phy addr */
		if (phyaddr_mask & (1 << i))
			continue;

		/* Read phy ID from PHY's PHYSID1&PHYSID2 */
		ret = fgmac4_phy_read(i, MII_PHYSID1, &phy_reg);
		if (ret) {
			printf("Failed to read PHYID1 register!!!\n");
			return -EACCES;
		}
		phy_id = (phy_reg & 0xFFFF) << 16;	/* get PHYID high */

		ret = fgmac4_phy_read(i, MII_PHYSID2, &phy_reg);
		if (ret) {
			printf("Failed to read PHYID2 register!!!\n");
			return -EACCES;
		}
		phy_id |= (phy_reg & 0xFFFF);		/* get PHYID low */

		/* If the phy_id is mostly Fs, there is no device there */
		if ((phy_id & 0x1fffffff) == 0x1fffffff) {
			printf("There is no PHY in slot(%d).\n", i);
			continue;
		}

		/* set the bit of installed PHY */
		fgmac4_info.phy_info.phy_addr = i;
		break;
	}
	return 0;
}

/*
 * fgmac4_init_phyparam -- initialize PHY parameters
 * @phy_info: pointer to PHY info structure
 *
 * Description: Initialize PHY info structure base on environment "ethspeed".
 */
static int fgmac4_init_phyparam(struct fgmac4_phy_info* phy_info)
{
	char* env_p = getenv("ethspeed");
	if ((env_p == NULL) || (!strcmp(env_p, "auto"))) { /* Augoneg */
		phy_info->autoneg = AUTONEG_ENABLE;
		phy_info->adv = ADVERTISED_10baseT_Half
				| ADVERTISED_10baseT_Full
				| ADVERTISED_100baseT_Half
				| ADVERTISED_100baseT_Full;
#ifdef CONFIG_PHY_SUPPORT_GIGA_AUTONEG
		phy_info->adv |= ADVERTISED_1000baseT_Half
				| ADVERTISED_1000baseT_Full;
#endif /* CONFIG_PHY_SUPPORT_GIGA_AUTONEG */
	} else {
		/* parameter error */
		if (strcmp(env_p, "1000f") && strcmp(env_p, "1000h")
			&& strcmp(env_p, "100f") && strcmp(env_p, "100h")
			&& strcmp(env_p, "10f") && strcmp(env_p, "10h") ) {
			return -EINVAL;
		}
#ifndef CONFIG_PHY_SUPPORT_GIGA_FORCE
		/* If do not support 1000BaseT force media */
		if (!strcmp(env_p, "1000f") || !strcmp(env_p, "1000h")) {
			return -EINVAL;
		}
#endif /* !CONFIG_PHY_SUPPORT_GIGA_FORCE */
		phy_info->autoneg = AUTONEG_DISABLE;
#ifdef CONFIG_PHY_SUPPORT_GIGA_FORCE
		if (!strcmp(env_p, "1000f")) {
			phy_info->speed = SPEED_1000;
			phy_info->duplex = DUPLEX_FULL;
		} else if (!strcmp(env_p, "1000h")) {
			phy_info->speed = SPEED_1000;
			phy_info->duplex = DUPLEX_HALF;
		} else if (!strcmp(env_p, "100f")) {
#else /* !CONFIG_PHY_SUPPORT_GIGA_FORCE */
		if (!strcmp(env_p, "100f")) {
#endif /* CONFIG_PHY_SUPPORT_GIGA_FORCE */
			phy_info->speed = SPEED_100;
			phy_info->duplex = DUPLEX_FULL;
		} else if (!strcmp(env_p, "100h")) {
			phy_info->speed = SPEED_100;
			phy_info->duplex = DUPLEX_HALF;
		} else if (!strcmp(env_p, "10f")) {
			phy_info->speed = SPEED_10;
			phy_info->duplex = DUPLEX_FULL;
		} else {
			phy_info->speed = SPEED_10;
			phy_info->duplex = DUPLEX_HALF;
		}
	}

	return 0;
}

/*
 * fgmac4_phy_set -- set PHY parameters to PHY
 * @phy_info: pointer to PHY info structure
 *
 * Description: Set PHY parameters to PHY.
 */
static int fgmac4_phy_set (struct fgmac4_phy_info* phy_info)
{
	if (phy_info->autoneg == AUTONEG_ENABLE) {
		u32 bmcr, advert, tmp;
#ifdef CONFIG_PHY_SUPPORT_GIGA_AUTONEG
		u32 advert2, tmp2;
#endif /* CONFIG_PHY_SUPPORT_GIGA_AUTONEG */

		if ((phy_info->adv & (ADVERTISED_10baseT_Half |
				      ADVERTISED_10baseT_Full |
				      ADVERTISED_100baseT_Half |
				      ADVERTISED_100baseT_Full |
				      ADVERTISED_1000baseT_Half |
				      ADVERTISED_1000baseT_Full)) == 0)
			return -EINVAL;

		/* advertise only what has been requested */ /* read */
		fgmac4_phy_read(phy_info->phy_addr, MII_ADVERTISE, &advert);
		tmp = advert & ~(ADVERTISE_ALL | ADVERTISE_100BASE4);
#ifdef CONFIG_PHY_SUPPORT_GIGA_AUTONEG
		fgmac4_phy_read(phy_info->phy_addr, MII_CTRL1000, &advert2);
		tmp2 = advert2
			& ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
#endif /* CONFIG_PHY_SUPPORT_GIGA_AUTONEG */
		if (phy_info->adv & ADVERTISED_10baseT_Half)
			tmp |= ADVERTISE_10HALF;
		if (phy_info->adv & ADVERTISED_10baseT_Full)
			tmp |= ADVERTISE_10FULL;
		if (phy_info->adv & ADVERTISED_100baseT_Half)
			tmp |= ADVERTISE_100HALF;
		if (phy_info->adv & ADVERTISED_100baseT_Full)
			tmp |= ADVERTISE_100FULL;
#ifdef CONFIG_PHY_SUPPORT_GIGA_AUTONEG
		if (phy_info->adv & ADVERTISED_1000baseT_Half)
			tmp2 |= ADVERTISE_1000HALF;
		if (phy_info->adv & ADVERTISED_1000baseT_Full)
			tmp2 |= ADVERTISE_1000FULL;
#endif /* CONFIG_PHY_SUPPORT_GIGA_AUTONEG */
		if (advert != tmp)
			fgmac4_phy_write(phy_info->phy_addr,MII_ADVERTISE, tmp);
#ifdef CONFIG_PHY_SUPPORT_GIGA_AUTONEG
		if (advert2 != tmp2)
			fgmac4_phy_write(phy_info->phy_addr,MII_CTRL1000, tmp2);
#endif /* CONFIG_PHY_SUPPORT_GIGA_AUTONEG */
		/* turn on autonegotiation, and force a renegotiate */
		fgmac4_phy_read(phy_info->phy_addr, MII_BMCR, &bmcr);
		bmcr |= (BMCR_ANENABLE | BMCR_ANRESTART);
		fgmac4_phy_write(phy_info->phy_addr, MII_BMCR, bmcr);
	} else {
		u32 bmcr, tmp;
		if (phy_info->speed != SPEED_10 &&
			phy_info->speed != SPEED_100 &&
			phy_info->speed != SPEED_1000)
			return -EINVAL;
#ifndef CONFIG_PHY_SUPPORT_GIGA_FORCE
		if (phy_info->speed == SPEED_1000)
			return -EINVAL;
#endif /* !CONFIG_PHY_SUPPORT_GIGA_FORCE */

		if (phy_info->duplex != DUPLEX_HALF
			&& phy_info->duplex != DUPLEX_FULL)
			return -EINVAL;
		/* turn off auto negotiation, set speed and duplexity */
		fgmac4_phy_read(phy_info->phy_addr, MII_BMCR, &bmcr);
		tmp = bmcr & ~(BMCR_ANENABLE | BMCR_SPEED100 |
				   BMCR_SPEED1000 | BMCR_FULLDPLX);
		if (phy_info->speed == SPEED_1000)
			tmp |= BMCR_SPEED1000;
		else if (phy_info->speed == SPEED_100)
			tmp |= BMCR_SPEED100;
		if (phy_info->duplex == DUPLEX_FULL)
			tmp |= BMCR_FULLDPLX;
		if (bmcr != tmp)
			fgmac4_phy_write(phy_info->phy_addr, MII_BMCR, tmp);
	}
	return 0;
}

/*
 * fgmac4_get_adv -- Get advertising info
 * @phy_addr: address of phy. range:0-31
 * @reg_addr: phy register address
 *
 * Description: Get advertising info from MII_ADVERTISE and MII_LPA,
 * the reg_addr parameter can only be set to MII_ADVERTISE or MII_LPA.
 */
static u32 fgmac4_get_adv(u16 phy_addr, u16 reg_addr)
{
	u32 advert = 0, result = 0;

	fgmac4_phy_read(phy_addr, reg_addr, &advert);

	if (advert & ADVERTISE_10HALF)
		result |= ADVERTISED_10baseT_Half;
	if (advert & ADVERTISE_10FULL)
		result |= ADVERTISED_10baseT_Full;
	if (advert & ADVERTISE_100HALF)
		result |= ADVERTISED_100baseT_Half;
	if (advert & ADVERTISE_100FULL)
		result |= ADVERTISED_100baseT_Full;

	return result;
}

/*
 * fgmac4_phy_get -- get setted speed and duplex info from PHY
 * @phy_addr: address of phy. range:0-31
 * @speed_p: pointer which is used to contain getted speed info
 * @duplex_p: pointer which is used to contain getted duplex info
 *
 * Description: Get setted speed and duplex info from PHY
 */
static int fgmac4_phy_get (u16 phy_addr, u32* speed_p, u32* duplex_p)
{
	u32 bmcr, bmsr, ctrl1000 = 0, stat1000 = 0;
	u32 nego, local_advertising = 0, lp_advertising = 0;

	fgmac4_phy_read(phy_addr, MII_BMCR, &bmcr);
	fgmac4_phy_read(phy_addr, MII_BMSR, &bmsr);

	if (bmcr & BMCR_ANENABLE) {

#ifdef CONFIG_PHY_SUPPORT_GIGA_AUTONEG
		fgmac4_phy_read(phy_addr, MII_CTRL1000, &ctrl1000);
		fgmac4_phy_read(phy_addr, MII_STAT1000, &stat1000);
#endif /* CONFIG_PHY_SUPPORT_GIGA_AUTONEG */

		local_advertising |= fgmac4_get_adv(phy_addr, MII_ADVERTISE);
		if (ctrl1000 & ADVERTISE_1000HALF)
			local_advertising |= ADVERTISED_1000baseT_Half;
		if (ctrl1000 & ADVERTISE_1000FULL)
			local_advertising |= ADVERTISED_1000baseT_Full;

		if (bmsr & BMSR_ANEGCOMPLETE) {
			lp_advertising = fgmac4_get_adv(phy_addr, MII_LPA);
			if (stat1000 & LPA_1000HALF)
				lp_advertising |=
					ADVERTISED_1000baseT_Half;
			if (stat1000 & LPA_1000FULL)
				lp_advertising |=
					ADVERTISED_1000baseT_Full;
		} else {
			lp_advertising = 0;
		}

		nego = local_advertising & lp_advertising;

		if (nego & (ADVERTISED_1000baseT_Full |
				ADVERTISED_1000baseT_Half)) {
			*speed_p = SPEED_1000;
			*duplex_p = !!(nego & ADVERTISED_1000baseT_Full);
		} else if (nego & (ADVERTISED_100baseT_Full |
				   ADVERTISED_100baseT_Half)) {
			*speed_p = SPEED_100;
			*duplex_p = !!(nego & ADVERTISED_100baseT_Full);
		} else {
			*speed_p = SPEED_10;
			*duplex_p = !!(nego & ADVERTISED_10baseT_Full);
		}
	} else {
		*speed_p = ((bmcr & BMCR_SPEED1000 &&
				(bmcr & BMCR_SPEED100) == 0) ? SPEED_1000 :
				(bmcr & BMCR_SPEED100) ? SPEED_100 : SPEED_10);
		*duplex_p = (bmcr & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF;
	}

	return 0;
}

/*
 * fgmac4_probe -- initialize private info
 *
 * Description: Initialize private info
 */
static int fgmac4_probe(void)
{
	int i,err;

	/* get MDC clock from SYS_CLOCK */
	if (FGMAC4_CLK < 20) {
		printf("Unavailable FGMAC4_CLK(%d), aborting!!!\n", FGMAC4_CLK);
		return -EINVAL;

	} else if (FGMAC4_CLK < 35) {
		fgmac4_info.mdc_clk = 2;
	} else if (FGMAC4_CLK < 60) {
		fgmac4_info.mdc_clk = 3;
	} else if (FGMAC4_CLK < 100) {
		fgmac4_info.mdc_clk = 0;
	} else if (FGMAC4_CLK < 150) {
		fgmac4_info.mdc_clk = 1;
	} else if (FGMAC4_CLK < 250) {
		fgmac4_info.mdc_clk = 4;
	} else if (FGMAC4_CLK < 300) {
		fgmac4_info.mdc_clk = 5;
	} else {
		printf("Unavailable FGMAC4_CLK(%d), aborting!!!\n", FGMAC4_CLK);
		return -EINVAL;
	}

	err = fgmac4_get_phyaddr();	/* get phy install information */
	if (err) {
		printf("Failed to find PHY, aborting!!!\n");
		return -EINVAL;
	}

	/* initialize private info */
	fgmac4_info.rx_ring_num = 0;
	fgmac4_info.tx_ring_num = 0;

	fgmac4_info.rx_buf_sz = BUFFER_SIZE;	/* 1536 bytes */
	fgmac4_info.rx_buf_array = (unsigned char*)fgmac4_buf;
	fgmac4_info.tx_buf_array = fgmac4_info.rx_buf_array
					+ FGMAC4_RDESC_NUM * BUFFER_SIZE;

	/* Descriptor initialization */
	fgmac4_info.ring_dma = (dma_addr_t)fgmac4_ring;
	fgmac4_info.rx_ring = (struct fgmac4_desc *)(fgmac4_info.ring_dma);

	fgmac4_info.tx_ring =
		(struct fgmac4_desc *)((unsigned long)fgmac4_info.rx_ring
			+ FGMAC4_RDESC_NUM * sizeof(struct fgmac4_desc));

	memset(fgmac4_info.rx_ring, 0,
				sizeof(struct fgmac4_desc) * FGMAC4_RDESC_NUM);
	memset(fgmac4_info.tx_ring, 0,
				sizeof(struct fgmac4_desc) * FGMAC4_TDESC_NUM);

	/* set TX/RX end ring flags */
	fgmac4_info.rx_ring[FGMAC4_RDESC_NUM - 1].opts2 =
						FGMAC4_RDES1_RER;
	fgmac4_info.tx_ring[FGMAC4_TDESC_NUM - 1].opts1 =
						FGMAC4_TDES0_TER;
	/* init receive ring */
	for(i = 0; i < FGMAC4_RDESC_NUM; i++) {
		struct fgmac4_desc *rx_ring;

		rx_ring = &fgmac4_info.rx_ring[i];
		rx_ring->opts2 |= FGMAC4_RDES1_DIC | BUFFER_SIZE;

		rx_ring->addr1 = (u32)(&fgmac4_info.rx_buf_array[i * BUFFER_SIZE]);
		rx_ring->addr2 = 0;
		dmb();
		rx_ring->opts1 = OWN_BIT;

	}
	return 0;
}

/*
 * fgmac4_phy_aneg_wait -- wait the completion of autonegotiation
 * @phy_info: pointer to PHY info structure
 *
 * Description: Wait the completion of autonegotiation
 */
static int fgmac4_phy_aneg_wait(struct fgmac4_phy_info* phy_info)
{
	u32 ctl = 0, wtime;
	int ret = 0;

	/* sure autonegotiation success */
	wtime = ANEG_WAIT_COUNT;
	while (wtime--) {
		ret = fgmac4_phy_read(phy_info->phy_addr, MII_BMSR, &ctl);
		if (ret)
			return -EACCES;

		if ((ctl & BMSR_LSTATUS) && (ctl & BMSR_ANEGCOMPLETE)) {
			return 0;
		}
		udelay(DELAY_TIME);
	}

	return -EBUSY;
}

/*
 * fgmac4_setup_aneg -- setup autonegotiation
 * @phy_info: pointer to PHY info structure
 *
 * Description: Setup autonegotiation
 */
static int fgmac4_setup_aneg(struct fgmac4_phy_info* phy_info)
{
	int ret = 0;

	ret = fgmac4_init_phyparam(phy_info);
	if (ret)
		return ret;
	ret = fgmac4_phy_set(phy_info);
	if (ret)
		return ret;

	if (phy_info->autoneg == AUTONEG_ENABLE) {
		/* wait for completion of auto negotiation */
		ret = fgmac4_phy_aneg_wait(phy_info);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * fgmac4_write_hwaddr -- set FGMAC's MAC address
 * @netdev: pointer to ethernet device structure
 *
 * Description: Set FGMAC's MAC address
 */
static int fgmac4_write_hwaddr(struct eth_device *netdev)
{
	/* Mac Add = UU:VV:WW:XX:YY:ZZ
	 * MAR0H   = 32'h0000ZZYY
	 * MAR0L   = 32'hXXWWVVUU
	 */
	u32 reg_data;
	/* The validation of enetaddr is guaranteed by eth_initialize */
	unsigned char* mac_addr = netdev->enetaddr;

	printf(" MAC=%x:%x:%x:%x:%x:%x ",mac_addr[0], mac_addr[1],
		mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
	reg_data = (mac_addr[3] << 24)|(mac_addr[2] << 16)
		  |(mac_addr[1] << 8)|mac_addr[0];
	WRITE_REG(reg_data, FGMAC4_REG_MAR0L);
	reg_data = (mac_addr[5] << 8)|mac_addr[4];
	WRITE_REG(reg_data, FGMAC4_REG_MAR0H);

	return 0;
}

/*
 * fgmac4_disable_intr_all -- disable all FGMAC's interrupts
 *
 * Description: Disable all FGMAC's interrupts
 */
static void fgmac4_disable_intr_all(void)
{
	u32 val;

	/* disable all interrupt */
	WRITE_REG(0, FGMAC4_REG_IER);
	WRITE_REG((FGMAC4_IMR_LPIIM | FGMAC4_IMR_TSIM
		   | FGMAC4_IMR_PIM | FGMAC4_IMR_RGIM),
		FGMAC4_REG_IMR);
	/* Mask all interrupt of MMC */
	WRITE_REG(0x00FFFFFF, FGMAC4_REG_MMC_INTR_MASK_RX);
	WRITE_REG(0x01FFFFFF, FGMAC4_REG_MMC_INTR_MASK_TX);
	WRITE_REG(0x3FFF3FFF, FGMAC4_REG_MMC_IPC_INTR_MASK_RX);
	/* clear status register */
	val = READ_REG(FGMAC4_REG_SR);
	WRITE_REG(val, FGMAC4_REG_SR);
}

/*
 * fgmac4_init -- initialize fgmac4
 * @eth_device: pointer to ethernet device structure
 * @bd: pointer to board info structure
 *
 * Description: Initialize fgmac4
 */
static int fgmac4_init(struct eth_device *netdev, bd_t *bd)
{
	u32 val;
	int err;
	u32 rx_adr, tx_adr;
	u32 speed, duplex;

	err = fgmac4_probe();
	if(err)
		return err;

	/* DMA Initialization */

	/* 1.Set BMR(MDC Bus Mode Register) DMA reg 0 */
	val = (FGMAC4_BMR_8XPBL & SET_1)
	    | (FGMAC4_BMR_USP & SET_1)
	    | (FGMAC4_BMR_FB & SET_1)
	    | (FGMAC4_BMR_DA & SET_1)	/* DMA Arbitration Scheme:RX First */
	    | (FGMAC4_BMR_ATDS & SET_1) /* Use Alternative */
	    | FGMAC4_BMR_RPBL_16
	    | FGMAC4_BMR_PBL_16	/* When 8xPBL is set max value of PBL is 16 */
	    | FGMAC4_BMR_DSL;		/* Descripter Skip Length is 0 */

	WRITE_REG(val, FGMAC4_REG_BMR);

	/* 2. Disable interrupts */
	fgmac4_disable_intr_all();

	/* Set PMTR GMAC reg 11 */
	/* Disable Wake Up Frame | Disable Magic Packet */
	val = (FGMAC4_PMTR_WFE & SET_0)		/* Wake Up Frame Disable */
	    | (FGMAC4_PMTR_MPE & SET_0);		/* Magic Packet Disable  */

	WRITE_REG(val, FGMAC4_REG_PMTR);

	/* 3.Set MFFR(MAC Frame Filter Register) GMAC reg 1
	 *   With this set the LAN can do:
	 *   (1).Receive broadcast frames
	 *   (2).Receive packets that has the same unicast address as MARn
	 */
	WRITE_REG(0, FGMAC4_REG_MFFR);

	/* 4.Set MHTRH(MAC Hash Table Register High) GMAC reg 2
	 *       MHTRL(MAC Hash Table Register Low)  GMAC reg 3
	 */
	WRITE_REG(0, FGMAC4_REG_MHTRH);
	WRITE_REG(0, FGMAC4_REG_MHTRL);

	/* switch to GMII port */
	val = READ_REG(FGMAC4_REG_MCR) & (~FGMAC4_MCR_MII_PORT);
	WRITE_REG(val, FGMAC4_REG_MCR);
	udelay(10);
	/* 5.Setup PHY Autonegotiation */
	err = fgmac4_setup_aneg(&fgmac4_info.phy_info);
	if (err) {
		printf("Failed to setup AutoNegotiation!!!\n");
		return err;
	}

	/* 6.Set MCR(MAC Configuration Register) GMAC reg 0 */
	val = (FGMAC4_MCR_WD & SET_1)		/* Disable RX Watchdog timeout*/
	    | (FGMAC4_MCR_JD & SET_1)		/* Disable TX Jabber timer    */
	    | (FGMAC4_MCR_BE & SET_1)		/* Frame Burst Enable         */
	    | (FGMAC4_MCR_JE & SET_0)		/* Jumbo Frame Disable        */
	    | (FGMAC4_MCR_DO & SET_0)		/* Enable Receive Own         */
	    | (FGMAC4_MCR_LM & SET_0)		/* Not Loop-back Mode         */
	    | (FGMAC4_MCR_DR & SET_0)		/* Enable Retry               */
	    | (FGMAC4_MCR_DC & SET_0)		/* Deferral Check Disable     */
	    | (FGMAC4_MCR_TX_ENABLE & SET_1)	/* Enable Transmitter         */
	    | (FGMAC4_MCR_RX_ENABLE & SET_1)	/* Enable Receiver            */
	    | FGMAC4_MCR_BL_00;		/* Back-off Limit is 0                */

	err = fgmac4_phy_get(fgmac4_info.phy_info.phy_addr, &speed, &duplex);
	if (err) {
		printf("Failed to get speed and duplex info !!!\n");
		return err;
	}
	if (duplex == DUPLEX_FULL) {
		val |= FGMAC4_MCR_FULL_DUPLEX;
		val &= ~FGMAC4_MCR_IFG_MASK;

		val |= FGMAC4_MCR_IFG_80;
		val &= ~FGMAC4_MCR_DCRS;
	} else {
		val &= ~FGMAC4_MCR_FULL_DUPLEX;
		val &= ~FGMAC4_MCR_IFG_MASK;
		val |= FGMAC4_MCR_IFG_64;
		val |= FGMAC4_MCR_DCRS;
	}
	if (speed == SPEED_1000) {	/* GMII Port */
		/* MCR[bit15]: 0 = GMII Port; 1 = MII Port */
		val &= ~FGMAC4_MCR_MII_PORT;
	} else {	/* MII Port */
		val |= FGMAC4_MCR_MII_PORT;
	}
	WRITE_REG(val, FGMAC4_REG_MCR);

	udelay(10);		/* wait 10us */

	/* 7.Set RDLAR(MDC Receive Descriptor List Address Register)
	 *       TDLAR(MDC Transmit Descriptor List Address Register) */

	rx_adr = (u32)fgmac4_info.rx_ring;
	tx_adr = (u32)fgmac4_info.tx_ring;

	WRITE_REG(rx_adr, FGMAC4_REG_RDLAR);
	WRITE_REG(tx_adr, FGMAC4_REG_TDLAR);

	/* 8.Set OMR to start RX */
	val =
	    FGMAC4_OMR_TTC_256B		/* TX after 256 bytes written in FIFO */
	    /* | FGMAC4_OMR_START_TX */	/* Start Transmissin */
	    | FGMAC4_OMR_START_RX;	/* Start Receive     */

	WRITE_REG(val, FGMAC4_REG_OMR);

	return 0;
}

/*
 * fgmac4_halt -- stop hardware
 * @netdev: pointer to ethernet device structure
 *
 * Description: In this function, the driver will stop the TX/RX engine and
 * clear the status of hardware.
 */
static void fgmac4_halt(struct eth_device *netdev)
{
	u32 val;

	/* stop TX and RX process */
	val = 0;
	WRITE_REG(val, FGMAC4_REG_OMR);

	/* clear desc */
	fgmac4_info.ring_dma = 0;
	fgmac4_info.rx_ring = NULL;
	fgmac4_info.tx_ring = NULL;
	fgmac4_info.rx_buf_array = NULL;
	fgmac4_info.tx_buf_array = NULL;

	/* clear status register */
	val = READ_REG(FGMAC4_REG_SR);
	WRITE_REG(val, FGMAC4_REG_SR);

	return;
}

/*
 * fgmac4_send -- send a packet to media from the upper layer.
 * @netdev: pointer to ethernet device structure
 * @packet: pointer to header address of transmitting data
 * @length: data length
 *
 * Description: The send function does what you think -- transmit the specified
 * packet whose size is specified by length (in bytes).  You should not return
 * until the transmission is complete, and you should leave the state such that
 * the send function can be called multiple times in a row.
 */
static int fgmac4_send(struct eth_device *netdev, volatile void *packet,
							int length)
{
	u32 val;
	int tmo, ret = 0;

	struct fgmac4_desc *tx_ring;
	unsigned char* tx_buff_addr;
	int len = length;

	tx_ring = &fgmac4_info.tx_ring[fgmac4_info.tx_ring_num];

	tx_buff_addr = &fgmac4_info.tx_buf_array[fgmac4_info.tx_ring_num * BUFFER_SIZE];

	/* copy data to tx buffer */
	memcpy(tx_buff_addr, (const void*)packet, len);
	while (len < ETH_ZLEN)
		tx_buff_addr[len++] = '\0';

	fgmac4_info.tx_ring_num++;
	if(fgmac4_info.tx_ring_num == FGMAC4_TDESC_NUM) {
		fgmac4_info.tx_ring_num = 0;
	}

	/* set up transmit desc (TDES0-TDES3) and set TDES0[31] */
	tx_ring->opts1 |= FGMAC4_TDES0_FS | FGMAC4_TDES0_LS;
	tx_ring->opts2 = len;
	tx_ring->addr1  = (u32)tx_buff_addr;
	tx_ring->addr2  = 0;

	tx_ring->opts1  |= OWN_BIT;

	/* Make sure all needed operations were finished before starting DMA */
	dmb();

	/* set ST bit (DMA 6[13]) DMA enters the Runstate */
	val = READ_REG(FGMAC4_REG_OMR);
	val |= FGMAC4_OMR_START_TX;
	WRITE_REG(val, FGMAC4_REG_OMR);

	/* Transmit Poll Demand */
	val = 0xffffffff;
	WRITE_REG(val, FGMAC4_REG_TPDR);

	/* wait transmission complete */
	tmo = get_timer(0) + 5 * CONFIG_SYS_HZ; /* timer clk */

	while ((tx_ring->opts1 & 0x80000000) == OWN_BIT) {
		if (get_timer(0) >= tmo) {
			printf("transmission timeout SR[0x%08x]\n", READ_REG(FGMAC4_REG_SR));

			ret = -ETIMEDOUT;
			break;
		}
	}

	/* set ST bit to stop DMA */
	val = READ_REG(FGMAC4_REG_OMR);
	val &= ~FGMAC4_OMR_START_TX;
	WRITE_REG(val, FGMAC4_REG_OMR);

	return ret;
}

/*
 * fgmac4_rx -- received a packet and pass to upper layer
 * @netdev: pointer to ethernet device structure
 *
 * Description: The recv function should process packets as long as the hardware
 * has them readily available before returning.  i.e. you should drain the
 * hardware fifo. For each packet you receive, you should call the NetReceive()
 * function on it along with the packet length.  The common code sets up packet
 * buffers for you already in the .bss (NetRxPackets), so there should be no
 * need to allocate your own.  This doesn't mean you must use the NetRxPackets
 * array however; you're free to call the NetReceive() function with any buffer
 * you wish.
 * int ape_recv(struct eth_device *dev)
 * {
 * 	int length, i = 0;
 * 	...
 * 	while (packets_are_available()) {
 * 		...
 * 		length = ape_get_packet(&NetRxPackets[i]);
 * 		...
 * 		NetReceive(&NetRxPackets[i], length);
 * 		...
 * 		if (++i >= PKTBUFSRX)
 * 			i = 0;
 * 		...
 * 	}
 * 	...
 * 	return 0;
 */
static int fgmac4_rx(struct eth_device *netdev)
{
	u32 rx_len;
	int i;

	struct fgmac4_desc *rx_ring;

	/* recevie packet */
	for(i = 0;i < FGMAC4_RDESC_NUM; i++) {
		rx_ring = &fgmac4_info.rx_ring[i];

		if ((rx_ring->opts1 & 0x80000000) != OWN_BIT) {

			/* Read error state */
			if (rx_ring->opts1 & FGMAC4_RX_DESC_ERR) {
				printf("RX Desc has error status(0x%08x:0x%08x).\n",
						rx_ring->opts1,
						READ_REG (FGMAC4_REG_SR));
				rx_ring->opts1 = OWN_BIT;

				return -EIO;
			}
			/* Get data length */
			rx_len = ((rx_ring->opts1 >> 16) & 0x3FFF) - 4;

			/* Read received packet from buf */
			NetReceive((uchar *)rx_ring->addr1, rx_len);

			/*
			* Make sure all data operations were completed before
			* restart DMA, or, a potential data corruption may
			* happen while DMA is still going on.
			*/
			dmb();
			rx_ring->opts1 = OWN_BIT;
			break;
		}
	}

	return 0;
}
/*
 * fgmac4_initialize -- register ethernet device
 * @bis: pointer to board info
 *
 * Description: Register ethernet device
 */
int fgmac4_initialize(bd_t *bis)
{
	int ret;
	struct eth_device *dev = &(fgmac4_info.netdev);

	dev->init = fgmac4_init;
	dev->halt = fgmac4_halt;
	dev->send = fgmac4_send;
	dev->recv = fgmac4_rx;
	dev->write_hwaddr = fgmac4_write_hwaddr;
	sprintf(dev->name, "fgmac4");

	ret  = eth_register(dev);
	return ret;
}

