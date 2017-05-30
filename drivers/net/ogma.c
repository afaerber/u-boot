/*
 * u-boot/drivers/net/ogma.c
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

#include "ogma.h"

//#define OGMA_DEBUG 1

#ifdef OGMA_DEBUG
void dump_taiki_reg(void);
#define OGMA_LOG 1
#endif

#ifdef OGMA_LOG
#define ogma_print printf
#else
#define ogma_print(fmt, args...)
#endif

/*
 * Bit field definitions for PHY status register
 */
#define OGMA_PHY_SR_REG_AN_C                           (0x0020U)
#define OGMA_PHY_SR_REG_LINK                           (0x0004U)


/*
 * Bit field definitions for PHY 1000Base status register
 */
#define OGMA_PHY_1000BASE_REG_FULL                     (0x0800U)

/*
 * Bit field definitions for PHY ANLPA/ANA register
 */
#define OGMA_PHY_ANLPA_REG_TXF                         (0x0100U)
#define OGMA_PHY_ANLPA_REG_TXD                         (0x0080U)
#define OGMA_PHY_ANLPA_REG_TF                          (0x0040U)
#define OGMA_PHY_ANLPA_REG_TD                          (0x0020U)

/*
 * Bit field definitions for PHY ctrl register
 */
#define OGMA_PHY_CTRL_REG_RESET             (1U << 15)
#define OGMA_PHY_CTRL_REG_LOOPBACK          (1U << 14)
#define OGMA_PHY_CTRL_REG_SPSEL_LSB         (1U << 13)
#define OGMA_PHY_CTRL_REG_AUTO_NEGO_EN      (1U << 12)
#define OGMA_PHY_CTRL_REG_POWER_DOWN        (1U << 11)
#define OGMA_PHY_CTRL_REG_ISOLATE           (1U << 10)
#define OGMA_PHY_CTRL_REG_RESTART_AUTO_NEGO (1U << 9)
#define OGMA_PHY_CTRL_REG_DUPLEX_MODE       (1U << 8)
#define OGMA_PHY_CTRL_REG_COL_TEST          (1U << 7)
#define OGMA_PHY_CTRL_REG_SPSEL_MSB         (1U << 6)
#define OGMA_PHY_CTRL_REG_UNIDIR_EN         (1U << 5)

/*
 * Bit field definitions for PHY MASTER-SLAVE Ctrl register
 */
#define OGMA_PHY_MSC_REG_1000BASE_FULL (1U << 9)

/**
 * PHY register offset constants
 */
#define OGMA_PHY_REG_ADDR_CTRL        (0U)
#define OGMA_PHY_REG_ADDR_SR          (1U)
#define OGMA_PHY_REG_ADDR_ANA         (4U)
#define OGMA_PHY_REG_ADDR_ANLPA       (5U)
#define OGMA_PHY_REG_ADDR_MSC         (9U)
#define OGMA_PHY_REG_ADDR_1000BASE_SR (10U)

#define OGMA_RX_DESC_BASE CONFIG_DRIVER_OGMA_BUF_START 
#define OGMA_RX_BUF_ADDR (OGMA_RX_DESC_BASE + 0x00100000)
#define OGMA_TX_DESC_BASE (OGMA_RX_BUF_ADDR - 16)

#define OGMA_RX_PKT_BUF_LEN 1522
#define RX_DESC_NUM	250

#if ((OGMA_RX_DESC_BASE + RX_DESC_NUM * 16 > OGMA_TX_DESC_BASE) || (OGMA_RX_BUF_ADDR + RX_DESC_NUM * 2048 >= CONFIG_DRIVER_OGMA_BUF_END))
#error "RX_DESC_NUM too large"
#endif

ogma_ctrl_t *ctrl_p;

u32 rx_count = 0;


static struct eth_info ogma_info;

u8 phy_dev_addr = 1;

unsigned int flow_ctrl_start_threshold = 36;
unsigned int flow_ctrl_stop_threshold = 48;
unsigned short pause_time = 256;
int flow_ctrl = 0;
unsigned int ogma_phy_interface = OGMA_PHY_INTERFACE_RGMII;

static u32 scb_set_pkt_ctrl_reg_value = 0;
static u32 scb_set_normal_tx_phys_addr = 0;

static int ogma_write_hwaddr(struct eth_device *netdev)
{
	/* Mac Add = UU:VV:WW:XX:YY:ZZ
	 * MAR0H   = 32'h0000ZZYY
	 * MAR0L   = 32'hXXWWVVUU
	 */

	/* The validation of enetaddr is guaranteed by eth_initialize */
	unsigned char* mac_addr = netdev->enetaddr;

	printf(" MAC=%x:%x:%x:%x:%x:%x ",mac_addr[0], mac_addr[1],
		mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

	return 0;
}

static unsigned long ogma_calc_pkt_ctrl_reg_param (void)
{
    unsigned long param = OGMA_PKT_CTRL_REG_MODE_NRM;
    
    /* don't need other parameters for U-Boot */

    return param;
}

static void ogma_netdev_get_phy_link_status(
    unsigned int *link_status_flag_p,
    unsigned int *auto_nego_complete_flag_p,
    unsigned int *latched_link_down_flag_p,
    unsigned int *link_speed_p,
    unsigned int *half_duplex_flag_p
)
{

    unsigned short reg_val_sr, reg_val_ana, reg_val_anlpa, reg_val_msc, reg_val_mss;
    unsigned short common_settings;

    *link_status_flag_p = 0;
    *auto_nego_complete_flag_p = 0;
    *latched_link_down_flag_p = 0;
    *link_speed_p = OGMA_PHY_LINK_SPEED_10M;
    *half_duplex_flag_p = 0;

    /* Read PHY status register */
    reg_val_sr =
        ogma_get_phy_reg(phy_dev_addr,
                         OGMA_PHY_REG_ADDR_SR);

    if ((reg_val_sr & OGMA_PHY_SR_REG_LINK) == 0) {
        /*
         * Read PHY status register again to obtain up-to-date link status,
         */
        *latched_link_down_flag_p = 1;

        reg_val_sr =
            ogma_get_phy_reg(phy_dev_addr,
                             OGMA_PHY_REG_ADDR_SR);
    }

    if ((reg_val_sr & OGMA_PHY_SR_REG_LINK) != 0) {
        *link_status_flag_p = 1;
    }

    if ((reg_val_sr & OGMA_PHY_SR_REG_AN_C) != 0) {
        *auto_nego_complete_flag_p = 1;
    }

    /*
     * Determine link speed and duplex
     */
    if (((*link_status_flag_p) == 1) && ((*auto_nego_complete_flag_p) == 1)) {

        /* Read Auto-Negotiation Advertisement register */
        reg_val_ana =
            ogma_get_phy_reg(phy_dev_addr,
                             OGMA_PHY_REG_ADDR_ANA);

        /* Read Auto-Negotiation Link Partner Base Page Ability register */
        reg_val_anlpa =
            ogma_get_phy_reg(phy_dev_addr,
                             OGMA_PHY_REG_ADDR_ANLPA);

        /* Read MASTER-SLAVE Control register */
        reg_val_msc =
            ogma_get_phy_reg(phy_dev_addr,
                             OGMA_PHY_REG_ADDR_MSC);

        /* Read MASTER-SLAVE Status register (1000Base status) */
        reg_val_mss =
            ogma_get_phy_reg(phy_dev_addr,
                             OGMA_PHY_REG_ADDR_1000BASE_SR);

        /* Determine negotiation results */
        if (((reg_val_msc & OGMA_PHY_MSC_REG_1000BASE_FULL) != 0) &&
            ((reg_val_mss & OGMA_PHY_1000BASE_REG_FULL) != 0)) {

            *link_speed_p = OGMA_PHY_LINK_SPEED_1G;

        } else {

            common_settings = (reg_val_ana & reg_val_anlpa);

            if (common_settings & OGMA_PHY_ANLPA_REG_TXF) {
                *link_speed_p = OGMA_PHY_LINK_SPEED_100M;
            } else if (common_settings & OGMA_PHY_ANLPA_REG_TXD) {
                *link_speed_p = OGMA_PHY_LINK_SPEED_100M;
                *half_duplex_flag_p = 1;
            } else if (common_settings & OGMA_PHY_ANLPA_REG_TF) {
                *link_speed_p = OGMA_PHY_LINK_SPEED_10M;
            } else { /* OGMA_PHY_ANLPA_REG_TD */
                *link_speed_p = OGMA_PHY_LINK_SPEED_10M;
                *half_duplex_flag_p = 1;
            }

        }

    } /* if (((*link_status_flag_p) == 1) && ((*auto_nego_complete_flag_p) == 1)) */

}

static ogma_err_t ogma_netdev_configure_mac(
    int link_speed,
    int half_duplex_flag
    )
{

    unsigned int ogma_err;
    ogma_gmac_mode_t ogma_gmac_mode;

    memset(&ogma_gmac_mode, 0, sizeof(ogma_gmac_mode_t));

    ogma_gmac_mode.link_speed = link_speed;
    ogma_gmac_mode.half_duplex_flag = (int)half_duplex_flag;
    ogma_gmac_mode.flow_ctrl_enable_flag = (int)flow_ctrl;
    ogma_gmac_mode.flow_ctrl_start_threshold =
        (u16)flow_ctrl_start_threshold;
    ogma_gmac_mode.flow_ctrl_stop_threshold =
        (u16)flow_ctrl_stop_threshold;
/*
    ogma_err =
        ogma_stop_gmac(1, 1);
    if (ogma_err != OGMA_ERR_OK) {
        printf("ogma_stop_gmac() failed with error status %d\n", ogma_err);
        goto out;
    }

    ogma_err = ogma_set_gmac_mode(&ogma_gmac_mode);
    if (ogma_err != OGMA_ERR_OK) {
        printf("ogma_set_gmac() failed with error status %d\n", ogma_err);
        goto out;
    }
*/
    ogma_err =
        ogma_start_gmac(1, 1);
    if (ogma_err != OGMA_ERR_OK) {
        printf("ogma_start_gmac() failed with error status %d\n", ogma_err);
        goto out;
    }

	//dump_taiki_reg();

out:
    return ogma_err;

}

#define LINKUP_WAIT_TIMEOUT 5
/**
 * Check PHY link status and configure F_GMAC4MT if necessary.
 */
static int ogma_netdev_phy_poll(void)
{

    unsigned int link_status_flag;
    unsigned int auto_nego_complete_flag;
    unsigned int latched_link_down_flag;
    unsigned int link_speed;
    unsigned half_duplex_flag;

    unsigned long ogma_err;
    int timeout;

    for ( timeout = 0; timeout < LINKUP_WAIT_TIMEOUT; timeout++) {
        ogma_netdev_get_phy_link_status(
            &link_status_flag,
            &auto_nego_complete_flag,
            &latched_link_down_flag,
            &link_speed,
            &half_duplex_flag);

        if ( (link_status_flag != 0) && (auto_nego_complete_flag != 0) ) {
            break;
        }
        mdelay(1000);
    }

	if ( (link_status_flag == 0) || (auto_nego_complete_flag == 0) ) {
		printf("Network is linkdown\n");
		return -ETIME;
	}

	ctrl_p->gmac_mode.half_duplex_flag = half_duplex_flag;
	ctrl_p->gmac_mode.link_speed = link_speed;

#if OGMA_DEBUG
	ogma_print("link status: %i\n", link_status_flag);
	ogma_print("auto nego complete: %i\n", auto_nego_complete_flag);
	ogma_print("latched link down: %i\n", latched_link_down_flag);
	ogma_print("link speed: %i\n", link_speed);
	ogma_print("half duplex: %i\n", half_duplex_flag);
#endif

    /*
     * Configure GMAC if link is up and auto negotiation is complete.
     */
    if ((ogma_err = ogma_netdev_configure_mac(
             link_speed,
             half_duplex_flag))
        != OGMA_ERR_OK) {
        printf("ogma_netdev_configure_mac() failed");
        link_status_flag = auto_nego_complete_flag = 0;
        return -EBUSY;
    }


    /* Update saved PHY status */
//    ogma_netdev_p->prev_link_status_flag = link_status_flag;
//    ogma_netdev_p->prev_auto_nego_complete_flag = auto_nego_complete_flag;
    return 0;
}

static int stopped = 1;

static int ogma_netdev_open(void)
{
    int err = 0;

    //Tx channel int enable
    ogma_write_reg(OGMA_REG_ADDR_NRM_TX_INTEN, 1<<17);

#ifdef OGMA_DEBUG
	unsigned long value;
    value = ogma_read_reg(OGMA_REG_ADDR_NRM_TX_INTEN);
    ogma_print("Tx int enable=0x%x\n", value);
#endif

    //Tx packet count int
    ogma_write_reg(OGMA_REG_ADDR_NRM_TX_DONE_TXINT_PKTCNT, 1);


    //Rx channel int enable
    ogma_write_reg(OGMA_REG_ADDR_NRM_RX_INTEN, 1 << 15);

#ifdef OGMA_DEBUG
    value = ogma_read_reg(OGMA_REG_ADDR_NRM_RX_INTEN);
    ogma_print("Rx int enable=0x%x\n", value);
#endif

    //Rx packet count int
    ogma_write_reg(OGMA_REG_ADDR_NRM_RX_RXINT_PKTCNT, 1);


#ifdef OGMA_DEBUG
    //Top interrent enable status
    value = ogma_read_reg(OGMA_REG_ADDR_TOP_INTEN);
    ogma_print("top status int enable 1=0x%x\n",value);
#endif

    ogma_write_reg(OGMA_REG_ADDR_TOP_INTEN_SET, 3);

#ifdef OGMA_DEBUG
    value = ogma_read_reg(OGMA_REG_ADDR_TOP_INTEN);
    ogma_print("top status int enable 2=0x%x\n", value);
#endif

//    ogma_write_reg(OGMA_REG_ADDR_TOP_STATUS, 3);


    if ( ( err = ogma_netdev_phy_poll() ) == 0) {
    
        ogma_print("f_taiki initialized\n");
    } else {
        ogma_print("f_taiki initialized error.\n");
    }

	stopped = 0;

    return err;
}

#define WAIT_FW_RDY_TIMEOUT 50

static int ogma_init(struct eth_device *netdev, bd_t *bd)
{
    unsigned int ogma_err = 1;
    unsigned long value;
	int timeout;

    ctrl_p = (ogma_ctrl_t *)malloc(sizeof(ogma_ctrl_t));

    ogma_write_reg( OGMA_REG_ADDR_CLK_EN,
                    OGMA_CLK_EN_REG_DOM_G | OGMA_CLK_EN_REG_DOM_C | OGMA_CLK_EN_REG_DOM_D);

	/* save the descriptor start address for scb */
	scb_set_normal_tx_phys_addr = ogma_read_reg(OGMA_REG_ADDR_NRM_TX_DESC_START);

	/* setup descriptor start address */
	ogma_write_reg(OGMA_REG_ADDR_NRM_TX_DESC_START, OGMA_TX_DESC_BASE);
	ogma_write_reg(OGMA_REG_ADDR_NRM_RX_DESC_START, OGMA_RX_DESC_BASE);

    /* set pkt desc ring config */
	value =
        ( 0x0UL /*ctrl_p->normal_mode_hwconf.rx_tmr_mode_flag*/ <<
          OGMA_REG_DESC_RING_CONFIG_TMR_MODE) |
        ( 0x1UL /*ctrl_p->normal_mode_hwconf.rx_little_endian_flag*/ <<
          OGMA_REG_DESC_RING_CONFIG_DAT_ENDIAN) |
        ( 0x1UL << OGMA_REG_DESC_RING_CONFIG_CFG_UP) |
        ( 0x1UL <<OGMA_REG_DESC_RING_CONFIG_CH_RST);

    ogma_write_reg(OGMA_REG_ADDR_NRM_TX_CONFIG, value);
    ogma_write_reg(OGMA_REG_ADDR_NRM_RX_CONFIG, value);

	ogma_print("checking CH_RST\n");

	timeout = WAIT_FW_RDY_TIMEOUT;

	/*
     * Waits until TX CH_RST bit is cleared.
     */
    while ( timeout-- && ( ogma_read_reg(OGMA_REG_ADDR_NRM_TX_CONFIG)
              & ( 0x1UL <<OGMA_REG_DESC_RING_CONFIG_CFG_UP) ) != 0) {
        udelay(1000);
    }

	if(timeout < 0) {
		ogma_err = -ETIME;
		goto err; 
	}

	timeout = WAIT_FW_RDY_TIMEOUT;

    /*
     * Waits until RX CH_RST bit is cleared.
     */
    while ( timeout-- && ( ogma_read_reg(OGMA_REG_ADDR_NRM_RX_CONFIG)
              & ( 0x1UL <<OGMA_REG_DESC_RING_CONFIG_CFG_UP) ) != 0) {
        udelay(1000);
    }

	if(timeout < 0) {
		ogma_err = -ETIME;
		goto err; 
	}

	ogma_print("TX and RX config is set\n");

	/* read and save pkt ctrl register for scb */
	scb_set_pkt_ctrl_reg_value = ogma_read_reg(OGMA_REG_ADDR_PKT_CTRL);

	/* set PKT_CTRL */
    value = ogma_calc_pkt_ctrl_reg_param();

	value |= OGMA_PKT_CTRL_REG_MODE_NRM;

	/* change to noromal mode */
    ogma_write_reg( OGMA_REG_ADDR_DMA_MH_CTRL,
                    OGMA_DMA_MH_CTRL_REG_MODE_TRANS);

    ogma_write_reg( OGMA_REG_ADDR_PKT_CTRL,
                    value);

	/* Clear TX/RX descriptor ring memory space */
	memset((void *)OGMA_TX_DESC_BASE, 0, 0x100000);
	memset((void *)OGMA_RX_DESC_BASE, 0, 0x100000);

	int idx = 0;
	/* Allocate Rx descriptor */
	for (idx=0; idx<RX_DESC_NUM; idx++)
	{
		value = ( 1UL << OGMA_RX_PKT_DESC_RING_OWN_FIELD) |
        	( 1UL << OGMA_RX_PKT_DESC_RING_FS_FIELD) |
        	( 1UL << OGMA_RX_PKT_DESC_RING_LS_FIELD) ; /* OWN = FS = LS = 1 */


    	if ( idx == (RX_DESC_NUM-1) ) {
        	value |= ( 0x1U << OGMA_RX_PKT_DESC_RING_LD_FIELD); /* LD = 1 */
    	}

		/* fill descriptor*/
		*((unsigned int*)(OGMA_RX_DESC_BASE + idx*16)) = value;
		*((unsigned int*)(OGMA_RX_DESC_BASE + idx*16 + 4)) = OGMA_RX_BUF_ADDR + idx*2048;//2k
		*((unsigned int*)(OGMA_RX_DESC_BASE + idx*16 + 8)) = OGMA_RX_PKT_BUF_LEN;

		//if ( idx == (RX_DESC_NUM-1) ) {
		//	*((unsigned int*)(OGMA_RX_DESC_BASE + idx*16 + 4)) = OGMA_RX_BUF_ADDR;//2k
	}

	udelay(2000);

	return ogma_netdev_open();

err:
	return ogma_err;
}

static void change_to_taiki_mode(void)
{
	u32 value;

	/* set normal tx desc ring start addr */
	ogma_write_reg(OGMA_REG_ADDR_NRM_TX_DESC_START, scb_set_normal_tx_phys_addr);

	/* reset normal tx desc ring */
    value =
        ( 0x1UL << OGMA_REG_DESC_RING_CONFIG_CFG_UP) |
        ( 0x1UL <<OGMA_REG_DESC_RING_CONFIG_CH_RST);

    ogma_write_reg(OGMA_REG_ADDR_NRM_TX_CONFIG, value);

    /*
     * Waits until TX CH_RST bit is cleared.
     */
    while ( ( ogma_read_reg( OGMA_REG_ADDR_NRM_TX_CONFIG)
              & ( 0x1UL <<OGMA_REG_DESC_RING_CONFIG_CFG_UP) ) != 0) {
        ;
    }

	/* chande to taiki mode */
    ogma_write_reg( OGMA_REG_ADDR_DMA_MH_CTRL,
                    OGMA_DMA_MH_CTRL_REG_MODE_TRANS);

    ogma_write_reg( OGMA_REG_ADDR_PKT_CTRL,
                    scb_set_pkt_ctrl_reg_value);

	/* Wait Change mode to Taiki Complete */
    udelay(2000);

    /* Clear mode change complete IRQ */
    ogma_write_reg(OGMA_REG_ADDR_MODE_TRANS_COMP_STATUS, 
        (OGMA_MODE_TRANS_COMP_IRQ_T2N | OGMA_MODE_TRANS_COMP_IRQ_N2T));

}

static void ogma_halt(struct eth_device *netdev)
{
	if(stopped == 1)
		return;

	ogma_stop_gmac(1, 1);

	change_to_taiki_mode();

	ogma_write_reg(OGMA_REG_ADDR_TOP_STATUS, OGMA_TOP_IRQ_REG_NRM_RX | OGMA_TOP_IRQ_REG_NRM_TX);

	/* Reset PHY device. */
    ogma_set_phy_reg(phy_dev_addr, /* phy_addr */
                      0, /* reg_addr */
                      (ogma_get_phy_reg(phy_dev_addr,0) | (1UL << 15)));
    
    /* Wait for reset to finish. */
    while(((ogma_get_phy_reg(phy_dev_addr,0)) & (1UL << 15)) != 0) {
        ;
    }


	/* disable core */
	//ogma_write_reg(OGMA_REG_ADDR_DIS_CORE, 1);

	/* disable clk */
	ogma_write_reg(OGMA_REG_ADDR_CLK_EN, 0);

	rx_count = 0;
	stopped = 1;

	return;
}

static int ogma_send(struct eth_device *netdev, void *packet,
							int length)
{
	unsigned int value;
	unsigned int attr;
	static unsigned int tx_idx = 0;

#ifdef OGMA_DEBUG
	ogma_print("ogma_send(), packet=0x%x, length=%d\n",packet,length);

	value = ogma_read_reg(OGMA_REG_ADDR_TOP_STATUS);
	ogma_print("Top status=0x%x\n", value);
	value = ogma_read_reg(OGMA_REG_ADDR_NRM_TX_STATUS);
	ogma_print("Tx status=0x%x\n", value);//Check bit 6,7. It should be 00
#endif

	attr = ( 1UL << OGMA_TX_PKT_DESC_RING_OWN_FIELD) |
        ( 1 << OGMA_TX_PKT_DESC_RING_LD_FIELD) | //Last Descriptor
        ( 0 << OGMA_TX_PKT_DESC_RING_DRID_FIELD) | //Descriptor ring id
        ( 1U << OGMA_TX_PKT_DESC_RING_PT_FIELD) |
        ( OGMA_DESC_RING_ID_GMAC << OGMA_TX_PKT_DESC_RING_TDRID_FIELD) | //Target descriptor ring id
        ( 1/*first_flag*/ << OGMA_TX_PKT_DESC_RING_FS_FIELD) | //Assume only one segement
        ( 1/*last_flag*/ << OGMA_TX_PKT_DESC_RING_LS_FIELD) | //Assume only one segement
        ( 0/*tx_pkt_ctrl_p->cksum_offload_flag*/ <<
          OGMA_TX_PKT_DESC_RING_CO_FIELD) |
        ( 0/*tx_pkt_ctrl_p->tcp_seg_offload_flag*/ << OGMA_TX_PKT_DESC_RING_SO_FIELD) |
        ( 1U << OGMA_TX_PKT_DESC_RING_TRS_FIELD);

	//printf("tx_idx=%d\n",tx_idx);
	*((unsigned int*)(OGMA_TX_DESC_BASE + tx_idx*16)) = attr;
	*((unsigned int*)(OGMA_TX_DESC_BASE + tx_idx*16 + 4)) = (unsigned int)packet;
	*((unsigned int*)(OGMA_TX_DESC_BASE + tx_idx*16 + 8)) = length;
	
	/* Make sure all needed operations were finished before starting trasmit */
	flush_dcache_range((unsigned long)packet, (unsigned long)packet + (unsigned long)length);
	dmb();

	ogma_write_reg(OGMA_REG_ADDR_NRM_TX_PKTCNT, 1);

	do {
		value = ogma_read_reg(OGMA_REG_ADDR_TOP_STATUS);
		udelay(50);
	} while (!(value & 0x1));

#ifdef OGMA_DEBUG
	value = ogma_read_reg(OGMA_REG_ADDR_NRM_TX_DONE_PKTCNT);
	ogma_print("Tx done=0x%x\n",value);
	value = ogma_read_reg(OGMA_REG_ADDR_NRM_TX_STATUS);
	ogma_print("Tx status=0x%x\n",value);//Check bit 6,7. It should be 00
#endif

	//Clear
	value = (OGMA_CH_IRQ_REG_EMPTY & ( OGMA_CH_IRQ_REG_EMPTY | OGMA_CH_IRQ_REG_ERR));
	ogma_write_reg(OGMA_REG_ADDR_NRM_TX_STATUS, value);

#ifdef OGMA_DEBUG
	ogma_print("ogma_send done\n");
#endif

	return 0;
}


static int ogma_rx(struct eth_device *netdev)
{
	u32 i, rx_num;
	u32 value, len, top_status;
	unsigned char * buf;

	top_status = ogma_read_reg(OGMA_REG_ADDR_TOP_STATUS);

	if (top_status & 0x2)
	{
		rx_num = ogma_read_reg(OGMA_REG_ADDR_NRM_RX_PKTCNT);
		ogma_print("rx_num=%d\n",rx_num);
		if (rx_num > 0)
		{
			for (i=0; i<rx_num; i++)
			{
				value = *((unsigned int*)(OGMA_RX_DESC_BASE + rx_count*16));// attr
				buf = (unsigned char *)*((unsigned int*)(OGMA_RX_DESC_BASE + rx_count*16 + 4));// buf
				len = *((unsigned int*)(OGMA_RX_DESC_BASE + rx_count*16 + 8)) >> 16 ;// len

				ogma_print("info=0x%x\n",value);
				ogma_print("buf=0x%x\n",buf);
				ogma_print("rlen=%d\n",len);
				NetReceive(buf, len);

				value = ( 1UL << OGMA_RX_PKT_DESC_RING_OWN_FIELD) |
        			( 1UL << OGMA_RX_PKT_DESC_RING_FS_FIELD) |
        			( 1UL << OGMA_RX_PKT_DESC_RING_LS_FIELD) ; /* OWN = FS = LS = 1 */

				if ( rx_count == (RX_DESC_NUM-1) )
        			value |= ( 0x1U << OGMA_RX_PKT_DESC_RING_LD_FIELD); /* LD = 1 */

				*((unsigned int*)(OGMA_RX_DESC_BASE + rx_count*16)) = value;
				*((unsigned int*)(OGMA_RX_DESC_BASE + rx_count*16 + 4)) = OGMA_RX_BUF_ADDR + rx_count*2048;//2k
				*((unsigned int*)(OGMA_RX_DESC_BASE + rx_count*16 + 8)) = OGMA_RX_PKT_BUF_LEN;

				rx_count++;
				//printf("rx_count=%d\n",rx_count);
				if (rx_count == RX_DESC_NUM)
					rx_count = 0;

			}

		}
		ogma_write_reg(OGMA_REG_ADDR_NRM_RX_STATUS, 1 << 15);
		//ogma_write_reg(OGMA_REG_ADDR_TOP_STATUS, 0x3);

		ogma_write_reg(OGMA_REG_ADDR_NRM_RX_RXINT_PKTCNT, 1);

		ogma_write_reg(OGMA_REG_ADDR_NRM_RX_INTEN, 1 << 15);
	}

	return 0;
}

int ogma_initialize(bd_t *bis)
{
	int ret;
	struct eth_device *dev = &(ogma_info.netdev);

	dev->init = ogma_init;
	dev->halt = ogma_halt;
	dev->send = ogma_send;
	dev->recv = ogma_rx;
	dev->write_hwaddr = ogma_write_hwaddr;
	sprintf(dev->name, "f_taiki");

	ret  = eth_register(dev);
	return ret;
}

#ifdef OGMA_DEBUG
void dump_taiki_reg(void)
{
	printf("OGMA_REG_ADDR_TOP_STATUS=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_TOP_STATUS));
	printf("OGMA_REG_ADDR_TOP_INTEN=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_TOP_INTEN));
	printf("OGMA_REG_ADDR_TOP_INTEN_SET=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_TOP_INTEN_SET));
	printf("OGMA_REG_ADDR_TOP_INTEN_CLR=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_TOP_INTEN_CLR));
	printf("OGMA_REG_ADDR_NRM_TX_STATUS=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_STATUS));
	printf("OGMA_REG_ADDR_NRM_TX_INTEN=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_INTEN));
	printf("OGMA_REG_ADDR_NRM_TX_INTEN_SET=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_INTEN_SET));
	printf("OGMA_REG_ADDR_NRM_TX_INTEN_CLR=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_INTEN_CLR));
	printf("OGMA_REG_ADDR_NRM_TX_INTEN_CLR=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_INTEN_CLR));
	printf("OGMA_REG_ADDR_NRM_RX_STATUS=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_STATUS));
	printf("OGMA_REG_ADDR_NRM_RX_INTEN=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_INTEN));
	printf("OGMA_REG_ADDR_NRM_RX_INTEN_SET=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_INTEN_SET));
	printf("OGMA_REG_ADDR_NRM_RX_INTEN_CLR=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_INTEN_CLR));
	printf("OGMA_REG_ADDR_PKTC_CMD_BUF=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_PKTC_CMD_BUF));
	printf("OGMA_REG_ADDR_DMAC_HM_CMD_BUF=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_DMAC_HM_CMD_BUF));
	printf("OGMA_REG_ADDR_DMAC_MH_CMD_BUF=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_DMAC_MH_CMD_BUF));
	printf("OGMA_REG_ADDR_DIS_CORE=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_DIS_CORE));
	printf("OGMA_REG_ADDR_CLK_EN=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_CLK_EN));
	printf("OGMA_REG_ADDR_SOFT_RST=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_SOFT_RST));
	printf("OGMA_REG_ADDR_PKT_CTRL=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_PKT_CTRL));
	printf("OGMA_REG_ADDR_COM_INIT=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_COM_INIT));
	printf("OGMA_REG_ADDR_DMA_TMR_CTRL=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_DMA_TMR_CTRL));
	printf("OGMA_REG_ADDR_F_TAIKI_MC_VER=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_F_TAIKI_MC_VER));
	printf("OGMA_REG_ADDR_F_TAIKI_VER=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_F_TAIKI_VER));
	printf("OGMA_REG_ADDR_DMA_HM_CTRL=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_DMA_HM_CTRL));
	printf("OGMA_REG_ADDR_DMA_MH_CTRL=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_DMA_MH_CTRL));
	printf("OGMA_REG_ADDR_NRM_TX_PKTCNT=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_PKTCNT));
	printf("OGMA_REG_ADDR_NRM_TX_DONE_TXINT_PKTCNT=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_DONE_TXINT_PKTCNT));
	printf("OGMA_REG_ADDR_NRM_RX_RXINT_PKTCNT=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_RXINT_PKTCNT));
	printf("OGMA_REG_ADDR_NRM_TX_TXINT_TMR=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_TXINT_TMR));
	printf("OGMA_REG_ADDR_NRM_RX_RXINT_TMR=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_RXINT_TMR));
	printf("OGMA_REG_ADDR_NRM_TX_DONE_PKTCNT=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_DONE_PKTCNT));
	printf("OGMA_REG_ADDR_NRM_RX_PKTCNT=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_PKTCNT));
	printf("OGMA_REG_ADDR_NRM_TX_TMR=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_TMR));
	printf("OGMA_REG_ADDR_NRM_RX_TMR=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_TMR));
	printf("OGMA_REG_ADDR_NRM_TX_DESC_START=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_DESC_START));
	printf("OGMA_REG_ADDR_NRM_RX_DESC_START=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_DESC_START));
	printf("OGMA_REG_ADDR_RESERVED_RX_DESC_START=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_RESERVED_RX_DESC_START));
	printf("OGMA_REG_ADDR_RESERVED_TX_DESC_START=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_RESERVED_TX_DESC_START));
	printf("OGMA_REG_ADDR_NRM_TX_CONFIG=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_TX_CONFIG));
	printf("OGMA_REG_ADDR_NRM_RX_CONFIG=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_NRM_RX_CONFIG));
	printf("OGMA_REG_ADDR_MAC_DATA=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_MAC_DATA));
	printf("OGMA_REG_ADDR_MAC_CMD=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_MAC_CMD));
	printf("OGMA_REG_ADDR_MAC_FLOW_TH=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_MAC_FLOW_TH));
	printf("OGMA_REG_ADDR_MAC_INTF_SEL=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_MAC_INTF_SEL));
	printf("OGMA_REG_ADDR_MAC_REG_BASE=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_MAC_REG_BASE));
	printf("OGMA_REG_ADDR_MAC_DESC_INIT=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_MAC_DESC_INIT));
	printf("OGMA_REG_ADDR_MAC_DESC_SOFT_RST=0x%x\n", ogma_read_reg(OGMA_REG_ADDR_MAC_DESC_SOFT_RST));
}
#endif /* OGMA_DEBUG */
