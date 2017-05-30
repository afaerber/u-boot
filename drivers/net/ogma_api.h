/**
 * ogma_api.h
 *
 *  Copyright (c) 2011 - 2013 Fujitsu Semiconductor Limited.
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

#ifndef OGMA_API_H
#define OGMA_API_H

#include <asm/types.h>
#include "ogma_config.h"

/**
 * Check configuration macro settings.
 */

#ifdef OGMA_CONFIG_CLK_HZ
#if ( (OGMA_CONFIG_CLK_HZ < 0x200000) || (OGMA_CONFIG_CLK_HZ > 0x10000000) )
#error "OGMA_CONFIG_CLK_HZ is not appropriate."
#endif /* ( (OGMA_CONFIG_CLK_HZ < 0x200000) || (OGMA_CONFIG_CLK_HZ > 0x10000000) ) */
#else /* ! OGMA_CONFIG_CLK_HZ */
#error "OGMA_CONFIG_CLK_HZ is not given."
#endif /* OGMA_CONFIG_CLK_HZ */

#ifndef OGMA_CONFIG_GMAC_CLK_HZ
#define OGMA_CONFIG_GMAC_CLK_HZ OGMA_CONFIG_CLK_HZ
#endif

/**
 * Number of Common Descriptor ring id
 */
#define OGMA_DESC_RING_ID_NRM_TX      0
#define OGMA_DESC_RING_ID_NRM_RX      1
#define OGMA_DESC_RING_ID_RESERVED_RX 2
#define OGMA_DESC_RING_ID_RESERVED_TX 3
#define OGMA_DESC_RING_ID_GMAC   15
#define OGMA_DESC_RING_ID_MAX     3


/**
 * Numbre of TCP Segmentation length limits
 */
#define OGMA_TCP_SEG_LEN_MAX 1460
#define OGMA_TCP_JUMBO_SEG_LEN_MAX 8960
#define OGMA_TCP_SEG_LEN_MIN  536

/**
 * Number of checksum calculation result for received packet
 */
#define OGMA_RX_CKSUM_RESULT_OK       0x1
#define OGMA_RX_CKSUM_RESULT_NG       0x2
#define OGMA_RX_CKSUM_RESULT_NOTAVAIL 0x0

/**
 * Number of top interrupt enable register bit field
 */
#define OGMA_TOP_IRQ_REG_CODE_LOAD_END   (1UL << 20)
#define OGMA_TOP_IRQ_REG_NRM_RX	         (1UL <<  1)
#define OGMA_TOP_IRQ_REG_NRM_TX	         (1UL <<  0)


/**
 *  Number of top channel enable register bit field
 */
#define OGMA_CH_IRQ_REG_EMPTY   (1UL << 17)
#define OGMA_CH_IRQ_REG_ERR     (1UL << 16)
#define OGMA_CH_IRQ_REG_PKT_CNT (1UL << 15)
#define OGMA_CH_IRQ_REG_TIMEUP  (1UL << 14)
#define OGMA_CH_IRQ_REG_RCV     (OGMA_CH_IRQ_REG_PKT_CNT | OGMA_CH_IRQ_REG_TIMEUP)

/**
 *  Number of top channel enable register bit field for F_TAIKI
 */
#define OGMA_CH_IRQ_REG_TX_DONE (1UL << 15)
#define OGMA_CH_IRQ_REG_SND     (OGMA_CH_IRQ_REG_TX_DONE | OGMA_CH_IRQ_REG_TIMEUP)

/**
 *  Number of mode trans comp irq enable register bit field
 */
#define OGMA_MODE_TRANS_COMP_IRQ_N2T (1UL << 20)
#define OGMA_MODE_TRANS_COMP_IRQ_T2N (1UL << 19)

/**
 * Number of various limits
 */
#define OGMA_DESC_ENTRY_NUM_MIN        2 
#define OGMA_DESC_ENTRY_NUM_MAX        2047
#define OGMA_INT_PKTCNT_MAX            2047


/**
 * Number of ogma phy interface setting
 */
#define OGMA_PHY_INTERFACE_GMII  0
#define OGMA_PHY_INTERFACE_RGMII 1
#define OGMA_PHY_INTERFACE_RMII  4

/**
 * Number of ogma link speed setting
 */
#define OGMA_PHY_LINK_SPEED_1G   0
#define OGMA_PHY_LINK_SPEED_100M 1
#define OGMA_PHY_LINK_SPEED_10M  2


/**
 * Number of flow control limits
 */
#define OGMA_FLOW_CTRL_START_THRESHOLD_MAX 383U
#define OGMA_FLOW_CTRL_STOP_THRESHOLD_MAX  383U
#define OGMA_FLOW_CTRL_PAUSE_TIME_MIN      5

#define OGMA_TX_PKT_DESC_RING_OWN_FIELD        (31)
#define OGMA_TX_PKT_DESC_RING_LD_FIELD         (30)
#define OGMA_TX_PKT_DESC_RING_DRID_FIELD       (24)
#define OGMA_TX_PKT_DESC_RING_PT_FIELD         (21)
#define OGMA_TX_PKT_DESC_RING_TDRID_FIELD      (16)
#define OGMA_TX_PKT_DESC_RING_CC_FIELD         (15)
#define OGMA_TX_PKT_DESC_RING_FS_FIELD         (9)
#define OGMA_TX_PKT_DESC_RING_LS_FIELD         (8)
#define OGMA_TX_PKT_DESC_RING_CO_FIELD         (7)
#define OGMA_TX_PKT_DESC_RING_SO_FIELD         (6)
#define OGMA_TX_PKT_DESC_RING_TRS_FIELD        (4)

#define OGMA_RX_PKT_DESC_RING_OWN_FIELD        (31)
#define OGMA_RX_PKT_DESC_RING_LD_FIELD         (30)
#define OGMA_RX_PKT_DESC_RING_SDRID_FIELD      (24)
#define OGMA_RX_PKT_DESC_RING_FR_FIELD         (23)
#define OGMA_RX_PKT_DESC_RING_ER_FIELD         (21)
#define OGMA_RX_PKT_DESC_RING_ERROR_CODE_FIELD (16)
#define OGMA_RX_PKT_DESC_RING_TDRID_FIELD      (12)
#define OGMA_RX_PKT_DESC_RING_FS_FIELD         (9)
#define OGMA_RX_PKT_DESC_RING_LS_FIELD         (8)
#define OGMA_RX_PKT_DESC_RING_CO_FIELD         (6)

#define OGMA_RX_PKT_DESC_RING_ERROR_CODE_FIELD_MASK (0x3)

#define OGMA_MAX_TX_PKT_LEN       1518U
#define OGMA_MAX_TX_JUMBO_PKT_LEN 9018U

#define OGMA_CLK_EN_REG_DOM_ALL 0x3f

enum ogma_err_e{
    OGMA_ERR_OK = 0,
    OGMA_ERR_PARAM,
    OGMA_ERR_ALLOC,
    OGMA_ERR_BUSY,
    OGMA_ERR_RANGE,
    OGMA_ERR_DATA,
    OGMA_ERR_NOTAVAIL,
    OGMA_ERR_INTERRUPT,
    OGMA_ERR_AGAIN,
    OGMA_ERR_INVALID
};

typedef void *ogma_handle_t;
typedef struct ogma_param_s ogma_param_t;
typedef struct ogma_pkt_ctrl_param_s ogma_pkt_ctrl_param_t;
typedef struct ogma_desc_ring_param_s ogma_desc_ring_param_t;
typedef enum ogma_err_e ogma_err_t;
typedef unsigned char ogma_desc_ring_id_t;
typedef struct ogma_tx_pkt_ctrl_s ogma_tx_pkt_ctrl_t;
typedef struct ogma_rx_pkt_info_s ogma_rx_pkt_info_t;
typedef struct ogma_frag_info_s ogma_frag_info_t;
typedef struct ogma_gmac_config_s ogma_gmac_config_t;
typedef struct ogma_gmac_mode_s ogma_gmac_mode_t;

struct ogma_gmac_config_s{
    unsigned char phy_interface;
};

struct ogma_pkt_ctrl_param_s{
    unsigned int log_chksum_er_flag:1;
    unsigned int log_hd_imcomplete_flag:1;
    unsigned int log_hd_er_flag:1;
};

struct ogma_desc_ring_param_s{
    unsigned int valid_flag:1;
    unsigned int little_endian_flag:1;
    unsigned int tmr_mode_flag:1;
    unsigned short entry_num;
};

struct ogma_param_s{
    unsigned int use_gmac_flag:1;
    unsigned int use_jumbo_pkt_flag:1;
    ogma_pkt_ctrl_param_t pkt_ctrl_param;
    ogma_desc_ring_param_t desc_ring_param[OGMA_DESC_RING_ID_MAX+1];
    ogma_gmac_config_t gmac_config;
};

struct ogma_tx_pkt_ctrl_s{
    unsigned int cksum_offload_flag:1;
    unsigned int tcp_seg_offload_flag:1;
    ogma_desc_ring_id_t target_desc_ring_id;
    unsigned short tcp_seg_len;
};


struct ogma_rx_pkt_info_s{
    unsigned int fragmented_flag:1;
    unsigned int err_flag:1;
    unsigned int rx_cksum_result:2;
    unsigned char err_code;
};


struct ogma_frag_info_s{
    unsigned long phys_addr;
    void *addr;
    unsigned long len;
};

struct ogma_gmac_mode_s{
    unsigned int half_duplex_flag:1;
    unsigned int flow_ctrl_enable_flag:1;
    u8 link_speed;
    u16 flow_ctrl_start_threshold;
    u16 flow_ctrl_stop_threshold;
    u16 pause_time;
};


/**************************
***************************
***************************/

ogma_err_t ogma_start_gmac (
    int rx_flag,
    int tx_flag
    );

ogma_err_t ogma_stop_gmac (
    int rx_flag,
    int tx_flag
    );

ogma_err_t ogma_set_gmac_mode (
    const ogma_gmac_mode_t *gmac_mode_p
    );

void ogma_set_phy_reg (
    u8 phy_addr,
    u8 reg_addr,
    u16 value
    );
    
u16 ogma_get_phy_reg (
    u8 phy_addr,
    u8 reg_addr
    );

/*
unsigned long ogma_get_desc_ring_irq_enable (
    ogma_desc_ring_id_t ring_id
    );

unsigned long ogma_get_desc_ring_irq_status_non_clear (
    ogma_desc_ring_id_t ring_id,
    int mask_flag
    );

ogma_err_t ogma_clear_desc_ring_irq_status (
    ogma_desc_ring_id_t ring_id,
    unsigned long value
    );
*/

unsigned long ogma_get_hw_ver (void
    );

unsigned long ogma_get_mcr_ver (void
    );

#endif /* OGMA_API_H*/
