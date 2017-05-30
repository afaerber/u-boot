/**
 * ogma_itnernal.h
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
 *
 */
#ifndef OGMA_INTERNAL_H
#define OGMA_INTERNAL_H

#include "ogma_api.h"


/* Just a million to prevent typing errors. */
#define OGMA_CLK_MHZ (1000000)

#define OGMA_INSTANCE_NUM_MAX 1

#define OGMA_RX_PKT_BUF_LEN 1522
#define OGMA_RX_JUMBO_PKT_BUF_LEN 9022
#define OGMA_DUMMY_DESC_ENTRY_LEN 48

/*
extern const unsigned long desc_ring_irq_inten_reg_addr[OGMA_DESC_RING_ID_MAX + 1];
extern const unsigned long desc_ring_irq_inten_set_reg_addr[OGMA_DESC_RING_ID_MAX + 1];
extern const unsigned long desc_ring_irq_inten_clr_reg_addr[OGMA_DESC_RING_ID_MAX + 1];
*/

typedef struct ogma_ctrl_s ogma_ctrl_t;
typedef struct ogma_desc_ring_s ogma_desc_ring_t;
typedef struct ogma_clk_ctrl_s ogma_clk_ctrl_t;
typedef union ogma_desc_entry_priv_u ogma_desc_entry_priv_t;

struct ogma_clk_ctrl_s{
    u32 dmac_req_num;
    u32 core_req_num;
    u8 mac_req_num;
};

/*
struct ogma_desc_ring_s{

    ogma_desc_ring_id_t ring_id;

    ogma_desc_ring_param_t param;

    unsigned int rx_desc_ring_flag:1;

    unsigned int tx_desc_ring_flag:1;

    unsigned int running_flag:1;

    unsigned int full_flag:1;

    unsigned char desc_entry_len;

    unsigned short head_idx;

    unsigned short tail_idx;

    unsigned short rx_num;

    unsigned short tx_done_num;

    void *desc_ring_cpu_addr;

    dma_addr_t desc_ring_phys_addr;

    ogma_frag_info_t *frag_info_p;

    ogma_desc_entry_priv_t *priv_data_p;
};
*/
struct ogma_ctrl_s{
    ogma_ctrl_t *next_p;

    unsigned int core_enabled_flag:1;

    unsigned int gmac_rx_running_flag:1;

    unsigned int gmac_tx_running_flag:1;

    unsigned int gmac_mode_valid_flag:1;

    ogma_param_t param;

    ogma_clk_ctrl_t clk_ctrl;

    unsigned long rx_pkt_buf_len;

//    ogma_desc_ring_t desc_ring[OGMA_DESC_RING_ID_MAX+1];


    ogma_gmac_mode_t gmac_mode;

    void *dummy_desc_entry_addr;

    dma_addr_t dummy_desc_entry_phys_addr;

};

#endif /* OGMA_INTERNAL_H */
