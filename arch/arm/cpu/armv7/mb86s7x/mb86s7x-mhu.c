/*
 * mb86s7x MHU support
 * Copyright (C) 2014 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
 */

#include <config.h>
#include <asm/arch/hardware.h>
#include <common.h>


#ifdef CONFIG_MB86S7X_MHU

DECLARE_GLOBAL_DATA_PTR;


#ifndef MB86S7X_MHU_PHYS
#error MHU Address not specified
#endif

#include "mhu.h"
#include <asm/io.h>
#include "scb_mhu_api.h"


#define RX_CHAN 0x20
#define TX_CHAN 0x120

int mhu_send(u32 cmd)
{
	u32 val, count = 10000000;

	/* defeat any pending traffic */
	writel(readl(MB86S7X_MHU_PHYS + TX_CHAN + INTR_STAT_OFS),
				MB86S7X_MHU_PHYS + TX_CHAN + INTR_CLR_OFS);
	writel(readl(MB86S7X_MHU_PHYS + RX_CHAN + INTR_STAT_OFS),
				MB86S7X_MHU_PHYS + RX_CHAN + INTR_CLR_OFS);

	/* notify scb we're sending this command */
	writel(cmd, MB86S7X_MHU_PHYS + TX_CHAN + INTR_SET_OFS);

	/* wait for command taken */
	do {
		val = readl(MB86S7X_MHU_PHYS + TX_CHAN + INTR_STAT_OFS);
	} while (--count && val);
	if (val)
		return val;

	/* wait for reply */

	count = 10000000;
	do {
		val = readl(MB86S7X_MHU_PHYS + RX_CHAN + INTR_STAT_OFS);
	} while (--count && !val);

	writel(readl(MB86S7X_MHU_PHYS + RX_CHAN + INTR_STAT_OFS),
				MB86S7X_MHU_PHYS + RX_CHAN + INTR_CLR_OFS);

	return !val;
}

u32 get_scb_version(void)
{
	struct cmd_scb_version *cmd;
	u32 ret;

	cmd = cmd_to_scb;
	cmd->payload_size = sizeof(*cmd);
	cmd->version = 0;
	cmd->config_version = 0;

	ret = mhu_send(CMD_SCB_CAPABILITY_GET_REQ);

	if(!ret) {
		cmd = rsp_from_scb;
		return cmd->version;
	}

	return 0;
}

u32 get_sys_flash_size(void)
{
	switch(get_scb_version() & MB86S7X_SYS_FLASH_SIZE_MASK) {
		case MB86S7X_SYS_FLASH_SIZE_8MB:
			return 0x00800000;
		case MB86S7X_SYS_FLASH_SIZE_2MB:
			return 0x00200000;
		case MB86S7X_SYS_FLASH_SIZE_4MB:
		default:
			return 0x00400000;
	}
}

int set_power_state(u32 pd_index, u32 state)
{
	struct cmd_powerdomain volatile *cmd = cmd_to_scb;
		
	cmd->payload_size = sizeof(*cmd);
	cmd->powerdomain_index = pd_index;
	cmd->state = state;
	
	if (mhu_send(CMD_POWERDOMAIN_SET_REQ)) {
			//debug(" Failed to set power domain state\n");
			return -1;
	}

	return 0;
}

int get_power_state(u32 pd_index)
{
	struct cmd_powerdomain volatile *cmd = cmd_to_scb;
		
	cmd->payload_size = sizeof(*cmd);
	cmd->powerdomain_index = pd_index;
		
	if (mhu_send(CMD_POWERDOMAIN_GET_REQ)) {
			//debug(" Failed to get power domain state\n");
			return PHYS_SDRAM_SIZE;
	}
		
	cmd = rsp_from_scb;
	return cmd->state;
}

int set_clk_state(u32 cntrlr, u32 domain, u32 port, u32 en)
{
	struct cmd_periclk_control *cmd = cmd_to_scb;

	cmd->payload_size = sizeof(*cmd);
	cmd->cntrlr = cntrlr;
	cmd->domain = domain;
	cmd->port = port;
	cmd->en = en;
	
	if(!mhu_send(CMD_PERI_CLOCK_GATE_SET_REQ)) {
		cmd = rsp_from_scb;
		if(cmd->en != 1)
			return -1;
	}

	return 0;
}

int get_memory_layout(void)
{
	int m = 0;
	int ret = 0;
	struct cmd_memory_layout *cmd = cmd_to_scb;

	cmd->payload_size = sizeof(*cmd);
	ret = mhu_send(CMD_MEMORY_LAYOUT_GET_REQ); 

	if (!ret) {
		cmd = rsp_from_scb;
		for (m = 0; m < cmd->count_regions; m++) {			
#if defined(CONFIG_MB86S7X)
			gd->bd->bi_dram[m].start_high = 
				(ulong)(cmd->regions[m].start >> 32);
			gd->bd->bi_dram[m].start = 
				(ulong)(cmd->regions[m].start & 0xFFFFFFFF);
			gd->bd->bi_dram[m].size_high = 
				(ulong)(cmd->regions[m].length >> 32);
			gd->bd->bi_dram[m].size = (ulong)
				(cmd->regions[m].length & 0xFFFFFFFF);
#else
			gd->bd->bi_dram[m].start = cmd->regions[m].start;
			gd->bd->bi_dram[m].size = cmd->regions[m].length;
#endif
		}
	}

	return ret;
}

int mhu_check_pcie_capability(void)
{
	struct cmd_scb_version volatile *cmd = cmd_to_scb;

	cmd->payload_size = sizeof(*cmd);

	if (mhu_send(CMD_SCB_CAPABILITY_GET_REQ))
		return 0;
	else {
		cmd = rsp_from_scb;
		
		/* older version scb don't have capabilities field */
		if (cmd->payload_size < 12)
			return 0;
		return (cmd->capabilities[0] & S7X_SCB_CAPABILITY0_PCIE) > 0;
	}
}

int mhu_check_video_out_capability(void)
{
	struct cmd_scb_version volatile *cmd = cmd_to_scb;

	cmd->payload_size = sizeof(*cmd);

	if (mhu_send(CMD_SCB_CAPABILITY_GET_REQ))
		return 0;
	else {
		cmd = rsp_from_scb;

		/* older version scb don't have capabilities field */
		if (cmd->payload_size < 12)
			return 0;
		return (cmd->capabilities[0] & S7X_SCB_CAPABILITY0_VIDEO_OUT) > 0;
	}
}

#endif
