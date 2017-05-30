/*
 *  u-boot/board/fujitsu/msb7701/msb7701.c
 *
 * Copyright (C) 2016 Fujitsu Electronics Inc.
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
#include <netdev.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/hardware.h>
#include <fdt.h>
#include <libfdt.h>
#include <fdt_support.h>
#include "mhu.h"

extern void lcd_ctrl_init(void *lcd_base);
DECLARE_GLOBAL_DATA_PTR;

#define LOAD_OFFSET 0x100

/*
 * Miscellaneous platform dependent initialisations
 */
int board_init(void)
{
	gd->bd->bi_arch_number = MACH_TYPE_MB8AC0300;
	gd->bd->bi_boot_params = CONFIG_SYS_LOAD_ADDR + LOAD_OFFSET;

	gd->flags = 0; /* check me */

/*	icache_enable();  */

	return 0;
}

/*
 * DRAM configuration
 */
int dram_init(void)
{
#ifdef CONFIG_DRIVER_FGMAC4
	/* we use upper 1MB non-cached region for fgmac4's tx/rx buffers */
	gd->ram_size = PHYS_SDRAM_SIZE - CONFIG_FGMAC4_BUF_SIZE;
#else
	gd->ram_size = PHYS_SDRAM_SIZE;
#endif
	return 0;
}

#ifdef CONFIG_MB86S7X_MHU
void dram_init_banksize(void)
{
	int i;
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
		gd->bd->bi_dram[i].start_high = 0;
		gd->bd->bi_dram[i].start =      0;
		gd->bd->bi_dram[i].size_high =  0;
		gd->bd->bi_dram[i].size =       0;
	}

	if (get_memory_layout()) {
		gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
		gd->bd->bi_dram[0].size =  gd->ram_size;
	}
}
#endif

#ifdef CONFIG_DISPLAY_BOARDINFO
/**
 * Print board information
 */
int checkboard(void)
{
	printf("BOARD: Fujitsu Electronics MSB7701 Board\n");

	return 0;
}
#endif	/* CONFIG_DISPLAY_BOARDINFO */

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
void fdt_status_update(void * blob, char * node_name, char * status)
{
	char * status_str = NULL;
	int nodeoffset;

	nodeoffset = fdt_path_offset(blob, node_name);
	status_str = (char *)fdt_getprop(blob, nodeoffset, "status", NULL);
	if (status_str == NULL || !strcmp(status_str, "auto")) {
		fdt_setprop_string(blob, nodeoffset, "status", status);
	}

	return;
}

int board_late_init(void)
{
	char *brsel_str;
	u32 brsel;

	/* bootmode from flash */
	brsel_str = getenv("brsel");
	if (brsel_str == NULL) {
		brsel_str = "";	  /* first boot */
	}

	/* BRSEL: DIPSW setting */
	brsel = readl(brsel_from_scb);

	printf("BRSEL: 0x%08X\n", brsel);
	if (brsel & 0x01) {
		/* SD#0 */
		if (strcmp(brsel_str, "sd") != 0) {
			printf("Recovery SD default boot settings\n");
			setenv("brsel",    "sd");
			setenv("bootcmd",  CONFIG_BOOTCOMMAND_SD);
			setenv("bootargs", CONFIG_BOOTARGS_SD);
			saveenv();
		}
	} else if (brsel & 0x10) {
		/* eMMC#0 */
		if (strcmp(brsel_str, "emmc") != 0) {
			printf("Recovery eMMC default boot settings\n");
			setenv("brsel",   "emmc");
			setenv("bootcmd",  CONFIG_BOOTCOMMAND_EMMC);
			setenv("bootargs", CONFIG_BOOTARGS_EMMC);
			saveenv();
		}
	} else {
		/* do nothing */
	}

	return 0;
}

void ft_board_setup(void *blob, bd_t *bd)
{
	char * status_str = NULL;
	char * bind_addr  = NULL;
	char * fb0_addr   = NULL;
	char * fb1_addr   = NULL;

	int nodeoffset;
	int video_out_capability = mhu_check_video_out_capability();

	nodeoffset = fdt_path_offset(blob, "/pcie1_x2len@0x33E00600");
	status_str = (char *)fdt_getprop(blob, nodeoffset, "status", NULL);
	if (status_str == NULL || !strcmp(status_str, "auto")) {
		if(mhu_check_pcie_capability()) {
			printf("      PCIe  : Auto Port Bifurcation. status -> okay\n");
			fdt_setprop_string(blob, nodeoffset, "status", "okay");
		}
		else {
			printf("      PCIe  : Auto Port Bifurcation. status -> disabled\n");
			fdt_setprop_string(blob, nodeoffset, "status", "disabled");
		}
	}

	/* Setup display switch */
	/* mhu_check_video_out_capability() = 0 : HDMI              */
	/*                                  = 1 : MIPI-DSI(default) */
	if(video_out_capability) {
		/* fdt_setprop_string() for MIPI-DSI */
		fdt_status_update(blob, "/fdb.0/mipidsi@fb0"     , "okay");
		fdt_status_update(blob, "/fdb.0/hdmitx14@fb1"    , "disabled");
		fdt_status_update(blob, "/fdb.0/iris-dsi@fb0"    , "okay");
		fdt_status_update(blob, "/fdb.0/iris-hdmi@fb1"   , "disabled");
		fdt_status_update(blob, "/fdb.0/fdbdrm.0"        , "okay");
		fdt_status_update(blob, "/fdb.0/f_hdmi_audio_dai", "disabled");
		fdt_status_update(blob, "/f_hdmi_codec"          , "disabled");
		fdt_status_update(blob, "/f_hdmi_audio"          , "disabled");
		fdt_status_update(blob, "/i2c1/sc16is750@48"     , "okay");
		fdt_status_update(blob, "/i2c1/sn65dsi84@2c"     , "okay");
		fdt_status_update(blob, "/gpio_backlight"        , "okay");
		fdt_status_update(blob, "/touch-wxga"            , "okay");
	} else {
		/* fdt_setprop_string() for HDMI */
		fdt_status_update(blob, "/fdb.0/mipidsi@fb0"     , "disabled");
		fdt_status_update(blob, "/fdb.0/hdmitx14@fb1"    , "okay");
		fdt_status_update(blob, "/fdb.0/iris-dsi@fb0"    , "disabled");
		fdt_status_update(blob, "/fdb.0/iris-hdmi@fb1"   , "okay");
		fdt_status_update(blob, "/fdb.0/fdbdrm.0"        , "okay");
		fdt_status_update(blob, "/fdb.0/f_hdmi_audio_dai", "okay");
		fdt_status_update(blob, "/f_hdmi_codec"          , "okay");
		fdt_status_update(blob, "/f_hdmi_audio"          , "okay");
		fdt_status_update(blob, "/i2c1/sc16is750@48"     , "disabled");
		fdt_status_update(blob, "/i2c1/sn65dsi84@2c"     , "disabled");
		fdt_status_update(blob, "/gpio_backlight"        , "disabled");
		fdt_status_update(blob, "/touch-wxga"            , "disabled");
	}

	nodeoffset = fdt_path_offset(blob, "/fdb.0/fdbdrm.0");
	bind_addr = (char *)fdt_getprop(blob, nodeoffset, "bind", NULL);
	if (bind_addr != NULL || !bind_addr[3]) {
		fb0_addr = (char *)fdt_getprop(blob, nodeoffset, "fb0_addr", NULL);
		fb1_addr = (char *)fdt_getprop(blob, nodeoffset, "fb1_addr", NULL);

		bind_addr[3] = video_out_capability ? fb0_addr[3] : fb1_addr[3];
		if(video_out_capability) {
			printf("      Video : Auto Port Selection. Selected MIPI-DSI Type.\n");
		} else {
			printf("      Video : Auto Port Selection. Selected HDMI Type.\n");
		}
	}
}
#endif
