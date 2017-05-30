/*
 *  u-boot/board/fujitsu/mb86s73/mb86s73.c
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
#include <netdev.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/hardware.h>
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
	printf("BOARD: Socionext MB86S73 EVB\n");

	return 0;
}
#endif	/* CONFIG_DISPLAY_BOARDINFO */
