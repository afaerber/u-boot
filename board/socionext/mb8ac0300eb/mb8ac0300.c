/*
 *  u-boot/board/fujitsu/mb8ac0300/mb8ac0300.c
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

extern void lcd_ctrl_init(void *lcd_base);
DECLARE_GLOBAL_DATA_PTR;

static const unsigned char logo_rle[] __attribute__ ((section (".text"))) = {
#include "initlogo.h"
};

#define LOAD_OFFSET 0x100

#define SKIP_LINES 50

/*
 * Miscellaneous platform dependent initialisations
 */
int board_init(void)
{
	u16 *fb = (void *)0x50000000;
	u16 *p = &fb[SKIP_LINES * 800];
	u16 *l = (u16 *)logo_rle;
	u16 col;
	int size = ARRAY_SIZE(logo_rle) / 4;
	u16 n;

	gd->bd->bi_arch_number = MACH_TYPE_MB8AC0300;
	gd->bd->bi_boot_params = CONFIG_SYS_LOAD_ADDR + LOAD_OFFSET;

	gd->flags = 0; /* check me */

	while (size-- > 0) {
		n = *l++;
		col = *l++;

		while (n--)
			*p++ = col;
	}

	flush_dcache_range(fb, &fb[800 * 480]);

	puts("starting lcd\n");
	lcd_ctrl_init(fb);

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

#ifdef CONFIG_DISPLAY_BOARDINFO
/**
 * Print board information
 */
int checkboard(void)
{
	printf("BOARD: Fujitsu Semiconductor MB8AC0300-EVB\n\n");

	return 0;
}
#endif	/* CONFIG_DISPLAY_BOARDINFO */
