/*
 * JZ4780 common routines
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <mach/jz4780.h>
#include <mach/jz4780_dram.h>
#include <spl.h>

#ifdef CONFIG_SPL_BUILD
/* Pointer to the global data structure for SPL */
DECLARE_GLOBAL_DATA_PTR;
gd_t gdata __attribute__ ((section(".bss")));

/* NOTE: The SRAM in JZ4780 is too small and full memset() doesn't fit */
static void bzero(char *dst, u32 len)
{
	while (len--)
		*dst = 0;
}

void board_init_f(ulong dummy)
{
	struct spl_image_info spl_image;
	struct spl_boot_device bootdev = { .boot_device = BOOT_DEVICE_MMC1 };
	/* Set global data pointer */
	gd = &gdata;

	timer_init();
	pll_init();
	sdram_init();
	enable_caches();

	/* Clear the BSS */
	bzero(__bss_start, (char *)&__bss_end - __bss_start);

	gd->flags |= GD_FLG_SPL_INIT;

	spl_mmc_load_image(&spl_image, &bootdev);

	jump_to_image_no_args(&spl_image);
}

u32 spl_boot_device(void)
{
	return BOOT_DEVICE_MMC1;
}

u32 spl_boot_mode(const u32 boot_device)
{
	return MMCSD_MODE_RAW;
}
#endif /* CONFIG_SPL_BUILD */

int dram_init(void)
{
	return sdram_size(0) + sdram_size(1);
}

ulong board_get_usable_ram_top(ulong total_size)
{
	return CONFIG_SYS_SDRAM_BASE + (256 * 1024 * 1024);
}

int print_cpuinfo(void)
{
	printf("CPU:   Ingenic JZ4780\n");
	return 0;
}

/* WDT */
#define WDT_TDR		0x00
#define WDT_TCER	0x04
#define WDT_TCNT	0x08
#define WDT_TCSR	0x0C

/* Register definition */
#define WDT_TCSR_PRESCALE_BIT	3
#define WDT_TCSR_PRESCALE_MASK	(0x7 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE1	(0x0 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE4	(0x1 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE16	(0x2 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE64	(0x3 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE256	(0x4 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE1024	(0x5 << WDT_TCSR_PRESCALE_BIT)
#define WDT_TCSR_EXT_EN		BIT(2)
#define WDT_TCSR_RTC_EN		BIT(1)
#define WDT_TCSR_PCK_EN		BIT(0)

#define WDT_TCER_TCEN		BIT(0)

void _machine_restart(void)
{
	void __iomem *wdt_regs = (void __iomem *)WDT_BASE;

	mdelay(100);

	/*
	 * Select the EXTAL as the timer clock input, and make each
	 * WDT clock tick equal 4 ticks of the system clock.
	 */
	writew(WDT_TCSR_PRESCALE4 | WDT_TCSR_EXT_EN, wdt_regs + WDT_TCSR);

	/* Reset the WDT counter to zero. */
	writew(0, wdt_regs + WDT_TCNT);

	/*
	 * Reset after 4ms
	 *
	 *         1 sec    CONFIG_SYS_EXTAL ticks
	 *  4ms * ------- * ---------------------- = Number of WDT clock ticks
	 *        1000 ms           1 sec
	 *
	 * As noted above the number of system clock ticks have been
	 * effectively multiplied by 4.  All that's left here for the
	 * computation of WDT clock ticks is to divide by 1000
	 * (one thousand).
	 */
	writew(CONFIG_SYS_EXTAL / 1000, wdt_regs + WDT_TDR);

	jz4780_tcu_wdt_start();

	/* WDT start */
	writeb(WDT_TCER_TCEN, wdt_regs + WDT_TCER);

	for (;;)
		;
}
