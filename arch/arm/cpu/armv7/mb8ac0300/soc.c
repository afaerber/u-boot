/*
 * u-boot/arch/arm/cpu/armv7/mb8ac0300/soc.c
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
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <netdev.h>
#include <asm/errno.h>
#include <asm/pl310.h>

struct pl310_regs *const pl310reg = (struct pl310_regs *)CONFIG_SYS_PL310_BASE;

/*
 * Reset the cpu by setting software reset request bit
 */
void reset_cpu(ulong ignored)
{
	/*
	 *  CRG11
	 */

	/* CRRRS : Asserting Request for RRESETn */
	writel(0x0, CRG_BASE + CRG_CRRRS);

	/* CRRSC : Software reset mode */
	writel(0x100, CRG_BASE + CRG_CRRSC);

	/* CRSWR : Software reset request */
	writel(0x1, CRG_BASE + CRG_CRSWR);

	while (1);

}

#ifdef CONFIG_SYS_L2_PL310
/*
 * L2(PL310) dcache function
 */
void v7_outer_cache_enable(void)
{
	writel(0x1, &pl310reg->pl310_ctrl); /* reg1_control Register */
}

void v7_outer_cache_disable(void)
{
	writel(0x0, &pl310reg->pl310_ctrl); /* reg1_control Register */
}
#endif

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif

#if defined(CONFIG_DISPLAY_CPUINFO)
int print_cpuinfo (void)
{
	printf("CPU:   MB8AC0300:Cortex-A9 Dual Core %d MHz\n",
			(CONFIG_MB8AC0300_IOCLK/1000000));
	return 0;
}
#endif

#if CONFIG_DRIVER_FGMAC4
int cpu_eth_init(bd_t *bis)
{
	return fgmac4_initialize(bis);
}
#endif

#if defined(CONFIG_F_SDH30_SDHCI)
extern
int f_sdh30_sdhci_init(int regbase, int max_clk, int min_clk, int quirks);
extern void f_sdh30_reset(void);

int cpu_mmc_init(bd_t *bd)
{
	return f_sdh30_sdhci_init(F_SDH30_BASE, 0, 400000, 0);
}

void arch_preboot_os(void)
{
	f_sdh30_reset();
}
#endif
