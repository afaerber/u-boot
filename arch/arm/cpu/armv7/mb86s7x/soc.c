/*
 * u-boot/arch/arm/cpu/armv7/mb86s7x/soc.c
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
#include <sdhci.h>
#ifdef CONFIG_MB86S7X_MHU
#include <mhu.h>
#include <scb_mhu_api.h>
#endif

#ifdef CONFIG_MB86S7X_HS_SPI
extern void hs_spi_cleanup(void);
#endif

void reset_cpu(ulong ignored)
{
	/* Use MHU command to do hard reset */

#ifdef CONFIG_MB86S7X_MHU
	struct cmd_hard_reset volatile *cmd = cmd_to_scb;

	cmd->payload_size = sizeof(*cmd);
	cmd->delay = 10;

	if (mhu_send(CMD_HARD_RESET_REQ)) {
		puts(" Failed do hard reset\n");
	}
	else {
		/* going to reset */
	}

#endif
	puts("Fail to restart.\n");

	while (1);

}

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
#if defined(CONFIG_MB86S70)
	printf("CPU:   MB86S70:Cortex-A15/A7 big.LITTLE\n");
#elif defined(CONFIG_MB86S72)
	printf("CPU:   MB86S72:Cortex-A15/A7 big.LITTLE\n");
#elif defined(CONFIG_MB86S71)
	printf("CPU:   MB86S71:Cortex-A15/A7 big.LITTLE\n");
#elif defined(CONFIG_MB86S73)
	printf("CPU:   MB86S73:Cortex-A7x2\n");
#else
#error Undefined Board
#endif
	return 0;
}
#endif

#if CONFIG_DRIVER_FGMAC4
int cpu_eth_init(bd_t *bis)
{
	return fgmac4_initialize(bis);
}
#endif

#if defined(CONFIG_DRIVER_OGMA)
int cpu_eth_init(bd_t *bis)
{
	return ogma_initialize(bis);
}
#endif

#if defined(CONFIG_F_SDH30_SDHCI)
extern
int f_sdh30_sdhci_init(int regbase, int tmclk, int max_clk, int min_clk, int quirks);
extern void f_sdh30_reset(void);

int cpu_mmc_init(bd_t *bd)
{
	int err_sd = -1, err_emmc = -1;
	int emmc_max_clk = 0;
	int quirks = 0;

#if defined(CONFIG_EMMC_MAX_CLOCK)
	emmc_max_clk = CONFIG_EMMC_MAX_CLOCK;
#endif
#if defined(CONFIG_MMC_ADMA) && defined(CONFIG_SDHCI_AUTO_CMD23)
	quirks = SDHCI_QUIRK_AUTO_CMD23;
#elif defined(CONFIG_SDHCI_AUTO_CMD12)
	quirks = SDHCI_QUIRK_AUTO_CMD12;
#endif
#if F_EMMC_BASE
	err_emmc = f_sdh30_sdhci_init(F_EMMC_BASE, CONFIG_EMMC_CLOCK, emmc_max_clk, 400000, quirks);
#endif
	err_sd = f_sdh30_sdhci_init(F_SDH30_BASE, CONFIG_SD_CLOCK, 0, 400000, SDHCI_QUIRK_WAIT_SEND_CMD);
	/* if both sd and emmc is not found, return error*/
	if(err_sd && err_emmc)
		return -1;
	else
		return 0;
}

void arch_preboot_os(void)
{
#ifdef CONFIG_MB86S7X_HS_SPI
	hs_spi_cleanup();
#endif
	f_sdh30_reset();
}
#endif
