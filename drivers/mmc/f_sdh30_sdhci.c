/*
 * u-boot/drivers/mmc/f_sdh30_sdhci.c
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
#include <malloc.h>
#include <mmc.h>
#include <sdhci.h>

extern int mmc_go_idle(struct mmc* mmc);

int f_sdh30_sdhci_init(int regbase, int tmclk, int max_clk, int min_clk, int quirks)
{
	int ret = 0;
	struct sdhci_host *host = NULL;

	host = (struct sdhci_host *)malloc(sizeof (struct sdhci_host));
	if (!host) {
		printf("sdhci_host malloc fail!\n");
		return -1;
	}

	host->name = "f_sdh30_sdhci";
	host->ioaddr = (void *)regbase;
	host->quirks = quirks;
	host->version = sdhci_readw(host, SDHCI_HOST_VERSION);
	host->host_caps = MMC_MODE_BUS_WIDTH_TEST;

	ret = add_sdhci(host, max_clk, min_clk);
	if (ret == 0) {
		host->mmc->tm_clock = tmclk;
	}
	
	return ret;
}

void f_sdh30_reset(void)
{
	struct mmc *mmc;
	int dev_num, i;

	dev_num = get_mmc_num();
	if (dev_num < 0) {
		printf("No MMC device available\n");
		return;
	}

	for (i = 0; i < dev_num; i++) {
		mmc = find_mmc_device(i);
		if (mmc && mmc->has_init) {
			mmc_go_idle(mmc);
			sdhci_writeb((struct sdhci_host *)mmc->priv,
				     0, SDHCI_POWER_CONTROL);
		}
	}
}
