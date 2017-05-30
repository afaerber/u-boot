/*
 *  u-boot/board/fujistu/mb86s72/flash.c
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
#include <spi_flash.h>

static struct spi_flash *flash[2] = {0};
int chip_select = 0;

flash_info_t	flash_info[CONFIG_SYS_MAX_FLASH_BANKS]; /* info for FLASH chips	*/
flash_info_t	flash_chips[2];

#define MAX_TABLE_SIZE CONFIG_SYS_MAX_FLASH_SECT / 8 + 1

typedef struct {
	uint32_t digest;
	u8 protect_table[MAX_TABLE_SIZE];
} protect_tb_t;

protect_tb_t tb[2];

int init_protect_table(flash_info_t *info, int cs)
{
	int len = MAX_TABLE_SIZE;
	int ret;
	int i;

	chip_select = cs;
	/* read the table */
	ret = spi_flash_read(flash[0], CONFIG_PROTECTION_TB_OFFSET + 
		cs * flash[0]->sector_size, sizeof(protect_tb_t), &tb[cs]);
	if (ret)
		return ret;

	/*  check if the table is there */
	if(crc32(0, tb[cs].protect_table, MAX_TABLE_SIZE) != tb[cs].digest) {
		for (i = 0; i < info->sector_count; i++) 
			info->protect[i] = 0;		
		for (i = 0; i < len; i++)
			tb[cs].protect_table[i] = 0;
		
		return 0;
	}

	/* loop: read the corresponding bit for the sector and set status */
	for (i = 0; i < info->sector_count; i++) {
		info->protect[i]= (tb[cs].protect_table[i / 8] >> (i % 8)) & 0x1;
		if (info->protect[i]) 
			spi_write_lock_status(flash[cs], i * flash[cs]->sector_size, 1);
	}

	return 0;
}

int save_protect_table(flash_info_t *info)
{
	int ret;
	int len = MAX_TABLE_SIZE;
	int old_cs = chip_select;

	tb[old_cs].digest = crc32(0, tb[old_cs].protect_table, len);

	/* erase and rewrite the sector */
	chip_select = 0;
	
	ret = flash_erase(&flash_chips[0], CONFIG_PROTECTION_TB_OFFSET / flash[chip_select]->sector_size + old_cs, 
		CONFIG_PROTECTION_TB_OFFSET / flash[chip_select]->sector_size + old_cs);
	if (ret)
		goto out;

	ret = write_buff(&flash_chips[0], (uchar *)&tb[old_cs], 
		CONFIG_PROTECTION_TB_OFFSET + CONFIG_SYS_FLASH_BASE + 
		old_cs * flash[old_cs]->sector_size, sizeof(protect_tb_t));
out:
	chip_select = old_cs;

	return ret;
}

int update_protect_table(flash_info_t *info, long sector, int prot, int cs)
{

	/* find the bit to update */
	u8 byte = tb[cs].protect_table[sector / 8];
	u8 mask = (1 << (sector % 8)) & 0xff;

	byte = (byte & (~mask)) | (prot << (sector % 8));

	tb[cs].protect_table[sector / 8] = byte;

	return 0;
}

unsigned long flash_init (void)
{
	int i;
	u32 sys_flash_size = get_sys_flash_size();

	for (i = 0; i < 2; i++) {
		flash_chips[i].flash_id = FLASH_UNKNOWN;
	}

	flash[0] = spi_flash_probe(0, 0, 12500000, 0);
	
	if(flash[0]) {
		flash_chips[0].flash_id = 0x00;
		flash_chips[0].size = flash[0]->size;
		if (flash_chips[0].size > sys_flash_size)
			flash_chips[0].size = sys_flash_size;
		flash_chips[0].sector_count = flash[0]->size / flash[0]->sector_size;
		if (flash_chips[0].sector_count > CONFIG_SYS_MAX_FLASH_SECT)
			flash_chips[0].sector_count = CONFIG_SYS_MAX_FLASH_SECT;
	}

	flash[1] = spi_flash_probe(0, 1, 12500000, 0);
	if(flash[1]) {
		flash_chips[1].flash_id = 0x01;
		flash_chips[1].size = flash[1]->size;
		if (flash_chips[1].size > sys_flash_size)
			flash_chips[1].size = sys_flash_size;
		flash_chips[1].sector_count = flash[1]->size / flash[1]->sector_size;
		if (flash_chips[1].sector_count > CONFIG_SYS_MAX_FLASH_SECT)
			flash_chips[1].sector_count = CONFIG_SYS_MAX_FLASH_SECT;
	}


	for(i = 0; i < flash_chips[0].sector_count; i++) {
		flash_chips[0].start[i] = CONFIG_SYS_FLASH_BASE + i * flash[0]->sector_size;
	}

	for(i = 0; i < flash_chips[1].sector_count; i++) {
		flash_chips[1].start[i] = CONFIG_SYS_FLASH_BASE + i * flash[1]->sector_size;
	}

	init_protect_table(&flash_chips[0], 0);
	init_protect_table(&flash_chips[1], 1);

	/* using chip select 0 as default */
	chip_select = 0;
	memcpy(&flash_info[0], &flash_chips[0], sizeof(flash_info_t));

	return flash_info[0].size;
}

void flash_print_info (flash_info_t *info)
{
	int i;
	if(flash[chip_select])
		printf("SPI Flash: %s\n", flash[chip_select]->name);
	
	printf ("  Size: %ld MB in %d Sectors\n",
		info->size >> 20, info->sector_count);

	printf ("  Sector Start Addresses:");
	for (i = 0; i < info->sector_count; i++) {
		if ((i % 4) == 0) {
			printf ("\n   ");
		}
		printf (" %08lX%s", info->start[i],
			info->protect[i] ? " (RO)" : "     ");
	}
	printf ("\n");
}

int	flash_erase (flash_info_t *info, int s_first, int s_last)
{
	int offset = s_first * flash[chip_select]->sector_size;
	size_t size = (s_last - s_first + 1) * flash[chip_select]->sector_size;
	
	return spi_flash_erase(flash[chip_select], offset, size);
}

int read_flash_buff (uchar *dest, ulong addr, ulong cnt)
{
	return spi_flash_read(flash[chip_select], addr - CONFIG_SYS_FLASH_BASE, cnt, dest);
}


int write_buff (flash_info_t *info, uchar *src, ulong addr, ulong cnt)
{
	return spi_flash_write(flash[chip_select], addr - CONFIG_SYS_FLASH_BASE, cnt, src);
}

int flash_real_protect(flash_info_t *info, long sector, int prot)
{
	int ret;
	int offset = sector * flash[0]->sector_size;

	/* check for protect table sector, cannot protect it */
	if (chip_select == 0) {
		if(offset == CONFIG_PROTECTION_TB_OFFSET || 
			offset == (CONFIG_PROTECTION_TB_OFFSET + flash[0]->sector_size)) {
			printf("Error:The protection bitmap table cannot be protected.\n");
			return -1;
		}
	}
	
	ret = spi_write_lock_status(flash[chip_select], sector * flash[chip_select]->sector_size, prot);
	if (!ret)
		info->protect[sector] = prot;

	ret = update_protect_table(info, sector, prot, chip_select);
	
	return ret;
}

int flash_chip_select(int cs)
{
	if (!flash[cs])
		return -1;
	
	chip_select = cs;
	memcpy(&flash_info[0], &flash_chips[cs], sizeof(flash_info_t));

	return 0;
}

int get_chip_select(void)
{
	return chip_select;
}
