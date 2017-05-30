/*
 *  u-boot/common/cmd_romfs.c
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
#include <command.h>
#include <romfs.h>

int do_romfsload(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	void *romfs = NULL;
	char *filepath = NULL;
	unsigned int addr = 0x0;
	unsigned int maxsize = 1024 * 1024 * 100;

	if (argc < 3)
		return cmd_usage(cmdtp);

	/* parse the parameters: offset, addr, file name, max size */
	romfs = (void *)simple_strtoul(argv[1], NULL, 16);
	addr = simple_strtoul(argv[2], NULL, 16);
	filepath = argv[3];

	if (argc >= 5)
		maxsize = simple_strtoul(argv[4], NULL, 16);

	if (romfs_mount(romfs)) {
		printf("Fail to mount romfs\n");
		return 1;
	}

	if (romfs_filesystem_read(filepath, (void *)addr, maxsize) < 1) {
		printf("Fail to find the file\n");
		return 1;
	}

	return 0;
}

U_BOOT_CMD(
	romfsload,	4,	0,	do_romfsload,
	"load binary file from a rom filesystem",
	"[offset] [addr] [filename] <size>\n"
	"    - load binary file 'filename' which max size 'size' from romfs at physical address 'offset'\n"
	"    to address 'addr'\n"
);
