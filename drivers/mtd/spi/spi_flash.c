/*
 * SPI flash interface
 *
 * Copyright (C) 2008 Atmel Corporation
 * Copyright (C) 2010 Reinhard Meyer, EMK Elektronik
 *
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <spi_flash.h>
#include <watchdog.h>

#include "spi_flash_internal.h"
static int ref_count = 0;

static void spi_flash_addr(u32 addr, u8 *cmd, u32 addr_size)
{
	/* cmd[0] is actual command */
	if (addr_size == 4) {
		cmd[1] = addr >> 24;
		cmd[2] = addr >> 16;
		cmd[3] = addr >> 8;
		cmd[4] = addr >> 0;
	} else {
		cmd[1] = addr >> 16;
		cmd[2] = addr >> 8;
		cmd[3] = addr >> 0;
	}
}

static int spi_flash_read_write(struct spi_slave *spi,
				const u8 *cmd, size_t cmd_len,
				const u8 *data_out, u8 *data_in,
				size_t data_len)
{
	unsigned long flags = SPI_XFER_BEGIN;
	int ret;

	if (data_len == 0)
		flags |= SPI_XFER_END;

	ret = spi_xfer(spi, cmd_len * 8, cmd, NULL, flags);
	if (ret) {
		debug("SF: Failed to send command (%zu bytes): %d\n",
				cmd_len, ret);
	} else if (data_len != 0) {
		ret = spi_xfer(spi, data_len * 8, data_out, data_in, SPI_XFER_END);
		if (ret)
			debug("SF: Failed to transfer %zu bytes of data: %d\n",
					data_len, ret);
	}

	return ret;
}

int spi_flash_cmd(struct spi_slave *spi, u8 cmd, void *response, size_t len)
{
	return spi_flash_cmd_read(spi, &cmd, 1, response, len);
}

int spi_flash_cmd_read(struct spi_slave *spi, const u8 *cmd,
		size_t cmd_len, void *data, size_t data_len)
{
	return spi_flash_read_write(spi, cmd, cmd_len, NULL, data, data_len);
}

int spi_flash_cmd_write(struct spi_slave *spi, const u8 *cmd, size_t cmd_len,
		const void *data, size_t data_len)
{
	return spi_flash_read_write(spi, cmd, cmd_len, data, NULL, data_len);
}

static int read_rfsr(struct spi_flash *flash)
{
	ssize_t retval;
	u8 code = 0x70;
	u8 val;

	retval = spi_flash_cmd_read(flash->spi, &code, 1, &val, 1);

	if (retval < 0) {
		return retval;
	}

	return val;
}

static int wait_rfs_ready(struct spi_flash *flash)
{
	unsigned int timeout = 1000;
	int sr;
	
	do {
		if ((sr = read_rfsr(flash)) < 0)
			break;
		else if (sr & 0x80)
			return 0;

		mdelay(10);
		timeout--;
	
	} while (timeout > 0);

	return 1;
}

int spi_flash_cmd_write_multi(struct spi_flash *flash, u32 offset,
		size_t len, const void *buf)
{
	unsigned long page_addr, byte_addr, page_size;
	size_t chunk_len, actual;
	int ret;
	u8 cmd[5];

	page_size = flash->page_size;
	page_addr = offset / page_size;
	byte_addr = offset % page_size;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		debug("SF: unable to claim SPI bus\n");
		return ret;
	}

	cmd[0] = CMD_PAGE_PROGRAM;
	for (actual = 0; actual < len; actual += chunk_len) {
		chunk_len = min(len - actual, page_size - byte_addr);

		spi_flash_addr(offset + actual, cmd, flash->address_len);

		if (flash->address_len == 4)
			debug("PP: 0x%p => cmd = { 0x%02x 0x%02x%02x%02x%02x } chunk_len = %zu\n",
		      buf + actual, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], chunk_len);
		else
			debug("PP: 0x%p => cmd = { 0x%02x 0x%02x%02x%02x } chunk_len = %zu\n",
		      buf + actual, cmd[0], cmd[1], cmd[2], cmd[3], chunk_len);

		ret = spi_flash_cmd_write_enable(flash);
		if (ret < 0) {
			debug("SF: enabling write failed\n");
			break;
		}

		ret = spi_flash_cmd_write(flash->spi, cmd, flash->address_len + 1,
					  buf + actual, chunk_len);
		if (ret < 0) {
			debug("SF: write failed\n");
			break;
		}

		ret = spi_flash_cmd_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret)
			break;

#ifdef CONFIG_QUIRK_N25Q512A
		ret = wait_rfs_ready(flash);
		if (ret)
			break;
#endif

		page_addr++;
		byte_addr = 0;
	}

	debug("SF: program %s %zu bytes @ %#x\n",
	      ret ? "failure" : "success", len, offset);

	spi_release_bus(flash->spi);
	return ret;
}

int spi_flash_read_common(struct spi_flash *flash, const u8 *cmd,
		size_t cmd_len, void *data, size_t data_len)
{
	struct spi_slave *spi = flash->spi;
	int ret;

	spi_claim_bus(spi);
	ret = spi_flash_cmd_read(spi, cmd, cmd_len, data, data_len);	
	spi_release_bus(spi);

	return ret;
}

#define BUFF_SIZE 65536
int spi_flash_cmd_read_fast(struct spi_flash *flash, u32 offset,
		size_t len, void *data)
{
	int ret = 0;
	u8 cmd[5];
	cmd[0] = CMD_READ_ARRAY_FAST;
	u8 output[BUFF_SIZE];
	u8 *tmp = output;
	u8 *data_in = (u8 *)data;
	u32 next_offset = offset;
	size_t remain_size = len;
	size_t copy_size;

	while (remain_size > 0) {
		if (remain_size + flash->dummy_read / 8 <= BUFF_SIZE) 
			copy_size = remain_size + flash->dummy_read / 8;
		else 
			copy_size = BUFF_SIZE;

		spi_flash_addr(next_offset, cmd, flash->address_len);
		ret = spi_flash_read_common(flash, cmd, flash->address_len + 1, tmp, copy_size);
		if (ret)
			break;

		copy_size -= (flash->dummy_read / 8);
		next_offset += copy_size;
		remain_size -= copy_size;

		/* discard the dummy bytes */
		tmp += (flash->dummy_read / 8);
		memcpy(data_in, tmp, copy_size);
		data_in += copy_size;
	}

	return ret;
}

int spi_flash_cmd_poll_bit(struct spi_flash *flash, unsigned long timeout,
			   u8 cmd, u8 poll_bit)
{
	struct spi_slave *spi = flash->spi;
	unsigned long timebase;
	int ret;
	u8 status;

	ret = spi_xfer(spi, 8, &cmd, NULL, SPI_XFER_BEGIN);
	if (ret) {
		debug("SF: Failed to send command %02x: %d\n", cmd, ret);
		return ret;
	}

	timebase = get_timer(0);
	do {
		WATCHDOG_RESET();

		ret = spi_xfer(spi, 8, NULL, &status, 0);
		if (ret)
			return -1;

		if ((status & poll_bit) == 0)
			break;

	} while (get_timer(timebase) < timeout);

	spi_xfer(spi, 0, NULL, NULL, SPI_XFER_END);

	if ((status & poll_bit) == 0)
		return 0;

	/* Timed out */
	debug("SF: time out!\n");
	return -1;
}

int spi_flash_cmd_wait_ready(struct spi_flash *flash, unsigned long timeout)
{
	return spi_flash_cmd_poll_bit(flash, timeout,
		CMD_READ_STATUS, STATUS_WIP);
}

int spi_flash_cmd_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	u32 start, end, erase_size;
	int ret;
	u8 cmd[5];

	erase_size = flash->sector_size;
	if (offset % erase_size || len % erase_size) {
		debug("SF: Erase offset/length not multiple of erase size\n");
		return -1;
	}

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		debug("SF: Unable to claim SPI bus\n");
		return ret;
	}

	if (erase_size == 4096)
		cmd[0] = CMD_ERASE_4K;
	else
		cmd[0] = CMD_ERASE_64K;
	start = offset;
	end = start + len;

	while (offset < end) {
		spi_flash_addr(offset, cmd, flash->address_len);
		offset += erase_size;

		if (flash->address_len == 4)
			debug("SF: erase %2x %2x %2x %2x %2x (%x)\n", cmd[0], cmd[1],
		      		cmd[2], cmd[3], cmd[4], offset);
		else
			debug("SF: erase %2x %2x %2x %2x (%x)\n", cmd[0], cmd[1],
		      		cmd[2], cmd[3], offset);

		ret = spi_flash_cmd_write_enable(flash);
		if (ret)
			goto out;

		ret = spi_flash_cmd_write(flash->spi, cmd, flash->address_len + 1, NULL, 0);
		if (ret)
			goto out;

		ret = spi_flash_cmd_wait_ready(flash, SPI_FLASH_PAGE_ERASE_TIMEOUT);
		if (ret)
			goto out;
#ifdef CONFIG_QUIRK_N25Q512A
		ret = wait_rfs_ready(flash);
		if (ret)
			goto out;
#endif
	}

	debug("SF: Successfully erased %zu bytes @ %#x\n", len, start);

 out:
	spi_release_bus(flash->spi);
	return ret;
}

int spi_flash_cmd_write_status(struct spi_flash *flash, u8 sr)
{
	u8 cmd;
	int ret;

	ret = spi_flash_cmd_write_enable(flash);
	if (ret < 0) {
		debug("SF: enabling write failed\n");
		return ret;
	}

	cmd = CMD_WRITE_STATUS;
	ret = spi_flash_cmd_write(flash->spi, &cmd, 1, &sr, 1);
	if (ret) {
		debug("SF: fail to write status register\n");
		return ret;
	}

	ret = spi_flash_cmd_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
	if (ret < 0) {
		debug("SF: write status register timed out\n");
		return ret;
	}

	return 0;
}

#define CMD_EN_4BYTE 0xB7
#define CMD_DIS_4BYTE 0xE9

int spi_set_4byte_mode(struct spi_flash *flash, int en)
{
	u8 cmd;
	int ret;

	ret = spi_flash_cmd_write_enable(flash);
	if (ret < 0) {
		debug("SF: enabling write failed\n");
		return ret;
	}

	cmd = en ? CMD_EN_4BYTE : CMD_DIS_4BYTE;
	ret = spi_flash_cmd_write(flash->spi, &cmd, 1, NULL, 0);
	if (ret) {
		debug("SF: fail to set address mode\n");
		return ret;
	}

	flash->address_len = en ? 4 : 3;

	return 0;
}

/*
 * The following table holds all device probe functions
 *
 * shift:  number of continuation bytes before the ID
 * idcode: the expected IDCODE or 0xff for non JEDEC devices
 * probe:  the function to call
 *
 * Non JEDEC devices should be ordered in the table such that
 * the probe functions with best detection algorithms come first.
 *
 * Several matching entries are permitted, they will be tried
 * in sequence until a probe function returns non NULL.
 *
 * IDCODE_CONT_LEN may be redefined if a device needs to declare a
 * larger "shift" value.  IDCODE_PART_LEN generally shouldn't be
 * changed.  This is the max number of bytes probe functions may
 * examine when looking up part-specific identification info.
 *
 * Probe functions will be given the idcode buffer starting at their
 * manu id byte (the "idcode" in the table below).  In other words,
 * all of the continuation bytes will be skipped (the "shift" below).
 */
#define IDCODE_CONT_LEN 0
#define IDCODE_PART_LEN 5
static const struct {
	const u8 shift;
	const u8 idcode;
	struct spi_flash *(*probe) (struct spi_slave *spi, u8 *idcode);
} flashes[] = {
	/* Keep it sorted by define name */
#ifdef CONFIG_SPI_FLASH_ATMEL
	{ 0, 0x1f, spi_flash_probe_atmel, },
#endif
#ifdef CONFIG_SPI_FLASH_EON
	{ 0, 0x1c, spi_flash_probe_eon, },
#endif
#ifdef CONFIG_SPI_FLASH_MACRONIX
	{ 0, 0xc2, spi_flash_probe_macronix, },
#endif
#ifdef CONFIG_SPI_FLASH_SPANSION
	{ 0, 0x01, spi_flash_probe_spansion, },
#endif
#ifdef CONFIG_SPI_FLASH_SST
	{ 0, 0xbf, spi_flash_probe_sst, },
#endif
#ifdef CONFIG_SPI_FLASH_STMICRO
	{ 0, 0x20, spi_flash_probe_stmicro, },
#endif
#ifdef CONFIG_SPI_FLASH_WINBOND
	{ 0, 0xef, spi_flash_probe_winbond, },
#endif
#ifdef CONFIG_SPI_FRAM_RAMTRON
	{ 6, 0xc2, spi_fram_probe_ramtron, },
# undef IDCODE_CONT_LEN
# define IDCODE_CONT_LEN 6
#endif
	/* Keep it sorted by best detection */
#ifdef CONFIG_SPI_FLASH_STMICRO
	{ 0, 0xff, spi_flash_probe_stmicro, },
#endif
#ifdef CONFIG_SPI_FRAM_RAMTRON_NON_JEDEC
	{ 0, 0xff, spi_fram_probe_ramtron, },
#endif
};
#define IDCODE_LEN (IDCODE_CONT_LEN + IDCODE_PART_LEN)

struct spi_flash *spi_flash_probe(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int spi_mode)
{
	struct spi_slave *spi;
	struct spi_flash *flash = NULL;
	int ret, i, shift;
	u8 idcode[IDCODE_LEN], *idp;

	spi = spi_setup_slave(bus, cs, max_hz, spi_mode);
	if (!spi) {
		printf("SF: Failed to set up slave\n");
		return NULL;
	}

	ret = spi_claim_bus(spi);
	if (ret) {
		debug("SF: Failed to claim SPI bus: %d\n", ret);
		goto err_claim_bus;
	}

	/* Read the ID codes */
	ret = spi_flash_cmd(spi, CMD_READ_ID, idcode, sizeof(idcode));
	if (ret)
		goto err_read_id;

#ifdef DEBUG
	printf("SF: Got idcodes\n");
	print_buffer(0, idcode, 1, sizeof(idcode), 0);
#endif

	/* count the number of continuation bytes */
	for (shift = 0, idp = idcode;
	     shift < IDCODE_CONT_LEN && *idp == 0x7f;
	     ++shift, ++idp)
		continue;

	/* search the table for matches in shift and id */
	for (i = 0; i < ARRAY_SIZE(flashes); ++i)
		if (flashes[i].shift == shift && flashes[i].idcode == *idp) {
			/* we have a match, call probe */
			flash = flashes[i].probe(spi, idp);
			if (flash)
				break;
		}

	if (!flash) {
		printf("SF: Unsupported manufacturer %02x\n", *idp);
		goto err_manufacturer_probe;
	}

	/* Don't print these to keep the print board info format*/
#ifndef CONFIG_FLASH_CMD_FOR_SF
	printf("SF: Detected %s with page size ", flash->name);
	print_size(flash->sector_size, ", total ");
	print_size(flash->size, "\n");
#endif

	spi_release_bus(spi);

	ref_count++;

	return flash;

err_manufacturer_probe:
err_read_id:
	spi_release_bus(spi);
err_claim_bus:
	//spi_free_slave(spi);
	return NULL;
}

void spi_flash_free(struct spi_flash *flash)
{
	ref_count--;
	if (ref_count <= 0 && flash->address_len == 4)
		spi_set_4byte_mode(flash, 0);
	spi_free_slave(flash->spi);
	free(flash);
}

#ifdef CONFIG_FLASH_CMD_FOR_SF
int spi_read_lock_status(struct spi_flash *flash, u32 offset, u8 *lock)
{
	int ret;;
	u8 cmd[4];
	
	ret = spi_flash_cmd_write_enable(flash);
		if (ret)
			return -1;

	/* send read lock status command */
	cmd[0] = 0xE8;
	spi_flash_addr(offset, cmd, flash->address_len);
	ret = spi_flash_cmd_read(flash->spi, cmd, sizeof(cmd), lock, 1);
	if (ret)
		debug("Fail to read loc status register.\n");

	/* wait for completion */
	ret = spi_flash_cmd_wait_ready(flash, SPI_FLASH_PAGE_ERASE_TIMEOUT);
	if (ret)
		return ret;
#ifdef CONFIG_QUIRK_N25Q512A
	ret = wait_rfs_ready(flash);
	if (ret)
		return ret;
#endif

	return 0;
	
}

int spi_write_lock_status(struct spi_flash *flash, u32 offset, u8 lock)
{
	int ret;
	u8 cmd[5];
	
	ret = spi_flash_cmd_write_enable(flash);
		if (ret)
			return -1;

	/* send write lock status command */
	cmd[0] = 0xE5;
	spi_flash_addr(offset, cmd, flash->address_len);
	cmd[4] = lock;
	ret = spi_flash_cmd_write(flash->spi, cmd, sizeof(cmd), NULL, 0);
	if (ret)
		debug("fail to write lock status register.\n");

	/* wait for completion */
	ret = spi_flash_cmd_wait_ready(flash, SPI_FLASH_PAGE_ERASE_TIMEOUT);
	if (ret)
		return ret;
#ifdef CONFIG_QUIRK_N25Q512A
	ret = wait_rfs_ready(flash);
	if (ret)
		return ret;
#endif

	return 0;
}
#endif /* CONFIG_FLASH_CMD_FOR_SF */
