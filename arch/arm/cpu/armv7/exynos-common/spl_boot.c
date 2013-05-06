/*
 * Copyright (C) 2011 Samsung Electronics
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <linux/compiler.h>
#include <config.h>
#include <elog.h>
#include <spi.h>
#include <asm/arch/board.h>
#include <asm/arch/clock.h>
#include <asm/arch-exynos/spi.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/power.h>
#include <linux/lzo.h>

DECLARE_GLOBAL_DATA_PTR;

#define OM_STAT		(0x1f << 1)

struct wake_event {
	struct event_header header;
	uint8_t sleep_type;
	uint8_t checksum;
} __packed;

/**
 * Copy data from SD or MMC device to RAM.
 *
 * @param offset	Block offset of the data
 * @param nblock	Number of blocks
 * @param dst		Destination address
 * @return 1 = True or 0 = False
 */
typedef u32 (*mmc_copy_func_t)(u32 offset, u32 nblock, u32 dst);

/**
 * Copy data from SPI flash to RAM.
 *
 * @param offset	Block offset of the data
 * @param nblock	Number of blocks
 * @param dst		Destination address
 * @return 1 = True or 0 = False
 */
typedef u32 (*spi_copy_func_t)(u32 offset, u32 nblock, u32 dst);


/**
 * Copy data through USB.
 *
 * @return 1 = True or 0 = False
 */
typedef u32 (*usb_copy_func_t)(void);

/*
 * Set/clear program flow prediction and return the previous state.
 */
static int config_branch_prediction(int set_cr_z)
{
	unsigned int cr;

	/* System Control Register: 11th bit Z Branch prediction enable */
	cr = get_cr();
	set_cr(set_cr_z ? cr | CR_Z : cr & ~CR_Z);

	return cr & CR_Z;
}

static void spi_rx_tx(struct exynos_spi *regs, int todo, void *dinp,
		      void const *doutp, int word_chunks)
{
	uint8_t *rxp = (uint8_t *)dinp;
	uint8_t *txp = (uint8_t *)doutp;
	int rx_lvl, tx_lvl;
	uint out_bytes, in_bytes;
	int chunk_size;

	out_bytes = in_bytes = todo;

	if (word_chunks)
		chunk_size = 4;
	else
		chunk_size = 1;

	setbits_le32(&regs->ch_cfg, SPI_CH_RST);
	clrbits_le32(&regs->ch_cfg, SPI_CH_RST);
	writel((todo / chunk_size) | SPI_PACKET_CNT_EN, &regs->pkt_cnt);

	while (in_bytes || out_bytes) {
		uint32_t spi_sts;
		int temp = 0xffffffff;

		spi_sts = readl(&regs->spi_sts);
		rx_lvl = ((spi_sts >> 15) & 0x7f);
		tx_lvl = ((spi_sts >> 6) & 0x7f);
		while (tx_lvl < 32 && out_bytes) {
			if (doutp) {
				memcpy(&temp, txp, chunk_size);
				txp += chunk_size;
			}
			writel(temp, &regs->tx_data);
			out_bytes -= chunk_size;
			tx_lvl += chunk_size;
		}
		while (rx_lvl >= chunk_size && in_bytes) {
			temp = readl(&regs->rx_data);
			if (dinp) {
				memcpy(rxp, &temp, chunk_size);
				rxp += chunk_size;
			}
			in_bytes -= chunk_size;
			rx_lvl -= chunk_size;
		}
	}
}

/**
 * Prepare the spi controller for a transaction
 *
 * @param regs		spi controller register structure
 */
static void exynos_spi_init(struct exynos_spi *regs, int word_chunks)
{
	clock_set_rate(PERIPH_ID_SPI1, 50000000); /* set spi clock to 50Mhz */
	/* set the spi1 GPIO */
	exynos_pinmux_config(PERIPH_ID_SPI1, PINMUX_FLAG_NONE);

	/* set pktcnt and enable it */
	writel(4 | SPI_PACKET_CNT_EN, &regs->pkt_cnt);
	/* set FB_CLK_SEL */
	writel(SPI_FB_DELAY_180, &regs->fb_clk);
	/* set CH_WIDTH and BUS_WIDTH */
	clrbits_le32(&regs->mode_cfg, SPI_MODE_CH_WIDTH_MASK |
				      SPI_MODE_BUS_WIDTH_MASK);
	if (word_chunks)
		setbits_le32(&regs->mode_cfg, SPI_MODE_CH_WIDTH_WORD |
					      SPI_MODE_BUS_WIDTH_WORD);
	else
		setbits_le32(&regs->mode_cfg, SPI_MODE_CH_WIDTH_BYTE |
					      SPI_MODE_BUS_WIDTH_BYTE);
	clrbits_le32(&regs->ch_cfg, SPI_CH_CPOL_L); /* CPOL: active high */

	/* clear rx and tx channel if set priveously */
	clrbits_le32(&regs->ch_cfg, SPI_RX_CH_ON | SPI_TX_CH_ON);

	setbits_le32(&regs->swap_cfg, SPI_RX_SWAP_EN |
		SPI_RX_BYTE_SWAP | SPI_RX_HWORD_SWAP);

	/* do a soft reset */
	setbits_le32(&regs->ch_cfg, SPI_CH_RST);
	clrbits_le32(&regs->ch_cfg, SPI_CH_RST);

	/* now set rx and tx channel ON */
	setbits_le32(&regs->ch_cfg, SPI_RX_CH_ON | SPI_TX_CH_ON | SPI_CH_HS_EN);
	clrbits_le32(&regs->cs_reg, SPI_SLAVE_SIG_INACT); /* CS low */
}

/**
 * Shut down the spi controller after a transaction
 *
 * @param regs		spi controller register structure
 */
static void exynos_spi_finish(struct exynos_spi *regs)
{
	setbits_le32(&regs->cs_reg, SPI_SLAVE_SIG_INACT);/* make the CS high */

	/*
	 * Let put controller mode to BYTE as
	 * SPI driver does not support WORD mode yet
	 */
	clrbits_le32(&regs->mode_cfg, SPI_MODE_CH_WIDTH_WORD |
					SPI_MODE_BUS_WIDTH_WORD);
	writel(0, &regs->swap_cfg);

	/*
	 * Flush spi tx, rx fifos and reset the SPI controller
	 * and clear rx/tx channel
	 */
	clrsetbits_le32(&regs->ch_cfg, SPI_CH_HS_EN, SPI_CH_RST);
	clrbits_le32(&regs->ch_cfg, SPI_CH_RST);
	clrbits_le32(&regs->ch_cfg, SPI_TX_CH_ON | SPI_RX_CH_ON);
}

/**
 * Read data from spi flash
 *
 * @param offset	offset to read from
 * @parma size		size of data to read, must be a divisible by 4
 * @param addr		address to read data into
 */
static void exynos_spi_read(unsigned int offset, unsigned int size,
			    uintptr_t addr)
{
	int upto, todo;
	int i;
	struct exynos_spi *regs = (struct exynos_spi *)samsung_get_base_spi1();

	exynos_spi_init(regs, 1);

	/* Send read instruction (0x3h) followed by a 24 bit addr */
	writel((SF_READ_DATA_CMD << 24) | (offset & 0xffffff), &regs->tx_data);

	/* waiting for TX done */
	while (!(readl(&regs->spi_sts) & SPI_ST_TX_DONE));

	for (upto = 0, i = 0; upto < size; upto += todo, i++) {
		todo = min(size - upto, (1 << 15));
		spi_rx_tx(regs, todo, (void *)(addr + (i << 15)), NULL, 1);
	}

	exynos_spi_finish(regs);
}

/**
 * Write data into spi flash
 *
 * @param offset	offset to write to
 * @parma size		size of data to write
 * @param addr		address of data to write
 */
static void exynos_spi_write(unsigned int offset, unsigned int size,
			     uintptr_t addr)
{
	struct exynos_spi *regs = (struct exynos_spi *)samsung_get_base_spi1();
	uint8_t *data = (uint8_t *)addr;

	/*
	 * Send one byte at a time so can ignore page size. We're not writing
	 * much so performance shouldn't matter.
	 */
	while (size--) {
		/* Send a write enable command */
		exynos_spi_init(regs, 0);
		writel(SF_WRITE_ENABLE_CMD, &regs->tx_data);
		while (!(readl(&regs->spi_sts) & SPI_ST_TX_DONE));
		exynos_spi_finish(regs);

		exynos_spi_init(regs, 0);

		/* Send a write command followed by a 24 bit addr */
		writel(SF_WRITE_DATA_CMD, &regs->tx_data);
		writel(offset >> 16, &regs->tx_data);
		writel(offset >> 8, &regs->tx_data);
		writel(offset >> 0, &regs->tx_data);

		/* Write out a byte. */
		spi_rx_tx(regs, 1, NULL, data, 0);

		/* waiting for TX done */
		while (!(readl(&regs->spi_sts) & SPI_ST_TX_DONE));

		exynos_spi_finish(regs);

		data++;
		offset++;
	}
}

/**
 * Small helper for exynos_find_end_of_log.
 *
 * @param regs		SPI controller registers
 * @param data		A buffer for the data
 * @param offset	The offset to read from, which is updated
 */
static void exynos_spi_get4bytes(struct exynos_spi *regs, void *data,
				 int *offset)
{
	spi_rx_tx(regs, 4, data, NULL, 1);
	*offset += 4;
}

/* Scan for the end of the event log coming in over SPI. */
static int exynos_find_end_of_log(void)
{
	struct exynos_spi *regs = (struct exynos_spi *)samsung_get_base_spi1();
	uint32_t word;
	uint8_t bytes[4];
	int header_size;
	int offset = 0;
	int log_offset;
	int found = 0;

	exynos_spi_init(regs, 1);

	/* Send read instruction (0x3h) followed by a 24 bit addr */
	writel((SF_READ_DATA_CMD << 24) | CONFIG_ELOG_OFFSET, &regs->tx_data);

	/* waiting for TX done */
	while (!(readl(&regs->spi_sts) & SPI_ST_TX_DONE));

	exynos_spi_get4bytes(regs, &word, &offset);
	if (word != ELOG_SIGNATURE) {
		printf("Bad event log signature.\n");
		log_offset = -1;
		goto error;
	}

	exynos_spi_get4bytes(regs, bytes, &offset);
	header_size = bytes[1];

	/* set log_offset to the start of the first event */
	log_offset = header_size;
	if (log_offset >= CONFIG_ELOG_SIZE) {
		printf("Reached the end of the log.\n");
		log_offset = -1;
		goto error;
	}

	while (1) {
		uint8_t type;
		uint8_t event_length;

		while (log_offset >= offset)
			exynos_spi_get4bytes(regs, bytes, &offset);

		/* the type is the first byte */
		type = bytes[log_offset % 4];
		/* if we found the end, stop looking */
		if (type == ELOG_TYPE_EOL) {
			found = 1;
			break;
		}

		/* load more bytes if the size is in the next chunk */
		if (log_offset + 1 >= offset)
			exynos_spi_get4bytes(regs, bytes, &offset);

		/* scan past this event */
		event_length = bytes[(log_offset + 1) % 4];
		log_offset += event_length;
		if (log_offset >= CONFIG_ELOG_SIZE) {
			printf("Reached the end of the log.\n");
			log_offset = -1;
			goto error;
		}
	}

	if (!found) {
		printf("Didn't find the end of the log.\n");
		log_offset = -1;
		goto error;
	}

error:
	exynos_spi_finish(regs);
	return log_offset;
}

/* Put a wake event in the event log. */
static void exynos_log_wake_event(void)
{
	int offset;
	struct wake_event event;
	uint8_t sleep_type = 3;

	offset = exynos_find_end_of_log();

	if (offset < 0)
		return;
	/* offset is relative to the log, we need relative to flash */
	offset += CONFIG_ELOG_OFFSET;

	elog_prepare_event(&event, ELOG_TYPE_ACPI_WAKE, &sleep_type, 1);
	exynos_spi_write(offset, sizeof(event), (uintptr_t)&event);
}


/* The memcpy function is not in SPL u-boot, so create one. */
void *memcpy(void *d, const void *s, size_t n)
{
	const char *sptr = s;
	char *dptr = d;
	size_t i;

	for (i = 0; i < n; i++)
		*dptr++ = *sptr++;
	return d;
}

/* Copy U-Boot image to RAM */
static void copy_uboot_to_ram(void)
{
	unsigned int sec_boot_check;
	unsigned int uboot_size;
	enum compress_t compress_type;
	uintptr_t uboot_load_addr;
	int is_cr_z_set;
	enum boot_mode boot_mode;
	mmc_copy_func_t mmc_copy;
#if defined(CONFIG_SPL_LZO_SUPPORT)
	int ret;
#endif

	usb_copy_func_t usb_copy;

	uboot_size = exynos_get_uboot_size();
	boot_mode = exynos_get_boot_device();
	compress_type = exynos_get_compress_type();

	if (compress_type == UBOOT_COMPRESS_NONE) {
		uboot_load_addr = CONFIG_SYS_TEXT_BASE;
#if defined(CONFIG_SPL_LZO_SUPPORT)
	} else if (compress_type == UBOOT_COMPRESS_LZO) {
		/* Load U-Boot image to 1MB after the text base. */
		uboot_load_addr = CONFIG_SYS_TEXT_BASE + 0x100000;
#endif
	} else {
		panic("Invalid compression type %u\n", compress_type);
	}

	if (boot_mode == BOOT_MODE_OM) {
		/* Read iRAM location to check for secondary USB boot mode */
		sec_boot_check = readl(EXYNOS_IRAM_SECONDARY_BASE);
		if (sec_boot_check == EXYNOS_USB_SECONDARY_BOOT)
			boot_mode = BOOT_MODE_USB;
	}
	debug("U-Boot size %u\n", uboot_size);

	if (boot_mode == BOOT_MODE_OM)
		boot_mode = readl(EXYNOS_POWER_BASE) & OM_STAT;

	switch (boot_mode) {
#if defined(CONFIG_EXYNOS_SPI_BOOT)
	case BOOT_MODE_SERIAL:
		/* let us our own function to copy u-boot from SF */
		exynos_spi_read(SPI_FLASH_UBOOT_POS, uboot_size,
				uboot_load_addr);
		break;
#endif
	case BOOT_MODE_MMC:
		mmc_copy = *(mmc_copy_func_t *)EXYNOS_COPY_MMC_FNPTR_ADDR;
		assert(!(uboot_size & 511));
		mmc_copy(BL2_START_OFFSET, uboot_size / 512, uboot_load_addr);
		break;
	case BOOT_MODE_USB:
		/*
		 * iROM needs program flow prediction to be disabled
		 * before copy from USB device to RAM
		 */
		is_cr_z_set = config_branch_prediction(0);
		usb_copy = *(usb_copy_func_t *)
				EXYNOS_COPY_USB_FNPTR_ADDR;
		usb_copy();
		config_branch_prediction(is_cr_z_set);
		/*
		 * The above usb_copy() loads U-Boot to a fixed location, i.e.
		 * CONFIG_SYS_TEXT_BASE. Move it before uncompress.
		 */
		if (uboot_load_addr != CONFIG_SYS_TEXT_BASE) {
			memcpy((void *)uboot_load_addr,
			       (void *)CONFIG_SYS_TEXT_BASE, uboot_size);
		}
		break;
	default:
		panic("Invalid boot mode selection\n");
		break;
	}
	debug("U-Boot copied\n");

#if defined(CONFIG_SPL_LZO_SUPPORT)
	if (compress_type == UBOOT_COMPRESS_LZO) {
		ret = lzop_decompress((void *)uboot_load_addr, uboot_size,
				(void *)CONFIG_SYS_TEXT_BASE, &uboot_size);
		if (ret < 0)
			panic("LZO: uncompress error -%u\n", -ret);
		debug("U-Boot uncompressed\n");
	}
#endif
}

/* The memzero function is not in SPL u-boot, so create one. */
void memzero(void *s, size_t n)
{
	char *ptr = s;
	size_t i;

	for (i = 0; i < n; i++)
		*ptr++ = '\0';
}

/**
 * Set up the U-Boot global_data pointer
 *
 * This sets the address of the global data, and sets up basic values.
 *
 * @param gdp	Value to give to gd
 */
static void setup_global_data(gd_t *gdp)
{
	gd = gdp;
	memzero((void *)gd, sizeof(gd_t));
	gd->flags |= GD_FLG_RELOC;
	gd->baudrate = CONFIG_BAUDRATE;
	gd->have_console = 1;
}

/* Tell the loaded U-Boot that it was loaded from SPL */
static void exynos5_set_spl_marker(void)
{
	uint32_t *marker = (uint32_t *)CONFIG_SPL_MARKER;

	*marker = EXYNOS5_SPL_MARKER;
}

/* Board-specific call to see if wakeup is allowed. */
static int __def_board_wakeup_permitted(void)
{
	return 1;
}
int board_wakeup_permitted(void)
	__attribute__((weak, alias("__def_board_wakeup_permitted")));

void board_init_f(unsigned long bootflag)
{
	/*
	 * The gd struct is only needed for serial initialization. Since this
	 * function is called in SPL u-boot. We store the gd struct in the
	 * stack instead of the default memory region which may not be
	 * initialized.
	 */
	__attribute__((aligned(8))) gd_t local_gd;
	__attribute__((noreturn)) void (*uboot)(void);

	exynos5_set_spl_marker();
	setup_global_data(&local_gd);

	/*
	 * Init subsystems, and resume if required. For a normal boot this
	 * will set up the UART and display a message.
	 */
	if (lowlevel_init_subsystems()) {
		if (!board_wakeup_permitted())
			power_reset();
		else
			exynos_log_wake_event();
		power_exit_wakeup();
	}

	printf("\n\nU-Boot SPL, board rev %u\n", board_get_revision());

	copy_uboot_to_ram();
	/* Jump to U-Boot image */
	uboot = (void *)CONFIG_SYS_TEXT_BASE;
	uboot();
	/* Never returns Here */
	panic("%s: u-boot jump failed", __func__);
}

/* Place Holders */
void board_init_r(gd_t *id, ulong dest_addr)
{
	/* Function attribute is no-return */
	/* This Function never executes */
	while (1)
		;
}

void save_boot_params(u32 r0, u32 r1, u32 r2, u32 r3) {}

/*
 * The following functions are required when linking console library to SPL.
 *
 * Enabling UART in SPL u-boot requires console library. But some
 * functions we needed in the console library depends on a bunch
 * of library in libgeneric, like lib/ctype.o, lib/div64.o, lib/string.o,
 * and lib/vsprintf.o. Adding them makes the SPL u-boot too large and not
 * fit into the expected size.
 *
 * So we mock these functions in SPL, i.e. vsprintf(), panic(), etc.,
 * in order to cut its dependency.
 */

static int _vscnprintf(char *buf, size_t size, const char *fmt, va_list args)
{
	char *str = buf, *s;
	char *end = str + size - 1;
	ulong u;

	if (size == 0)
		return -1;

	/*
	 * We won't implement all full functions of vsprintf().
	 * We only implement %s and %u, and ignore others and directly use
	 * the original format string as its result.
	 */

	while (*fmt && (str < end)) {
		if (*fmt != '%') {
			*str++ = *fmt++;
			continue;
		}
		fmt++;
		switch (*fmt) {
		case '%':
			*str++ = *fmt++;
			break;
		case 's':
			fmt++;
			s = va_arg(args, char *);
			while (*s && (str < end))
				*str++ = *s++;
			break;
		case 'u':
			fmt++;
			u = va_arg(args, ulong);
			s = simple_itoa(u);
			while (*s && (str < end))
				*str++ = *s++;
			break;
		default:
			/* Print the original string for unsupported formats */
			*str++ = '%';
			if  (str < end)
				*str++ = *fmt++;
		}
	}
	*str = '\0';
	return str - buf;
}

/*
 * Need to wrap definition of vscnprintf in ifdef so that the macro in
 * vsprintf.h doesn't trip us up.
 */
#ifdef CONFIG_SYS_VSNPRINTF
int vscnprintf(char *buf, size_t size, const char *fmt, va_list args)
{
	return _vscnprintf(buf, size, fmt, args);
}
#endif

/* Implement vsprintf in case someone doesn't have CONFIG_SYS_VSNPRINTF */
int vsprintf(char *buf, const char *fmt, va_list args)
{
	return _vscnprintf(buf, CONFIG_SYS_PBSIZE, fmt, args);
}

void panic(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vprintf(fmt, args);
	putc('\n');
	va_end(args);
#if defined(CONFIG_PANIC_HANG)
	hang();
#else
	udelay(100000);		/* allow messages to go out */
	do_reset(NULL, 0, 0, NULL);
#endif
	while (1)
		;
}

void __assert_fail(const char *assertion, const char *file, unsigned line,
		const char *function)
{
	/* This will not return */
	panic("%s:%u: %s: Assertion `%s' failed.", file, line, function,
			assertion);
}

char *simple_itoa(ulong i)
{
	/* 21 digits plus null terminator, good for 64-bit or smaller ints */
	static char local[22] __attribute__((section(".data")));
	char *p = &local[21];

	*p-- = '\0';
	do {
		*p-- = '0' + i % 10;
		i /= 10;
	} while (i > 0);
	return p + 1;
}
