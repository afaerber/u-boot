#include <common.h>
#include <flash.h>
#include <malloc.h>

#include <asm/errno.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/concat.h>
#include <mtd/cfi_flash.h>
#include <asm/hardware.h>
#include <mhu.h>
#include <scb_mhu_api.h>

#define PAGE_SHIFT  12
#define PAGE_MASK       (~((1 << PAGE_SHIFT) - 1)) 

#define KERNEL_SECTOR_SIZE	512

struct mb86s70_scb_dev {
	unsigned long total_kernel_sectors;   /* Device size in kernel blocks */
	unsigned long total_hard_sectors;     /* Device size in SCB blocks */
	unsigned long total_hard_bytes;	      /* Device size in SCB bytes */
	unsigned long erase_sector_size;
	struct mtd_info mtd;
	struct mtd_partition *parts;
	unsigned int num_parts;
};

static struct mb86s70_scb_dev scb_dev;
static char scb_mtd_names[16];

static int scb_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	size_t from = instr->addr;
	size_t len = instr->len;
	int ret = 0;
	struct cmd_stg_block_erase *s;

	if (from >= scb_dev.total_hard_bytes)
		return -EINVAL;

	if (from + len > scb_dev.total_hard_bytes)
		len = scb_dev.total_hard_bytes - from;

	instr->state = MTD_ERASING;

	while (len) {
		s = cmd_to_scb;
		s->payload_size = sizeof(*s);
		s->sector = from / SCB_SECTOR_SIZE;

		ret = mhu_send(CMD_STG_BLOCK_ERASE_REQ);
		if (ret < 0) {
			printf("erase cmd failed!\n");
			return ret;
		}

		s = rsp_from_scb;

		if (len > scb_dev.erase_sector_size)
			len -= scb_dev.erase_sector_size;
		else
			len = 0;

		if (s->result) {
			printf("erase sector %d failed, result:%x\n", s->sector, s->result);
			instr->state = MTD_ERASE_FAILED;
			ret = 1;
			goto bail;
		}

		from += scb_dev.erase_sector_size;
	}

	instr->state = MTD_ERASE_DONE;
	ret = 0;

bail:
	mtd_erase_callback(instr);

	return ret;
}

static int scb_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	struct cmd_stg_block_read *r;
	int ret;
	u32 initial_offset;

	if (from >= scb_dev.total_hard_bytes)
		return -EINVAL;

	if (from + len > scb_dev.total_hard_bytes)
		len = scb_dev.total_hard_bytes - from;

	if (retlen)
		*retlen = len;

	initial_offset = from % SCB_SECTOR_SIZE;

	while (len) {
		r = cmd_to_scb;
		r->payload_size = sizeof(*r);
		r->sector = from / SCB_SECTOR_SIZE;
		r->result = 1;

		memcpy(r->data, buf, SCB_SECTOR_SIZE);

		ret = mhu_send(CMD_STG_BLOCK_READ_REQ);
		if (ret < 0) {
			printf("read cmd failed, ret:0x%x\n", ret);
			goto bail;
		}

		r = rsp_from_scb;

		if (r->result) {
			printf("read failed at %d-byte sector %d\n",
						SCB_SECTOR_SIZE, r->sector);
			goto bail;
		}


		memcpy(buf, r->data + initial_offset, 
			len > SCB_SECTOR_SIZE ? SCB_SECTOR_SIZE : len);
		initial_offset = 0;

		from += SCB_SECTOR_SIZE;
		if (len >= SCB_SECTOR_SIZE)
			len -= SCB_SECTOR_SIZE;
		else
			len = 0;
		buf += SCB_SECTOR_SIZE;
	}

	return 0;

bail:
	if (retlen)
		*retlen -= len;

	return ret;
}

static int scb_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct cmd_stg_block_write *w;
	int ret;

	if (to >= scb_dev.total_hard_bytes)
		return -EINVAL;

	if (to + len > scb_dev.total_hard_bytes)
		len = scb_dev.total_hard_bytes - to;


	if (retlen)
		*retlen = len;

	while (len) {
		w = cmd_to_scb;
		w->payload_size = sizeof(*w);
		w->sector = to / SCB_SECTOR_SIZE;
		w->result = 1;

		memcpy(w->data, buf, SCB_SECTOR_SIZE);

		ret = mhu_send(CMD_STG_BLOCK_WRITE_REQ);
		if (ret < 0) {
			printf("write cmd failed\n");
			goto bail;
		}

		w = rsp_from_scb;

		if (w->result) {
			printf("write failed at %d-byte sector %d\n",
						SCB_SECTOR_SIZE, w->sector);
			goto bail;
		}


		to += SCB_SECTOR_SIZE;
		if (len >= SCB_SECTOR_SIZE)
			len -= SCB_SECTOR_SIZE;
		else
			len = 0;
		buf += SCB_SECTOR_SIZE;
	}

	return 0;

bail:
	if (retlen)
		*retlen -= len;

	return ret;
}

static void scb_mtd_sync(struct mtd_info *mtd)
{
	/* Not Implemented */
}

static int scb_mtd_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	/* Not Implemented */
	return 0;
}

static int scb_mtd_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	/* Not Implemented */
	return 0;
}

#define SCB_DEV_MIN_SUPPORTED_VER 0x509

static int
mb86s70_scb_get_mtd_info(struct mb86s70_scb_dev *priv)
{
	unsigned long erase, stg;
	struct cmd_stg_get_size *s;
	int ret;
	u32 version;

	version = get_scb_version();
	if(version < SCB_DEV_MIN_SUPPORTED_VER) {
		printf("Cannot access stroage!\n");
		return -1;
	}

	s = cmd_to_scb;
	s->payload_size = sizeof(*s);
	ret = mhu_send(CMD_STG_GET_SIZE_REQ);
	if (ret < 0) {
		printf("%s:%d failed!\n", __func__, __LINE__);
		return ret;
	}

	s = rsp_from_scb;

	erase = s->erase_block_size_bytes;
	stg = s->count_sectors;

	priv->erase_sector_size = erase;
	/* force usable storage to erase sector alignment */
	stg = ((((stg * SCB_SECTOR_SIZE) / erase) * erase) & PAGE_MASK) /
								SCB_SECTOR_SIZE;
	/* use as many kernel sectors as will fit */
	priv->total_kernel_sectors = ((stg * SCB_SECTOR_SIZE) & PAGE_MASK) /
							KERNEL_SECTOR_SIZE;
	priv->total_hard_bytes = priv->total_kernel_sectors *
						KERNEL_SECTOR_SIZE;
	/* express the kernel sectors as scb sectors (128 byte) */
	priv->total_hard_sectors = priv->total_hard_bytes / SCB_SECTOR_SIZE;

	return 0;
}

int scb_mtd_init(void)
{
	struct mtd_info *mtd;

	mtd = &scb_dev.mtd;

	mb86s70_scb_get_mtd_info(&scb_dev);

	sprintf(scb_mtd_names, "scb_stroage");
	mtd->name		= scb_mtd_names;
	mtd->type		= MTD_NORFLASH;
	mtd->flags		= MTD_CAP_NORFLASH;
	mtd->size		= scb_dev.total_hard_bytes;
	mtd->erasesize	= scb_dev.erase_sector_size;
	mtd->writesize	= KERNEL_SECTOR_SIZE;
	mtd->erase		= scb_mtd_erase;
	mtd->read		= scb_mtd_read;
	mtd->write		= scb_mtd_write;
	mtd->sync		= scb_mtd_sync;
	mtd->lock		= scb_mtd_lock;
	mtd->unlock		= scb_mtd_unlock;
	mtd->priv		= &scb_dev;

	if (add_mtd_device(mtd))
		return -ENOMEM;

	return 0;
}
