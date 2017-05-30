/*
 * Copyright (C) 2013 Linaro Ltd
 */

#include <common.h>
#include <malloc.h>
#include <asm/io.h>

#include "videomodes.h"
#include <video_fb.h>
#include <linux/list.h>
#include <linux/fb.h>
#include <asm/arch/hardware.h>

struct mb8ac0300_par {
	u32 pseudo_palette[16];
	u32 *gpu_base;
	dma_addr_t fbpaddr; /* real start of allocation */
	void * fb_va;
	u32 framesize;
	u32 skip;
	u32 round;
};

#define MB8AC0300GPU_BLOCK_OFS_CRTC0 0x400
#define MB8AC0300GPU_BLOCK_OFS_CRTC1 0x500

/* offset in 4-byte units */
enum {
	MB8AC0300_CRTC_OFS__H_CTR_SIZE = 0,
	MB8AC0300_CRTC_OFS__H_ADDR_TIME_START = 1,
	MB8AC0300_CRTC_OFS__H_RIGHT_BORDER_START = 2,
	MB8AC0300_CRTC_OFS__H_BLANK_START = 3,
	MB8AC0300_CRTC_OFS__H_LEFT_BORDER_START = 6,
	MB8AC0300_CRTC_OFS__H_SYNC_START = 4,
	MB8AC0300_CRTC_OFS__H_BACK_PORCH_START = 5,
	MB8AC0300_CRTC_OFS__H_INTERRUPT_END_START = 7,
	MB8AC0300_CRTC_OFS__H_DMA_END_START = 8,
	MB8AC0300_CRTC_OFS__V_COUNTER_SIZE = 9,
	MB8AC0300_CRTC_OFS__V_ADDR_TIME_START = 10,
	MB8AC0300_CRTC_OFS__V_BOTTOM_BORDER_START = 11,
	MB8AC0300_CRTC_OFS__V_BLANK_START = 12,
	MB8AC0300_CRTC_OFS__V_TOP_BORDER_START = 15,
	MB8AC0300_CRTC_OFS__V_SYNC_START = 13,
	MB8AC0300_CRTC_OFS__V_BACK_PORCH_START = 14,
	MB8AC0300_CRTC_OFS__V_INTERRUPT_END_START = 16,
	MB8AC0300_CRTC_OFS__V_INCREMENT_H_VALUE = 17,
	MB8AC0300_CRTC_OFS__OUT_SIZE = (0x5c >> 2),
	MB8AC0300_CRTC_OFS__DATA_SIZE = (0x90 >> 2),
	MB8AC0300_CRTC_OFS__ADR_FB0 = (0x68 >> 2),
	MB8AC0300_CRTC_OFS__ADR_FB1 = (0x6c >> 2),
	MB8AC0300_CRTC_OFS__SIGNAL_POL = 18,
	MB8AC0300_CRTC_OFS__DATA_FMT = (0x70 >> 2),
	MB8AC0300_CRTC_OFS__PIC_BORDER_H_END_START = 0x18,
	MB8AC0300_CRTC_OFS__PIC_BORDER_V_END_START = 0x19,
	MB8AC0300_CRTC_OFS__RAM_ADR_GAMMA = (0x80 >> 2), /* same ofs for fb0/1 */
	MB8AC0300_CRTC_OFS__RAM_DATA_GAMMA = 0x21,
	MB8AC0300_CRTC_OFS__START = (0x74 >> 2),
};

/* H Counter Size */
#define HCOUNT_SIZE (1056 - 1)
/* H Addr Time Start */
#define H_ADD_TIM_S (215 + 6 - 1)
/* H Sync Start */
#define H_SYNC_STAT (6 - 1)
/* H Back Porch Start */
#define H_BACKPO_ST (128 + 6 - 1)
/* H Right Border Start/Blank Start */
#define HRBORDER_ST (1021 - 1)     

/* V Counter Size */
#define VCOUNT_SIZE (525 - 1)
/* V Addr Time Start */
#define V_ADD_TIM_S (35 + 2 - 1)
/* V Bottom Border Start/Blank Start */
#define VBBORDER_ST (515 + 2 - 1)
/* V Sync Start (2) */
#define V_SYNC_STAT 1
/* V Back Porch Start */
#define V_BACKPO_ST (12 - 1)

/* V Increment H Value */
#define V_INCR_H_VA (H_SYNC_STAT - 3)
/* DAT_SIZE(OUT_HEIGHT/OUT_WIDTH) */
#define DATSIZE_H_W ((360 << 16) | 600)
/* OUT_SIZE(OUT_HEIGHT/OUT_WIDTH) */
#define OUTSIZE_H_W ((480 << 16) | 800)
/* SIGNAL_POL  V,H Posedge OUT */
#define SIGNAL__POL 0

/* H Counter Size */
#define HCOUNT_SIZE1 (1056 - 1)
/* H Addr Time Start */
#define H_ADD_TIM_S1 (215 + 6 - 1)
/* H Left Border Start */
#define HLBORDER_ST1 H_ADD_TIM_S1
/* H Left Border Start */
#define HLBORDER_ST H_ADD_TIM_S1
/* H Sync Start */
#define H_SYNC_STAT1 (6 - 1)
/* H Back Porch Start */
#define H_BACKPO_ST1 (128 + 6 - 1)
/* H Right Border Start/Blank Start */
#define HRBORDER_ST1 (1021 - 1)

/* V Counter Size */
#define VCOUNT_SIZE1 (525 - 1)
/* V Addr Time Start */
#define V_ADD_TIM_S1 (35 + 2 - 1)
/* V TOP Border Start */
#define VTBORDER_ST V_ADD_TIM_S1
/* V Bottom Border Start/Blank Start */
#define VBBORDER_ST1 (515 + 2 - 1)
/* V Sync Start (2) */
#define V_SYNC_STAT1 1
/* V Back Porch Start */
#define V_BACKPO_ST1 (12 - 1)
/* V TOP Border Start */
#define VTBORDER_ST1 V_ADD_TIM_S1
/* V Increment H Value */
#define V_INCR_H_VA1 (H_SYNC_STAT1 - 3)
/* DAT_SIZE(OUT_HEIGHT/OUT_WIDTH) */
#define DATSIZE_H_W1 ((360 << 16) | 600)
/* OUT_SIZE(OUT_HEIGHT/OUT_WIDTH) */
#define OUTSIZE_H_W1 ((480 << 16) | 800)
/* SIGNAL_POL V,H Posedge OUT */
#define SIGNAL__POL1 0


void lcd_ctrl_init(void *lcd_base)
{
	volatile u32 __iomem *crtc = (void *)
			       (GPU_BASE + MB8AC0300GPU_BLOCK_OFS_CRTC0);
	volatile u32 __iomem *crtc0 = crtc;
	volatile u32 __iomem *gpio = (void *)GPIO1_BASE;
	u32 col;
	int bpp = 16;
	int bytes_pp = (bpp + 7) / 8;
	int mode = 2;
	int xres = 800;
	int yres = 480;

	crtc[MB8AC0300_CRTC_OFS__START] = 0;

	/* set GPIO mux state to LCD pixel data + control signals */

	gpio[0x2c >> 2] = 0xff;
	gpio[0x28 >> 2] = 0xff;
	gpio[0x24 >> 2] = 0xff;
	gpio[0x1c >> 2] = 0xff;
	gpio[0x18 >> 2] = 0xff;
	gpio[0x14 >> 2] = 0xff;
	gpio[0x20 >> 2] = 0xf0;
	gpio[0x10 >> 2] = 0xf0;

	crtc[MB8AC0300_CRTC_OFS__H_CTR_SIZE] = HCOUNT_SIZE;
	crtc[MB8AC0300_CRTC_OFS__H_ADDR_TIME_START] = H_ADD_TIM_S;
	crtc[MB8AC0300_CRTC_OFS__H_RIGHT_BORDER_START] = HRBORDER_ST;
	crtc[MB8AC0300_CRTC_OFS__H_BLANK_START] = HRBORDER_ST;
	crtc[MB8AC0300_CRTC_OFS__H_LEFT_BORDER_START] = HLBORDER_ST;
	crtc[MB8AC0300_CRTC_OFS__H_SYNC_START] = H_SYNC_STAT;
	crtc[MB8AC0300_CRTC_OFS__H_BACK_PORCH_START] = H_BACKPO_ST;
	crtc[MB8AC0300_CRTC_OFS__H_INTERRUPT_END_START] =
					(H_BACKPO_ST << 16) | H_SYNC_STAT;
	crtc[MB8AC0300_CRTC_OFS__H_DMA_END_START] =
					((H_SYNC_STAT + 1) << 16) | H_SYNC_STAT;
	crtc[MB8AC0300_CRTC_OFS__V_COUNTER_SIZE] = VCOUNT_SIZE;
	crtc[MB8AC0300_CRTC_OFS__V_ADDR_TIME_START] = V_ADD_TIM_S;
	crtc[MB8AC0300_CRTC_OFS__V_BOTTOM_BORDER_START] = VBBORDER_ST;
	crtc[MB8AC0300_CRTC_OFS__V_BLANK_START] = VBBORDER_ST;
	crtc[MB8AC0300_CRTC_OFS__V_TOP_BORDER_START] = VTBORDER_ST;
	crtc[MB8AC0300_CRTC_OFS__V_SYNC_START] = V_SYNC_STAT;
	crtc[MB8AC0300_CRTC_OFS__V_BACK_PORCH_START] = V_BACKPO_ST;
	crtc[MB8AC0300_CRTC_OFS__V_INTERRUPT_END_START] =
					(V_BACKPO_ST << 16) | V_SYNC_STAT;
	crtc[MB8AC0300_CRTC_OFS__V_INCREMENT_H_VALUE] = V_INCR_H_VA;
	crtc[MB8AC0300_CRTC_OFS__OUT_SIZE] = (yres << 16) | xres;
	crtc[MB8AC0300_CRTC_OFS__DATA_SIZE] = xres * bytes_pp;

	crtc[MB8AC0300_CRTC_OFS__ADR_FB0] = (u32)lcd_base;
	crtc[MB8AC0300_CRTC_OFS__ADR_FB1] = (u32)lcd_base;
	crtc[MB8AC0300_CRTC_OFS__SIGNAL_POL] = SIGNAL__POL;
	crtc[MB8AC0300_CRTC_OFS__DATA_FMT] =
			(8 << 16) | (3 << 8) | (0 << 4) | mode;
	crtc[MB8AC0300_CRTC_OFS__PIC_BORDER_H_END_START] =
					(HRBORDER_ST << 16) | H_ADD_TIM_S;
	crtc[MB8AC0300_CRTC_OFS__PIC_BORDER_V_END_START] =
					(VBBORDER_ST << 16) | V_ADD_TIM_S;

	crtc0[MB8AC0300_CRTC_OFS__RAM_ADR_GAMMA] = 0;
	for (col = 0; col < 0x01000000; col += 0x010101)
		crtc[MB8AC0300_CRTC_OFS__RAM_DATA_GAMMA] = col;

	/* crtc1 */

	crtc = (void *)(GPU_BASE + MB8AC0300GPU_BLOCK_OFS_CRTC1);

	crtc[MB8AC0300_CRTC_OFS__H_CTR_SIZE] = HCOUNT_SIZE1;
	crtc[MB8AC0300_CRTC_OFS__H_ADDR_TIME_START] = H_ADD_TIM_S1;
	crtc[MB8AC0300_CRTC_OFS__H_RIGHT_BORDER_START] = HRBORDER_ST1;
	crtc[MB8AC0300_CRTC_OFS__H_BLANK_START] = HRBORDER_ST1;
	crtc[MB8AC0300_CRTC_OFS__H_LEFT_BORDER_START] = HLBORDER_ST1;
	crtc[MB8AC0300_CRTC_OFS__H_SYNC_START] = H_SYNC_STAT1;
	crtc[MB8AC0300_CRTC_OFS__H_BACK_PORCH_START] = H_BACKPO_ST1;
	crtc[MB8AC0300_CRTC_OFS__H_INTERRUPT_END_START] =
					(H_BACKPO_ST1 << 16) | H_SYNC_STAT1;
	crtc[MB8AC0300_CRTC_OFS__H_DMA_END_START] =
				((H_SYNC_STAT1 + 1) << 16) | H_SYNC_STAT1;
	crtc[MB8AC0300_CRTC_OFS__V_COUNTER_SIZE] = VCOUNT_SIZE1;
	crtc[MB8AC0300_CRTC_OFS__V_ADDR_TIME_START] = V_ADD_TIM_S1;
	crtc[MB8AC0300_CRTC_OFS__V_BOTTOM_BORDER_START] = VBBORDER_ST1;
	crtc[MB8AC0300_CRTC_OFS__V_BLANK_START] = VBBORDER_ST1;
	crtc[MB8AC0300_CRTC_OFS__V_TOP_BORDER_START] = VTBORDER_ST1;
	crtc[MB8AC0300_CRTC_OFS__V_SYNC_START] = V_SYNC_STAT1;
	crtc[MB8AC0300_CRTC_OFS__V_BACK_PORCH_START] = V_BACKPO_ST1;
	crtc[MB8AC0300_CRTC_OFS__V_INTERRUPT_END_START] =
					(V_BACKPO_ST1 << 16) | V_SYNC_STAT1;
	crtc[MB8AC0300_CRTC_OFS__V_INCREMENT_H_VALUE] = V_INCR_H_VA1;
	crtc[MB8AC0300_CRTC_OFS__OUT_SIZE] = (yres << 16) | xres;
	crtc[MB8AC0300_CRTC_OFS__DATA_SIZE] = xres * bytes_pp;

	crtc[MB8AC0300_CRTC_OFS__ADR_FB0] = (u32)lcd_base;
	crtc[MB8AC0300_CRTC_OFS__ADR_FB1] = (u32)lcd_base;
	crtc[MB8AC0300_CRTC_OFS__SIGNAL_POL] = SIGNAL__POL1;
	crtc[MB8AC0300_CRTC_OFS__DATA_FMT] =
			(8 << 16) | (3 << 8) | (0 << 4) | mode;
	crtc[MB8AC0300_CRTC_OFS__PIC_BORDER_H_END_START] =
					(HRBORDER_ST1 << 16) | H_ADD_TIM_S1;
	crtc[MB8AC0300_CRTC_OFS__PIC_BORDER_V_END_START] =
					(VBBORDER_ST1 << 16) | V_ADD_TIM_S1;

	crtc0[MB8AC0300_CRTC_OFS__RAM_ADR_GAMMA] = 0;
	for (col = 0; col < 0x03000000; col += 0x010101)
		crtc[MB8AC0300_CRTC_OFS__RAM_DATA_GAMMA] = col;

	/* start both CRTC */

	crtc0[MB8AC0300_CRTC_OFS__START] = 0x10101;
	crtc[MB8AC0300_CRTC_OFS__START] = 0x10101;
}

