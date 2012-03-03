/*
 * Copyright (c) 2010 Sascha Hauer <s.hauer@pengutronix.de>
 * Copyright (C) 2005-2009 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#define DEBUG
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/module.h>

#include <mach/common.h>
#include <mach/ipu-v3.h>
#include <drm/imx-ipu-v3.h>

#include "ipu-prv.h"

u32 ipu_cm_read(struct ipu_soc *ipu, unsigned offset)
{
	return readl(ipu->cm_reg + offset);
}

void ipu_cm_write(struct ipu_soc *ipu, u32 value, unsigned offset)
{
	writel(value, ipu->cm_reg + offset);
}

static inline u32 ipu_idmac_read(struct ipu_soc *ipu, unsigned offset)
{
	return readl(ipu->idmac_reg + offset);
}

static inline void ipu_idmac_write(struct ipu_soc *ipu, u32 value, unsigned offset)
{
	writel(value, ipu->idmac_reg + offset);
}

void ipu_srm_dp_sync_update(struct ipu_soc *ipu)
{
	u32 val;

	val = ipu_cm_read(ipu, IPU_SRM_PRI2);
	val |= 0x8;
	ipu_cm_write(ipu, val, IPU_SRM_PRI2);
}
EXPORT_SYMBOL_GPL(ipu_srm_dp_sync_update);



#define IPU_CPMEM_WORD(word, ofs, size) ((((word) * 160 + (ofs)) << 8) | (size))

#define IPU_FIELD_UBO		IPU_CPMEM_WORD(0, 46, 22)
#define IPU_FIELD_VBO		IPU_CPMEM_WORD(0, 68, 22)
#define IPU_FIELD_IOX		IPU_CPMEM_WORD(0, 90, 4)
#define IPU_FIELD_RDRW		IPU_CPMEM_WORD(0, 94, 1)
#define IPU_FIELD_SO		IPU_CPMEM_WORD(0, 113, 1)
#define IPU_FIELD_SLY		IPU_CPMEM_WORD(1, 102, 14)
#define IPU_FIELD_SLUV		IPU_CPMEM_WORD(1, 128, 14)

#define IPU_FIELD_XV		IPU_CPMEM_WORD(0, 0, 10)
#define IPU_FIELD_YV		IPU_CPMEM_WORD(0, 10, 9)
#define IPU_FIELD_XB		IPU_CPMEM_WORD(0, 19, 13)
#define IPU_FIELD_YB		IPU_CPMEM_WORD(0, 32, 12)
#define IPU_FIELD_NSB_B		IPU_CPMEM_WORD(0, 44, 1)
#define IPU_FIELD_CF		IPU_CPMEM_WORD(0, 45, 1)
#define IPU_FIELD_SX		IPU_CPMEM_WORD(0, 46, 12)
#define IPU_FIELD_SY		IPU_CPMEM_WORD(0, 58, 11)
#define IPU_FIELD_NS		IPU_CPMEM_WORD(0, 69, 10)
#define IPU_FIELD_SDX		IPU_CPMEM_WORD(0, 79, 7)
#define IPU_FIELD_SM		IPU_CPMEM_WORD(0, 86, 10)
#define IPU_FIELD_SCC		IPU_CPMEM_WORD(0, 96, 1)
#define IPU_FIELD_SCE		IPU_CPMEM_WORD(0, 97, 1)
#define IPU_FIELD_SDY		IPU_CPMEM_WORD(0, 98, 7)
#define IPU_FIELD_SDRX		IPU_CPMEM_WORD(0, 105, 1)
#define IPU_FIELD_SDRY		IPU_CPMEM_WORD(0, 106, 1)
#define IPU_FIELD_BPP		IPU_CPMEM_WORD(0, 107, 3)
#define IPU_FIELD_DEC_SEL	IPU_CPMEM_WORD(0, 110, 2)
#define IPU_FIELD_DIM		IPU_CPMEM_WORD(0, 112, 1)
#define IPU_FIELD_BNDM		IPU_CPMEM_WORD(0, 114, 3)
#define IPU_FIELD_BM		IPU_CPMEM_WORD(0, 117, 2)
#define IPU_FIELD_ROT		IPU_CPMEM_WORD(0, 119, 1)
#define IPU_FIELD_HF		IPU_CPMEM_WORD(0, 120, 1)
#define IPU_FIELD_VF		IPU_CPMEM_WORD(0, 121, 1)
#define IPU_FIELD_THE		IPU_CPMEM_WORD(0, 122, 1)
#define IPU_FIELD_CAP		IPU_CPMEM_WORD(0, 123, 1)
#define IPU_FIELD_CAE		IPU_CPMEM_WORD(0, 124, 1)
#define IPU_FIELD_FW		IPU_CPMEM_WORD(0, 125, 13)
#define IPU_FIELD_FH		IPU_CPMEM_WORD(0, 138, 12)
#define IPU_FIELD_EBA0		IPU_CPMEM_WORD(1, 0, 29)
#define IPU_FIELD_EBA1		IPU_CPMEM_WORD(1, 29, 29)
#define IPU_FIELD_ILO		IPU_CPMEM_WORD(1, 58, 20)
#define IPU_FIELD_NPB		IPU_CPMEM_WORD(1, 78, 7)
#define IPU_FIELD_PFS		IPU_CPMEM_WORD(1, 85, 4)
#define IPU_FIELD_ALU		IPU_CPMEM_WORD(1, 89, 1)
#define IPU_FIELD_ALBM		IPU_CPMEM_WORD(1, 90, 3)
#define IPU_FIELD_ID		IPU_CPMEM_WORD(1, 93, 2)
#define IPU_FIELD_TH		IPU_CPMEM_WORD(1, 95, 7)
#define IPU_FIELD_SL		IPU_CPMEM_WORD(1, 102, 14)
#define IPU_FIELD_WID0		IPU_CPMEM_WORD(1, 116, 3)
#define IPU_FIELD_WID1		IPU_CPMEM_WORD(1, 119, 3)
#define IPU_FIELD_WID2		IPU_CPMEM_WORD(1, 122, 3)
#define IPU_FIELD_WID3		IPU_CPMEM_WORD(1, 125, 3)
#define IPU_FIELD_OFS0		IPU_CPMEM_WORD(1, 128, 5)
#define IPU_FIELD_OFS1		IPU_CPMEM_WORD(1, 133, 5)
#define IPU_FIELD_OFS2		IPU_CPMEM_WORD(1, 138, 5)
#define IPU_FIELD_OFS3		IPU_CPMEM_WORD(1, 143, 5)
#define IPU_FIELD_SXYS		IPU_CPMEM_WORD(1, 148, 1)
#define IPU_FIELD_CRE		IPU_CPMEM_WORD(1, 149, 1)
#define IPU_FIELD_DEC_SEL2	IPU_CPMEM_WORD(1, 150, 1)

struct ipu_cpmem_word {
	u32 data[5];
	u32 res[3];
};

struct ipu_ch_param {
	struct ipu_cpmem_word word[2];
};

static int ipu_alloc_ch_param(struct ipu_channel *channel)
{
	struct ipu_soc *ipu = channel->ipu;

	pr_crit("allocate channel %d parameters\n", channel->num);

	channel->cpmem = ipu->cpmem_base + channel->num;
	if (!channel->param)
		channel->param = kzalloc(sizeof(struct ipu_ch_param), GFP_KERNEL);

	if (!channel->param)
		return -ENOMEM;

	return 0;
}

static void ipu_free_ch_param(struct ipu_channel *channel)
{
	pr_crit("free channel %d parameters\n", channel->num);

	if (channel->param)
		kfree(channel->param);
}

static void ipu_ch_param_commit(struct ipu_channel *ch)
{
	struct ipu_ch_param *param = ch->param;
	void *address = ch->cpmem;
	int i, w;

	pr_crit("committing channel %d parameters to CPMEM\n", ch->num);

	for (w = 0; w < 2; w++) {
		for (i = 0; i < 5; i++) {
			writel(param->word[w].data[i], address);
			address += 4;
		}
		address += 12;
	}
}

static void ipu_ch_param_save(struct ipu_channel *ch)
{
	struct ipu_ch_param *param = ch->param;
	void *address = ch->cpmem;
	int i, w;

	pr_crit("saving CPMEM %d parameters\n", ch->num);

	for (w = 0; w < 2; w++) {
		for (i = 0; i < 5; i++) {
			param->word[w].data[i] = readl(address);
			address += 4;
		}
		address += 12;
	}
}

static void ipu_ch_param_set_field(struct ipu_ch_param *base, u32 wbs, u32 v)
{
	u32 bit = (wbs >> 8) % 160;
	u32 size = wbs & 0xff;
	u32 word = (wbs >> 8) / 160;
	u32 i = bit / 32;
	u32 ofs = bit % 32;
	u32 mask = (1 << size) - 1;
	u32 field, readback, *address;

	address = &base->word[word].data[i];

	field = readl(address) & ~(mask << ofs);
	field |= v << ofs;
	writel(field, address);
	readback = readl(address);

	pr_crit("%s (0x%x) wrote [%d:%d]0x%08x readback 0x%08x\n",
		__func__, ((u32)address & 0xfff), bit, bit+size, field, readback);

	if ((bit + size - 1) / 32 > i) {
		address = &base->word[word].data[i+1];

		field = readl(address) & ~(v >> (mask ? (32 - ofs) : 0));
		field |= v >> (ofs ? (32 - ofs) : 0);
		writel(field, address);
		readback = readl(address);
		pr_crit("%s (0x%x) wrote [%d:%d]0x%08x readback 0x%08x (field overlaps 32-bit boundary)\n",
			__func__, ((u32)address & 0xfff), bit, bit+size, field, readback);
	}
}

static u32 ipu_ch_param_read_field(struct ipu_ch_param *base, u32 wbs)
{
	u32 bit = (wbs >> 8) % 160;
	u32 size = wbs & 0xff;
	u32 word = (wbs >> 8) / 160;
	u32 i = bit / 32;
	u32 ofs = bit % 32;
	u32 mask = (1 << size) - 1;
	u32 field, *address, tmp;

	address = &base->word[word].data[i];

	field = (readl(address) >> ofs) & mask;

	if ((bit + size - 1) / 32 > i) {
		address = &base->word[word].data[i+1];
		tmp = readl(address) & (mask >> (ofs ? (32 - ofs) : 0));
		field |= tmp << (ofs ? (32 - ofs) : 0);
	}

	pr_crit("%s read [%d:%d]0x%08x\n", __func__, bit, bit+size, field);

	return field;
}

void ipu_channel_set_format_rgb(struct ipu_channel *ch, struct ipu_rgb *rgb)
{
	int bpp = 0, npb = 0, ro, go, bo, to;
	struct ipu_ch_param *p = (ch->enabled) ? p = ch->cpmem : ch->param;

	ro = rgb->bits_per_pixel - rgb->red.length - rgb->red.offset;
	go = rgb->bits_per_pixel - rgb->green.length - rgb->green.offset;
	bo = rgb->bits_per_pixel - rgb->blue.length - rgb->blue.offset;
	to = rgb->bits_per_pixel - rgb->transp.length - rgb->transp.offset;

	ipu_ch_param_set_field(p, IPU_FIELD_WID0, rgb->red.length - 1);
	ipu_ch_param_set_field(p, IPU_FIELD_OFS0, ro);
	ipu_ch_param_set_field(p, IPU_FIELD_WID1, rgb->green.length - 1);
	ipu_ch_param_set_field(p, IPU_FIELD_OFS1, go);
	ipu_ch_param_set_field(p, IPU_FIELD_WID2, rgb->blue.length - 1);
	ipu_ch_param_set_field(p, IPU_FIELD_OFS2, bo);

	if (rgb->transp.length) {
		ipu_ch_param_set_field(p, IPU_FIELD_WID3, rgb->transp.length - 1);
		ipu_ch_param_set_field(p, IPU_FIELD_OFS3, to);
	} else {
		ipu_ch_param_set_field(p, IPU_FIELD_WID3, 7);
		ipu_ch_param_set_field(p, IPU_FIELD_OFS3, rgb->bits_per_pixel);
	}

	switch (rgb->bits_per_pixel) {
	case 32:
		bpp = 0;
		npb = 15;
		break;
	case 24:
		bpp = 1;
		npb = 19;
		break;
	case 16:
		bpp = 3;
		npb = 31;
		break;
	case 8:
		bpp = 5;
		npb = 63;
		break;
	}
	ipu_ch_param_set_field(p, IPU_FIELD_BPP, bpp);
	ipu_ch_param_set_field(p, IPU_FIELD_NPB, npb);
	ipu_ch_param_set_field(p, IPU_FIELD_PFS, 7); /* rgb mode */
}
EXPORT_SYMBOL_GPL(ipu_channel_set_format_rgb);

void ipu_channel_set_yuv_interleaved(struct ipu_channel *ch, u32 pixel_format)
{
	struct ipu_ch_param *p = (ch->enabled) ? p = ch->cpmem : ch->param;

	switch (pixel_format) {
	case IPU_PIX_FMT_UYVY:
		ipu_ch_param_set_field(p, IPU_FIELD_BPP, 3);	/* bits/pixel */
		ipu_ch_param_set_field(p, IPU_FIELD_PFS, 0xA);	/* pix format */
		ipu_ch_param_set_field(p, IPU_FIELD_NPB, 15);	/* burst size */
		break;
	case IPU_PIX_FMT_YUYV:
		ipu_ch_param_set_field(p, IPU_FIELD_BPP, 3);	/* bits/pixel */
		ipu_ch_param_set_field(p, IPU_FIELD_PFS, 0x8);	/* pix format */
		ipu_ch_param_set_field(p, IPU_FIELD_NPB, 31);	/* burst size */
		break;
	}
}
EXPORT_SYMBOL_GPL(ipu_channel_set_yuv_interleaved);

void ipu_channel_set_yuv_planar(struct ipu_channel *ch, u32 pixel_format, int stride,
	int width, int height)
{
	struct ipu_ch_param *p = (ch->enabled) ? p = ch->cpmem : ch->param;
	int u_offset, v_offset;
	int uv_stride = 0;

	switch (pixel_format) {
	case IPU_PIX_FMT_YUV420P:
		ipu_ch_param_set_field(p, IPU_FIELD_PFS, 2);   /* pix format */
		uv_stride = stride / 2;
		u_offset = stride * height;
		v_offset = u_offset + (uv_stride * height / 2);
		ipu_ch_param_set_field(p, IPU_FIELD_NPB, 31);  /* burst size */
		ipu_ch_param_set_field(p, IPU_FIELD_SLUV, uv_stride - 1);
		ipu_ch_param_set_field(p, IPU_FIELD_UBO, u_offset / 8);
		ipu_ch_param_set_field(p, IPU_FIELD_VBO, v_offset / 8);
		break;
	}
}
EXPORT_SYMBOL_GPL(ipu_channel_set_yuv_planar);

void ipu_channel_set_buffer(struct ipu_channel *ch, int bufnum, dma_addr_t buf)
{
	struct ipu_ch_param *p = (ch->enabled) ? p = ch->cpmem : ch->param;

	/* cause a fuss if the channel is enabled, since you can't change the buffer! */
	WARN_ON(ch->enabled);

	if (bufnum)
		ipu_ch_param_set_field(p, IPU_FIELD_EBA1, buf >> 3);
	else
		ipu_ch_param_set_field(p, IPU_FIELD_EBA0, buf >> 3);
}
EXPORT_SYMBOL_GPL(ipu_channel_set_buffer);

void ipu_channel_set_resolution(struct ipu_channel *ch, int xres, int yres)
{
	struct ipu_ch_param *p = (ch->enabled) ? p = ch->cpmem : ch->param;

	ipu_ch_param_set_field(p, IPU_FIELD_FW, xres - 1);
	ipu_ch_param_set_field(p, IPU_FIELD_FH, yres - 1);
}
EXPORT_SYMBOL_GPL(ipu_channel_set_resolution);

void ipu_channel_set_stride(struct ipu_channel *ch, int stride)
{
	struct ipu_ch_param *p = (ch->enabled) ? p = ch->cpmem : ch->param;

	ipu_ch_param_set_field(p, IPU_FIELD_SLY, stride - 1);
}
EXPORT_SYMBOL_GPL(ipu_channel_set_stride);

void ipu_channel_set_high_priority(struct ipu_channel *ch)
{
	struct ipu_ch_param *p = (ch->enabled) ? p = ch->cpmem : ch->param;

	if (!cpu_is_mx53())
		ipu_ch_param_set_field(p, IPU_FIELD_ID, 1);
};
EXPORT_SYMBOL_GPL(ipu_channel_set_high_priority);

void ipu_channel_interlaced_scan(struct ipu_channel *ch, int stride)
{
	struct ipu_ch_param *p = (ch->enabled) ? p = ch->cpmem : ch->param;

	ipu_ch_param_set_field(p, IPU_FIELD_SO, 1);
	ipu_ch_param_set_field(p, IPU_FIELD_ILO, stride / 8);
	ipu_ch_param_set_field(p, IPU_FIELD_SLY, (stride * 2) - 1);
};

void ipu_channel_set_burstsize(struct ipu_channel *ch, int burstsize)
{
	struct ipu_ch_param *p = (ch->enabled) ? p = ch->cpmem : ch->param;

	ipu_ch_param_set_field(p, IPU_FIELD_NPB, burstsize - 1);
};




struct ipu_channel *ipu_idmac_get(struct ipu_soc *ipu, unsigned num)
{
	struct ipu_channel *channel;

	dev_dbg(ipu->dev, "%s %d\n", __func__, num);

	if (num > 63)
		return ERR_PTR(-ENODEV);

	mutex_lock(&ipu->channel_lock);

	channel = &ipu->channel[num];

	if (channel->busy) {
		channel = ERR_PTR(-EBUSY);
		goto out;
	}

	channel->busy = 1;
	channel->num = num;

	ipu_alloc_ch_param(channel);

out:
	mutex_unlock(&ipu->channel_lock);

	return channel;
}
EXPORT_SYMBOL_GPL(ipu_idmac_get);

void ipu_idmac_put(struct ipu_channel *channel)
{
	struct ipu_soc *ipu = channel->ipu;

	dev_dbg(ipu->dev, "%s %d\n", __func__, channel->num);

	mutex_lock(&ipu->channel_lock);

	channel->busy = 0;
	ipu_free_ch_param(channel);

	mutex_unlock(&ipu->channel_lock);
}
EXPORT_SYMBOL_GPL(ipu_idmac_put);

#define idma_mask(ch)			(1 << (ch & 0x1f))

void ipu_idmac_set_double_buffer(struct ipu_channel *channel, bool doublebuffer)
{
	struct ipu_soc *ipu = channel->ipu;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ipu->lock, flags);

	reg = ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(channel->num));
	if (doublebuffer)
		reg |= idma_mask(channel->num);
	else
		reg &= ~idma_mask(channel->num);
	ipu_cm_write(ipu, reg, IPU_CHA_DB_MODE_SEL(channel->num));

	spin_unlock_irqrestore(&ipu->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_idmac_set_double_buffer);

int ipu_module_enable(struct ipu_soc *ipu, u32 mask)
{
	unsigned long lock_flags;
	u32 val;

	spin_lock_irqsave(&ipu->lock, lock_flags);

	val = ipu_cm_read(ipu, IPU_CONF);
	val |= mask;
	ipu_cm_write(ipu, val, IPU_CONF);

	spin_unlock_irqrestore(&ipu->lock, lock_flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_module_enable);

int ipu_module_disable(struct ipu_soc *ipu, u32 mask)
{
	unsigned long lock_flags;
	u32 val;

	spin_lock_irqsave(&ipu->lock, lock_flags);

	val = ipu_cm_read(ipu, IPU_CONF);
	val &= ~mask;
	ipu_cm_write(ipu, val, IPU_CONF);

	spin_unlock_irqrestore(&ipu->lock, lock_flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_module_disable);

void ipu_idmac_select_buffer(struct ipu_channel *channel, u32 buf_num)
{
	struct ipu_soc *ipu = channel->ipu;
	unsigned int chno = channel->num;
	unsigned long flags;

	spin_lock_irqsave(&ipu->lock, flags);

	/* Mark buffer as ready. */
	if (buf_num == 0)
		ipu_cm_write(ipu, idma_mask(chno), IPU_CHA_BUF0_RDY(chno));
	else
		ipu_cm_write(ipu, idma_mask(chno), IPU_CHA_BUF1_RDY(chno));

	spin_unlock_irqrestore(&ipu->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_idmac_select_buffer);

int ipu_idmac_enable_channel(struct ipu_channel *channel)
{
	struct ipu_soc *ipu = channel->ipu;
	u32 val;
	unsigned long flags;

	ipu_get(ipu);

	spin_lock_irqsave(&ipu->lock, flags);

	ipu_ch_param_commit(channel);

	val = ipu_idmac_read(ipu, IDMAC_CHA_EN(channel->num));
	val |= idma_mask(channel->num);
	ipu_idmac_write(ipu, val, IDMAC_CHA_EN(channel->num));

	spin_unlock_irqrestore(&ipu->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_idmac_enable_channel);

int ipu_idmac_disable_channel(struct ipu_channel *channel)
{
	struct ipu_soc *ipu = channel->ipu;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&ipu->lock, flags);

	/* Disable DMA channel(s) */
	val = ipu_idmac_read(ipu, IDMAC_CHA_EN(channel->num));
	val &= ~idma_mask(channel->num);
	ipu_idmac_write(ipu, val, IDMAC_CHA_EN(channel->num));

	ipu_ch_param_save(channel);

	/* Set channel buffers NOT to be ready */
	ipu_cm_write(ipu, 0xf0000000, IPU_GPR); /* write one to clear */

	if (ipu_cm_read(ipu, IPU_CHA_BUF0_RDY(channel->num)) & idma_mask(channel->num)) {
		ipu_cm_write(ipu, idma_mask(channel->num),
			     IPU_CHA_BUF0_RDY(channel->num));
	}
	if (ipu_cm_read(ipu, IPU_CHA_BUF1_RDY(channel->num)) & idma_mask(channel->num)) {
		ipu_cm_write(ipu, idma_mask(channel->num),
			     IPU_CHA_BUF1_RDY(channel->num));
	}

	ipu_cm_write(ipu, 0x0, IPU_GPR); /* write one to set */

	/* Reset the double buffer */
	val = ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(channel->num));
	val &= ~idma_mask(channel->num);
	ipu_cm_write(ipu, val, IPU_CHA_DB_MODE_SEL(channel->num));

	spin_unlock_irqrestore(&ipu->lock, flags);

	ipu_put(ipu);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_idmac_disable_channel);

static int ipu_reset(struct ipu_soc *ipu)
{
	int timeout = 10000;
	u32 val;

	/* hard reset the IPU */
	val = readl(MX51_IO_ADDRESS(MX51_SRC_BASE_ADDR));
	val |= 1 << 3;
	writel(val, MX51_IO_ADDRESS(MX51_SRC_BASE_ADDR));

	mdelay(300); /* FIXME: such a big delay needed? */

	ipu_cm_write(ipu, 0x807FFFFF, IPU_MEM_RST);

	while (ipu_cm_read(ipu, IPU_MEM_RST) & 0x80000000) {
		if (!timeout--)
			return -ETIME;
		udelay(100);
	}

	return 0;
}

static int ipu_submodules_init(struct ipu_soc *ipu, struct platform_device *pdev,
		unsigned long ipu_base, struct clk *ipu_clk)
{
	char *unit;
	int ret;
	struct device *dev = &pdev->dev;

	ret = ipu_di_init(ipu, dev, 0, ipu_base + IPU_DI0_REG_BASE,
			IPU_CONF_DI0_EN, ipu_clk);
	if (ret) {
		unit = "di0";
		goto err_di_0;
	}

	ret = ipu_di_init(ipu, dev, 1, ipu_base + IPU_DI1_REG_BASE,
			IPU_CONF_DI1_EN, ipu_clk);
	if (ret) {
		unit = "di1";
		goto err_di_1;
	}

	ret = ipu_dc_init(ipu, dev, ipu_base + IPU_DC_REG_BASE,
			ipu_base + IPU_DC_TMPL_REG_BASE);
	if (ret) {
		unit = "dc_template";
		goto err_dc;
	}

	ret = ipu_dmfc_init(ipu, dev, ipu_base + IPU_DMFC_REG_BASE, ipu_clk);
	if (ret) {
		unit = "dmfc";
		goto err_dmfc;
	}

	ret = ipu_dp_init(ipu, dev, ipu_base + IPU_SRM_REG_BASE);
	if (ret) {
		unit = "dp";
		goto err_dp;
	}

	return 0;

err_dp:
	ipu_dmfc_exit(ipu);
err_dmfc:
	ipu_dc_exit(ipu);
err_dc:
	ipu_di_exit(ipu, 1);
err_di_1:
	ipu_di_exit(ipu, 0);
err_di_0:
	dev_err(&pdev->dev, "init %s failed with %d\n", unit, ret);
	return ret;
}

void ipu_get(struct ipu_soc *ipu)
{
	mutex_lock(&ipu->channel_lock);

	if (atomic_inc_return(&ipu->usecount) == 1)
		clk_enable(ipu->clk);

	mutex_unlock(&ipu->channel_lock);
}
EXPORT_SYMBOL_GPL(ipu_get);

void ipu_put(struct ipu_soc *ipu)
{
	mutex_lock(&ipu->channel_lock);

	if (atomic_dec_return(&ipu->usecount) == 0)
		clk_disable(ipu->clk);

	WARN_ON(atomic_read(&ipu->usecount) < 0);

	mutex_unlock(&ipu->channel_lock);
}
EXPORT_SYMBOL_GPL(ipu_put);

static void ipu_irq_handle(struct ipu_soc *ipu, const int *regs, int num_regs)
{
	unsigned long status;
	int i, bit, irq_base;

	for (i = 0; i < num_regs; i++) {

		status = ipu_cm_read(ipu, IPU_INT_STAT(regs[i]));
		status &= ipu_cm_read(ipu, IPU_INT_CTRL(regs[i]));

		irq_base = ipu->irq_start + regs[i] * 32;
		for_each_set_bit(bit, &status, 32)
			generic_handle_irq(irq_base + bit);
	}
}

static void ipu_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct ipu_soc *ipu = irq_desc_get_handler_data(desc);
	const int int_reg[] = { 0, 1, 2, 3, 10, 11, 12, 13, 14};

	ipu_irq_handle(ipu, int_reg, ARRAY_SIZE(int_reg));
}

static void ipu_err_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct ipu_soc *ipu = irq_desc_get_handler_data(desc);
	const int int_reg[] = { 4, 5, 8, 9};

	ipu_irq_handle(ipu, int_reg, ARRAY_SIZE(int_reg));
}

static void ipu_ack_irq(struct irq_data *d)
{
	struct ipu_soc *ipu = irq_data_get_irq_chip_data(d);
	unsigned int irq = d->irq - ipu->irq_start;

	ipu_cm_write(ipu, 1 << (irq % 32), IPU_INT_STAT(irq / 32));
}

static void ipu_unmask_irq(struct irq_data *d)
{
	struct ipu_soc *ipu = irq_data_get_irq_chip_data(d);
	unsigned int irq = d->irq - ipu->irq_start;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ipu->lock, flags);
	reg = ipu_cm_read(ipu, IPU_INT_CTRL(irq / 32));
	reg |= 1 << (irq % 32);
	ipu_cm_write(ipu, reg, IPU_INT_CTRL(irq / 32));
	spin_unlock_irqrestore(&ipu->lock, flags);
}

static void ipu_mask_irq(struct irq_data *d)
{
	struct ipu_soc *ipu = irq_data_get_irq_chip_data(d);
	unsigned int irq = d->irq - ipu->irq_start;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ipu->lock, flags);
	reg = ipu_cm_read(ipu, IPU_INT_CTRL(irq / 32));
	reg &= ~(1 << (irq % 32));
	ipu_cm_write(ipu, reg, IPU_INT_CTRL(irq / 32));
	spin_unlock_irqrestore(&ipu->lock, flags);
}

static struct irq_chip ipu_irq_chip = {
	.name = "IPU",
	.irq_ack = ipu_ack_irq,
	.irq_mask = ipu_mask_irq,
	.irq_unmask = ipu_unmask_irq,
};

static void ipu_submodules_exit(struct ipu_soc *ipu)
{
	ipu_dp_exit(ipu);
	ipu_dmfc_exit(ipu);
	ipu_dc_exit(ipu);
	ipu_di_exit(ipu, 1);
	ipu_di_exit(ipu, 0);
}

static int platform_remove_devices_fn(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static void platform_device_unregister_children(struct platform_device *pdev)
{
	device_for_each_child(&pdev->dev, NULL, platform_remove_devices_fn);
}

static int ipu_add_subdevice_pdata(struct device *dev,
		const char *name, int id, void *pdata, int irq)
{
	struct platform_device *pdev;
	struct resource res[] = {
		{
			.flags = IORESOURCE_IRQ,
			.start = irq,
			.end = irq,
		},
	};

	pdev = platform_device_register_resndata(dev, name, id, res,
			ARRAY_SIZE(res), NULL, 0);
	return pdev ? 0 : -EINVAL;
}

static int ipu_add_client_devices(struct ipu_soc *ipu)
{
	int ret;

	ret = ipu_add_subdevice_pdata(ipu->dev, "imx-ipuv3-ovl", 0, NULL,
			ipu->irq_start +
			IPU_IRQ_EOF(IPUV3_CHANNEL_MEM_FG_SYNC));
	ret |= ipu_add_subdevice_pdata(ipu->dev, "imx-ipuv3-camera", 0, NULL,
			ipu->irq_start +
			IPU_IRQ_EOF(IPUV3_CHANNEL_CSI0));
	ret |= ipu_add_subdevice_pdata(ipu->dev, "imx-drm", 0, NULL,
			ipu->irq_start +
			IPU_IRQ_EOF(IPUV3_CHANNEL_MEM_FG_SYNC));

	if (ret)
		platform_device_unregister_children(to_platform_device(ipu->dev));

	return ret;
}

static int ipu_irq_init(struct ipu_soc *ipu)
{
	int i;

	ipu->irq_start = irq_alloc_descs(-1, 0, IPU_NUM_IRQS, 0);
	if (ipu->irq_start < 0)
		return ipu->irq_start;

	for (i = ipu->irq_start; i < ipu->irq_start + IPU_NUM_IRQS; i++) {
		irq_set_chip_and_handler(i, &ipu_irq_chip, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
		irq_set_chip_data(i, ipu);
	}

	irq_set_chained_handler(ipu->irq_sync, ipu_irq_handler);
	irq_set_handler_data(ipu->irq_sync, ipu);
	irq_set_chained_handler(ipu->irq_err, ipu_err_irq_handler);
	irq_set_handler_data(ipu->irq_err, ipu);

	return 0;
}

static void ipu_irq_exit(struct ipu_soc *ipu)
{
	int i;

	irq_set_chained_handler(ipu->irq_err, NULL);
	irq_set_handler_data(ipu->irq_err, NULL);
	irq_set_chained_handler(ipu->irq_sync, NULL);
	irq_set_handler_data(ipu->irq_sync, NULL);

	for (i = ipu->irq_start; i < ipu->irq_start + IPU_NUM_IRQS; i++) {
		set_irq_flags(i, 0);
		irq_set_chip(i, NULL);
		irq_set_chip_data(i, NULL);
	}

	irq_free_descs(ipu->irq_start, IPU_NUM_IRQS);
}

static int __devinit ipu_probe(struct platform_device *pdev)
{
	struct ipu_soc *ipu;
	struct resource *res;
	unsigned long ipu_base;
	int i, ret, irq_sync, irq_err;

	irq_sync = platform_get_irq(pdev, 0);
	irq_err = platform_get_irq(pdev, 1);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res || irq_sync < 0 || irq_err < 0)
		return -ENODEV;

	ipu_base = res->start + 0x1e000000; // this should be better done

	ipu = devm_kzalloc(&pdev->dev, sizeof(*ipu), GFP_KERNEL);
	if (!ipu)
		return -ENODEV;

	for (i = 0; i < 64; i++)
		ipu->channel[i].ipu = ipu;

	spin_lock_init(&ipu->lock);
	mutex_init(&ipu->channel_lock);

	ipu->cm_reg = devm_ioremap(&pdev->dev, ipu_base + IPU_CM_REG_BASE, PAGE_SIZE);
	ipu->idmac_reg = devm_ioremap(&pdev->dev, ipu_base + IPU_IDMAC_REG_BASE, PAGE_SIZE);
	ipu->cpmem_base = devm_ioremap(&pdev->dev, ipu_base + IPU_CPMEM_REG_BASE, PAGE_SIZE);
	if (!ipu->cm_reg || !ipu->idmac_reg || !ipu->cpmem_base) {
		printk(KERN_CRIT "%s ioremap bases\n", __func__);
		ret = -ENOMEM;
		goto failed_ioremap;
	}

	ipu->clk = clk_get(&pdev->dev, "ipu");
	if (IS_ERR(ipu->clk)) {
		ret = PTR_ERR(ipu->clk);
		dev_err(&pdev->dev, "clk_get failed with %d", ret);
		goto failed_clk_get;
	}

	platform_set_drvdata(pdev, ipu);

	ipu_get(ipu);

	ipu->dev = &pdev->dev;
	ipu->irq_sync = irq_sync;
	ipu->irq_err = irq_err;

	ret = ipu_irq_init(ipu);
	if (ret)
		goto out_failed_irq;

	ipu_reset(ipu);

	ret = ipu_submodules_init(ipu, pdev, ipu_base, ipu->clk);
	if (ret)
		goto failed_submodules_init;

	/* Set sync refresh channels as high priority */
	ipu_idmac_write(ipu, 0x18800000, IDMAC_CHA_PRI(0));

	/* Set MCU_T to divide MCU access window into 2 */
	ipu_cm_write(ipu, 0x00400000L | (IPU_MCU_T_DEFAULT << 18), IPU_DISP_GEN);

	ret = ipu_add_client_devices(ipu);
	if (ret) {
		dev_err(&pdev->dev, "adding client devices failed with %d\n", ret);
		goto failed_add_clients;
	}

	ipu_put(ipu);

	return 0;

failed_add_clients:
	ipu_submodules_exit(ipu);
failed_submodules_init:
	ipu_irq_exit(ipu);
out_failed_irq:
	ipu_put(ipu);
	clk_put(ipu->clk);
failed_clk_get:
failed_ioremap:
	return ret;
}

static int __devexit ipu_remove(struct platform_device *pdev)
{
	struct ipu_soc *ipu = platform_get_drvdata(pdev);
	struct resource *res;
	int ipu_usecount;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	platform_device_unregister_children(pdev);
	ipu_submodules_exit(ipu);
	ipu_irq_exit(ipu);

	ipu_usecount = atomic_read(&ipu->usecount);

	if (ipu_usecount) {
		dev_err(ipu->dev, "unbalanced use count: %d\n", ipu_usecount);
		clk_disable(ipu->clk);
	}

	clk_put(ipu->clk);

	return 0;
}

static struct platform_driver imx_ipu_driver = {
	.driver = {
		.name = "imx-ipuv3",
	},
	.probe = ipu_probe,
	.remove = __devexit_p(ipu_remove),
};

static int __init imx_ipu_init(void)
{
	int32_t ret;

	ret = platform_driver_register(&imx_ipu_driver);
	return 0;
}
subsys_initcall(imx_ipu_init);

static void __exit imx_ipu_exit(void)
{
	platform_driver_unregister(&imx_ipu_driver);
}
module_exit(imx_ipu_exit);

MODULE_DESCRIPTION("i.MX IPU v3 driver");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_LICENSE("GPL");
