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
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <drm/imx-ipu-v3.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <mach/clock.h>
#include "ipu-prv.h"

#define SYNC_WAVE 0

#define DC_DISP_ID_SYNC(di)	(di)

struct ipu_di {
	void __iomem *base;
	int id;
	u32 module;
	struct clk *clk;
	struct clk *ipu_clk;
	bool external_clk;
	bool inuse;
	bool initialized;
	struct clk pixel_clk;
	struct clk_lookup *clk_lookup;
	struct ipu_soc *ipu;
};

static struct ipu_di dis[2];

static DEFINE_MUTEX(di_mutex);
static struct device *ipu_dev;

struct di_sync_config {
	int run_count;
	int run_src;
	int offset_count;
	int offset_src;
	int repeat_count;
	int cnt_clr_src;
	int cnt_polarity_gen_en;
	int cnt_polarity_clr_src;
	int cnt_polarity_trigger_src;
	int cnt_up;
	int cnt_down;
};

enum di_pins {
	DI_PIN11 = 0,
	DI_PIN12 = 1,
	DI_PIN13 = 2,
	DI_PIN14 = 3,
	DI_PIN15 = 4,
	DI_PIN16 = 5,
	DI_PIN17 = 6,
	DI_PIN_CS = 7,

	DI_PIN_SER_CLK = 0,
	DI_PIN_SER_RS = 1,
};

enum di_sync_wave {
	DI_SYNC_NONE = 0,
	DI_SYNC_CLK = 1,
	DI_SYNC_INT_HSYNC = 2,
	DI_SYNC_HSYNC = 3,
	DI_SYNC_VSYNC = 4,
	DI_SYNC_DE = 6,
};

#define DI_GENERAL		0x0000
#define DI_BS_CLKGEN0		0x0004
#define DI_BS_CLKGEN1		0x0008
#define DI_SW_GEN0(gen)		(0x000c + 4 * ((gen) - 1))
#define DI_SW_GEN1(gen)		(0x0030 + 4 * ((gen) - 1))
#define DI_STP_REP(gen)		(0x0148 + 4 * (((gen) - 1)/2))
#define DI_SYNC_AS_GEN		0x0054
#define DI_DW_GEN(gen)		(0x0058 + 4 * (gen))
#define DI_DW_SET(gen, set)	(0x0088 + 4 * ((gen) + 0xc * (set)))
#define DI_SER_CONF		0x015c
#define DI_SSC			0x0160
#define DI_POL			0x0164
#define DI_AW0			0x0168
#define DI_AW1			0x016c
#define DI_SCR_CONF		0x0170
#define DI_STAT			0x0174

#define DI_SW_GEN0_RUN_COUNT(x)			((x) << 19)
#define DI_SW_GEN0_RUN_SRC(x)			((x) << 16)
#define DI_SW_GEN0_OFFSET_COUNT(x)		((x) << 3)
#define DI_SW_GEN0_OFFSET_SRC(x)		((x) << 0)

#define DI_SW_GEN1_CNT_POL_GEN_EN(x)		((x) << 29)
#define DI_SW_GEN1_CNT_CLR_SRC(x)		((x) << 25)
#define DI_SW_GEN1_CNT_POL_TRIGGER_SRC(x)	((x) << 12)
#define DI_SW_GEN1_CNT_POL_CLR_SRC(x)		((x) << 9)
#define DI_SW_GEN1_CNT_DOWN(x)			((x) << 16)
#define DI_SW_GEN1_CNT_UP(x)			(x)
#define DI_SW_GEN1_AUTO_RELOAD			(0x10000000)

#define DI_DW_GEN_ACCESS_SIZE_OFFSET		24
#define DI_DW_GEN_COMPONENT_SIZE_OFFSET		16

#define DI_GEN_DI_CLK_EXT			(1 << 20)
#define DI_GEN_DI_VSYNC_EXT			(1 << 21)
#define DI_GEN_POLARITY_1			(1 << 0)
#define DI_GEN_POLARITY_2			(1 << 1)
#define DI_GEN_POLARITY_3			(1 << 2)
#define DI_GEN_POLARITY_4			(1 << 3)
#define DI_GEN_POLARITY_5			(1 << 4)
#define DI_GEN_POLARITY_6			(1 << 5)
#define DI_GEN_POLARITY_7			(1 << 6)
#define DI_GEN_POLARITY_8			(1 << 7)

#define DI_POL_DRDY_DATA_POLARITY		(1 << 7)
#define DI_POL_DRDY_POLARITY_15			(1 << 4)

#define DI_VSYNC_SEL_OFFSET			13

static inline u32 ipu_di_read(struct ipu_di *di, unsigned offset)
{
	return readl(di->base + offset);
}

static inline void ipu_di_write(struct ipu_di *di, u32 value, unsigned offset)
{
	writel(value, di->base + offset);
}

static unsigned long pixel_clk_get_rate(struct clk *clk)
{
	struct ipu_di *di = container_of(clk, struct ipu_di, pixel_clk);
	unsigned long inrate = clk_get_rate(clk->parent);
	unsigned long outrate = 0;
	u32 div = ipu_di_read(di, DI_BS_CLKGEN0);

	if (div == 0) {
		goto done;
	}

	outrate = (inrate << 4) / div;

done:
	dev_dbg(ipu_dev, "%s: inrate: %ld div: 0x%08x outrate: %ld\n",
			__func__, inrate, div, outrate);

	return outrate;
}

static int pixel_clk_calc_div(unsigned long inrate, unsigned long outrate)
{
	int div;

	if (inrate <= outrate)
		return 1 << 4;

	div = DIV_ROUND_UP((inrate << 4), outrate);

	return div;
}

static unsigned long pixel_clk_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long inrate = clk_get_rate(clk_get_parent(clk));
	unsigned long outrate;
	int div;

	div = pixel_clk_calc_div(inrate, rate);

	outrate = (inrate << 4) / div;

	dev_dbg(ipu_dev, "%s: inrate: %ld div: 0x%08x outrate: %ld wanted: %ld\n",
			__func__, inrate, div, outrate, rate);

	return outrate;
}

static int pixel_clk_set_rate(struct clk *clk, unsigned long rate)
{
	struct ipu_di *di = container_of(clk, struct ipu_di, pixel_clk);
	unsigned long inrate = clk_get_rate(clk_get_parent(clk));
	int div;

	div = pixel_clk_calc_div(inrate, rate);

	ipu_di_write(di, div, DI_BS_CLKGEN0);

	/* Setup pixel clock timing */
	/* Down time is half of period */
	ipu_di_write(di, (div >> 4)<<16, DI_BS_CLKGEN1);

	dev_info(ipu_dev, "%s: inrate: %ld desired: %ld div: %d actual: %ld\n", __func__, inrate, rate, div, (inrate << 4)/div);

	return 0;
}

static int pixel_clk_set_parent(struct clk *clk, struct clk *parent)
{
	struct ipu_di *di = container_of(clk, struct ipu_di, pixel_clk);
	u32 di_gen = ipu_di_read(di, DI_GENERAL);
	if (parent == di->ipu_clk) {
		dev_dbg(ipu_dev, "%s using internal clock\n", __func__);
		di_gen &= ~DI_GEN_DI_CLK_EXT;
	}
	else if (!IS_ERR(di->clk) && parent == di->clk) {
		dev_dbg(ipu_dev, "%s using external clock\n", __func__);
		di_gen |= DI_GEN_DI_CLK_EXT;
	}
	else {
		dev_dbg(ipu_dev, "%s invalid parent\n", __func__);
		return -EINVAL;
	}
	ipu_di_write(di, di_gen, DI_GENERAL);
	return 0;
}

static int pixel_clk_enable(struct clk *clk)
{
	struct ipu_di *di = container_of(clk, struct ipu_di, pixel_clk);
	u32 disp_gen = ipu_cm_read(di->ipu, IPU_DISP_GEN);
	disp_gen |= clk->id ? IPU_DI1_COUNTER_RELEASE : IPU_DI0_COUNTER_RELEASE;
	ipu_cm_write(di->ipu, disp_gen, IPU_DISP_GEN);

	return 0;
}

static void pixel_clk_disable(struct clk *clk)
{
	struct ipu_di *di = container_of(clk, struct ipu_di, pixel_clk);
	u32 disp_gen = ipu_cm_read(di->ipu, IPU_DISP_GEN);
	disp_gen &= clk->id ? ~IPU_DI1_COUNTER_RELEASE : ~IPU_DI0_COUNTER_RELEASE;
	ipu_cm_write(di->ipu, disp_gen, IPU_DISP_GEN);
}

static void ipu_di_data_wave_config(struct ipu_di *di,
				     int wave_gen,
				     int access_size, int component_size)
{
	u32 reg;
	reg = (access_size << DI_DW_GEN_ACCESS_SIZE_OFFSET) |
	    (component_size << DI_DW_GEN_COMPONENT_SIZE_OFFSET);
	ipu_di_write(di, reg, DI_DW_GEN(wave_gen));
}

static void ipu_di_data_pin_config(struct ipu_di *di, int wave_gen, int di_pin, int set,
				    int up, int down)
{
	u32 reg;

	reg = ipu_di_read(di, DI_DW_GEN(wave_gen));
	reg &= ~(0x3 << (di_pin * 2));
	reg |= set << (di_pin * 2);
	ipu_di_write(di, reg, DI_DW_GEN(wave_gen));

	ipu_di_write(di, (down << 16) | up, DI_DW_SET(wave_gen, set));
}

static void ipu_di_sync_config(struct ipu_di *di, struct di_sync_config *config, int start, int count)
{
	u32 reg;
	int i;

	for (i = 0; i < count; i++) {
		struct di_sync_config *c = &config[i];
		int wave_gen = start + i + 1;

		pr_debug("%s %d\n", __func__, wave_gen);
		if ((c->run_count >= 0x1000) || (c->offset_count >= 0x1000) || (c->repeat_count >= 0x1000) ||
			(c->cnt_up >= 0x400) || (c->cnt_down >= 0x400)) {
			dev_err(ipu_dev, "DI%d counters out of range.\n", di->id);
			return;
		}

		reg = DI_SW_GEN0_RUN_COUNT(c->run_count) |
			DI_SW_GEN0_RUN_SRC(c->run_src) |
			DI_SW_GEN0_OFFSET_COUNT(c->offset_count) |
			DI_SW_GEN0_OFFSET_SRC(c->offset_src);
		ipu_di_write(di, reg, DI_SW_GEN0(wave_gen));

		reg = DI_SW_GEN1_CNT_POL_GEN_EN(c->cnt_polarity_gen_en) |
			DI_SW_GEN1_CNT_CLR_SRC(c->cnt_clr_src) |
			DI_SW_GEN1_CNT_POL_TRIGGER_SRC(c->cnt_polarity_trigger_src) |
			DI_SW_GEN1_CNT_POL_CLR_SRC(c->cnt_polarity_clr_src) |
			DI_SW_GEN1_CNT_DOWN(c->cnt_down) |
			DI_SW_GEN1_CNT_UP(c->cnt_up);

		if (c->repeat_count == 0) {
			/* Enable auto reload */
			reg |= DI_SW_GEN1_AUTO_RELOAD;
		}

		ipu_di_write(di, reg, DI_SW_GEN1(wave_gen));

		reg = ipu_di_read(di, DI_STP_REP(wave_gen));
		reg &= ~(0xffff << (16 * ((wave_gen - 1) & 0x1)));
		reg |= c->repeat_count << (16 * ((wave_gen - 1) & 0x1));
		ipu_di_write(di, reg, DI_STP_REP(wave_gen));
	}
}

static void ipu_di_sync_config_interlaced(struct ipu_di *di, struct ipu_di_signal_cfg *sig)
{
	u32 h_total = sig->width + sig->h_sync_width + sig->h_start_width + sig->h_end_width;
	u32 v_total = sig->height + sig->v_sync_width + sig->v_start_width + sig->v_end_width;
	u32 reg;
	struct di_sync_config cfg[] = {
		{
			.run_count = h_total / 2 - 1,
			.run_src = DI_SYNC_CLK,
		}, {
			.run_count = h_total - 11,
			.run_src = DI_SYNC_CLK,
			.cnt_down = 4,
		}, {
			.run_count = v_total * 2 - 1,
			.run_src = DI_SYNC_INT_HSYNC,
			.offset_count = 1,
			.offset_src = DI_SYNC_INT_HSYNC,
			.cnt_down = 4,
		}, {
			.run_count = v_total / 2 - 1,
			.run_src = DI_SYNC_HSYNC,
			.offset_count = sig->v_start_width,
			.offset_src = DI_SYNC_HSYNC,
			.repeat_count = 2,
			.cnt_clr_src = DI_SYNC_VSYNC,
		}, {
			.run_src = DI_SYNC_HSYNC,
			.repeat_count = sig->height / 2,
			.cnt_clr_src = 4,
		}, {
			.run_count = v_total - 1,
			.run_src = DI_SYNC_HSYNC,
		}, {
			.run_count = v_total / 2 - 1,
			.run_src = DI_SYNC_HSYNC,
			.offset_count = 9,
			.offset_src = DI_SYNC_HSYNC,
			.repeat_count = 2,
			.cnt_clr_src = DI_SYNC_VSYNC,
		}, {
			.run_src = DI_SYNC_CLK,
			.offset_count = sig->h_start_width,
			.offset_src = DI_SYNC_CLK,
			.repeat_count = sig->width,
			.cnt_clr_src = 5,
		}, {
			.run_count = v_total - 1,
			.run_src = DI_SYNC_INT_HSYNC,
			.offset_count = v_total / 2,
			.offset_src = DI_SYNC_INT_HSYNC,
			.cnt_clr_src = DI_SYNC_HSYNC,
			.cnt_down = 4,
		}
	};

	ipu_di_sync_config(di, cfg, 0, ARRAY_SIZE(cfg));

	/* set gentime select and tag sel */
	reg = ipu_di_read(di, DI_SW_GEN1(9));
	reg &= 0x1FFFFFFF;
	reg |= (3 - 1) << 29 | 0x00008000;
	ipu_di_write(di, reg, DI_SW_GEN1(9));

	ipu_di_write(di, v_total / 2 - 1, DI_SCR_CONF);
}

static void ipu_di_sync_config_noninterlaced(struct ipu_di *di,
		struct ipu_di_signal_cfg *sig, int div)
{
	u32 h_total = sig->width + sig->h_sync_width + sig->h_start_width +
		sig->h_end_width;
	u32 v_total = sig->height + sig->v_sync_width + sig->v_start_width +
		sig->v_end_width;
	struct di_sync_config cfg[] = {
		{
			.run_count = h_total - 1,
			.run_src = DI_SYNC_CLK,
		} , {
			.run_count = h_total - 1,
			.run_src = DI_SYNC_CLK,
			.offset_count = div * sig->v_to_h_sync,
			.offset_src = DI_SYNC_CLK,
			.cnt_polarity_gen_en = 1,
			.cnt_polarity_trigger_src = DI_SYNC_CLK,
			.cnt_down = sig->h_sync_width * 2,
		} , {
			.run_count = v_total - 1,
			.run_src = DI_SYNC_INT_HSYNC,
			.cnt_polarity_gen_en = 1,
			.cnt_polarity_trigger_src = DI_SYNC_INT_HSYNC,
			.cnt_down = sig->v_sync_width * 2,
		} , {
			.run_src = DI_SYNC_HSYNC,
			.offset_count = sig->v_sync_width + sig->v_start_width,
			.offset_src = DI_SYNC_HSYNC,
			.repeat_count = sig->height,
			.cnt_clr_src = DI_SYNC_VSYNC,
		} , {
			.run_src = DI_SYNC_CLK,
			.offset_count = sig->h_sync_width + sig->h_start_width,
			.offset_src = DI_SYNC_CLK,
			.repeat_count = sig->width,
			.cnt_clr_src = 5,
		} , {
			/* unused */
		} , {
			/* unused */
		} , {
			/* unused */
		} , {
			/* unused */
		},
	};

	ipu_di_write(di, v_total - 1, DI_SCR_CONF);
	ipu_di_sync_config(di, cfg, 0, ARRAY_SIZE(cfg));
}

int ipu_di_init_sync_panel(struct ipu_di *di, struct ipu_di_signal_cfg *sig)
{
	u32 reg;
	u32 di_gen, vsync_cnt;
	u32 div;
	u32 h_total, v_total;

	struct clk *di_parent;
	u32 rounded_rate;
	u32 rounded_parent;

	dev_dbg(ipu_dev, "disp %d: panel size = %d x %d\n",
		di->id, sig->width, sig->height);

	if ((sig->v_sync_width == 0) || (sig->h_sync_width == 0))
		return -EINVAL;

	h_total = sig->width + sig->h_sync_width + sig->h_start_width + sig->h_end_width;
	v_total = sig->height + sig->v_sync_width + sig->v_start_width + sig->v_end_width;

	mutex_lock(&di_mutex);
	ipu_get(di->ipu);

	di_parent = clk_get_parent(di->clk);
	clk_set_parent(&di->pixel_clk, di->ipu_clk);
	di->external_clk = false;

	rounded_rate = clk_round_rate(&di->pixel_clk, sig->clock_rate);

	dev_dbg(ipu_dev, "di_parent %lu expected %u rounded %u\n", clk_get_rate(di_parent), sig->clock_rate, rounded_rate);

	/* if we are instructed to use an external clock and the rate from the IPU is
	 * not within a reasonable range, actually use the external clock
	 */
	if (sig->ext_clk &&
		((rounded_rate >= sig->clock_rate + sig->clock_rate/200) ||
		(rounded_rate <= sig->clock_rate - sig->clock_rate/200 ))) {

		dev_dbg(ipu_dev, "fixing external clock\n");
		rounded_rate = sig->clock_rate * 2;
		dev_dbg(ipu_dev, "rounded to %u\n", rounded_rate);
		rounded_parent = clk_round_rate(di_parent, rounded_rate);
		while (rounded_rate < rounded_parent) {
			if (rounded_rate / rounded_parent < 8)
				rounded_rate += sig->clock_rate * 2;
			else
				rounded_rate *= 2;
			dev_dbg(ipu_dev, "loop: rounded to %u\n", rounded_rate);
		}
		dev_dbg(ipu_dev, "rounded to %u\n", rounded_rate);
		clk_set_rate(di_parent, rounded_rate);
		rounded_rate = clk_round_rate(di->clk, sig->clock_rate);
		dev_dbg(ipu_dev, "rounded to %u\n", rounded_rate);
		clk_set_rate(di->clk, rounded_rate);
		clk_set_parent(&di->pixel_clk, di->clk);
		di->external_clk = true;
	}

	rounded_rate = clk_round_rate(&di->pixel_clk, sig->clock_rate);
	dev_dbg(ipu_dev, "rounded to %u\n", rounded_rate);
        clk_set_rate(&di->pixel_clk, rounded_rate);

	msleep(5);

	/* integer portion of divider */
        {
		u32 rr = clk_get_rate(clk_get_parent(&di->pixel_clk));
		div = rr / rounded_rate;
		dev_dbg(ipu_dev, "integer portion of div is %u (%u/%u)\n", div, rr, rounded_rate);
	}

	ipu_di_data_wave_config(di, SYNC_WAVE, div - 1, div - 1);
	ipu_di_data_pin_config(di, SYNC_WAVE, DI_PIN15, 3, 0, div * 2);

	di_gen = 0;
	if (di->external_clk)
		di_gen |= DI_GEN_DI_CLK_EXT | DI_GEN_DI_VSYNC_EXT;

	if (sig->interlaced) {
		ipu_di_sync_config_interlaced(di, sig);

		/* set y_sel = 1 */
		di_gen |= 0x10000000;
		di_gen |= DI_GEN_POLARITY_5;
		di_gen |= DI_GEN_POLARITY_8;

		vsync_cnt = 7;

		if (sig->Hsync_pol)
			di_gen |= DI_GEN_POLARITY_3;
		if (sig->Vsync_pol)
			di_gen |= DI_GEN_POLARITY_2;
	} else {
		ipu_di_sync_config_noninterlaced(di, sig, div);

		vsync_cnt = 3;

		if (sig->Hsync_pol)
			di_gen |= DI_GEN_POLARITY_2;
		if (sig->Vsync_pol)
			di_gen |= DI_GEN_POLARITY_3;
	}

	ipu_di_write(di, di_gen, DI_GENERAL);
	ipu_di_write(di, (--vsync_cnt << DI_VSYNC_SEL_OFFSET) | 0x00000002,
		     DI_SYNC_AS_GEN);

	reg = ipu_di_read(di, DI_POL);
	reg &= ~(DI_POL_DRDY_DATA_POLARITY | DI_POL_DRDY_POLARITY_15);

	if (sig->enable_pol)
		reg |= DI_POL_DRDY_POLARITY_15;
	if (sig->data_pol)
		reg |= DI_POL_DRDY_DATA_POLARITY;

	ipu_di_write(di, reg, DI_POL);

	ipu_put(di->ipu);
	mutex_unlock(&di_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_di_init_sync_panel);

void ipu_set_vga_delayed_hsync_vsync(struct ipu_di_signal_cfg *sig,
		uint32_t hsync_delay, uint32_t vsync_delay)
{
	int h_total = sig->width + sig->h_sync_width + sig->h_start_width + sig->h_end_width;
	int v_total = sig->height + sig->v_sync_width + sig->v_start_width + sig->v_end_width;
	u32 di_gen;
	struct ipu_di *di = &dis[1];
	struct di_sync_config cfg[] = {
		{
			/* counter 7 for delay HSYNC */
			.run_count = h_total - 1,
			.run_src = DI_SYNC_CLK,
			.offset_count = hsync_delay,
			.offset_src = DI_SYNC_CLK,
			.repeat_count = 0,
			.cnt_clr_src = DI_SYNC_NONE,
			.cnt_polarity_gen_en = 1,
			.cnt_polarity_clr_src = DI_SYNC_NONE,
			.cnt_polarity_trigger_src = DI_SYNC_CLK,
			.cnt_up = 0,
			.cnt_down = sig->h_sync_width * 2,
		}, {
			/* counter 8 for delay VSYNC */
			.run_count = v_total - 1,
			.run_src = DI_SYNC_INT_HSYNC,
			.offset_count = vsync_delay,
			.offset_src = DI_SYNC_INT_HSYNC,
			.repeat_count = 0,
			.cnt_clr_src = DI_SYNC_NONE,
			.cnt_polarity_gen_en = 1,
			.cnt_polarity_clr_src = DI_SYNC_NONE,
			.cnt_polarity_trigger_src = DI_SYNC_INT_HSYNC,
			.cnt_up = 0,
			.cnt_down = sig->v_sync_width * 2,
		}
	};

	ipu_di_sync_config(di, cfg, 6, ARRAY_SIZE(cfg));

	di_gen = ipu_di_read(di, DI_GENERAL);
	di_gen &= ~DI_GEN_POLARITY_2;
	di_gen &= ~DI_GEN_POLARITY_3;
	di_gen &= ~DI_GEN_POLARITY_7;
	di_gen &= ~DI_GEN_POLARITY_8;
	if (sig->Hsync_pol)
		di_gen |= DI_GEN_POLARITY_7;
	if (sig->Vsync_pol)
		di_gen |= DI_GEN_POLARITY_8;
	ipu_di_write(di, di_gen, DI_GENERAL);
}
EXPORT_SYMBOL_GPL(ipu_set_vga_delayed_hsync_vsync);

int ipu_di_enable(struct ipu_di *di)
{
	ipu_get(di->ipu);

	if (di->external_clk)
		clk_enable(di->clk);

	clk_enable(&di->pixel_clk);

	ipu_module_enable(di->ipu, di->module);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_di_enable);

int ipu_di_disable(struct ipu_di *di)
{
	ipu_module_disable(di->ipu, di->module);
	ipu_put(di->ipu);

	if (di->external_clk)
		clk_disable(di->clk);

	clk_disable(&di->pixel_clk);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_di_disable);

static DEFINE_MUTEX(ipu_di_lock);

struct ipu_di *ipu_di_get(struct ipu_soc *ipu, int disp)
{
	struct ipu_di *di;

	if (disp > 1)
		return ERR_PTR(-EINVAL);

	di = &dis[disp];

	mutex_lock(&ipu_di_lock);

	if (!di->initialized) {
		di = ERR_PTR(-ENOSYS);
		goto out;
	}

	if (di->inuse) {
		di = ERR_PTR(-EBUSY);
		goto out;
	}

	di->inuse = true;
out:
	mutex_unlock(&ipu_di_lock);

	return di;
}
EXPORT_SYMBOL_GPL(ipu_di_get);

void ipu_di_put(struct ipu_di *di)
{
	mutex_lock(&ipu_di_lock);

	di->inuse = false;

	mutex_unlock(&ipu_di_lock);
}
EXPORT_SYMBOL_GPL(ipu_di_put);

int ipu_di_init(struct ipu_soc *ipu, struct device *dev, int id,
		unsigned long base,
		u32 module, struct clk *ipu_clk)
{
	char *clk_id, *con_id;
	struct ipu_di *di = &dis[id];

	if (id > 1)
		return -EINVAL;

	if (id)
		clk_id = "di1";
	else
		clk_id = "di0";

	ipu_dev = dev;

	di->clk = clk_get(dev, clk_id);
	if (IS_ERR(di->clk))
		return PTR_ERR(di->clk);

	di->module = module;
	di->id = id;
	di->ipu_clk = ipu_clk;
	di->base = devm_ioremap(dev, base, PAGE_SIZE);
	if (!di->base)
	{
		printk(KERN_CRIT "%s ioremap bases\n", __func__);
		return -ENOMEM;
	}

	dev_dbg(dev, "DI%d base: 0x%08lx\n", id, base);
	di->initialized = true;
	di->inuse = false;
	di->ipu = ipu;

	if (id == 0)
		con_id = "pixel_clk0";
	else
		con_id = "pixel_clk1";

#if defined(CONFIG_CLK_DEBUG)
	strncpy(di->pixel_clk.name, con_id, 32);
#endif
	di->pixel_clk.get_rate = pixel_clk_get_rate;
	di->pixel_clk.round_rate = pixel_clk_round_rate;
	di->pixel_clk.set_rate = pixel_clk_set_rate;
	di->pixel_clk.set_parent = pixel_clk_set_parent;
	di->pixel_clk.parent = ipu_clk;
	di->pixel_clk.enable = pixel_clk_enable;
	di->pixel_clk.disable = pixel_clk_disable;

	di->clk_lookup = clkdev_alloc(&di->pixel_clk, con_id, "imx-drm.0");
	clkdev_add(di->clk_lookup);
	clk_debug_register(&di->pixel_clk);

	return 0;
}

void ipu_di_exit(struct ipu_soc *ipu, int id)
{
	struct ipu_di *di = &dis[id];

	clkdev_drop(di->clk_lookup);
	clk_put(di->clk);
	di->initialized = false;
}
