/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

#ifndef __ASM_ARCH_IPU_H__
#define __ASM_ARCH_IPU_H__

#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/bitmap.h>
#include <linux/fb.h>
#include <mach/hardware.h>

struct ipu_soc;

/*
 * IPU Pixel Formats
 *
 * Pixel formats are defined with ASCII FOURCC code. The pixel format codes are
 * the same used by V4L2 API.
 */

/* Generic or Raw Data Formats */
#define IPU_PIX_FMT_GENERIC v4l2_fourcc('I', 'P', 'U', '0')	/* IPU Generic Data */
#define IPU_PIX_FMT_GENERIC_32 v4l2_fourcc('I', 'P', 'U', '1')	/* IPU Generic Data */
#define IPU_PIX_FMT_LVDS666 v4l2_fourcc('L', 'V', 'D', '6')	/* IPU Generic Data */
#define IPU_PIX_FMT_LVDS888 v4l2_fourcc('L', 'V', 'D', '8')	/* IPU Generic Data */
/* RGB Formats */
#define IPU_PIX_FMT_RGB332  V4L2_PIX_FMT_RGB332			/*  8  RGB-3-3-2    */
#define IPU_PIX_FMT_RGB555  V4L2_PIX_FMT_RGB555			/* 16  RGB-5-5-5    */
#define IPU_PIX_FMT_RGB565  V4L2_PIX_FMT_RGB565			/* 1 6  RGB-5-6-5   */
#define IPU_PIX_FMT_RGB666  v4l2_fourcc('R', 'G', 'B', '6')	/* 18  RGB-6-6-6    */
#define IPU_PIX_FMT_BGR666  v4l2_fourcc('B', 'G', 'R', '6')	/* 18  BGR-6-6-6    */
#define IPU_PIX_FMT_BGR24   V4L2_PIX_FMT_BGR24			/* 24  BGR-8-8-8    */
#define IPU_PIX_FMT_RGB24   V4L2_PIX_FMT_RGB24			/* 24  RGB-8-8-8    */
#define IPU_PIX_FMT_GBR24   v4l2_fourcc('G', 'B', 'R', '3')	/* 24  GBR-8-8-8    */
#define IPU_PIX_FMT_BGR32   V4L2_PIX_FMT_BGR32			/* 32  BGR-8-8-8-8  */
#define IPU_PIX_FMT_BGRA32  v4l2_fourcc('B', 'G', 'R', 'A')	/* 32  BGR-8-8-8-8  */
#define IPU_PIX_FMT_RGB32   V4L2_PIX_FMT_RGB32			/* 32  RGB-8-8-8-8  */
#define IPU_PIX_FMT_RGBA32  v4l2_fourcc('R', 'G', 'B', 'A')	/* 32  RGB-8-8-8-8  */
#define IPU_PIX_FMT_ABGR32  v4l2_fourcc('A', 'B', 'G', 'R')	/* 32  ABGR-8-8-8-8 */
/* YUV Interleaved Formats */
#define IPU_PIX_FMT_YUYV    V4L2_PIX_FMT_YUYV			/* 16 YUV 4:2:2 */
#define IPU_PIX_FMT_UYVY    V4L2_PIX_FMT_UYVY			/* 16 YUV 4:2:2 */
#define IPU_PIX_FMT_Y41P    V4L2_PIX_FMT_Y41P			/* 12 YUV 4:1:1 */
#define IPU_PIX_FMT_YUV444  V4L2_PIX_FMT_YUV444			/* 24 YUV 4:4:4 */
/* two planes -- one Y, one Cb + Cr interleaved  */
#define IPU_PIX_FMT_NV12    V4L2_PIX_FMT_NV12			/* 12  Y/CbCr 4:2:0  */
/* YUV Planar Formats */
#define IPU_PIX_FMT_GREY    V4L2_PIX_FMT_GREY			/* 8  Greyscale */
#define IPU_PIX_FMT_YVU410P V4L2_PIX_FMT_YVU410P		/* 9  YVU 4:1:0 */
#define IPU_PIX_FMT_YUV410P V4L2_PIX_FMT_YUV410P		/* 9  YUV 4:1:0 */
#define IPU_PIX_FMT_YVU420P v4l2_fourcc('Y', 'V', '1', '2')	/* 12 YVU 4:2:0 */
#define IPU_PIX_FMT_YUV420P v4l2_fourcc('I', '4', '2', '0')	/* 12 YUV 4:2:0 */
#define IPU_PIX_FMT_YUV420P2 v4l2_fourcc('Y', 'U', '1', '2')	/* 12 YUV 4:2:0 */
#define IPU_PIX_FMT_YVU422P v4l2_fourcc('Y', 'V', '1', '6')	/* 16 YVU 4:2:2 */
#define IPU_PIX_FMT_YUV422P V4L2_PIX_FMT_YUV422P		/* 16 YUV 4:2:2 */

/*
 * Bitfield of Display Interface signal polarities.
 */
struct ipu_di_signal_cfg {
	unsigned datamask_en:1;
	unsigned ext_clk:1;
	unsigned interlaced:1;
	unsigned odd_field_first:1;
	unsigned clksel_en:1;
	unsigned clkidle_en:1;
	unsigned data_pol:1;	/* true = inverted */
	unsigned clk_pol:1;	/* true = rising edge */
	unsigned enable_pol:1;
	unsigned Hsync_pol:1;	/* true = active high */
	unsigned Vsync_pol:1;

	u16 width;
	u16 height;
	u32 pixel_fmt;
	u16 h_start_width;
	u16 h_sync_width;
	u16 h_end_width;
	u16 v_start_width;
	u16 v_sync_width;
	u16 v_end_width;
	u32 v_to_h_sync;

	u32 clock_rate;
};

typedef enum {
	IPU_COLORSPACE_RGB,
	IPU_COLORSPACE_YCBCR,
	IPU_COLORSPACE_YUV,
	IPU_COLORSPACE_UNKNOWN,
} ipu_color_space_t;

#define IPU_IRQ_EOF(channel)		(channel)		/* 0 .. 63 */
#define IPU_IRQ_NFACK(channel)		((channel) + 64)	/* 64 .. 127 */
#define IPU_IRQ_NFB4EOF(channel)	((channel) + 128)	/* 128 .. 191 */
#define IPU_IRQ_EOS(channel)		((channel) + 192)	/* 192 .. 255 */

#define IPU_IRQ_DP_SF_START		(448 + 2)
#define IPU_IRQ_DP_SF_END		(448 + 3)
#define IPU_IRQ_BG_SF_END		IPU_IRQ_DP_SF_END,
#define IPU_IRQ_DC_FC_0			(448 + 8)
#define IPU_IRQ_DC_FC_1			(448 + 9)
#define IPU_IRQ_DC_FC_2			(448 + 10)
#define IPU_IRQ_DC_FC_3			(448 + 11)
#define IPU_IRQ_DC_FC_4			(448 + 12)
#define IPU_IRQ_DC_FC_6			(448 + 13)
#define IPU_IRQ_VSYNC_PRE_0		(448 + 14)
#define IPU_IRQ_VSYNC_PRE_1		(448 + 15)

#define IPU_IRQ_COUNT	(15 * 32)

struct ipu_channel;

/*
 * IPU Image DMA Controller (idmac) functions
 */
struct ipu_channel *ipu_idmac_get(struct ipu_soc *ipu, unsigned channel);
void ipu_idmac_put(struct ipu_channel *);

int ipu_idmac_enable_channel(struct ipu_channel *channel);
int ipu_idmac_disable_channel(struct ipu_channel *channel);

void ipu_idmac_set_double_buffer(struct ipu_channel *channel, bool doublebuffer);
void ipu_idmac_select_buffer(struct ipu_channel *channel, u32 buf_num);

/*
 * IPU Display Controller (dc) functions
 */
struct ipu_dc;
struct ipu_dc *ipu_dc_get(struct ipu_soc *ipu, int channel);
void ipu_dc_put(struct ipu_dc *dc);
int ipu_dc_init_sync(struct ipu_dc *dc, int di, bool interlaced,
		u32 pixel_fmt, u32 width);
void ipu_dc_init_async(struct ipu_dc *dc, int di, bool interlaced);
void ipu_dc_enable_channel(struct ipu_dc *dc);
void ipu_dc_disable_channel(struct ipu_dc *dc);

/*
 * IPU Display Interface (di) functions
 */
struct ipu_di;
struct ipu_di *ipu_di_get(struct ipu_soc *ipu, int disp);
void ipu_di_put(struct ipu_di *);
int ipu_di_disable(struct ipu_di *);
int ipu_di_enable(struct ipu_di *);
int ipu_di_init_sync_panel(struct ipu_di *, struct ipu_di_signal_cfg *sig);

/*
 * IPU Display Multi FIFO Controller (dmfc) functions
 */
struct dmfc_channel;
int ipu_dmfc_enable_channel(struct dmfc_channel *dmfc);
void ipu_dmfc_disable_channel(struct dmfc_channel *dmfc);
int ipu_dmfc_alloc_bandwidth(struct dmfc_channel *dmfc, unsigned long bandwidth_mbs,
		int burstsize);
void ipu_dmfc_free_bandwidth(struct dmfc_channel *dmfc);
int ipu_dmfc_init_channel(struct dmfc_channel *dmfc, int width);
struct dmfc_channel *ipu_dmfc_get(struct ipu_soc *ipu, int ipu_channel);
void ipu_dmfc_put(struct dmfc_channel *dmfc);

/*
 * IPU Display Processor (dp) functions
 */
#define IPU_DP_FLOW_SYNC_BG	0
#define IPU_DP_FLOW_SYNC_FG	1
#define IPU_DP_FLOW_ASYNC0_BG	2
#define IPU_DP_FLOW_ASYNC0_FG	3
#define IPU_DP_FLOW_ASYNC1_BG	4
#define IPU_DP_FLOW_ASYNC1_FG	5

struct ipu_dp *ipu_dp_get(struct ipu_soc *ipu, unsigned int flow);
void ipu_dp_put(struct ipu_dp *);
int ipu_dp_enable_channel(struct ipu_dp *dp);
void ipu_dp_disable_channel(struct ipu_dp *dp);
int ipu_dp_setup_channel(struct ipu_dp *dp,
		ipu_color_space_t in, ipu_color_space_t out);
int ipu_dp_set_window_pos(struct ipu_dp *, u16 x_pos, u16 y_pos);
int ipu_dp_set_global_alpha(struct ipu_dp *dp, bool enable, u8 alpha,
		bool bg_chan);


/*
 * IPU IDMAC channel helpers
 */

struct ipu_rgb {
	struct fb_bitfield	red;
	struct fb_bitfield	green;
	struct fb_bitfield	blue;
	struct fb_bitfield	transp;
	int			bits_per_pixel;
};

void ipu_channel_set_resolution(struct ipu_channel *ch, int xres, int yres);
void ipu_channel_set_stride(struct ipu_channel *ch, int stride);
void ipu_channel_set_high_priority(struct ipu_channel *ch);
void ipu_channel_set_format_rgb(struct ipu_channel *ch, struct ipu_rgb *rgb);
void ipu_channel_set_buffer(struct ipu_channel *ch, int bufnum, dma_addr_t buf);

#endif
