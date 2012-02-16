/*
 * Copyright (C) 2011 Genesi USA, Inc.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/module.h>
#include "siihdmi.h"

#define to_siihdmi(x) container_of(x, struct siihdmi_tx, encon)

static int siihdmi_write(struct i2c_client *client, uint8_t addr, uint8_t val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, addr, val);
	if (ret) {
		dev_dbg(&client->dev, "%s failed with %d\n", __func__, ret);
	}
	return ret;
}

static uint8_t siihdmi_read(struct i2c_client *client, uint8_t addr)
{
	int dat;

	dat = i2c_smbus_read_byte_data(client, addr);

	return dat;
}

static void siihdmi_poweron(struct siihdmi_tx *tx)
{
	struct i2c_client *client = tx->client;

	/* Turn on DVI or HDMI */
	if (tx->sink.type == SINK_TYPE_HDMI) {
		siihdmi_write(client, SIIHDMI_TPI_REG_SYS_CTRL, 0x01 | 4);
	} else {
		siihdmi_write(client, SIIHDMI_TPI_REG_SYS_CTRL, 0x00);
	}

	return;
}

static void siihdmi_poweroff(struct siihdmi_tx *tx)
{
	struct i2c_client *client = tx->client;

	/* disable tmds before changing resolution */
	if (tx->sink.type == SINK_TYPE_HDMI) {
		siihdmi_write(client, SIIHDMI_TPI_REG_SYS_CTRL, 0x11);
	} else {
		siihdmi_write(client, SIIHDMI_TPI_REG_SYS_CTRL, 0x10);
	}
	return;
}

static int siihdmi_get_modes(struct drm_encoder_connector *encon)
{
	struct siihdmi_tx *tx = to_siihdmi(encon);
	struct i2c_client *client = tx->client;
	struct i2c_adapter *adap = client->adapter;
	struct drm_connector *connector = &encon->connector;
	struct edid *edid;
	int ret;
	int old, dat, cnt = 100;

	old = siihdmi_read(client, SIIHDMI_TPI_REG_SYS_CTRL);

	siihdmi_write(client, SIIHDMI_TPI_REG_SYS_CTRL, old | 0x4);
	do {
		cnt--;
		msleep(10);
		dat = siihdmi_read(client, SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt)
		return -ETIMEDOUT;

	siihdmi_write(client, SIIHDMI_TPI_REG_SYS_CTRL, old | 0x06);

	edid = drm_get_edid(connector, adap);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		connector->display_info.raw_edid = NULL;
		kfree(edid);
	}

	cnt = 100;
	do {
		cnt--;
		siihdmi_write(client, SIIHDMI_TPI_REG_SYS_CTRL, old & ~0x6);
		msleep(10);
		dat = siihdmi_read(client, SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((dat & 0x6) && cnt);

	if (!cnt)
		ret = -1;

	siihdmi_write(client, SIIHDMI_TPI_REG_SYS_CTRL, old);

	return 0;
}

static irqreturn_t siihdmi_detect_handler(int irq, void *data)
{
	struct siihdmi_tx *tx = data;
	struct i2c_client *client = tx->client;
	int dat;

	dat = siihdmi_read(client, SIIHDMI_TPI_REG_ISR);
	if (dat & 0x1) {
		/* cable connection changes */
		if (dat & 0x4) {
			DRM_DEBUG("plugin\n");
		} else {
			DRM_DEBUG("plugout\n");
		}
	}
	siihdmi_write(client, SIIHDMI_TPI_REG_ISR, dat);

	return IRQ_HANDLED;
}


static int siihdmi_mode_valid(struct drm_encoder_connector *encon,
			  struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void siihdmi_mode_set(struct drm_encoder_connector *encon,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct siihdmi_tx *tx = to_siihdmi(encon);
	struct i2c_client *client = tx->client;
	u16 data[4];
	u8 *tmp;
	int i;

	/* Power up */
	siihdmi_write(client, SIIHDMI_TPI_REG_PWR_STATE, 0x00);

	dev_dbg(&client->dev, "%s: %dx%d, pixclk %d\n", __func__,
			mode->hdisplay, mode->vdisplay,
			mode->clock * 1000);

	/* set TPI video mode */
	data[0] = mode->clock / 10;
	data[1] = mode->vrefresh * 100;
	data[2] = mode->htotal;
	data[3] = mode->vtotal;
	tmp = (u8 *)data;

	for (i = 0; i < 8; i++) {
		siihdmi_write(client, i, tmp[i]);
	}

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	siihdmi_write(client, SIIHDMI_TPI_REG_INPUT_BUS_PIXEL_REPETITION, 0x70);
	/* Set input format to RGB */
	siihdmi_write(client, SIIHDMI_TPI_REG_AVI_INPUT_FORMAT, 0x00);
	/* set output format to RGB */
	siihdmi_write(client, SIIHDMI_TPI_REG_AVI_OUTPUT_FORMAT, 0x00);
	/* audio setup */
	siihdmi_write(client, SIIHDMI_TPI_REG_I2S_ORIGINAL_FREQ_SAMPLE_LENGTH, 0x00);
	siihdmi_write(client, SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL, 0x40);
	siihdmi_write(client, SIIHDMI_TPI_REG_I2S_AUDIO_SAMPLING_HBR, 0x00);
}

static void siihdmi_dpms(struct drm_encoder_connector *encon, int mode)
{
	struct siihdmi_tx *tx = to_siihdmi(encon);

	if (mode)
		siihdmi_poweroff(tx);
	else
		siihdmi_poweron(tx);
}

static void siihdmi_prepare(struct drm_encoder_connector *encon)
{
	struct siihdmi_tx *tx = to_siihdmi(encon);

	siihdmi_poweroff(tx);
}

static void siihdmi_commit(struct drm_encoder_connector *encon)
{
	struct siihdmi_tx *tx = to_siihdmi(encon);

	siihdmi_poweron(tx);
}

struct drm_encoder_connector_funcs siihdmi_funcs = {
	.dpms = siihdmi_dpms,
	.prepare = siihdmi_prepare,
	.commit = siihdmi_commit,
	.get_modes = siihdmi_get_modes,
	.mode_valid = siihdmi_mode_valid,
	.mode_set = siihdmi_mode_set,
};

/* I2C driver functions */

static int
siihdmi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int dat, ret;
	struct siihdmi_tx *tx;
	const char *drm_name = "imx-drm.0"; /* FIXME: pass from pdata */
	int encon_id = 0; /* FIXME: pass from pdata */

	tx = kzalloc(sizeof(*tx), GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	tx->client = client;
	tx->platform = client->dev.platform_data;

	/* Set 902x in hardware TPI mode on and jump out of D3 state */
	if (siihdmi_write(client, 0xc7, 0x00) < 0) {
		dev_err(&client->dev, "siihdmi: cound not find device\n");
		return -ENODEV;
	}

	dat = siihdmi_read(client, SIIHDMI_TPI_REG_DEVICE_ID);
	if (dat != 0xb0) {
		dev_err(&client->dev, "not found. id is 0x%02x instead of 0xb0\n",
				dat);
		return -ENODEV;
	}

	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL, siihdmi_detect_handler,
				IRQF_TRIGGER_FALLING,
				"siihdmi_det", tx);
		siihdmi_write(client, SIIHDMI_TPI_REG_IER, 0x01);
	}

	tx->encon.funcs = &siihdmi_funcs;

	i2c_set_clientdata(client, tx);

	drm_encon_register(drm_name, encon_id, &tx->encon);

	dev_info(&client->dev, "initialized\n");

	return 0;
}

static int siihdmi_remove(struct i2c_client *client)
{
	struct siihdmi_tx *tx;
	int ret;

	tx = i2c_get_clientdata(client);

	ret = drm_encon_unregister(&tx->encon);
	if (ret)
		return ret;

	kfree(tx);

	return 0;
}

static struct i2c_device_id siihdmi_ids[] = {
	{ "sii9022", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, siihdmi_ids);

static struct i2c_driver siihdmi_i2c_driver = {
	.probe = siihdmi_probe,
	.remove = siihdmi_remove,
	.driver = {
		.name = "siihdmi",
	},
	.id_table = siihdmi_ids,
};

static int __init siihdmi_init(void)
{
	return i2c_add_driver(&siihdmi_i2c_driver);
}

static void __exit siihdmi_exit(void)
{
	i2c_del_driver(&siihdmi_i2c_driver);
}

MODULE_AUTHOR("Saleem Abdulrasool <compnerd@compnerd.org>");
MODULE_DESCRIPTION("Silicon Image HDMI transmitter driver");
MODULE_LICENSE("GPL");

module_init(siihdmi_init);
module_exit(siihdmi_exit);
