/*
 * Copyright (C) 2010 Francisco Jerez.
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

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_encon.h>

struct sii902x_encoder_params {
};

struct sii902x_priv {
	struct sii902x_encoder_params config;
	struct i2c_client *client;
	struct drm_encoder_connector encon;
};

#define to_sii902x(x) container_of(x, struct sii902x_priv, encon)

static int sii902x_write(struct i2c_client *client, uint8_t addr, uint8_t val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, addr, val);
	if (ret) {
		dev_dbg(&client->dev, "%s failed with %d\n", __func__, ret);
	}
	return ret;
}

static uint8_t sii902x_read(struct i2c_client *client, uint8_t addr)
{
	int dat;

	dat = i2c_smbus_read_byte_data(client, addr);

	return dat;
}

static int hdmi_cap = 0; /* FIXME */

static void sii902x_poweron(struct sii902x_priv *priv)
{
	struct i2c_client *client = priv->client;

	/* Turn on DVI or HDMI */
	if (hdmi_cap) {
		sii902x_write(client, 0x1A, 0x01 | 4);
	} else {
		sii902x_write(client, 0x1A, 0x00);
	}

	return;
}

static void sii902x_poweroff(struct sii902x_priv *priv)
{
	struct i2c_client *client = priv->client;

	/* disable tmds before changing resolution */
	if (hdmi_cap) {
		sii902x_write(client, 0x1A, 0x11);
	} else {
		sii902x_write(client, 0x1A, 0x10);
	}
	return;
}

static int sii902x_get_modes(struct drm_encoder_connector *encon)
{
	struct sii902x_priv *priv = to_sii902x(encon);
	struct i2c_client *client = priv->client;
	struct i2c_adapter *adap = client->adapter;
	struct drm_connector *connector = &encon->connector;
	struct edid *edid;
	int ret;
	int old, dat, cnt = 100;

	old = sii902x_read(client, 0x1A);

	sii902x_write(client, 0x1A, old | 0x4);
	do {
		cnt--;
		msleep(10);
		dat = sii902x_read(client, 0x1A);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt)
		return -ETIMEDOUT;

	sii902x_write(client, 0x1A, old | 0x06);

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
		sii902x_write(client, 0x1A, old & ~0x6);
		msleep(10);
		dat = sii902x_read(client, 0x1A);
	} while ((dat & 0x6) && cnt);

	if (!cnt)
		ret = -1;

	sii902x_write(client, 0x1A, old);

	return 0;
}

static irqreturn_t sii902x_detect_handler(int irq, void *data)
{
	struct sii902x_priv *priv = data;
	struct i2c_client *client = priv->client;
	int dat;

	dat = sii902x_read(client, 0x3D);
	if (dat & 0x1) {
		/* cable connection changes */
		if (dat & 0x4) {
			DRM_DEBUG("plugin\n");
		} else {
			DRM_DEBUG("plugout\n");
		}
	}
	sii902x_write(client, 0x3D, dat);

	return IRQ_HANDLED;
}


static int sii902x_mode_valid(struct drm_encoder_connector *encon,
			  struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void sii902x_mode_set(struct drm_encoder_connector *encon,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct sii902x_priv *priv = to_sii902x(encon);
	struct i2c_client *client = priv->client;
	u16 data[4];
	u8 *tmp;
	int i;

	/* Power up */
	sii902x_write(client, 0x1E, 0x00);

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
		sii902x_write(client, i, tmp[i]);
	}

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	sii902x_write(client, 0x08, 0x70);
	/* Set input format to RGB */
	sii902x_write(client, 0x09, 0x00);
	/* set output format to RGB */
	sii902x_write(client, 0x0A, 0x00);
	/* audio setup */
	sii902x_write(client, 0x25, 0x00);
	sii902x_write(client, 0x26, 0x40);
	sii902x_write(client, 0x27, 0x00);
}

static void sii902x_dpms(struct drm_encoder_connector *encon, int mode)
{
	struct sii902x_priv *priv = to_sii902x(encon);

	if (mode)
		sii902x_poweroff(priv);
	else
		sii902x_poweron(priv);
}

static void sii902x_prepare(struct drm_encoder_connector *encon)
{
	struct sii902x_priv *priv = to_sii902x(encon);

	sii902x_poweroff(priv);
}

static void sii902x_commit(struct drm_encoder_connector *encon)
{
	struct sii902x_priv *priv = to_sii902x(encon);

	sii902x_poweron(priv);
}

struct drm_encoder_connector_funcs sii902x_funcs = {
	.dpms = sii902x_dpms,
	.prepare = sii902x_prepare,
	.commit = sii902x_commit,
	.get_modes = sii902x_get_modes,
	.mode_valid = sii902x_mode_valid,
	.mode_set = sii902x_mode_set,
};

/* I2C driver functions */

static int
sii902x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int dat, ret;
	struct sii902x_priv *priv;
	const char *drm_name = "imx-drm.0"; /* FIXME: pass from pdata */
	int encon_id = 0; /* FIXME: pass from pdata */

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	/* Set 902x in hardware TPI mode on and jump out of D3 state */
	if (sii902x_write(client, 0xc7, 0x00) < 0) {
		dev_err(&client->dev, "SII902x: cound not find device\n");
		return -ENODEV;
	}

	/* read device ID */
	dat = sii902x_read(client, 0x1b);
	if (dat != 0xb0) {
		dev_err(&client->dev, "not found. id is 0x%02x instead of 0xb0\n",
				dat);
		return -ENODEV;
	}

	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL, sii902x_detect_handler,
				IRQF_TRIGGER_FALLING,
				"SII902x_det", priv);
		sii902x_write(client, 0x3c, 0x01);
	}

	priv->encon.funcs = &sii902x_funcs;

	i2c_set_clientdata(client, priv);

	drm_encon_register(drm_name, encon_id, &priv->encon);

	dev_info(&client->dev, "initialized\n");

	return 0;
}

static int sii902x_remove(struct i2c_client *client)
{
	struct sii902x_priv *priv;
	int ret;

	priv = i2c_get_clientdata(client);

	ret = drm_encon_unregister(&priv->encon);
	if (ret)
		return ret;

	kfree(priv);

	return 0;
}

static struct i2c_device_id sii902x_ids[] = {
	{ "sii9022", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sii902x_ids);

static struct i2c_driver sii902x_i2c_driver = {
	.probe = sii902x_probe,
	.remove = sii902x_remove,
	.driver = {
		.name = "sii902x",
	},
	.id_table = sii902x_ids,
};

static int __init sii902x_init(void)
{
	return i2c_add_driver(&sii902x_i2c_driver);
}

static void __exit sii902x_exit(void)
{
	i2c_del_driver(&sii902x_i2c_driver);
}

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("Silicon Image sii902x HDMI transmitter driver");
MODULE_LICENSE("GPL");

module_init(sii902x_init);
module_exit(sii902x_exit);
