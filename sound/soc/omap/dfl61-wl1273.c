/*
 * dfl61-wl1273.c -- SoC audio for WL1273 on Nokia DFL61 class devices
 *
 * Author: Matti Aaltonen <matti.j.aaltonen@nokia.com>
 *
 * Copyright:   (C) 2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <sound/soc.h>
#include <asm/mach-types.h>
#include <plat/dfl61-audio.h>
#include <plat/mcbsp.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <plat/omap_hwmod.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/wl1273.h"


static struct snd_soc_card dfl61wl1273_sound_card;

static int dfl61wl1273_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int err;

	if (omap_mcbsp_st_add_controls(rtd))
		dev_dbg(codec->dev, "Unable to set Sidetone for McBSP4\n");
	
	err = dfl61_set_dma_op_mode("omap-mcbsp.4", true);
	if (err < 0)
		return err;

	return 0;
}

static int dfl61wl1273_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	/*struct snd_soc_dai *codec_dai = rtd->codec_dai;*/
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	unsigned int fmt;
	int r;
	
	r = wl1273_get_format(codec, &fmt);
	if (r) {
		pr_err("Can't get format: %d\n", r);
		return r;
	}

	/* FIXME Not supported by wl1273-fm */
	/* Set codec DAI configuration */
	/*r = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (r < 0) {
		pr_err("Can't set codec DAI configuration: %d\n", r);
		return r;
	}*/

	/* Set cpu DAI configuration */
	r = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (r < 0) {
		pr_err("Can't set cpu DAI configuration: %d\n", r);
		return r;
	}
	return 0;
}

static struct snd_soc_ops dfl61wl1273_ops = {
	.hw_params = dfl61wl1273_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link dfl61wl1273_dai[] = {
	{
		.name = "BT/FM PCM",
		.stream_name = "BT/FM Stream",
		.cpu_dai_name = "omap-mcbsp.4",
		.codec_dai_name = "wl1273-fm",
		.platform_name = "omap-pcm-audio",
		.codec_name = "wl1273-codec",
		.init = dfl61wl1273_init,
		.ops = &dfl61wl1273_ops,
	},
};

/* Audio card */
static struct snd_soc_card dfl61wl1273_sound_card = {
	.name = "dfl61-wl1273",
	.owner = THIS_MODULE,
	.dai_link = dfl61wl1273_dai,
	.num_links = ARRAY_SIZE(dfl61wl1273_dai),
};

static __devinit int dfl61wl1273_probe(struct platform_device *pdev)
{
	int err;
	struct snd_soc_card *card = &dfl61wl1273_sound_card;

	if (!machine_is_nokia_rm680() && !machine_is_nokia_rm696())
		return -ENODEV;	
	printk(KERN_INFO "ALSA for WL1273 on Nokia DFL61 init\n");

	card->dev = &pdev->dev;

	err = snd_soc_register_card(card);
	if (err) {
		printk(KERN_ERR "Error: %d\n", err);
		return err;
	}

	printk(KERN_INFO "DFL61 WL1273 init done\n");

	return 0;
}

static int __devexit dfl61wl1273_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = &dfl61wl1273_sound_card;

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver dfl61wl1273_driver = {
	.probe  = dfl61wl1273_probe,
	.remove = __devexit_p(dfl61wl1273_remove),
	.driver = {
		.name = "dfl61audio-wl1273",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(dfl61wl1273_driver);

MODULE_AUTHOR("Matti Aaltonen <matti.j.aaltonen@nokia.com>");
MODULE_DESCRIPTION("ALSA SoC for WL1273 on Nokia DFL61 class devices");
MODULE_LICENSE("GPL");
