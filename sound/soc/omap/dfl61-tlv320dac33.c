/*
 * dfl61-tlv320dac33.c -- SoC audio for tlv320dac33 on Nokia DFL61 class
 *			  devices
 *
 * Author:	Peter Ujfalusi <peter.ujfalusi@nokia.com>
 *
 * Copyright: (C) 2009 Nokia Corporation
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/export.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <plat/dfl61-audio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/tlv320dac33.h"
#include "../codecs/tpa6130a2.h"

static int hp_state;
static int pm_mode;
static struct snd_soc_card dfl61dac33_sound_card;

static int dfl61dac33_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int err;

	/* Set codec DAI configuration */
	err = snd_soc_dai_set_fmt(codec_dai,
				SND_SOC_DAIFMT_LEFT_J |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0)
		return err;

	/* Set cpu DAI configuration */
	err = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_LEFT_J |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0)
		return err;

	/* Set the codec system clock for DAC and ADC */
	err = snd_soc_dai_set_sysclk(codec_dai, TLV320DAC33_SLEEPCLK, 32768,
					    SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("dfl61-tlv320dac33: Can't set codec system clock\n");
		return err;
	}

	return 0;
}

static int get_mode_kcontrol(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = pm_mode;

	return 0;
}

static int set_mode(struct snd_soc_codec *codec, int mode)
{
	/* FIXME there is no omap_mcbsp_set_sidle_mode*/
	switch (mode) {
	case 0:
		/* Element + NOIDLE */
		dfl61_set_dma_op_mode("omap-mcbsp.2", false);
		/*omap_mcbsp_set_sidle_mode(1, MCBSP_SIDLE_NO_IDLE, 0);*/
		break;
	case 1:
		/* Element + SMARTIDLE */
		dfl61_set_dma_op_mode("omap-mcbsp.2", false);
		/*omap_mcbsp_set_sidle_mode(1, MCBSP_SIDLE_SMART_IDLE,
					  MCBSP_CLKACT_IOFF_FOFF);*/
		break;
	case 2:
		/* Threshold + NOIDLE */
		dfl61_set_dma_op_mode("omap-mcbsp.2", true);
		/*omap_mcbsp_set_sidle_mode(1, MCBSP_SIDLE_NO_IDLE, 0);*/
		break;
	case 3:
		/* Threshold + SMARTIDLE */
		dfl61_set_dma_op_mode("omap-mcbsp.2", true);
		/*omap_mcbsp_set_sidle_mode(1, MCBSP_SIDLE_SMART_IDLE,
					  MCBSP_CLKACT_IOFF_FOFF);*/
		break;
	}

	pm_mode = mode;
	return 1;
}

static int set_mode_kcontrol(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (pm_mode == ucontrol->value.integer.value[0])
		return 0;

	if (codec->active)
		return 0;

	return set_mode(codec, ucontrol->value.integer.value[0]);
}

static const char *mcbsp_mode_texts[] = {
	"Element NOIDLE", "Element SMARTIDLE",
	"Threshold NOIDLE", "Threshold SMARTIDLE",
};

static const struct soc_enum mcbsp_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mcbsp_mode_texts),
			    mcbsp_mode_texts);

static const struct snd_kcontrol_new dfl61dac33_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_ENUM_EXT("McBSP Use Case", mcbsp_mode_enum,
		     get_mode_kcontrol, set_mode_kcontrol),
};

static void dfl6dac33_hp_enable(struct snd_soc_codec *codec, int enable)
{
	if (hp_state == enable)
		return;

	if (enable) {
		if (!snd_soc_dapm_get_pin_status(&codec->dapm, "Headphone"))
			return;
		msleep(30);
		tpa6130a2_stereo_enable(codec, 1);
	} else {
		tpa6130a2_stereo_enable(codec, 0);
	}
	hp_state = enable;
}

static int dfl6dac33_hp_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		dfl6dac33_hp_enable(w->codec, 1);
	else
		dfl6dac33_hp_enable(w->codec, 0);
	return 0;
}

static int dfl61dac33_stream_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dfl6dac33_hp_enable(w->codec, 0);
		break;
	case SND_SOC_DAPM_POST_PMU:
		dfl6dac33_hp_enable(w->codec, 1);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget dfl61dac33_dapm_widgets[] = {
	/* Outputs */
	SND_SOC_DAPM_LINE("FMTX_L Line Out", NULL),
	SND_SOC_DAPM_LINE("FMTX_R Line Out", NULL),
	SND_SOC_DAPM_HP("Headphone", dfl6dac33_hp_event),
	/* Inputs */
	SND_SOC_DAPM_LINE("twl4030 interconnect", NULL),

	SND_SOC_DAPM_PRE("DFL61-DAC33 stream Start", dfl61dac33_stream_event),
	SND_SOC_DAPM_POST("DFL61-DAC33 stream Stop", dfl61dac33_stream_event),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphone", NULL, "LEFT_LO"},
	{"Headphone", NULL, "RIGHT_LO"},

	{"FMTX_L Line Out", NULL, "LEFT_LO"},
	{"FMTX_R Line Out", NULL, "RIGHT_LO"},

	{"LINER", NULL, "twl4030 interconnect"},
	{"LINEL", NULL, "twl4030 interconnect"},

};

int dfl61_request_hp_enable(bool enable)
{
	if (!dfl61dac33_sound_card.rtd->codec) {
		pr_err("dfl61-request_hp_enable");
		return -ENODEV;
	}

	if (enable)
		tpa6130a2_stereo_enable(dfl61dac33_sound_card.rtd->codec, 1);
	else
		tpa6130a2_stereo_enable(dfl61dac33_sound_card.rtd->codec, 0);

	return 0;
}
EXPORT_SYMBOL(dfl61_request_hp_enable);

void dfl61dac33_interconnect_enable(int enable)
{
	if (!dfl61dac33_sound_card.rtd->codec)
		return;

	if (enable)
		snd_soc_dapm_enable_pin(&dfl61dac33_sound_card.rtd->codec->dapm,
			"twl4030 interconnect");
	else
		snd_soc_dapm_disable_pin(&dfl61dac33_sound_card.rtd->codec->dapm,
			"twl4030 interconnect");

	snd_soc_dapm_sync(&dfl61dac33_sound_card.rtd->codec->dapm);
}
EXPORT_SYMBOL(dfl61dac33_interconnect_enable);


static int dfl61dac33_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret;
	struct snd_soc_codec *codec = rtd->codec;

	/* Add TPA6130A2 controls */
	ret = tpa6130a2_add_controls(codec);
	if (ret < 0)
		return ret;

	snd_soc_add_card_controls(rtd->card, dfl61dac33_controls,
				ARRAY_SIZE(dfl61dac33_controls));

	snd_soc_dapm_new_controls(&codec->dapm, dfl61dac33_dapm_widgets,
				ARRAY_SIZE(dfl61dac33_dapm_widgets));

	/* Set up DFL61 specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_limit_volume(codec, "TPA6140A2 Headphone Playback Volume", 21);

	snd_soc_dapm_disable_pin(&codec->dapm, "twl4030 interconnect");

	/* Set the default McBSP mode to Threshold + SMARTIDLE */
	set_mode(codec, 3);
	return 0;
}

static struct snd_soc_ops dfl61dac33_ops = {
	.hw_params = dfl61dac33_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link dfl61dac33_dai[] = {
	{
		.name = "TLV320DAC33",
		.stream_name = "DAC33",
		.cpu_dai_name = "omap-mcbsp.2",
		.codec_dai_name = "tlv320dac33-hifi",
		.platform_name = "omap-pcm-audio",
		.codec_name = "tlv320dac33-codec.2-0019",
		.init = dfl61dac33_init,
		.ops = &dfl61dac33_ops,
	},
};

/* Audio card */
static struct snd_soc_card dfl61dac33_sound_card = {
	.name = "dfl61-dac33",
	.owner = THIS_MODULE,
	.dai_link = dfl61dac33_dai,
	.num_links = ARRAY_SIZE(dfl61dac33_dai),	
};

static __devinit int dfl61dac33_probe(struct platform_device *pdev)
{
	int err;
	struct snd_soc_card *card = &dfl61dac33_sound_card;

	if (!machine_is_nokia_rm680() && !machine_is_nokia_rm696())
		return -ENODEV;

	printk(KERN_INFO "ALSA for TLV320DAC33 on Nokia DFL61 init\n");

	card->dev = &pdev->dev;

	err = snd_soc_register_card(card);
	if (err) {
		printk(KERN_ERR "Error: %d\n", err);
		return err;
	}
	printk(KERN_INFO "DFL61 TLV320DAC33 init done\n");

	return 0;

}

static int __devexit dfl61dac33_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = &dfl61dac33_sound_card;

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver dfl61dac33_driver = {
	.probe  = dfl61dac33_probe,
	.remove = __devexit_p(dfl61dac33_remove),
	.driver = {
		.name = "dfl61audio-tlv320dac33",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(dfl61dac33_driver);

MODULE_AUTHOR("Peter Ujfalusi");
MODULE_DESCRIPTION("ALSA SoC for tlv320dac33 on Nokia DFL61 class devices");
MODULE_LICENSE("GPL");
