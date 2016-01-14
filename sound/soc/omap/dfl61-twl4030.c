/*
 * dfl61-twl4030.c -- SoC audio for TWL4030 on Nokia DFL61 class devices
 *
 * Author: Peter Ujfalusi <peter.ujfalusi@nokia.com>
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
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <plat/dfl61-audio.h>
#include <linux/mfd/twl4030-audio.h>
#include <plat/omap_hwmod.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include "../fs/sysfs/sysfs.h"


#include <asm/mach-types.h>

#include "omap-mcbsp.h"

#define IHF_ENABLE_GPIO		(192 + 7)
#define PIEZO_ENABLE_GPIO	(98)
#define JACK_REPORT_MASK	(SND_JACK_MECHANICAL | SND_JACK_AVOUT | \
				 SND_JACK_HEADSET)
 
static enum dfl61audio_twl4030_config audio_config;
static unsigned int sys_freq;

static struct snd_soc_card dfl61twl_sound_card;
static struct snd_soc_jack dfl61_jack;

static struct dfl61audio_hsmic_event *hsmic_event;

#define to_device_attribute(x) container_of(x, struct device_attribute, attr)

int dfl61_set_dma_op_mode(const char *omap_mcbsp_name, bool threshold)
{
	struct device_attribute *dma_op_mode_device_attribute = NULL;
	const char *dma_op_mode_attribute_name="dma_op_mode";
	const char *dma_op_mode_threshold = "threshold";
	const char *dma_op_mode_element = "element";
	struct sysfs_dirent *sd;
	struct device *omap_mcbsp_dev = bus_find_device_by_name(&platform_bus_type, NULL, omap_mcbsp_name);
	/*u8 idlemode;
	struct omap_hwmod *oh = omap_hwmod_lookup("mcbsp3");*/

	if (!omap_mcbsp_dev) {
		printk(KERN_ERR "Can't find %s device", omap_mcbsp_name);
		return -ENODEV;
	}

	if (!omap_mcbsp_dev->kobj.sd) {
		printk(KERN_ERR "%s: no sd entry", omap_mcbsp_name);
		return -ENODEV;
	}

	sd = sysfs_get_dirent(omap_mcbsp_dev->kobj.sd, NULL, dma_op_mode_attribute_name);
	if (!sd) {
		printk(KERN_ERR "%s: sd contains no %s", omap_mcbsp_name, dma_op_mode_attribute_name);
		return -ENODEV;
	}
	
	if (!sd->s_attr.attr) {
		printk(KERN_ERR "%s: %s sys_dirent has no s_attr.attr", omap_mcbsp_name, dma_op_mode_attribute_name);
		return -ENODEV;
	}

	dma_op_mode_device_attribute = to_device_attribute(sd->s_attr.attr);
	if (!dma_op_mode_device_attribute) {
		printk(KERN_ERR "%s: device_attribute not found", omap_mcbsp_name);
		return -ENODEV;
	}

	dma_op_mode_device_attribute->store(omap_mcbsp_dev, dma_op_mode_device_attribute, (threshold) ? dma_op_mode_threshold : dma_op_mode_element, strlen((threshold) ? dma_op_mode_threshold : dma_op_mode_element));
	
	/*FIXME causes In-band Error seen by MPU at address 0*/
	/*if (!oh) {
		printk(KERN_ERR "Unable to find omap_hwmod mcbsp3\n");
		return -ENODEV;
	}
	
	if (oh->class->sysc->idlemodes & SIDLE_SMART_WKUP)
		idlemode = HWMOD_IDLEMODE_SMART_WKUP;
	else
		idlemode = HWMOD_IDLEMODE_SMART;
	omap_hwmod_set_slave_idlemode(oh, idlemode);*/

	/* Create jack for accessory reporting */
	return 0;
}
EXPORT_SYMBOL(dfl61_set_dma_op_mode);

void dfl61_jack_report(int status)
{
	if (dfl61_jack.codec->card)
		snd_soc_jack_report(&dfl61_jack, status, JACK_REPORT_MASK);
	else
		pr_err("dfl61-twl4030: Cannot report jack status");
}
EXPORT_SYMBOL(dfl61_jack_report);

int dfl61_request_hsmicbias(bool enable)
{
	struct snd_soc_dapm_context *dapm = &dfl61twl_sound_card.rtd->codec->dapm;

	if (!dapm) {
		pr_err("dfl61-twl4030: Cannot set hsmicbias yet");
		return -ENODEV;
	}

	if (enable)
		snd_soc_dapm_force_enable_pin(dapm, "Headset Mic Bias");
	else
		snd_soc_dapm_disable_pin(dapm, "Headset Mic Bias");

	return snd_soc_dapm_sync(dapm);
}
EXPORT_SYMBOL(dfl61_request_hsmicbias);

void dfl61_register_hsmic_event_cb(struct dfl61audio_hsmic_event *event)
{
	hsmic_event = event;
}
EXPORT_SYMBOL(dfl61_register_hsmic_event_cb);

static int dfl61twl_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;

	unsigned int fmt;
	int ret;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;

		if (audio_config == AUDIO_CONFIG3)
			snd_soc_dapm_disable_pin(&card->dapm, "Digital2 Mic");
		break;
	case 4: /* Four channel TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;

		if (audio_config == AUDIO_CONFIG3)
			snd_soc_dapm_enable_pin(&card->dapm, "Digital2 Mic");
		break;
	default:
		return -EINVAL;
	}
	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("dfl61-twl4030: Can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		pr_err("dfl61-twl4030: Can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sys_freq,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("dfl61-twl4030: Can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static int dfl61twl_spk_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		gpio_set_value_cansleep(IHF_ENABLE_GPIO, 1);
	else
		gpio_set_value_cansleep(IHF_ENABLE_GPIO, 0);

	return 0;
}

static int dfl61twl_dac33_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		dfl61dac33_interconnect_enable(1);
	else
		dfl61dac33_interconnect_enable(0);

	return 0;
}

static int dfl61twl_piezo_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		gpio_set_value(PIEZO_ENABLE_GPIO, 1);
	else
		gpio_set_value(PIEZO_ENABLE_GPIO, 0);

	return 0;
}

static int dfl61twl_hsmic_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	if (!hsmic_event || !hsmic_event->event)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event))
		hsmic_event->event(hsmic_event->private, 1);
	else
		hsmic_event->event(hsmic_event->private, 0);

	return 0;
}

/* DAPM widgets and routing for audio config1 */
static const struct snd_soc_dapm_widget config1_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", dfl61twl_spk_event),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_SPK("Vibra", NULL),
	SND_SOC_DAPM_SPK("Piezo", dfl61twl_piezo_event),

	SND_SOC_DAPM_MIC("Digital Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static const struct snd_soc_dapm_route config1_audio_map[] = {
	{"Ext Spk", NULL, "PREDRIVER"},
	{"Earpiece", NULL, "EARPIECE"},
	{"Vibra", NULL, "VIBRA"},
	{"Piezo", NULL, "HSOL"},

	{"DIGIMIC0", NULL, "Mic Bias 1"},
	{"Mic Bias 1", NULL, "Digital Mic"},

	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},
};

/* DAPM widgets and routing for audio config2 */
static const struct snd_soc_dapm_widget config2_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", dfl61twl_spk_event),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_SPK("Vibra", NULL),
	SND_SOC_DAPM_SPK("Piezo", dfl61twl_piezo_event),
	SND_SOC_DAPM_SPK("DAC33 interconnect", dfl61twl_dac33_event),

	SND_SOC_DAPM_MIC("Digital Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),

	SND_SOC_DAPM_LINE("FMRX Left Line-in", NULL),
	SND_SOC_DAPM_LINE("FMRX Right Line-in", NULL),
};

static const struct snd_soc_dapm_route config2_audio_map[] = {
	{"Ext Spk", NULL, "PREDRIVER"},
	{"Earpiece", NULL, "EARPIECE"},
	{"Vibra", NULL, "VIBRA"},
	{"Piezo", NULL, "HSOL"},
	{"DAC33 interconnect", NULL, "PREDRIVEL"},

	{"DIGIMIC0", NULL, "Mic Bias 1"},
	{"Mic Bias 1", NULL, "Digital Mic"},

	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},

	{"AUXL", NULL, "FMRX Left Line-in"},
	{"AUXR", NULL, "FMRX Right Line-in"},
};

/* DAPM widgets and routing for audio config3 */
static const struct snd_soc_dapm_widget config3_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", dfl61twl_spk_event),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_SPK("HAC", NULL),
	SND_SOC_DAPM_SPK("Vibra", NULL),
	SND_SOC_DAPM_SPK("Piezo", dfl61twl_piezo_event),
	SND_SOC_DAPM_SPK("DAC33 interconnect", dfl61twl_dac33_event),

	SND_SOC_DAPM_MIC("Digital Mic", NULL),
	SND_SOC_DAPM_MIC("Digital2 Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", dfl61twl_hsmic_event),

	SND_SOC_DAPM_LINE("FMRX Left Line-in", NULL),
	SND_SOC_DAPM_LINE("FMRX Right Line-in", NULL),
};

static const struct snd_soc_dapm_route config3_audio_map[] = {
	{"Ext Spk", NULL, "PREDRIVER"},
	{"Earpiece", NULL, "EARPIECE"},
	{"HAC", NULL, "HFL"},
	{"Vibra", NULL, "VIBRA"},
	{"Piezo", NULL, "HSOL"},
	{"DAC33 interconnect", NULL, "PREDRIVEL"},

	{"DIGIMIC0", NULL, "Mic Bias 1"},
	{"Mic Bias 1", NULL, "Digital Mic"},

	{"DIGIMIC1", NULL, "Mic Bias 2"},
	{"Mic Bias 2", NULL, "Digital2 Mic"},

	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},

	{"AUXL", NULL, "FMRX Left Line-in"},
	{"AUXR", NULL, "FMRX Right Line-in"},
};

/* DAPM widgets and routing for audio config4 */
static const struct snd_soc_dapm_widget config4_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", dfl61twl_spk_event),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_SPK("HAC", NULL),
	SND_SOC_DAPM_SPK("Vibra", NULL),
	SND_SOC_DAPM_SPK("DAC33 interconnect", dfl61twl_dac33_event),

	SND_SOC_DAPM_MIC("Digital Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", dfl61twl_hsmic_event),

	SND_SOC_DAPM_LINE("FMRX Left Line-in", NULL),
	SND_SOC_DAPM_LINE("FMRX Right Line-in", NULL),
};

static const struct snd_soc_dapm_route config4_audio_map[] = {
	{"Ext Spk", NULL, "PREDRIVER"},
	{"Earpiece", NULL, "EARPIECE"},
	{"HAC", NULL, "HFL"},
	{"Vibra", NULL, "VIBRA"},
	{"DAC33 interconnect", NULL, "PREDRIVEL"},

	{"DIGIMIC0", NULL, "Mic Bias 1"},
	{"Mic Bias 1", NULL, "Digital Mic"},

	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},

	{"AUXL", NULL, "FMRX Left Line-in"},
	{"AUXR", NULL, "FMRX Right Line-in"},
};

/* Pre DAC routings for the twl4030 codec */
static const char *twl4030_predacl1_texts[] = {
	"SDRL1", "SDRM1", "SDRL2", "SDRM2",
};
static const char *twl4030_predacr1_texts[] = {
	"SDRR1", "SDRM1", "SDRR2", "SDRM2"
};
static const char *twl4030_predacl2_texts[] = {"SDRL2", "SDRM2"};
static const char *twl4030_predacr2_texts[] = {"SDRR2", "SDRM2"};

static const struct soc_enum twl4030_predacl1_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_RX_PATH_SEL, 2,
			ARRAY_SIZE(twl4030_predacl1_texts),
			twl4030_predacl1_texts);

static const struct soc_enum twl4030_predacr1_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_RX_PATH_SEL, 0,
			ARRAY_SIZE(twl4030_predacr1_texts),
			twl4030_predacr1_texts);

static const struct soc_enum twl4030_predacl2_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_RX_PATH_SEL, 5,
			ARRAY_SIZE(twl4030_predacl2_texts),
			twl4030_predacl2_texts);

static const struct soc_enum twl4030_predacr2_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_RX_PATH_SEL, 4,
			ARRAY_SIZE(twl4030_predacr2_texts),
			twl4030_predacr2_texts);



static const struct snd_kcontrol_new dfl61twl_controls[] = {
	/* Mux controls before the DACs */
	SOC_ENUM("DACL1 Playback Mux", twl4030_predacl1_enum),
	SOC_ENUM("DACR1 Playback Mux", twl4030_predacr1_enum),
	SOC_ENUM("DACL2 Playback Mux", twl4030_predacl2_enum),
	SOC_ENUM("DACR2 Playback Mux", twl4030_predacr2_enum),
};

static int dfl61twl_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int err;

	err = snd_soc_jack_new(codec, "Jack",
				JACK_REPORT_MASK , &dfl61_jack);
	if (err < 0)
		return err;
		

	/* Add DFL61 specific controls */
	snd_soc_add_codec_controls(codec, dfl61twl_controls,
				ARRAY_SIZE(dfl61twl_controls));

	/* Disable unused pins */
	snd_soc_dapm_nc_pin(dapm, "MAINMIC");
	snd_soc_dapm_nc_pin(dapm, "CARKITMIC");
	snd_soc_dapm_nc_pin(dapm, "SUBMIC");

	snd_soc_dapm_nc_pin(dapm, "HSOR");
	snd_soc_dapm_nc_pin(dapm, "CARKITL");
	snd_soc_dapm_nc_pin(dapm, "CARKITR");
	snd_soc_dapm_nc_pin(dapm, "HFR");

	switch (audio_config) {
	case AUDIO_CONFIG1:
		/* Disable unused pins */
		snd_soc_dapm_nc_pin(dapm, "AUXL");
		snd_soc_dapm_nc_pin(dapm, "AUXR");
		snd_soc_dapm_nc_pin(dapm, "PREDRIVEL");
		snd_soc_dapm_nc_pin(dapm, "HFL");
		snd_soc_dapm_nc_pin(dapm, "DIGIMIC1");
		/* Add config specific widgets and routes */
		snd_soc_dapm_new_controls(dapm, config1_dapm_widgets,
					ARRAY_SIZE(config1_dapm_widgets));
		snd_soc_dapm_add_routes(dapm, config1_audio_map,
					ARRAY_SIZE(config1_audio_map));
		break;
	case AUDIO_CONFIG2:
		/* Disable unused pins */
		snd_soc_dapm_nc_pin(dapm, "HFL");
		snd_soc_dapm_nc_pin(dapm, "DIGIMIC1");
		/* Add config specific widgets and routes */
		snd_soc_dapm_new_controls(dapm, config2_dapm_widgets,
					ARRAY_SIZE(config2_dapm_widgets));
		snd_soc_dapm_add_routes(dapm, config2_audio_map,
					ARRAY_SIZE(config2_audio_map));
		break;
	case AUDIO_CONFIG3:
		/* Add config specific widgets and routes */
		snd_soc_dapm_new_controls(dapm, config3_dapm_widgets,
					ARRAY_SIZE(config3_dapm_widgets));
		snd_soc_dapm_add_routes(dapm, config3_audio_map,
					ARRAY_SIZE(config3_audio_map));
		break;
	case AUDIO_CONFIG4:
		/* Disable unused pins */
		snd_soc_dapm_nc_pin(dapm, "DIGIMIC1");
		snd_soc_dapm_nc_pin(dapm, "HSOL");
		/* Add config specific widgets and routes */
		snd_soc_dapm_new_controls(dapm, config4_dapm_widgets,
					ARRAY_SIZE(config4_dapm_widgets));
		snd_soc_dapm_add_routes(dapm, config4_audio_map,
					ARRAY_SIZE(config4_audio_map));
		break;
	}

	if (omap_mcbsp_st_add_controls(rtd))
		dev_dbg(codec->dev, "Unable to set Sidetone for McBSP3\n");

	err = dfl61_set_dma_op_mode("omap-mcbsp.3", true);
	if (err < 0)
		return err;

	return 0;
}

static struct snd_soc_ops dfl61twl_ops = {
	.hw_params = dfl61twl_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link dfl61twl_dai[] = {
	{
		.name = "TWL4030",
		.stream_name = "TWL4030",
		.cpu_dai_name = "omap-mcbsp.3",
		.codec_dai_name = "twl4030-hifi",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF |
			   SND_SOC_DAIFMT_CBM_CFM,
		.init = dfl61twl_init,
		.ops = &dfl61twl_ops,
	},
};

/* Audio card */
static struct snd_soc_card dfl61twl_sound_card = {
	.name = "dfl61-twl4030",
	.owner = THIS_MODULE,
	.dai_link = dfl61twl_dai,
	.num_links = ARRAY_SIZE(dfl61twl_dai),	
};

static __devinit int dfl61twl_probe(struct platform_device *pdev)
{
	int err;
	struct dfl61audio_twl4030_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct snd_soc_card *card = &dfl61twl_sound_card;

	if (!machine_is_nokia_rm680() && !machine_is_nokia_rm696())
		return -ENODEV;	
	printk(KERN_INFO "ALSA SoC for TWL4030 on Nokia DFL61 init\n");

	card->dev = &pdev->dev;

	if (!pdata) {
		dev_err(&pdev->dev, "Missing pdata\n");
		return -ENODEV;
	}

	audio_config = pdata->audio_config;
	sys_freq = pdata->freq;

	err = gpio_request_one(IHF_ENABLE_GPIO,
			       GPIOF_DIR_OUT | GPIOF_INIT_LOW, "IHF_EN");
	if (err < 0) {
		pr_err("dfl61-twl4030: Can not get gpio for IHF\n");
		goto err_gpio;
	}

	if (audio_config != AUDIO_CONFIG4) {
		gpio_request_one(PIEZO_ENABLE_GPIO,
			       GPIOF_DIR_OUT | GPIOF_INIT_LOW, "PIEZO_EN");
		if (err < 0) {
			pr_err("dfl61-twl4030: Can not get gpio for Piezo\n");
			gpio_free(IHF_ENABLE_GPIO);
			goto err_gpio;
		}
	}

	err = snd_soc_register_card(card);
	if (err) {
		printk(KERN_ERR "Error: %d\n", err);
		goto err1;
	}

	printk(KERN_INFO "DFL61 TWL SoC init done (config %d)\n", audio_config + 1);

	return 0;
err1:
	gpio_free(IHF_ENABLE_GPIO);
	if (audio_config != AUDIO_CONFIG4)
		gpio_free(PIEZO_ENABLE_GPIO);
	printk(KERN_ERR "Unable to add platform device\n");	
err_gpio:

	return err;
}

static int __devexit dfl61twl_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = &dfl61twl_sound_card;

	snd_soc_unregister_card(card);
	gpio_free(IHF_ENABLE_GPIO);

	if (audio_config != AUDIO_CONFIG4)
		gpio_free(PIEZO_ENABLE_GPIO);

	return 0;
}

static struct platform_driver dfl61twl_driver = {
	.probe  = dfl61twl_probe,
	.remove = __devexit_p(dfl61twl_remove),
	.driver = {
		.name = "dfl61audio-twl4030",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(dfl61twl_driver);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@nokia.com>");
MODULE_DESCRIPTION("ALSA SoC for TWL4030 on Nokia DFL61 class devices");
MODULE_LICENSE("GPL");
