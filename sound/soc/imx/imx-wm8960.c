/*
 * sound/soc/imx/3ds-wm8960.c --  SoC audio for i.MX 3ds boards with
 *                                  wm8960 codec
 *
 * Copyright 2009 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <mach/audmux.h>

#include "../codecs/wm8960.h"
#include "imx-ssi.h"

static struct imx_wm8960_priv {
	int mclk;
	int sysclk;
	int hw;
	struct platform_device *pdev;
} card_priv;

static struct snd_soc_jack hs_jack;
static struct snd_soc_card imx_wm8960;

/* Headphones jack detection DAPM pins */
static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Ext Spk",
		.mask = SND_JACK_HEADPHONE,
		.invert = 1,
	},
 };

/* Headphones jack detection gpios */
static struct snd_soc_jack_gpio hs_jack_gpios[] = {
	[0] = {
		/* gpio is set on per-platform basis */
		.name           = "hp-gpio",
		.report         = SND_JACK_HEADPHONE,
		.debounce_time	= 200,
	},
};

static int bclk_divs[] = {
	10, 15, 20, 30, 40, 55, 60, 80, 110, 120, 160, 220, 240, 320
};

static int dac_divs[] = {
	10*256, 15*256, 20*256, 30*256, 40*256, 55*256, 60*256
};

static int wm8960_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct imx_wm8960_priv *priv = &card_priv;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 dai_format;
	unsigned int channels = params_channels(params);
	int lrclk_rate, bclk_rate, sysdiv, dacdiv, bclkdiv;
	int best, i;
	int ret;

	lrclk_rate = params_rate(params);

	bclk_rate = lrclk_rate * 2;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bclk_rate *= 16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		bclk_rate *= 20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		bclk_rate *= 24;
		break;
	default:
		return -EINVAL;
	}

	pr_debug("\nSYSCLK is %dHz, target BCLK %dHz\n",
		 priv->sysclk, bclk_rate);

	/* Divide SYSCLK by 2 : 11.2896MHz = 22.5792MHz / 2  */
	sysdiv = WM8960_SYSCLK_DIV_2;
	priv->sysclk = priv->mclk / 2;

	/* DACCLK: 44.1KHz = 11.2896 / (1.0 * 256) */
	best = 0;
	for (i = 0; i < ARRAY_SIZE(bclk_divs); i++) {
		ret = (priv->sysclk * 10 / dac_divs[i]) - lrclk_rate;
		if (ret < 0) /* BCLK table is sorted */
			break;
		best = i;
	}
	dacdiv = best << 3;

	/* Class D Switching Clock */
	snd_soc_dai_set_clkdiv(codec_dai, WM8960_DCLKDIV, 0x1c0);

	/* BCLK: */
	best = 0;
	for (i = 0; i < ARRAY_SIZE(bclk_divs); i++) {
		ret = (priv->sysclk * 10 / bclk_divs[i]) - bclk_rate;
		if (ret < 0) /* BCLK table is sorted */
			break;
		best = i;
	}
	bclk_rate = priv->sysclk * 10 / bclk_divs[best];
	bclkdiv = best << 0;

	pr_debug("\nUsing BCLK_DIV %d for actual BCLK %dHz\n",
		 bclk_divs[best], bclk_rate);
	pr_debug("\nsysdiv %d, dacdiv %d, bclkdiv %d\n", sysdiv, dacdiv, bclkdiv);

	snd_soc_dai_set_clkdiv(codec_dai, WM8960_SYSCLKDIV, sysdiv);
	snd_soc_dai_set_clkdiv(codec_dai, WM8960_DACDIV, dacdiv);
	snd_soc_dai_set_clkdiv(codec_dai, WM8960_BCLKDIV, bclkdiv);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* TODO: The SSI driver should figure this out for us */
	switch (channels) {
	case 2:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffc, 0xfffffffc, 2, 0);
		break;
	case 1:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffe, 0xfffffffe, 1, 0);
		break;
	default:
		return -EINVAL;
	}

	/* set cpu DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops imx_wm8960_hifi_ops = {
	.hw_params = wm8960_params,
};

static int wm8960_jack_func;
static int wm8960_spk_func;

static const char *jack_function[] = { "off", "on"};

static const char *spk_function[] = { "off", "on" };

static const struct soc_enum wm8960_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static int wm8960_get_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = wm8960_jack_func;
	return 0;
}

static int wm8960_set_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (wm8960_jack_func == ucontrol->value.enumerated.item[0])
		return 0;

	wm8960_jack_func = ucontrol->value.enumerated.item[0];
	if (wm8960_jack_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphone Jack");

	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int wm8960_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = wm8960_spk_func;
	return 0;
}

static int wm8960_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (wm8960_spk_func == ucontrol->value.enumerated.item[0])
		return 0;

	wm8960_spk_func = ucontrol->value.enumerated.item[0];
	if (wm8960_spk_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Ext Spk");

	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int spk_amp_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct imx_wm8960_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->amp_enable == NULL)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event))
		plat->amp_enable(1);
	else
		plat->amp_enable(0);

	return 0;
}

/* imx_3stack card dapm widgets */
static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", spk_amp_event),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_kcontrol_new wm8960_machine_controls[] = {
	SOC_ENUM_EXT("Headphone Jack", wm8960_enum[0], wm8960_get_jack, wm8960_set_jack),
	SOC_ENUM_EXT("Ext Spk", wm8960_enum[1], wm8960_get_spk, wm8960_set_spk),
};

/* imx_3stack machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {

	/* HP_OUT --> Headphone Jack */
	{"Headphone Jack", NULL, "HP_L"},
	{"Headphone Jack", NULL, "HP_R"},

	/* LINE_OUT --> Ext Speaker */
	{"Ext Spk", NULL, "SPK_LN"},
	{"Ext Spk", NULL, "SPK_LP"},
	{"Ext Spk", NULL, "SPK_RN"},
	{"Ext Spk", NULL, "SPK_RP"},
};

static int imx_3stack_wm8960_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

	ret = snd_soc_add_controls(codec, wm8960_machine_controls,
			ARRAY_SIZE(wm8960_machine_controls));
	if (ret)
		return ret;

	/* Add imx_3stack specific widgets */
	snd_soc_dapm_new_controls(&codec->dapm, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	/* Set up imx_3stack specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_disable_pin(&codec->dapm, "Line In Jack");
	snd_soc_dapm_disable_pin(&codec->dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(&codec->dapm, "Ext Spk");
	snd_soc_dapm_sync(&codec->dapm);

	if (hs_jack_gpios[0].gpio != -1) {
		/* Jack detection API stuff */
		ret = snd_soc_jack_new(codec, "Headphone Jack",
				       SND_JACK_HEADPHONE, &hs_jack);
		if (ret)
			return ret;

		ret = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins),
					hs_jack_pins);
		if (ret) {
			printk(KERN_ERR "failed to call  snd_soc_jack_add_pins\n");
			return ret;
		}

		ret = snd_soc_jack_add_gpios(&hs_jack,
					ARRAY_SIZE(hs_jack_gpios), hs_jack_gpios);
		if (ret)
			printk(KERN_WARNING "failed to call snd_soc_jack_add_gpios\n");
	}

	return 0;
}

static struct snd_soc_dai_link imx_wm8960_dai[] = {
	{
		.name		= "HiFi",
		.stream_name	= "HiFi",
		.codec_dai_name	= "wm8960-hifi",
		.codec_name	= "wm8960-codec.1-001a",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.init		= imx_3stack_wm8960_init,
		.ops		= &imx_wm8960_hifi_ops,
	},
};

static struct snd_soc_card imx_wm8960 = {
	.name		= "wm8960-audio",
	.dai_link	= imx_wm8960_dai,
	.num_links	= ARRAY_SIZE(imx_wm8960_dai),
	//.pop_time	= 1,
};

static struct platform_device *imx_wm8960_snd_device;

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	/* SSI0 mastered by port 5 */
	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

static int __devinit imx_wm8960_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	int ret = 0;

	card_priv.pdev = pdev;

	imx_audmux_config(plat->src_port, plat->ext_port);

	ret = -EINVAL;
	if (plat->init && plat->init())
		return ret;

	card_priv.mclk = plat->sysclk;

	hs_jack_gpios[0].gpio = plat->hp_gpio;
	hs_jack_gpios[0].invert = plat->hp_active_low;

	return 0;
}

static int imx_wm8960_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->finit)
		plat->finit();

	return 0;
}

static int imx_wm8960_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->osc_enable)
		plat->osc_enable(0);

	return 0;
}

static int imx_wm8960_resume(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->osc_enable)
		plat->osc_enable(1);

	return 0;
}

static void imx_wm8960_shutdown(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->osc_enable)
		plat->osc_enable(0);
}

static struct platform_driver imx_wm8960_audio_driver = {
	.probe = imx_wm8960_probe,
	.remove = imx_wm8960_remove,
	.shutdown = imx_wm8960_shutdown,
	.suspend = imx_wm8960_suspend,
	.resume = imx_wm8960_resume,
	.driver = {
		   .name = "imx-wm8960",
		   },
};

static int __init imx_wm8960_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_wm8960_audio_driver);
	if (ret)
		return -ENOMEM;

	imx_wm8960_snd_device = platform_device_alloc("soc-audio", 1);
	if (!imx_wm8960_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_wm8960_snd_device, &imx_wm8960);

	ret = platform_device_add(imx_wm8960_snd_device);

	if (ret) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		platform_device_put(imx_wm8960_snd_device);
	}

	return ret;
}

static void __exit imx_wm8960_exit(void)
{
	platform_driver_unregister(&imx_wm8960_audio_driver);
	platform_device_unregister(imx_wm8960_snd_device);
}

module_init(imx_wm8960_init);
module_exit(imx_wm8960_exit);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("PhyCORE ALSA SoC driver");
MODULE_LICENSE("GPL");
