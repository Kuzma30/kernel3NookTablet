/*
 * omap4_aic3100.c - SoC audio for acclaim board
 *
 * Copyright (C) 2010 Mistral Solutions
 * Santosh Sivaraj <santosh.s@mistralsolutions.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <asm/mach-types.h>
#include <plat/hardware.h>
#include <plat/mcbsp.h>
#include <plat/mux.h>
#include "abe/abe_main.h"
#include "omap-abe.h"
#include "omap-dmic.h"
#include "omap-pcm.h"
#include "omap-mcbsp.h"

#include "../codecs/tlv320aic3100.h"

#undef DEBUG
#ifdef DEBUG
#define DBG(x...)	printk(x)
#else
#define DBG(x...)
#endif

#if defined(CONFIG_SND_OMAP_SOC_ACCLAIM_NO_ABE) &&	\
	defined(CONFIG_SND_OMAP_SOC_ACCLAIM)
#error "Cannot compile with and without ABE support"
#endif

#define ACCLAIM_HS_DETECT_GPIO 	102

static struct wake_lock omap_wakelock;
static struct clk *sys_clkout2;

static int acclaim_headset_jack_status_check(void);

static struct i2c_client *aic3100_client;
static struct snd_soc_jack hs_jack;

static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headset Stereophone",
		.mask = SND_JACK_HEADPHONE,
	},
};

/*
 * Headset jack detection gpios
 */
static struct snd_soc_jack_gpio hs_jack_gpios[] = {
	{
		.gpio = ACCLAIM_HS_DETECT_GPIO,
		.name = "hsdet-gpio",
		.report = SND_JACK_HEADSET,
		.debounce_time = 200,
		.jack_status_check = acclaim_headset_jack_status_check,
	},
};

/*
 * acclaim_evt_hw_params - Machine Driver's hw_params call-back handler routine.
 */
static int acclaim_evt_hw_params (struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	void __iomem *phymux_base = NULL;
	u32 phy_val;
	int ret;

	DBG(KERN_INFO "omap4_hw_params invoked... New IMGAE\n");

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Enabling the 19.2 Mhz Master Clock Output
		from OMAP4 for Acclaim Board */
	phymux_base = ioremap(0x4A30A000, 0x1000);
	phy_val = __raw_readl(phymux_base + 0x0314);
	phy_val = (phy_val & 0xFFF0FEFF) | (0x00010100);
	__raw_writel(phy_val, phymux_base + 0x0314);
	iounmap(phymux_base);


	/* Mistral added the test code to configure the McBSP4 CONTROL_MCBSP_LP
	 * register. This register ensures that the FSX and FSR on McBSP4 are
	 * internally short and both of them see the same signal from the
	 * External Audio Codec.
	 */

	/*	The  following is required for mcbsp 4 operation.
	 *	The folowing lines set mcbsp 4 mux configurations on omap processor
	 *	uncomment the following 4 lines if you intend on using mcbsp4
	 */
	/*
	  phymux_base = ioremap (0x4a100000, 0x1000);
	  __raw_writel (0xC0000000, phymux_base + 0x61c);
	  __raw_writel (0x05020502, phymux_base + 0xc4);
	  __raw_writel (0x05020502, phymux_base + 0xc8);
	  __raw_writel (0x05020002, phymux_base + 0xcc);

	  iounmap(phymux_base);
	*/
	/* Set the codec system clock for DAC and ADC. The
         * third argument is specific to the board being used.
         */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 19200000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	DBG(KERN_INFO "omap4_hw_params passed...\n");

	return 0;
}

/*
 * acclaim_evt_startup - Machine Driver's startup routine.
 */
static int acclaim_evt_startup (struct snd_pcm_substream *substream)
{
	DBG(KERN_ALERT "%s: initiating wake lock\n", __func__);
	wake_lock(&omap_wakelock);
	DBG(KERN_ALERT "%s: calling clk_enable(sys_clkout2)", __func__);
	return 0;
}

/*
 * acclaim_evt_shutdown - Machine Driver's shutdown routine.
 */
static void acclaim_evt_shutdown (struct snd_pcm_substream *substream)
{
//	clk_disable(sys_clkout2);
	wake_unlock(&omap_wakelock);
}

static struct snd_soc_ops acclaim_ops = {
//	.startup = acclaim_evt_startup,
	.hw_params = acclaim_evt_hw_params,
//	.shutdown = acclaim_evt_shutdown,
};

static const struct snd_soc_dapm_widget aic3100_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Ext Spk", NULL),
};

/*
 * Initialization routine.
 */
static int acclaim_aic3100_init (struct snd_soc_pcm_runtime *pcm)
{
	struct snd_soc_codec *codec = pcm->codec;
	int gpiostatus;

        DBG(KERN_INFO  "acclaim_aic3100_init..\n");

	gpiostatus = snd_soc_jack_new(codec, "Headset Jack",
				      SND_JACK_HEADSET, &hs_jack);
	if (gpiostatus != 0) {
		printk (KERN_ERR "snd_soc_jack_new failed(%d)\n", gpiostatus);
        }

	gpiostatus = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins),
					   hs_jack_pins);
	if (gpiostatus != 0) {
		printk (KERN_ERR"snd_soc_jack_add_pins failed(%d)\n",
			gpiostatus);
        }

	gpiostatus = snd_soc_jack_add_gpios(&hs_jack, ARRAY_SIZE(hs_jack_gpios),
					    hs_jack_gpios);

        if (gpiostatus != 0)
		printk (KERN_ERR "snd_soc_jack_add_gpios failed..%d\n",
			gpiostatus);

	/* For our Driver, the Codec Driver itself manages the POP
	*  polling and hence
	* we will reset the ALSA pmdown_time to zero.
	*/
	pcm->pmdown_time = 0;
	return 0;
}


static int mcbsp_be_hw_params_fixup (struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
							  SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int be_id;
	unsigned int threshold;
	unsigned int val, min_mask;

	be_id = rtd->dai_link->be_id;

	switch (be_id) {
	case OMAP_ABE_DAI_MM_FM:
		channels->min = 2;
		threshold = 2;
                val = SNDRV_PCM_FORMAT_S16_LE;
		break;
	case OMAP_ABE_DAI_BT_VX:
		channels->min = 1;
		threshold = 1;
                val = SNDRV_PCM_FORMAT_S16_LE;
		break;
	default:
		threshold = 1;
                val = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}

        min_mask = snd_mask_min(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
                                               SNDRV_PCM_HW_PARAM_FIRST_MASK]);


        snd_mask_reset(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
                                      SNDRV_PCM_HW_PARAM_FIRST_MASK],
		       min_mask);

        snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
                                    SNDRV_PCM_HW_PARAM_FIRST_MASK], val);

	omap_mcbsp_set_tx_threshold(cpu_dai->id, threshold);
	omap_mcbsp_set_rx_threshold(cpu_dai->id, threshold);
	return 0;
}


/*
 * This function is to check the Headset Jack Status
 */
static int acclaim_headset_jack_status_check (void)
{
	int gpio_status;

	gpio_status = gpio_get_value(ACCLAIM_HS_DETECT_GPIO);
	/*
	 * If the js_jack codec Member is not empty, Invoke the
	 * headset_Speaker_Path routine Codec Driver.
	 */
	if (hs_jack.codec != NULL)
		aic3100_headset_speaker_path (hs_jack.codec, !gpio_status);
	return 0;
}

static struct snd_soc_dai_driver mach_dai[] = {
	{
		.name = "aic3100_mach_dai",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC3100_RATES,
			.formats = AIC3100_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC3100_RATES,
			.formats = AIC3100_FORMATS,
		},
	},
/* TODO: make this a separate FM CODEC driver or DUMMY */
	{
		.name = "FM Digital",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};
static const char *mm1_be[] = {
	OMAP_ABE_BE_MM_EXT0,
};

static const char *mm_lp_be[] = {
	OMAP_ABE_BE_MM_EXT0,
};

/* ABE Port configuration structure introduced within the
 * DAI_LINK Structure as private_data void pointer member
 */
t_port_config mm_ext0_config = {
	/* uplink port configuration */
	.abe_port_id_ul = MM_EXT_IN_PORT,
	.serial_id_ul = MCBSP2_RX,
	.sample_format_ul = STEREO_RSHIFTED_16,
#ifdef CONFIG_ABE_44100
	.sample_rate_ul = 44100,
#else
	.sample_rate_ul = 48000,
#endif
	.bit_reorder_ul = 0,

	/* down link port configuration */
	.abe_port_id_dl = MM_EXT_OUT_PORT,
	.serial_id_dl = MCBSP2_TX,
	.sample_format_dl = STEREO_RSHIFTED_16,
#ifdef CONFIG_ABE_44100
	.sample_rate_dl = 44100,
#else
	.sample_rate_dl = 48000,
#endif
	.bit_reorder_dl = 0,
};

/* DAI_LINK Structure definition with both Front-End and
 * Back-end DAI Declarations.
 */
static struct snd_soc_dai_link acclaim_dai_link_abe[] = {
	{
		.name = "tlv320aic3100 LP",
		.stream_name = "Multimedia",

		/* ABE components - MM-DL (mmap) */
		.cpu_dai_name = "MultiMedia1 LP",
		.platform_name = "omap-aess-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm_lp_be,
		.num_be = ARRAY_SIZE(mm_lp_be),
		.fe_playback_channels = 2,
		.no_host_mode = SND_SOC_DAI_LINK_OPT_HOST,
	},
	{
		.name = "tlv320aic3100",
		.stream_name = "Multimedia",

		/* ABE components - MM-UL & MM_DL */
		.cpu_dai_name = "MultiMedia2",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm1_be,
		.num_be = ARRAY_SIZE(mm1_be),
		.fe_playback_channels = 2,
		.fe_capture_channels = 8,
		.no_host_mode = SND_SOC_DAI_LINK_OPT_HOST,
	},
	/*{
		.name = "tlv320aic3100-bypass-abe",
		.stream_name = "Multimedia Capture",

		.cpu_dai_name = "MultiMedia2",
		.platform_name = "omap-pcm-audio",

		.codec_name = "tlv320aic3100-codec.2-0018",
		.codec_dai_name = "tlv320aic3100-dai",
		.init = acclaim_aic3100_init,
		.ops = &acclaim_ops,
	},*/
/*
 * Backend DAIs - i.e. dynamically matched interfaces, invisible to userspace.
 * Matched to above interfaces at runtime, based upon use case.
 */

	{
		.name = OMAP_ABE_BE_MM_EXT0,
		.stream_name = "FM",

		/* ABE components - MCBSP3 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.platform_name = "omap-aess-audio",

		/* FM */
		.codec_dai_name = "tlv320aic3100-dai",
		.codec_name = "tlv320aic3100-codec.2-0018",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &acclaim_ops,
		.be_id = OMAP_ABE_DAI_MM_FM,
		.private_data = &mm_ext0_config,
		.init = acclaim_aic3100_init,
	},
};
static struct snd_soc_dai_link acclaim_dai_link[] = {
        {
                .name = "tlv320aic3100",
                .stream_name = "AIC3100 Audio",
                .codec_dai_name = "tlv320aic3100-dai",
                .cpu_dai_name = "omap-mcbsp-dai.3",
                .codec_name = "tlv320aic3100-codec.2-0018",
                .platform_name = "omap-pcm-audio",
                .init = acclaim_aic3100_init,
                .ops = &acclaim_ops,
        },
};


static struct snd_soc_card snd_soc_card_acclaim = {
	.name = "OMAP4_ACCLAIM",
	.long_name = "OMAP4 Acclaim AIC3100",
	.dai_link = acclaim_dai_link,
	.num_links = ARRAY_SIZE(acclaim_dai_link),
};

static struct snd_soc_card snd_soc_card_acclaim_abe = {
	.name = "OMAP4_ACCLAIM_ABE",
	.long_name = "OMAP4 Acclaim AIC3100 ABE",
	.dai_link = acclaim_dai_link_abe,
	.num_links = ARRAY_SIZE(acclaim_dai_link_abe),
};

static struct platform_device *acclaim_snd_device;

/*
 * Initialization routine.
 */
static int __init acclaim_soc_init (void)
{
	int ret = 0;
	struct device *dev;
	unsigned char board_ver = 3;
	void __iomem *phymux_base = NULL;
	u32 val;

	printk(KERN_INFO "omap3epd-sound: Audio SoC init\n");
	acclaim_snd_device = platform_device_alloc("soc-audio", -1);
	if (!acclaim_snd_device) {
		printk(KERN_INFO "Platform device allocation failed\n");
		return -ENOMEM;
	}
#ifdef CONFIG_ABE_44100
#warning "Configuring the ABE at 44.1 Khz."
#endif
	if(board_ver == 3) {
		printk(KERN_INFO "Found Board EVT 2.1-ABE support Enabled\n");
		platform_set_drvdata(acclaim_snd_device, &snd_soc_card_acclaim_abe);
	} else {
		printk(KERN_INFO "Not found Board EVT 2.1 - ABE support ommitted.\n");
		platform_set_drvdata(acclaim_snd_device, &snd_soc_card_acclaim);
	}
	ret = platform_device_add(acclaim_snd_device);
	if (ret){
		printk(KERN_INFO "%s: platform device allocation failed\n", __func__);
		goto err1;
	}
	snd_soc_register_dais(&acclaim_snd_device->dev, mach_dai, ARRAY_SIZE(mach_dai));
	dev = &acclaim_snd_device->dev;

	/*
	 * Enable the GPIO related code-base on the ACCLAIM Board for
	 * Headphone/MIC Detection
	 */
	phymux_base = ioremap (0x4a100000, 0x1000);
	val = __raw_readl(phymux_base + 0x90);
	val =  ((val & 0xFEFCFFFE) | 0x01030003);
	/*__raw_writel (0x01030003, phymux_base + 0x90); */
	__raw_writel (val, phymux_base + 0x90);
	iounmap(phymux_base);
	return 0;

err1:
	platform_device_put(acclaim_snd_device);

	return ret;
}

/*
 * shutdown routine.
 */
static void __exit acclaim_soc_exit (void)
{
	wake_lock_destroy(&omap_wakelock);
	platform_device_unregister(acclaim_snd_device);
}

module_init(acclaim_soc_init);
module_exit(acclaim_soc_exit);

MODULE_AUTHOR("Santosh Sivaraj <santosh.s@mistralsolutions.com>");
MODULE_DESCRIPTION("ALSA SoC for Acclaim Board");
MODULE_LICENSE("GPL");
