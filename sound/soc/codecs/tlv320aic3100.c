/*
 * linux/sound/soc/codecs/tlv320aic3100.c
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * Based on sound/soc/codecs/wm8753.c by Liam Girdwood
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Revision History
 *
 * 0.1			Nov-2010	Reference driver implentation for
 *					OMAP3 Platforms
 * 0.2			Jun-2011	Ported the code-base to 2.6.35 Kernel
 * 0.3			Jul-2011	Added the ABE based Playback and Recording feature
 * 0.4			Jul-2011	Fixed/Updated the Recording feature
 *					at Android level
 * 0.5			Aug-2011	Addressing the ABE LP changes and gain
 *					for playback path
 * 0.6			Aug-2011	Fixed the Recording CM Settings and also the
 *					MIC Coarse Gain, Fine Gain and PGA Settings for
 *					better Recording Quality at Android. Added support
 *					for High-Pass and Low-Pass Filters on the Recording
 *					Path.
 * 0.7			Sep-2011	Updated the Speaker Gain to 12db from 6db
 * 0.8			Sep-2011	Increased the MIC PGA Gain to 30db from 24db as per
 *					the recommendation from the customer
 * 0.9 			Sep-2011	Updated the DRC related Registers since the
 *					customer requested the feature. DRC is enabled on
 *					Speaker Path. As part of DRC feature, also updated
 *					the DAC PRB mode to 2.
 * 1.0			Sep-2011	As per the discussion with TI R&D Team, switching off
 *					Codec Power supplies during suspend() and resume()
 *					has been disabled due to limitation on current HW.
 * 1.1			Oct-2011	Added aic3100_hp_power_up() and aic3100_hp_power_down()
 *					functions to handle Headset Driver Power-up and
 *					Power-down sequences. Invoked the headset Power up
 *					function once during initialization.
 * 1.2			Oct-2011	Added function aic3100_get_record_status() to
 *					get Recording status.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>

#include "tlv320aic3100.h"
#include <mach/gpio.h>

#include <linux/clk.h>
#include <plat/clock.h>

#define NUM_INIT_REGS (sizeof(aic3100_reg_init) /       \
                       sizeof(struct aic3100_configs))

#define AIC_FORCE_SWITCHES_ON
#define CODEC_POWER_OFF

#ifdef CONFIG_ADAPTIVE_FILTER
extern void aic3100_add_biquads_controls (struct snd_soc_codec *codec);
extern int aic3100_add_EQ_mixer_controls (struct snd_soc_codec *codec);
extern int aic3100_parse_biquad_array (struct snd_soc_codec *codec);
extern int aic3100_update_biquad_array (struct snd_soc_codec *codec,
					int speaker_active, int playback_active);
#endif

static int snd_soc_info_volsw_2r_aic3100(struct snd_kcontrol *,
					 struct snd_ctl_elem_info *);
static int snd_soc_get_volsw_2r_aic3100(struct snd_kcontrol *,
					struct snd_ctl_elem_value *);
static int snd_soc_put_volsw_2r_aic3100(struct snd_kcontrol *,
					struct snd_ctl_elem_value *);
static int __new_control_info(struct snd_kcontrol *,
                              struct snd_ctl_elem_info *);
static int __new_control_get(struct snd_kcontrol *,
                             struct snd_ctl_elem_value *);
static int __new_control_put(struct snd_kcontrol *,
                             struct snd_ctl_elem_value *);
static int aic3100_dac_mute (struct snd_soc_codec *codec, int mute);

static int aic3100_hp_power_up  (struct snd_soc_codec *codec);
static int aic3100_hp_power_down(struct snd_soc_codec *codec);

#define SOC_SINGLE_AIC3100(xname)                                       \
        {                                                               \
                .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,     \
                        .info = __new_control_info, .get = __new_control_get, \
                        .put = __new_control_put,                       \
                        .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,      \
                        }

#define SOC_DOUBLE_R_AIC3100(xname, reg_left, reg_right, shift, mask, invert) \
        {	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),   \
                        .info = snd_soc_info_volsw_2r_aic3100,          \
                        .get = snd_soc_get_volsw_2r_aic3100,            \
			.put = snd_soc_put_volsw_2r_aic3100,		\
                        .private_value = (reg_left) | ((shift) << 8)  | \
                        ((mask) << 12) | ((invert) << 20) | ((reg_right) << 24) }




void __iomem *phymuxbase = NULL;
u32 phy_val;

static u8 aic3100_reg_ctl;

/*
 * whenever aplay/arecord is run, aic3100_hw_params() function gets called.
 * This function reprograms the clock dividers etc. this flag can be used to
 * disable this when the clock dividers are programmed by pps config file
 */
static int soc_static_freq_config = 1;

/* Codec Private Struct variable */
struct aic3100_priv *aic3100;

/*
 * Global Variables introduced to reduce Headphone Analog Volume Control
 * Registers at run-time
 */
struct i2c_msg i2c_right_transaction[120];
struct i2c_msg i2c_left_transaction[120];

/*
 * soc_enum array Structure Initialization
 */
static const char *dac_mute_control[] = {"UNMUTE" , "MUTE"};
static const char *hpdriver_voltage_control[] = {"1.35V", "1.5V", "1.65V",
						 "1.8V"};
static const char *drc_status_control[] = {"DISABLED", "ENABLED"};
static const char *adc_mute_control[] = {"Unmute", "Mute"};
static const char *micpga_selection[] = { "off", "10k", "20k", "40k" };
static const char *mic_pga_gain[] = {"Configurable Gain", "0 db Gain"};

static const struct soc_enum aic3100_dapm_enum[] = {
	SOC_ENUM_SINGLE (DAC_MUTE_CTRL_REG, 3, 2, dac_mute_control),
	SOC_ENUM_SINGLE (DAC_MUTE_CTRL_REG, 2, 2, dac_mute_control),
	SOC_ENUM_SINGLE (HEADPHONE_DRIVER, 3, 4, hpdriver_voltage_control),
        SOC_ENUM_DOUBLE (DRC_CTRL_1, 6, 5, 2,  drc_status_control),
	SOC_ENUM_SINGLE (ADC_FGA, 7, 2, adc_mute_control),
	SOC_ENUM_SINGLE (MIC_GAIN, 6, 4, micpga_selection),
	SOC_ENUM_SINGLE (MIC_GAIN, 4, 4, micpga_selection),
	SOC_ENUM_SINGLE (MIC_GAIN, 2, 4, micpga_selection),
	SOC_ENUM_SINGLE (ADC_IP_SEL, 6, 4, micpga_selection),
	SOC_ENUM_SINGLE (ADC_IP_SEL, 4, 4, micpga_selection),
	SOC_ENUM_SINGLE (MIC_PGA, 7, 2, mic_pga_gain),
};

/*
 * controls that need to be exported to the user space
 */
static const struct snd_kcontrol_new aic3100_snd_controls[] = {
	/* Output */
	/* sound new kcontrol for PCM Playback volume control */
	SOC_DOUBLE_R_AIC3100("DAC Playback Volume", LDAC_VOL, RDAC_VOL, 0, 0xAf,
			     0),
	/* sound new kcontrol for HP driver gain */
	SOC_DOUBLE_R("HP Driver Gain", HPL_DRIVER, HPR_DRIVER, 3, 0x09, 0),
	/* sound new kcontrol for LO driver gain */
	SOC_SINGLE("SPK Driver Gain", SPL_DRIVER, 3, 0x03, 0),
	/* sound new kcontrol for HP mute */
	SOC_DOUBLE_R("HP DAC Playback Switch", HPL_DRIVER, HPR_DRIVER, 2,
		     0x01, 1),
	/* sound new kcontrol for LO mute */
	SOC_SINGLE("LO DAC Playback Switch", SPL_DRIVER, 2,
		     0x01, 1),

	/*
	 * sound new kcontrol for Analog Volume Control for headphone
	 * and Speaker Outputs Please refer to Table 5-24 of the Codec
	 * DataSheet
	 */
	SOC_DOUBLE_R_AIC3100("HP Analog Volume",   L_ANLOG_VOL_2_HPL,
			     R_ANLOG_VOL_2_HPR, 0, 0x7F, 1),
	SOC_SINGLE("SPKR Analog Volume", L_ANLOG_VOL_2_SPL,
			      0, 0x7F, 0),

	/* sound new kcontrol for Programming the registers from user space */
	SOC_SINGLE_AIC3100("Program Registers"),

	/* Enumerations SOCs Controls */
	SOC_ENUM("LEFT  DAC MUTE", aic3100_dapm_enum[LEFT_DAC_MUTE_ENUM]),
	SOC_ENUM("RIGHT DAC MUTE", aic3100_dapm_enum[RIGHT_DAC_MUTE_ENUM]),
	SOC_ENUM("HP Driver Voltage level",
		 aic3100_dapm_enum[HP_DRIVER_VOLTAGE_ENUM]),
	SOC_ENUM("DRC Status", aic3100_dapm_enum[DRC_STATUS_ENUM]),

	/* Dynamic Range Compression Control */
	SOC_SINGLE("DRC Hysteresis Value (0=0db 3=db)", DRC_CTRL_1, 0, 0x03, 0),
	SOC_SINGLE("DRC Threshold Value (0=-3db,7=-24db)", DRC_CTRL_1, 2, 0x07, 0),
	SOC_SINGLE("DRC Hold Time",   DRC_CTRL_2, 3, 0x0F, 0),
	SOC_SINGLE("DRC Attack Time", DRC_CTRL_3, 4, 0x0F, 0),
	SOC_SINGLE("DRC Delay Rate",  DRC_CTRL_3, 0, 0x0F, 0),

	SOC_ENUM("ADC MUTE", aic3100_dapm_enum[ADC_MUTE_ENUM]),
	SOC_SINGLE("ADC Fine Gain Control", ADC_FGA, 4, 0x04, 1),
	SOC_SINGLE("ADC Coarse Gain Control", ADC_CGA, 0, 0xFF, 0),
	SOC_SINGLE("ADC Power Control", ADC_DIG_MIC, 7, 0x01, 0),
	SOC_SINGLE("MICPGA Gain", MIC_PGA, 0, 119, 0),
	SOC_ENUM("MIC1LP +ve terminal sel", aic3100_dapm_enum[MICPGA_LP_ENUM]),
	SOC_ENUM("MIC1RP +ve terminal sel", aic3100_dapm_enum[MICPGA_RP_ENUM]),
	SOC_ENUM("MIC1LM +ve terminal sel", aic3100_dapm_enum[MICPGA_LM_ENUM]),

	SOC_ENUM("CM -ve terminal sel", aic3100_dapm_enum[MICPGA_CM_ENUM]),
	SOC_ENUM("MIC1LM -ve terminal sel", aic3100_dapm_enum[MIC1LM_ENUM]),
};

/*
 * the structure contains the different values for mclk
 *
 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc,
 * madc, blck_N, codec_speficic_initializations
 */
static const struct aic3100_rate_divs aic3100_divs[] = {
	/* 8k rate */
	{19200000, 8000, 1, 5, 1200, 768, 16, 1, 128, 48, 2, 24},
	/* 11.025k rate */
	{19200000, 11025, 1, 4, 4100, 256, 15, 2, 128, 30, 2, 8},
	/* 12K rate */
	{19200000, 12000, 1, 4, 8000, 256, 15, 2, 128, 30, 2, 8},
	/* 16k rate */
	{19200000, 16000, 1, 5, 1200, 256, 12, 2, 128, 24, 2, 8},
	/* 22.05k rate */
	{19200000, 22050, 1, 4, 7040, 256, 8, 2, 128, 16, 2, 8},
	/* 24k rate */
	{19200000, 24000, 1, 5, 1200, 256, 8, 2, 128, 16, 2, 8},
	/* 32k rate */
	{19200000, 32000, 1, 5, 1200, 256, 6, 2, 128, 12, 2, 8},
	/* 44.1k rate */
	{19200000, 44100, 1, 4, 7040, 128, 4, 4, 128, 8, 2, 4},
	/* 48k rate */
	{19200000, 48000, 1, 5, 1200, 128, 4, 4, 128, 4, 4, 4},
	/*96k rate */
	{19200000, 96000, 1, 5, 1200, 256, 2, 2, 128, 4, 2, 8},
	/*192k */
	{19200000, 192000, 1, 5, 1200, 256, 2, 1, 128, 4, 1, 16},
};

/*
 * We are caching the registers here.  The following table contains the page
 * 0 and page 1 registers' reset values.
 */
static const u8 aic3100_reg[] = {
	0x00, 0x00, 0x02, 0x00, 0x00, 0x11, 0x04, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x80, 0x80,
	0x08, 0x00, 0x01, 0x01, 0x80, 0x80, 0x04, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 32 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x02, 0x00,
	0x02, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x14,
	0x0C, 0x00, 0x00, 0x00, 0x0F, 0x38, 0x00, 0x00, /* 64 */
	0x00, 0x00, 0x00, 0xEE, 0x10, 0xD8, 0x7E, 0x73,
	0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* Page 0 registers' reset values */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 128 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
	0x05, 0x3E, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F,
	0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#define	AIC3100_CACHEREGNUM (ARRAY_SIZE(aic3100_reg))

/*
 * aic3100 initialization data
 *
 * This structure contains the initialization required for aic3100.  These
 * registers values (reg_val) are written into the respective aic3100
 * register offset (reg_offset) to initialize aic3100.  These values are used
 * in aic3100_init() function only.
 */
static const struct aic3100_configs aic3100_reg_init[] = {
	/* Clock settings */
	{CLK_REG_1, CODEC_MUX_VALUE},

	/* Switch off PLL while Initiazling Codec */
	{INTERFACE_SET_REG_1, BCLK_DIR_CTRL},

	{INTERFACE_SET_REG_2, DAC_MOD_CLK_2_BDIV_CLKIN},

	{HP_POP_CTRL, (BIT7 | HP_POWER_UP_3_04_SEC | HP_DRIVER_3_9_MS | CM_VOLTAGE_FROM_AVDD)},
					/* CM_VOLTAGE_FROM_BAND_GAP)}, */
        {PGA_RAMP_CTRL, 0x70},        /* Speaker Ramp up time scaled to 30.5ms */

	{HEADPHONE_DRIVER,0x14},   /* Turn OFF the Headphone Driver by default */
	{CLASSD_SPEAKER_AMP, 0x06},/* Turn OFF the Speaker Driver by default */

	/* DAC Output Mixer Setting */
	{DAC_MIX_CTRL, RDAC_2_RAMP | LDAC_2_LAMP}, /*For aic31xx this is applicable...enabling DAC
						     routing through mixer amplifier individually for left & right
						     DAC..[1][35]... */

	{L_ANLOG_VOL_2_HPL, HP_DEFAULT_VOL},  /* Set volume of left Analog HPL to 0db attenuation [1][36] 0x9E*/
	{R_ANLOG_VOL_2_HPR, HP_DEFAULT_VOL},  /*Only applicable for 31xx...3120 does not have this register [1][37]*/
	{L_ANLOG_VOL_2_SPL, SPK_DEFAULT_VOL}, /*Only applicable for 31xx...3120 does not have this register [1][37] 0x80*/
	{R_ANLOG_VOL_2_SPR, SPK_DEFAULT_VOL}, /*Only applicable for 31xx...3120 does not have this register [1][37]*/

	 /* mute HP Driver */
	{HPL_DRIVER, 0x00}, /* muting HP_left(31xx)*/
	{HPR_DRIVER, 0x00}, /* muting HP_right(31xx)*/

	{HP_DRIVER_CTRL, 0x06},/* Changed by default to line-out Mode */
	{SPL_DRIVER, 0x08},
	/* Headset Detect setting */
	{INTL_CRTL_REG_1, 0xC0},
	{GPIO_CRTL_REG_1, 0x00},  /* Old value of 0x14 can be used if Codec GPIO is required for Headset Detect */
	{MICBIAS_CTRL, 0x0B},

	/* ADC Setting */
	{ADC_DIG_MIC, 0x00},
	{ADC_PRB_SEL_REG, 0x05},

	/* ADC Channel Fine Gain */
	{ADC_FGA, 0x80},
	/* ADC channel Coarse Gain */
	{ADC_CGA, 0x00},
	/* ON the board MIC1LM has been configured */
	{MIC_GAIN, 0xc0},
	{ADC_IP_SEL, 0x30},
	{CM_SET, 0x00},
	/* Initial Default configuration for ADC PGA */

	{MIC_PGA, 0x3C}, /* Reduced the MIC_PGA Settings as per TI ASIC team */

	{HP_SPK_ERR_CTL, 3}, /* error control register */

	/* DAC PRB configured to PRB_2 */
	{DAC_PRB_SEL_REG, 0x02},

	/*DRC settings */
	{DRC_CTRL_3, 0xB6},
	{DRC_CTRL_2, 0x00},
	{DRC_CTRL_1, 0x1A}, /* Increased Threshold to -21db */
};


#ifdef DAPM_SUPPORT

/*
 * DAPM Mixer Controls
 */
/* Left DAC_L Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", DAC_MIX_CTRL, 6, 2, 1),
	SOC_DAPM_SINGLE("MIC1_L switch", DAC_MIX_CTRL, 5, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {

	SOC_DAPM_SINGLE("R_DAC switch", DAC_MIX_CTRL, 2, 2, 1),
	SOC_DAPM_SINGLE("MIC1_R switch", DAC_MIX_CTRL, 1, 1, 0),
};

static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", DAC_MIX_CTRL, 6, 2, 1),
	SOC_DAPM_SINGLE("MIC1_L switch", DAC_MIX_CTRL, 5, 1, 0),
};

static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", DAC_MIX_CTRL, 2, 2, 1),
	SOC_DAPM_SINGLE("MIC1_R switch", DAC_MIX_CTRL, 1, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_L switch", MIC_GAIN, 6, 1, 0),
	SOC_DAPM_SINGLE("MIC1_M switch", MIC_GAIN, 2, 1, 0),
};

static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_R switch", MIC_GAIN, 4, 1, 0),
	SOC_DAPM_SINGLE("MIC1_M switch", MIC_GAIN, 2, 1, 0),
};


/*
 * DAPM Widget Controls
 */
static const struct snd_soc_dapm_widget aic3100_dapm_widgets[] = {
	/* Left DAC to Left Outputs */
	/* dapm widget (stream domain) for left DAC */
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", DAC_CHN_REG, 7, 1),

	/* dapm widget (path domain) for left DAC_L Mixer */

	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpl_output_mixer_controls[0],
			   ARRAY_SIZE(hpl_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPL Power", HEADPHONE_DRIVER, 7, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lol_output_mixer_controls[0],
			   ARRAY_SIZE(lol_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOL Power", CLASSD_SPEAKER_AMP, 7, 1, NULL, 0),

	/* Right DAC to Right Outputs */

	/* dapm widget (stream domain) for right DAC */
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", DAC_CHN_REG, 6, 1),
	/* dapm widget (path domain) for right DAC_R mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpr_output_mixer_controls[0],
			   ARRAY_SIZE(hpr_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPR Power", HEADPHONE_DRIVER, 6, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lor_output_mixer_controls[0],
			   ARRAY_SIZE(lor_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOR Power", CLASSD_SPEAKER_AMP, 6, 1, NULL, 0),

	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			   &left_input_mixer_controls[0],
			   ARRAY_SIZE(left_input_mixer_controls)),

	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			   &right_input_mixer_controls[0],
			   ARRAY_SIZE(right_input_mixer_controls)),

	/*
	 * No widgets are required for ADC since the AIC3100 Audio
	 * Codec Chipset does not contain a ADC
	 */

	/* dapm widget (platform domain) name for HPLOUT */
	SND_SOC_DAPM_OUTPUT("HPL"),
	/* dapm widget (platform domain) name for HPROUT */
	SND_SOC_DAPM_OUTPUT("HPR"),
	/* dapm widget (platform domain) name for LOLOUT */
	SND_SOC_DAPM_OUTPUT("LOL"),
	/* dapm widget (platform domain) name for LOROUT */
	SND_SOC_DAPM_OUTPUT("LOR"),

	/* dapm widget (platform domain) name for MIC1LP */
	SND_SOC_DAPM_INPUT("MIC1LP"),
	/* dapm widget (platform domain) name for MIC1RP*/
	SND_SOC_DAPM_INPUT("MIC1RP"),
	/* dapm widget (platform domain) name for MIC1LM */
	SND_SOC_DAPM_INPUT("MIC1LM"),
};


/*
 * DAPM audio route definition - Defines an audio route originating at source
 * via control and finishing at sink.
 */
static const struct snd_soc_dapm_route aic3100_dapm_routes[] = {
	/* ******** Right Output ******** */
	{"HPR Output Mixer", "R_DAC switch", "Right DAC"},
	{"HPR Output Mixer",  "MIC1_R switch", "MIC1RP"},

	{"HPR Power", NULL, "HPR Output Mixer"},
	{"HPR", NULL, "HPR Power"},


	{"LOR Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOR Output Mixer",  "MIC1_R switch", "MIC1RP"},

	{"LOR Power", NULL, "LOR Output Mixer"},
	{"LOR", NULL, "LOR Power"},

	/* ******** Left Output ******** */
	{"HPL Output Mixer", "L_DAC switch", "Left DAC"},
	{"HPL Output Mixer", "MIC1_L switch", "MIC1LP"},

	{"HPL Power", NULL, "HPL Output Mixer"},
	{"HPL", NULL, "HPL Power"},


	{"LOL Output Mixer", "L_DAC switch", "Left DAC"},
	{"LOL Output Mixer", "MIC1_L switch", "MIC1LP"},

	{"LOL Power", NULL, "LOL Output Mixer"},
	{"LOL", NULL, "LOL Power"},
};

#define AIC3100_DAPM_ROUTE_NUM (sizeof(aic3100_dapm_routes)/		\
				sizeof(struct snd_soc_dapm_route))
#endif	/*DAPM SUPPORT*/


#ifdef AIC3100_GPIO_INTR_SUPPORT
/* GPIO Interrupt Worker Thread related Global Vars */
static struct work_struct codec_int_work;
#endif

/*
 * aic3100_change_page - switch between page 0 and page 1.
 */
int aic3100_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	u8 data[2];

	data[0] = 0;
	data[1] = new_page;
	aic3100->page_no = new_page;
	DBG("##aic3100_change_page => %d\nw 30 %02x %02x\n", new_page, data[0],
	    data[1]);

	DBG("w 30 %02x %02x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ERR "Error in changing page to %d\n", new_page);
		return -1;
	}
	return 0;
}

#if defined (EN_REG_CACHE)
/*
 * aic3100_write_reg_cache - write aic3100 register cache
 */
static inline void aic3100_write_reg_cache (struct snd_soc_codec *codec,
                                            u16 reg, u8 value)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	u8 *cache = codec->reg_cache;

	if (reg >= AIC3100_CACHEREGNUM) {
		return;
	}
	cache[reg] = value;
}

/*
 * aic3100_read_reg_cache -read the aic3100 registers through the Register
 * Cache Array instead of I2C Transfers
 */
static unsigned char
aic3100_read_reg_cache(struct snd_soc_codec *codec, unsigned int reg)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	u8 *cache = codec->reg_cache;

	/* Confirm the Register Offset is within the Array bounds */
	if (reg >= AIC3100_CACHEREGNUM) {
		return 0;
	}

	return (cache[reg]);
}
#endif

/*
 * aic3100_write - write to the aic3100 register space.
 */
int aic3100_write (struct snd_soc_codec *codec, unsigned int reg,
                   unsigned int value)
{
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	u8 data[2];
	u8 page;

	DBG(KERN_INFO "%s: started\n", __func__);
	page = reg / 128;
	data[AIC3100_REG_OFFSET_INDEX] = reg % 128;

	if (aic3100->page_no != page) {
		aic3100_change_page(codec, page);
	}

	/* data is
	 *   D15..D8 aic3100 register offset
	 *   D7...D0 register data
	 */
	data[AIC3100_REG_DATA_INDEX] = value & AIC3100_8BITS_MASK;

#if defined(EN_REG_CACHE)
	if ((page == 0) || (page == 1)) {
		aic3100_write_reg_cache(codec, reg, value);
	}
#endif

	if (!data[AIC3100_REG_OFFSET_INDEX]) {
		/* if the write is to reg0 update aic3100->page_no */
		aic3100->page_no = value;
	}

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ERR "Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 * aic3100_read - read the aic3100 register space.
 */
unsigned int aic3100_read (struct snd_soc_codec *codec, unsigned int reg)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	u8 value;
	u8 page = reg / 128;

        /* Can be used to optimize the Reads from page 0 and 1 */
#if defined (EN_REG_CACHE)
        u8 cache_value;
        if ((page == 0) || (page == 1)) {
                cache_value = aic3100_read_reg_cache (codec, reg);
                DBG("Reg%x-Cache %02x\n", reg, value);
        }
#endif
	reg = reg % 128;

	if (aic3100->page_no != page) {
		aic3100_change_page(codec, page);
	}

	codec->hw_write(codec->control_data, (char *)&reg, 1);
        i2c_master_recv(codec->control_data, &value, 1);

	return value;
}
/*
 *----------------------------------------------------------------------------
 * Function : debug_print_registers
 * Purpose  : Debug routine to dump all the Registers of Page 0
 *
 *----------------------------------------------------------------------------
 */
void debug_print_registers (struct snd_soc_codec *codec)
{
	int i;
	u8 data;

	printk( "### Page 0 Regs from 0 to 95\n");

	for (i = 0; i < 95; i++) {
		data = (u8) aic3100_read(codec, i);
		printk(KERN_INFO "reg = %d val = %x\n", i, data);
	}
	printk("-------------------------------------------\n");
        printk("### Page 1 Regs from 30 to 52\n");

	for (i = 158; i < 180; i++) {
		data = (u8) aic3100_read(codec, i);
		printk(KERN_INFO "reg = %d val = %x\n", (i%128), data);
	}


	DBG ("####SPL_DRIVER_GAIN %d  \n\n",
			aic3100_read (codec, SPL_DRIVER));

	DBG ("##### L_ANALOG_VOL_2_SPL %d R_ANLOG_VOL_2_SPR %d \n\n",
			aic3100_read (codec, L_ANLOG_VOL_2_SPL),
			aic3100_read (codec, R_ANLOG_VOL_2_SPR));

	DBG ("#### LDAC_VOL %d RDAC_VOL %d \n\n",
			aic3100_read (codec, LDAC_VOL),
			aic3100_read (codec, RDAC_VOL));
	DBG ("###OVER Temperature STATUS ( Page 0 Reg 3) %x\n\n",
			aic3100_read (codec, OT_FLAG));
	DBG ("###SHORT CIRCUIT STATUS (Page 0 Reg 44) %x\n\n",
			aic3100_read (codec, INTR_FLAG_1));

	DBG ("###INTR_FLAG: SHORT_CKT(Page 0 Reg 46) %x\n\n",
			aic3100_read (codec, INTR_FLAG_2));
	DBG ("###Speaker_Driver_Short_Circuit ( Page 1 Reg 32)%x\n\n",
			aic3100_read (codec, CLASSD_SPEAKER_AMP));

	DBG ("@@@  MIC_PGA (P1 R47) = 0x%x \n\n",
			aic3100_read (codec, MIC_PGA));

	DBG ("@@@  ADC_FGA (P0 R82) = 0x%x \n\n",
			aic3100_read (codec, ADC_FGA));

	DBG ("@@@  ADC_CGA (P0 R83) = 0x%x \n\n",
			aic3100_read (codec, ADC_CGA));


}

/*
 * __new_control_get - read data of new control for program the aic3100
 * registers.
 */
static int __new_control_get (struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;

	val = aic3100_read(codec, aic3100_reg_ctl);
	ucontrol->value.integer.value[0] = val;

	return 0;
}

/*
 * __new_control_put - this is called to pass data from user/application to
 * the driver.
 */
static int __new_control_put (struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);

	u32 data_from_user = ucontrol->value.integer.value[0];
	u8 data[2];

	aic3100_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0]) {
		aic3100->page_no = data[1];
	}

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ALERT "Error in i2c write\n");
		return -EIO;
	}

	return 0;
}

/*
 * snd_soc_info_volsw_2r_aic3100 - Info routine for the ASoC Widget related
 * to the Volume Control
 */
static int snd_soc_info_volsw_2r_aic3100 (struct snd_kcontrol *kcontrol,
                                          struct snd_ctl_elem_info *uinfo)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	int mask = (kcontrol->private_value >> 12) & 0xff;

	uinfo->type = mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN :
		SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mask;
	return 0;
}

/*
 * snd_soc_get_volsw_2r_aic3100 - Callback to get the value of a double mixer
 * control that spans two registers.
 */
static int snd_soc_get_volsw_2r_aic3100 (struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & AIC3100_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & AIC3100_8BITS_MASK;
	int mask;
	int shift;
	unsigned short val, val2;

	/* Check the id name of the kcontrol and configure the mask and
	 * shift */
	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		mask = AIC3100_8BITS_MASK;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		mask = 0xF;
		shift = 3;
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		mask = 0x3;
		shift = 3;
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		mask = 0x7F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "HP Analog Volume")) {
		mask = 0x7F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "SPKR Analog Volume")) {
		mask = 0x7F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "ADC Coarse Gain Control")) {
		mask = 0x7F;
		shift = 0;
	} else {
		printk(KERN_ALERT "Invalid kcontrol name\n");
		return -1;
	}

	val = (snd_soc_read(codec, reg) >> shift) & mask;
	val2 = (snd_soc_read(codec, reg2) >> shift) & mask;

	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		ucontrol->value.integer.value[0] =
			(val <= 48) ? (val + 127) : (val - 129);
		ucontrol->value.integer.value[1] =
			(val2 <= 48) ? (val2 + 127) : (val2 - 129);
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		ucontrol->value.integer.value[0] =
			(val <= 9) ? (val + 0) : (val - 15);
		ucontrol->value.integer.value[1] =
			(val2 <= 9) ? (val2 + 0) : (val2 - 15);
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		ucontrol->value.integer.value[0] =
			((val/6) <= 4) ? ((val/6 -1)*6) : ((val/6 - 0)*6);
		ucontrol->value.integer.value[1] =
			((val2/6) <= 4) ? ((val2/6-1)*6) : ((val2/6 - 0)*6);
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		ucontrol->value.integer.value[0] =
			((val*2) <= 40) ? ((val*2 + 24)/2) : ((val*2 - 254)/2);
		ucontrol->value.integer.value[1] =
			((val2*2) <= 40) ? ((val2*2 + 24)/2) :
			((val2*2 - 254)/2);
	} else if (!strcmp(kcontrol->id.name, "HP Analog Volume")) {
		ucontrol->value.integer.value[0] =
			((val*2) <= 40) ? ((val*2 +24)/2) :((val2*2 - 254)/2);
		ucontrol->value.integer.value[1] =
			((val*2) <=40) ? ((val*2 + 24)/2) : ((val2*2 - 254)/2);
	} else if (!strcmp(kcontrol->id.name, "SPKR Analog Volume")) {
		ucontrol->value.integer.value[0] =
			((val*2) <= 40) ? ((val*2 +24)/2) :((val2*2 - 254)/2);
		ucontrol->value.integer.value[1] =
			((val*2) <=40) ? ((val*2 + 24)/2) : ((val2*2 - 254)/2);
	} else if (!strcmp(kcontrol->id.name, "ADC Coarse Gain Control")) {
		ucontrol->value.integer.value[0] = val - 40;
	}

	return 0;
}

/*
 * snd_soc_put_volsw_2r_aic3100 - Callback to set the value of a double mixer
 * control that spans two registers.
 */
static int snd_soc_put_volsw_2r_aic3100 (struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & AIC3100_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & AIC3100_8BITS_MASK;
	int err;
	unsigned short val, val2, val_mask;

	val = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	/* TODO: This block needs to be revisted */
	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		val = (val >= 127) ? (val - 127) : (val + 129);
		val2 = (val2 >= 127) ? (val2 - 127) : (val2 + 129);
		val_mask = AIC3100_8BITS_MASK;	/* 8 bits */
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		val = (val >= 0) ? (val - 0) : (val + 15);
		val2 = (val2 >= 0) ? (val2 - 0) : (val2 + 15);
		val_mask = 0xF;	/* 4 bits */
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		val = (val/6 >= 1) ? ((val/6 +1)*6) : ((val/6 + 0)*6);
		val2 = (val2/6 >= 1) ? ((val2/6 +1)*6) : ((val2/6 + 0)*6);
		val_mask = 0x3;	/* 2 bits */
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		val = (val*2 >= 24) ? ((val*2 - 24)/2) : ((val*2 + 254)/2);
		val2 = (val2*2 >= 24) ? ((val2*2 - 24)/2) : ((val2*2 + 254)/2);
		val_mask = 0x7F;	/* 7 bits */
	} else if (!strcmp(kcontrol->id.name, "HP Analog Volume")) {
                val_mask = 0x7F; /* 7 Bits */
	} else if (!strcmp(kcontrol->id.name, "SPKR Analog Volume")) {
                val_mask = 0x7F; /* 7 Bits */
	} else if (!strcmp(kcontrol->id.name, "ADC Coarse Gain Control")) {
		val_mask = 0x7F;
		val = (val > (104 - 40))? val = 104 : (val + 40);
	} else {
		printk(KERN_ALERT "Invalid control name\n");
		return -1;
	}

	if ((err = snd_soc_update_bits(codec, reg, val_mask, val)) < 0) {
		printk(KERN_ALERT "Error while updating bits\n");
		return err;
	}

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}

/*
 * __new_control_info - This function is to initialize data for new control
 * required to program the aic3100 registers.
 */
static int __new_control_info (struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_info *uinfo)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;

	return 0;
}

/*
 * aic3100_get_divs - get required divisor from the "aic3100_divs" table.
 */
static inline int aic3100_get_divs (int mclk, int rate)
{
	int i;
	DBG(KERN_INFO "%s: started\n", __func__);
	DBG("###+ aic3100_get_divs mclk(%d) rate(%d)\n", mclk, rate);

	for (i = 0; i < ARRAY_SIZE(aic3100_divs); i++) {
		if ((aic3100_divs[i].rate == rate)
		    && (aic3100_divs[i].mclk == mclk)) {
			return i;
		}
	}
	printk(KERN_ALERT "Master clock and sample rate is not supported\n");
	return -EINVAL;
}

#ifdef DAPM_SUPPORT
/*
 * aic3100_add_widgets
 *
 * The following are the main widgets supported: Left DAC to Left Outputs,
 * Right DAC to Right Outputs, Left Inputs to Left ADC, Right Inputs to Right
 * ADC
 */
static int aic3100_add_widgets(struct snd_soc_codec *codec)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	DBG("+ aic3100_add_widgets num_widgets(%d) num_routes(%d)\n",
            ARRAY_SIZE(aic3100_dapm_widgets), AIC3100_DAPM_ROUTE_NUM);
	snd_soc_dapm_new_controls(codec->dapm, aic3100_dapm_widgets,
				  ARRAY_SIZE(aic3100_dapm_widgets));

	DBG("snd_soc_dapm_add_routes\n");

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec->dapm, aic3100_dapm_routes,
				AIC3100_DAPM_ROUTE_NUM);

	return 0;
}
#endif

/* 
 * aic3100_hp_power_up
 *
 * Function to enable the Headset Driver along with proper polling 
 * sequence.
 */
static int aic3100_hp_power_up (struct snd_soc_codec *codec)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	u8 value;
	volatile u16 counter;
	/* Put the HP Analog Volumes to lower levels first
	 * the Analog volumes is based on soft-stepping and hence
	 * a delay between them.
	 */
	aic3100_write (codec, L_ANLOG_VOL_2_HPL, 0x75);
	mdelay(5);
	aic3100_write (codec, R_ANLOG_VOL_2_HPR, 0x75);
	mdelay(5);


	DBG ("%s: Entered \n", __func__);
	/* Switch ON Left and Right Headphone Drivers. Switching them here before
	 * Headphone Volume ramp helps in reducing the pop sounds.
	 */
	if(!aic3100->i2c_regs_status) {
		DBG ("Entered the pop control section \n",__func__);
		aic3100_write(codec, HP_POP_CTRL, (BIT7 | HP_POWER_UP_3_04_SEC \
			| HP_DRIVER_3_9_MS | CM_VOLTAGE_FROM_BAND_GAP));
		aic3100->i2c_regs_status = 1;
	}
	else {
		aic3100_write(codec, HP_POP_CTRL, (BIT7 | HP_POWER_UP_76_2_MSEC \
				| HP_DRIVER_3_9_MS | CM_VOLTAGE_FROM_BAND_GAP));
	}

	/* MUTE the Headphone Left and Right */
        value = aic3100_read (codec, HPL_DRIVER);
        aic3100_write (codec, HPL_DRIVER, (value & ~0x06));

        value = aic3100_read (codec, HPR_DRIVER);
        aic3100_write (codec, HPR_DRIVER, (value & ~0x06));

	/* Switch ON Left and Right Headphone Drivers. Switching them here before
	 * Headphone Volume ramp helps in reducing the pop sounds.
	 */
	value = aic3100_read (codec, HEADPHONE_DRIVER);
	aic3100_write (codec, HEADPHONE_DRIVER, (value | 0xC0));

	/* Check for the DAC FLAG Register to know if the Left
	   Output Driver is powered up */
	counter = 0;
	do {
		mdelay(1);
		value = aic3100_read (codec, DAC_FLAG_1);
		counter++;
	} while ((counter < 300) && (value & 0x22) == 0);

	DBG ("Headset Power-ON %d\n", counter);

	/* Switch OFF the Class_D Speaker Amplifier */
	value = aic3100_read (codec, CLASSD_SPEAKER_AMP);
	aic3100_write (codec, CLASSD_SPEAKER_AMP, (value & ~0xC0));

	/* Update the Status Variable in the Priv Struct */
	aic3100->hp_driver_powered = 1;

	DBG ("--%s\n",__func__);
	return 0;
}

/*
 * aic3100_hp_power_down
 *
 * Function to power down the Headset Driver with proper polling
 * sequence. 
 */
static int aic3100_hp_power_down(struct snd_soc_codec *codec)
{

	DBG(KERN_INFO "%s: started\n", __func__);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	u8 value;
	volatile u16 counter;

	DBG ("%s: Entered \n", __func__);

	/* Switch off the Head phone Drivers */
	value = aic3100_read (codec, HEADPHONE_DRIVER);
	aic3100_write (codec, HEADPHONE_DRIVER, (value & ~0xC0)); 

	/* Now first check if the HPR is fully powered down */
	counter = 0;
	do {
		mdelay(1);
		value = aic3100_read (codec, DAC_FLAG_1);
		counter++;
	}while ((counter < 200) && ((value & 0x22) != 0));

	DBG ("Headset PowerDown %d\n", counter);

	value = aic3100_read (codec, CLASSD_SPEAKER_AMP);
	aic3100_write (codec, CLASSD_SPEAKER_AMP, (value & ~0xC0));

	/* Update the Priv Struct Member again */
	aic3100->hp_driver_powered = 0;

	return 0;
}


/*
 * aic3100_power_up
 *
 * Powers up the DAC and the Headphone or the Speaker Driver
 * based on the status of the codec Private structure.
 */
static int aic3100_power_up (struct snd_soc_codec *codec)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	u8 value;
	volatile u16 counter;
	int gpio_status;

	DBG ("%s: Entered \n", __func__);

	/* Check the status of the Headset GPIO once */
	gpio_status = gpio_get_value (HEADSET_DETECT_GPIO_PIN);

	/* Updating the private structure with the newly read GPIO value */
	aic3100->headset_connected = !gpio_status;

	if (aic3100->master) {	/* Switch on PLL */
		value = aic3100_read(codec, CLK_REG_2);
		aic3100_write(codec, CLK_REG_2, (value | ENABLE_PLL));
		/* Adding 10ms PLL startup delay as per datasheet
		 * section 5.2.1.3*/
		mdelay(10);

		/* Switch on NDAC Divider */
		value = aic3100_read(codec, NDAC_CLK_REG);
		aic3100_write(codec, NDAC_CLK_REG, value | ENABLE_NDAC);

		/* Switch on MDAC Divider */
		value = aic3100_read(codec, MDAC_CLK_REG);
		aic3100_write(codec, MDAC_CLK_REG, value | ENABLE_MDAC);

		/* Switch on NADC Divider */
		value =	aic3100_read(codec, NADC_CLK_REG);
		aic3100_write(codec, NADC_CLK_REG, value | ENABLE_MDAC);

		/* Switch on MADC Divider */
		value =	aic3100_read(codec, MADC_CLK_REG);
		aic3100_write(codec, MADC_CLK_REG, value | ENABLE_MDAC);

		/* Switch on BCLK_N Divider */
		value =	aic3100_read(codec, BCLK_N_VAL);
		aic3100_write(codec, BCLK_N_VAL, value | ENABLE_BCLK);
	} else {	/* Switch on PLL */
		value = aic3100_read(codec, CLK_REG_2);
		aic3100_write(codec, CLK_REG_2, (value | ENABLE_PLL));
		/* Adding 10ms PLL startup delay as per datasheet
		 * section 5.2.1.3*/
		mdelay(10);

		/* Switch on NDAC Divider */
		value = aic3100_read(codec, NDAC_CLK_REG);
		aic3100_write(codec, NDAC_CLK_REG, value | ENABLE_NDAC);

		/* Switch on MDAC Divider */
		value =	aic3100_read(codec, MDAC_CLK_REG);
		aic3100_write(codec, MDAC_CLK_REG, value | ENABLE_MDAC);

		/* Switch on NADC Divider */
		value = aic3100_read(codec, NADC_CLK_REG);
		aic3100_write(codec, NADC_CLK_REG, value | ENABLE_MDAC);

		/* Switch on MADC Divider */
		value =	aic3100_read(codec, MADC_CLK_REG);
		aic3100_write(codec, MADC_CLK_REG, value | ENABLE_MDAC);

		/* Switch on BCLK_N Divider */
		value =	aic3100_read(codec, BCLK_N_VAL);
		aic3100_write(codec, BCLK_N_VAL, value | ENABLE_BCLK);
	}

	/* Codec Power is directly handled by Driver code-base */
	if (aic3100->master && (aic3100->power_status != 1) ) {
		/* Check if we need to enable the DAC or the ADC section based on the
                 * actual request. The Playback_stream member of the aic31xx_priv
                 * struct holds the reqd information.
                 */
		if (aic3100->playback_stream) {

			/* Switch ON the Class_D Speaker Amplifier */
			/*
                        value = aic3100_read (codec, CLASSD_SPEAKER_AMP);
                        aic3100_write (codec, CLASSD_SPEAKER_AMP, (value | 0xC0));
			*/
			mdelay(1);
			/* Put the DAC volume and Headphone volume to lowest levels first */
			aic3100_write (codec, LDAC_VOL, DAC_DEFAULT_VOL);
			aic3100_write (codec, RDAC_VOL, DAC_DEFAULT_VOL);

			aic3100_write (codec, L_ANLOG_VOL_2_HPL, HP_DEFAULT_VOL);
			aic3100_write (codec, R_ANLOG_VOL_2_HPR, HP_DEFAULT_VOL);

			/* Switch ON Left and Right DACs */
			value = aic3100_read (codec, DAC_CHN_REG);
			aic3100_write (codec, DAC_CHN_REG, (value | ENABLE_DAC_CHN));

			/* Check for the DAC FLAG register to know if the DAC is
			   really powered up */
			counter = 0;
			do {
				mdelay(1);
				value = aic3100_read (codec, DAC_FLAG_1);
				counter++;
			} while ((counter < 20) && ((value & 0x88) == 0));

			/* Check whether the Headset or Speaker Driver needs Power Up */
			if (aic3100->headset_connected) {
				if (!aic3100->hp_driver_powered) {
					DBG ("Headset Not Powered.. Powering it up..\n");
					aic3100_hp_power_up (codec);
				} else
					DBG ("Headset Already powered..\n");
                         } else {
				/* Forcing the ANALOG volume to speaker driver to 0 dB */
				aic3100_write (codec, L_ANLOG_VOL_2_SPL, 0x80);

				aic3100_write (codec, R_ANLOG_VOL_2_SPR, 0x80);

				/* Switch ON the Class_D Speaker Amplifier */
				value = aic3100_read (codec, CLASSD_SPEAKER_AMP);
				aic3100_write (codec, CLASSD_SPEAKER_AMP, (value | 0xC0));
				mdelay(1);
			}
                } /*SND_RV_PCM*/

		else {
			/* ADC Channel Power Up */
			value = aic3100_read (codec, ADC_DIG_MIC);
			aic3100_write (codec, ADC_DIG_MIC, (value | 0x80));

			/* Check for the ADC FLAG register to know if the ADC is
			   really powered up */
			counter = 0;
			do {
				mdelay(1);
				value = aic3100_read (codec, ADC_FLAG);
				counter++;
			} while ((counter < 40) && ((value & 0x40) == 0));

			DBG("##-- aic31xx_power_up\n");
			/* Power Up control of MICBIAS */

			value = aic3100_read (codec, MICBIAS_CTRL);
			aic3100_write (codec, MICBIAS_CTRL, (value | (BIT1 | BIT0)));
		}

		aic3100->power_status = 1;

	}
	DBG ("%s: Exiting \n", __func__);
	return 0;
}

/*
 * aic3100_adc_mute
 *
 * Mutes or Unmutes the ADC part of the Codec depending on the
 * value of the Mute flag.
 */
static int aic3100_adc_mute (struct snd_soc_codec *codec, int mute)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	u8 value;
	int retval = 0;
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);

	value = aic3100_read(codec, ADC_FGA);

	if ((mute) && (aic3100->mute != 1)) {
		retval = aic3100_write(codec, ADC_FGA, (value | BIT7));

		aic3100->mute = 1;
	}
	else if (!mute && (aic3100->mute != 0)) {
		retval = aic3100_write(codec, ADC_FGA, (value & ~BIT7));

		aic3100->mute = 0;
	}

#ifdef AIC3100_DEBUG
	debug_print_registers(codec);
#endif
	return (retval);
}

/*
 * aic3100_power_down
 *
 * Powers down the DAC and the Headphone or the Speaker Driver
 * based on the status of the codec Private structure.
 */
static int aic3100_power_down (struct snd_soc_codec *codec)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	volatile u8 value;
        volatile u32 counter;

	DBG("##++ aic31xx_power_down aic31xx->master=%d\n", aic3100->master);

	/* If PowerStatus member is set, we will turn off HS, Spk Drivers, ADC and DAC */
	if (aic3100->power_status != 0) {

		/* Check whether the Headset or Speaker Driver needs Power Down */
		if(aic3100->headset_connected) {
			aic3100_hp_power_down (codec);
		} else {
			/* Switch OFF the Class_D Speaker Amplifier */
			value = aic3100_read (codec, CLASSD_SPEAKER_AMP);
			aic3100_write (codec, CLASSD_SPEAKER_AMP, (value & ~0xC0));

                        /* Left and Right Speaker Analog Volume is muted */
                        aic3100_write (codec, L_ANLOG_VOL_2_SPL, 0x7F);

                        aic3100_write (codec, R_ANLOG_VOL_2_SPR, 0x7F);

	        }
		aic3100->netflix_mode = 0;
                /* Switch OFF Left and Right DACs */

		value = aic3100_read (codec, DAC_CHN_REG);
		aic3100_write (codec, DAC_CHN_REG, (value & ~ENABLE_DAC_CHN));

                /* Check for the DAC FLAG register to know if the DAC is really
                   powered down */
                counter = 0;
                do {
			udelay(100);
                        value = aic3100_read (codec, DAC_FLAG_1);
                        counter++;
                } while ((counter < 100) && ((value & 0x88) != 0));

		/* ADC Channel Power down */
		value = aic3100_read (codec, ADC_DIG_MIC);
		aic3100_write (codec, ADC_DIG_MIC, (value & ~BIT7));

                /* Check for the ADC FLAG register to know if the ADC is
                really powered down */
                counter = 0;
                do {
			udelay(100);
			value = aic3100_read(codec, ADC_FLAG);
			counter++;
                } while ((counter < 50) && ((value & 0x40) != 0));

               /* Power down control of MICBIAS */

		value = aic3100_read (codec, MICBIAS_CTRL);
		aic3100_write (codec, MICBIAS_CTRL, (value & ~(BIT1 | BIT0)));

	        aic3100->power_status = 0;

	}

	/* If the Codec is Master, then we will turn off the PLL and BCLK also */
	if (aic3100->master) {
		/* Switch off PLL */
		value = aic3100_read(codec, CLK_REG_2);
		aic3100_write(codec, CLK_REG_2, (value & ~ENABLE_PLL));

		/* Switch off NDAC Divider */
		value = aic3100_read(codec, NDAC_CLK_REG);
		aic3100_write(codec, NDAC_CLK_REG, value &
			      ~ENABLE_NDAC);

		/* Switch off MDAC Divider */
		value = aic3100_read(codec, MDAC_CLK_REG);
		aic3100_write(codec, MDAC_CLK_REG, value &
			      ~ENABLE_MDAC);

		/* Switch off NADC Divider */
		value = aic3100_read(codec, NADC_CLK_REG);
		aic3100_write(codec, NADC_CLK_REG, value &
			      ~ENABLE_NDAC);

		/* Switch off MADC Divider */
		value = aic3100_read(codec, MADC_CLK_REG);
		aic3100_write(codec, MADC_CLK_REG, value &
			      ~ENABLE_MDAC);
		value = aic3100_read(codec, BCLK_N_VAL);

		/* Switch off BCLK_N Divider */
		aic3100_write(codec, BCLK_N_VAL, value & ~ENABLE_BCLK);
	}

	DBG ("##%s: Exiting \n", __func__);
	return 0;


}

/*
 * aic31xx_startup
 *
 * This function check for the PCM Substream
 */
int aic3100_startup (struct snd_pcm_substream *substream, struct snd_soc_codec *codec)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);

	/* Check if the previous Playback/session was incomplete */

	if ((aic3100->mute == 0) && (aic3100->power_status == 1) &&
	    (aic3100->playback_stream == 1) &&
	    (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)) {
		printk (KERN_INFO "%s: Entered session check \n", __func__);
		return -EBUSY;
	}

	return 0;
}

/*
 * aic3100_hw_params
 *
 * This function is to set the hardware parameters for aic3100.  The
 * functions set the sample rate and audio serial data word length.
 */
static int aic3100_hw_params (struct snd_pcm_substream *substream,
                              struct snd_pcm_hw_params *params,
                              struct snd_soc_dai *tmp)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	int i, ret;
	u8 data;

	DBG("##+ SET aic3100_hw_params\n");

	mutex_lock(&aic3100->mutex);

	/* Invoke the Codec Startup FUnction and validate the use-case */
	if ((ret = aic3100_startup (substream, codec)) != 0) {
		printk (KERN_INFO "Another Audio Session in Progress. "
			"Cannot start second one..\n");

		mutex_unlock(&aic3100->mutex);
		return (ret);
	}

	/* Check if the previous Playback/session was incomplete */
	if ((aic3100->mute == 0) && (aic3100->power_status == 1)) {
		printk(KERN_INFO"Entered session check\n");
		if (aic3100->capture_stream) {
			ret = aic3100_adc_mute(codec, 1);
		} else {
			ret = aic3100_dac_mute(codec, 1);
		}
	}
	aic3100_power_down(codec);
	codec->dapm.bias_level = 2;

	i = aic3100_get_divs(aic3100->sysclk, params_rate(params));

	/* Setting the playback status */

	/* Update the capture_stream Member of the Codec's Private structure
	 * to denote that we will be performing Audio capture from now on.
	 */
	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aic3100->capture_stream = 0;
		aic3100->playback_stream=1;
		aic3100->playback_status = 1;
        } else {
		aic3100->capture_stream = 1;
                aic3100->playback_stream=0;
		aic3100->playback_status = 0;
	}

	if (i < 0) {
		printk(KERN_ALERT "sampling rate not supported\n");
		mutex_unlock(&aic3100->mutex);
		return i;
	}

	if (soc_static_freq_config) {
		/*
		 * We will fix R value to 1 and will make P & J=K.D as
		 * varialble
		 */

		/* Setting P & R values */
		aic3100_write(codec, CLK_REG_2,
			      ((aic3100_divs[i].p_val << 4) | 0x01));

		/* J value */
		aic3100_write(codec, CLK_REG_3, aic3100_divs[i].pll_j);

		/* MSB & LSB for D value */
		aic3100_write(codec, CLK_REG_4, (aic3100_divs[i].pll_d >> 8));
		aic3100_write(codec, CLK_REG_5,
			      (aic3100_divs[i].pll_d & AIC3100_8BITS_MASK));

		/* NDAC divider value */
		aic3100_write(codec, NDAC_CLK_REG, aic3100_divs[i].ndac);

		/* MDAC divider value */
		aic3100_write(codec, MDAC_CLK_REG, aic3100_divs[i].mdac);

		/* NADC and MADC divider values */
		aic3100_write(codec, NADC_CLK_REG, aic3100_divs[i].nadc);
		aic3100_write(codec, MADC_CLK_REG, aic3100_divs[i].madc);

		/* DOSR MSB & LSB values */
		aic3100_write(codec, DAC_OSR_MSB, aic3100_divs[i].dosr >> 8);
		aic3100_write(codec, DAC_OSR_LSB,
			      aic3100_divs[i].dosr & AIC3100_8BITS_MASK);

		/* AOSR value */
		aic3100_write(codec, ADC_OSR_REG, aic3100_divs[i].aosr);
	}
	/* BCLK N divider */
	aic3100_write(codec, BCLK_N_VAL, aic3100_divs[i].blck_N);

	data = aic3100_read(codec, INTERFACE_SET_REG_1);
	data = data & ~(3 << 4);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (AIC3100_WORD_LEN_20BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (AIC3100_WORD_LEN_24BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (AIC3100_WORD_LEN_32BITS << DAC_OSR_MSB_SHIFT);
		break;
	}

	/* Write to Page 0 Reg 27 for the Codec Interface control 1
	 * Register */
	aic3100_write(codec, INTERFACE_SET_REG_1, data);

        /* Add the Processing blocks section as per the discussion
	 * with Design team */
        aic3100_write (codec, DAC_PRB_SEL_REG, 0x02);

	DBG("##- SET aic3100_hw_params\n");
	/*	Enabling audio clock source from omap4 */
	phymuxbase = ioremap(0x4A30A000, 0x1000);
        phy_val = __raw_readl(phymuxbase + 0x0314);
        phy_val = (phy_val & 0xFFF0FEFF) | (0x00010100);
        __raw_writel(phy_val, phymuxbase + 0x0314);
	iounmap(phymuxbase);

	/*setting pmdown_time of pcm rtd structure to 0*/
	rtd->pmdown_time = 0;

	mutex_unlock(&aic3100->mutex);
	return 0;
}

/*
 * aic3100_config_hp_volume
 *
 * Configure the I2C Transaction global variables. One of them is for ramping down the HP
 * Analog Volume and the other one is for ramping up the HP Analog Volume
 */
void aic3100_config_hp_volume (struct snd_soc_codec *codec, int mute)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct i2c_client *client = codec->control_data;
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);

	unsigned int count;
	struct aic3100_configs  *pReg;
	signed char regval;
        unsigned char low_value;
        unsigned int  reg_update_count;

	/* User has requested to mute or bring down the Headphone Analog Volume
         * Move from 0 db to -35.2 db
         */
	if (mute > 0) {
		pReg = &aic3100->hp_analog_right_vol[0];

		for (count = 0, regval = 0; regval <= 30; count++, regval +=1) {
			(pReg + count)->reg_offset = (R_ANLOG_VOL_2_HPR - PAGE_1);
			(pReg + count)->reg_val = (0x80 | regval);
		}
                (pReg +  (count -1))->reg_val = (0x80 | HEADPHONE_ANALOG_VOL_MIN);

		pReg = &aic3100->hp_analog_left_vol[0];

		for (count = 0, regval = 0; regval <= 30; count++, regval +=1) {
			(pReg + count)->reg_offset = (L_ANLOG_VOL_2_HPL - PAGE_1);
			(pReg + count)->reg_val = (0x80 | regval);
		}
                (pReg +  (count -1))->reg_val = (0x80 | HEADPHONE_ANALOG_VOL_MIN);
                reg_update_count = count - 1;
	} else {
		/* User has requested to unmute or bring up the Headphone Analog
                 * Volume Move from -35.2 db to 0 db
	         */
		pReg = &aic3100->hp_analog_right_vol[0];

                low_value = HEADPHONE_ANALOG_VOL_MIN;

		for (count = 0, regval = low_value; regval >= 0;
                     count++, regval-=1) {
			(pReg + count)->reg_offset = (R_ANLOG_VOL_2_HPR - PAGE_1);
			(pReg + count)->reg_val = (0x80 | regval);
		}
                (pReg + (count-1))->reg_val = (0x80);

		pReg = &aic3100->hp_analog_left_vol[0];

		for (count = 0, regval = low_value; regval >=0;
                     count++, regval-=1) {
			(pReg + count)->reg_offset = (L_ANLOG_VOL_2_HPL - PAGE_1);
			(pReg + count)->reg_val = (0x80 | regval);
		}
                (pReg + (count -1))->reg_val = (0x80);
                reg_update_count = count;
	}

	/* Change to Page 1 */
	aic3100_change_page (codec, 1);

	if (aic3100->i2c_regs_status == 0) {
                for (count = 0; count < reg_update_count; count++) {
			i2c_right_transaction[count].addr = client->addr;
			i2c_right_transaction[count].flags =
                                client->flags & I2C_M_TEN;
			i2c_right_transaction[count].len = 2;
			i2c_right_transaction[count].buf = (char *)
                                &aic3100->hp_analog_right_vol[count];
                }

                for (count = 0; count < reg_update_count; count++) {
			i2c_left_transaction[count].addr = client->addr;
			i2c_left_transaction[count].flags =
                                client->flags & I2C_M_TEN;
			i2c_left_transaction[count].len = 2;
			i2c_left_transaction[count].buf = (char *)
                                &aic3100->hp_analog_left_vol[count];
                }
                aic3100->i2c_regs_status = 1;
	}
	/* Perform bulk I2C transactions */
	if(i2c_transfer(client->adapter, i2c_right_transaction,
                        reg_update_count) != reg_update_count) {
		printk ("Error while Write brust i2c data error on "
                        "R_ANLOG_VOL_2_HPR!\n");
	}


	if(i2c_transfer(client->adapter, i2c_left_transaction,
                        reg_update_count) != reg_update_count) {
		printk ("Error while Write brust i2c data error on "
                        "L_ANLOG_VOL_2_HPL!\n");
	}
}

/*
 * aic3100_dac_mute - mute or unmute the left and right DAC
 */
static int aic3100_dac_mute (struct snd_soc_codec *codec, int mute)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	u8 dac_reg;
	volatile u8 value;
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	volatile u16 time_out_counter;

	DBG ("##+ new aic31xx_mute_codec %d (current state is %d, headset_connected=%d) \n",
	     mute, aic3100->mute, aic3100->headset_connected);

	dac_reg = aic3100_read(codec, DAC_MUTE_CTRL_REG);

	/* Also update the global Playback Status Flag. This is required for
	 * biquad update.
	*/
	if ((mute) && (aic3100->mute != 1)) {
		aic3100->playback_status = 0;
		if (aic3100->headset_connected) {

			/* Switch ON the Class_D Speaker Amplifier */
			/*
			value = aic3100_read (codec, CLASSD_SPEAKER_AMP);
			aic3100_write (codec, CLASSD_SPEAKER_AMP, (value | 0xC0));
			*/
			/* Page 47 of the datasheets requires unmuting HP and
			   Speaker drivers first */
			/* MUTE the Headphone Left and Right */
			value = aic3100_read (codec, HPL_DRIVER);
			aic3100_write (codec, HPL_DRIVER, (value & ~0x06));

			value = aic3100_read (codec, HPR_DRIVER);
			aic3100_write (codec, HPR_DRIVER, (value & ~0x06));

#if 0
			aic3100_config_hp_volume (codec, mute);
#else
			/* Bring the HP Analog Volume Control Registers back to default value */
			value = (aic3100_read (codec, R_ANLOG_VOL_2_HPR) & 0x7F);
			while (value < 0x40) { /* Changed from 0x20 to 0x40 */
				value++;
				aic3100_write (codec, R_ANLOG_VOL_2_HPR, value);
				aic3100_write (codec, L_ANLOG_VOL_2_HPL, value);
				mdelay(1);
			}
#endif

		} else {
			/* MUTE THE Class-D Speaker Driver. Note that Netflix Mode  may be
                           active earlier. Hence reset the MUTE Flag and bring the Speaker
                           Gain back at 12 db which is the value for regular playbacks.
                        */
			value = aic3100_read (codec, SPL_DRIVER);
                        aic3100_write (codec, SPL_DRIVER, ((value & ~BIT2) | BIT3));


			/*Switch off the DRC*/
			value = aic3100_read (codec, DRC_CTRL_1);
			aic3100_write (codec, DRC_CTRL_1, (value & ~ (BIT6 | BIT5)));

		}

		aic3100_write (codec, DAC_MUTE_CTRL_REG, (dac_reg | MUTE_ON));

		/* Change the DACL and DACR volumes values to lowest value [-63.5dB]*/
		aic3100_write (codec, LDAC_VOL, 0x81);
		aic3100_write (codec, RDAC_VOL, 0x81);
		time_out_counter = 0;
		do {
			mdelay(1);
			/* Poll the DAC_FLAG register Page 0 38 for the DAC MUTE
			   Operation Completion Status */
			value = aic3100_read (codec, DAC_FLAG_2);
			time_out_counter++;
		} while ((time_out_counter < 20) && ((value & 0x11) == 0));

		/* Mute the ADC channel */

		value = aic3100_read(codec, ADC_FGA);
		aic3100_write(codec, ADC_FGA, (value | BIT7));

		aic3100->mute = 1;
	}
	else if ((!mute) && (aic3100->mute != 0)) {
		aic3100->playback_status = 1;

		/* Check whether Playback or Record Session is about to Start */
		if (aic3100->playback_stream) {

			/* We will enable the DAC UNMUTE first and finally the
			   Headphone UNMUTE to avoid pops */
			if (aic3100->headset_connected) {

				/*Read the contents of the Page 0 Reg 63 DAC Data-Path
				  Setup Register. Just retain the upper two bits and
				  lower two bits
				*/
				value = (aic3100_read(codec, DAC_CHN_REG) & 0xC3);
				aic3100_write(codec, DAC_CHN_REG, (value | LDAC_2_LCHN | RDAC_2_RCHN));

				/* Restore the values of the DACL and DACR */
				aic3100_write (codec, LDAC_VOL, 0xF9);	/* old value 0xFD */
				aic3100_write (codec, RDAC_VOL, 0xF9);	/* old Value 0xFD */

				/*DRC disable for headset path */
				aic3100_write(codec, DRC_CTRL_1,
					aic3100_read(codec, DRC_CTRL_1) & 0x9F);

				time_out_counter = 0;
				do {
					mdelay(1);
					value = aic3100_read (codec, DAC_FLAG_2);
					time_out_counter ++;
				} while ((time_out_counter < 100) && ((value & 0x11) == 0));

				aic3100_write (codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

#if 0
				aic3100_config_hp_volume (codec, mute);
#else
				/* Bring the HP Analog Volume Control Registers back to default value */
				value = aic3100_read (codec, R_ANLOG_VOL_2_HPR);
				while (value >= 1) { // TI audio team changing HP gain back to -0.5dB
					value--;
					aic3100_write (codec, R_ANLOG_VOL_2_HPR, (0x80 | value));
					aic3100_write (codec, L_ANLOG_VOL_2_HPL, (0x80 | value));
					mdelay(2);
				}


				DBG("##Moved R_ANLOG_VOL_2_HPR to %d\r\n", (value & 0x7F));
#endif
				/* Page 47 of the datasheets requires unmuting HP and
				   Speaker drivers first */
				/* UNMUTE the Headphone Left and Right */
				value = aic3100_read (codec, HPL_DRIVER);
				aic3100_write (codec, HPL_DRIVER, (value | 0x04));

				value = aic3100_read (codec, HPR_DRIVER);
				aic3100_write (codec, HPR_DRIVER, (value | 0x04));

			} else {
				/* Page 47 of the datasheets requires unmuting HP and
				   Speaker drivers first */
				/* MUTE the Headphone Left and Right */
				value = aic3100_read (codec, HPL_DRIVER);
				aic3100_write (codec, HPL_DRIVER, (value & ~0x06));

				value = aic3100_read (codec, HPR_DRIVER);
				aic3100_write (codec, HPR_DRIVER, (value & ~0x06));

				/*Read the contents of the Page 0 Reg 63 DAC Data-Path
				  Setup Register. Just retain the upper two bits and
				  lower two bits
				*/
				value = (aic3100_read(codec, DAC_CHN_REG) & 0xC3);
				aic3100_write(codec, DAC_CHN_REG,
					      (value | LDAC_LCHN_RCHN_2));

				/* Forcing the ANALOG volume to speaker driver to 0 dB */
				aic3100_write(codec, L_ANLOG_VOL_2_SPL, 0x80);
				aic3100_write(codec, R_ANLOG_VOL_2_SPR, 0x80);

				if (aic3100->netflix_mode == 1) {
					/* Set the L/R DAC gain to 8 dB for netflix optimization*/
					aic3100_write(codec, LDAC_VOL, 0x10);
					aic3100_write(codec, RDAC_VOL, 0x10);
				} else {

					/* Set the L/R DAC to 0
					 * Speaker Power Consumption under 0.75W
					 */
					aic3100_write(codec, LDAC_VOL, 0x00);
					aic3100_write(codec, RDAC_VOL, 0x00);
				}

				/*DRC enable for speaker path*/
                                value = aic3100_read (codec, DRC_CTRL_1);
				aic3100_write(codec, DRC_CTRL_1, (value | (BIT6 | BIT5)));

				time_out_counter = 0;
				do {
					mdelay(1);
					value = aic3100_read (codec, DAC_FLAG_2);
					time_out_counter ++;
				} while ((time_out_counter < 100) && ((value & 0x11) == 0));

				aic3100_write (codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

				if (aic3100->netflix_mode == 1) {
					/*UNMUTE the Class-D Speaker Driver and set the gain to 6dB*/
					value = aic3100_read (codec, SPL_DRIVER);
					aic3100_write (codec, SPL_DRIVER, ((value | BIT2) & 0xE7));
				} else {

					/* UNMUTE THE Left Class-D Speaker Driver */
					value = aic3100_read (codec, SPL_DRIVER);
					aic3100_write (codec, SPL_DRIVER, ((value | BIT2) & 0xEF));
				}
			}

		}
		aic3100->power_status = 1;
		aic3100->mute = 0;
	}

	DBG("##-aic31xx_mute_codec %d\n", mute);

	return 0;
}

/*
 * aic3100_mute - mute or unmute the left and right DAC
 */
static int aic3100_mute (struct snd_soc_dai *dai, int mute)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	int result;
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(dai->codec);

        DBG("##aic3100_mute handler mute=%d.\r\n", mute);

	mutex_lock(&aic3100->mutex);

	/* Check the value of the capture_stream variable to
	 * find out whether we need to Unmute the ADC block
	 * or the DAC Block.
	 */
	if (aic3100->capture_stream) {
		result = aic3100_adc_mute(dai->codec, mute);
	} else {
		result = aic3100_dac_mute(dai->codec, mute);
	}

	mutex_unlock(&aic3100->mutex);
	return result;
}

/* aic3100_set_dai_sysclk -
 * DAI Callback function to set the Codec Master Clock Input
 */
static int aic3100_set_dai_sysclk (struct snd_soc_dai *codec_dai,
				   int clk_id, unsigned int freq, int dir)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);

	DBG("##aic3100_set_dai_sysclk clk_id(%d) (%d)\n", clk_id, freq);

	switch (freq) {
	case CODEC_SYSCLK_FREQ:
		aic3100->sysclk = freq;
		return 0;
	}
	printk(KERN_ALERT "Invalid frequency to set DAI system clock\n");
	return -EINVAL;
}

/* aic3100_set_dai_fmt -
 * DAI call-back function for setting the Codec Operating Format
*/
static int aic3100_set_dai_fmt (struct snd_soc_dai *codec_dai,
				unsigned int fmt)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	u8 iface_reg;

	iface_reg = aic3100_read(codec, INTERFACE_SET_REG_1);
	iface_reg = iface_reg & ~(3 << 6 | 3 << 2);

	DBG("##+ aic3100_set_dai_fmt (%x) \n", fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic3100->master = 1;
		iface_reg |= BIT_CLK_MASTER | WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic3100->master = 0;
		break;
	default:
		printk(KERN_ALERT "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= (AIC3100_DSP_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg |= (AIC3100_RIGHT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= (AIC3100_LEFT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	default:
		printk(KERN_ALERT "Invalid DAI interface format\n");
		return -EINVAL;
	}

	DBG("##- aic3100_set_dai_fmt (%x) \n", iface_reg);
	aic3100_write(codec, INTERFACE_SET_REG_1, iface_reg);
	return 0;
}

/*
 * aic3100_set_bias_level - This function is to get triggered when dapm
 * events occurs.
 */
static int aic3100_set_bias_level (struct snd_soc_codec *codec,
                                   enum snd_soc_bias_level level)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	int result = 0;
	u8 value;
	volatile u16 counter;

	mutex_lock(&aic3100->mutex);

	DBG("##++ aic3100_set_bias_level %d\n", level);

	if (level == codec->dapm.bias_level) {
		DBG("##set_bias_level: level returning...\r\n");
		mutex_unlock(&aic3100->mutex);
		return 0;
	}

	switch (level) {
		/* full On */
	case SND_SOC_BIAS_ON:
		DBG("##aic3100_set_bias_level ON\n");
		aic3100_power_up(codec);
		break;

	case SND_SOC_BIAS_PREPARE:                  /* partial On */
		DBG("##aic3100_set_bias_level PREPARE\n");
		break;

	case SND_SOC_BIAS_STANDBY:                 /* Off, with power ??? */
		DBG("##aic3100_set_bias_level STANDBY\n");

		if (aic3100->capture_stream) {
			aic3100_adc_mute(codec, 1);
		}
           	aic3100_power_down(codec);
		break;

	case SND_SOC_BIAS_OFF:                     /* Off, without power */
		DBG("##aic3100_set_bias_level OFF\n");
		if (aic3100->capture_stream) {
			result = aic3100_adc_mute(codec, 1);
		} else {
			result = aic3100_dac_mute(codec, 1);
		}
		printk (KERN_INFO "SOC_BIAS_OFF: Switching Headset...\n");
		/* Switch off the Headset Driver here */
                /* Before switch off put the Headphone Driver in line-out Mode */
                value = aic3100_read (codec, HP_DRIVER_CTRL);
                aic3100_write (codec, HP_DRIVER_CTRL, (value | (BIT2 | BIT1)));

                /* Switch off the Head phone Drivers */
                value = aic3100_read (codec, HEADPHONE_DRIVER);
                aic3100_write (codec, HEADPHONE_DRIVER, (value & ~0xC0)); /* 0xCC */

                /* Now first check if the HPR is fully powered down */
                counter = 0;
                do {
                      mdelay(1);
                      value = aic3100_read (codec, DAC_FLAG_1);
                      counter++;
                 }while ((counter < 300) && ((value & 0x22) != 0));

		aic3100_power_down(codec);
		/* Program the i2c_reg_status back to zero. */
		aic3100->i2c_regs_status = 0;
		break;
	}
	codec->dapm.bias_level = level;
	DBG("##-- aic3100_set_bias_level\n");

	mutex_unlock(&aic3100->mutex);

	return result;
}

/* aic3100_suspend - ALSA callback function called during
 * system level suspend operation
*/
static int aic3100_suspend (struct snd_soc_codec *codec, pm_message_t state)
{
	u8 val;
	DBG(KERN_INFO "%s: started\n", __func__);
	printk(KERN_INFO "+ aic3100_suspend\n");

	aic3100_set_bias_level(codec, SND_SOC_BIAS_OFF);

	/* Bit 7 of Page 1/ Reg 46 gives the soft powerdown control. Setting this bit
	 * will further reduces the amount of power consumption
	 */
	val = aic3100_read(codec,MICBIAS_CTRL);
	aic3100_write(codec, MICBIAS_CTRL, val | BIT7);

	/*Disable Audio clock from FREF_CLK1_OUT*/
	omap_writew(omap_readw(0x4a30a314) & 0xFEFF, 0x4a30a314);

#ifdef CODEC_POWER_OFF
	gpio_set_value(AUDIO_CODEC_PWR_ON_GPIO, 0);
#endif  /* #ifdef CODEC_POWER_OFF */

	DBG ("%s: Exiting \n", __func__);

	return 0;
}

/* aic3100_resume - ALSA calback function called during
 * system level resume operation
 */
static int aic3100_resume (struct snd_soc_codec *codec)
{
	u8 val;
	DBG(KERN_INFO "%s: started\n", __func__);
	printk(KERN_INFO "+ aic3100_resume\n");

#ifdef CODEC_POWER_OFF
	gpio_set_value(AUDIO_CODEC_PWR_ON_GPIO, 1);
	
	/* sleep for 10 ms to allow the voltage to stabilize */
	msleep(10);
#endif /* #ifdef CODEC_POWER_OFF */

	/*Enable Audio clock from FREF_CLK1_OUT*/
	omap_writew(omap_readw(0x4a30a314) | ~0xFEFF, 0x4a30a314);

	/* Perform the Device Soft Power UP */
	val = aic3100_read(codec,MICBIAS_CTRL);
	aic3100_write(codec, MICBIAS_CTRL, val & ~BIT7);

        DBG("aic3100_resume: Suspend_bias_level %d\r\n",
	    codec->dapm.suspend_bias_level);
        DBG("aic3100_resume: codec_bias_level %d\r\n", codec->dapm.bias_level);

	DBG("- aic3100_resume\n");

	return 0;
}

#ifdef AIC3100_GPIO_INTR_SUPPORT

static irqreturn_t aic3100_codec_irq_handler (int irq, void* data)
{
	DBG (KERN_ALERT "interrupt of codec found\n");
	schedule_work(&codec_int_work);
	return IRQ_HANDLED;
}

#endif /* #ifdef AIC3100_GPIO_INTR_SUPPORT */

/*
 * tlv320aic3100_init - initialise the aic3100 driver register the mixer and
 * codec interfaces with the kernel.
 */
static int tlv320aic3100_init (struct snd_soc_codec * codec)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int i = 0;
	int codec_interrupt_gpio = AUDIO_CODEC_INTERRUPT /* AUDIO_GPIO_INTERRUPT_1 */;
	int codec_interrupt = 0;
	u32 val;
	DBG(KERN_ALERT "##+tlv320aic3100_init\n");

	/* Initialize private data for the codec */
	aic3100->mute              =  1;
	aic3100->headset_connected =  0;
	aic3100->power_status      =  0;
	aic3100->playback_status   =   0;
	aic3100->capture_stream    =  0;
	aic3100->i2c_regs_status   =   0;

	mutex_init(&aic3100->mutex);

	/* GPIOs 101-AUD-CODEC-EN signal as per Schematics,
	 * GPIO 102-HS-nDETECT as per Schematics */
	phymuxbase = ioremap(0x4A100000, 0x1000);
	val = __raw_readl(phymuxbase + 0x90);
	val = (val & 0xFFF0FFF0) | 0x00030003;
	__raw_writel(val, phymuxbase + 0x90);
	/* setting clock enable register
	phymuxbase = ioremap(0x4A30A000, 0x1000);
	val = __raw_readl(phymuxbase + 0x0314);
	val = (val & 0xFFF0FEFF) | (0x00020100);
	__raw_writel(val, phymuxbase + 0x0314);*/
	iounmap(phymuxbase);

	ret = gpio_request(codec_interrupt_gpio, "Codec Interrupt");
	if (ret < 0) {
		printk(KERN_INFO "%s: error in gpio request. codec interrupt"
				"failed\n", __func__);
		return ret;
	}
	gpio_direction_input(codec_interrupt_gpio);
	codec_interrupt = OMAP_GPIO_IRQ(codec_interrupt_gpio);


	ret = gpio_request(AUDIO_CODEC_PWR_ON_GPIO, AUDIO_CODEC_PWR_ON_GPIO_NAME);
	if(ret < 0) {
		printk(KERN_ERR "%s: Unable get gpio for CODEC POWER %d\n",
				__func__, AUDIO_CODEC_PWR_ON_GPIO);
	}
	gpio_direction_output(AUDIO_CODEC_PWR_ON_GPIO, 1);
	gpio_set_value(AUDIO_CODEC_PWR_ON_GPIO, 1);
	mdelay(10);

	ret = gpio_request(AUDIO_CODEC_RESET_GPIO, AUDIO_CODEC_RESET_GPIO_NAME);
	if(ret < 0) {
		printk(KERN_ERR "%s: Unable get gpio for Reset %d\n",
				__func__, AUDIO_CODEC_RESET_GPIO);
	}
	gpio_direction_output(AUDIO_CODEC_RESET_GPIO, 1);
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 1);
	mdelay(10);

	phymuxbase = ioremap(0x4A100000, 0x1000);

	/* GPIOs 101-AUD-CODEC-EN signal as per Schematics,
	 * GPIO 102-HS-nDETECT as per Schematics */
	val = __raw_readl(phymuxbase + 0x90);
	val = (val & 0xFFF8FFF8) | 0x00030003;
	__raw_writel(val, phymuxbase + 0x90);
	/* setting clock enable register - AUD-CLK-19M2-OUT
	phymuxbase = ioremap(0x4A30A000, 0x1000);
	val = __raw_readl(phymuxbase + 0x0314);
	val = (val & 0xFFF0FEFF) | (0x00020100);
	__raw_writel(val, phymuxbase + 0x0314);*/
	iounmap(phymuxbase);
	aic3100->page_no = 0;

        aic3100_change_page(codec, 0x00);
	aic3100_write(codec, RESET, 0x01);
        mdelay(10);

	for (i = 0; i < NUM_INIT_REGS; i++) {
		aic3100_write(codec, aic3100_reg_init[i].reg_offset,
			      aic3100_reg_init[i].reg_val);
                mdelay (10); /* Added delay across register writes */
	}

	/* Power-off the DAC and Headphone Drivers initially */
	aic3100->power_status = 1;
	aic3100->headset_connected = 1;
	aic3100_power_down(codec);
	aic3100->mute=0;
	aic3100_dac_mute(codec, 1);
	aic3100->headset_connected = 0;

	/* We will power-up and Power-off the Headset Driver here.
	 * Once the Headset Driver is powered ON and OFF, it will
	 * continue to stay at the Common-Mode Voltage rather
	 * than discharging completely. This helps in quick 
	 * headset playback for future iterations.
	*/
	aic3100_hp_power_up (codec);
	aic3100_hp_power_down (codec);

#ifdef AIC3100_GPIO_INTR_SUPPORT
	ret = request_irq(codec_interrupt, aic3100_codec_irq_handler,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			  IRQF_DISABLED | IRQF_SHARED , "aic3100_codec_int",
			  codec);
	if(ret != 0) {
		printk(KERN_ERR "%s: irq registration failed\n", __func__);
		free_irq(codec_interrupt, codec);
	}
#endif

	printk(KERN_ALERT "##-tlv320aic3100_init\n");
	return ret;

}

/*
 * aic3100_headset_speaker_path - This function is to check for the presence
 * of Headset and configure the Headphone of the Class D Speaker driver
 * Registers appropriately.
 */
int aic3100_headset_speaker_path (struct snd_soc_codec *codec, int gpio_status)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);
	int headset_present = 0;

	mutex_lock(&aic3100->mutex);

	headset_present = gpio_status;

	aic3100->headset_connected = headset_present;
	DBG ("%s: Headset Status %d PlaybackStatus %d\n", __func__, headset_present,
		aic3100->playback_status);
        if (aic3100->playback_status == 1) {

		/* If codec was not powered up, power up the same. */
		if(headset_present) {
                        aic3100_write(codec, HEADPHONE_DRIVER, 0xD4);
                        aic3100_write(codec, CLASSD_SPEAKER_AMP, 0x06);
		} else {

                        aic3100_write(codec, HEADPHONE_DRIVER ,0x14); /* OFF */
                        aic3100_write(codec, CLASSD_SPEAKER_AMP ,0x86 ); /* ON */
                }

                /* We will force the aic3100->mute to 1 to ensure that the
                 * following function executes completely. */
                aic3100->mute = 1;
                /* Now unmute the appropriate Codec sections with
		 * Volume Ramping */
                aic3100_dac_mute(codec, 0);
	}

#ifdef CONFIG_ADAPTIVE_FILTER
	/* Update the Biquad Array. Note that second parameter
	 *  required for the update_biquad_array is speaker_active
	 */
	aic3100_update_biquad_array(codec, !headset_present,
					    aic3100->playback_status);
#endif

	mutex_unlock(&aic3100->mutex);
	return 0;
}

/*
 * DAI ops
 */
static struct snd_soc_dai_ops aic3100_dai_ops = {
        .hw_params      = aic3100_hw_params,
        .digital_mute   = aic3100_mute,
        .set_sysclk     = aic3100_set_dai_sysclk,
        .set_fmt        = aic3100_set_dai_fmt,
};

/*
 * aic3100_get_record_status
 *
 * Helper function which checks if the Record Path is
 * active within the Audio Codec Driver. This function
 * returns the following:
 * 0	- No Recording Session in Progress or Recording stopped
 * 1	- Recording session is active and in progress
 */
int aic3100_get_record_status(void)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	/* Check the global Codec Priv Struct */
	if (unlikely(aic3100 == 0))
		return 0;

	/* Check the Priv Struct power_status and
	 * capture_status variable along with mute
	 * All these three will let us know if recording is
	 * active and in progress.
	 */
	if (aic3100->power_status && aic3100->capture_stream &&
			aic3100->mute == 0)
		return 1;

	return 0;
}
EXPORT_SYMBOL_GPL(aic3100_get_record_status);

#ifdef AIC3100_GPIO_INTR_SUPPORT
/*
 * i2c_aic3100_headset_access_work - Worker Thread Function
 */
static void aic3100_codec_access_work (struct work_struct *work)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	/* place holder for any code related to GPIO 103 interrrupt
	 * its a generic configurable interrupt from the codec.
	 * Enable AIC3100_GPIO_INTR_SUPPORT to enable the interrupt routines
	 * and this work queue function.
	 */
}

/*
 * codec_int_gpio_bh - general codec interrupt via GPIO
 */
static void codec_int_gpio_bh (struct work_struct *work)
{
	printk(KERN_DEBUG "Work Done\n");
}

#endif

/* ---------------------------------------------------------------------
 * AIC3100 Audio Driver: sysfs files for debugging
 */

static ssize_t aic3100_netflix_mode_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    DBG(KERN_INFO "%s: started\n", __func__);
    struct aic3100_priv *aic3100 = dev_get_drvdata(dev);

    printk (KERN_INFO "#%s: Function Entered..\n", __func__);

    return sprintf(buf, "%d", aic3100->netflix_mode);
}

/* Audio Driver NetFlix Mode Set Routine */
static ssize_t aic3100_netflix_mode_set(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    DBG(KERN_INFO "%s: started\n", __func__);
    struct aic3100_priv *aic3100 = dev_get_drvdata(dev);
    int mode;

    if (buf != NULL) {
        sscanf (buf, "%d", &mode);

        if ((mode == 0) || (mode == 1)) {
            aic3100->netflix_mode = mode;
            printk (KERN_INFO "#%s: Configuring Netflix Mode to %d\n", __func__, mode);
        }
        else {
            printk (KERN_INFO "#%s: Mode value Invalid..\n", __func__, mode);
            aic3100->netflix_mode = 0;
        }
    }
    else
        aic3100->netflix_mode = 0;

    return count;
}

static DEVICE_ATTR(netflix_mode, 0777, aic3100_netflix_mode_show, aic3100_netflix_mode_set);

/*-------------------------------------------------------------------------------------------------*/

typedef unsigned int (*hw_read_t)(struct snd_soc_codec *, unsigned int);

/*
 * aic3100_probe - This is first driver function called by the SoC core
 * driver.
 */
static int aic3100_probe (struct snd_soc_codec *codec)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	int ret = 0, gpio = AUDIO_CODEC_PWR_ON_GPIO;
	struct aic3100_priv *aic3100 = snd_soc_codec_get_drvdata(codec);

        DBG("+aic3100_probe: function entered\r\n");

        if (!codec) {
                printk(KERN_ERR "aic3100_probe: Codec not yet Registered..\n");
                return -ENODEV;
        }

        ret = gpio_request(gpio, AUDIO_CODEC_PWR_ON_GPIO_NAME);
        gpio_direction_output(gpio, 0);
        gpio_set_value(gpio, 1);

	codec->control_data = aic3100->control_data;
	codec->hw_write = (hw_write_t) i2c_master_send;
	codec->hw_read = (hw_read_t) i2c_master_recv;

        /* Codec Initialization Routine*/
        ret = tlv320aic3100_init(codec);
        if (ret < 0) {
                printk(KERN_ERR "%s: Failed to initialise device\n", __func__);
                return ret;
        }
	/* Register the sysfs files for debugging */
	/* Create SysFS files */
	ret = device_create_file(codec->dev, &dev_attr_netflix_mode);
	if (ret)
		printk (KERN_ERR "error creating sysfs files\n");
	else
		printk (KERN_INFO "Created the sysfs Files for Audio Driver\n");

	snd_soc_add_controls(codec, aic3100_snd_controls,
			     ARRAY_SIZE(aic3100_snd_controls));
#ifdef CONFIG_ADAPTIVE_FILTER
	aic3100_add_biquads_controls (codec);

	aic3100_add_EQ_mixer_controls (codec);

        aic3100_parse_biquad_array (codec);
#endif
        DBG("-aic3100_probe function exited..\r\n");

	return ret;
}

/*
 * Remove aic3100 soc device
 */
static int aic3100_remove (struct snd_soc_codec *codec)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	/* power down chip */
	if (codec->control_data)
		aic3100_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

/*
 * It is SoC Codec DAI structure which has DAI capabilities viz., playback
 * and capture, DAI runtime information viz. state of DAI and pop wait state,
 * and DAI private data.  The aic3100 rates ranges from 8k to 192k The PCM
 * bit format supported are 16, 20, 24 and 32 bits
 */
struct snd_soc_dai_driver aic3100_dai = {
	.name = "tlv320aic3100-dai",
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
	.ops = &aic3100_dai_ops,
};

static struct snd_soc_codec_driver aic3100_codec = {
	.read = aic3100_read,
	.write = aic3100_write,
	.set_bias_level = aic3100_set_bias_level,
	.reg_cache_size = AIC3100_CACHEREGNUM,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = aic3100_reg,
	.probe = aic3100_probe,
	.remove = aic3100_remove,
	.suspend = aic3100_suspend,
	.resume = aic3100_resume,
};

/*
 * This function attaches the i2c client and initializes aic3100 codec.
 */
static int tlv320aic3100_i2c_probe (struct i2c_client *i2c,
                                    const struct i2c_device_id *id)
{
	int ret;
	DBG(KERN_INFO "%s: started\n", __func__);
        aic3100 = kzalloc (sizeof (struct aic3100_priv), GFP_KERNEL);
        if (aic3100 == NULL) {
                printk(KERN_ERR "tlv320aic3100_i2c_probe: Failed to create "
				"Codec Private Data\n");
                return -ENOMEM;
        }

	aic3100->control_data = i2c;
	i2c_set_clientdata(i2c, aic3100);

#ifdef AIC3100_GPIO_INTR_SUPPORT
	INIT_WORK(&codec_init_works, aic3100_codec_access_work);
#endif

	ret = snd_soc_register_codec(&i2c->dev, &aic3100_codec,
				     &aic3100_dai, 1);

	if (ret < 0) {
		printk(KERN_ERR "tlv320aic3100_i2c_probe: failed to attach"
				" codec at addr\n");
	} else {
                DBG("aic3100_register sucess...\r\n");
        }
        DBG("tlv320aic3100_i2c_probe exited...\r\n");
	return ret;
}

/*
 * This function removes the i2c client and uninitializes AIC3100 codec.
 */
static int __exit tlv320aic3100_i2c_remove (struct i2c_client *i2c)
{
	DBG(KERN_INFO "%s: started\n", __func__);
	snd_soc_unregister_codec(&i2c->dev);
	kfree(i2c_get_clientdata(i2c));

	return 0;
}

/* i2c Device ID Struct used during Driver Initialization and Shutdown */
static const struct i2c_device_id tlv320aic3100_id[] = {
        {"tlv320aic3100", 0},
        {}
};

MODULE_DEVICE_TABLE(i2c, tlv320aic3100_id);

/* Definition of the struct i2c_driver structure */
static struct i2c_driver tlv320aic3100_i2c_driver = {
	.driver = {
		.name = "tlv320aic3100-codec",
                .owner = THIS_MODULE,
	},
	.probe = tlv320aic3100_i2c_probe,
	.remove = __exit_p(tlv320aic3100_i2c_remove),
	.id_table = tlv320aic3100_id,
};

/* I2C Init Routine */
static int __init aic3100_i2c_init (void)
{
        DBG(KERN_INFO "%s: started\n", __func__);
        int ret;
	DBG(KERN_INFO "%s: mod init\n", __func__);
        ret = i2c_add_driver(&tlv320aic3100_i2c_driver);
        if (ret) {
		printk(KERN_ERR "%s: error regsitering i2c driver, %d\n",
        			               __func__, ret);
	}
	return ret;
}

/* I2C Exit Routine */
static void __exit aic3100_i2c_exit (void)
{
        DBG(KERN_INFO "%s: started\n", __func__);
        i2c_del_driver(&tlv320aic3100_i2c_driver);
}

module_init(aic3100_i2c_init);
module_exit(aic3100_i2c_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC3100 codec driver");
MODULE_AUTHOR("ravindra@mistralsolutions.com");
MODULE_AUTHOR("santosh.s@mistralsolutions.com");
MODULE_AUTHOR("preetam@mistralsolutions.com");
MODULE_LICENSE("GPL");
