/*
 * linux/sound/soc/codecs/tlv320aic3100.h
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1   ASoC driver support    Mistral         14-04-2010
 *
 * Rev 0.2   Updated based Review Comments Mistral      29-06-2010
 *
 * Rev 0.3   Updated for Codec Family Compatibility     12-07-2010
 *
 * Rev 0.4   Ported to 2.6.35 kernel
 *
 * Rev 0.5   Updated the code-base with ADC related register #defines.
 *
 * Rev 0.6   Added the PAGE_3 #define and the Prog Delay Timer Register
 *	     define to be used for HP and DAC Rampup time.
 */

#ifndef _TLV320AIC3100_H
#define _TLV320AIC3100_H

#define AUDIO_NAME "aic3100"
#define AIC3100_VERSION "1.0"

/*#define AIC3100_DEBUG*/

#ifdef AIC3100_DEBUG
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...)
#endif

#define TLV320AIC3100ID                  (0x18)

#define CONFIG_TILOAD
#undef CONFIG_TILOAD

/* Build Macro for enabling/disabling the Adaptive Filtering */
#define CONFIG_ADAPTIVE_FILTER

/* DAPM Support enable */
/*#define DAPM_SUPPORT*/
#undef DAPM_SUPPORT

/* GPIO interrupt Support*/
/*#define AIC3100_GPIO_INTR_SUPPORT*/
#undef AIC3100_GPIO_INTR_SUPPORT

/* Enable register caching on write */
/*#define EN_REG_CACHE*/

/* The default clock for OMAP 4 */
#define CODEC_SYSCLK_FREQ 19200000
#define    HEADSET_DETECT_GPIO_PIN 		102
#define    AUDIO_CODEC_HPH_DETECT_GPIO		(102)
#define    AUDIO_CODEC_PWR_ON_GPIO		(101)
#define    AUDIO_CODEC_INTERRUPT		(103)
#define    AUDIO_CODEC_RESET_GPIO		(104)
#define    AUDIO_CODEC_PWR_ON_GPIO_NAME		"audio_codec_pwron"
#define    AUDIO_CODEC_RESET_GPIO_NAME		"audio_codec_reset"

#define AIC3100_RATES	(SNDRV_PCM_RATE_8000_192000)

/* aic3100 supports the word formats 16bits, 20bits, 24bits and 32 bits */
#define AIC3100_FORMATS (SNDRV_PCM_FMTBIT_S16_LE  |	\
			 SNDRV_PCM_FMTBIT_S20_3LE |	\
			 SNDRV_PCM_FMTBIT_S24_3LE |	\
			 SNDRV_PCM_FMTBIT_S32_LE)

#define HEADPHONE_ANALOG_VOL_MIN	0x20

/* Audio data word length = 16-bits (default setting) */
#define AIC3100_WORD_LEN_16BITS		0x00
#define AIC3100_WORD_LEN_20BITS		0x01
#define AIC3100_WORD_LEN_24BITS		0x02
#define AIC3100_WORD_LEN_32BITS		0x03

/* sink: name of target widget */
#define AIC3100_WIDGET_NAME		0
/* control: mixer control name */
#define AIC3100_CONTROL_NAME		1
/* source: name of source name */
#define AIC3100_SOURCE_NAME		2

/* D15..D8 aic3100 register offset */
#define AIC3100_REG_OFFSET_INDEX    	0
/* D7...D0 register data */
#define AIC3100_REG_DATA_INDEX		1

/* Serial data bus uses I2S mode (Default mode) */
#define AIC3100_I2S_MODE		0x00
#define AIC3100_DSP_MODE		0x01
#define AIC3100_RIGHT_JUSTIFIED_MODE	0x02
#define AIC3100_LEFT_JUSTIFIED_MODE	0x03

/* 8 bit mask value */
#define AIC3100_8BITS_MASK           	0xFF

/* shift value for CLK_REG_3 register */
#define CLK_REG_3_SHIFT			6	//to be check
/* shift value for DAC_OSR_MSB register */
#define DAC_OSR_MSB_SHIFT		4	//to be check

/* number of codec specific register for configuration */
#define NO_FEATURE_REGS     		2

/* aic3100 register space */

/* ****************** Page 0 Registers **************************************/
#define	PAGE_SELECT			0
#define	RESET				1
#define OT_FLAG				3
#define	CLK_REG_1			4
#define	CLK_REG_2			5
#define	CLK_REG_3			6
#define	CLK_REG_4			7
#define	CLK_REG_5			8
#define	NDAC_CLK_REG			11
#define	MDAC_CLK_REG			12
#define DAC_OSR_MSB			13
#define DAC_OSR_LSB			14
#define	NADC_CLK_REG			18
#define	MADC_CLK_REG			19
#define ADC_OSR_REG			20
#define CLK_MUX_REG_9		        25
#define CLK_REG_10			26
#define INTERFACE_SET_REG_1	        27
#define AIS_REG_2			28
#define INTERFACE_SET_REG_2		29
#define BCLK_N_VAL			30
#define AIS_REG_4			31
#define INTERFACE_SET_REG_4		32
#define INTERFACE_SET_REG_5		33
#define I2C_FLAG			34
#define ADC_FLAG			36
#define DAC_FLAG_1			37
#define DAC_FLAG_2			38
#define OVERFLOW_FLAG			39
#define INTR_FLAG_1			44
#define INTR_FLAG_2			46
#define INTL_CRTL_REG_1			48
#define INT2_CTRL			49
#define GPIO_CRTL_REG_1			51
#define DOUT_CTRL			53
#define DIN_CTL				54
#define DAC_PRB_SEL_REG			60
#define ADC_PRB_SEL_REG			61
#define DAC_CHN_REG			63
#define DAC_MUTE_CTRL_REG		64
#define LDAC_VOL			65
#define RDAC_VOL			66
#define HEADSET_DETECT			67
#define DRC_CTRL_1			68
#define DRC_CTRL_2			69
#define DRC_CTRL_3			70
#define BEEP_GEN_L			71
#define BEEP_GEN_R			72
#define BEEP_LEN_MSB			73
#define BEEP_LEN_MID			74
#define BEEP_LEN_LSB			75
#define BEEP_SINX_MSB			76
#define BEEP_SINX_LSB			77
#define BEEP_COSX_MSB			78
#define BEEP_COSX_LSB			79
#define ADC_DIG_MIC			81
#define	ADC_FGA				82
#define	ADC_CGA				83

/*Channel AGC Control Register 1*/
#define AGC_CTRL_1			    86
/*Channel AGC Control Register 2*/
#define AGC_CTRL_2			    87
/*Channel AGC Control Register 3 */
#define AGC_CTRL_3			    88
/*Channel AGC Control Register 4 */
#define AGC_CTRL_4			    89
/*Channel AGC Control Register 5 */
#define AGC_CTRL_5			    90
/*Channel AGC Control Register 6 */
#define AGC_CTRL_6			    91
/*Channel AGC Control Register 7 */
#define AGC_CTRL_7			    92
/* AGC Gain applied reading (RO) */
#define AGC_CTRL_8			    93
/* VOL/MICDET-Pin SAR ADC Volume Control */
#define VOL_MICDECT_ADC			116
/* VOL/MICDET-Pin Gain*/
#define VOL_MICDECT_GAIN		117

/******************** Page 1 Registers **************************************/
#define PAGE_1				128
#define HP_SPK_ERR_CTL                  (PAGE_1 + 30)
/* Headphone drivers */
#define HEADPHONE_DRIVER		(PAGE_1 + 31)
/* Class-D Speakear Amplifier */
#define CLASSD_SPEAKER_AMP	(PAGE_1 + 32)
/* HP Output Drivers POP Removal Settings */
#define HP_POP_CTRL			(PAGE_1 + 33)
/* Output Driver PGA Ramp-Down Period Control */
#define PGA_RAMP_CTRL			(PAGE_1 + 34)
/* DAC_L and DAC_R Output Mixer Routing */
#define DAC_MIX_CTRL			(PAGE_1 + 35)
/*Left Analog Vol to HPL */
#define L_ANLOG_VOL_2_HPL		(PAGE_1 + 36)
/* Right Analog Vol to HPR */
#define R_ANLOG_VOL_2_HPR		(PAGE_1 + 37)
/* Left Analog Vol to SPL */
#define L_ANLOG_VOL_2_SPL		(PAGE_1 + 38)
/* Right Analog Vol to SPR */
#define R_ANLOG_VOL_2_SPR		(PAGE_1 + 39)
/* HPL Driver */
#define HPL_DRIVER			(PAGE_1 + 40)
/* HPR Driver */
#define HPR_DRIVER			(PAGE_1 + 41)
/* SPL Driver */
#define SPL_DRIVER			(PAGE_1 + 42)
/* HP Driver Control */
#define HP_DRIVER_CTRL			(PAGE_1 + 44)
/*MICBIAS Configuration Register*/
#define MICBIAS_CTRL			(PAGE_1 + 46)
/* MIC PGA*/
#define MIC_PGA				(PAGE_1 + 47)
/* Delta-Sigma Mono ADC Channel Fine-Gain Input Selection for P-Terminal */
#define MIC_GAIN			(PAGE_1 + 48)
/* ADC Input Selection for M-Terminal */
#define ADC_IP_SEL			(PAGE_1 + 49)
/*MICBIAS Configuration*/
#define CM_SET	         		(PAGE_1 + 50)
/****************************************************************************/
/*  Page 3 Registers 				              	  	    */
/****************************************************************************/
#define PAGE_3				(128 * 3)

/* Timer Clock MCLK Divider */
#define TIMER_CLOCK_MCLK_DIVIDER	(PAGE_3 + 16)

/****************************************************************************/
/****************************************************************************/
#define BIT7		(1 << 7)
#define BIT6		(1 << 6)
#define BIT5		(1 << 5)
#define BIT4		(1 << 4)
#define	BIT3		(1 << 3)
#define BIT2		(1 << 2)
#define BIT1		(1 << 1)
#define BIT0		(1 << 0)

#define HP_UNMUTE			BIT2
#define HPL_UNMUTE			BIT3
#define ENABLE_DAC_CHN			(BIT6 | BIT7)
#define ENABLE_ADC_CHN			(BIT6 | BIT7)
#define BCLK_DIR_CTRL			0x00
#define CODEC_CLKIN_MASK		0x03
#define MCLK_2_CODEC_CLKIN		0x00
#define PLLCLK_2_CODEC_CLKIN	0x03
#define CODEC_MUX_VALUE			0x03
/*Bclk_in selection*/
#define BDIV_CLKIN_MASK			0x03
#define	DAC_MOD_CLK_2_BDIV_CLKIN 	0x01

#define SOFT_RESET			0x01
#define PAGE0				0x00
#define PAGE1				0x01
#define BIT_CLK_MASTER			0x08
#define WORD_CLK_MASTER			0x04
#define ENABLE_PLL			BIT7
#define ENABLE_NDAC			BIT7
#define ENABLE_MDAC			BIT7
#define ENABLE_NADC			BIT7
#define ENABLE_MADC			BIT7
#define ENABLE_BCLK			BIT7
#define ENABLE_DAC			(0x03 << 6)
#define ENABLE_ADC			BIT7
#define LDAC_2_LCHN			BIT4
#define RDAC_2_RCHN			BIT2
#define RDAC_2_RAMP			BIT2
#define LDAC_2_LAMP			BIT6
#define LDAC_CHNL_2_HPL			BIT3
#define RDAC_CHNL_2_HPR			BIT3
#define LDAC_LCHN_RCHN_2		(BIT4 | BIT5)
#define SOFT_STEP_2WCLK			BIT0

#define MUTE_ON				0x0C
#define DEFAULT_VOL			0x0
#define HEADSET_ON_OFF			0xC0
/* DEFAULT VOL MODIFIED [OLD VAL = 0XFC] */
#define DAC_DEFAULT_VOL			0x81
/* HP DEFAULT VOL Updated from -78.3db to -16db */
#define HP_DEFAULT_VOL			0x20
#define SPK_DEFAULT_VOL			0x75
#define DISABLE_ANALOG			BIT3
#define LDAC_2_HPL_ROUTEON		BIT3
#define RDAC_2_HPR_ROUTEON		BIT3
#define LINEIN_L_2_LMICPGA_10K		BIT6
#define LINEIN_L_2_LMICPGA_20K		BIT7
#define LINEIN_L_2_LMICPGA_40K		(0x3 << 6)
#define LINEIN_R_2_RMICPGA_10K		BIT6
#define LINEIN_R_2_RMICPGA_20K		BIT7
#define LINEIN_R_2_RMICPGA_40K		(0x3 << 6)

/* DAC volume normalization, 0=-127dB, 127=0dB, 175=+48dB */
#define DAC_MAX_VOLUME                  175
#define DAC_POS_VOL                     127
/* Headphone POP Removal Settings defines */
#define HP_POWER_UP_0_USEC	0
#define HP_POWER_UP_15_3_USEC   BIT3
#define HP_POWER_UP_153_USEC    BIT4
#define HP_POWER_UP_1_53_MSEC   (BIT4 | BIT3)
#define HP_POWER_UP_15_3_MSEC   BIT5
#define HP_POWER_UP_76_2_MSEC   (BIT5 | BIT3)
#define HP_POWER_UP_153_MSEC    (BIT5 | BIT4)
#define HP_POWER_UP_304_MSEC    (BIT5 | BIT4 | BIT3)
#define HP_POWER_UP_610_MSEC    (BIT6)
#define HP_POWER_UP_1_2_SEC     (BIT6 | BIT3)
#define HP_POWER_UP_3_04_SEC	(BIT6 | BIT4)
#define HP_POWER_UP_6_1_SEC	(BIT6 | BIT4 | BIT3)
/* Driver Ramp-up Time Settings */
#define HP_DRIVER_0_MS		0
#define HP_DRIVER_0_98_MS	BIT1
#define HP_DRIVER_1_95_MS	BIT2
#define HP_DRIVER_3_9_MS	(BIT2 | BIT1)

/* Common Mode VOltage SEttings */
#define CM_VOLTAGE_FROM_AVDD		0
#define CM_VOLTAGE_FROM_BAND_GAP	BIT0

/****************************************************************************/
/*  DAPM related Enum Defines  		              	  	            */
/****************************************************************************/

#define LEFT_DAC_MUTE_ENUM		0
#define RIGHT_DAC_MUTE_ENUM 		1
#define HP_DRIVER_VOLTAGE_ENUM  	2
#define DRC_STATUS_ENUM         	3
#define ADC_MUTE_ENUM	         	4
#define MICPGA_LP_ENUM			5
#define MICPGA_RP_ENUM			6
#define MICPGA_LM_ENUM			7
#define MICPGA_CM_ENUM			8
#define MIC1LM_ENUM			9
#define MICPGA_GAIN_ENUM		10

/*  List of ramp-up step times */
#define HP_RAMP_UP_STIME_0MS		(0x0 << 1)
#define HP_RAMP_UP_STIME_0_98MS		(0x1 << 1)
#define HP_RAMP_UP_STIME_1_95MS		(0x2 << 1)
#define HP_RAMP_UP_STIME_3_9MS		(0x3 << 1)

/* List of Common-mode voltage settings */
#define HP_RESISTOR_COMMON_MODE 	0x00
#define HP_BANDGAP_COMMON_MODE  	0x01

#define AUDIO_GPIO_INTERRUPT_1		103

//#define AIC3100_GPIO_INTR_SUPPORT
#undef AIC3100_GPIO_INTR_SUPPORT

/*
 * ****************************************************************************
 * Structures Definitions
 * ****************************************************************************
 */
/*
 *----------------------------------------------------------------------------
 * @struct  aic3100_setup_data |
 *          i2c specific data setup for aic3100.
 * @field   unsigned short |i2c_address |
 *          Unsigned short for i2c address.
 *----------------------------------------------------------------------------
 */
struct aic3100_setup_data {
	unsigned short i2c_address;
};

/*
 *----------------------------------------------------------------------------
 * @struct  aic3100_configs |
 *          aic3100 initialization data which has register offset and register
 *          value.
 * @field   u16 | reg_offset |
 *          aic3100 Register offsets required for initialization..
 * @field   u8 | reg_val |
 *          value to set the aic3100 register to initialize the aic3100
 *----------------------------------------------------------------------------
 */
struct aic3100_configs {
	u16 reg_offset;
	u8 reg_val;
};

/*
 *----------------------------------------------------------------------------
 * @struct  aic3100_priv |
 *          aic3100 priviate data structure to set the system clock, mode and
 *          page number.
 * @field   u32 | sysclk |
 *          system clock
 * @field   s32 | master |
 *          master/slave mode setting for aic3100
 * @field   u8 | page_no |
 *          page number. Here, page 0 and page 1 are used.
 * @field   u8 | codec   |
 *          codec strucuture. Used while freeing the Driver Resources
 *----------------------------------------------------------------------------
 */
struct aic3100_priv {
	u32 sysclk;
	s32 master;
	u8 page_no;
	void *control_data;
	u8  mute;
	u8  headset_connected;
	u8  headset_current_status;
	u8  power_status;
	u8  playback_status;
	u8 capture_stream;
	struct mutex mutex;
	struct snd_soc_codec codec;
	u8  i2c_regs_status;
	u32   hp_driver_powered;
	u32   hp_driver_ramp_time;
	u8 playback_stream;	/* 1 denotes Playback and 0 denotes record */
	struct aic3100_configs hp_analog_right_vol[120];
	struct aic3100_configs hp_analog_left_vol[120];
	u8 netflix_mode;
};


/*
 *----------------------------------------------------------------------------
 * @struct  aic3100_rate_divs |
 *          Setting up the values to get different freqencies
 *
 * @field   u32 | mclk |
 *          Master clock
 * @field   u32 | rate |
 *          sample rate
 * @field   u8 | p_val |
 *          value of p in PLL
 * @field   u32 | pll_j |
 *          value for pll_j
 * @field   u32 | pll_d |
 *          value for pll_d
 * @field   u32 | dosr |
 *          value to store dosr
 * @field   u32 | ndac |
 *          value for ndac
 * @field   u32 | mdac |
 *          value for mdac
 * @field   u32 | aosr |
 *          value for aosr
 * @field   u32 | nadc |
 *          value for nadc
 * @field   u32 | madc |
 *          value for madc
 * @field   u32 | blck_N |
 *          value for block N
 * @field   u32 | aic3100_configs |
 *          configurations for aic3100 register value
 *----------------------------------------------------------------------------
 */
struct aic3100_rate_divs {
	u32 mclk;
	u32 rate;
	u8 p_val;
	u8 pll_j;
	u16 pll_d;
	u16 dosr;
	u8 ndac;
	u8 mdac;
	u16 aosr;
	u8 nadc;
	u8 madc;
	u8 blck_N;
	struct aic3100_configs codec_specific_regs[NO_FEATURE_REGS];
};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai |
 *          It is SoC Codec DAI structure which has DAI capabilities viz.,
 *          playback and capture, DAI runtime information viz. state of DAI
 *			and pop wait state, and DAI private data.
 *----------------------------------------------------------------------------
 */
extern struct snd_soc_dai tlv320aic3100_dai;
extern struct snd_soc_codec_device soc_codec_dev_aic3100;

int aic3100_headset_speaker_path(struct snd_soc_codec *, int);
int aic3100_change_page(struct snd_soc_codec *codec, u8 new_page);

#endif				/* _TLV320DAC3100_H */
