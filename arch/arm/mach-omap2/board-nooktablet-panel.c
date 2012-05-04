
/*
 * Board support file for OMAP4430 ACCLAIM.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
//#include <linux/leds-omap4430sdp-display.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/omapfb.h>

//#include <linux/twl6040-vib.h>
//#include <linux/wl12xx.h>
//#include <linux/cdc_tcxo.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/board-nooktablet.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include "control.h"
//#include <plat/timer-gp.h>
//#include <plat/display.h>
#include <video/omapdss.h>
#include <video/omap-panel-nokia-dsi.h>
#include <plat/vram.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>

#include <linux/wakelock.h>
//#include <plat/opp_twl_tps.h>
#include <plat/syntm12xx.h>
//#include <plat/hwspinlock.h>
#include <plat/dmtimer.h>
#include "mux.h"

#include <linux/temphack.h>

#define LCD_RST_DELAY		100
#define LCD_INIT_DELAY		200

//#define DEFAULT_BACKLIGHT_BRIGHTNESS	10
static void acclaim4430_init_display_led(void)
{
#ifndef TEMP_HACK
 	if (acclaim_board_type() >= EVT2) {
  		printk(KERN_INFO "init_display_led: evt2 hardware\n");
  		omap_mux_init_signal("abe_dmic_din2.dmtimer11_pwm_evt", OMAP_MUX_MODE5);
	} else {
		printk(KERN_INFO "init_display_led: evt1 hardware\n");
		printk(KERN_INFO "WARNING: brigthness control disabled on EVT1 hardware\n");
		/* mux the brightness control pin as gpio, because on EVT1 it is connected to
		   timer8 and we cannot use timer8 because of audio conflicts causing crash */
		omap_mux_init_signal("usbb1_ulpitll_dat4.gpio_92", OMAP_MUX_MODE3);
		if (gpio_request(92, "EVT1 BACKLIGHT"))
			printk(KERN_ERR "ERROR: failed to request backlight gpio\n");
		else
			gpio_direction_output(92, 0);
	}
#else
	printk(KERN_INFO "Temporary Hack for LCD PWM LED\n");
	printk(KERN_INFO "WARNING: brigthness control disabled now\n");
	/* mux the brightness control pin as gpio, because on EVT1 it is connected to
	   timer8 and we cannot use timer8 because of audio conflicts causing crash */
	omap_mux_init_signal("abe_dmic_din2.gpio_121", OMAP_MUX_MODE3);
//	if (gpio_request(121, "EVT1 BACKLIGHT"))
//		printk(KERN_ERR "ERROR: failed to request backlight gpio\n");
//	else
//		gpio_direction_output(121, 0);
#endif
}

static void acclaim4430_disp_backlight_setpower(struct omap_pwm_led_platform_data *pdata, int on_off)
{
	printk(KERN_INFO "Backlight set power, on_off = %d\n",on_off);
	if (on_off)
		gpio_direction_output(38, (acclaim_board_type() >= EVT2) ? 1 : 0);
	else
		gpio_direction_output(38, (acclaim_board_type() >= EVT2) ? 0 : 1);
	gpio_direction_output(44, 0);
	gpio_direction_output(45, 0);
	pr_debug("%s: on_off:%d\n", __func__, on_off);
	// enable this fixed backlight startup for A100 on low level
	// but could generate a little white flash at start
	msleep(500);
	printk(KERN_INFO "Backlight set power end\n");
}

#ifndef TEMP_HACK
static struct omap_pwm_led_platform_data acclaim4430_disp_backlight_data = {
	.name 		 = "lcd-backlight",
	.default_trigger  = "backlight",
	.intensity_timer = 11,
	.bkl_max    = 254,
	.bkl_min    = 0,
	.bkl_freq    = 30000,
	.invert     = 1,
/*	.def_on		 = 0,
	.def_brightness	 = DEFAULT_BACKLIGHT_BRIGHTNESS,*/
	.set_power	 = acclaim4430_disp_backlight_setpower,
	
};

static struct platform_device sdp4430_disp_led = {
	.name	=	"omap_pwm_led",
	.id	=	0,
	.dev	= {
		.platform_data = &acclaim4430_disp_backlight_data,
	},
};

static struct platform_device *sdp4430_panel_devices[] __initdata = {
	&sdp4430_disp_led,
};
#endif
static void sdp4430_panel_get_resource(void)
{
	int ret_val = 0;

	pr_info("sdp4430_panel_get_resource\n");
	ret_val = gpio_request(38, "BOXER BL PWR EN");

	if ( ret_val ) {
		printk("%s : Could not request bl pwr en\n",__FUNCTION__);
	}
	ret_val = gpio_request(44, "BOXER CABC0");
	if ( ret_val ){
		printk( "%s : could not request CABC0\n",__FUNCTION__);
	}
	ret_val = gpio_request(45, "BOXER CABC1");
	if ( ret_val ) {
		printk("%s: could not request CABC1\n",__FUNCTION__);
	}
}

static struct boxer_panel_data boxer_panel;

static inline struct boxer_panel_data * get_panel_data(struct omap_dss_device *dssdev)
{
	return dssdev->data;
}

static int nooktablet_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	acclaim4430_disp_backlight_setpower(NULL,1);
	pr_info("NookTablet LCD enable!\n");
	return 0;
}

static void nooktablet_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	acclaim4430_disp_backlight_setpower(NULL,0);
	pr_info("NookTablet LCD disable!\n");
}

static struct omap_dss_device sdp4430_boxer_device = {
	.phy		= {
		.dpi	= {
			.data_lines	= 24,
		},
	},
	.clocks		= {
		.dispc	= {
			.channel	= {
				.lck_div        = 1,
				.pck_div        = 4,
				.lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
		},
#if 0
		.dsi	= {
			.regn		= 16, /*it is (N+1)*/
			.regm		= 115,
			.regm_dispc	= 3,
			.regm_dsi	= 3,
			.dsi_fclk_src   = OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DSI,
		},
#endif
	},
        .panel          = {
		.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
				  OMAP_DSS_LCD_IHS,
		.timings	= {
			.x_res          = 1024,
			.y_res          = 600,
			.pixel_clock    = 46000, /* in kHz */
			.hfp            = 160,   /* HFP fix 160 */
			.hsw            = 10,    /* HSW = 1~140 */
			.hbp            = 150,   /* HSW + HBP = 160 */
			.vfp            = 12,    /* VFP fix 12 */
			.vsw            = 3,     /* VSW = 1~20 */
			.vbp            = 20,    /* VSW + VBP = 23 */
		},
        	.width_in_um = 158000,
        	.height_in_um = 92000,
        },
#if 0
	.ctrl = {
		.pixel_size = 24,
	},
#endif
	.name			= "lcd2",
	.driver_name		= "boxer_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
  	.platform_enable	= nooktablet_panel_enable_lcd,
  	.platform_disable	= nooktablet_panel_disable_lcd,
	.max_backlight_level	= 255,
};

static struct omap_dss_device *sdp4430_dss_devices[] = {
 	&sdp4430_boxer_device,
};
 
static struct omap_dss_board_info sdp4430_dss_data = {
	.num_devices	= ARRAY_SIZE(sdp4430_dss_devices),
	.devices	= sdp4430_dss_devices,
	.default_device	= &sdp4430_boxer_device,
};

static struct spi_board_info tablet_spi_board_info[] __initdata = {
{
	.modalias= "boxer_disp_spi",
	.bus_num= 4,     /* McSPI4 */
	.chip_select= 0,
	.max_speed_hz= 375000,
	},
};

void __init acclaim_panel_init(void)
{
	sdp4430_panel_get_resource();
	acclaim4430_init_display_led();
	omap_display_init(&sdp4430_dss_data);
	int ret;
	
	spi_register_board_info(tablet_spi_board_info,ARRAY_SIZE(tablet_spi_board_info));

	omap_mux_enable_wkup("sys_nirq1");
	omap_mux_enable_wkup("sys_nirq2");
#ifndef TEMP_HACK
	platform_add_devices(sdp4430_panel_devices, ARRAY_SIZE(sdp4430_panel_devices));
#endif
}
