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
#include <linux/leds-omap4430sdp-display.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/twl6040-vib.h>
#include <linux/wl12xx.h>
//#include <linux/cdc_tcxo.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/board-nooktablet.h>
#include "leds-omap-pwm.h"

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include "control.h"
#include "timer-gp.h"
#include <plat/display.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>

#include <linux/wakelock.h>
//#include <plat/opp_twl_tps.h>
#include <plat/syntm12xx.h>
//#include <plat/hwspinlock.h>
#include <plat/dmtimer.h>
#include "mux.h"

#define DEFAULT_BACKLIGHT_BRIGHTNESS	105

static void nooktablet_init_display_led(void)
{
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
}

static void nooktablet_disp_backlight_setpower(struct omap_pwm_led_platform_data *pdata, int state)
{
	if (state)
		gpio_direction_output(38, (acclaim_board_type() >= EVT2) ? 1 : 0);
	else
		gpio_direction_output(38, (acclaim_board_type() >= EVT2) ? 0 : 1);
	gpio_direction_output(44, 0);
	gpio_direction_output(45, 0);
	printk("[BL set power] %d\n", state);
}

static struct omap_pwm_led_platform_data nooktablet_disp_backlight_data = {
	.name 		 = "lcd-backlight",
	.intensity_timer = 11,
	.def_on		 = 0,
	.def_brightness	 = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.set_power	 = nooktablet_disp_backlight_setpower,
};

static struct platform_device nooktablet_disp_led = {
	.name	=	"omap_pwm_led",
	.id	=	-1,
	.dev	= {
		.platform_data = &nooktablet_disp_backlight_data,
	},
};

static struct platform_device *nooktablet_devices[] __initdata = {
	&nooktablet_disp_led,
};

/*--------------------------------------------------------------------------*/

static void nooktablet_panel_get_resource(void)
{
	int ret_val = 0;

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

static struct omap_dss_device sdp4430_boxer_device = {
	.name				= "boxerLCD",
	.driver_name		= "boxer_panel_drv",
	.type				= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.channel			= OMAP_DSS_CHANNEL_LCD2,
	.data				= &boxer_panel,
};

static struct omap_dss_device *sdp4430_dss_devices[] = {
	&sdp4430_boxer_device,
};

static __initdata struct omap_dss_board_info sdp4430_dss_data = {
	.num_devices	=	ARRAY_SIZE(sdp4430_dss_devices),
	.devices	=	sdp4430_dss_devices,
	.default_device	=	&sdp4430_boxer_device,
};

void __init nooktablet_panel_init(void)
{
	nooktablet_panel_get_resource();
	omap_display_init(&sdp4430_dss_data);
	nooktablet_init_display_led();
	platform_add_devices(nooktablet_devices, ARRAY_SIZE(nooktablet_devices));
}
