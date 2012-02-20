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

#define HDMI_GPIO_CT_CP_HPD		60
#define HDMI_GPIO_HPD			63  /* Hot plug pin for HDMI */
#define HDMI_GPIO_LS_OE 41

#define LCD_RST_DELAY		100
#define LCD_INIT_DELAY		200

static struct spi_device *boxer_spi;

#define DEFAULT_BACKLIGHT_BRIGHTNESS	10

static struct gpio sdp4430_hdmi_gpios[] = {
	{HDMI_GPIO_CT_CP_HPD,  GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_hpd"   },
	{HDMI_GPIO_LS_OE,      GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_ls_oe" },
};

static void acclaim4430_init_display_led(void)
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

static void acclaim4430_disp_backlight_setpower(struct omap_pwm_led_platform_data *pdata, int state)
{
	if (state)
		gpio_direction_output(38, (acclaim_board_type() >= EVT2) ? 1 : 0);
	else
		gpio_direction_output(38, (acclaim_board_type() >= EVT2) ? 0 : 1);
	gpio_direction_output(44, 0);
	gpio_direction_output(45, 0);
	printk("[BL set power] %d\n", state);
}

static struct omap_pwm_led_platform_data acclaim4430_disp_backlight_data = {
	.name 		 = "lcd-backlight",
	.intensity_timer = 11,
	.def_on		 = 0,
	.def_brightness	 = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.set_power	 = acclaim4430_disp_backlight_setpower,
};

static struct platform_device sdp4430_disp_led = {
	.name	=	"omap_pwm_led",
	.id	=	-1,
	.dev	= {
		.platform_data = &acclaim4430_disp_backlight_data,
	},
};

static struct platform_device *sdp4430_devices[] __initdata = {
	&sdp4430_disp_led,
};

/*--------------------------------------------------------------------------*/

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
static void sdp4430_hdmi_mux_init(void)
{
	u32 r;
	int status;
	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd.hdmi_hpd",
				OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("gpmc_wait2.gpio_100",
			OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("hdmi_cec.hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT_PULLUP);

	/* strong pullup on DDC lines using unpublished register */
	r = ((1 << 24) | (1 << 28)) ;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	gpio_request(HDMI_GPIO_HPD, NULL);
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA);
	gpio_direction_input(HDMI_GPIO_HPD);

	status = gpio_request_array(sdp4430_hdmi_gpios,
			ARRAY_SIZE(sdp4430_hdmi_gpios));
	if (status)
		pr_err("%s:Cannot request HDMI GPIOs %x \n", __func__, status);
}
static struct boxer_panel_data boxer_panel;

static inline struct boxer_panel_data * get_panel_data(struct omap_dss_device *dssdev)
{
	return dssdev->data;
}

static int nooktablet_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	pr_info("NookTablet LCD enable!\n");

	return 0;
}

static void nooktablet_panel_disable_lcd(struct omap_dss_device *dssdev)
{
  	pr_info("NookTablet LCD disable!\n");
}
static struct omap_dss_device sdp4430_boxer_device = {
	.name				= "boxerLCD",
	.driver_name			= "boxer_panel",
	.type				= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines		= 24,
	.channel			= OMAP_DSS_CHANNEL_LCD2,
	.data				= &boxer_panel,
 	//.platform_enable		= nooktablet_panel_enable_lcd,
 	//.platform_disable		= nooktablet_panel_disable_lcd,
};


 static struct omap_dss_device sdp4430_hdmi_device = {
 	.name = "hdmi",
 	.driver_name = "hdmi_panel",
 	.type = OMAP_DISPLAY_TYPE_HDMI,
 	.clocks	= {
 		.dispc	= {
 			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
 		},
 		.hdmi	= {
 			.regn	= 15,
 			.regm2	= 1,
 		},
 	},
 	.hpd_gpio = HDMI_GPIO_HPD,
 	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device *sdp4430_dss_devices[] = {
 	&sdp4430_boxer_device,
//	&sdp4430_hdmi_device,
};
 
 static struct omap_dss_board_info sdp4430_dss_data = {
 	.num_devices	= ARRAY_SIZE(sdp4430_dss_devices),
 	.devices	= sdp4430_dss_devices,
 	.default_device	= &sdp4430_boxer_device,
 };
 
 #define BLAZE_FB_RAM_SIZE                SZ_16M /* 1920×1080*4 * 2 */
 static struct omapfb_platform_data blaze_fb_pdata = {
 	.mem_desc = {
 		.region_cnt = 1,
 		.region = {
 			[0] = {
 				.size = BLAZE_FB_RAM_SIZE,
 			},
 		},
 	},
 };

void __init acclaim_panel_init(void)
{
	sdp4430_panel_get_resource();

//	sdp4430_hdmi_mux_init();
	omap_vram_set_sdram_vram(BLAZE_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&blaze_fb_pdata);
	omap_display_init(&sdp4430_dss_data);
	acclaim4430_init_display_led();	
	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
}
