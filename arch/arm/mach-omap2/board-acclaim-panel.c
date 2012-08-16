/*
 * arch/arm/mach-omap2/board-acclaim-panel.c
 *
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/platform_device.h>
#include <linux/omapfb.h>
#include <video/omapdss.h>
#include <video/omap-panel-boxer.h>

#include <linux/i2c/twl.h>

#include <plat/android-display.h>
#include <plat/vram.h>
#include <plat/omap_apps_brd_id.h>

#include "board-acclaim.h"
#include "control.h"
#include "mux.h"


int __init tablet_panel_init(void)
{
	tablet_lcd_init();
	tablet_hdmi_mux_init();

	omapfb_set_platform_data(&tablet_fb_pdata);

	omap_display_init(get_panel_data(tablet_panel_type)->board_info);
	platform_device_register(&omap4_tablet_disp_led);

	i2c_register_board_info(2, omap4xx_i2c_bus2_d2l_info,
		ARRAY_SIZE(omap4xx_i2c_bus2_d2l_info));

	return 0;
}
