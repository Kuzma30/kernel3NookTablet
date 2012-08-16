/*
 * Button and Button LED support OMAP44xx tablet.
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>

#include <plat/omap_apps_brd_id.h>

#include "mux.h"
#include "board-acclaim.h"


static const int acclaim_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(1, 0, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data acclaim_keymap_data = {
	.keymap			= acclaim_keymap,
	.keymap_size		= ARRAY_SIZE(acclaim_keymap),
};

void keypad_pad_wkup(int enable)
{
 	int (*set_wkup_fcn)(const char *muxname);

 	/* PAD wakup for keyboard is needed for off mode
 	 * due to IO isolation.
 	 */
 	if (!off_mode_enabled)
 		return;

 	if (enable)
 		set_wkup_fcn = omap_mux_enable_wkup;
 	else
 		set_wkup_fcn = omap_mux_disable_wkup;

 	set_wkup_fcn("kpd_col0.kpd_col0");
 	set_wkup_fcn("kpd_row0.kpd_row0");
 	set_wkup_fcn("kpd_row1.kpd_row1");
}

void keyboard_mux_init(void)
{
	// Column mode
	omap_mux_init_signal("kpd_col0.kpd_col0",
			OMAP_WAKEUP_EN | OMAP_MUX_MODE0);
	// Row mode
	omap_mux_init_signal("kpd_row0.kpd_row0",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE0 |
			OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row1.kpd_row1",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE0 |
			OMAP_INPUT_EN);
}

static struct omap4_keypad_platform_data acclaim_keypad_data = {
	.keymap_data		= &acclaim_keymap_data,
	.rows				= 2,
	.cols				= 1,
	.keypad_pad_wkup    = keypad_pad_wkup,
};

static struct gpio_keys_button acclaim_gpio_keys[] = {
	{
		.code 		= KEY_POWER,
		.gpio 		= 29,
		.desc 		= "POWER",
		.active_low = 0,
		.wakeup 	= 1,
	},
	{

		.code 		= KEY_HOME,
		.gpio 		= 32,
		.desc 		= "HOME",
		.active_low = 1,
		.wakeup 	= 1,
	}
};

static struct gpio_keys_platform_data acclaim_gpio_keys_info = {
	.buttons	= acclaim_gpio_keys,
	.nbuttons	= ARRAY_SIZE ( acclaim_gpio_keys ),
};

static struct platform_device acclaim_gpio_keys_device = {
	.name		= "gpio-keys",
	.id			= -1,
	.dev		= {
				.platform_data = &acclaim_gpio_keys_info,
	},
};


static struct platform_device *tablet_devices[] __initdata = {
	&acclaim_gpio_keys_device,
};

int __init acclaim_button_init(void)
{
	int status;
 	keyboard_mux_init();
	status = omap4_keyboard_init(&sdp4430_keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);

	platform_add_devices(tablet_devices, ARRAY_SIZE(tablet_devices));
	return 0;

}
