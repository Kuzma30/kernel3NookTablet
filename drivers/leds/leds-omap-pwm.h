/* drivers/leds/leds-omap-pwm.h
*
* Driver to control backlight LEDs using OMAP/TWL6030 PWM timers.
* Internal context strucure.
*
* Copyright (C) 2011 MM Solutions
* Author: Atanas Tulbenski <atulbenski@mm-sol.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/
#ifndef __LEDS_OMAP_PWM_H
#define __LEDS_OMAP_PWM_H

#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/leds.h>

struct omap_pwm_led {
	struct platform_device			*pdev;
	struct led_classdev			cdev;
	struct omap_pwm_led_platform_data	*pdata;
	enum led_brightness			brightness;
	int powered;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend			early_suspend;
#endif
	void					*ctx;

	void (*pwm_power_off) (struct omap_pwm_led *self);
	void (*pwm_power_on)  (struct omap_pwm_led *self);
	void (*pwm_set_cycle) (struct omap_pwm_led *self, int cycle);
};

#endif
