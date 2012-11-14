/* drivers/leds/leds-omap-pwm-dmtimer.h
*
* Driver to control backlight LEDs using OMAP GPTIMER PWM timers.
* Exported functions.
*
* Copyright (C) 2011 MM Solutions
* Author: Atanas Tulbenski <atulbenski@mm-sol.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/
#ifndef __LEDS_OMAP_PWM_DMTIMER_H
#define __LEDS_OMAP_PWM_DMTIMER_H

int omap_dmtimer_pwm_led_init(struct omap_pwm_led *led);
int omap_dmtimer_pwm_led_release(struct omap_pwm_led *led);

#endif
