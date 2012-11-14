/* drivers/leds/leds-omap-pwm.c
 *
 * Driver to control backlight LEDs using OMAP/TWL603x PWM timers.
 *
 * Copyright (C) 2011 MM Solutions
 * Author: Atanas Tulbenski <atulbenski@mm-sol.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/ctype.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <plat/board.h>
#include <linux/spinlock.h>
#ifdef CONFIG_ARCH_OMAP4
#include <mach/omap4-common.h>
#endif
#include "leds-omap-pwm.h"
#ifdef CONFIG_OMAP_PWM_LED_TYPE_DMTIMER
#include "leds-omap-pwm-dmtimer.h"
#endif
#ifdef CONFIG_OMAP_PWM_LED_TYPE_TWL6030
#include "leds-omap-pwm-twl6030.h"
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void omap_pwm_led_early_suspend(struct early_suspend *handler);
static void omap_pwm_led_late_resume(struct early_suspend *handler);
#endif

static void omap_pwm_led_set(struct led_classdev *led_cdev,
			     enum led_brightness value)
{
	struct omap_pwm_led *led;

	led = container_of(led_cdev, struct omap_pwm_led, cdev);

	if (value != led->brightness) {
		if (led->pwm_set_cycle)
			led->pwm_set_cycle(led, value);
	}
}

static int omap_pwm_led_probe(struct platform_device *pdev)
{
	struct omap_pwm_led_platform_data *pdata = pdev->dev.platform_data;
	struct omap_pwm_led *led;
	int ret = 0;

	led = kzalloc(sizeof(struct omap_pwm_led), GFP_KERNEL);
	if (led == NULL) {
		dev_err(&pdev->dev, "No memory for device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, led);
	led->cdev.brightness_set = omap_pwm_led_set;
	led->cdev.default_trigger = NULL;
	led->cdev.name = pdata->name;
	led->pdata = pdata;
	led->pdev = pdev;
	led->brightness = pdata->def_brightness;

#ifdef CONFIG_OMAP_PWM_LED_TYPE_DMTIMER
	dev_info(&pdev->dev, "OMAP PWM LED (%s) at GP timer %d\n",
		pdata->name, pdata->pwm_module_id);
#endif
#ifdef CONFIG_OMAP_PWM_LED_TYPE_TWL6030
	dev_info(&pdev->dev, "OMAP PWM LED (%s) at TWL6030 PWM%d\n",
		 pdata->name, pdata->pwm_module_id);
#endif
	if (pdata->def_brightness)
		led->cdev.brightness = pdata->def_brightness;

	/* register our new led device */
	ret = led_classdev_register(&pdev->dev, &led->cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "led_classdev_register failed\n");
		kfree(led);
		return ret;
	}

#ifdef CONFIG_OMAP_PWM_LED_TYPE_DMTIMER
	ret = omap_dmtimer_pwm_led_init(led);
#endif
#ifdef CONFIG_OMAP_PWM_LED_TYPE_TWL6030
	ret = twl6030_pwm_led_init(led);
#endif
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request intensity pwm timer\n");
		led_classdev_unregister(&led->cdev);
		kfree(led);
		return ret;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	led->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	led->early_suspend.suspend = omap_pwm_led_early_suspend;
	led->early_suspend.resume  = omap_pwm_led_late_resume;
	register_early_suspend(&led->early_suspend);
#endif

	if (pdata->def_brightness && pdata->set_power) {
		pdata->set_power(pdata, 1);
	}

	return ret;
}

static int omap_pwm_led_remove(struct platform_device *pdev)
{
	int ret = -EINVAL;
	struct omap_pwm_led *led = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&led->early_suspend);
#endif
#ifdef CONFIG_OMAP_PWM_LED_TYPE_DMTIMER
	ret = omap_dmtimer_pwm_led_release(led);
#endif
#ifdef CONFIG_OMAP_PWM_LED_TYPE_TWL6030
	ret = twl6030_pwm_led_release(led);
#endif

	led_classdev_unregister(&led->cdev);

	kfree(led);

	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void omap_pwm_led_early_suspend(struct early_suspend *handler)
{
	struct omap_pwm_led *led;
	struct led_classdev *cdev;

	led = container_of(handler, struct omap_pwm_led, early_suspend);
	cdev = &led->cdev;

	if (led->pdata->set_power != NULL)
		led->pdata->set_power(led->pdata, 0);

	if (led->pwm_power_off)
		led->pwm_power_off(led);

	led_classdev_suspend(&led->cdev);

#ifdef CONFIG_ARCH_OMAP4
	//dpll_cascading_blocker_release(cdev->dev);
#endif
}

static void omap_pwm_led_late_resume(struct early_suspend *handler)
{
	struct omap_pwm_led *led;
	struct led_classdev *cdev;

	led = container_of(handler, struct omap_pwm_led, early_suspend);
	cdev = &led->cdev;

#ifdef CONFIG_ARCH_OMAP4
	//dpll_cascading_blocker_hold(cdev->dev);
#endif

	if (led->pwm_power_on)
		led->pwm_power_on(led);

	if (led->pdata->set_power != NULL)
		led->pdata->set_power(led->pdata, 1);

	led_classdev_resume(&led->cdev);
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int omap_pwm_led_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct omap_pwm_led *led = platform_get_drvdata(pdev);

	led_classdev_suspend(&led->cdev);
	return 0;
}

static int omap_pwm_led_resume(struct platform_device *pdev)
{
	struct omap_pwm_led *led = platform_get_drvdata(pdev);

	led_classdev_resume(&led->cdev);
	return 0;
}
#else
#define omap_pwm_led_suspend NULL
#define omap_pwm_led_resume NULL
#endif

static struct platform_driver omap_pwm_led_driver = {
	.probe		= omap_pwm_led_probe,
	.remove		= omap_pwm_led_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= omap_pwm_led_suspend,
	.resume		= omap_pwm_led_resume,
#endif
	.driver		= {
		.name		= "omap_pwm_led",
		.owner		= THIS_MODULE,
	},
};

static int __init omap_pwm_led_init(void)
{
	return platform_driver_register(&omap_pwm_led_driver);
}

static void __exit omap_pwm_led_exit(void)
{
	platform_driver_unregister(&omap_pwm_led_driver);
}

module_init(omap_pwm_led_init);
module_exit(omap_pwm_led_exit);

MODULE_AUTHOR("Atanas Tulbenski <atulbenski@mm-sol.com>");
MODULE_DESCRIPTION("OMAP PWM LED driver");
MODULE_LICENSE("GPL");
