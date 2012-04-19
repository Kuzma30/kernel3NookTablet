/* drivers/leds/leds-omap_pwm.c
 *
 * Driver to blink LEDs using OMAP PWM timers
 *
 * Copyright (C) 2006 Nokia Corporation
 * Author: Timo Teras
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
<<<<<<< HEAD
=======
#undef DEBUG
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
<<<<<<< HEAD
#include <linux/earlysuspend.h>
=======
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/ctype.h>
#include <linux/sched.h>
<<<<<<< HEAD
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <plat/board.h>
#include <plat/dmtimer.h>
#include <plat/omap-pm.h>
#include <linux/pm_runtime.h>
#include "leds-omap-pwm.h"

/* 38400000 / (1 << (COUNTER_DEVIDER + 1)) */
#define COUNTER_DEVIDER		5 /* 600000 Hz counter in freq */
#define COUNTER_LOAD_VAL	(0xFFFFFFFF - 4687 - 4) /* 128 Hz PWM out freq */
#define COUNTER_TO_MATCH_GUARD	80

#define TIMER_INT_FLAGS		(OMAP_TIMER_INT_MATCH | \
				OMAP_TIMER_INT_OVERFLOW)

#ifdef CONFIG_HAS_EARLYSUSPEND
static void omap_pwm_led_early_suspend(struct early_suspend *handler);
static void omap_pwm_led_late_resume(struct early_suspend *handler);
#endif

struct omap_pwm_led {
	struct led_classdev cdev;
	struct work_struct work;
=======
#include <linux/clk.h>
#include <asm/delay.h>
#include <plat/board.h>
#include <plat/dmtimer.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define MAX_GPTIMER_ID		12

struct omap_pwm_led {
	struct led_classdev cdev;
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	struct omap_pwm_led_platform_data *pdata;
	struct omap_dm_timer *intensity_timer;
	struct omap_dm_timer *blink_timer;
	int powered;
	unsigned int on_period, off_period;
	enum led_brightness brightness;
<<<<<<< HEAD
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend  early_suspend;
#endif
	atomic_t cached_match_val;
};

static inline unsigned int get_match_val(unsigned char index)
{
	return match_data[index];
}

=======
};

>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
static inline struct omap_pwm_led *pdev_to_omap_pwm_led(struct platform_device *pdev)
{
	return platform_get_drvdata(pdev);
}

static inline struct omap_pwm_led *cdev_to_omap_pwm_led(struct led_classdev *led_cdev)
{
	return container_of(led_cdev, struct omap_pwm_led, cdev);
}

<<<<<<< HEAD
static inline struct omap_pwm_led *work_to_omap_pwm_led(struct work_struct *work)
{
	return container_of(work, struct omap_pwm_led, work);
}

static void omap_pwm_led_set_blink(struct omap_pwm_led *led)
{
	int def_on = 1;

	if (led->pdata)
		def_on = led->pdata->def_on;

=======
static void omap_pwm_led_set_blink(struct omap_pwm_led *led)
{
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	if (!led->powered)
		return;

	if (led->on_period != 0 && led->off_period != 0) {
		unsigned long load_reg, cmp_reg;

		load_reg = 32768 * (led->on_period + led->off_period) / 1000;
		cmp_reg = 32768 * led->on_period / 1000;

		omap_dm_timer_stop(led->blink_timer);
		omap_dm_timer_set_load(led->blink_timer, 1, -load_reg);
		omap_dm_timer_set_match(led->blink_timer, 1, -cmp_reg);
<<<<<<< HEAD
		omap_dm_timer_set_pwm(led->blink_timer,
					  def_on, 1,
=======
		omap_dm_timer_set_pwm(led->blink_timer, 1, 1,
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_write_counter(led->blink_timer, -2);
		omap_dm_timer_start(led->blink_timer);
	} else {
<<<<<<< HEAD
		omap_dm_timer_set_pwm(led->blink_timer,
					  def_on, 1,
=======
		omap_dm_timer_set_pwm(led->blink_timer, 1, 1,
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(led->blink_timer);
	}
}

<<<<<<< HEAD
static inline void omap_pwm_set_match(struct omap_dm_timer *timer,
				      unsigned int val)
{
	omap_dm_timer_set_match(timer, 1, val);
	omap_dm_timer_set_int_disable(timer, TIMER_INT_FLAGS);
}

static irqreturn_t intensity_timer_match_interrupt(int irq, void *arg)
{
	struct omap_pwm_led	*led;
	struct omap_dm_timer	*timer;
	unsigned int		counter;
	unsigned int		match_val;
	unsigned int		current_match_val;
	unsigned int		status;

	led   = (struct omap_pwm_led *) arg;
	timer = (struct omap_dm_timer *) led->intensity_timer;
	match_val = atomic_read(&led->cached_match_val);

	/* disable interrupts */
	local_irq_disable();
		
	/* get int status */
	status = omap_dm_timer_read_status(timer);

	/* get current match value */
	current_match_val = omap_dm_timer_get_match(timer);

	/* We must update match register only in case:
	* - new match value is bigger than old one
	* - when old match value is bigger than new one, current
	* counter value must be bigger than old match value or
	* lower than new match value.
	*
	* If this conditions are not met, we will write a match value,
	* at moment when match event doesn't trigered yet and the new
	* match value is lower than counter. This will result in missing
	* the match event for this period.
	*/
	
	counter = omap_dm_timer_read_counter(timer);
	if ((counter + COUNTER_TO_MATCH_GUARD) < match_val)
		omap_pwm_set_match(timer, match_val);
	else if (counter > current_match_val)
		omap_pwm_set_match(timer, match_val);

	/* acknowledge interrupts */
	omap_dm_timer_write_status(timer, status);

	/* enable interrupts */
	local_irq_enable();
	
	return IRQ_HANDLED;
}

static void omap_pwm_led_set_pwm_cycle(struct omap_pwm_led *led, int cycle)
{
	struct omap_dm_timer	*timer;
	unsigned int		match_val;
	unsigned int		current_match_val;

	timer = (struct omap_dm_timer *) led->intensity_timer;

	current_match_val = atomic_read(&led->cached_match_val);
	match_val = get_match_val(cycle);

	if (current_match_val < match_val) {
		omap_dm_timer_set_match(timer, 1, match_val);
		atomic_set(&led->cached_match_val, match_val);
	} else {
		atomic_set(&led->cached_match_val, match_val);
		omap_dm_timer_set_int_enable(timer, TIMER_INT_FLAGS);
	}
=======
static void omap_pwm_led_pad_enable(struct omap_pwm_led *led)
{
	if (led->pdata->set_pad)
		led->pdata->set_pad(led->pdata, 1);
}

static void omap_pwm_led_pad_disable(struct omap_pwm_led *led)
{
	if (led->pdata->set_pad)
		led->pdata->set_pad(led->pdata, 0);
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
}

static void omap_pwm_led_power_on(struct omap_pwm_led *led)
{
<<<<<<< HEAD
	int def_on = 1;
	unsigned int timerval;
	int err;

	if (led->pdata)
		def_on = led->pdata->def_on;

=======
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	if (led->powered)
		return;
	led->powered = 1;

<<<<<<< HEAD
	/* Select clock source */
	omap_dm_timer_enable(led->intensity_timer);
	omap_dm_timer_set_source(led->intensity_timer, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_set_prescaler(led->intensity_timer, COUNTER_DEVIDER);
	/* Enable PWM timers */
	if (led->blink_timer != NULL) {
		omap_dm_timer_enable(led->blink_timer);
		omap_dm_timer_set_source(led->blink_timer,
					 OMAP_TIMER_SRC_32_KHZ);
		omap_pwm_led_set_blink(led);
	}

	omap_dm_timer_set_pwm(led->intensity_timer, def_on ? 0 : 1, 1,
			      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

	omap_dm_timer_set_load(led->intensity_timer, 1, COUNTER_LOAD_VAL);
	omap_dm_timer_start(led->intensity_timer);
	omap_pwm_led_set_pwm_cycle(led, 0);

	timerval = omap_dm_timer_read_counter(led->intensity_timer);
	if (timerval < COUNTER_LOAD_VAL)
		omap_dm_timer_write_counter(led->intensity_timer, -2);
=======
	pr_debug("%s: brightness: %i\n", 
			__func__, led->brightness);
	
	/* Select clock */
	omap_dm_timer_set_source(led->intensity_timer, OMAP_TIMER_SRC_SYS_CLK);
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel

	/* Turn voltage on */
	if (led->pdata->set_power != NULL)
		led->pdata->set_power(led->pdata, 1);

<<<<<<< HEAD
	/* register timer match and overflow interrupts */
	err = request_irq(omap_dm_timer_get_irq(led->intensity_timer),
			  intensity_timer_match_interrupt,
			  IRQF_DISABLED, "led intensity timer", (void *)led);
	if (err) {
		printk(KERN_ERR "%s(%s) : unable to get gptimer%d IRQ\n",
		       __func__, __FILE__, led->pdata->intensity_timer);
=======
	/* explicitly enable the timer, saves some SAR later */
	omap_dm_timer_enable(led->intensity_timer);
	
	/* Enable PWM timers */
	if (led->blink_timer != NULL) {
		omap_dm_timer_set_source(led->blink_timer,
					 OMAP_TIMER_SRC_32_KHZ);
		omap_pwm_led_set_blink(led);
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	}
}

static void omap_pwm_led_power_off(struct omap_pwm_led *led)
{
<<<<<<< HEAD
	int def_on = 1;

=======
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	if (!led->powered)
		return;
	led->powered = 0;

<<<<<<< HEAD
	if (led->pdata->set_power != NULL)
		led->pdata->set_power(led->pdata, 0);

	/* disable timer match interrupt */
	omap_dm_timer_set_int_disable(led->intensity_timer,
				      OMAP_TIMER_INT_MATCH);
	free_irq(omap_dm_timer_get_irq(led->intensity_timer), (void *)led);
	if (led->pdata)
		def_on = led->pdata->def_on;

	/* Everything off */
	omap_dm_timer_set_pwm(led->intensity_timer,
				  def_on ? 0 : 1, 1,
				  OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_stop(led->intensity_timer);
	omap_dm_timer_disable(led->intensity_timer);

	if (led->blink_timer != NULL) {
		omap_dm_timer_stop(led->blink_timer);
		omap_dm_timer_disable(led->blink_timer);
	}

	atomic_set(&led->cached_match_val, 0);
}

static void omap_pwm_led_set(struct led_classdev *led_cdev,
			     enum led_brightness value)
{
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);

	if (value != led->brightness) {
		led->brightness = value;
		schedule_work(&led->work);
	}
}

static void omap_pwm_led_work(struct work_struct *work)
{
	struct omap_pwm_led *led = work_to_omap_pwm_led(work);

	if (led->brightness != LED_OFF) {
		omap_pwm_led_power_on(led);
		omap_pwm_led_set_pwm_cycle(led, led->brightness);
	} else {
		omap_pwm_led_set_pwm_cycle(led, led->brightness);
		omap_pwm_led_power_off(led);
=======
	pr_debug("%s: brightness: %i\n", 
			__func__, led->brightness);
	
	if (led->pdata->set_power != NULL)
		led->pdata->set_power(led->pdata, 0);

	/* Everything off */
	omap_dm_timer_stop(led->intensity_timer);

	if (led->blink_timer != NULL)
		omap_dm_timer_stop(led->blink_timer);
}

static void pwm_set_speed(struct omap_dm_timer *gpt,
		int frequency, int duty_cycle)
{
	u32 val;
	u32 period;
	struct clk *timer_fclk;

	/* and you will have an overflow in 1 sec         */
	/* so,                              */
	/* freq_timer     -> 1s             */
	/* carrier_period -> 1/carrier_freq */
	/* => carrier_period = freq_timer/carrier_freq */

	timer_fclk = omap_dm_timer_get_fclk(gpt);
	period = clk_get_rate(timer_fclk) / frequency;

	val = 0xFFFFFFFF+1-period;
	omap_dm_timer_set_load(gpt, 1, val);

	val = 0xFFFFFFFF+1-(period*duty_cycle/256);
	omap_dm_timer_set_match(gpt, 1, val);

	/* assume overflow first: no toogle if first trig is match */
	omap_dm_timer_write_counter(gpt, 0xFFFFFFFE);
}

static void omap_pwm_led_set_pwm_cycle(struct omap_pwm_led *led, int cycle)
{
	int pwm_frequency = 10000;
	int def_on;
	
	pr_debug("%s: cycle: %i\n", 
			__func__, cycle);
	
	if (led->pdata->bkl_max)
		cycle = ( (cycle * led->pdata->bkl_max ) / 255);
	
	if (cycle < led->pdata->bkl_min)
		cycle = led->pdata->bkl_min;

	if (led->pdata->bkl_freq)
		pwm_frequency = led->pdata->bkl_freq;

	if (cycle != LED_FULL)
		def_on = led->pdata->invert ? 1:0;
	else
		def_on = led->pdata->invert ? 0:1;
	
	omap_dm_timer_set_pwm(led->intensity_timer, def_on, 1,
		      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

	if (cycle != LED_FULL) {
		pwm_set_speed(led->intensity_timer, pwm_frequency, 256-cycle);
		omap_dm_timer_start(led->intensity_timer);
	} else
		omap_dm_timer_stop(led->intensity_timer);
}

static void omap_pwm_led_set(struct led_classdev *led_cdev,
			     enum led_brightness value)
{
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);

	pr_debug("%s: brightness: %i\n", __func__, value);

	if (led->brightness != value) {
		if (led->brightness == LED_OFF ||
			led_cdev->flags & LED_SUSPENDED) {
			/* LED currently OFF */
			omap_pwm_led_power_on(led);
			if (value < led->pdata->bkl_min*2) {
				// some backlight stepup can't start without medium value during variable time
				omap_pwm_led_set_pwm_cycle(led, led->pdata->bkl_min*3);
				omap_pwm_led_pad_enable(led);
				msleep(50);
				omap_pwm_led_set_pwm_cycle(led, value);
			
			} else {
				omap_pwm_led_set_pwm_cycle(led, value);
				omap_pwm_led_pad_enable(led);
			}
		} else
			/* just set the new cycle */
			omap_pwm_led_set_pwm_cycle(led, value);
			
		if (value == LED_OFF) {
			/* LED now suspended */
			omap_pwm_led_pad_disable(led);
			omap_pwm_led_set_pwm_cycle(led, value);
			omap_pwm_led_power_off(led);
		}
		led->brightness = value;
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	}
}

static ssize_t omap_pwm_led_on_period_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);

	return sprintf(buf, "%u\n", led->on_period) + 1;
}

static ssize_t omap_pwm_led_on_period_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);
	int ret = -EINVAL;
	unsigned long val;
	char *after;
	size_t count;

	val = simple_strtoul(buf, &after, 10);
	count = after - buf;
	if (*after && isspace(*after))
		count++;

	if (count == size) {
		led->on_period = val;
		omap_pwm_led_set_blink(led);
		ret = count;
	}

	return ret;
}

static ssize_t omap_pwm_led_off_period_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);

	return sprintf(buf, "%u\n", led->off_period) + 1;
}

static ssize_t omap_pwm_led_off_period_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);
	int ret = -EINVAL;
	unsigned long val;
	char *after;
	size_t count;

	val = simple_strtoul(buf, &after, 10);
	count = after - buf;
	if (*after && isspace(*after))
		count++;

	if (count == size) {
		led->off_period = val;
		omap_pwm_led_set_blink(led);
		ret = count;
	}

	return ret;
}

static DEVICE_ATTR(on_period, 0644, omap_pwm_led_on_period_show,
				omap_pwm_led_on_period_store);
static DEVICE_ATTR(off_period, 0644, omap_pwm_led_off_period_show,
				omap_pwm_led_off_period_store);

static int omap_pwm_led_probe(struct platform_device *pdev)
{
	struct omap_pwm_led_platform_data *pdata = pdev->dev.platform_data;
	struct omap_pwm_led *led;
	int ret;

<<<<<<< HEAD
=======
	if (pdata->intensity_timer < 1 || pdata->intensity_timer > MAX_GPTIMER_ID)
		return -EINVAL;

	if (pdata->blink_timer != 0 || pdata->blink_timer > MAX_GPTIMER_ID)
		return -EINVAL;

>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	led = kzalloc(sizeof(struct omap_pwm_led), GFP_KERNEL);
	if (led == NULL) {
		dev_err(&pdev->dev, "No memory for device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, led);
	led->cdev.brightness_set = omap_pwm_led_set;
<<<<<<< HEAD
	led->cdev.default_trigger = NULL;
	led->cdev.name = pdata->name;
	led->pdata = pdata;
	led->brightness = pdata->def_brightness;
	INIT_WORK(&led->work, omap_pwm_led_work);
=======
	led->cdev.default_trigger = pdata->default_trigger;
	led->cdev.name = pdata->name;
	led->pdata = pdata;
	led->brightness = LED_OFF;
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel

	dev_info(&pdev->dev, "OMAP PWM LED (%s) at GP timer %d/%d\n",
		 pdata->name, pdata->intensity_timer, pdata->blink_timer);

<<<<<<< HEAD
	if (pdata->def_brightness)
		led->cdev.brightness = pdata->def_brightness;

=======
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	/* register our new led device */
	ret = led_classdev_register(&pdev->dev, &led->cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "led_classdev_register failed\n");
		goto error_classdev;
	}

	/* get related dm timers */
	led->intensity_timer = omap_dm_timer_request_specific(pdata->intensity_timer);
	if (led->intensity_timer == NULL) {
		dev_err(&pdev->dev, "failed to request intensity pwm timer\n");
		ret = -ENODEV;
		goto error_intensity;
	}
<<<<<<< HEAD
	omap_dm_timer_disable(led->intensity_timer);
=======

	if (led->pdata->invert)
		omap_dm_timer_set_pwm(led->intensity_timer, 1, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel

	if (pdata->blink_timer != 0) {
		led->blink_timer = omap_dm_timer_request_specific(pdata->blink_timer);
		if (led->blink_timer == NULL) {
<<<<<<< HEAD
			dev_err(&pdev->dev,
				"failed to request blinking pwm timer\n");
			ret = -ENODEV;
			goto error_blink1;
		}
		omap_dm_timer_disable(led->blink_timer);

		ret = device_create_file(led->cdev.dev,
					       &dev_attr_on_period);
		if (ret)
=======
			dev_err(&pdev->dev, "failed to request blinking pwm timer\n");
			ret = -ENODEV;
			goto error_blink1;
		}
		ret = device_create_file(led->cdev.dev,
					       &dev_attr_on_period);
		if(ret)
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
			goto error_blink2;

		ret = device_create_file(led->cdev.dev,
					&dev_attr_off_period);
<<<<<<< HEAD
		if (ret)
			goto error_blink3;

	}
	if (led->brightness) {
		schedule_work(&led->work);
	} else {
		omap_pwm_led_power_on(led);
		omap_pwm_led_power_off(led);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	led->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	led->early_suspend.suspend = omap_pwm_led_early_suspend;
	led->early_suspend.resume  = omap_pwm_led_late_resume;
	register_early_suspend(&led->early_suspend);
#endif
=======
		if(ret)
			goto error_blink3;

	}

>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	return 0;

error_blink3:
	device_remove_file(led->cdev.dev,
				 &dev_attr_on_period);
error_blink2:
	dev_err(&pdev->dev, "failed to create device file(s)\n");
error_blink1:
	omap_dm_timer_free(led->intensity_timer);
error_intensity:
	led_classdev_unregister(&led->cdev);
error_classdev:
	kfree(led);
	return ret;
}

static int omap_pwm_led_remove(struct platform_device *pdev)
{
	struct omap_pwm_led *led = pdev_to_omap_pwm_led(pdev);

<<<<<<< HEAD
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&led->early_suspend);
#endif

=======
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	device_remove_file(led->cdev.dev,
				 &dev_attr_on_period);
	device_remove_file(led->cdev.dev,
				 &dev_attr_off_period);
	led_classdev_unregister(&led->cdev);

	omap_pwm_led_set(&led->cdev, LED_OFF);
	if (led->blink_timer != NULL)
		omap_dm_timer_free(led->blink_timer);
	omap_dm_timer_free(led->intensity_timer);
	kfree(led);

	return 0;
}

<<<<<<< HEAD
#ifdef CONFIG_HAS_EARLYSUSPEND
static void omap_pwm_led_early_suspend(struct early_suspend *handler)
{
	struct omap_pwm_led *led;

	led = container_of(handler, struct omap_pwm_led, early_suspend);

	flush_work(&led->work);
	/* Make sure the led is switched OFF NOW in this current thread!!! */
	omap_pwm_led_power_off(led);

	led_classdev_suspend(&led->cdev);
}

static void omap_pwm_led_late_resume(struct early_suspend *handler)
{
	struct omap_pwm_led *led;

	led = container_of(handler, struct omap_pwm_led, early_suspend);

	led_classdev_resume(&led->cdev);
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int omap_pwm_led_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct omap_pwm_led *led = pdev_to_omap_pwm_led(pdev);

=======
#ifdef CONFIG_PM
static int omap_pwm_led_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct omap_pwm_led *led = pdev_to_omap_pwm_led(pdev);

	pr_debug("%s: brightness: %i\n", __func__,
			led->brightness);
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
	led_classdev_suspend(&led->cdev);
	return 0;
}

static int omap_pwm_led_resume(struct platform_device *pdev)
{
	struct omap_pwm_led *led = pdev_to_omap_pwm_led(pdev);

<<<<<<< HEAD
=======
	pr_debug("%s: brightness: %i\n", __func__,
			led->brightness);
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
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
<<<<<<< HEAD
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= omap_pwm_led_suspend,
	.resume		= omap_pwm_led_resume,
#endif
=======
	.suspend	= omap_pwm_led_suspend,
	.resume		= omap_pwm_led_resume,
>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
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

MODULE_AUTHOR("Timo Teras");
MODULE_DESCRIPTION("OMAP PWM LED driver");
MODULE_LICENSE("GPL");
<<<<<<< HEAD
=======

>>>>>>> 5e49fc5... new backlight driver from Archos Gen9 kernel
