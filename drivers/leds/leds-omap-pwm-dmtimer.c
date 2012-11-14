/* drivers/leds/leds-omap-pwm-dmtimer.c
*
* Driver to control backlight LEDs using OMAP GPTIMER PWM timers.
*
* Copyright (C) 2011 MM Solutions
* Author: Atanas Tulbenski <atulbenski@mm-sol.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <plat/dmtimer.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <plat/board.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include "leds-omap-pwm.h"
#include "leds-omap-dmtimer-pwm-lut.h"

/* 600000 Hz counter input frequency */
#define COUNTER_DEVIDER		5
/* 128 Hz PWM output frequency */
#define COUNTER_LOAD_VAL	(0xFFFFFFFF - 4687 - 4)
#define COUNTER_TO_MATCH_GUARD	80

#define TIMER_INT_FLAGS		(OMAP_TIMER_INT_MATCH | \
				OMAP_TIMER_INT_OVERFLOW)

struct omap_pwm_dmtimer_context {
	atomic_t		cached_match_val;
	struct omap_dm_timer	*intensity_timer;
	struct semaphore	if_lock;
	atomic_t		irq_done;
};

/*
  Get MATCH value from lookup table.
*/
static inline unsigned int get_match_val(unsigned char index)
{
	return match_data[index];
}

static inline void omap_pwm_set_match(struct omap_dm_timer *timer,
				      unsigned int val)
{
	omap_dm_timer_enable(timer);
	omap_dm_timer_set_match(timer, 1, val);
	omap_dm_timer_enable(timer);
	omap_dm_timer_set_int_disable(timer, TIMER_INT_FLAGS);
	omap_dm_timer_enable(timer);
}

static irqreturn_t intensity_timer_match_interrupt(int irq, void *arg)
{
	struct omap_pwm_led		*led;
	struct omap_pwm_dmtimer_context *ctx;
	struct omap_dm_timer		*timer;
	unsigned int			counter;
	unsigned int			match_val;
	unsigned int			current_match_val;
	unsigned int			status;
	unsigned int			updated;

	led	  = (struct omap_pwm_led *) arg;
	ctx	  = (struct omap_pwm_dmtimer_context *) led->ctx;
	timer	  = (struct omap_dm_timer *) ctx->intensity_timer;
	match_val = atomic_read(&ctx->cached_match_val);
	updated	  = 0;

	/* get int status */
	omap_dm_timer_enable(timer);
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
	omap_dm_timer_enable(timer);
	counter = omap_dm_timer_read_counter(timer);

	if ((counter + COUNTER_TO_MATCH_GUARD) < match_val) {
		omap_pwm_set_match(timer, match_val);
		updated = 1;
	} else if (counter > current_match_val) {
		omap_pwm_set_match(timer, match_val);
		updated = 1;
	}

	/* acknowledge interrupts */
	omap_dm_timer_write_status(timer, status);

	if (updated)
		atomic_set(&ctx->irq_done, 1);

	return IRQ_HANDLED;
}

static void omap_pwm_led_set_pwm_cycle(struct omap_pwm_led *led, int cycle)
{
	struct omap_pwm_dmtimer_context *ctx;
	struct omap_dm_timer		*timer;
	unsigned int			match_val;
	unsigned int			current_match_val;

	ctx   = (struct omap_pwm_dmtimer_context *) led->ctx;
	timer = (struct omap_dm_timer *) ctx->intensity_timer;

	current_match_val = atomic_read(&ctx->cached_match_val);

	match_val = get_match_val(cycle);

	if (current_match_val < match_val) {
		omap_dm_timer_set_match(timer, 1, match_val);
		omap_dm_timer_enable(timer);
		atomic_set(&ctx->cached_match_val, match_val);
	} else {
		atomic_set(&ctx->cached_match_val, match_val);
		omap_dm_timer_set_int_enable(timer, TIMER_INT_FLAGS);
		omap_dm_timer_enable(timer);

		while (!atomic_read(&ctx->irq_done))
			schedule();

		atomic_set(&ctx->irq_done, 0);
	}
}

static void omap_pwm_led_power_on(struct omap_pwm_led *led)
{
	int		def_on = 1;
	unsigned int	timerval;
	int		err;
	struct omap_pwm_dmtimer_context *ctx;

	ctx = (struct omap_pwm_dmtimer_context *) led->ctx;

	if (led->pdata)
		def_on = led->pdata->def_on;

	/* Select clock source */
	omap_dm_timer_enable(ctx->intensity_timer);
	omap_dm_timer_set_source(ctx->intensity_timer, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_set_prescaler(ctx->intensity_timer, COUNTER_DEVIDER);

	omap_dm_timer_set_pwm(ctx->intensity_timer, def_on ? 0 : 1, 1,
			      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

	omap_dm_timer_set_load(ctx->intensity_timer, 1, COUNTER_LOAD_VAL);
	omap_dm_timer_start(ctx->intensity_timer);
	atomic_set(&ctx->irq_done, 0);

	atomic_set(&ctx->cached_match_val, 0);
	omap_pwm_led_set_pwm_cycle(led, led->brightness);

	omap_dm_timer_enable(ctx->intensity_timer);
	timerval = omap_dm_timer_read_counter(ctx->intensity_timer);
	if (timerval < COUNTER_LOAD_VAL)
		omap_dm_timer_write_counter(ctx->intensity_timer, -2);

	/* register timer match and overflow interrupts */
	err = request_irq(omap_dm_timer_get_irq(ctx->intensity_timer),
			intensity_timer_match_interrupt,
			IRQF_DISABLED, "led intensity timer", (void *)led);
	if (err) {
		printk(KERN_ERR "%s(%s) : unable to get gptimer%d IRQ\n",
			__func__, __FILE__, led->pdata->pwm_module_id);
	}
}

static void omap_pwm_led_power_off(struct omap_pwm_led *led)
{
	int def_on = 1;
	struct omap_pwm_dmtimer_context *ctx;

	ctx = (struct omap_pwm_dmtimer_context *) led->ctx;

	/* disable timer match interrupt */
	omap_dm_timer_set_int_disable(ctx->intensity_timer,
					OMAP_TIMER_INT_MATCH);
	free_irq(omap_dm_timer_get_irq(ctx->intensity_timer), (void *)led);
	if (led->pdata)
		def_on = led->pdata->def_on;

	/* Everything off */
	omap_dm_timer_set_pwm(ctx->intensity_timer,
			def_on ? 0 : 1, 1,
			OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_stop(ctx->intensity_timer);
	omap_dm_timer_disable(ctx->intensity_timer);
}

static void omap_dmtimer_pwm_led_power_off(struct omap_pwm_led *led)
{
	struct omap_pwm_dmtimer_context *ctx;

	ctx = (struct omap_pwm_dmtimer_context *) led->ctx;

	down(&ctx->if_lock);

	if (led->powered) {
		omap_pwm_led_power_off(led);
		led->powered = 0;
	}

	up(&ctx->if_lock);
}

static void omap_dmtimer_pwm_led_power_on(struct omap_pwm_led *led)
{
	struct omap_pwm_dmtimer_context *ctx;

	ctx = (struct omap_pwm_dmtimer_context *) led->ctx;

	down(&ctx->if_lock);

	if (!led->powered) {
		omap_pwm_led_power_on(led);
		led->powered = 1;
	}

	up(&ctx->if_lock);
}

static void omap_dmtimer_pwm_led_set(struct omap_pwm_led *led, int cycle)
{
	struct omap_pwm_dmtimer_context *ctx;

	ctx = (struct omap_pwm_dmtimer_context *) led->ctx;

	down(&ctx->if_lock);

	if (led->powered) {
		if (led->brightness != cycle) {
			omap_pwm_led_set_pwm_cycle(led, cycle);
			led->brightness = cycle;
		}
	}

	up(&ctx->if_lock);
}

int omap_dmtimer_pwm_led_init(struct omap_pwm_led *led)
{
	int				ret = 0;
	int				module_id;
	struct omap_pwm_dmtimer_context	*ctx;

	module_id = led->pdata->pwm_module_id;
	ctx = kzalloc(sizeof(struct omap_pwm_dmtimer_context), GFP_KERNEL);
	if (ctx == NULL) {
		dev_err(&led->pdev->dev, "No memory for device\n");
		ret = -ENOMEM;
		return ret;
	}

	led->ctx = ctx;
	/* get related dm timer */
	ctx->intensity_timer = omap_dm_timer_request_specific(module_id);
	if (ctx->intensity_timer == NULL) {
		dev_err(&led->pdev->dev, "Unable to allocate DMTIMER%d\n",
			module_id);
		kfree(ctx);
		ret = -ENODEV;
		return ret;
	}

	omap_dm_timer_disable(ctx->intensity_timer);

	omap_pwm_led_power_on(led);
	led->powered = 1;

	sema_init(&ctx->if_lock, 1);

	led->pwm_power_on  = omap_dmtimer_pwm_led_power_on;
	led->pwm_power_off = omap_dmtimer_pwm_led_power_off;
	led->pwm_set_cycle = omap_dmtimer_pwm_led_set;

	return ret;
}

int omap_dmtimer_pwm_led_release(struct omap_pwm_led *led)
{
	struct omap_pwm_dmtimer_context *ctx;

	ctx = (struct omap_pwm_dmtimer_context *)led->ctx;

	omap_pwm_led_power_off(led);
	led->powered = 0;

	omap_dm_timer_free(ctx->intensity_timer);

	led->pwm_power_on  = NULL;
	led->pwm_power_off = NULL;
	led->pwm_set_cycle = NULL;

	kfree(ctx);
	led->ctx = NULL;

	return 0;
}

