
/*
 * arch/arm/mach-omap2/board-acclaim-touch.c
 *
 * Based on arch/arm/mach-omap2/board-44xx-tablet-touch.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/input/touch_platform.h>

#include <plat/i2c.h>

#include "board-acclaim.h"
#include "mux.h"

#define FT5x06_I2C_SLAVEADDRESS  	(0x70 >> 1)
#define OMAP_FT5x06_GPIO         	37
#define OMAP_FT5x06_RESET_GPIO   	39

int ft5x06_dev_init(int resource)
{
	if (resource){
		omap_mux_init_signal("gpmc_ad13.gpio_37", OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
		omap_mux_init_signal("gpmc_ad15.gpio_39", OMAP_PIN_OUTPUT );

		if (gpio_request(OMAP_FT5x06_RESET_GPIO, "ft5x06_reset") < 0){
			printk(KERN_ERR "can't get ft5x06 xreset GPIO\n");
			return -1;
		}

		if (gpio_request(OMAP_FT5x06_GPIO, "ft5x06_touch") < 0) {
			printk(KERN_ERR "can't get ft5x06 interrupt GPIO\n");
			return -1;
		}

		gpio_direction_input(OMAP_FT5x06_GPIO);
	} else {
		gpio_free(OMAP_FT5x06_GPIO);
		gpio_free(OMAP_FT5x06_RESET_GPIO);
	}

	return 0;
}

static void ft5x06_platform_suspend(void)
{
	printk("----------------ft5x06 platform suspend-----------\n");
	//omap_mux_init_signal("gpmc_ad13.gpio_37", OMAP_PIN_INPUT );
}

static void ft5x06_platform_resume(void)
{
	printk("-----------------ft5x06 platform resume-------------\n");
	//omap_mux_init_signal("gpmc_ad13.gpio_37", OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
}

static struct ft5x06_platform_data ft5x06_platform_data = {
	.maxx = 600,
	.maxy = 1024,
	.flags = REVERSE_Y_FLAG,
	.reset_gpio = OMAP_FT5x06_RESET_GPIO,
	.use_st = FT_USE_ST,
	.use_mt = FT_USE_MT,
	.use_trk_id = 1,
	.use_sleep = FT_USE_SLEEP,
	.use_gestures = 1,
	.platform_suspend = ft5x06_platform_suspend,
	.platform_resume = ft5x06_platform_resume,
};
static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
	{
 		I2C_BOARD_INFO(FT_I2C_NAME, FT5x06_I2C_SLAVEADDRESS),
 		.platform_data = &ft5x06_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_FT5x06_GPIO),
	},
};

int __init acclaim_touch_init(void)
{
	gpio_request(OMAP4_TOUCH_IRQ_1, "atmel touch irq");
	gpio_direction_input(OMAP4_TOUCH_IRQ_1);
	omap_mux_init_signal("gpmc_ad11.gpio_35",
			OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_MUX_MODE3 |
			OMAP_INPUT_EN | OMAP_PIN_OFF_INPUT_PULLUP);

	i2c_register_board_info(4, omap4xx_i2c_bus4_touch_info,
		ARRAY_SIZE(omap4xx_i2c_bus4_touch_info));

	return 0;
}
