
/*
 * Board support file for BN acclaim.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-4430sdp.c
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
#include <linux/moduleparam.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/hwspinlock.h>
#include <linux/bootmem.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/bq2415x.h>
#include <linux/gpio_keys.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6130x.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/omapfb.h>
#include <linux/reboot.h>
#include <linux/twl6040-vib.h>
#include <linux/wl12xx.h>
#include <linux/memblock.h>
#include <linux/mfd/twl6040-codec.h>

#include <linux/input/ft5x06.h>

#ifdef CONFIG_INPUT_KXTJ9
#include <linux/input/kxtj9.h>
#endif

#ifdef CONFIG_INPUT_KXTF9
#include <linux/input/kxtf9.h>
#endif

#include <linux/power/max17042.h>
#include <linux/power/max8903.h>

#include <mach/board-nooktablet.h>
#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/lpddr2-samsung.h>
#include <mach/dmm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/omap-serial.h>
#include <plat/remoteproc.h>
#include <video/omapdss.h>
#include <plat/vram.h>
#include <plat/omap-pm.h>
#include <plat/android-display.h>
#include <mach/omap4_ion.h>
#include "omap_ram_console.h"
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "common-board-devices.h"
#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include <linux/skbuff.h>
#include <plat/omap-serial.h>

#ifdef CONFIG_INPUT_KXTF9
#define KXTF9_DEVICE_ID                 "kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS         0x0F
#define KXTF9_GPIO_FOR_PWR              34
#endif

#ifdef CONFIG_INPUT_KXTJ9
#define KXTJ9_DEVICE_ID                 "kxtj9"
#define KXTJ9_I2C_SLAVE_ADDRESS         0x0F
#define KXTJ9_GPIO_FOR_PWR              34
#endif

#define CONFIG_SERIAL_OMAP_IDLE_TIMEOUT 5

#define GPIO_WIFI_PWEN                  114
#define GPIO_WIFI_PMENA			118
#define GPIO_WIFI_IRQ			115

#define FT5x06_I2C_SLAVEADDRESS  	(0x70 >> 1)
#define OMAP_FT5x06_GPIO         	37 /*99*/
#define OMAP_FT5x06_RESET_GPIO   	39 /*46*/

#define TWL6030_RTC_GPIO 		6
#define CONSOLE_UART			UART1

#define SAMSUNG_SDRAM                   0x1
#define ELPIDA_SDRAM                    0x3
#define HYNIX_SDRAM                     0x6

#define MAX17042_GPIO_FOR_IRQ           65
#define KXTF9_GPIO_FOR_IRQ              66
#define KXTJ9_GPIO_FOR_IRQ              66

#define LCD_RST_DELAY		        100
#define LCD_INIT_DELAY		        200

#define DEFAULT_BACKLIGHT_BRIGHTNESS	105

#define ACCLAIM_FB_RAM_SIZE             SZ_16M /* 1920×1080*4 * 2 */

void acclaim_panel_init(void);

#ifdef CONFIG_BATTERY_MAX17042
static void acclaim_max17042_dev_init(void)
{
	printk("board-nooktablet.c: max17042_dev_init ...\n");

	if (gpio_request(MAX17042_GPIO_FOR_IRQ, "max17042_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for max17042 IRQ\n");
		return;
	}

	printk("board-nooktablet.c: max17042_dev_init > "
	       "Init max17042 irq pin %d !\n", MAX17042_GPIO_FOR_IRQ);
	gpio_direction_input(MAX17042_GPIO_FOR_IRQ);
	printk("max17042 GPIO pin read %d\n", 
	       gpio_get_value(MAX17042_GPIO_FOR_IRQ));
}
#endif

#ifdef CONFIG_INPUT_KXTF9
static void kxtf9_dev_init(void)
{
	printk("board-nooktablet.c: kxtf9_dev_init ...\n");

	if (gpio_request(KXTF9_GPIO_FOR_IRQ, "kxtf9_irq") < 0)
		{
			printk("Can't get GPIO for kxtf9 IRQ\n");
			return;
		}

	printk("board-nooktablet.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n",
	       KXTF9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTF9_GPIO_FOR_IRQ);
}


struct kxtf9_platform_data kxtf9_platform_data_here = {
	.min_interval   = 1,
	.poll_interval  = 1000,

	.g_range        = KXTF9_G_8G,
	.shift_adj      = SHIFT_ADJ_2G,

	/* Map the axes from the sensor to the device */
	/* SETTINGS FOR acclaim */
	.axis_map_x     = 1,
	.axis_map_y     = 0,
	.axis_map_z     = 2,
	.negate_x       = 0,
	.negate_y       = 1,
	.negate_z       = 0,
	.data_odr_init          = ODR12_5F,
	.ctrl_reg1_init         = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init          = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.int_ctrl_init          = KXTF9_IEN,
	.tilt_timer_init        = 0x03,
	.engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init         = 0x16,
	.wuf_thresh_init        = 0x28,
	.tdt_timer_init         = 0x78,
	.tdt_h_thresh_init      = 0xFF,
	.tdt_l_thresh_init      = 0x14,
	.tdt_tap_timer_init     = 0x53,
	.tdt_total_timer_init   = 0x24,
	.tdt_latency_timer_init = 0x10,
	.tdt_window_timer_init  = 0xA0,

	.gpio = KXTF9_GPIO_FOR_IRQ,
};
#endif

#ifdef CONFIG_INPUT_KXTJ9
static void kxtj9_dev_init(void)
{
	printk("board-nooktablet.c: kxtj9_dev_init ...\n");

	if (gpio_request(KXTJ9_GPIO_FOR_IRQ, "kxtj9_irq") < 0)
		{
			printk("Can't get GPIO for kxtj9 IRQ\n");
			return;
		}

	printk("board-nooktablet.c: kxtj9_dev_init > Init kxtj9 irq pin %d !\n",
	       KXTJ9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTJ9_GPIO_FOR_IRQ);
}


struct kxtj9_platform_data kxtj9_platform_data_here = {
	.min_interval = 1,
	.g_range = KXTJ9_G_8G,
	/* Map the axes from the sensor to the device */
	/* SETTINGS FOR acclaim */
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.data_odr_init = ODR12_5F,
	.res_12bit = 1,
};
#endif

int ft5x06_dev_init(int resource)
{
	if (resource){
		omap_mux_init_signal("gpmc_ad13.gpio_37", 
				     OMAP_PIN_INPUT
				     | OMAP_PIN_OFF_WAKEUPENABLE);
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
}
 
static void ft5x06_platform_resume(void)
{
	printk("-----------------ft5x06 platform resume-------------\n");
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

#ifdef CONFIG_CHARGER_MAX8903

static struct resource max8903_gpio_resources_evt1a[] = {
	{	.name	= MAX8903_TOKEN_GPIO_CHG_EN,
		.start	= MAX8903_GPIO_CHG_EN,
		.end	= MAX8903_GPIO_CHG_EN,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_FLT,
		.start	= MAX8903_GPIO_CHG_FLT,
		.end	= MAX8903_GPIO_CHG_FLT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_IUSB,
		.start	= MAX8903_GPIO_CHG_IUSB,
		.end	= MAX8903_GPIO_CHG_IUSB,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_USUS,
		.start	= MAX8903_GPIO_CHG_USUS_EVT1A,
		.end	= MAX8903_GPIO_CHG_USUS_EVT1A,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_ILM,
		.start	= MAX8903_GPIO_CHG_ILM_EVT1A,
		.end	= MAX8903_GPIO_CHG_ILM_EVT1A,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_UOK,
		.start	= MAX8903_UOK_GPIO_FOR_IRQ,
		.end	= MAX8903_UOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_DOK,
		.start	= MAX8903_DOK_GPIO_FOR_IRQ,
		.end	= MAX8903_DOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}
};

static struct resource max8903_gpio_resources_evt1b[] = {
	{	.name	= MAX8903_TOKEN_GPIO_CHG_EN,
		.start	= MAX8903_GPIO_CHG_EN,
		.end	= MAX8903_GPIO_CHG_EN,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_FLT,
		.start	= MAX8903_GPIO_CHG_FLT,
		.end	= MAX8903_GPIO_CHG_FLT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_IUSB,
		.start	= MAX8903_GPIO_CHG_IUSB,
		.end	= MAX8903_GPIO_CHG_IUSB,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_USUS,
		.start	= MAX8903_GPIO_CHG_USUS_EVT1B,
		.end	= MAX8903_GPIO_CHG_USUS_EVT1B,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_ILM,
		.start	= MAX8903_GPIO_CHG_ILM_EVT1B,
		.end	= MAX8903_GPIO_CHG_ILM_EVT1B,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_UOK,
		.start	= MAX8903_UOK_GPIO_FOR_IRQ,
		.end	= MAX8903_UOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_DOK,
		.start	= MAX8903_DOK_GPIO_FOR_IRQ,
		.end	= MAX8903_DOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}
};

static struct resource max8903_gpio_resources_dvt[] = {
	{	.name	= MAX8903_TOKEN_GPIO_CHG_EN,
		.start	= MAX8903_GPIO_CHG_EN,
		.end	= MAX8903_GPIO_CHG_EN,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_FLT,
		.start	= MAX8903_GPIO_CHG_FLT,
		.end	= MAX8903_GPIO_CHG_FLT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_IUSB,
		.start	= MAX8903_GPIO_CHG_IUSB,
		.end	= MAX8903_GPIO_CHG_IUSB,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_USUS,
		.start	= MAX8903_GPIO_CHG_USUS_DVT,
		.end	= MAX8903_GPIO_CHG_USUS_DVT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_ILM,
		.start	= MAX8903_GPIO_CHG_ILM_DVT,
		.end	= MAX8903_GPIO_CHG_ILM_DVT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_UOK,
		.start	= MAX8903_UOK_GPIO_FOR_IRQ,
		.end	= MAX8903_UOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_DOK,
		.start	= MAX8903_DOK_GPIO_FOR_IRQ,
		.end	= MAX8903_DOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}
};

static struct platform_device max8903_charger_device = {
	.name           = "max8903_charger",
	.id             = -1,
};

static inline void acclaim_init_charger(void)
{
	const int board_type = acclaim_board_type();

	pr_info("Acclaim init charger.\n");
	if (board_type >= DVT) {
		max8903_charger_device.resource = max8903_gpio_resources_dvt;
		max8903_charger_device.num_resources
			= ARRAY_SIZE(max8903_gpio_resources_dvt);
	} else if (board_type >= EVT1B) {
		max8903_charger_device.resource = max8903_gpio_resources_evt1b;
		max8903_charger_device.num_resources
			= ARRAY_SIZE(max8903_gpio_resources_evt1b);
	} else if (board_type == EVT1A) {
		max8903_charger_device.resource = max8903_gpio_resources_evt1a;
		max8903_charger_device.num_resources
			= ARRAY_SIZE(max8903_gpio_resources_evt1a);
	} else {
		pr_err("%s: Acclaim board %d not supported\n", __func__, 
		       board_type);
		return;
	}
	platform_device_register(&max8903_charger_device);
}

#endif

#ifdef CONFIG_BATTERY_MAX17042
struct max17042_platform_data max17042_platform_data_here = {

	.gpio = MAX17042_GPIO_FOR_IRQ,

};
#endif

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
	.rows			= 2,
	.cols			= 1,
	.keypad_pad_wkup        = keypad_pad_wkup,
};

static struct gpio_keys_button acclaim_gpio_buttons[] = {
	{
		.code 		= KEY_POWER,
		.gpio 		= 29,
		.desc 		= "POWER",
		.active_low 	= 0,
		.wakeup 	= 1,
	},
	{
		.code 		= KEY_HOME,
		.gpio 		= 32,
		.desc 		= "HOME",
		.active_low 	= 1,
		.wakeup 	= 1,
	}
};

static struct gpio_keys_platform_data acclaim_gpio_key_info ={
	.buttons	= acclaim_gpio_buttons,
	.nbuttons	= ARRAY_SIZE ( acclaim_gpio_buttons ),
};

static struct platform_device acclaim_keys_gpio = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data = &acclaim_gpio_key_info,
	},
};

static struct platform_device acclaim_aic3110 = {
        .name = "tlv320aic3110-codec",
        .id = -1,
};

/*******************************************************/
static struct regulator_consumer_supply tp_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi2"),
};

static struct regulator_init_data tp_vinit = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(tp_supply),
	.consumer_supplies = tp_supply,
};

static struct fixed_voltage_config touch_reg_data = {
	.supply_name = "vdd_lcdtp",
	.microvolts = 3300000,
	.gpio = 36,
	.enable_high = 1,
	.enabled_at_boot = 1,
	.init_data = &tp_vinit,
};

static struct platform_device touch_regulator_device = {
	.name   = "reg-fixed-voltage",
	.id     = 0,
	.dev    = {
		.platform_data = &touch_reg_data,
	},
};

static struct regulator_consumer_supply lcd_supply[] = {
	{ .supply = "leds_pwm"},
};

static struct regulator_init_data lcd_vinit = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(lcd_supply),
	.consumer_supplies = lcd_supply,
};

static struct fixed_voltage_config lcd_reg_data = {
	.supply_name = "vdd_lcd",
	.microvolts = 3300000,
	.gpio = 121,
	.enable_high = 1,
	.enabled_at_boot = 1,
	.init_data = &lcd_vinit,
};

static struct platform_device lcd_regulator_device = {
	.name   = "reg-fixed-voltage",
	.id     = -1,
	.dev    = {
		.platform_data = &lcd_reg_data,
	},
};

/***************************************************************/
static struct platform_device *acclaim_devices[] __initdata = {
	&acclaim_aic3110,
	&acclaim_keys_gpio,
	&lcd_regulator_device,
	&touch_regulator_device,
};

static void __init acclaim_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 100,
};

static struct twl4030_usb_data acclaim_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA 
		| MMC_CAP_8_BIT_DATA 
		| MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable   = true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA 
		| MMC_CAP_8_BIT_DATA
		| MMC_CAP_1_8V_DDR,
		.gpio_wp	= -EINVAL,
		.nonremovable 	= false,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.name           = "wl1271",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA
		| MMC_CAP_POWER_OFF_CARD
		| MMC_PM_KEEP_POWER,
		.gpio_cd	= -EINVAL,
 		.gpio_wp        = -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable 	= true,
	},
	{}      /* Terminator */
};

/* External SD-card */
static struct regulator_consumer_supply acclaim_vmmc_supply[] = {
 	{
		.supply = "vmmc",
 		.dev_name = "omap_hsmmc.0",
 	},
};

static struct regulator_consumer_supply acclaim_vwlan_supply[] = {
	{
		.supply = "vwlan",
	},
};

static int wl12xx_set_power(struct device *dev, int slot, int on, int vdd)
{
	printk(KERN_WARNING"%s: %d\n", __func__, on);
	if (on) {
		gpio_set_value(GPIO_WIFI_PWEN, on);
		udelay(800);
		gpio_set_value(GPIO_WIFI_PMENA, on);
	} else {
		gpio_set_value(GPIO_WIFI_PMENA, on);
		gpio_set_value(GPIO_WIFI_PWEN, on);
	}
	return 0;
}

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev
		= container_of(dev, struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	
	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret){
			pr_err("Failed configuring MMC1 card detect\n");
		}
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
			MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	if (pdev->id == 2) {
		ret = 0;
		pdata->slots[0].mmc_data.built_in = 1;
	}

	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init = omap4_twl6030_hsmmc_late_init;
}

static int __init 
acclaim_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);
	return 0;
}

static struct regulator_consumer_supply audio_supply[] = {
        { .supply = "audio-pwr", },
};

static struct regulator_init_data acclaim_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.always_on	= true,
	},
};

static struct regulator_init_data acclaim_vaux3 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.always_on	= true,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = acclaim_vwlan_supply,
};

static struct regulator_init_data acclaim_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = acclaim_vmmc_supply,
};

static struct regulator_init_data acclaim_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data acclaim_vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
				| REGULATOR_CHANGE_MODE
				| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = audio_supply,
};

static struct regulator_init_data acclaim_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		//.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data acclaim_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		//.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
		| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
		| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
		.always_on	= true,
	},
};


static struct regulator_consumer_supply vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "twl6030_usb"),
};

static struct regulator_init_data acclaim_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		//.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
		| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
		| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(vusb_supply),
	.consumer_supplies      = vusb_supply,
};

static struct regulator_init_data acclaim_clk32kg = {
	.constraints = {
		.valid_ops_mask	 = REGULATOR_CHANGE_STATUS,
		.always_on	= true,
	},
};

static struct twl4030_madc_platform_data acclaim_gpadc_data = {
	.irq_line	= 1,
};

static int acclaim_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};

static struct twl4030_bci_platform_data acclaim_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= acclaim_batt_table,
	.tblsize			= ARRAY_SIZE(acclaim_batt_table),
};


static struct twl4030_platform_data acclaim_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &acclaim_vmmc,
	.vpp		= &acclaim_vpp,
	.vusim		= &acclaim_vusim,
	.vana		= &acclaim_vana,
	.vcxio		= &acclaim_vcxio,
	.vusb		= &acclaim_vusb,
	.vaux1		= &acclaim_vaux1,
	.vaux3		= &acclaim_vaux3,
	.clk32kg	= &acclaim_clk32kg,
	.usb		= &acclaim_usbphy_data,
	.bci		= &acclaim_bci_data,

	/* children */
	.madc           = &acclaim_gpadc_data,
};

static struct i2c_board_info __initdata acclaim_i2c_1_boardinfo[] = {
#ifdef CONFIG_INPUT_KXTF9
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(66),
	},
#endif

#ifdef CONFIG_INPUT_KXTJ9
	{
		I2C_BOARD_INFO(KXTJ9_DEVICE_ID, KXTJ9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtj9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(66),
	},
#endif

	{
		I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
		.platform_data = &max17042_platform_data_here,
		.irq = OMAP_GPIO_IRQ(65),
	},
};


static struct i2c_board_info __initdata acclaim_i2c_2_boardinfo[] = {
	{
 		I2C_BOARD_INFO(FT_I2C_NAME, FT5x06_I2C_SLAVEADDRESS),
 		.platform_data = &ft5x06_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_FT5x06_GPIO),
	},
};

static void __init show_acclaim_board_revision(int revision)
{
	switch (revision) {
	case EVT1A:
		printk(KERN_INFO "Board revision %s\n", "EVT1A");
		break;
	case EVT1B:
		printk(KERN_INFO "Board revision %s\n", "EVT1B");
		break;
	case EVT2:
		printk(KERN_INFO "Board revision %s\n", "EVT2");
		break;
	case DVT:
		printk(KERN_INFO "Board revision %s\n", "DVT");
		break;
	case PVT:
		printk(KERN_INFO "Board revision %s\n", "PVT");
		break;
	default:
		printk(KERN_ERR "Board revision UNKNOWN (0x%x)\n", 
		       revision);
		break;
	}
}

void __init acclaim_board_init(void)
{
	const int board_type = acclaim_board_type();
	show_acclaim_board_revision(board_type);

	omap_mux_init_signal("sys_pwron_reset_out", OMAP_MUX_MODE3);
	omap_mux_init_signal("fref_clk3_req", 
			     OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN);
}

static void __init 
acclaim_i2c_hwspinlock_init(int bus_id, int spinlock_id,
			    struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
		       bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata acclaim_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata acclaim_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata acclaim_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata acclaim_i2c_4_bus_pdata;

static int __init acclaim_i2c_init(void)
{
	int err;

	acclaim_i2c_hwspinlock_init(1, 0, &acclaim_i2c_1_bus_pdata);
	acclaim_i2c_hwspinlock_init(2, 1, &acclaim_i2c_2_bus_pdata);
	acclaim_i2c_hwspinlock_init(3, 2, &acclaim_i2c_3_bus_pdata);
	acclaim_i2c_hwspinlock_init(4, 3, &acclaim_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &acclaim_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &acclaim_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &acclaim_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &acclaim_i2c_4_bus_pdata);

	omap4_pmic_init("twl6030", &acclaim_twldata);
	
	err = i2c_register_board_info(1,
				      acclaim_i2c_1_boardinfo, 
				      ARRAY_SIZE(acclaim_i2c_1_boardinfo));
	if (err)
		return err;

	omap_register_i2c_bus(2, 400, acclaim_i2c_2_boardinfo,
			      ARRAY_SIZE(acclaim_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400, NULL, 0);
	omap_register_i2c_bus(4, 400, NULL, 0);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	return 0;
}

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux acclaim_board_mux[] __initdata = {
	OMAP4_MUX(USBB1_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#else
#define board_mux	NULL
#endif

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */

static __initdata struct emif_device_details emif_devices_samsung = {
        .cs0_device = &samsung_4G_S4,
        .cs1_device = 0
};

static __initdata struct emif_device_details emif_devices_512_samsung = {
        .cs0_device = &samsung_2G_S4,
        .cs1_device = 0
};

static __initdata struct emif_device_details emif_devices_elpida = {
        .cs0_device = &lpddr2_elpida_2G_S4_dev,
        .cs1_device = &lpddr2_elpida_2G_S4_dev
};

static __initdata struct emif_device_details emif_devices_512_elpida = {
        .cs0_device = &lpddr2_elpida_2G_S4_dev,
        .cs1_device = 0
};

static struct omap_device_pad acclaim_uart1_pads[] __initdata = {
	{
		.name	= "uart3_cts_rctx.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name	= "mcspi1_cs1.uart1_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX	| OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP
			| OMAP_PIN_OFF_WAKEUPENABLE
			| OMAP_MUX_MODE1,
		.idle	= OMAP_PIN_INPUT_PULLUP
			| OMAP_PIN_OFF_WAKEUPENABLE
			 |OMAP_MUX_MODE1,
	},
};

static struct omap_uart_port_info acclaim_uart_info __initdata = {
	.use_dma	= 0,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
};

static inline void acclaim_serial_init(void)
{
	pr_info(KERN_INFO "Board serial init\n");
	omap_serial_init_port_pads(0, acclaim_uart1_pads,
		ARRAY_SIZE(acclaim_uart1_pads), &acclaim_uart_info);
}

static struct wl12xx_platform_data omap4_acclaim_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38
};

void config_wlan_mux(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, 
			   OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);
}

static void __init acclaim_wifi_init(void)
{
	struct device *dev;
	struct omap_mmc_platform_data *pdata;
	int ret;
	printk(KERN_WARNING"%s: start\n", __func__);
	
	ret = gpio_request(GPIO_WIFI_PMENA, "wifi_pmena");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
		       GPIO_WIFI_PMENA);
		goto out;
	}
	gpio_direction_output(GPIO_WIFI_PMENA, 0);
      
	ret = gpio_request(GPIO_WIFI_PWEN, "wifi_pwen");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
		       GPIO_WIFI_PWEN);
		goto out;
	}
	gpio_direction_output(GPIO_WIFI_PWEN, 0);
  
	ret = gpio_request(GPIO_WIFI_IRQ, "wifi_irq");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
		       GPIO_WIFI_IRQ);
		goto out;
	}
	gpio_direction_input(GPIO_WIFI_IRQ);

	dev = mmc[2].dev;
	if (!dev) {
	      
		pr_err("wl12xx mmc device initialization failed\n");
		goto out;
	}
 
	pdata = dev->platform_data;
	if (!pdata) {
		pr_err("Platfrom data of wl12xx device not set\n");
		goto out;
	}
   
	pdata->slots[0].set_power = wl12xx_set_power;
	config_wlan_mux ();

	if (wl12xx_set_platform_data(&omap4_acclaim_wlan_data))
		pr_err("Error setting wl12xx data\n"); 
 out:
	return;
}


static void acclaim_enable_rtc_gpio(void){
	/* To access twl registers we enable gpio6
	 * we need this so the RTC driver can work.
	 */
	gpio_request(TWL6030_RTC_GPIO, "h_SYS_DRM_MSEC");
	gpio_direction_output(TWL6030_RTC_GPIO, 1);

	omap_mux_init_signal("fref_clk0_out.gpio_wk6", \
			     OMAP_PIN_OUTPUT | OMAP_PIN_OFF_NONE);
                
	return;
}

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
struct usbhs_omap_board_data usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init acclaim_ehci_ohci_init(void)
{
	omap_mux_init_signal("usbb2_ulpitll_clk.gpio_157", \
			     OMAP_PIN_OUTPUT | \
			     OMAP_PIN_OFF_NONE);
	
	// Power on the ULPI PHY
	if (gpio_is_valid(BLAZE_MDM_PWR_EN_GPIO)) {
		gpio_request(BLAZE_MDM_PWR_EN_GPIO, "USBB1 PHY VMDM_3V3");
		gpio_direction_output(BLAZE_MDM_PWR_EN_GPIO, 1);
	}

	usbhs_init(&usbhs_bdata);

	return;

}
#else
static void __init acclaim_ehci_ohci_init(void){}
#endif

static void acclaim_set_osc_timings(void)
{
	/* Device Oscilator
	 * tstart = 2ms + 2ms = 4ms.
	 * tshut = Not defined in oscillator data sheet so setting to 1us
	 */
	omap_pm_set_osc_lp_time(4000, 1);
}

static void acclaim_ram_init(void)
{
	ulong sdram_size = get_sdram_size();

	if (sdram_vendor() == SAMSUNG_SDRAM) {
		if (sdram_size == SZ_512M) {
			omap_emif_setup_device_details
				(&emif_devices_512_samsung, 
				 &emif_devices_512_samsung);
		} else if (sdram_size == SZ_1G) {
			omap_emif_setup_device_details
				(&emif_devices_samsung, 
				 &emif_devices_samsung);
		} else {
			pr_err("sdram memory size does not exist, "
			       "default to using 1024MB \n");
			omap_emif_setup_device_details
				(&emif_devices_samsung, 
				 &emif_devices_samsung);
		}
		printk(KERN_INFO"Samsung DDR Memory \n");
	} else if (sdram_vendor() == ELPIDA_SDRAM) {
		if (sdram_size == SZ_512M) {
			omap_emif_setup_device_details
				(&emif_devices_512_elpida, 
				 &emif_devices_512_elpida);
		} else if (sdram_size == SZ_1G) {
			omap_emif_setup_device_details	(&emif_devices_elpida, 
							 &emif_devices_elpida);
		} else {
			pr_err("sdram memory size does not exist, "
			       "default to using 1024MB \n");
			omap_emif_setup_device_details(&emif_devices_elpida,
						       &emif_devices_elpida);
		}
		printk(KERN_INFO"Elpida DDR Memory \n");
	} else if (sdram_vendor() == HYNIX_SDRAM) {
		/* Re-use ELPIDA timings as they are absolutely the same */
		if (sdram_size == SZ_512M) {
			omap_emif_setup_device_details
				(&emif_devices_512_elpida,
				 &emif_devices_512_elpida);
		} else if (sdram_size == SZ_1G) {
			omap_emif_setup_device_details(&emif_devices_elpida, 
						       &emif_devices_elpida);
		} else {
			pr_err("sdram memory size does not exist, "
			       "default to using 1024MB \n");
			omap_emif_setup_device_details
				(&emif_devices_elpida, &emif_devices_elpida);
		}
		printk(KERN_INFO"Hynix DDR Memory \n");
	} else
		pr_err("Memory type does not exist\n");
}


static void __init acclaim_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(acclaim_board_mux, NULL, package);

	acclaim_board_init();
	acclaim_ram_init ();
	omap4_create_board_props();
	acclaim_set_osc_timings();
	acclaim_i2c_init();
	acclaim_enable_rtc_gpio();
	omap4_register_ion();
	platform_add_devices(acclaim_devices, ARRAY_SIZE(acclaim_devices));
	
#ifdef CONFIG_CHARGER_MAX8903	
	acclaim_init_charger();
#endif
	
	acclaim_serial_init();

	acclaim_twl6030_hsmmc_init(mmc);
	acclaim_wifi_init();
#ifdef CONFIG_INPUT_KXTF9
	kxtf9_dev_init();
#endif
	
#ifdef CONFIG_INPUT_KXTJ9
	kxtj9_dev_init();
#endif

#ifdef CONFIG_BATTERY_MAX17042
	acclaim_max17042_dev_init();
#endif
	acclaim_ehci_ohci_init();

	usb_musb_init(&musb_board_data);

	keyboard_mux_init();
	status = omap4_keyboard_init(&acclaim_keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);

	omap_dmm_init();
	acclaim_panel_init();
	omap_enable_smartreflex_on_init();
	if (enable_suspend_off)
		omap_pm_enable_off_mode();

}

static void acclaim_init_display_led(void)
{
	if (acclaim_board_type() >= EVT2) {
		printk(KERN_INFO "init_display_led: evt2 hardware\n");
		omap_mux_init_signal("abe_dmic_din2.dmtimer11_pwm_evt",
				     OMAP_MUX_MODE5);
	} else {
		printk(KERN_INFO "init_display_led: evt1 hardware\n"
		       "brigthness control disabled on EVT1 hardware\n");
		/*
		   mux the brightness control pin as gpio, because on EVT1
		   it is connected to timer8 and we cannot use timer8
		   because of audio conflicts causing crash
		*/
		omap_mux_init_signal("usbb1_ulpitll_dat4.gpio_92",
				     OMAP_MUX_MODE3);
		if (gpio_request(92, "EVT1 BACKLIGHT"))
			printk(KERN_ERR
			       "ERROR: failed to request backlight gpio\n");
		else
			gpio_direction_output(92, 0);
	}
}

static void 
acclaim_disp_backlight_setpower(struct omap_pwm_led_platform_data *pdata,
				int on_off)
{
	printk(KERN_INFO "Backlight set power, on_off = %d\n",on_off);
	if (on_off) {
		msleep(500);
		gpio_direction_output(38, (acclaim_board_type() >= EVT2)
				      ? 1 : 0);
	} else {
		gpio_direction_output(38, (acclaim_board_type() >= EVT2) 
				      ? 0 : 1);
	}
	gpio_direction_output(44, 0);
	gpio_direction_output(45, 0);
	pr_debug("%s: on_off:%d\n", __func__, on_off);

	printk(KERN_INFO "Backlight set power end\n");
}

static struct omap_pwm_led_platform_data acclaim_disp_backlight_data = {
	.name 		 = "lcd-backlight",
	.default_trigger  = "backlight",
	.intensity_timer = 11,
	.bkl_max    = 254,
	.bkl_min    = 5,
	.bkl_freq    = 128*2,
	.invert     = 1,
	.def_brightness	 = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.set_power	 = acclaim_disp_backlight_setpower,
};

static struct platform_device acclaim_disp_led = {
	.name	=	"omap_pwm_led",
	.id	=	0,
	.dev	= {
		.platform_data = &acclaim_disp_backlight_data,
	},
};

static struct platform_device *acclaim_panel_devices[] __initdata = {
	&acclaim_disp_led,
};

static void acclaim_panel_get_resource(void)
{
	int ret_val = 0;

	pr_info("acclaim_panel_get_resource\n");
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

static inline struct boxer_panel_data * 
get_panel_data(struct omap_dss_device *dssdev)
{
	return dssdev->data;
}

static struct omap_dss_device acclaim_boxer_device = {
	.phy		= {
		.dpi	= {
			.data_lines	= 24,
		},
	},
	.clocks		= {
		.dispc	= {
			.channel	= {
				.lck_div        = 1,
				.pck_div        = 4,
				.lcd_clk_src    = 
				OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
		},
	},
	.panel          = {
		.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS,
		.timings	= {
			.x_res          = 1024,
			.y_res          = 600,
			.pixel_clock    = 46000, /* in kHz */
			.hfp            = 160,   /* HFP fix 160 */
			.hsw            = 10,    /* HSW = 1~140 */
			.hbp            = 150,   /* HSW + HBP = 160 */
			.vfp            = 12,    /* VFP fix 12 */
			.vsw            = 3,     /* VSW = 1~20 */
			.vbp            = 20,    /* VSW + VBP = 23 */
		},
		.width_in_um = 158000,
		.height_in_um = 92000,
	},
	.name			= "lcd2",
	.driver_name		= "boxer_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

static struct omap_dss_device *acclaim_dss_devices[] = {
	&acclaim_boxer_device,
};
 
static struct omap_dss_board_info acclaim_dss_data = {
	.num_devices	= ARRAY_SIZE(acclaim_dss_devices),
	.devices	= acclaim_dss_devices,
	.default_device	= &acclaim_boxer_device,
};

static struct spi_board_info tablet_spi_board_info[] __initdata = {
	{
		.modalias= "boxer_disp_spi",
		.bus_num= 4,     /* McSPI4 */
		.chip_select= 0,
		.max_speed_hz= 375000,
	},
};

static struct omapfb_platform_data acclaim_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = ACCLAIM_FB_RAM_SIZE,
			},
		},
	},
};
 

void acclaim_panel_init(void)
{
	acclaim_panel_get_resource();
	acclaim_init_display_led();
	omapfb_set_platform_data(&acclaim_fb_pdata); 
	omap_display_init(&acclaim_dss_data);
	spi_register_board_info(tablet_spi_board_info,
				ARRAY_SIZE(tablet_spi_board_info));
	platform_add_devices(acclaim_panel_devices,
			     ARRAY_SIZE(acclaim_panel_devices));
}

static void __init acclaim_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static void __init acclaim_reserve(void)
{
	omap_init_ram_size();

#ifdef CONFIG_ION_OMAP
	omap_android_display_setup(&acclaim_dss_data,
				   NULL,
				   NULL,
				   &acclaim_fb_pdata,
				   get_omap_ion_platform_data());
	omap_ion_init();
#else
	omap_android_display_setup(&acclaim_dss_data,
				   NULL,
				   NULL,
				   &acclaim_fb_pdata,
				   NULL);
#endif

	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			      OMAP_RAM_CONSOLE_SIZE_DEFAULT);

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);

	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM,
				    PHYS_ADDR_DUCATI_SIZE +
				    OMAP4_ION_HEAP_SECURE_INPUT_SIZE +
				    OMAP4_ION_HEAP_SECURE_OUTPUT_WFDHDCP_SIZE);

	omap_reserve();
}

MACHINE_START(OMAP4_NOOKTABLET, "acclaim")
.boot_params	= 0x80000100,
	.reserve	= acclaim_reserve,
	.map_io		= acclaim_map_io,
	.init_early	= acclaim_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= acclaim_init,
	.timer		= &omap_timer,
MACHINE_END
