/*
 * Board support file for OMAP44xx tablet.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Dan Murphy <dmurphy@ti.com>
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
#include <linux/memblock.h>
#include <linux/reboot.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/hwspinlock.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <plat/omap-serial.h>

#include <mach/dmm.h>
#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/omap-serial.h>
#include <plat/remoteproc.h>
#include <plat/omap-pm.h>
#include <linux/wakelock.h>
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "common-board-devices.h"
#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "board-acclaim.h"
#include "omap4_ion.h"

#ifdef CONFIG_INPUT_KXTF9
#include <linux/input/kxtf9.h>
#endif // CONFIG_INPUT_KXTF9

#ifdef CONFIG_BATTERY_MAX17042
#include <linux/max17042.h>
#endif //CONFIG_BATTERY_MAX17042

#ifdef CONFIG_CHARGER_MAX8903
#include <linux/max8903.h>
#endif //CONFIG_CHARGER_MAX8903

#ifdef CONFIG_INPUT_KXTF9

#define KXTF9_DEVICE_ID                 "kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS         0x0F
#define KXTF9_GPIO_FOR_PWR              34

static int kxtf9_gpio_for_irq = 0;
struct kxtf9_platform_data kxtf9_platform_data_here = {
	.min_interval   = 1,
	.poll_interval  = 1000,

	.g_range        = KXTF9_G_8G,
	.shift_adj      = SHIFT_ADJ_2G,

	/* Map the axes from the sensor to the device */
	/* SETTINGS FOR acclaim */
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.axis_map_x     = 1,
	.axis_map_y     = 0,
	.axis_map_z     = 2,
	.negate_x       = 1,
	.negate_y       = 0,
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

	.gpio = 0,
};

static void kxtf9_dev_init(void)
{
	printk("%s: %s ...\n", __FILE__, __func__);

	kxtf9_platform_data_here.gpio = kxtf9_gpio_for_irq;

	if (gpio_request(kxtf9_gpio_for_irq, "kxtf9_irq") < 0)
	{
		printk("%s: %s: Can't get GPIO for kxtf9 IRQ\n", __FILE__, __func__);
		return;
	}

	printk("%s: %s > Init kxtf9 irq pin %d !\n", __FILE__, __func__,
			kxtf9_gpio_for_irq);
	gpio_direction_input(kxtf9_gpio_for_irq);
}
#endif //CONFIG_INPUT_KXTF9

#ifdef CONFIG_BATTERY_MAX17042
static int max17042_gpio_for_irq = 0;
struct max17042_platform_data max17042_platform_data_here = {
	.gpio = 0,
};

static void max17042_dev_init(void)
{
	printk("%s: enter ...\n", __func__);

	max17042_platform_data_here.gpio = max17042_gpio_for_irq;

	if (gpio_request(max17042_gpio_for_irq, "max17042_irq") < 0)
	{
		printk(KERN_ERR "%s: Can't get GPIO for max17042 IRQ\n", __func__);
		return;
	}
	printk("%s: Init max17042 irq pin %d !\n", __func__, max17042_gpio_for_irq);
	gpio_direction_input(max17042_gpio_for_irq);
	printk("%s: max17042 GPIO pin read %d\n", __func__, gpio_get_value(max17042_gpio_for_irq));
}
#endif // CONFIG_BATTERY_MAX17042

#ifdef CONFIG_CHARGER_MAX8903
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

static inline void max8903_init_charger(void)
{
	max8903_charger_device.resource = max8903_gpio_resources_dvt;
	max8903_charger_device.num_resources = ARRAY_SIZE(max8903_gpio_resources_dvt);
	platform_device_register(&max8903_charger_device);
}
#endif //CONFIG_CHARGER_MAX8903

#define GPIO_WIFI_PMENA		114
#define GPIO_WIFI_IRQ		115
#define GPIO_WIFI_PWEN		118

#define OMAP4_MDM_PWR_EN_GPIO       157
#define GPIO_WK30		    30


static struct omap_board_config_kernel tablet_config[] __initdata = {
};

static void __init omap_tablet_init_early(void)
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
#else
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 200,
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		=  MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
		.no_off_init	= true,
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_wp	= -EINVAL,
	},
        {
                .mmc            = 3,
                .caps           = MMC_CAP_4_BIT_DATA,
                .gpio_cd        = -EINVAL,
                .gpio_wp        = 4,
                .ocr_mask       = MMC_VDD_165_195,
        },
	{}	/* Terminator */
};

static struct regulator_consumer_supply tablet_vaux_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.1",
	},
};
static struct regulator_consumer_supply tablet_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
};
static struct regulator_consumer_supply tablet_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.0"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.1"),
};

static struct regulator_consumer_supply omap4_tablet_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.2",
};

static struct regulator_init_data tablet_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on	= 1,
	},
//	.num_consumer_supplies = 1,
//	.consumer_supplies = &omap4_tablet_vmmc5_supply,
};

static struct fixed_voltage_config tablet_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.8V */
	.gpio			= GPIO_WIFI_PMENA,
	.startup_delay		= 70000, /* 70msec */
	.enable_high		= 1,
	.enabled_at_boot	= 1,
	.init_data		= &tablet_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &tablet_vwlan,
	},
};


static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
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
	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data tablet_vaux1 = {
        .constraints = {
                .min_uV                 = 1000000,
                .max_uV                 = 3000000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask  	= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
                .always_on      = true,
        },
	.num_consumer_supplies  = 1,
	.consumer_supplies      = tablet_vaux_supply,
};

static struct regulator_consumer_supply sdp4430_vaux2_supply[] = {
	REGULATOR_SUPPLY("av-switch", "soc-audio"),
};

static struct regulator_init_data tablet_vaux2 = {
        .constraints = {
                .min_uV                 = 1200000,
                .max_uV                 = 2800000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask  	= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
                .always_on      = 1,
        },
	.num_consumer_supplies	= 1,
	.consumer_supplies	= sdp4430_vaux2_supply,
};

static struct regulator_consumer_supply sdp4430_vwlan_supply[] = {
        {
                .supply = "vwlan",
        },
};

static struct regulator_init_data tablet_vaux3 = {
        .constraints = {
                .min_uV                 = 1800000,
                .max_uV                 = 1800000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask  	= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
                .always_on      = true,
        },
        .num_consumer_supplies = 1,
        .consumer_supplies = sdp4430_vwlan_supply,
};


/* VMMC1 for MMC1 card */
static struct regulator_init_data tablet_vmmc = {
        .constraints = {
                .min_uV                 = 1200000,
                .max_uV                 = 3000000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask  	= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
	.num_consumer_supplies  = 1,
	.consumer_supplies      = tablet_vmmc_supply,
};

static struct regulator_init_data tablet_vpp = {
        .constraints = {
                .min_uV                 = 1800000,
                .max_uV                 = 2500000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask  	= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
};

static struct regulator_init_data tablet_vusim = {
        .constraints = {
                .min_uV                 = 1200000,
                .max_uV                 = 2900000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask 	= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
};

static struct regulator_init_data tablet_vana = {
        .constraints = {
                .min_uV                 = 2100000,
                .max_uV                 = 2100000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask  	= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
};

static struct regulator_init_data tablet_vcxio = {
        .constraints = {
                .min_uV                 = 1800000,
                .max_uV                 = 1800000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask  	= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
                .always_on      = true,
        },
	.num_consumer_supplies	= ARRAY_SIZE(tablet_vcxio_supply),
	.consumer_supplies	= tablet_vcxio_supply,
};

static struct regulator_consumer_supply sdp4430_vdac_supply[] = {
	{
		.supply = "hdmi_vref",
	},
};

static struct regulator_init_data tablet_vdac = {
        .constraints = {
                .min_uV                 = 1800000,
                .max_uV                 = 1800000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask  	= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
	.num_consumer_supplies  = ARRAY_SIZE(sdp4430_vdac_supply),
	.consumer_supplies      = sdp4430_vdac_supply,
};

static struct regulator_init_data tablet_vusb = {
        .constraints = {
                .min_uV                 = 3300000,
                .max_uV                 = 3300000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
                .valid_ops_mask  	= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
};

static struct regulator_init_data tablet_clk32kg = {
        .constraints = {
                .valid_modes_mask       = REGULATOR_MODE_NORMAL,
                .valid_ops_mask  	= REGULATOR_CHANGE_STATUS,
                .always_on      	= true,
        },
};

static struct twl4030_madc_platform_data twl6030_gpadc = {
	.irq_line = -1,
};

static struct twl4030_platform_data tablet_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &tablet_vmmc,
	.vpp		= &tablet_vpp,
//	.vusim		= &tablet_vusim,
	.vana		= &tablet_vana,
	.vcxio		= &tablet_vcxio,
//	.vdac		= &tablet_vdac,
	.vusb		= &tablet_vusb,
	.vaux1		= &tablet_vaux1,
//	.vaux2		= &tablet_vaux2,
	.vaux3		= &tablet_vaux3,
	.clk32kg	= &tablet_clk32kg,
	.usb		= &omap4_usbphy_data,

	.madc		= &twl6030_gpadc,
};

enum I2C_BUS_1_DEVICE_INDEX {
	I2C_INDEX_1_START = 0x00,
#ifdef CONFIG_INPUT_KXTF9
	I2C_INDEX_1_KXTF9,
#endif //CONFIG_INPUT_KXTF9
#ifdef CONFIG_BATTERY_MAX17042
	I2C_INDEX_1_MAX17042,
#endif //CONFIG_BATTERY_MAX17042
	I2C_INDEX_1_INVALID
};

static struct i2c_board_info __initdata sdp4430_i2c_boardinfo[] = {
#ifdef CONFIG_INPUT_KXTF9
	[I2C_INDEX_1_KXTF9] =
		{
			I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
			.platform_data = &kxtf9_platform_data_here,
			.irq = 0,
		},
#endif //CONFIG_INPUT_KXTF9
#ifdef CONFIG_BATTERY_MAX17042
	[I2C_INDEX_1_MAX17042] =
		{
			I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
			.platform_data = &max17042_platform_data_here,
			.irq = 0,
		},
#endif //CONFIG_BATTERY_MAX17042
};

static struct i2c_board_info __initdata tablet_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("tlv320aic31xx-codec", 0x18),
	}

};

static void __init tablet_pmic_mux_init(void)
{

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
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

static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_4_bus_pdata;

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &sdp4430_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &sdp4430_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &sdp4430_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &sdp4430_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &sdp4430_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &sdp4430_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &sdp4430_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &sdp4430_i2c_4_bus_pdata);

	omap4_pmic_init("twl6030", &tablet_twldata); //	i2c_register_board_info(1, &sdp4430_i2c_boardinfo, 1);

#ifdef CONFIG_INPUT_KXTF9
	kxtf9_gpio_for_irq = 66;
	sdp4430_i2c_boardinfo[I2C_INDEX_1_KXTF9].irq = OMAP_GPIO_IRQ(kxtf9_gpio_for_irq);
	kxtf9_dev_init();
#endif //CONFIG_INPUT_KXTF9
#ifdef CONFIG_BATTERY_MAX17042
	max17042_gpio_for_irq = 65;
	sdp4430_i2c_boardinfo[I2C_INDEX_1_MAX17042].irq = OMAP_GPIO_IRQ(max17042_gpio_for_irq);
	max17042_dev_init();
#endif //CONFIG_BATTERY_MAX17042
	i2c_register_board_info(1, sdp4430_i2c_boardinfo, ARRAY_SIZE(sdp4430_i2c_boardinfo));
	i2c_register_board_info(2, tablet_i2c_3_boardinfo, ARRAY_SIZE(tablet_i2c_3_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	omap_register_i2c_bus(4, 400, NULL, 0);

	omap2_i2c_pullup(3, I2C_PULLUP_STD_860_OM_FAST_500_OM);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/*
	 * Drive MSECURE high for TWL6030 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}


static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

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
static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
	.cs1_device = &lpddr2_elpida_2G_S4_dev
};

#else
#define board_mux	NULL
#define board_wkup_mux	NULL
#endif

static struct omap_device_pad tablet_uart1_pads[] __initdata = {
	{
		.name	= "mcspi1_cs2.uart1_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
	{
		.name	= "mcspi1_cs3.uart1_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name	= "uart3_cts.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name	= "mcspi1_cs1.uart1_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
};

static struct omap_device_pad tablet_uart4_pads[] __initdata = {
	{
		.name	= "abe_dmic_clk1.uart4_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE5,
	},
	{
		.name	= "abe_dmic_din1.uart4_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE5,
	},
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_uart_port_info tablet_uart_info_uncon __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = 0,
};

static struct omap_uart_port_info tablet_uart_info __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
};

static inline void __init board_serial_init(void)
{
	/* console */
	omap_serial_init_port_pads(0, tablet_uart1_pads,
		ARRAY_SIZE(tablet_uart1_pads), &tablet_uart_info);
}


static void omap4_tablet_wifi_mux_init(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);
        omap_mux_init_gpio(GPIO_WIFI_PWEN, OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_cts.sdmmc3_clk",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_rts.sdmmc3_cmd",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_rx.sdmmc3_dat0",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_tx.sdmmc3_dat1",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("abe_mcbsp1_dx.sdmmc3_dat2",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("abe_mcbsp1_fsx.sdmmc3_dat3",
				OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
}

static struct wl12xx_platform_data omap4_tablet_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_38_4,
};

static void __init omap4_tablet_wifi_init(void)
{
	omap4_tablet_wifi_mux_init();

        gpio_request(GPIO_WIFI_PWEN, "wifi_pwen");
	gpio_direction_output(GPIO_WIFI_PWEN, 1);

	if (wl12xx_set_platform_data(&omap4_tablet_wlan_data))
		pr_err("Error setting wl12xx data\n");
	platform_device_register(&omap_vwlan_device);
}

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init omap4_ehci_ohci_init(void)
{
	omap_mux_init_signal("fref_clk3_req.gpio_wk30", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE | OMAP_PULL_ENA);

	/* Enable 5V,1A USB power on external HS-USB ports */
	if (gpio_is_valid(GPIO_WK30)) {
		gpio_request(GPIO_WK30, "USB POWER GPIO");
		gpio_direction_output(GPIO_WK30, 1);
		gpio_set_value(GPIO_WK30, 0);
	}

	omap_mux_init_signal("usbb2_ulpitll_clk.gpio_157", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);

	/* Power on the ULPI PHY */
	if (gpio_is_valid(OMAP4_MDM_PWR_EN_GPIO)) {
		gpio_request(OMAP4_MDM_PWR_EN_GPIO, "USBB1 PHY VMDM_3V3");
		gpio_direction_output(OMAP4_MDM_PWR_EN_GPIO, 1);
	}

	usbhs_init(&usbhs_bdata);

	return;

}
#else
static void __init omap4_ehci_ohci_init(void){}
#endif

static int tablet_notifier_call(struct notifier_block *this,
					unsigned long code, void *cmd)
{
	void __iomem *sar_base;
	u32 v = 0;

	sar_base = omap4_get_sar_ram_base();

	if (!sar_base)
		return notifier_from_errno(-ENOMEM);

	if ((code == SYS_RESTART) && (cmd != NULL)) {
		/* cmd != null; case: warm boot */
		if (!strcmp(cmd, "bootloader")) {
			/* Save reboot mode in scratch memory */
			strcpy(sar_base + 0xA0C, cmd);
			v |= OMAP4430_RST_GLOBAL_WARM_SW_MASK;
		} else if (!strcmp(cmd, "recovery")) {
			/* Save reboot mode in scratch memory */
			strcpy(sar_base + 0xA0C, cmd);
			v |= OMAP4430_RST_GLOBAL_WARM_SW_MASK;
		} else {
			v |= OMAP4430_RST_GLOBAL_COLD_SW_MASK;
		}
	}

	omap4_prm_write_inst_reg(0xfff, OMAP4430_PRM_DEVICE_INST,
			OMAP4_RM_RSTST);
	omap4_prm_write_inst_reg(v, OMAP4430_PRM_DEVICE_INST, OMAP4_RM_RSTCTRL);
	v = omap4_prm_read_inst_reg(WKUP_MOD, OMAP4_RM_RSTCTRL);

	return NOTIFY_DONE;
}

static struct notifier_block tablet_reboot_notifier = {
	.notifier_call = tablet_notifier_call,
};

static void __init omap_tablet_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;
	int tablet_rev = 0;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);
	omap_emif_setup_device_details(&emif_devices, &emif_devices);

	omap_board_config = tablet_config;
	omap_board_config_size = ARRAY_SIZE(tablet_config);
	tablet_rev = omap_init_board_version(0);
	register_reboot_notifier(&tablet_reboot_notifier);
	omap4_create_board_props();
	omap4_i2c_init();
#ifdef CONFIG_CHARGER_MAX8903
	max8903_init_charger();
#endif //CONFIG_CHARGER_MAX8903
	acclaim_touch_init();
	omap_dmm_init();
	acclaim_panel_init();
	tablet_pmic_mux_init();
	acclaim_button_init();
	omap4_register_ion();
	board_serial_init();
	omap4_tablet_wifi_init();
	omap4_twl6030_hsmmc_init(mmc);

	omap4_ehci_ohci_init();
	usb_musb_init(&musb_board_data);

	omap_enable_smartreflex_on_init();
	if (enable_suspend_off)
			omap_pm_enable_off_mode();

}

static void __init omap_tablet_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static void __init omap_tablet_reserve(void)
{
	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE);
#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif
	omap_reserve();
}


MACHINE_START(OMAP_ACCLAIM, "OMAP4 acclaim board")
	/* Maintainer: Dan Murphy - Texas Instruments Inc */
	.boot_params	= 0x80000100,
	.reserve	= omap_tablet_reserve,
	.map_io		= omap_tablet_map_io,
	.init_early	= omap_tablet_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= omap_tablet_init,
	.timer		= &omap_timer,
MACHINE_END
