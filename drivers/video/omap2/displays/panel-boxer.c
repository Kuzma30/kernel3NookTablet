/*
 * Boxer panel support
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@xxxxxxxxx>
 *
 * Copyright (c) 2010 Barnes & Noble
 * David Bolcsfoldi <dbolcsfoldi@xxxxxxxxxxxxx>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/workqueue.h>

#include <plat/mcspi.h>
#include <mach/gpio.h>
#include <mach/gpio.h>
#include <plat/mux.h>
#include <asm/mach-types.h>

#include <video/omapdss.h>

/* Delay between Panel configuration and Panel enabling */
#define LCD_RST_DELAY		100
#define LCD_INIT_DELAY		200

#define LCD_XRES		1024
#define LCD_YRES		600

#define LCD_PIXCLOCK_MIN	39000 /* CPT MIN PIX Clock is 39MHz */
#define Lcd_Pixclock_Typ	46000 /* Typical PIX clock is 45MHz */
#define LCD_PIXCLOCK_MAX	52000 /* Maximum is 52MHz */

/* Current Pixel clock */
#define LCD_PIXEL_CLOCK		46000

static struct workqueue_struct *boxer_panel_wq;
static struct omap_dss_device *boxer_panel_dssdev;
static struct regulator *boxer_panel_regulator;
static struct spi_device *boxer_spi_device;
static atomic_t boxer_panel_is_enabled = ATOMIC_INIT(0);

/* Get FT i2c adapter for lock/unlock it */
struct i2c_adapter *g_ft_i2c_adapter = NULL;

extern void register_ft_i2c_adapter(struct i2c_adapter *adapter)
{
	g_ft_i2c_adapter = adapter;
}

extern void unregister_ft_i2c_adapter(struct i2c_adapter *adapter)
{
	g_ft_i2c_adapter = NULL;
}
/*NEC NL8048HL11-01B  Manual
 * defines HFB, HSW, HBP, VFP, VSW, VBP as shown below
 */

static struct omap_video_timings boxer_panel_timings = {
	/* 1024 x 600 @ 60 Hz  Reduced blanking VESA CVT 0.31M3-R */
	.x_res          = LCD_XRES,
	.y_res          = LCD_YRES,
	.pixel_clock    = LCD_PIXEL_CLOCK,
	.hfp            = 48,
	.hsw            = 40,
	.hbp            = 65,
	.vfp            = 3,
	.vsw            = 10,
	.vbp            = 25,
};

static void boxer_get_resolution(struct omap_dss_device *dssdev,
				 u16 *xres, u16 *yres)
{

	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

int boxer_get_recommended_bpp(struct omap_dss_device *dssdev)
{
	return 24;
}

static int boxer_panel_probe(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IPC;
	dssdev->panel.timings = boxer_panel_timings;
	return 0;
}

static void boxer_panel_remove(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
}

static int spi_send(struct spi_device *spi, unsigned char reg_addr,
		    unsigned char reg_data)
{
	int ret = 0;
	uint16_t msg;
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	msg = (reg_addr << 10) | reg_data;

	if (spi_write(spi, (unsigned char *)&msg, 2))
		printk(KERN_ERR "error in spi_write %x\n", msg);

	udelay(10);

	return ret;
}

static void boxer_init_panel(void)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	spi_send(boxer_spi_device, 0, 0x00);
	spi_send(boxer_spi_device, 0x00, 0xad);
	spi_send(boxer_spi_device, 0x01, 0x30);
	spi_send(boxer_spi_device, 0x02, 0x40);
	spi_send(boxer_spi_device, 0x0e, 0x5f);
	spi_send(boxer_spi_device, 0x0f, 0xa4);
	spi_send(boxer_spi_device, 0x0d, 0x00);
	spi_send(boxer_spi_device, 0x02, 0x43);
	spi_send(boxer_spi_device, 0x0a, 0x28);
	spi_send(boxer_spi_device, 0x10, 0x41);
}

static void boxer_panel_work_func(struct work_struct *work)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	if (!regulator_is_enabled(boxer_panel_regulator)){
		regulator_enable(boxer_panel_regulator);
		printk(KERN_INFO " boxer : %s called , line %d, regulator_enable(boxer_panel_regulator); \n", __FUNCTION__ , __LINE__);
	}

	msleep(LCD_RST_DELAY);

	boxer_spi_device->mode = SPI_MODE_0;
	boxer_spi_device->bits_per_word = 16;
	spi_setup(boxer_spi_device);

	boxer_init_panel();

	msleep(LCD_INIT_DELAY);

	if (boxer_panel_dssdev->platform_enable){
		boxer_panel_dssdev->platform_enable(boxer_panel_dssdev);
		printk(KERN_INFO " boxer : %s called , line %d, boxer_panel_dssdev->platform_enable(boxer_panel_dssdev)\n", __FUNCTION__ , __LINE__);
	}
}

static DECLARE_WORK(boxer_panel_work, boxer_panel_work_func);

static int boxer_panel_enable(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	if (atomic_add_unless(&boxer_panel_is_enabled, 1, 1)) {
		printk(KERN_INFO " boxer : %s called , line %d, queue_work(boxer_panel_wq, &boxer_panel_work)\n", __FUNCTION__ , __LINE__);
		boxer_panel_dssdev = dssdev;
		queue_work(boxer_panel_wq, &boxer_panel_work);
	}
	dssdev->state=OMAP_DSS_DISPLAY_ACTIVE;
	return 0;
}

static void boxer_panel_disable(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	if (atomic_dec_and_test(&boxer_panel_is_enabled)) {
		cancel_work_sync(&boxer_panel_work);

		if (dssdev->platform_disable){
			printk(KERN_INFO " boxer : %s called , line %d, dssdev->platform_disable(dssdev);\n", __FUNCTION__ , __LINE__);
			dssdev->platform_disable(dssdev);
		}
		if (regulator_is_enabled(boxer_panel_regulator)){
			printk(KERN_INFO " boxer : %s called , line %d, regulator_disable(boxer_panel_regulator);\n", __FUNCTION__ , __LINE__);
			regulator_disable(boxer_panel_regulator);
		}
	} else {
		printk(KERN_WARNING "%s: attempting to disable panel twice!\n",
		       __func__);
		WARN_ON(1);
	}
	dssdev->state=OMAP_DSS_DISPLAY_DISABLED;
}

static int boxer_panel_suspend(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	boxer_panel_disable(dssdev);
	dssdev->state=OMAP_DSS_DISPLAY_DISABLED;
	return 0;
}

static int boxer_panel_resume(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	return boxer_panel_enable(dssdev);
}

static struct omap_dss_driver boxer_driver = {
	.probe          = boxer_panel_probe,
	.remove         = boxer_panel_remove,

	.enable         = boxer_panel_enable,
	.disable        = boxer_panel_disable,
	.suspend        = boxer_panel_suspend,
	.resume         = boxer_panel_resume,
	.get_resolution = boxer_get_resolution,
	.get_recommended_bpp = boxer_get_recommended_bpp,
	.driver		= {
		.name	= "boxer_panel",
		.owner	= THIS_MODULE,
	},
};

static ssize_t lcd_reg_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int argc;
	char **args;
	unsigned long r, val;
	int ret;

	struct spi_device *spi = to_spi_device(dev);

	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	args = argv_split(GFP_KERNEL, buf, &argc);

	if (args == NULL) {
		dev_err(dev, "error getting arguments\n");
		return count;
	}

	if (argc == 2) {
		ret = strict_strtoul(*args, 0, (unsigned long *)&r);
		if (ret)
			return ret;
		args++;
		ret = strict_strtoul(*args, 0, (unsigned long *)&val);
		if (ret)
			return ret;
		dev_info(dev, "set lcd panel spi reg %lu = %lu\n", r, val);
		spi_send(spi, r, val);
	}
	argv_free(args);

	return count;
}


static DEVICE_ATTR(lcd_reg, S_IWUSR, NULL, lcd_reg_store);

static struct attribute *boxer_lcd_spi_attributes[] = {
	&dev_attr_lcd_reg,
	NULL
};


static struct attribute_group boxer_lcd_spi_attributes_group = {
	.attrs = boxer_lcd_spi_attributes,
};



static int boxer_spi_probe(struct spi_device *spi)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 16;
	spi->chip_select=0;
	spi_setup(spi);

	boxer_spi_device = spi;

	boxer_init_panel();

	if (sysfs_create_group(&spi->dev.kobj, &boxer_lcd_spi_attributes_group))
		printk(KERN_WARNING "error creating sysfs entries\n");

	omap_dss_register_driver(&boxer_driver);
	return 0;
}

static int boxer_spi_remove(struct spi_device *spi)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	sysfs_remove_group(&spi->dev.kobj, &boxer_lcd_spi_attributes_group);
	omap_dss_unregister_driver(&boxer_driver);

	return 0;
}


static struct spi_driver boxer_spi_driver = {
	.probe           = boxer_spi_probe,
	.remove		= __devexit_p(boxer_spi_remove),
	.driver         = {
		.name   = "boxer_disp_spi",
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
	},
};

static int __init boxer_lcd_init(void)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	int ret = 0;

	boxer_panel_wq = create_singlethread_workqueue("boxer-panel-wq");


	printk(KERN_WARNING "Enabling power for LCD\n");
	boxer_panel_regulator = regulator_get(NULL, "vlcd");
	printk(KERN_INFO " boxer : %s called , line %d, regulator_get\n", __FUNCTION__ , __LINE__);

	if (IS_ERR(boxer_panel_regulator)) {
		printk(KERN_ERR "Unable to get vlcd regulator, reason: %ld!\n",
		       IS_ERR(boxer_panel_regulator));
		ret = -ENODEV;
		goto out;
	}
	
	if (g_ft_i2c_adapter) {
		i2c_lock_adapter(g_ft_i2c_adapter);
	}
	
	ret = regulator_enable(boxer_panel_regulator);
	printk(KERN_INFO " boxer : %s called , line %d, Enabling boxer panel regulator vlcd\n", __FUNCTION__ , __LINE__);
	
	if (g_ft_i2c_adapter) {
		i2c_unlock_adapter(g_ft_i2c_adapter);
	}
	if (ret) {
		printk(KERN_ERR "Failed to enable regulator vlcd!\n");
		regulator_put(boxer_panel_regulator);
		goto out;
	}

	return spi_register_driver(&boxer_spi_driver);
out:
	return ret;
}

static void __exit boxer_lcd_exit(void)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	spi_unregister_driver(&boxer_spi_driver);
	regulator_disable(boxer_panel_regulator);
	regulator_put(boxer_panel_regulator);
	destroy_workqueue(boxer_panel_wq);
}

module_init(boxer_lcd_init);
module_exit(boxer_lcd_exit);
MODULE_LICENSE("GPL");
