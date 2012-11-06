/*
 * Boxer (Kindle Fire Version) panel support
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Copyright (c) 2010 Barnes & Noble
 * David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>
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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/mach-types.h>
#include <video/omapdss.h>
#include <linux/gpio.h>

//#define DEBUG
/* Delay between Panel configuration and Panel enabling */
#define LCD_RST_DELAY		100
#define OMAP_FT5x06_POWER_GPIO  36

static bool first_boot = true;
static struct spi_device *boxer_spi_device;

static int spi_send(struct spi_device *spi,
		    unsigned char reg_addr, unsigned char reg_data)
{
	uint16_t msg = 0;
	int result = 0;
	msg = (reg_addr << 10) | reg_data;
	result = spi_write(spi, (unsigned char *)&msg, 2);
	if (result != 0)
		printk(KERN_ERR "error in spi_write %x\n", msg);
	else
		udelay(10);

	return result;
}

static ssize_t set_cabc(struct device *dev, struct device_attribute *attr,
			const char *buffer, size_t count)
{
	int mode = 0;
	sscanf(buffer, "%d", &mode);
	printk("the cabc value is %d\n", mode);
	switch (mode) {
	case 0:
		printk("set the cabc value to 0b00\n");
		spi_send(boxer_spi_device, 0x01, 0x30);
		break;
	case 1:
		printk("set the cabc value to 0b01\n");
		spi_send(boxer_spi_device, 0x01, 0x70);
		break;
	case 2:
		printk("set the cabc value to 0b10\n");
		spi_send(boxer_spi_device, 0x01, 0xb0);
		break;
	case 3:
		printk("set the cabc value to 0b11\n");
		spi_send(boxer_spi_device, 0x01, 0xf0);
		break;
	default:
		printk("set the cabc value to 0b00\n");
		spi_send(boxer_spi_device, 0x01, 0x30);
		break;
	}
	return count;
}

DEVICE_ATTR(cabc, S_IRUGO | S_IWUGO, NULL, set_cabc);

static struct attribute *otter1_panel_attributes[] = {
	&dev_attr_cabc.attr,
	NULL
};

static struct attribute_group otter1_panel_attribute_group = {
	.attrs = otter1_panel_attributes
};

static void boxer_panel_spi_init (void)
{
	printk	("%s\n", __FUNCTION__);

	gpio_direction_output (OMAP_FT5x06_POWER_GPIO, 1);
	msleep(LCD_RST_DELAY);
	spi_send(boxer_spi_device, 0x00, 0x21);
	spi_send(boxer_spi_device, 0x00, 0xa5);
	spi_send(boxer_spi_device, 0x01, 0x30);
	spi_send(boxer_spi_device, 0x02, 0x40);
	spi_send(boxer_spi_device, 0x0e, 0x5f);
	spi_send(boxer_spi_device, 0x0f, 0xa4);
	spi_send(boxer_spi_device, 0x0d, 0x00);
	spi_send(boxer_spi_device, 0x02, 0x43);
	spi_send(boxer_spi_device, 0x0a, 0x28);
	spi_send(boxer_spi_device, 0x10, 0x41);
	spi_send(boxer_spi_device, 0x00, 0xad);
}

static void boxer_get_resolution(struct omap_dss_device *dssdev,
				 u16 * xres, u16 * yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static int boxer_panel_probe(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__,
	       __LINE__);

	omap_writel(0x00020000, 0x4a1005cc);	//PCLK impedance
	return 0;
}

static void boxer_panel_remove(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n",
	       __FUNCTION__ , __LINE__);
}

static int boxer_panel_start(struct omap_dss_device *dssdev)
{
	int r = 0;

	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__,
	       __LINE__);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	printk(KERN_INFO " omapdss_dpi_display_enable == %d\n", r);
	if (r)
		goto err0;

	return 0;
err0:
	return r;
}

static void boxer_panel_stop(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__,
	       __LINE__);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	msleep(350);
	gpio_direction_output (OMAP_FT5x06_POWER_GPIO, 0);

	omapdss_dpi_display_disable(dssdev);
}

static int boxer_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__,
	       __LINE__);
	r = boxer_panel_start(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void boxer_panel_disable(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__,
	       __LINE__);
	boxer_panel_stop(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int boxer_panel_suspend(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__,
	       __LINE__);

	if (first_boot) {
		first_boot = false;
		return 0;
	}

	boxer_panel_stop(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int boxer_panel_resume(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__,
	       __LINE__);

	if (first_boot) {
		first_boot = false;
		return;
	}
	else {
		boxer_panel_spi_init ();
	}

	boxer_panel_start(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return 0;
}

static void boxer_panel_set_timings(struct omap_dss_device *dssdev,
				    struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void boxer_panel_get_timings(struct omap_dss_device *dssdev,
				    struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int boxer_panel_check_timings(struct omap_dss_device *dssdev,
				     struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver boxer_driver = {
	.probe = boxer_panel_probe,
	.remove = boxer_panel_remove,

	.enable = boxer_panel_enable,
	.disable = boxer_panel_disable,
	.suspend = boxer_panel_suspend,
	.resume = boxer_panel_resume,

	.get_resolution = boxer_get_resolution,

	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.set_timings = boxer_panel_set_timings,
	.get_timings = boxer_panel_get_timings,
	.check_timings = boxer_panel_check_timings,

	.driver = {
		   .name = "boxer_panel",
		   .owner = THIS_MODULE,
		   },
};

static int boxer_spi_probe(struct spi_device *spi)
{
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 16;
	spi->chip_select = 0;
	boxer_spi_device = spi;
	spi_setup(boxer_spi_device);

	if (sysfs_create_group(&spi->dev.kobj, &otter1_panel_attribute_group))
		printk ("%s problem creating sysfs group\n", __FUNCTION__);

	if (gpio_request(OMAP_FT5x06_POWER_GPIO,
			 "ft5x06_touch_power") < 0) {
		printk(KERN_ERR "can't get ft5x06 power GPIO\n");
		return -1;
	}

	boxer_panel_spi_init ();

	return omap_dss_register_driver(&boxer_driver);
}

static int boxer_spi_remove(struct spi_device *spi)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__,
	       __LINE__);
	sysfs_remove_group(&spi->dev.kobj, &otter1_panel_attribute_group);
	omap_dss_unregister_driver(&boxer_driver);

	return 0;
}

static struct spi_driver boxer_spi_driver = {
	.probe = boxer_spi_probe,
	.remove = __devexit_p(boxer_spi_remove),
	.driver = {
#ifndef CONFIG_MACH_OMAP4_NOOKTABLET
		   .name = "otter1_disp_spi",
#else
		   .name = "boxer_disp_spi",
#endif
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
};

static int __init boxer_lcd_init(void)
{
	return spi_register_driver(&boxer_spi_driver);
}

static void __exit boxer_lcd_exit(void)
{
	spi_unregister_driver(&boxer_spi_driver);
}

module_init(boxer_lcd_init);
module_exit(boxer_lcd_exit);

MODULE_LICENSE("GPL");
