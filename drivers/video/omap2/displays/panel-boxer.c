/*
 * Boxer panel support
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
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/mach-types.h>

#include <plat/display.h>
#include <mach/board-nooktablet.h>

/* Delay between Panel configuration and Panel enabling */
#define LCD_RST_DELAY		100
#define LCD_INIT_DELAY		200


static struct spi_device *boxer_spi;
static struct omap_video_timings boxer_panel_timings = {
	.x_res          = 1024,
	.y_res          = 600,
	.pixel_clock    = 46000, /* in kHz */
	.hfp            = 160,
	.hsw            = 10,
	.hbp            = 160,
	.vfp            = 10,
	.vsw            = 2,
	.vbp            = 23,
};

struct panel_config {
	u32 width_in_mm;
	u32 height_in_mm;
};

static struct panel_config panel_configs[] = {
	{
		.width_in_mm = 153,
		.height_in_mm = 90,
	}
};

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

static inline struct boxer_panel_data * get_panel_data(struct omap_dss_device *dssdev)
{
	return dssdev->data;
}

static int spi_send(struct spi_device *spi,
		    unsigned char reg_addr, unsigned char reg_data)
{
	uint16_t msg = 0;
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	msg = (reg_addr << 10) | reg_data;

	if (spi_write(spi, (unsigned char *) &msg, 2))
		printk(KERN_ERR "error in spi_write %x\n", msg);

	udelay(10);

	return 0;
}

static void boxer_get_timings(struct omap_dss_device *dssdev,
                        struct omap_video_timings *timings)
{
        *timings = dssdev->panel.timings;
}

static void boxer_get_dimension(struct omap_dss_device *dssdev,
		u32 *width, u32 *height)
{
	*width = dssdev->panel.width_in_mm;
	*height = dssdev->panel.height_in_mm;
}

static int boxer_panel_probe(struct omap_dss_device *dssdev)
{
	int ret = 0;
	struct panel_config *panel_config = NULL;
	struct boxer_panel_data *panel_data = get_panel_data(dssdev);

	panel_data->vlcd = regulator_get(NULL, "vlcd");

	if (IS_ERR(panel_data->vlcd)) {
		ret = PTR_ERR(panel_data->vlcd);
		dev_err(&dssdev->dev, "failed to get vlcd regulator: %d\n", ret);
		goto err;
	}

	/* experimental setup - panel_config structure might be
	 * further changed internally if needed*/
	panel_config = &panel_configs[0];

	dssdev->panel.width_in_mm = panel_config->width_in_mm;
	dssdev->panel.height_in_mm = panel_config->height_in_mm;

	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	dssdev->panel.config	= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
				  OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IPC;
	dssdev->panel.timings	= boxer_panel_timings;
err:
	return ret;
}

static void boxer_panel_remove(struct omap_dss_device *dssdev)
{
	struct boxer_panel_data *panel_data = get_panel_data(dssdev);
	regulator_put(panel_data->vlcd);

	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
}

static int boxer_panel_start(struct omap_dss_device *dssdev)
{
	int r = 0;
	struct boxer_panel_data *panel_data = get_panel_data(dssdev);

	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);

	// If the regulator is off enable it otherwise we assume u-boot already
	// re-initialized the panel
	if (!regulator_is_enabled(panel_data->vlcd)) {
		/* lock i2c since the power on will affect i2c bus communication */
		if (g_ft_i2c_adapter) {
			i2c_lock_adapter(g_ft_i2c_adapter);
		}
		
		r = regulator_enable(panel_data->vlcd);
		mdelay(2);
		
		if (g_ft_i2c_adapter) {
			i2c_unlock_adapter(g_ft_i2c_adapter);
		}

		if (r) {
			dev_err(&dssdev->dev, "failed to enable regulator: %d\n", r);
			goto err;
		}

		if (dssdev->platform_enable) {
			r = dssdev->platform_enable(dssdev);
			if (r)
				return r;
		}

		r = omapdss_dpi_display_enable(dssdev);
		if (r && dssdev->platform_disable)
			dssdev->platform_disable(dssdev);

		msleep(LCD_RST_DELAY);
		spi_send(boxer_spi, 0x00, 0xad);
		spi_send(boxer_spi, 0x01, 0x30);
		spi_send(boxer_spi, 0x02, 0x40);
		spi_send(boxer_spi, 0x0e, 0x5f);
		spi_send(boxer_spi, 0x0f, 0xa4);
		spi_send(boxer_spi, 0x0d, 0x00);
		spi_send(boxer_spi, 0x02, 0x43);
		spi_send(boxer_spi, 0x0a, 0x28);
		spi_send(boxer_spi, 0x10, 0x41);
		msleep(LCD_INIT_DELAY); 
	} else {
		// if regulator is on this must mean first boot i.e. u-boot left the regulator on 
		// this is tracked by the enable_at_boot parameter in the regulator configuration

		// we re-initialize the dss to avoid lockups when changing display timing during video playback
		if ((r = omapdss_dpi_display_enable(dssdev))) {
			goto err;
		}
	}

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

err:
	return r;
}

static void boxer_panel_stop(struct omap_dss_device *dssdev)
{
	struct boxer_panel_data *panel_data = get_panel_data(dssdev);

	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	omapdss_dpi_display_disable(dssdev);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	// Only disable the regulator if it's enabled
	// otherwise we are going to get a warning
	if (regulator_is_enabled(panel_data->vlcd) && 
		regulator_disable(panel_data->vlcd)) {
		dev_err(&dssdev->dev, "failed to disable regulator\n");
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int boxer_panel_enable(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	return boxer_panel_start(dssdev);
}

static void boxer_panel_disable(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	boxer_panel_stop(dssdev);
}

static int boxer_panel_suspend(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);

	boxer_panel_stop(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int boxer_panel_resume(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	return boxer_panel_start(dssdev);
}

static int boxer_panel_reset(struct omap_dss_device *dssdev, enum omap_dss_reset_phase phase)
{
	int ret = 0;

	pr_warning("panel reset! reason: %d", phase);
	switch (phase) {
	case OMAP_DSS_RESET_OFF:
	case OMAP_DSS_RESET_BOTH:
	{
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
			omapdss_dpi_display_disable(dssdev);
		else
			pr_warning("reset disabled/suspended panel, state: %d", dssdev->state);

		ret = omapdss_dpi_display_enable(dssdev);

		if (ret)
			pr_warning("omapdss_dpi_display_enabled failed: %d", ret);

		break;
	}

	case OMAP_DSS_RESET_ON:
		break;
	}

	return ret;
}

static struct omap_dss_driver boxer_driver = {
	.probe		= boxer_panel_probe,
	.remove		= boxer_panel_remove,
	.enable		= boxer_panel_enable,
	.disable	= boxer_panel_disable,
	.suspend	= boxer_panel_suspend,
	.resume		= boxer_panel_resume,
	.get_timings    = boxer_get_timings,
	.set_timings    = dpi_set_timings,
	.check_timings  = dpi_check_timings,
	.get_dimension  = boxer_get_dimension,
	.reset			= boxer_panel_reset,
	.driver		= {
		.name	= "boxer_panel",
		.owner	= THIS_MODULE,
	},
};

static int boxer_spi_probe(struct spi_device *spi)
{
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 16;
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	printk(KERN_INFO "boxer: spi setup returned : %d",spi_setup(spi));

	boxer_spi = spi;

	return omap_dss_register_driver(&boxer_driver);
}

static int boxer_spi_remove(struct spi_device *spi)
{
	printk(KERN_INFO " boxer : %s called , line %d\n", __FUNCTION__ , __LINE__);
	omap_dss_unregister_driver(&boxer_driver);
	return 0;
}


static struct spi_driver boxer_spi_driver = {
	.probe		= boxer_spi_probe,
	.remove		= __devexit_p(boxer_spi_remove),
	.driver		= {
		.name	= "boxer_disp_spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
};

static int __init boxer_lcd_init(void)
{
	printk(KERN_INFO " boxer : %s called , line %d", __FUNCTION__ , __LINE__);
	return spi_register_driver(&boxer_spi_driver);
}

static void __exit boxer_lcd_exit(void)
{
	printk(KERN_INFO " boxer : %s called , line %d", __FUNCTION__ , __LINE__);
	spi_unregister_driver(&boxer_spi_driver);
}

module_init(boxer_lcd_init);
module_exit(boxer_lcd_exit);

MODULE_LICENSE("GPL");
