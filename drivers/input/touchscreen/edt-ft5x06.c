/*
 * Copyright (C) 2011 Simon Budig, &lt;simon.budig@xxxxxxxxxxxxxxxxx>
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a driver for the EDT "Polytouch" family of touch controllers
 * based on the FocalTech FT5x06 line of chips.
 *
 * Development of this driver has been sponsored by Glyn:
 *    <a rel="nofollow" href="http://www.glyn.com/Products/Displays">http://www.glyn.com/Products/Displays</a>
 */

#include <linux/module.h>;
#include <linux/interrupt.h>;
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/gpio.h>

#include <linux/input/edt-ft5x06.h>

#define DRIVER_VERSION "v0.5"

#define WORK_REGISTER_THRESHOLD   0x00
#define WORK_REGISTER_GAIN        0x30
#define WORK_REGISTER_OFFSET      0x31
#define WORK_REGISTER_NUM_X       0x33
#define WORK_REGISTER_NUM_Y       0x34

#define WORK_REGISTER_OPMODE      0x3c
#define FACTORY_REGISTER_OPMODE   0x01

struct edt_ft5x06_i2c_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	int irq;
	int irq_pin;
	int reset_pin;
	int num_x;
	int num_y;

	struct mutex mutex;
	bool factory_mode;
	int threshold;
	int gain;
	int offset;
};

static int edt_ft5x06_ts_readwrite(struct i2c_client *client,
                                   u16 wr_len, u8 *wr_buf,
                                   u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i, ret;

	i = 0;
	if (wr_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}
	if (rd_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}

	ret = i2c_transfer(client->adapter, wrmsg, i);
	if (ret < 0) {
		dev_err(&amp;client->dev, "i2c_transfer failed: %d\n", ret);
		return ret;
	}

	return ret;
}


static irqreturn_t edt_ft5x06_ts_isr(int irq, void *dev_id)
{
	struct edt_ft5x06_i2c_ts_data *tsdata = dev_id;
	unsigned char touching = 0;
	unsigned char rdbuf[26], wrbuf[1];
	int i, have_abs, type, ret;

	memset(wrbuf, 0, sizeof(wrbuf));
	memset(rdbuf, 0, sizeof(rdbuf));

	wrbuf[0] = 0xf9;

	mutex_lock(&amp;tsdata->mutex);
	ret = edt_ft5x06_ts_readwrite(tsdata->client,
	                              1, wrbuf,
	                              sizeof(rdbuf), rdbuf);
	mutex_unlock(&amp;tsdata->mutex);
	if (ret < 0) {
		dev_err(&amp;tsdata->client->dev,
		        "Unable to write to i2c touchscreen!\n");
		goto out;
	}

	if (rdbuf[0] != 0xaa || rdbuf[1] != 0xaa || rdbuf[2] != 26) {
		dev_err(&amp;tsdata->client->dev,
		        "Unexpected header: %02x%02x%02x!\n",
		        rdbuf[0], rdbuf[1], rdbuf[2]);
	}

	have_abs = 0;
	touching = rdbuf[3];
	for (i = 0; i < touching; i++) {
		type = rdbuf[i*4+5] >> 6;
		/* ignore Touch Down and Reserved events */
		if (type == 0x01 || type == 0x03)
			continue;

		if (!have_abs) {
			input_report_key(tsdata->input, BTN_TOUCH,    1);
			input_report_abs(tsdata->input, ABS_PRESSURE, 1);
			input_report_abs(tsdata->input, ABS_X,
			                 ((rdbuf[i*4+5] << 8) |
			                  rdbuf[i*4+6]) &amp; 0x0fff);
			input_report_abs(tsdata->input, ABS_Y,
			                 ((rdbuf[i*4+7] << 8) |
			                  rdbuf[i*4+8]) &amp; 0x0fff);
			have_abs = 1;
		}
		input_report_abs(tsdata->input, ABS_MT_POSITION_X,
		                 ((rdbuf[i*4+5] << 8) | rdbuf[i*4+6]) &amp; 0x0fff);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y,
		                 ((rdbuf[i*4+7] << 8) | rdbuf[i*4+8]) &amp; 0x0fff);
		input_report_abs(tsdata->input, ABS_MT_TRACKING_ID,
		                 (rdbuf[i*4+7] >> 4) &amp; 0x0f);
		input_mt_sync(tsdata->input);
	}
	if (!have_abs) {
		input_report_key(tsdata->input, BTN_TOUCH,    0);
		input_report_abs(tsdata->input, ABS_PRESSURE, 0);
	}
	input_sync(tsdata->input);

out:
	return IRQ_HANDLED;
}


static int edt_ft5x06_i2c_register_write(struct edt_ft5x06_i2c_ts_data *tsdata,
                                         u8 addr, u8 value)
{
	u8 wrbuf[4];
	int ret;

	wrbuf[0]  = tsdata->factory_mode ? 0xf3 : 0xfc;
	wrbuf[1]  = tsdata->factory_mode ? addr &amp; 0x7f : addr &amp; 0x3f;
	wrbuf[2]  = value;
	wrbuf[3]  = wrbuf[0] ^ wrbuf[1] ^ wrbuf[2];

	disable_irq(tsdata->irq);

	ret = edt_ft5x06_ts_readwrite(tsdata->client,
	                              4, wrbuf,
	                              0, NULL);

	enable_irq(tsdata->irq);

	return ret;
}

static int edt_ft5x06_i2c_register_read(struct edt_ft5x06_i2c_ts_data *tsdata,
                                        u8 addr)
{
	u8 wrbuf[2], rdbuf[2];
	int ret;

	wrbuf[0]  = tsdata->factory_mode ? 0xf3 : 0xfc;
	wrbuf[1]  = tsdata->factory_mode ? addr &amp; 0x7f : addr &amp; 0x3f;
	wrbuf[1] |= tsdata->factory_mode ? 0x80 : 0x40;

	disable_irq(tsdata->irq);

	ret = edt_ft5x06_ts_readwrite(tsdata->client,
	                              2, wrbuf,
	                              2, rdbuf);

	enable_irq(tsdata->irq);

	if ((wrbuf[0] ^ wrbuf[1] ^ rdbuf[0]) != rdbuf[1])
		dev_err(&amp;tsdata->client->dev,
		        "crc error: 0x%02x expected, got 0x%02x\n",
		        (wrbuf[0] ^ wrbuf[1] ^ rdbuf[0]), rdbuf[1]);

	return ret < 0 ? ret : rdbuf[0];
}

static ssize_t edt_ft5x06_i2c_setting_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
	struct edt_ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(dev);
	struct i2c_client *client = tsdata->client;
	int ret = 0;
	int *value;
	u8 addr;

	switch (attr->attr.name[0]) {
	case 't':    /* threshold */
		addr = WORK_REGISTER_THRESHOLD;
		value = &amp;tsdata->threshold;
		break;
	case 'g':    /* gain */
		addr = WORK_REGISTER_GAIN;
		value = &amp;tsdata->gain;
		break;
	case 'o':    /* offset */
		addr = WORK_REGISTER_OFFSET;
		value = &amp;tsdata->offset;
		break;
	default:
		dev_err(&amp;client->dev,
		        "unknown attribute for edt_ft5x06_i2c_setting_show: %s\n",
		        attr->attr.name);
		return -EINVAL;
	}

	mutex_lock(&amp;tsdata->mutex);

	if (tsdata->factory_mode) {
		dev_err(dev,
		        "setting register not available in factory mode\n");
		mutex_unlock(&amp;tsdata->mutex);
		return -EIO;
	}

	ret = edt_ft5x06_i2c_register_read(tsdata, addr);
	if (ret < 0) {
		dev_err(&amp;tsdata->client->dev,
		        "Unable to write to i2c touchscreen!\n");
		mutex_unlock(&amp;tsdata->mutex);
		return ret;
	}
	mutex_unlock(&amp;tsdata->mutex);

	if (ret != *value) {
		dev_info(&amp;tsdata->client->dev,
		         "i2c read (%d) and stored value (%d) differ. Huh?\n",
		         ret, *value);
		*value = ret;
	}

	return sprintf(buf, "%d\n", ret);
}

static ssize_t edt_ft5x06_i2c_setting_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t count)
{
	struct edt_ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(dev);
	struct i2c_client *client = tsdata->client;
	int ret = 0;
	u8 addr;
	unsigned int val;

	mutex_lock(&amp;tsdata->mutex);

	if (tsdata->factory_mode) {
		dev_err(dev,
		        "setting register not available in factory mode\n");
		ret = -EIO;
		goto out;
	}

	if (sscanf(buf, "%u", &amp;val) != 1) {
		dev_err(dev, "Invalid value for attribute %s\n",
		        attr->attr.name);
		ret = -EINVAL;
		goto out;
	}

	switch (attr->attr.name[0]) {
	case 't':    /* threshold */
		addr = WORK_REGISTER_THRESHOLD;
		val = val < 20 ? 20 : val > 80 ? 80 : val;
		tsdata->threshold = val;
		break;
	case 'g':    /* gain */
		addr = WORK_REGISTER_GAIN;
		val = val < 0 ? 0 : val > 31 ? 31 : val;
		tsdata->gain = val;
		break;
	case 'o':    /* offset */
		addr = WORK_REGISTER_OFFSET;
		val = val < 0 ? 0 : val > 31 ? 31 : val;
		tsdata->offset = val;
		break;
	default:
		dev_err(&amp;client->dev,
		        "unknown attribute for edt_ft5x06_i2c_setting_show: %s\n",
		        attr->attr.name);
		ret = -EINVAL;
		goto out;
	}

	ret = edt_ft5x06_i2c_register_write(tsdata, addr, val);

	if (ret < 0) {
		dev_err(&amp;tsdata->client->dev,
		        "Unable to write to i2c touchscreen!\n");
		goto out;
	}

out:
	mutex_unlock(&amp;tsdata->mutex);
	return ret < 0 ? ret : count;
}


static ssize_t edt_ft5x06_i2c_mode_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
	struct edt_ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", tsdata->factory_mode ? 1 : 0);
}

static ssize_t edt_ft5x06_i2c_mode_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count)
{
	struct edt_ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(dev);
	int i, ret = 0;
	unsigned int mode;

	if (sscanf(buf, "%u", &amp;mode) != 1 || (mode | 1) != 1) {
		dev_err(dev, "Invalid value for operation mode\n");
		return -EINVAL;
	}

	/* no change, return without doing anything */
	if (mode == tsdata->factory_mode)
		return count;

	mutex_lock(&amp;tsdata->mutex);
	if (!tsdata->factory_mode) { /* switch to factory mode */
		disable_irq(tsdata->irq);
		/* mode register is 0x3c when in the work mode */
		ret = edt_ft5x06_i2c_register_write(tsdata,
		                                    WORK_REGISTER_OPMODE, 0x03);
		if (ret < 0) {
			dev_err(dev, "failed to switch to factory mode (%d)\n",
			        ret);
		} else {
			tsdata->factory_mode = 1;
			for (i = 0; i < 10; i++) {
				mdelay(5);
				/* mode register is 0x01 when in factory mode */
				ret = edt_ft5x06_i2c_register_read(tsdata, FACTORY_REGISTER_OPMODE);
				if (ret == 0x03)
					break;
			}
			if (i == 10)
				dev_err(dev,
				        "not in factory mode after %dms.\n",
				        i*5);
		}
	} else {  /* switch to work mode */
		/* mode register is 0x01 when in the factory mode */
		ret = edt_ft5x06_i2c_register_write(tsdata,
		                                    FACTORY_REGISTER_OPMODE,
		                                    0x01);
		if (ret < 0) {
			dev_err(dev, "failed to switch to work mode (%d)\n",
			        ret);
		} else {
			tsdata->factory_mode = 0;
			for (i = 0; i < 10; i++) {
				mdelay(5);
				/* mode register is 0x01 when in factory mode */
				ret = edt_ft5x06_i2c_register_read(tsdata, WORK_REGISTER_OPMODE);
				if (ret == 0x01)
					break;
			}
			if (i == 10)
				dev_err(dev, "not in work mode after %dms.\n",
				        i*5);

			/* restore parameters */
			edt_ft5x06_i2c_register_write(tsdata,
			                              WORK_REGISTER_THRESHOLD,
			                              tsdata->threshold);
			edt_ft5x06_i2c_register_write(tsdata,
			                              WORK_REGISTER_GAIN,
			                              tsdata->gain);
			edt_ft5x06_i2c_register_write(tsdata,
			                              WORK_REGISTER_OFFSET,
			                              tsdata->offset);

			enable_irq(tsdata->irq);
		}
	}

	mutex_unlock(&amp;tsdata->mutex);
	return count;
}


static ssize_t edt_ft5x06_i2c_raw_data_show(struct device *dev,
                                            struct device_attribute *attr,
                                            char *buf)
{
	struct edt_ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(dev);
	int i, ret;
	char *ptr, wrbuf[3];

	if (!tsdata->factory_mode) {
		dev_err(dev, "raw data not available in work mode\n");
		return -EIO;
	}

	mutex_lock(&amp;tsdata->mutex);
	ret = edt_ft5x06_i2c_register_write(tsdata, 0x08, 0x01);
	for (i = 0; i < 100; i++) {
		ret = edt_ft5x06_i2c_register_read(tsdata, 0x08);
		if (ret < 1)
			break;
		udelay(1000);
	}

	if (i == 100 || ret < 0) {
		dev_err(dev, "waiting time exceeded or error: %d\n", ret);
		mutex_unlock(&amp;tsdata->mutex);
		return ret < 0 ? ret : -ETIMEDOUT;
	}

	ptr = buf;
	wrbuf[0] = 0xf5;
	wrbuf[1] = 0x0e;
	for (i = 0; i <= tsdata->num_x; i++) {
		wrbuf[2] = i;
		ret = edt_ft5x06_ts_readwrite(tsdata->client,
		                               3, wrbuf,
		                               tsdata->num_y * 2, ptr);
		if (ret < 0) {
			mutex_unlock(&amp;tsdata->mutex);
			return ret;
		}

		ptr += tsdata->num_y * 2;
	}

	mutex_unlock(&amp;tsdata->mutex);
	return ptr - buf;
}


static DEVICE_ATTR(gain,      0664,
                   edt_ft5x06_i2c_setting_show, edt_ft5x06_i2c_setting_store);
static DEVICE_ATTR(offset,    0664,
                   edt_ft5x06_i2c_setting_show, edt_ft5x06_i2c_setting_store);
static DEVICE_ATTR(threshold, 0664,
                   edt_ft5x06_i2c_setting_show, edt_ft5x06_i2c_setting_store);
static DEVICE_ATTR(mode,      0664,
                   edt_ft5x06_i2c_mode_show, edt_ft5x06_i2c_mode_store);
static DEVICE_ATTR(raw_data,  0444,
                   edt_ft5x06_i2c_raw_data_show, NULL);

static struct attribute *edt_ft5x06_i2c_attrs[] = {
	&amp;dev_attr_gain.attr,
	&amp;dev_attr_offset.attr,
	&amp;dev_attr_threshold.attr,
	&amp;dev_attr_mode.attr,
	&amp;dev_attr_raw_data.attr,
	NULL
};

static const struct attribute_group edt_ft5x06_i2c_attr_group = {
	.attrs = edt_ft5x06_i2c_attrs,
};

static int edt_ft5x06_i2c_ts_probe(struct i2c_client *client,
                                   const struct i2c_device_id *id)
{

	struct edt_ft5x06_i2c_ts_data *tsdata;
	struct edt_ft5x06_platform_data *pdata;
	struct input_dev *input;
	int error;
	u8 rdbuf[23];
	char *model_name, *fw_version;

	dev_dbg(&amp;client->dev, "probing for EDT FT5x06 I2C\n");

	if (!client->irq) {
		dev_err(&amp;client->dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!client->dev.platform_data) {
		dev_err(&amp;client->dev, "no platform data?\n");
		return -ENODEV;
	}

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&amp;client->dev, "failed to allocate driver data!\n");
		dev_set_drvdata(&amp;client->dev, NULL);
		return -ENOMEM;
	}

	dev_set_drvdata(&amp;client->dev, tsdata);
	tsdata->client = client;
	pdata = client->dev.platform_data;

	tsdata->reset_pin = pdata->reset_pin;
	mutex_init(&amp;tsdata->mutex);

	if (tsdata->reset_pin >= 0) {
		error = gpio_request(tsdata->reset_pin, NULL);
		if (error < 0) {
			dev_err(&amp;client->dev,
			        "Failed to request GPIO %d as reset pin, error %d\n",
			         tsdata->reset_pin, error);
			error = -ENOMEM;
			goto err_free_tsdata;
		}

		/* this pulls reset down, enabling the low active reset */
		if (gpio_direction_output(tsdata->reset_pin, 0) < 0) {
			dev_info(&amp;client->dev, "switching to output failed\n");
			goto err_free_reset_pin;
		}
	}

	/* request IRQ pin */
	tsdata->irq_pin = pdata->irq_pin;
	tsdata->irq = gpio_to_irq(tsdata->irq_pin);

	error = gpio_request(tsdata->irq_pin, NULL);
	if (error < 0) {
		dev_err(&amp;client->dev,
		        "Failed to request GPIO %d for IRQ %d, error %d\n",
		        tsdata->irq_pin, tsdata->irq, error);
		goto err_free_reset_pin;
	}
	gpio_direction_input(tsdata->irq_pin);

	if (tsdata->reset_pin >= 0) {
		/* release reset */
		mdelay(50);
		gpio_set_value(tsdata->reset_pin, 1);
		mdelay(100);
	}

	mutex_lock(&amp;tsdata->mutex);

	tsdata->factory_mode = 0;

	if (edt_ft5x06_ts_readwrite(client, 1, "\xbb", 22, rdbuf) < 0) {
		dev_err(&amp;client->dev, "probing failed\n");
		error = -ENODEV;
		goto err_free_irq_pin;
	}

	tsdata->threshold = edt_ft5x06_i2c_register_read(tsdata,
	                                                 WORK_REGISTER_THRESHOLD);
	tsdata->gain      = edt_ft5x06_i2c_register_read(tsdata,
	                                                 WORK_REGISTER_GAIN);
	tsdata->offset    = edt_ft5x06_i2c_register_read(tsdata,
	                                                 WORK_REGISTER_OFFSET);
	tsdata->num_x     = edt_ft5x06_i2c_register_read(tsdata,
	                                                 WORK_REGISTER_NUM_X);
	tsdata->num_y     = edt_ft5x06_i2c_register_read(tsdata,
	                                                 WORK_REGISTER_NUM_Y);

	mutex_unlock(&amp;tsdata->mutex);

	/* remove last '$' end marker */
	rdbuf[22] = '\0';
	if (rdbuf[21] == '$')
		rdbuf[21] = '\0';

	model_name = rdbuf + 1;
	/* look for Model/Version separator */
	fw_version = strchr(rdbuf, '*');

	if (fw_version) {
		fw_version[0] = '\0';
		fw_version++;
		dev_info(&amp;client->dev,
		         "Model \"%s\", Rev. \"%s\", %dx%d sensors\n",
		         model_name, fw_version, tsdata->num_x, tsdata->num_y);
	} else {
		dev_info(&amp;client->dev, "Product ID \"%s\"\n", model_name);
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&amp;client->dev, "failed to allocate input device!\n");
		error = -ENOMEM;
		goto err_free_irq_pin;
	}

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, tsdata->num_x * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, tsdata->num_y * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X,
	                     0, tsdata->num_x * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
	                     0, tsdata->num_y * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 15, 0, 0);

	input->name = kstrdup(model_name, GFP_NOIO);
	input->id.bustype = BUS_I2C;
	input->dev.parent = &amp;client->dev;

	input_set_drvdata(input, tsdata);

	tsdata->input = input;

	error = input_register_device(input);
	if (error)
		goto err_free_input_device;

	if (request_threaded_irq(tsdata->irq, NULL, edt_ft5x06_ts_isr,
	                         IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	                         client->name, tsdata)) {
		dev_err(&amp;client->dev, "Unable to request touchscreen IRQ.\n");
		input = NULL;
		error = -ENOMEM;
		goto err_unregister_device;
	}

	error = sysfs_create_group(&amp;client->dev.kobj,
	                           &amp;edt_ft5x06_i2c_attr_group);
	if (error)
		goto err_free_irq;

	device_init_wakeup(&amp;client->dev, 1);

	dev_dbg(&amp;tsdata->client->dev,
	        "EDT FT5x06 initialized: IRQ pin %d, Reset pin %d.\n",
	        tsdata->irq_pin, tsdata->reset_pin);

	return 0;

err_free_irq:
	free_irq(client->irq, tsdata);
err_unregister_device:
	input_unregister_device(input);
	input = NULL;
err_free_input_device:
	if (input) {
		kfree(input->name);
		input_free_device(input);
	}
err_free_irq_pin:
	gpio_free(tsdata->irq_pin);
err_free_reset_pin:
	if (tsdata->reset_pin >= 0)
		gpio_free(tsdata->reset_pin);
err_free_tsdata:
	kfree(tsdata);
	return error;
}

static int edt_ft5x06_i2c_ts_remove(struct i2c_client *client)
{
	struct edt_ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(&amp;client->dev);

	sysfs_remove_group(&amp;client->dev.kobj, &amp;edt_ft5x06_i2c_attr_group);

	free_irq(client->irq, tsdata);
	kfree(tsdata->input->name);
	input_unregister_device(tsdata->input);
	gpio_free(tsdata->irq_pin);
	if (tsdata->reset_pin >= 0)
		gpio_free(tsdata->reset_pin);
	kfree(tsdata);

	return 0;
}

static int edt_ft5x06_i2c_ts_suspend(struct i2c_client *client,
                                     pm_message_t mesg)
{
	struct edt_ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(&amp;client->dev);

	if (device_may_wakeup(&amp;client->dev))
		enable_irq_wake(tsdata->irq);

	return 0;
}

static int edt_ft5x06_i2c_ts_resume(struct i2c_client *client)
{
	struct edt_ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(&amp;client->dev);

	if (device_may_wakeup(&amp;client->dev))
		disable_irq_wake(tsdata->irq);

	return 0;
}

static const struct i2c_device_id edt_ft5x06_i2c_ts_id[] = {
	{ "edt-ft5x06", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, edt_ft5x06_i2c_ts_id);

static struct i2c_driver edt_ft5x06_i2c_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "edt_ft5x06_i2c",
	},
	.id_table = edt_ft5x06_i2c_ts_id,
	.probe    = edt_ft5x06_i2c_ts_probe,
	.remove   = edt_ft5x06_i2c_ts_remove,
	.suspend  = edt_ft5x06_i2c_ts_suspend,
	.resume   = edt_ft5x06_i2c_ts_resume,
};

static int __init edt_ft5x06_i2c_ts_init(void)
{
	return i2c_add_driver(&amp;edt_ft5x06_i2c_ts_driver);
}
module_init(edt_ft5x06_i2c_ts_init);

static void __exit edt_ft5x06_i2c_ts_exit(void)
{
	i2c_del_driver(&amp;edt_ft5x06_i2c_ts_driver);
}
module_exit(edt_ft5x06_i2c_ts_exit);

MODULE_AUTHOR("Simon Budig <simon.budig@xxxxxxxxxxxxxxxxx>");
MODULE_DESCRIPTION("EDT FT5x06 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
