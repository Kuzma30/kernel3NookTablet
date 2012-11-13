/* Source for:
 * Focaltech 5x06 touch screen controller. Some parts of the code are based on the Cypress
 * TrueTouch(TM) Standard Product I2C touchscreen driver. This driver can be found here:
 * drivers/input/touchscreen/cyttsp-i2c.c
 *
 * drivers/input/touchscreen/ft5x06-i2c.c
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * Copyright (c) 2010 Barnes & Noble.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include <linux/firmware.h>
#include <linux/input/ft5x06.h>

#define FT5x06_CPTM_ID_COMPANY        0x79
#define FT5x06_CPTM_ID_PRODUCT        0x03

#define FT5x06_NUM_TX                   28
#define FT5x06_NUM_RX                   16

#define FT5x06_PACKET_LENGTH           128
#define FT5x06_VERIFY_NUM_TRIES         10
#define FT5x06_CALIBRATION_NUM_BYTES     8
#define FT5x06_SCAN_DELAY               20

#define FT5x06_CROSSTALK_TEST_TYPE_EVEN  0
#define FT5x06_CROSSTALK_TEST_TYPE_ODD   1

#define FT5x06_ERR_NOT_FACTORY_MODE      1
#define FT5x06_ERR_NOT_WORKING_MODE      2
#define FT5x06_ERR_SCAN_NOT_DONE         3
#define FT5x06_ERR_INVALID_ID            4
#define FT5x06_ERR_INVALID_CHECKSUM      5

#define FT5x06_DRIVER_VERSION       0x0001

#define FT5x06_UPGRADEVER_REG		0xCD

#define FT5x06_DEBUG_VERBOSE             1

#define GET_COORDINATE(l,h) ((l | (( h & 0x0F)<<8)))
struct ft5x06 {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	//struct timer_list timer;
	char phys[32];
	struct ft5x06_platform_data *platform_data;
	u8 prv_tch;
	u16 prv_mt_pos[FT_NUM_TRK_ID][2];
	u8 crosstalk_test_type;
	u8 factory_mode_register;
	u8 working_mode_register;
	atomic_t irq_enabled;
	/* Ensures that only one function can specify the Device Mode at a time. */
	struct mutex device_mode_mutex;
	struct early_suspend early_suspend;
};

typedef struct ft506_touch_data {
	unsigned char x_h;
	unsigned char x_l;
	unsigned char y_h;
	unsigned char y_l;
	unsigned char pressure;
	unsigned char area;
} point_data;

typedef struct ft506 {
	unsigned char mode;
	unsigned char gesture_id;
	unsigned char status;
	point_data points[FT_NUM_MT_TCH_ID];
} ft506_data;

/*****************************************************************************
 * Function Prototypes
 ****************************************************************************/

static void ft5x06_xy_worker(struct work_struct *work);

static irqreturn_t ft5x06_irq(int irq, void *handle);

static int ft5x06_inlist(u16 prev_track[], u8 cur_trk_id, u8 * prev_loc,
			 u8 num_touches);
static int ft5x06_next_avail_inlist(u16 cur_trk[], u8 * new_loc,
				    u8 num_touches);

static int __devinit ft5x06_probe(struct i2c_client *client,
				  const struct i2c_device_id *id);
static int __devexit ft5x06_remove(struct i2c_client *client);

static int ft5x06_resume(struct i2c_client *client);
static int ft5x06_suspend(struct i2c_client *client, pm_message_t message);

static void ft5x06_early_suspend(struct early_suspend *handler);
static void ft5x06_late_resume(struct early_suspend *handler);

extern int ft5x06_dev_init(int resource);

/*****************************************************************************
 * Global Variables
 ****************************************************************************/

static struct workqueue_struct *ft5x06_ts_wq;

static struct ft5x06_xydata_t g_xy_data;

static const struct i2c_device_id ft5x06_id[] = {
	{FT_I2C_NAME, 0}, {}
};

MODULE_DEVICE_TABLE(i2c, ft5x06_id);

static struct i2c_driver ft5x06_driver = {
	.driver = {
		   .name = FT_I2C_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = ft5x06_probe,
	.remove = __devexit_p(ft5x06_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = NULL,
	.resume = NULL,
#else /* CONFIG_HAS_EARLYSUSPEND */
	.suspend = ft5x06_suspend,
	.resume = ft5x06_resume,
#endif /* CONFIG_HAS_EARLYSUSPEND */
	.id_table = ft5x06_id,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Focaltech 5x06 touchscreen driver");
MODULE_AUTHOR("B&N");

static ssize_t ft5x06_irq_status(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%u\n", atomic_read(&ts->irq_enabled));
}

static ssize_t ft5x06_irq_enable(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = i2c_get_clientdata(client);

	int err = 0;
	unsigned long value;

	if (size > 2) {
		return -EINVAL;
	}

	err = strict_strtoul(buf, 10, &value);
	if (err != 0) {
		return err;
	}

	switch (value) {
	case 0:
		if (atomic_cmpxchg(&ts->irq_enabled, 1, 0)) {
			printk(KERN_INFO
			       "%s() - Touch Panel IRQ %u has been DISABLED.\n",
			       __FUNCTION__, ts->client->irq);
			disable_irq(ts->client->irq);
		}
		err = size;
		break;

	case 1:
		if (!atomic_cmpxchg(&ts->irq_enabled, 0, 1)) {
			printk(KERN_INFO
			       "%s() - Touch Panel IRQ %u has been ENABLED.\n",
			       __FUNCTION__, ts->client->irq);
			enable_irq(ts->client->irq);
		}
		err = size;
		break;

	default:
		printk(KERN_ERR
		       "%s() - Invalid input specified (%lu). Touch Panel IRQ %u --> irq_enabled = %d\n",
		       __FUNCTION__, value, ts->client->irq,
		       atomic_read(&ts->irq_enabled));
		err = -EINVAL;
		break;
	}

	return err;
}

static DEVICE_ATTR(irq_enable, 0777, ft5x06_irq_status, ft5x06_irq_enable);

/* The ft5x06_xy_worker function reads the XY coordinates and sends them to
 * the input layer.  It is scheduled from the interrupt (or timer).
 */
void ft5x06_xy_worker(struct work_struct *work)
{
	struct ft5x06 *ts = container_of(work, struct ft5x06, work);
	int retval = 0;
	ft506_data tch_data;
	u8 _id;
	u8 id, tilt, rev_x, rev_y;
	u8 cur_tch;		/* number of current touches */
	u8 curr_tool_width;
	u8 event = 0;
	u16 x = 0;
	u16 y = 0;
	static u8 prev_gest = 0;
	static u8 gest_count = 0;

	g_xy_data.gest_id = 0;

	retval =
		i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE,
										sizeof(ft506_data), (u8 *)&tch_data);
	if (retval < 0) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read from the Touch Panel registers.\n",
		       __FUNCTION__);
	} else {
		g_xy_data.gest_id = tch_data.gesture_id;
	}

	/* some firmwares dublicate data in high bits so we use only 4 low bits*/
	cur_tch = tch_data.status & 0xf;

	if (cur_tch > FT_NUM_MT_TCH_ID) {
		cur_tch = FT_NUM_MT_TCH_ID;
	}

	/* set tool size */
	curr_tool_width = FT_SMALL_TOOL_WIDTH;

	/* Determine if display is tilted */
	if (FLIP_DATA(ts->platform_data->flags)) {
		tilt = true;
	} else {
		tilt = false;
	}

	/* Check for switch in origin */
	if (REVERSE_X(ts->platform_data->flags)) {
		rev_x = true;
	} else {
		rev_x = false;
	}

	if (REVERSE_Y(ts->platform_data->flags)) {
		rev_y = true;
	} else {
		rev_y = false;
	}

	u8 touches = 0;
	/* process the touches */
	for(id = 0; id < FT_NUM_MT_TCH_ID; id++)
	{
		_id = (tch_data.points[id].y_h>>4);
         event = tch_data.points[id].x_h >> 6;
         if (event == FT_EVENT_DOWN || event == FT_EVENT_MOVE) {
         	x = GET_COORDINATE(tch_data.points[id].x_l, tch_data.points[id].x_h);
	        y = GET_COORDINATE(tch_data.points[id].y_l, tch_data.points[id].y_h);

	        if (tilt)
	        {
	            FLIP_XY(x, y);
	        }
	        if (ts->platform_data->maxx != ts->platform_data->rawx) {
	        	x = x * ts->platform_data->maxx / ts->platform_data->rawx;
	        }
	        if (ts->platform_data->maxy != ts->platform_data->rawy) {
	        	y = y * ts->platform_data->maxy / ts->platform_data->rawy;
	        }
	        if (rev_x)
	        {
	            x = INVERT_X(x, ts->platform_data->maxx);
	        }
	        if (rev_y)
	        {
	            y = INVERT_Y(y, ts->platform_data->maxy-1);
	        }
	        touches++;
	        /* Fix sluggish scrolling */
	        if (x == ts->prv_mt_pos[_id][FT_XPOS] &&
	        	y == ts->prv_mt_pos[_id][FT_YPOS] && cur_tch == 1 && ts->prv_tch == cur_tch) {
	        	continue;
	        }
	        input_report_abs(ts->input, ABS_MT_PRESSURE, tch_data.points[id].pressure);
         	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, tch_data.points[id].area  >> 4);
			input_report_abs(ts->input, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
	        input_mt_sync(ts->input);
	        ts->prv_mt_pos[_id][FT_XPOS] = x;
	        ts->prv_mt_pos[_id][FT_YPOS] = y;
         } else if (event == FT_EVENT_UP) {
         	ts->prv_mt_pos[_id][FT_XPOS] = -1;
	        ts->prv_mt_pos[_id][FT_YPOS] = -1;
	        input_mt_sync(ts->input);
         }
	}

	input_report_key(ts->input, BTN_TOUCH, touches > 0);

	/* handle gestures */
	if (ts->platform_data->use_gestures) {
		if (g_xy_data.gest_id && touches) {
			if (prev_gest == g_xy_data.gest_id) {
				gest_count++;
			} else {
				gest_count = 0;
			}

			/* ACCLPLAT-688 Do not report key event */
			//input_report_key(ts->input, BTN_3, FT_TCH);

			input_report_abs(ts->input, ABS_HAT1X,
					 g_xy_data.gest_id);
			input_report_abs(ts->input, ABS_HAT2X, tch_data.status);
			input_report_abs(ts->input, ABS_HAT2Y, gest_count);

			prev_gest = g_xy_data.gest_id;
		} else {
			gest_count = 0;
		}
	}

	/* signal the view motion event */
	input_sync(ts->input);

	ts->prv_tch = cur_tch;

	return;
}

static int ft5x06_inlist(u16 prev_track[], u8 cur_trk_id, u8 * prev_loc,
			 u8 num_touches)
{
	u8 id = 0;

	*prev_loc = FT_IGNR_TCH;

	for (id = 0, *prev_loc = FT_IGNR_TCH; (id < num_touches); id++) {
		if (prev_track[id] == cur_trk_id) {
			*prev_loc = id;
			break;
		}
	}

	return ((*prev_loc < FT_NUM_TRK_ID) ? true : false);
}

static int ft5x06_next_avail_inlist(u16 cur_trk[], u8 * new_loc, u8 num_touches)
{
	u8 id;

	for (id = 0, *new_loc = FT_IGNR_TCH; (id < num_touches); id++) {
		if (cur_trk[id] > FT_NUM_TRK_ID) {
			*new_loc = id;
			break;
		}
	}

	return ((*new_loc < FT_NUM_TRK_ID) ? true : false);
}

/*************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function
 ************************************************************************/
static irqreturn_t ft5x06_irq(int irq, void *handle)
{
	struct ft5x06 *ts = (struct ft5x06 *)handle;

	//printk("%s : got irq!\n",__FUNCTION__);

	/* disable further interrupts until this interrupt is processed */
	// disable_irq_nosync(ts->client->irq);

	/* schedule motion signal handling */
	if (!work_pending(&ts->work)) {
		queue_work(ft5x06_ts_wq, &ts->work);
	}
	return IRQ_HANDLED;
}

/*************************************************************************
 * GPIO Helper Functions
 ************************************************************************/

static void ft5x06_reset_panel_via_gpio(int reset_gpio)
{
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Toggling GPIO %d to reset the Touch Panel...\n",
	       __FUNCTION__, reset_gpio);
#endif /* FT5x06_DEBUG_VERBOSE */

	gpio_direction_output(reset_gpio, 1);
	msleep(20);
	gpio_direction_output(reset_gpio, 0);
	msleep(20);
	gpio_direction_output(reset_gpio, 1);
	msleep(300);
}

static bool ft5x06_poll_gpio(const int gpio_num, const int requested_gpio_val)
{
	const int poll_count_limit = 20;
	const int poll_delay_ms = 5;
	int poll_count = 0;
	int gpio_val = -1;

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Waiting for GPIO %d to go to %d...\n",
	       __FUNCTION__, gpio_num, requested_gpio_val);
#endif /* FT5x06_DEBUG_VERBOSE */

	while ((poll_count_limit > poll_count++)
	       && (requested_gpio_val != gpio_val)) {
		msleep(poll_delay_ms);
		gpio_val = gpio_get_value(gpio_num);
	}

	return (requested_gpio_val == gpio_val);
}

/* Set the TP controller to Firmware update mode return 0 for success
 * caller needs to reset TP for exit update mode */
static int ft5x06_enter_fwupdate_mode(struct ft5x06 *ts)
{
	int retval = 0;
	int i = 0;
	u8 temp_buffer[4];

	printk(KERN_INFO "%s() - Step 1: Reset the CTPM\n", __FUNCTION__);
	retval =
	    i2c_smbus_write_byte_data(ts->client, FT5x06_WMREG_RESET,
				      FT5x06_CMD_RESET_CTPM_H);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write the CTPM Reset command (high byte) to the CTPM.\n",
		       __FUNCTION__);
		goto error_return;
	}

	msleep(50);

	retval =
	    i2c_smbus_write_byte_data(ts->client, FT5x06_WMREG_RESET,
				      FT5x06_CMD_RESET_CTPM_L);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write the CTPM Reset command (low byte) to the CTPM.\n",
		       __FUNCTION__);
		goto error_return;
	}

	msleep(30);

	printk(KERN_INFO
	       "%s() - Step 2: Put the CTPM in Firmware Upgrade mode\n",
	       __FUNCTION__);
	temp_buffer[0] = 0x55;
	temp_buffer[1] = 0xAA;

	for (i = 0; ((i < 5) && (0 >= retval)); i++) {
		struct i2c_msg msg = {
			.addr = ts->client->addr,
			.flags = 0,
			.len = 2,
			.buf = temp_buffer,
		};
		msleep(5);

		retval = i2c_transfer(ts->client->adapter, &msg, 1);
	}

	if (0 >= retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the CTPM in Firmware Update mode.\n",
		       __FUNCTION__);
		goto error_return;
	}
	return 0;

error_return:
	return -1;
}

/*************************************************************************
 * Factory Mode Helper Functions
 ************************************************************************/

static int ft5x06_enter_factory_mode(struct ft5x06 *ts)
{
	int retval = 0;
	u8 regval = 0;

	if (ts->client->irq) {
#if FT5x06_DEBUG_VERBOSE
		printk(KERN_DEBUG
		       "%s() - Disabling Touch Panel interrupts...\n",
		       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */
		disable_irq(ts->client->irq);
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Putting the Touch Panel in Factory Mode...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	regval = FT5x06_MODE_FACTORY;
	retval =
	    i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write to the Device Mode register.\n",
		       __FUNCTION__);
		goto error_enable_irq;
	}

	msleep(100);

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Verifying that the Touch Panel is in Factory Mode...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read from the Device Mode register.\n",
		       __FUNCTION__);
		goto error_enable_irq;
	}

	if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_FACTORY) {
		printk(KERN_ERR
		       "%s() - ERROR: The Touch Panel was not put in Factory Mode. The Device Mode register contains 0x%02X\n",
		       __FUNCTION__, regval);
		retval = FT5x06_ERR_NOT_FACTORY_MODE;
		goto error_enable_irq;
	}

	return 0;

error_enable_irq:
	if (ts->client->irq) {
#if FT5x06_DEBUG_VERBOSE
		printk(KERN_DEBUG "%s() - Enabling Touch Panel interrupts.\n",
		       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */
		enable_irq(ts->client->irq);
	}

	return retval;
}

static int ft5x06_exit_factory_mode(struct ft5x06 *ts)
{
	int retval = 0;
	u8 regval = 0;

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Putting the Touch Panel in Working Mode...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	regval = FT5x06_MODE_WORKING;
	retval =
	    i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write to the Device Mode register.\n",
		       __FUNCTION__);
		goto error_enable_irq;
	}

	msleep(100);

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Verifying that the Touch Panel is in Working Mode...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read from the Device Mode register.\n",
		       __FUNCTION__);
		goto error_enable_irq;
	}

	if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_WORKING) {
		printk(KERN_ERR
		       "%s() - ERROR: The Touch Panel was not put in Working Mode. The Device Mode Register contains 0x%02X\n",
		       __FUNCTION__, regval);
		retval = FT5x06_ERR_NOT_WORKING_MODE;
		goto error_enable_irq;
	}

	retval = 0;

error_enable_irq:
	if (ts->client->irq) {
#if FT5x06_DEBUG_VERBOSE
		printk(KERN_DEBUG "%s() - Enabling Touch Panel interrupts.\n",
		       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */
		enable_irq(ts->client->irq);
	}

	return retval;
}

static int ft5x06_read_data(struct i2c_client *client, char *output_buffer,
			    ssize_t output_buffer_size,
			    ssize_t * p_num_read_chars)
{
	int retval = 0;
	int i = 0;
	u16 dataval = 0x0000;
	u8 devmode = 0x00;
	u8 rownum = 0x00;

	u8 read_buffer[FT5x06_NUM_RX * 2];

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Verifying that the Touch Panel is in Factory Mode...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_read_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE,
					  sizeof(u8), &devmode);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read from the Device Mode register.\n",
		       __FUNCTION__);
		goto error_return;
	}

	if (FT5x06_MODE_FACTORY != (devmode & FT5x06_MODE_MASK)) {
		printk(KERN_ERR
		       "%s() - ERROR: The Touch Panel is not in Factory Mode.\n",
		       __FUNCTION__);
		retval = FT5x06_ERR_NOT_FACTORY_MODE;
		goto error_return;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Initiating a scan for raw data...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	devmode |= FT5x06_SCAN_START;
	retval =
	    i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE,
					   sizeof(u8), &devmode);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not initiate a raw data scan.\n",
		       __FUNCTION__);
		goto error_return;
	}

	msleep(FT5x06_SCAN_DELAY);

	retval =
	    i2c_smbus_read_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE,
					  sizeof(u8), &devmode);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the updated value of the Device Mode register.\n",
		       __FUNCTION__);
		goto error_return;
	}

	if (FT5x06_SCAN_DONE != (devmode & FT5x06_SCAN_MASK)) {
		printk(KERN_ERR
		       "%s() - ERROR: The raw data scan did not complete after %u ms.\n",
		       __FUNCTION__, FT5x06_SCAN_DELAY);
		retval = FT5x06_ERR_SCAN_NOT_DONE;
		goto error_return;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Reading raw data...\n", __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	for (rownum = 0; rownum < FT5x06_NUM_TX; rownum++) {
		memset(read_buffer, 0x00, (FT5x06_NUM_RX * 2));

		retval =
		    i2c_smbus_write_i2c_block_data(client,
						   FT5x06_FMREG_ROW_ADDR,
						   sizeof(u8), &rownum);
		if (0 != retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not write the row number.\n",
			       __FUNCTION__);
			goto error_return;
		}

		msleep(1);

		/* Read the data for this row */
		retval =
		    i2c_smbus_read_i2c_block_data(client,
						  FT5x06_FMREG_RAWDATA_0_H,
						  (FT5x06_NUM_RX * 2),
						  read_buffer);
		if (0 > retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not read row %u raw data.\n",
			       __FUNCTION__, rownum);
			goto error_return;
		}

		/* Each data value can be up to 5 chars long. Add a space and we need 6 chars to store it. */
		for (i = 0;
		     ((i < retval)
		      && ((*p_num_read_chars + 6) < output_buffer_size));
		     i += 2) {
			dataval = read_buffer[i];
			dataval = (dataval << 8);
			dataval |= read_buffer[i + 1];

			*p_num_read_chars +=
			    sprintf(&(output_buffer[*p_num_read_chars]), "%u ",
				    dataval);
		}

		output_buffer[*p_num_read_chars - 1] = '\n';
	}

	retval = 0;

error_return:
	return retval;
}

/*************************************************************************
 * Firmware Update
 ************************************************************************/

static int ft5x06_perform_fw_upgrade(struct device *dev,
				     const u8 * firmware_src_buffer,
				     u32 firmware_size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	int retval = 0;
	int i = 0;
	int j = 0;

	u32 num_packets = 0;
	u16 temp_val = 0;
	u8 checksum = 0;

	u8 temp_buffer[4];
	u8 packet_buffer[FT5x06_PACKET_LENGTH + 6];

	if (NULL == firmware_src_buffer) {
		printk(KERN_ERR
		       "%s() - ERROR: Firmware Source Buffer pointer is NULL.\n",
		       __FUNCTION__);
		retval = -EINVAL;
		goto error_return;
	}

	if (0 == firmware_size) {
		printk(KERN_ERR "%s() - ERROR: Firmware Source Size is ZERO.\n",
		       __FUNCTION__);
		retval = -EINVAL;
		goto error_return;
	}

    /*********************************************************************************************/
	printk(KERN_INFO "%s() - Step 1: Reset the CTPM\n", __FUNCTION__);

	retval = ft5x06_enter_fwupdate_mode(ts);
	if (0 != retval) {
		goto error_reset;
	}
    /*********************************************************************************************/
	printk(KERN_INFO "%s() - Step 3: Read the CTPM ID\n", __FUNCTION__);

	temp_buffer[0] = FT5x06_CMD_GET_ID;
	temp_buffer[1] = FT5x06_CMD_GET_ID_P1;
	temp_buffer[2] = FT5x06_CMD_GET_ID_P2;
	temp_buffer[3] = FT5x06_CMD_GET_ID_P3;

	retval = i2c_master_send(ts->client, temp_buffer, 4);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write the Get ID command to the CTPM.\n",
		       __FUNCTION__);
		goto error_reset;
	}

	msleep(10);

	retval = i2c_master_recv(ts->client, temp_buffer, 2);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the ID from the CTPM.\n",
		       __FUNCTION__);
		goto error_reset;
	}

	if ((FT5x06_CPTM_ID_COMPANY != temp_buffer[0])
	    || (FT5x06_CPTM_ID_PRODUCT != temp_buffer[1])) {
		printk(KERN_ERR
		       "%s() - ERROR: Invalid CPTM ID. Expected 0x%02X%02X, got 0x%02X%02X.\n",
		       __FUNCTION__, FT5x06_CPTM_ID_COMPANY,
		       FT5x06_CPTM_ID_PRODUCT, temp_buffer[0], temp_buffer[1]);
		retval = FT5x06_ERR_INVALID_ID;
		goto error_reset;
	}

    /*********************************************************************************************/
	printk(KERN_INFO "%s() - Step 4: Erase the old CTPM firmware\n",
	       __FUNCTION__);

	temp_buffer[0] = FT5x06_CMD_ERASE_FW;

	retval = i2c_master_send(ts->client, temp_buffer, 1);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write the Erase Firmware command to the CTPM.\n",
		       __FUNCTION__);
		goto error_reset;
	}

	msleep(1500);

    /*********************************************************************************************/
	printk(KERN_INFO
	       "%s() - Step 5: Write the new CTPM firmware to CTPM flash\n",
	       __FUNCTION__);

	/* We write everything but the last 8 bytes in packets.
	 * The first 6 of the last 8 bytes will be written in the footer.
	 * The final 2 bytes (which seem to be always 0xFF and 0x00) don't get written.
	 */
	firmware_size = firmware_size - 8;
	num_packets = (firmware_size) / FT5x06_PACKET_LENGTH;

	packet_buffer[0] = 0xBF;
	packet_buffer[1] = 0x00;

	/* Write whole packets */
	for (i = 0; i < num_packets; i++) {
		/* Target offset */
		temp_val = i * FT5x06_PACKET_LENGTH;

		packet_buffer[2] = (u8) (0x00FF & (temp_val >> 8));
		packet_buffer[3] = (u8) (0x00FF & (temp_val));

		/* Num bytes following header */
		temp_val = FT5x06_PACKET_LENGTH;

		packet_buffer[4] = (u8) (0x00FF & (temp_val >> 8));
		packet_buffer[5] = (u8) (0x00FF & (temp_val));

		for (j = 0; j < FT5x06_PACKET_LENGTH; j++) {
			/* Process byte j of packet i... */
			packet_buffer[6 + j] =
			    firmware_src_buffer[(i * FT5x06_PACKET_LENGTH) + j];
			checksum ^= packet_buffer[6 + j];
		}

		retval =
		    i2c_master_send(ts->client, packet_buffer,
				    FT5x06_PACKET_LENGTH + 6);
		if (0 > retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not write packet %u of %u to the CTPM.\n",
			       __FUNCTION__, i, num_packets);
			goto error_reset;
		}

		msleep(20);

		if (0 == ((i * FT5x06_PACKET_LENGTH) % 1024)) {
			printk(KERN_DEBUG "%s() - Uploaded %6d of %6u bytes.\n",
			       __FUNCTION__, (i * FT5x06_PACKET_LENGTH),
			       firmware_size);
		}
	}

	printk(KERN_DEBUG "%s() - Uploaded %6d of %6u bytes.\n", __FUNCTION__,
	       (i * FT5x06_PACKET_LENGTH), firmware_size);

	/* Write a partial packet if necessary */
	if (0 != (firmware_size % FT5x06_PACKET_LENGTH)) {
		/* Target offset */
		temp_val = num_packets * FT5x06_PACKET_LENGTH;

		packet_buffer[2] = (u8) (0x00FF & (temp_val >> 8));
		packet_buffer[3] = (u8) (0x00FF & (temp_val));

		/* Num bytes following header */
		temp_val = (firmware_size % FT5x06_PACKET_LENGTH);

		packet_buffer[4] = (u8) (0x00FF & (temp_val >> 8));
		packet_buffer[5] = (u8) (0x00FF & (temp_val));

		for (j = 0; j < temp_val; j++) {
			packet_buffer[6 + j] =
			    firmware_src_buffer[(num_packets *
						 FT5x06_PACKET_LENGTH) + j];
			checksum ^= packet_buffer[6 + j];
		}

		retval =
		    i2c_master_send(ts->client, packet_buffer, temp_val + 6);
		if (0 > retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not write partial packet to the CTPM.\n",
			       __FUNCTION__);
			goto error_reset;
		}

		msleep(20);
	}

	printk(KERN_DEBUG "%s() - Uploaded %6d of %6u bytes.\n", __FUNCTION__,
	       (i * FT5x06_PACKET_LENGTH) + temp_val, firmware_size);

	/* Write the firmware footer */
	for (i = 0; i < 6; i++) {
		packet_buffer[2] = 0x6F;
		packet_buffer[3] = 0xFA + i;
		packet_buffer[4] = 0x00;
		packet_buffer[5] = 0x01;

		packet_buffer[6] = firmware_src_buffer[firmware_size + i];
		checksum ^= packet_buffer[6];

		retval = i2c_master_send(ts->client, packet_buffer, 7);
		if (0 > retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not write partial packet to the CTPM.\n",
			       __FUNCTION__);
			goto error_reset;
		}

		msleep(20);
	}

    /*********************************************************************************************/
	printk(KERN_INFO "%s() - Step 6: Checksum verification\n",
	       __FUNCTION__);

	temp_buffer[0] = FT5x06_CMD_GET_CHECKSUM;

	retval = i2c_master_send(ts->client, temp_buffer, 1);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write the Get Checksum command to the CTPM.\n",
		       __FUNCTION__);
		goto error_reset;
	}

	msleep(10);

	retval = i2c_master_recv(ts->client, temp_buffer, 1);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the Checksum from the CTPM.\n",
		       __FUNCTION__);
		goto error_reset;
	}

	if (checksum != temp_buffer[0]) {
		printk(KERN_ERR
		       "%s() - ERROR: Checksum (0x%02X) did not match calculated value (0x%02X).\n",
		       __FUNCTION__, temp_buffer[0], checksum);
		retval = FT5x06_ERR_INVALID_CHECKSUM;
		goto error_reset;
	}

    /*********************************************************************************************/
	printk(KERN_INFO "%s() - Step 7: Reset the CTPM firmware\n",
	       __FUNCTION__);

	temp_buffer[0] = FT5x06_CMD_RESET_FW;

	retval = i2c_master_send(ts->client, temp_buffer, 1);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write the Reset Firmware command to the CTPM.\n",
		       __FUNCTION__);
		goto error_reset;
	}

	retval = 0;

error_reset:
    /*********************************************************************************************/
	printk(KERN_INFO "%s() - Step 8: Reset the CTPM\n", __FUNCTION__);

	ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);
	if (ts->platform_data->update_flags) {
		ts->platform_data->update_flags(ts->platform_data, ts->client);
	}

	printk(KERN_INFO "%s() - FIRMWARE UPDATE COMPLETE - Update %s\n",
	       __FUNCTION__, ((0 == retval) ? "Succeeded" : "Failed"));

error_return:
	return retval;
}

/*************************************************************************
 * SYSFS Store and Show functions
 ************************************************************************/

static ssize_t ft5x06_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%04X\n", FT5x06_DRIVER_VERSION);
}

static ssize_t ft5x06_rawbase_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int retval = 0;

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	retval = ft5x06_read_data(ts->client, buf, PAGE_SIZE, &num_read_chars);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read Raw Data from the Touch Panel.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
		goto error_return;
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_crosstalk_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	const u8 rx_offset =
	    ((FT5x06_CROSSTALK_TEST_TYPE_ODD ==
	      ts->crosstalk_test_type) ? 1 : 0);
	ssize_t num_read_chars = 0;

	u8 num_processed_rx_cac_values = 0x00;
	u8 i = 0x00;
	u8 regval = 0x00;
	int retval = 0;

	u8 original_rx_cac[FT5x06_NUM_RX];

	mutex_lock(&ts->device_mode_mutex);

	memset(original_rx_cac, 0x00, FT5x06_NUM_RX);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the device in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	/* Preserve the original values of the even or odd RXn_CAC registers and then set them to 0x00. */

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Modifying the RXn_CAC register values...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	for (i = rx_offset; i < FT5x06_NUM_RX; i += 2) {
		retval =
		    i2c_smbus_read_i2c_block_data(ts->client,
						  (FT5x06_FMREG_RX_0_CAC + i),
						  sizeof(u8), &regval);
		if (0 > retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not read from the RX%u_CAC register.\n",
			       __FUNCTION__, i);
			goto error_restore_cac_values;
		}

		original_rx_cac[num_processed_rx_cac_values] = regval;
		num_processed_rx_cac_values++;

		regval = 0x00;
		retval =
		    i2c_smbus_write_i2c_block_data(ts->client,
						   (FT5x06_FMREG_RX_0_CAC + i),
						   sizeof(u8), &regval);
		if (0 != retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n",
			       __FUNCTION__, i);
			goto error_restore_cac_values;
		}
	}

	msleep(100);

	retval = ft5x06_read_data(ts->client, buf, PAGE_SIZE, &num_read_chars);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read Raw Data from the Touch Panel.\n",
		       __FUNCTION__);
		goto error_restore_cac_values;
	}

error_restore_cac_values:
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Restoring the RXn_CAC register values...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	for (i = 0; i < num_processed_rx_cac_values; i++) {
		retval =
		    i2c_smbus_write_i2c_block_data(ts->client,
						   (FT5x06_FMREG_RX_0_CAC +
						    (2 * i) + rx_offset),
						   sizeof(u8),
						   &(original_rx_cac[i]));
		if (0 != retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not restore the original value of the RX%u_CAC register.\n",
			       __FUNCTION__, (2 * i) + rx_offset);
			goto error_restore_mode;
		}
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_crosstalk_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);
	unsigned long value = 0;
	int retval = 0;

	mutex_lock(&ts->device_mode_mutex);

	retval = strict_strtoul(buf, 10, &value);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n",
		       __FUNCTION__, buf);
		goto error_return;
	}

	switch (value) {
	case 0:
#if FT5x06_DEBUG_VERBOSE
		printk(KERN_DEBUG "%s() - Crosstalk Test Type is now EVEN.\n",
		       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */
		ts->crosstalk_test_type = FT5x06_CROSSTALK_TEST_TYPE_EVEN;
		break;

	case 1:
#if FT5x06_DEBUG_VERBOSE
		printk(KERN_DEBUG "%s() - Crosstalk Test Type is now ODD.\n",
		       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */
		ts->crosstalk_test_type = FT5x06_CROSSTALK_TEST_TYPE_ODD;
		break;

	default:
		printk(KERN_ERR
		       "%s() - ERROR: Invalid input value specified: %lu\n",
		       __FUNCTION__, value);
		goto error_return;
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return count;
}

static ssize_t ft5x06_icsupplier_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int retval = 0;
	u8 regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Reading IC Supplier value...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_FOCALTECH_ID,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the IC Supplier value.\n",
		       __FUNCTION__);
		goto error_return;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - IC Supplier value: 0x%02X\n", __FUNCTION__,
	       regval);
#endif /* FT5x06_DEBUG_VERBOSE */

	num_read_chars += snprintf(buf, PAGE_SIZE, "0x%02X\n", regval);

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_icpartno_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int retval = 0;
	u8 regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the device in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Reading IC PartNO value...\n", __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_IC_PARTNO,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the IC PartNO. value.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - IC PartNO value: 0x%02X\n", __FUNCTION__,
	       regval);
#endif /* FT5x06_DEBUG_VERBOSE */

	num_read_chars += snprintf(buf, PAGE_SIZE, "0x%02X\n", regval);

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_storecalibrateflash_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	int retval = 0;
	u8 regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Configuring the Calibration register...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	regval = FT5x06_CALIBRATE_SAVE_TO_FLASH;
	retval =
	    i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_CALIBRATE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write to the Calibration register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	msleep(1000);

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the current value of the Device Mode register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	if (FT5x06_MODE_WORKING != regval) {
		printk(KERN_ERR
		       "%s() - ERROR: The Device Mode register contained 0x%02X (expected 0x%02X).\n",
		       __FUNCTION__, regval, FT5x06_MODE_WORKING);
		goto error_restore_mode;
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return 0;
}

static ssize_t ft5x06_baseline_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int retval = 0;
	u8 regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Configuring the Baseline register...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	regval = FT5x06_BASELINE_ENABLE;
	retval =
	    i2c_smbus_write_i2c_block_data(ts->client,
					   FT5x06_FMREG_BASELINE_ENABLE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write to the Baseline register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	msleep(2);

	regval = FT5x06_BASELINE_DISABLE;
	retval =
	    i2c_smbus_read_i2c_block_data(ts->client,
					  FT5x06_FMREG_BASELINE_ENABLE,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the updated value of the Baseline register.\n",
		       __FUNCTION__);
		goto error_restore_baseline;
	}

	if (FT5x06_BASELINE_ENABLE != (FT5x06_BASELINE_MASK & regval)) {
		printk(KERN_ERR
		       "%s() - ERROR: The Baseline register contained 0x%02X (expected 0x%02X).\n",
		       __FUNCTION__, (FT5x06_BASELINE_MASK & regval),
		       FT5x06_BASELINE_ENABLE);
		goto error_restore_baseline;
	}

	/* Read raw data */

	retval = ft5x06_read_data(ts->client, buf, PAGE_SIZE, &num_read_chars);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read Raw Data from the Touch Panel.\n",
		       __FUNCTION__);
		goto error_restore_baseline;
	}

error_restore_baseline:
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Restoring the Baseline register...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	regval = FT5x06_BASELINE_DISABLE;
	retval =
	    i2c_smbus_write_i2c_block_data(ts->client,
					   FT5x06_FMREG_BASELINE_ENABLE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write to the Baseline register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	msleep(2);

	regval = FT5x06_BASELINE_ENABLE;
	retval =
	    i2c_smbus_read_i2c_block_data(ts->client,
					  FT5x06_FMREG_BASELINE_ENABLE,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the updated value of the Baseline register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	if (FT5x06_BASELINE_DISABLE != (FT5x06_BASELINE_MASK & regval)) {
		printk(KERN_ERR
		       "%s() - ERROR: The Baseline register contained 0x%02X (expected 0x%02X).\n",
		       __FUNCTION__, (FT5x06_BASELINE_MASK & regval),
		       FT5x06_BASELINE_DISABLE);
		goto error_restore_mode;
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_tpfwver_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int fwver = 0;

	mutex_lock(&ts->device_mode_mutex);
	fwver = i2c_smbus_read_byte_data(ts->client, FT5x06_WMREG_FW_VER);
	if (0 > fwver) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read Firmware version Number.\n",
		       __FUNCTION__);
		goto error_return;
	}

	num_read_chars =
	    snprintf(buf, PAGE_SIZE, "%02X\n", (fwver & 0x000000FF));

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

/* Read the vendorID from firmware protected area, need to enter into firmware mode */
static ssize_t ft5x06_vendorid_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	u8 vendorid = 0;
	int retval = 0;
	u8 temp_buffer[4];
	struct i2c_msg msgs[2];

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_fwupdate_mode(ts);
	if (0 != retval) {
		goto error_reset;
	}

	temp_buffer[0] = FT5x06_UPGRADEVER_REG;
	msgs[0].addr = ts->client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = temp_buffer;

	msgs[1].addr = ts->client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &vendorid;

	retval = i2c_transfer(ts->client->adapter, msgs, 2);
	if (0 >= retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the Upgrad Version from the CTPM.\n",
		       __FUNCTION__);
		goto error_reset;
	}

	num_read_chars = snprintf(buf, PAGE_SIZE, "0x%02X\n", vendorid);
error_reset:
	printk(KERN_INFO "%s() - Reset the Touch Panel\n", __FUNCTION__);
	ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_calibrate_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int i = 0;
	int retval = 0;
	u8 regval = 0x00;
	u8 devmode = 0x00;

	u8 calibration_data[FT5x06_CALIBRATION_NUM_BYTES];

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the device in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Configuring the Calibration register...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	regval = FT5x06_CALIBRATE_START;
	retval =
	    i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_CALIBRATE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write to the Calibration register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	msleep(1000);

	/* Configuring the Calibration register should put us back in Working Mode. */

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE,
					  sizeof(u8), &devmode);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the value of the Device Mode register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	if (FT5x06_MODE_WORKING != devmode) {
		printk(KERN_ERR
		       "%s() - ERROR: The Device Mode register contained 0x%02X (expected 0x%02X).\n",
		       __FUNCTION__, devmode, FT5x06_MODE_WORKING);
		goto error_restore_mode;
	}

	/* Go back to Factory Mode, but don't call ft5x06_enter_factory_mode()
	 * since that function will disable the IRQ a second time without
	 * re-enabling the IRQ first.
	 */
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Putting the Touch Panel in Factory Mode...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	regval = FT5x06_MODE_FACTORY;
	retval =
	    i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write to the Device Mode register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	msleep(100);

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Verifying that the Touch Panel is in Factory Mode...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read from the Device Mode register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_FACTORY) {
		printk(KERN_ERR
		       "%s() - ERROR: The Touch Panel was not put in Factory Mode. The Device Mode register contains 0x%02X\n",
		       __FUNCTION__, regval);
		retval = FT5x06_ERR_NOT_FACTORY_MODE;
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Reading calibration data...\n", __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	for (i = 0; i < FT5x06_CALIBRATION_NUM_BYTES; i++) {
		retval =
		    i2c_smbus_read_i2c_block_data(ts->client,
						  (FT5x06_FMREG_RX_0_1_OFFSET +
						   i), sizeof(u8),
						  &calibration_data[i]);
		if (0 > retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not read offsets for RX %d and %d.\n",
			       __FUNCTION__, (2 * i), ((2 * i) + 1));
			goto error_restore_mode;
		}
	}

	msleep(100);

	/* Each calibration byte holds two offsets that can each be up to 2 chars long. Add two spaces and we need 6 chars to store it. */
	for (i = 0;
	     ((i < FT5x06_CALIBRATION_NUM_BYTES)
	      && ((num_read_chars + 6) < PAGE_SIZE)); i++) {
		num_read_chars +=
		    sprintf(&(buf[num_read_chars]), "%u %u ",
			    (calibration_data[i] & 0x0F),
			    ((calibration_data[i] >> 4) & 0x0F));
	}

	buf[num_read_chars - 1] = '\n';

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_voltage_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int retval = 0;
	u8 voltage = 0;

	mutex_lock(&ts->device_mode_mutex);
	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the device in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Reading the Device Voltage...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client,
					  FT5x06_FMREG_DRIVER_VOLTAGE,
					  sizeof(u8), &voltage);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the value of the Device Voltage register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	num_read_chars +=
	    snprintf(buf, PAGE_SIZE, "%u\n", (FT5x06_VOLTAGE_MASK & voltage));

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_voltage_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	unsigned long voltage = 0;

	int retval = 0;
	u8 regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	retval = strict_strtoul(buf, 16, &voltage);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n",
		       __FUNCTION__, buf);
		goto error_return;
	}

	if (0x07 < voltage) {
		printk(KERN_ERR "%s() - ERROR: Invalid value specified: %lu\n",
		       __FUNCTION__, voltage);
		goto error_return;
	}

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the device in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	regval = (u8) (voltage & 0x000000FF);

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Writing 0x%02X to the Device Voltage register...\n",
	       __FUNCTION__, regval);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_write_i2c_block_data(ts->client,
					   FT5x06_FMREG_DRIVER_VOLTAGE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write 0x%02X to the Device Voltage register.\n",
		       __FUNCTION__, regval);
		goto error_restore_mode;
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}

static ssize_t ft5x06_interrupttest_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	const int gpio_num = irq_to_gpio(ts->client->irq);

	int test_result = 0;
	int retval = 0;
	u8 regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the device in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Verifying that the Interrupt GPIO is HIGH...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	if (!ft5x06_poll_gpio(gpio_num, 1)) {
		printk(KERN_ERR
		       "%s() - ERROR: The Interrupt GPIO is LOW when it should be HIGH.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	else {
		printk(KERN_DEBUG "%s() - The Interrupt GPIO is HIGH.\n",
		       __FUNCTION__);
	}

	printk(KERN_DEBUG "%s() - Toggling the Interrupt GPIO line...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	/* Note that the interrupt line can be toggled by writing any value to the INTERRRUPT_TOGGLE register. */
	regval = 0x00;
	retval =
	    i2c_smbus_write_i2c_block_data(ts->client,
					   FT5x06_FMREG_INTERRUPT_TOGGLE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write to the Interrupt Toggle register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Verifying that the Interrupt GPIO is LOW...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	if (!ft5x06_poll_gpio(gpio_num, 0)) {
		printk(KERN_ERR
		       "%s() - ERROR: The Interrupt GPIO is HIGH when it should be LOW.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	else {
		printk(KERN_DEBUG "%s() - The Interrupt GPIO is LOW.\n",
		       __FUNCTION__);
	}

	printk(KERN_DEBUG "%s() - Toggling the Interrupt GPIO line...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	regval = 0x00;
	retval =
	    i2c_smbus_write_i2c_block_data(ts->client,
					   FT5x06_FMREG_INTERRUPT_TOGGLE,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write to the Interrupt Toggle register.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Verifying that the Interrupt GPIO is HIGH...\n",
	       __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	if (!ft5x06_poll_gpio(gpio_num, 1)) {
		printk(KERN_ERR
		       "%s() - ERROR: The Interrupt GPIO is LOW when it should be HIGH.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	else {
		printk(KERN_DEBUG "%s() - The Interrupt GPIO is HIGH.\n",
		       __FUNCTION__);
	}
#endif /* FT5x06_DEBUG_VERBOSE */

	test_result = 1;

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Test Result: %s\n", __FUNCTION__,
	       (test_result ? "PASS" : "FAIL"));
#endif /* FT5x06_DEBUG_VERBOSE */
	mutex_unlock(&ts->device_mode_mutex);
	return snprintf(buf, PAGE_SIZE, "%d\n", test_result);
}

static ssize_t ft5x06_tpreset_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	mutex_lock(&ts->device_mode_mutex);
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Resetting Touch Panel\n", __FUNCTION__);
#endif /* FT5x06_DEBUG_VERBOSE */

	ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);

	mutex_unlock(&ts->device_mode_mutex);
	return 0;
}

static ssize_t ft5x06_fwupdate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);
	const struct firmware *fw_blob = NULL;
	char fw_name[256];

	mutex_lock(&ts->device_mode_mutex);

	// buf contains the newline-terminated firmware file name - remove the newline.
	strncpy(fw_name, buf, size - 1);
	fw_name[size - 1] = '\0';

	printk(KERN_DEBUG "%s() - Processing input file: '%s'\n", __FUNCTION__,
	       fw_name);

	if (request_firmware(&fw_blob, fw_name, dev)) {
		printk(KERN_ERR "%s() - ERROR: request_firmware failed\n",
		       __FUNCTION__);
		goto error_return;
	}

	if (ft5x06_perform_fw_upgrade(dev, fw_blob->data, fw_blob->size)) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not update the firmware using '%s'\n",
		       __FUNCTION__, fw_name);
		goto error_return;
	}

error_return:
	if (fw_blob) {
		release_firmware(fw_blob);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return size;
}

static ssize_t ft5x06_fwupdate_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x06_fmreg_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);
	unsigned long fmreg = 0;
	int retval = 0;

	mutex_lock(&ts->device_mode_mutex);

	retval = strict_strtoul(buf, 16, &fmreg);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n",
		       __FUNCTION__, buf);
		goto error_return;
	}

	if ((FT5x06_FMREG_MAX < fmreg) ||
	    (FT5x06_FMREG_RESERVED == fmreg) ||
	    ((FT5x06_FMREG_RESERVEDBLK_START <= fmreg)
	     && (FT5x06_FMREG_RESERVEDBLK_END >= fmreg))) {
		printk(KERN_ERR
		       "%s() - ERROR: Invalid register specified: %lu\n",
		       __FUNCTION__, fmreg);
		goto error_return;
	}

	ts->factory_mode_register = (u8) (fmreg & 0x000000FF);

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}

static ssize_t ft5x06_fmreg_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);
	return snprintf(buf, PAGE_SIZE, "0x%02X\n", ts->factory_mode_register);
}

static ssize_t ft5x06_fmval_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	unsigned long fmval = 0;

	int retval = 0;
	u8 regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	retval = strict_strtoul(buf, 16, &fmval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n",
		       __FUNCTION__, buf);
		goto error_return;
	}

	if (0xFF < fmval) {
		printk(KERN_ERR "%s() - ERROR: Invalid value specified: %lu\n",
		       __FUNCTION__, fmval);
		goto error_return;
	}

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the device in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}

	regval = (u8) (fmval & 0x000000FF);

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Writing 0x%02X to the register at offset 0x%02X...\n",
	       __FUNCTION__, regval, ts->factory_mode_register);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_write_i2c_block_data(ts->client,
					   ts->factory_mode_register,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write 0x%02X to the register at offset 0x%02X.\n",
		       __FUNCTION__, regval, ts->factory_mode_register);
		goto error_restore_mode;
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}

static ssize_t ft5x06_fmval_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int retval = 0;
	u8 regval = 0;

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not put the device in Factory Mode.\n",
		       __FUNCTION__);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Reading the register at offset 0x%02X...\n",
	       __FUNCTION__, ts->factory_mode_register);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client, ts->factory_mode_register,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the value of the register at offset 0x%02X.\n",
		       __FUNCTION__, ts->factory_mode_register);
		goto error_restore_mode;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - The register at offset 0x%02X contains value 0x%02X.\n",
	       __FUNCTION__, ts->factory_mode_register, regval);
#endif /* FT5x06_DEBUG_VERBOSE */

	num_read_chars += snprintf(buf, PAGE_SIZE, "0x%02X\n", regval);

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n",
		       __FUNCTION__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_wmreg_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);
	unsigned long wmreg = 0;
	int retval = 0;

	mutex_lock(&ts->device_mode_mutex);
	retval = strict_strtoul(buf, 16, &wmreg);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n",
		       __FUNCTION__, buf);
		goto error_return;
	}

	switch (wmreg) {
	case FT5x06_WMREG_DEVICE_MODE:
	case FT5x06_WMREG_GEST_ID:
	case FT5x06_WMREG_TD_STATUS:
	case FT5x06_WMREG_P1_XH:
	case FT5x06_WMREG_P1_XL:
	case FT5x06_WMREG_P1_YH:
	case FT5x06_WMREG_P1_YL:
	case FT5x06_WMREG_P2_XH:
	case FT5x06_WMREG_P2_XL:
	case FT5x06_WMREG_P2_YH:
	case FT5x06_WMREG_P2_YL:
	case FT5x06_WMREG_TH_TOUCH:
	case FT5x06_WMREG_RPT_RATE:
	case FT5x06_WMREG_OFFSET_LEFT_RIGHT:
	case FT5x06_WMREG_DISTANCE_ZOOM:
	case FT5x06_WMREG_LIB_VER_H:
	case FT5x06_WMREG_LIB_VER_L:
	case FT5x06_WMREG_PWR_MODE:
	case FT5x06_WMREG_FW_VER:
	case FT5x06_WMREG_FOCALTECH_ID:
	case FT5x06_WMREG_RESET:
		ts->working_mode_register = (u8) (wmreg & 0x000000FF);
		break;

	default:
		printk(KERN_ERR
		       "%s() - ERROR: Invalid register specified: %lu\n",
		       __FUNCTION__, wmreg);
		goto error_return;
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}

static ssize_t ft5x06_wmreg_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", ts->working_mode_register);
}

static ssize_t ft5x06_wmval_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	unsigned long wmval = 0;

	int retval = 0;
	u8 regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);
	retval = strict_strtoul(buf, 16, &wmval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n",
		       __FUNCTION__, buf);
		goto error_return;
	}

	if (0xFF < wmval) {
		printk(KERN_ERR "%s() - ERROR: Invalid value specified: %lu\n",
		       __FUNCTION__, wmval);
		goto error_return;
	}

	regval = (u8) (wmval & 0x000000FF);

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - Writing 0x%02X to the register at offset 0x%02X...\n",
	       __FUNCTION__, regval, ts->working_mode_register);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_write_i2c_block_data(ts->client,
					   ts->working_mode_register,
					   sizeof(u8), &regval);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not write 0x%02X to the register at offset 0x%02X.\n",
		       __FUNCTION__, regval, ts->working_mode_register);
		goto error_return;
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}

static ssize_t ft5x06_wmval_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int retval = 0;
	u8 regval = 0;

	mutex_lock(&ts->device_mode_mutex);

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG "%s() - Reading the register at offset 0x%02X...\n",
	       __FUNCTION__, ts->working_mode_register);
#endif /* FT5x06_DEBUG_VERBOSE */

	retval =
	    i2c_smbus_read_i2c_block_data(ts->client, ts->working_mode_register,
					  sizeof(u8), &regval);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not read the value of the register at offset 0x%02X.\n",
		       __FUNCTION__, ts->working_mode_register);
		goto error_return;
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_DEBUG
	       "%s() - The register at offset 0x%02X contains value 0x%02X.\n",
	       __FUNCTION__, ts->working_mode_register, regval);
#endif /* FT5x06_DEBUG_VERBOSE */

	num_read_chars += snprintf(buf, PAGE_SIZE, "0x%02X\n", regval);

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

/* sysfs */
static DEVICE_ATTR(version, S_IRUGO, ft5x06_version_show, NULL);
static DEVICE_ATTR(rawbase, S_IRUGO, ft5x06_rawbase_show, NULL);
static DEVICE_ATTR(crosstalk, S_IRUGO | S_IWUSR, ft5x06_crosstalk_show,
		   ft5x06_crosstalk_store);
static DEVICE_ATTR(icsupplier, S_IRUGO, ft5x06_icsupplier_show, NULL);
static DEVICE_ATTR(icpartno, S_IRUGO, ft5x06_icpartno_show, NULL);
static DEVICE_ATTR(storecalibrateflash, S_IRUGO,
		   ft5x06_storecalibrateflash_show, NULL);
static DEVICE_ATTR(baseline, S_IRUGO, ft5x06_baseline_show, NULL);
static DEVICE_ATTR(tpfwver, S_IRUGO, ft5x06_tpfwver_show, NULL);
static DEVICE_ATTR(vendorid, S_IRUGO, ft5x06_vendorid_show, NULL);
static DEVICE_ATTR(voltage, S_IRUGO | S_IWUSR, ft5x06_voltage_show,
		   ft5x06_voltage_store);
static DEVICE_ATTR(calibrate, S_IRUGO, ft5x06_calibrate_show, NULL);
static DEVICE_ATTR(interrupttest, S_IRUGO, ft5x06_interrupttest_show, NULL);
static DEVICE_ATTR(tpreset, S_IRUGO, ft5x06_tpreset_show, NULL);
static DEVICE_ATTR(fwupdate, S_IRUGO | S_IWUSR, ft5x06_fwupdate_show,
		   ft5x06_fwupdate_store);
static DEVICE_ATTR(fmreg, S_IRUGO | S_IWUSR, ft5x06_fmreg_show,
		   ft5x06_fmreg_store);
static DEVICE_ATTR(fmval, S_IRUGO | S_IWUSR, ft5x06_fmval_show,
		   ft5x06_fmval_store);
static DEVICE_ATTR(wmreg, S_IRUGO | S_IWUSR, ft5x06_wmreg_show,
		   ft5x06_wmreg_store);
static DEVICE_ATTR(wmval, S_IRUGO | S_IWUSR, ft5x06_wmval_show,
		   ft5x06_wmval_store);

static struct attribute *ft5x06_attributes[] = {
	&dev_attr_version.attr,
	&dev_attr_rawbase.attr,
	&dev_attr_crosstalk.attr,
	&dev_attr_icsupplier.attr,
	&dev_attr_icpartno.attr,
	&dev_attr_storecalibrateflash.attr,
	&dev_attr_baseline.attr,
	&dev_attr_tpfwver.attr,
	&dev_attr_vendorid.attr,
	&dev_attr_voltage.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_interrupttest.attr,
	&dev_attr_tpreset.attr,
	&dev_attr_fwupdate.attr,
	&dev_attr_fmreg.attr,
	&dev_attr_fmval.attr,
	&dev_attr_wmreg.attr,
	&dev_attr_wmval.attr,
	NULL
};

static struct attribute_group ft5x06_attribute_group = {
	.attrs = ft5x06_attributes
};

/* ************************************************************************
 * Probe and Initialization functions
 * ***********************************************************************/

/* ft5x06_initialize: Driver Initialization. This function takes
 * care of the following tasks:
 * 1. Create and register an input device with input layer
 * 2. Take FT5x06 device out of bootloader mode; go operational
 * 3. Start any timers/Work queues.
 * Note that validation has already been performed on the inputs.
 */
static int ft5x06_initialize(struct i2c_client *client, struct ft5x06 *ts)
{
	struct input_dev *input_device;
	int retval = 0;
	u8 id;

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device) {
		retval = -ENOMEM;
		printk(KERN_ERR
		       "%s() - ERROR: Could not allocate input device.\n",
		       __FUNCTION__);
		goto error_free_device;
	}

	ts->input = input_device;

	input_device->name = FT_I2C_NAME;
	input_device->phys = ts->phys;
	input_device->dev.parent = &client->dev;

	/* init the touch structures */
	ts->prv_tch = FT_NTCH;

	for (id = 0; id < FT_NUM_TRK_ID; id++) {
		ts->prv_mt_pos[id][FT_XPOS] = 0;
		ts->prv_mt_pos[id][FT_YPOS] = 0;
	}

	set_bit(EV_SYN, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	set_bit(EV_ABS, input_device->evbit);

	set_bit(BTN_TOUCH, input_device->keybit);

	input_set_abs_params(input_device, ABS_PRESSURE, 0, FT_MAXZ, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0X, 0,
			     ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0Y, 0,
			     ts->platform_data->maxy, 0, 0);

	if (ts->platform_data->use_gestures) {
		input_set_abs_params(input_device, ABS_HAT1X, 0, 255, 0, 0);	// Gesture code.
		input_set_abs_params(input_device, ABS_HAT2X, 0, 255, 0, 0);	// Gesture touch count.
		input_set_abs_params(input_device, ABS_HAT2Y, 0, 255, 0, 0);	// Gesture occur count.
	}

	if (ts->platform_data->use_mt) {
		input_set_abs_params(input_device, ABS_MT_POSITION_X, 0,
				     ts->platform_data->maxx, 0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0,
				     ts->platform_data->maxy, 0, 0);
		input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0,
				     FT_MAXZ, 0, 0);
		input_set_abs_params(input_device, ABS_MT_PRESSURE, 0, FT_MAXZ,
		                0, 0);
	}

	retval = input_register_device(input_device);
	if (0 != retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not register input device.\n",
		       __FUNCTION__);
		goto error_free_device;
	}

	/* Prepare our worker structure prior to setting up the timer/ISR */
	INIT_WORK(&ts->work, ft5x06_xy_worker);

	atomic_set(&ts->irq_enabled, 1);

	/* Interrupt setup */
	if (ts->client->irq) {
		/* request_irq() will call enable_irq() */
		retval =
		    request_irq(ts->client->irq, ft5x06_irq,
				IRQF_TRIGGER_FALLING, input_device->name, ts);
		if (retval) {
			printk(KERN_ERR
			       "%s() - ERROR: Could not request IRQ: %d\n",
			       __FUNCTION__, retval);
			goto error_free_irq;
		}
	}

	retval = sysfs_create_group(&client->dev.kobj, &ft5x06_attribute_group);
	if (retval) {
		printk(KERN_ERR
		       "%s() - ERROR: sysfs_create_group() failed: %d\n",
		       __FUNCTION__, retval);
	} else {
		printk(KERN_INFO "%s() - sysfs_create_group() succeeded.\n",
		       __FUNCTION__);
	}

	ts->crosstalk_test_type = FT5x06_CROSSTALK_TEST_TYPE_EVEN;
	ts->factory_mode_register = FT5x06_FMREG_DEVICE_MODE;
	ts->working_mode_register = FT5x06_WMREG_DEVICE_MODE;

	retval = device_create_file(&ts->client->dev, &dev_attr_irq_enable);
	if (retval < 0) {
		printk(KERN_ERR
		       "%s() - ERROR: File device creation failed: %d\n",
		       __FUNCTION__, retval);
		retval = -ENODEV;
		goto error_free_irq;
	}

	goto success;

error_free_irq:
	free_irq(ts->client->irq, ts);

error_free_device:
	if (input_device) {
		input_free_device(input_device);
		ts->input = NULL;
	}

success:
	return retval;
}

/* I2C driver probe function */
static int __devinit ft5x06_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct ft5x06 *ts;
	int retval = 0;
	u8 buffer = 0;

	// request gpio resources
	if (0 > ft5x06_dev_init(1)) {
		retval = -ENODEV;
		goto error_return;
	}

	ts = kzalloc(sizeof(struct ft5x06), GFP_KERNEL);
	if (NULL == ts) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not allocate %d bytes of kernel memory for ft5x06 struct.\n",
		       __FUNCTION__, sizeof(struct ft5x06));
		retval = -ENOMEM;
		goto error_devinit0;
	}

	ts->client = client;
	ts->platform_data = client->dev.platform_data;
	i2c_set_clientdata(client, ts);

	retval =
		i2c_smbus_read_i2c_block_data(ts->client, 0, sizeof(u8), &buffer);
	if (0 > retval) {
		printk(KERN_ERR "%s() - ERROR: FT5x06 not found on I2C bus.\n",
		       __FUNCTION__);
		goto error_free_ts;
	}

	printk(KERN_INFO "%s() - FT5x06 found on I2C bus.\n", __FUNCTION__);

	ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);

	// Need to initialize the SYSFS mutex before creating the SYSFS entries in ft5x06_initialize().
	mutex_init(&ts->device_mode_mutex);
	retval = ft5x06_initialize(client, ts);
	if (0 > retval) {
		printk(KERN_ERR
		       "%s() - ERROR: Controller could not be initialized.\n",
		       __FUNCTION__);
		goto error_mutex_destroy;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ft5x06_early_suspend;
	ts->early_suspend.resume = ft5x06_late_resume;

	register_early_suspend(&ts->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	if (ts->platform_data->update_flags) {
		ts->platform_data->update_flags(ts->platform_data, ts->client);
	}

	goto error_return;

error_mutex_destroy:
	mutex_destroy(&ts->device_mode_mutex);

error_free_ts:
	kfree(ts);

error_devinit0:
	ft5x06_dev_init(0);

error_return:
	return retval;
}

/* Function to manage power-on resume */
static int ft5x06_resume(struct i2c_client *client)
{
	int retval = 0;
	struct ft5x06 *ts = NULL;

#if FT5x06_DEBUG_VERBOSE
	printk(KERN_INFO "%s() - Driver is resuming start function.\n",
	       __FUNCTION__);
#endif
	ts = (struct ft5x06 *)i2c_get_clientdata(client);
	if (ts->platform_data->platform_resume) {
		ts->platform_data->platform_resume();
	}

	ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);
	mb();
	enable_irq(ts->client->irq);
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_INFO "%s() - Driver is resuming end function.\n",
	       __FUNCTION__);
#endif
	return retval;
}

/* Function to manage low power suspend */
static int ft5x06_suspend(struct i2c_client *client, pm_message_t message)
{
	struct ft5x06 *ts = NULL;

	ts = (struct ft5x06 *)i2c_get_clientdata(client);
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_INFO "%s() - Driver is suspending: start.\n", __FUNCTION__);
#endif
	/* Disable/enable irq call (in irq/worker function) are matched, disable here for suspend */
	disable_irq(ts->client->irq);
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_INFO "%s() - Driver is suspending: disable IRQ.\n",
	       __FUNCTION__);
#endif
	/* Wait for woker finish, even if worker enables irq, the irq still is disabled because of the above call */
	flush_workqueue(ft5x06_ts_wq);
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_INFO "%s() - Driver is suspending: flash workqueue.\n",
	       __FUNCTION__);
#endif
	/* No need to cancel since irq is disabled, there is no pending worker at this time */

	if (ts->platform_data->platform_suspend) {
#if FT5x06_DEBUG_VERBOSE
		printk(KERN_INFO "%s() - Driver is suspending: platform suspend start.\n", __FUNCTION__);
#endif
		ts->platform_data->platform_suspend();
#if FT5x06_DEBUG_VERBOSE
		printk(KERN_INFO "%s() - Driver is suspending: platform suspend end.\n", __FUNCTION__);
#endif
	}
	// keep focaltech controller in reset after this point
	gpio_direction_output(ts->platform_data->reset_gpio, 0);
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_INFO "%s() - Driver is suspending: keep focaltech controller in reset after this point.\n", __FUNCTION__);
	printk(KERN_INFO "%s() - Driver is suspending end.\n", __FUNCTION__);
#endif
	return 0;
}

/* registered in driver struct */
static int __devexit ft5x06_remove(struct i2c_client *client)
{
	struct ft5x06 *ts;
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_INFO "%s() - Driver is unregistering.\n", __FUNCTION__);
#endif
	/* clientdata registered on probe */
	ts = i2c_get_clientdata(client);
	device_remove_file(&ts->client->dev, &dev_attr_irq_enable);

	/* Start cleaning up by removing any delayed work and the timer */
	if (cancel_delayed_work_sync((struct delayed_work *)&ts->work) < 0) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not remove all work from the Work Queue.\n",
		       __FUNCTION__);
	}

	/* free up timer or irq */
	if (ts->client->irq == 0) {
	} else {
		free_irq(client->irq, ts);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	/* housekeeping */
	/* Wait until any outstanding SYSFS transaction has finished,
	 * and prevent any new ones from starting.
	 */
	mutex_lock(&ts->device_mode_mutex);
	/* Remove the SYSFS entries */
	sysfs_remove_group(&client->dev.kobj, &ft5x06_attribute_group);
	mutex_unlock(&ts->device_mode_mutex);
	mutex_destroy(&ts->device_mode_mutex);

	if (NULL != ts) {
		kfree(ts);
	}
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_INFO "%s() - Driver unregistration is complete.\n",
	       __FUNCTION__);
#endif
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x06_early_suspend(struct early_suspend *handler)
{
	struct ft5x06 *ts;
	ts = container_of(handler, struct ft5x06, early_suspend);
	ft5x06_suspend(ts->client, PMSG_SUSPEND);
}

static void ft5x06_late_resume(struct early_suspend *handler)
{
	struct ft5x06 *ts;
	ts = container_of(handler, struct ft5x06, early_suspend);
	ft5x06_resume(ts->client);
}
#else /* CONFIG_HAS_EARLYSUSPEND */
static void ft5x06_early_suspend(struct early_suspend *handler)
{
	/* Do Nothing */
}

static void ft5x06_late_resume(struct early_suspend *handler)
{
	/* Do Nothing */
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int ft5x06_init(void)
{
	int ret = 0;
#if FT5x06_DEBUG_VERBOSE
	printk(KERN_INFO "%s() - FT I2C Touchscreen Driver (Built %s @ %s)\n",
	       __FUNCTION__, __DATE__, __TIME__);
#endif
	ft5x06_ts_wq = create_singlethread_workqueue("ft5x06_ts_wq");
	if (NULL == ft5x06_ts_wq) {
		printk(KERN_ERR
		       "%s() - ERROR: Could not create the Work Queue due to insufficient memory.\n",
		       __FUNCTION__);
		ret = -ENOMEM;
	} else {
		ret = i2c_add_driver(&ft5x06_driver);
	}

	return ret;
}

static void ft5x06_exit(void)
{
	if (ft5x06_ts_wq) {
		destroy_workqueue(ft5x06_ts_wq);
	}

	return i2c_del_driver(&ft5x06_driver);
}

module_init(ft5x06_init);
module_exit(ft5x06_exit);
