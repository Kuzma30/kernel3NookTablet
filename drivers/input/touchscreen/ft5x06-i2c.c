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
#define PRINTK_COLORS
#define CONFIG_TOUCHSCREEN_FT5x06_TEST

#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/input/ft5x06.h>

#if defined(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#endif /* defined(CONFIG_DEBUG_FS) */

#ifdef CONFIG_TOUCHSCREEN_FT5x06_TEST
#include <asm/uaccess.h>

#define FT5x06_SCAN_DELAY_MS            20
#define FT5x06_CALIBRATION_DELAY_MS   300
#define FT5x06_TPK_UPGRADEVER1		0x5C
#define FT5x06_NONE_UPGRADEVER		0xCD
#define FT5x06_WINTEK_UPGRADEVER	0x89
#define FT5x06_FWID_REG			0xA8
#define FT5x06_TPK_FWID				0x79
#define FT5x06_WINTEK_FWID			0x89
#define NUM_RXCAC_REGS                  30
#endif

#define FTX_TAG "FTX"

#define FT5x06_CPTM_ID_COMPANY        0x79
#define FT5x06_CPTM_ID_PRODUCT        0x03
#define FT5606_CPTM_ID_PRODUCT        0x06
#define FT5506_CPTM_ID_PRODUCT        0x05

#define FT5x06_UPGRADE_H_DELAY		50
#define FT5x06_UPGRADE_L_DELAY		30
#define FT5x06_UPGRADE_ENTER_UPGRADE_MODE_DELAY	1
#define FT5x06_UPGRADE_ERASE_DELAY	2000
#define FT5x06_UPGRADE_PACKET_DELAY	20
#define FT5x06_UPGRADE_CHECKSUM_DELAY	1
#define FT5x06_UPGRADE_RESET_FW_DELAY	300

#define FT5606_UPGRADE_H_DELAY		50
#define FT5606_UPGRADE_L_DELAY		10
#define FT5606_UPGRADE_ENTER_UPGRADE_MODE_DELAY	100
#define FT5606_UPGRADE_ERASE_DELAY	2000
#define FT5606_UPGRADE_PACKET_DELAY	20
#define FT5606_UPGRADE_CHECKSUM_DELAY	1
#define FT5606_UPGRADE_RESET_FW_DELAY	300

#ifdef CONFIG_MACH_OMAP_HUMMINGBIRD
/* FT55 */
#define FT5x06_NUM_TX                   32
#define FT5x06_NUM_RX                   20
#else
/* FT56 */
#define FT5x06_NUM_TX                   38
#define FT5x06_NUM_RX                   26
#endif

#define FT5x06_PACKET_LENGTH           128
#define FT5x06_VERIFY_NUM_TRIES         10
#define FT5x06_CALIBRATION_NUM_BYTES     (FT5x06_NUM_RX)
#define FT5x06_SCAN_DELAY               200

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

#define FACTORY_MODE0	4
#define FACTORY_MODE1	6
#define FACTORY_MODE2	7
#define FACTORY_MODE3	3
#define FT5606_TEST_MODE_RX	20

enum ftx_factory_mode
{
	FACTORY_MODE_0,
	FACTORY_MODE_1,
	FACTORY_MODE_2,
	FACTORY_MODE_3,
	FACTORY_MODE_MAX
};

static u8 ft5606_factory_mode[FACTORY_MODE_MAX] = {
	[FACTORY_MODE_0] = FACTORY_MODE0,
	[FACTORY_MODE_1] = FACTORY_MODE1,
	[FACTORY_MODE_2] = FACTORY_MODE2,
	[FACTORY_MODE_3] = FACTORY_MODE3,
};

enum ftx_power_supply_state
{
	FTX_POWER_SUPPLY_OFF = 0,
	FTX_POWER_SUPPLY_ON
};

enum ftx_suspend_state
{
	FTX_SUSPEND = 0,
	FTX_ACTIVE
};
#define FTX_MAX_FINGERS 10
#define SWAP(a,b) \
			{ \
				(a) ^= (b); \
				(b) ^= (a); \
				(a) ^= (b); \
			}

typedef enum __dbg_level
{
	dbg_level_highest = 0,
	dbg_level_critical = dbg_level_highest,
	dbg_level_error,
	dbg_level_warning,
	dbg_level_info,
	dbg_level_debug,
	dbg_level_verbose,
	dbg_level_lowest
}dbg_level;
#define DBG_PRINT(msg_level,...) \
	{ \
		if((msg_level) <= (cur_dbg_level)) \
		{ \
			printk(KERN_INFO __VA_ARGS__); \
		} \
	}
#define DBG_PRINT_HEX(msg_level,...) \
	{ \
		if((msg_level) <= (cur_dbg_level)) \
		{ \
			print_hex_dump(KERN_INFO, ##__VA_ARGS__); \
		} \
	}

static int cur_dbg_level = dbg_level_highest;
module_param_named(debug_level, cur_dbg_level, int, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(debug_level, "Debug Level");

enum ftx_mt_protocol
{
	FTX_MT_PROTOCOL_NONE = 0,
	FTX_MT_PROTOCOL_A,
	FTX_MT_PROTOCOL_B,
	FTX_MT_PROTOCOL_MAX
};

typedef struct ft5x06_xydata
{
	unsigned x_h : 4;
	unsigned reserved_01 : 2;
	unsigned event : 2;
	unsigned x_l : 8;
	unsigned y_h : 4;
	unsigned tch_id : 4;
	unsigned y_l : 8;
	unsigned pressure : 8;
	unsigned reserved_02 : 4;
	unsigned area: 4;
}ft5x06_xydata_t;

typedef struct ft5x06_fingers
{
	unsigned long n_fingers;
	unsigned long n_active_fingers;
	unsigned long fingers_mask;
	ft5x06_xydata_t *fingers;
}ft5x06_fingers_t;

typedef struct ft5x06_info
{
	u8 lib_ver_h;
	u8 lib_ver_l;
	u8 fw_ver;
	u8 focaltech_id;
	/*
	 * This is not part of the controller info
	 * Put this always at the end
	 */
	u8 product_id;
}ft5x06_info_t;

typedef struct ft5x06_upgrade
{
    unsigned long upgrade_h_delay;
    unsigned long upgrade_l_delay;
    unsigned long upgrade_enter_upgrade_mode_delay;
    unsigned long upgrade_erase_delay;
    unsigned long upgrade_packet_delay;
    unsigned long upgrade_checksum_delay;
    unsigned long upgrade_reset_fw_delay;
}ft5x06_upgrade_t;

struct ft5x06 {
	char *device_name;
    struct i2c_client *client;
    struct input_dev *input;
    ft5x06_info_t info;
    struct workqueue_struct *ft5x06_ts_wq;
    struct work_struct work;
    struct ft5x06_platform_data *platform_data;
    u8 crosstalk_test_type;
    u8 factory_mode_register;
    u8 working_mode_register;
    atomic_t irq_enabled;
    atomic_t uses_mt_slots;
    atomic_t uses_events;

    struct mutex lock_fingers;
    atomic_t n_fingers_supported;
    ft5x06_fingers_t cur_fingers;
    ft5x06_fingers_t prev_fingers;

    u8 prev_gest;
    u8 gest_count;
    u8 report_rate;
    /* Ensures that only one function can specify the Device Mode at a time. */
    struct mutex device_mode_mutex;
    struct early_suspend early_suspend;
    int gesture_in_progress;
	int ftx_power_supply_state;
	struct mutex lock_ftx_power_supply_state;
	int ftx_suspend_state;
	struct mutex lock_ftx_suspend_state;
	atomic_t do_not_suspend_ftx;
	atomic_t force_resume_ftx;
	atomic_t power_on_ftx;
	ft5x06_upgrade_t ftx_upgrade_info;

#if defined(CONFIG_DEBUG_FS)
	struct dentry *dbgfs_root;
	struct ftx_dbgfs_entry *ftx_dbgfs_entries;

	struct mutex ftx_raw_data_mutex;
	u16 *ftx_raw_data;
	u16 ftx_raw_data_offset;
	int ftx_raw_data_tx_start, ftx_raw_data_tx_end;
	int ftx_raw_data_rx_start, ftx_raw_data_rx_end;
#endif /* defined(CONFIG_DEBUG_FS) */
};

#if defined(CONFIG_DEBUG_FS)
typedef struct ftx_dbgfs_entry
{
	char *name;
	struct dentry *dbgfs_file;
	ssize_t (*read)(struct ft5x06 *ts, struct file *file, char __user *buf, size_t count, loff_t *offset);
	ssize_t (*write)(struct ft5x06 *ts, struct file *file, const char __user *buf, size_t count, loff_t *ppos);
	struct ft5x06 *ts;
}ftx_dbgfs_entry_t;
#endif /* defined(CONFIG_DEBUG_FS) */

#define FT_TOUCH_EVENT_PRESS_DOWN 0x00
#define FT_TOUCH_EVENT_LIFT_UP 0x01
#define FT_TOUCH_EVENT_CONTACT 0x02
#define FT_TOUCH_EVENT_NO_EVENT 0x03

/*****************************************************************************
 * Function Prototypes
 ****************************************************************************/

static void ft5x06_process_touch(struct work_struct *work);
static irqreturn_t ft5x06_irq(int irq, void *handle);

static int __devinit ft5x06_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ft5x06_remove(struct i2c_client *client);
static void  ft5x06_shutdown(struct i2c_client *client);

static int  ft5x06_resume(struct device *dev);
static int  ft5x06_suspend(struct device *dev);

static void ft5x06_early_suspend(struct early_suspend *handler);
static void ft5x06_late_resume(struct early_suspend *handler);

#if defined(CONFIG_DEBUG_FS)
static int ftx_dbgfs_open(struct inode *inode, struct file *file);
static int ftx_dbgfs_release(struct inode *inode, struct file *file);
static ssize_t ftx_dbgfs_object_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t ftx_dbgfs_object_read(struct file *file, char __user *buf, size_t count, loff_t *offset);
static int ftx_dbgfs_create(struct ft5x06 *ts);
static int ftx_dbgfs_destroy(struct ft5x06 *ts);
#endif /* defined(CONFIG_DEBUG_FS) */

static int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
static int ft5x06_read_rawdata(struct i2c_client *client, int data[][FT5x06_NUM_RX]);

static int readCfg(char *filepath, int *pNum, int *pDelta);
static int writebuftofile(char * filepath, char *buf);
/*****************************************************************************
 * Global Variables
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FS)

static int ftx_get_raw_data(struct ft5x06 *ts)
{
	int iLoop, jLoop;
	int retval;
	unsigned int bytes_written = 0;
	u8 regval;
	int readlen;
	int max_data_points;

	mutex_lock(&ts->ftx_raw_data_mutex);

	if(
		(ts->ftx_raw_data == NULL)
		|| (ts->ftx_raw_data_tx_start < 0)
		|| (ts->ftx_raw_data_tx_end < 0)
		|| (ts->ftx_raw_data_rx_start < 0)
		|| (ts->ftx_raw_data_rx_end < 0)
		|| (ts->ftx_raw_data_tx_end >= ts->platform_data->max_tx_lines)
		|| (ts->ftx_raw_data_rx_end >= ts->platform_data->max_rx_lines)
	  )
	{
		retval = -EINVAL;
		goto error_return;
	}

	if(ts->client->irq)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Disabling touch interrupt.\n", dev_name(&(ts->client->dev)), __func__);
		disable_irq(ts->client->irq);
	}

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Putting touch panel in factory mode.\n", dev_name(&(ts->client->dev)), __func__);
	retval = i2c_smbus_write_byte_data(ts->client, FT5x06_WMREG_DEVICE_MODE, FT5x06_MODE_FACTORY);
	if(0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_enter_factory_mode;
	}

	msleep(100);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that touch panel is in factory mode.\n", dev_name(&(ts->client->dev)), __func__);
	retval = i2c_smbus_read_byte_data(ts->client, FT5x06_FMREG_DEVICE_MODE);
	if(0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_enter_factory_mode;
	}
	regval = retval & 0x000000FF;
	if((regval & FT5x06_MODE_MASK) != FT5x06_MODE_FACTORY)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Touch panel is not in factory mode; DEVICE MODE register contains 0x%02x\n", dev_name(&(ts->client->dev)), __func__, regval);
		retval = FT5x06_ERR_NOT_FACTORY_MODE;
		goto error_enter_factory_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Initiating a scan for raw data...\n", dev_name(&(ts->client->dev)), __func__);
	regval |=  FT5x06_SCAN_START;
	retval = i2c_smbus_write_byte_data(ts->client, FT5x06_FMREG_DEVICE_MODE, regval);
	if(0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(ts->client->dev)), __func__);
		goto error_initiate_scan;
	}

	msleep(FT5x06_SCAN_DELAY);

	retval = i2c_smbus_read_byte_data(ts->client, FT5x06_FMREG_DEVICE_MODE);
	if(0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read updated value of DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_read_scan_status;
	}
	regval = retval & 0x000000FF;
	if(FT5x06_SCAN_DONE != (regval & FT5x06_SCAN_MASK))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Raw data did not complete after %u ms.\n", dev_name(&(ts->client->dev)), __func__, FT5x06_SCAN_DELAY);
		retval = FT5x06_ERR_SCAN_NOT_DONE;
		goto error_scan_not_done;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reading raw data...\n", dev_name(&(ts->client->dev)), __func__);

	max_data_points = (ts->ftx_raw_data_tx_end - ts->ftx_raw_data_tx_start + 1) * (ts->ftx_raw_data_rx_end - ts->ftx_raw_data_rx_start + 1);
	memset(ts->ftx_raw_data, 0x00, max_data_points * 2);
	for(iLoop = ts->ftx_raw_data_tx_start; iLoop <= ts->ftx_raw_data_tx_end; iLoop++)
	{
		retval = i2c_smbus_write_byte_data(ts->client, FT5x06_FMREG_ROW_ADDR, iLoop);
		if (0 > retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write the row number %u\n", dev_name(&(ts->client->dev)), __func__, iLoop);
			goto error_read_raw_data;
		}

		msleep(1);

		if(ts->ftx_raw_data_rx_start < FT5606_TEST_MODE_RX)
		{
			regval = (ts->ftx_raw_data_rx_end + 1 - ts->ftx_raw_data_rx_start);
			readlen = regval > FT5606_TEST_MODE_RX ? FT5606_TEST_MODE_RX*2 : regval*2;
			regval = FT5x06_FMREG_RAWDATA_0_H + ts->ftx_raw_data_rx_start*2;

			retval = ft5x0x_i2c_Read(ts->client, &regval, 1,  (u8*)ts->ftx_raw_data + bytes_written, readlen);
			if (0 > retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read row %u raw data.\n", dev_name(&(ts->client->dev)), __func__, iLoop);
				goto error_read_raw_data;
			}
			bytes_written += readlen;
		}
		if(ts->ftx_raw_data_rx_end >=  FT5606_TEST_MODE_RX)
		{
			for(jLoop = 1; jLoop < ((ts->ftx_raw_data_rx_end+1)/FT5606_TEST_MODE_RX + (((ts->ftx_raw_data_rx_end+1) % FT5606_TEST_MODE_RX)?1:0)); jLoop++)
			{
				retval = i2c_smbus_write_byte_data(ts->client, FT5x06_FMREG_DEVICE_MODE, ft5606_factory_mode[jLoop] << 4);
				if (0 < retval)
				{
					DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not change to factory mode %d...\n", dev_name(&(ts->client->dev)), __func__, jLoop);
					goto error_read_raw_data;
				}

				if(ts->ftx_raw_data_rx_start >= FT5606_TEST_MODE_RX*jLoop)
				{
					regval = (ts->ftx_raw_data_rx_end  + 1 - ts->ftx_raw_data_rx_start);
					readlen = regval > FT5606_TEST_MODE_RX ? FT5606_TEST_MODE_RX*2 : regval*2;
					regval = FT5x06_FMREG_RAWDATA_0_H + (ts->ftx_raw_data_rx_start - FT5606_TEST_MODE_RX*jLoop)*2;
				}
				else
				{
					regval = (ts->ftx_raw_data_rx_end + 1 - FT5606_TEST_MODE_RX*jLoop) ;
					readlen = regval > FT5606_TEST_MODE_RX ? FT5606_TEST_MODE_RX*2 : regval*2;
					regval = FT5x06_FMREG_RAWDATA_0_H;
				}

				retval = ft5x0x_i2c_Read(ts->client, &regval, 1,  (u8*)ts->ftx_raw_data + bytes_written, readlen);
				if (0 > retval)
				{
					DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read row %u raw data.\n", dev_name(&(ts->client->dev)), __func__, iLoop);
					goto error_read_raw_data;
				}
				bytes_written += readlen;
			}
			retval = i2c_smbus_write_byte_data(ts->client, FT5x06_FMREG_DEVICE_MODE, ft5606_factory_mode[FACTORY_MODE_0] << 4);
			if (0 < retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not change to factory mode 0...\n", dev_name(&(ts->client->dev)), __func__);
				goto error_read_raw_data;
			}
		}

	}
	for(iLoop=0; iLoop < bytes_written/2; iLoop++)
	{
		ts->ftx_raw_data[iLoop] = ((ts->ftx_raw_data[iLoop] >> 8) & 0xFF) | ((ts->ftx_raw_data[iLoop] << 8) & 0xFF00);
	}

error_read_raw_data:
error_scan_not_done:
error_read_scan_status:
error_initiate_scan:
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Putting touch panel in working mode.\n", dev_name(&(ts->client->dev)), __func__);
	retval = i2c_smbus_write_byte_data(ts->client, FT5x06_WMREG_DEVICE_MODE, FT5x06_MODE_WORKING);
	if(0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
	}
	msleep(100);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that touch panel is in working mode.\n", dev_name(&(ts->client->dev)), __func__);
	retval = i2c_smbus_read_byte_data(ts->client, FT5x06_FMREG_DEVICE_MODE);
	if(0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
	}
	regval = retval & 0x000000FF;
	if((regval & FT5x06_MODE_MASK) != FT5x06_MODE_WORKING)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Touch panel is not in working mode; DEVICE MODE register contains 0x%02x\n", dev_name(&(ts->client->dev)), __func__, regval);
		retval = FT5x06_ERR_NOT_WORKING_MODE;
	}
error_enter_factory_mode:
	if(ts->client->irq)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Enabling touch interrupt.\n", dev_name(&(ts->client->dev)), __func__);
		enable_irq(ts->client->irq);
	}
error_return:
	mutex_unlock(&ts->ftx_raw_data_mutex);
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Read %d bytes.\n", dev_name(&(ts->client->dev)), __func__, bytes_written);
	return bytes_written;
}


static ssize_t ftx_dbgfs_write_raw_data_params(struct ft5x06 *ts, struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char *write_buf;
	u16 *tmp_raw_data;
	struct ftx_dbgfs_entry *tmp_dbgfs_entry = file->private_data;
	int tmp_tx_start, tmp_rx_start, tmp_tx_end, tmp_rx_end;
	int tmp_raw_data_offset;
	int retval;
	int buf_pos, buf_offset;
	int max_data_points;

	write_buf = kzalloc(count, GFP_KERNEL);
	if(write_buf == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: No memory for write buffer to %s.\n", dev_name(&ts->client->dev), __func__, tmp_dbgfs_entry->name);
		retval = -ENOMEM;
		goto err_alloc_write_buf;
	}

	if(copy_from_user(write_buf, buf, count))
	{
		retval = -EFAULT;
		goto err_copy_from_user;
	}

	buf_pos =0;
	buf_offset = 0;
	retval = sscanf(write_buf+buf_pos, "%d%n", &tmp_tx_start, &buf_offset);
	if (retval <= 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Bad format for tx_start.\n", dev_name(&ts->client->dev),__func__);
		retval = -EINVAL;
		goto err_bad_format;
	}
	buf_pos += buf_offset;

	retval = sscanf(write_buf+buf_pos, "%d%n", &tmp_rx_start, &buf_offset);
	if (retval <= 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Bad format for rx_start.\n", dev_name(&ts->client->dev),__func__);
		retval = -EINVAL;
		goto err_bad_format;
	}
	buf_pos += buf_offset;

	retval = sscanf(write_buf+buf_pos, "%d%n", &tmp_tx_end, &buf_offset);
	if (retval <= 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Bad format for tx_end.\n", dev_name(&ts->client->dev),__func__);
		retval = -EINVAL;
		goto err_bad_format;
	}
	buf_pos += buf_offset;

	retval = sscanf(write_buf+buf_pos, "%d%n", &tmp_rx_end, &buf_offset);
	if (retval <= 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Bad format for rx_end.\n", dev_name(&ts->client->dev),__func__);
		retval = -EINVAL;
		goto err_bad_format;
	}
	buf_pos += buf_offset;

	retval = sscanf(write_buf+buf_pos, "%d%n", &tmp_raw_data_offset, &buf_offset);
	if (retval <= 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Bad format for raw_data_offset.\n", dev_name(&ts->client->dev),__func__);
		retval = -EINVAL;
		goto err_bad_format;
	}
	buf_pos += buf_offset;

	if(
		(tmp_tx_start >= 0)
		&& (tmp_rx_start >= 0)
		&& (tmp_tx_end < ts->platform_data->max_tx_lines)
		&& (tmp_rx_end < ts->platform_data->max_rx_lines)
	  )
	{
		mutex_lock(&ts->ftx_raw_data_mutex);
		ts->ftx_raw_data_tx_start = tmp_tx_start;
		ts->ftx_raw_data_tx_end = tmp_tx_end;
		ts->ftx_raw_data_rx_start = tmp_rx_start;
		ts->ftx_raw_data_rx_end = tmp_rx_end;
		ts->ftx_raw_data_offset = tmp_raw_data_offset;
		mutex_unlock(&ts->ftx_raw_data_mutex);

		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: tx_start=%d,\trx_start=%d,\ttx_end=%d,\trx_end=%d,\toffset=%d\n", dev_name(&ts->client->dev), __func__, tmp_tx_start, tmp_rx_start, tmp_tx_end, tmp_rx_end, tmp_raw_data_offset);
	}
	else
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Invalid input: tx_start=%d,\trx_start=%d,\ttx_end=%d,\trx_end=%d,\toffset=%d\n", dev_name(&ts->client->dev), __func__, tmp_tx_start, tmp_rx_start, tmp_tx_end, tmp_rx_end, tmp_raw_data_offset);
		goto err_bad_format;
	}

err_alloc_raw_data_buf:
err_bad_format:
err_copy_from_user:
	kfree(write_buf);
err_alloc_write_buf:
	return count;
}

static ssize_t ftx_dbgfs_read_raw_data_params(struct ft5x06 *ts, struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct ftx_dbgfs_entry *tmp_dbgfs_entry = file->private_data;
	char *usr_buf;
	int chars_read = 0;

	usr_buf = kzalloc(256, GFP_KERNEL); /* xxx,xxx,xxx,xxx */
	if(usr_buf == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: No memory for usr buffer for %s.\n", dev_name(&ts->client->dev), __func__, tmp_dbgfs_entry->name);
		chars_read = -ENOMEM;
		goto err_alloc_usr_buf;
	}

	if(*offset == 0)
	{
		mutex_lock(&ts->ftx_raw_data_mutex);
		chars_read += sprintf(usr_buf+chars_read, "tx_start      =%3d\nrx_start      =%3d\ntx_end        =%3d\nrx_end        =%3d\noffset        =%5d\n", ts->ftx_raw_data_tx_start, ts->ftx_raw_data_rx_start, ts->ftx_raw_data_tx_end, ts->ftx_raw_data_rx_end, ts->ftx_raw_data_offset);
		mutex_unlock(&ts->ftx_raw_data_mutex);

		if(copy_to_user(buf, usr_buf, chars_read))
		{
			chars_read = -EFAULT;
			goto err_copy_to_user;
		}
		*offset += chars_read;
	}

err_copy_to_user:
	kfree(usr_buf);
err_alloc_usr_buf:
err_return:
	return chars_read;
}

static ssize_t ftx_dbgfs_read_raw_data(struct ft5x06 *ts, struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct ftx_dbgfs_entry *tmp_dbgfs_entry = file->private_data;
	char *usr_buf;
	int chars_to_read, bytes_to_read, chars_read, bytes_read;
	int iLoop;
	int max_data_points, available_data_points;


	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Requesting %d bytes from %llu\n", dev_name(&ts->client->dev), __func__, count, *offset);

	bytes_read = 0;
	chars_read = 0;

	mutex_lock(&ts->ftx_raw_data_mutex);
	max_data_points = (ts->ftx_raw_data_tx_end + 1 - ts->ftx_raw_data_tx_start) * (ts->ftx_raw_data_rx_end + 1 - ts->ftx_raw_data_rx_start);
	if(*offset == 0)
	{
		mutex_unlock(&ts->ftx_raw_data_mutex);
		iLoop = ftx_get_raw_data(ts);
		if(iLoop <= 0)
		{
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Error in collecting raw data\n", dev_name(&ts->client->dev), __func__);
			mutex_lock(&ts->ftx_raw_data_mutex);
			goto err_return;
		}
		mutex_lock(&ts->ftx_raw_data_mutex);
	}
	else if(*offset >= max_data_points*2)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Wrong offset %llu, only %d bytes available\n", dev_name(&ts->client->dev), __func__, *offset, max_data_points*2);
		goto err_return;
	}

	available_data_points = max_data_points - (*offset/2);

	if(count >= available_data_points*7) /* Each data point is 2 bytes(0 - 65535), which is 6 characters and a space */
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: More bytes %d requested, only %d bytes available\n", dev_name(&ts->client->dev), __func__, count, max_data_points*2);
		bytes_to_read = available_data_points * 2;
	}
	else
	{
		bytes_to_read = (count/7)*2; /* This is integer operation. Dont cancel out */
	}
	chars_to_read = (bytes_to_read/2) * 7;


	usr_buf = kzalloc(chars_to_read, GFP_KERNEL);
	if(usr_buf == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: No memory for usr buffer for %s.\n", dev_name(&ts->client->dev), __func__, tmp_dbgfs_entry->name);
		chars_read = -ENOMEM;
		goto err_alloc_usr_buf;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reading %d bytes(%d chars) from %s.\n", dev_name(&ts->client->dev), __func__, bytes_to_read, chars_to_read, tmp_dbgfs_entry->name);
	if(ts->ftx_raw_data)
	{
		for(iLoop = (*offset/2); bytes_read < bytes_to_read; iLoop++)
		{
			chars_read += sprintf(usr_buf+chars_read, "% 6d\t", ts->ftx_raw_data[iLoop] - ts->ftx_raw_data_offset);
			bytes_read += 2;
			if(((iLoop + 1)%(ts->ftx_raw_data_rx_end - ts->ftx_raw_data_rx_start + 1)) == 0)
			{
				usr_buf[chars_read-1] = '\n';
			}
		}
	}

	if(copy_to_user(buf, usr_buf, chars_read))
	{
		chars_read = -EFAULT;
		goto err_copy_to_user;
	}
	*offset += bytes_read;
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Read %d bytes (%d chars) from %s.\n", dev_name(&ts->client->dev), __func__, bytes_read, chars_read, tmp_dbgfs_entry->name);

err_copy_to_user:
	kfree(usr_buf);
err_alloc_usr_buf:
err_return:
	mutex_unlock(&ts->ftx_raw_data_mutex);
	return chars_read;
}

ftx_dbgfs_entry_t ftx_dbgfs_files[] = {
	{
		.name = "raw_data_params",
		.dbgfs_file = NULL,
		.read = ftx_dbgfs_read_raw_data_params,
		.write = ftx_dbgfs_write_raw_data_params,
		.ts = NULL,
	},
	{
		.name = "raw_data",
		.dbgfs_file = NULL,
		.read = ftx_dbgfs_read_raw_data,
		.write = NULL,
		.ts = NULL,
	},
	{0}
};

static int ftx_dbgfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
};
static int ftx_dbgfs_release(struct inode *inode, struct file *file)
{
	return 0;
};

static ssize_t ftx_dbgfs_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct ftx_dbgfs_entry *tmp_dbgfs_entry = file->private_data;

	if(tmp_dbgfs_entry->write)
	{
		return (tmp_dbgfs_entry->write(tmp_dbgfs_entry->ts, file, buf, count, ppos));
	}
	else
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Error in writing to %s\n", dev_name(&tmp_dbgfs_entry->ts->client->dev), __func__, tmp_dbgfs_entry->name);
		return -EINVAL;
	}
}

static ssize_t ftx_dbgfs_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct ftx_dbgfs_entry *tmp_dbgfs_entry = file->private_data;

	if(tmp_dbgfs_entry->read)
	{
		return (tmp_dbgfs_entry->read(tmp_dbgfs_entry->ts, file, buf, count, offset));
	}
	else
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Error in reading from to %s\n", dev_name(&tmp_dbgfs_entry->ts->client->dev), __func__, tmp_dbgfs_entry->name);
		return -EINVAL;
	}

}

static struct file_operations ftx_dbg_file_ops = {
	.owner   = THIS_MODULE,
	.open    = ftx_dbgfs_open,
	.read    = ftx_dbgfs_read,
	.write   = ftx_dbgfs_write,
//	.llseek  = ftx_dbgfs_lseek,
	.release = ftx_dbgfs_release
};

static int ftx_dbgfs_create(struct ft5x06 *ts)
{
	int ret;
	int n_files_created = 0;
	struct ftx_dbgfs_entry *tmp_dbgfs_entry = ts->ftx_dbgfs_entries;

	ts->dbgfs_root = debugfs_create_dir("ft5x06", NULL);
	if(ts->dbgfs_root == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Error in creating debugfs root.\n", dev_name(&ts->client->dev), __func__);
		ret = -ENOMEM;
		goto err_create_root;
	}

	while(tmp_dbgfs_entry->name != NULL)
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Creating debugfs file %s.\n", dev_name(&ts->client->dev), __func__, tmp_dbgfs_entry->name);
		tmp_dbgfs_entry->ts = ts;
		tmp_dbgfs_entry->dbgfs_file = debugfs_create_file(tmp_dbgfs_entry->name, 0644, ts->dbgfs_root, tmp_dbgfs_entry, &ftx_dbg_file_ops);
		if(tmp_dbgfs_entry->dbgfs_file == NULL)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Error in creating debugfs file %s.\n", dev_name(&ts->client->dev), __func__, tmp_dbgfs_entry->name);
			tmp_dbgfs_entry++;
			continue;
		}
		tmp_dbgfs_entry++;
		n_files_created++;
	}
	ret = 0;
	goto err_create_root;

//err_create_files:
//	debugfs_remove(ts->dbgfs_root);
err_create_root:
	return ret;
}

static int ftx_dbgfs_destroy(struct ft5x06 *ts)
{
	struct ftx_dbgfs_entry *tmp_dbgfs_entry = ts->ftx_dbgfs_entries;

	while((tmp_dbgfs_entry != NULL) && (tmp_dbgfs_entry->name != NULL))
	{
		if(tmp_dbgfs_entry->dbgfs_file != NULL)
		{
			debugfs_remove(tmp_dbgfs_entry->dbgfs_file);
		}
		tmp_dbgfs_entry++;
	}

	debugfs_remove(ts->dbgfs_root);
	return 0;
}
#endif /* CONFIG_DEBUG_FS */



static ssize_t ft5x06_irq_enabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct ft5x06 *ts = i2c_get_clientdata(client);

    return sprintf(buf, "%u\n", atomic_read(&ts->irq_enabled));
}
static ssize_t ft5x06_irq_enabled_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct ft5x06 *ts = i2c_get_clientdata(client);

    int err = 0;
    unsigned long value;


    if (size > 2)
    {
        return -EINVAL;
    }

    err = strict_strtoul(buf, 10, &value);
    if (err != 0)
    {
        return err;
    }

    switch (value)
    {
        case 0:
            if (atomic_cmpxchg(&ts->irq_enabled, 1, 0))
            {
                disable_irq(ts->client->irq);
                DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Touch Panel IRQ %u has been DISABLED.\n", dev_name(dev),__func__, ts->client->irq);
            }
            err = size;
            break;

        case 1:
            if (!atomic_cmpxchg(&ts->irq_enabled, 0, 1))
            {
                enable_irq(ts->client->irq);
                DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Touch Panel IRQ %u has been ENABLED.\n", dev_name(dev),__func__, ts->client->irq);
            }
            err = size;
            break;

        default:
            DBG_PRINT(dbg_level_warning, "%s: " FTX_TAG ": %s(): DEBUG: Invalid input specified (%lu). Touch Panel IRQ %u --> irq_enabled = %d\n", dev_name(dev), __func__, value, ts->client->irq, atomic_read(&ts->irq_enabled));
            err = -EINVAL;
            break;
    }

    return err;
}
static DEVICE_ATTR(irq_enabled, 0664, ft5x06_irq_enabled_show, ft5x06_irq_enabled_store);

static int ftx_read_info(struct ft5x06 *ts)
{
	u8  buffer[FT5x06_WMREG_FOCALTECH_ID - FT5x06_WMREG_LIB_VER_H + 1] = {0};
	int retval;

	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_LIB_VER_H, sizeof(u8) * (FT5x06_WMREG_FOCALTECH_ID - FT5x06_WMREG_LIB_VER_H + 1), buffer);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: I2C bus read error.\n", dev_name(&(ts->client->dev)), __func__);
		return retval;
	}
	ts->info.lib_ver_h    = buffer[FT5x06_WMREG_LIB_VER_H    - FT5x06_WMREG_LIB_VER_H];
	ts->info.lib_ver_l    = buffer[FT5x06_WMREG_LIB_VER_L    - FT5x06_WMREG_LIB_VER_H];
	ts->info.fw_ver       = buffer[FT5x06_WMREG_FW_VER       - FT5x06_WMREG_LIB_VER_H];
	ts->info.focaltech_id = buffer[FT5x06_WMREG_FOCALTECH_ID - FT5x06_WMREG_LIB_VER_H];

	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_CONTROLLER_ID, sizeof(u8), buffer);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: I2C bus read error.\n", dev_name(&(ts->client->dev)), __func__);
		buffer[0] = FT5606_CPTM_ID_PRODUCT; /* If FW does not support the Product ID register, act as a FT5606 */
	}
	if((buffer[0] == FT5606_CPTM_ID_PRODUCT) || (buffer[0] == FT5506_CPTM_ID_PRODUCT))
	{
		ts->info.product_id = buffer[0];
	}
	else /* Default to FT5606 */
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): INFO: Unsupported Product ID 0x%02x...defaulting to 5606.\n", dev_name(&(ts->client->dev)), __func__, buffer[0]);
		ts->info.product_id = FT5606_CPTM_ID_PRODUCT;
	}

	return 0;
}

static int ftx_allocate_fingers(struct ft5x06 *ts, int n_fingers)
{
	int ret;

	mutex_lock(&ts->lock_fingers);
	if(atomic_read(&ts->n_fingers_supported) != 0)
	{
		kfree(ts->prev_fingers.fingers);
		ts->prev_fingers.fingers = NULL;
		kfree(ts->cur_fingers.fingers);
		ts->cur_fingers.fingers = NULL;
		atomic_set(&ts->n_fingers_supported, 0);
	}
	if(n_fingers == 0)
	{
		ret = 0;
		goto err_success;
	}

	ts->cur_fingers.fingers = kzalloc(sizeof(ft5x06_xydata_t) * n_fingers, GFP_KERNEL);
	if (NULL == ts->cur_fingers.fingers)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not allocate memory for current fingers.\n", dev_name(&(ts->client->dev)), __func__);
		ret = -ENOMEM;
		goto err_cur_fingers;
	}

	ts->prev_fingers.fingers = kzalloc(sizeof(ft5x06_xydata_t) * n_fingers, GFP_KERNEL);
	if (NULL == ts->cur_fingers.fingers)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not allocate memory for prev fingers.\n", dev_name(&(ts->client->dev)), __func__);
		ret = -ENOMEM;
		goto err_prev_fingers;
	}

	atomic_set(&ts->n_fingers_supported, n_fingers);
	ret = n_fingers;
	goto err_success;

err_prev_fingers:
	kfree(ts->cur_fingers.fingers);
	ts->cur_fingers.fingers = NULL;
err_cur_fingers:
err_success:
	mutex_unlock(&ts->lock_fingers);
	return ret;
}

static int ftx_input_switch_protocol(struct ft5x06 *ts, int use_slots)
{
	switch(use_slots)
	{
	case FTX_MT_PROTOCOL_NONE:
	/* Support ST events */
#if defined(FTX_SUPPORT_ST)
		atomic_set(&ts->uses_mt_slots, FTX_MT_PROTOCOL_NONE);
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: ST device active.\n", dev_name(&(ts->client->dev)), __func__);
#endif /* defined(FTX_SUPPORT_ST) */
		break;
	case FTX_MT_PROTOCOL_A:
		atomic_set(&ts->uses_mt_slots, FTX_MT_PROTOCOL_A);
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Type A device active.\n", dev_name(&(ts->client->dev)), __func__);
		break;
	case FTX_MT_PROTOCOL_B:
		atomic_set(&ts->uses_mt_slots, FTX_MT_PROTOCOL_B);
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Type B device active.\n", dev_name(&(ts->client->dev)), __func__);
		break;
	default:
		DBG_PRINT(dbg_level_warning, "%s: " FTX_TAG ": %s(): DEBUG: Invalid protocol specified.\n", dev_name(&(ts->client->dev)), __func__);
		break;
	}
	return atomic_read(&ts->uses_mt_slots);
}

static int ftx_clear_all_touches(struct ft5x06 *ts)
{
	int iLoop;
	struct input_dev *input_dev = ts->input;
	ft5x06_fingers_t *cur_tch= &ts->cur_fingers;

	switch(atomic_read(&ts->uses_mt_slots))
	{
	case FTX_MT_PROTOCOL_B:
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Clearing all touches for Type B device.\n", dev_name(&(ts->client->dev)), __func__);
			mutex_lock(&ts->lock_fingers);
			for(iLoop = 0; iLoop < atomic_read(&ts->n_fingers_supported); iLoop++)
			{
					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Clearing touch id %d.\n", dev_name(&(ts->client->dev)), __func__, iLoop);
					/* Mark the beginning of the new contact packet */
					input_mt_slot(input_dev, iLoop);
					input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false); /* Report the touch as removed */
			}
			input_sync(ts->input);
			cur_tch->fingers_mask = 0x00; /* Clear all the bits from the current touch mask */
			mutex_unlock(&ts->lock_fingers);
		}
		break;
	case FTX_MT_PROTOCOL_A:
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Clearing all touches for Type A device.\n", dev_name(&(ts->client->dev)), __func__);
			mutex_lock(&ts->lock_fingers);
			/* Mark the end of the contact packet */
			input_mt_sync(ts->input);
			input_sync(ts->input);
			/* Mark the end of the contact packet - Doing it twice just to make sure we dont report any touch */
			input_mt_sync(ts->input);
			input_sync(ts->input);
			cur_tch->fingers_mask = 0x00; /* Clear all the bits from the current touch mask */
			mutex_unlock(&ts->lock_fingers);
		}
		break;
	default:
		{
		}
		break;
	}
	if (ts->platform_data->use_gestures)
	{
		ts->gesture_in_progress = 0;
		ts->gest_count = 0;
		ts->prev_gest = 0;
		//input_report_key(ts->input, BTN_3, FT_NTCH);
	}

	return 0;
}

/*
 * This function will not take care of the lock_fingers mutex handling.
 * That should be taken care by the caller.
 * This functions returns the number of active touches reported.
 */
static int report_typeB_event(struct ft5x06 *ts)
{
	struct input_dev *input_dev = ts->input;
	unsigned long iLoop;
	int finger_num = 0;
	ft5x06_fingers_t *cur_tch = &ts->cur_fingers;
	ft5x06_fingers_t *prev_tch= &ts->prev_fingers;

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting MT touch event using Type B protocol\n", dev_name(&(ts->client->dev)), __func__);
	if(atomic_read(&ts->uses_events) == 1) /* Events are reported. */
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting MT touch event using events\n", dev_name(&(ts->client->dev)), __func__);
		for(iLoop = 0; iLoop < cur_tch->n_fingers; iLoop++)
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting MT touch event for slot %u\n", dev_name(&(ts->client->dev)), __func__, cur_tch->fingers[iLoop].tch_id);
			/* Mark the beginning of the new contact packet */
			input_mt_slot(input_dev, cur_tch->fingers[iLoop].tch_id);
			switch(cur_tch->fingers[iLoop].event)
			{
			case FT_TOUCH_EVENT_NO_EVENT:
				{
					DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch event: NO EVENT\n", dev_name(&(ts->client->dev)), __func__);
				}
				/*
				 * Fall through
				 */
			case FT_TOUCH_EVENT_LIFT_UP:
				{
					DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch event: LIFT UP\n", dev_name(&(ts->client->dev)), __func__);
					cur_tch->fingers_mask &= ~(0x1 << cur_tch->fingers[iLoop].tch_id); /* Clear the bit for the touch */
					input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
				}
				break;
			case FT_TOUCH_EVENT_PRESS_DOWN:
				{
					DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch event: PRESS DOWN\n", dev_name(&(ts->client->dev)), __func__);
				}
				/*
				 * Fall through
				 */
			case FT_TOUCH_EVENT_CONTACT:
				{
					DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch event: CONTACT\n", dev_name(&(ts->client->dev)), __func__);
					cur_tch->fingers_mask |= (0x1 << cur_tch->fingers[iLoop].tch_id); /* Set the bit for the touch */
					input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);

					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting MT PRESSURE and TOUCH_MAJOR events: pressure=%u, touch_major=%u\n",
							dev_name(&(ts->client->dev)), __func__,
							cur_tch->fingers[iLoop].pressure, cur_tch->fingers[iLoop].area);
					input_report_abs(input_dev, ABS_MT_PRESSURE, cur_tch->fingers[iLoop].pressure);
					input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, cur_tch->fingers[iLoop].area);
					//input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, FT_SMALL_TOOL_WIDTH);

					DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch events X and Y: x=%u, y=%u\n", dev_name(&(ts->client->dev)), __func__,
							((cur_tch->fingers[iLoop].x_h << 8) | cur_tch->fingers[iLoop].x_l),
							((cur_tch->fingers[iLoop].y_h << 8) | cur_tch->fingers[iLoop].y_l));
					input_report_abs(input_dev, ABS_MT_POSITION_X, ((cur_tch->fingers[iLoop].x_h << 8) | cur_tch->fingers[iLoop].x_l));
					input_report_abs(input_dev, ABS_MT_POSITION_Y, ((cur_tch->fingers[iLoop].y_h << 8) | cur_tch->fingers[iLoop].y_l));
					finger_num++;
				}
				break;
			default:
				break;
			}
		}
	}
	else /* Points are reported. We need to keep track of the touch history */
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting MT touch event using points\n", dev_name(&(ts->client->dev)), __func__);
		/*
		 * We will first report all the current touches, and clear the bits for these touches from the previous touch mask.
		 * After this all the current touches will be reported, and the remaining touches in the previous touch mask will be the touches that were removed.
		 * So we will report the remaining touches from the previous touch mask as the touches that got removed.
		 */
		for(iLoop = 0; iLoop < cur_tch->n_fingers; iLoop++)
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting MT touch event for slot %u\n", dev_name(&(ts->client->dev)), __func__, cur_tch->fingers[iLoop].tch_id);
			cur_tch->fingers_mask |= (0x1 << cur_tch->fingers[iLoop].tch_id); /* Set the bit in the current touch mask, to track them */
			prev_tch->fingers_mask &= ~(0x1 << cur_tch->fingers[iLoop].tch_id); /* Clear the bit from the previous touch mask */

			/*
			 * Hummingbird focaltech fw does not report number of touches as 0 on penups for single touches,
			 * this hack fixes it.
			 */
			if ( ( cur_tch->fingers[iLoop].area == 0 ) && ( cur_tch->fingers[iLoop].pressure == 0 ) && (cur_tch->n_fingers == 1 ) )
			{
				prev_tch->fingers_mask &= ~(0x1 << iLoop); /* Clear the bit from the previous touch mask */
				cur_tch->fingers_mask &= ~(0x1 << iLoop); /* Clear the bit from the current touch mask */
				DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT remove event for slot %lu\n", dev_name(&(ts->client->dev)), __func__, iLoop);
				/* Mark the beginning of the new contact packet */
				input_mt_slot(input_dev, iLoop);
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false); /* Report the touch as removed */
				continue;
			}

			/* Mark the beginning of the new contact packet */
			input_mt_slot(input_dev, cur_tch->fingers[iLoop].tch_id);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);

			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting MT PRESSURE and TOUCH_MAJOR events: pressure=%u, touch_major=%u\n",
					dev_name(&(ts->client->dev)), __func__,
					cur_tch->fingers[iLoop].pressure, cur_tch->fingers[iLoop].area);
			input_report_abs(input_dev, ABS_MT_PRESSURE, cur_tch->fingers[iLoop].pressure);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, cur_tch->fingers[iLoop].area);
			//input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, FT_SMALL_TOOL_WIDTH);

			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch events X and Y: x=%u, y=%u\n", dev_name(&(ts->client->dev)), __func__,
					((cur_tch->fingers[iLoop].x_h << 8) | cur_tch->fingers[iLoop].x_l),
					((cur_tch->fingers[iLoop].y_h << 8) | cur_tch->fingers[iLoop].y_l));
			input_report_abs(input_dev, ABS_MT_POSITION_X, ((cur_tch->fingers[iLoop].x_h << 8) | cur_tch->fingers[iLoop].x_l));
			input_report_abs(input_dev, ABS_MT_POSITION_Y, ((cur_tch->fingers[iLoop].y_h << 8) | cur_tch->fingers[iLoop].y_l));
			finger_num++;
		}
		for(iLoop = 0; prev_tch->fingers_mask != 0; iLoop++)
		{
			if((prev_tch->fingers_mask & (0x1 << iLoop))) /* Test the bit for previous touch*/
			{
				prev_tch->fingers_mask &= ~(0x1 << iLoop); /* Clear the bit from the previous touch mask */
				cur_tch->fingers_mask &= ~(0x1 << iLoop); /* Clear the bit from the current touch mask */
				DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT remove event for slot %lu\n", dev_name(&(ts->client->dev)), __func__, iLoop);
				/* Mark the beginning of the new contact packet */
				input_mt_slot(input_dev, iLoop);
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false); /* Report the touch as removed */
			}
		}
	}
	return finger_num;
}

/*
 * This function will not take care of the lock_fingers mutex handling.
 * That should be taken care by the caller.
 * This functions returns the number of active touches reported.
 */
static int report_typeA_event(struct ft5x06 *ts)
{
	struct input_dev *input_dev = ts->input;
	unsigned long iLoop;
	int finger_num = 0;
	ft5x06_fingers_t *cur_tch = &ts->cur_fingers;

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting MT touch event using Type A protocol\n", dev_name(&(ts->client->dev)), __func__);
	for(iLoop = 0; iLoop < cur_tch->n_fingers; iLoop++)
	{
		switch(cur_tch->fingers[iLoop].event)
		{
		case FT_TOUCH_EVENT_NO_EVENT:
			{
				DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch event: NO EVENT\n", dev_name(&(ts->client->dev)), __func__);
			}
			/*
			 * Fall through
			 */
		case FT_TOUCH_EVENT_LIFT_UP:
			{
				DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch event: LIFT UP\n", dev_name(&(ts->client->dev)), __func__);
				input_mt_sync(ts->input);
			}
			break;
		case FT_TOUCH_EVENT_PRESS_DOWN:
			{
				DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch event: PRESS DOWN\n", dev_name(&(ts->client->dev)), __func__);
			}
			/*
			 * Fall through
			 */
		case FT_TOUCH_EVENT_CONTACT:
			{
				DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch event: CONTACT\n", dev_name(&(ts->client->dev)), __func__);

				DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting MT PRESSURE and TOUCH_MAJOR events: pressure=%u, touch_major=%u\n",
						dev_name(&(ts->client->dev)), __func__,
						cur_tch->fingers[iLoop].pressure, cur_tch->fingers[iLoop].area);
				int x = ((cur_tch->fingers[iLoop].x_h << 8) | cur_tch->fingers[iLoop].x_l);
				int y = ((cur_tch->fingers[iLoop].y_h << 8) | cur_tch->fingers[iLoop].y_l);
				int prev_x = ((ts->prev_fingers.fingers[iLoop].x_h << 8) | ts->prev_fingers.fingers[iLoop].x_l);
				int prev_y = ((ts->prev_fingers.fingers[iLoop].y_h << 8) | ts->prev_fingers.fingers[iLoop].y_l);
				finger_num++;
				if (x == prev_x && y == prev_y && cur_tch->n_fingers == 1 && ts->prev_fingers.n_fingers == 1) {
					continue;
				}
				input_report_abs(input_dev, ABS_MT_PRESSURE, cur_tch->fingers[iLoop].pressure);
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, cur_tch->fingers[iLoop].area);
				//input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, FT_SMALL_TOOL_WIDTH);

				DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch events X and Y: x=%u, y=%u\n", dev_name(&(ts->client->dev)), __func__,
						((cur_tch->fingers[iLoop].x_h << 8) | cur_tch->fingers[iLoop].x_l),
						((cur_tch->fingers[iLoop].y_h << 8) | cur_tch->fingers[iLoop].y_l));
				input_report_abs(input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
				/* Mark the end of the contact packet */
				input_mt_sync(ts->input);
			}
			break;
		default:
			break;
		}
	}
	return finger_num;
}

static void ft5x06_process_touch(struct work_struct *work)
{
	struct ft5x06 *ts = container_of(work, struct ft5x06, work);
	struct input_dev *input_dev = ts->input;
	unsigned long iLoop;
	int finger_num = 0;
	ft5x06_fingers_t *cur_tch = &ts->cur_fingers;
	u8 gest_id;
	int ret;
	u8 tempbuffer[2] = {0};

	static int pending_gesture_none;

	mutex_lock(&ts->lock_fingers);

	cur_tch->n_fingers = 0;
	cur_tch->fingers_mask = 0x0;
	ret = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_GEST_ID, sizeof(tempbuffer), (u8*)tempbuffer);
	if (ret >= 0)
	{
		cur_tch->n_fingers = tempbuffer[1] & 0xf;
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: %lu fingers detected\n", dev_name(&(ts->client->dev)), __func__, cur_tch->n_fingers);
		if(cur_tch->n_fingers > atomic_read(&ts->n_fingers_supported))
		{
			/* Could be a spurious interrupt; clear all the touches */
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: too many fingers(%lu) detected\n", dev_name(&(ts->client->dev)), __func__, cur_tch->n_fingers);
			cur_tch->n_fingers = 0;
		}

		for(iLoop = 0; iLoop < cur_tch->n_fingers; iLoop++)
		{
			ret = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_P1_XH + (iLoop * 6), sizeof(ft5x06_xydata_t), (u8 *)&(cur_tch->fingers[iLoop])); /* 6 = 4 xy data bytes + 1 pressure byte + 1 area byte */
			if(ret < 0)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read XY(%lu) from the Touch Panel registers.\n", dev_name(&(ts->client->dev)), __func__, iLoop);
				break;
			}
		}

		if(iLoop < cur_tch->n_fingers)
		{
			/*
			 * Error in reading the registers, ignore the interrupt
			 * TODO Clear all the touches (current behaviour) or Ignore the interrupt or report only the read events???
			 */
			cur_tch->n_fingers = 0; /* clear all the touches */
			//cur_tch.n_fingers = iLoop; /* report only the read events */
			//goto err_cleanup;
		}
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: ***************************************************************************.\n", dev_name(&(ts->client->dev)), __func__);
		for(iLoop = 0; iLoop < cur_tch->n_fingers; iLoop++)
		{
			DBG_PRINT_HEX(dbg_level_verbose, FTX_TAG ": ft5x06_process_touch(): VERBOSE: ", DUMP_PREFIX_NONE, 16, 1, &cur_tch->fingers[iLoop], sizeof(ft5x06_xydata_t), false);
		}
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: ***************************************************************************.\n", dev_name(&(ts->client->dev)), __func__);

		/* Determine if display is tilted */
		if (FLIP_DATA(ts->platform_data->flags))
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: tilted display, swapping X & Y\n", dev_name(&(ts->client->dev)), __func__);
			for(iLoop = 0; iLoop < cur_tch->n_fingers; iLoop++)
			{
				SWAP(cur_tch->fingers[iLoop].x_h, cur_tch->fingers[iLoop].y_h);
				SWAP(cur_tch->fingers[iLoop].x_l, cur_tch->fingers[iLoop].y_l);
			}
		}
		/* Check for switch in X origin */
		if (REVERSE_X(ts->platform_data->flags))
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: X origin reversed, shifting X origin\n", dev_name(&(ts->client->dev)), __func__);
			for(iLoop = 0; iLoop < cur_tch->n_fingers; iLoop++)
			{
				cur_tch->fingers[iLoop].x_h = (ts->platform_data->maxx - ((cur_tch->fingers[iLoop].x_h << 8) | cur_tch->fingers[iLoop].x_l)) >> 8;
				cur_tch->fingers[iLoop].x_l = (ts->platform_data->maxx - ((cur_tch->fingers[iLoop].x_h << 8) | cur_tch->fingers[iLoop].x_l)) & 0xff ;
			}
		}
		/* Check for switch in Y origin */
		if (REVERSE_Y(ts->platform_data->flags))
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Y origin reversed, shifting Y origin\n", dev_name(&(ts->client->dev)), __func__);
			for(iLoop = 0; iLoop < cur_tch->n_fingers; iLoop++)
			{
				cur_tch->fingers[iLoop].y_h = (ts->platform_data->maxy - ((cur_tch->fingers[iLoop].y_h << 8) | cur_tch->fingers[iLoop].y_l)) >> 8;
				cur_tch->fingers[iLoop].y_l = (ts->platform_data->maxy - ((cur_tch->fingers[iLoop].y_h << 8) | cur_tch->fingers[iLoop].y_l)) & 0xff ;
			}
		}

		/* Report the events to handlers */
		if(FTX_MT_PROTOCOL_B == atomic_read(&ts->uses_mt_slots)) /* Type B protocol */
		{
			finger_num = report_typeB_event(ts);
		}
		else if(FTX_MT_PROTOCOL_A == atomic_read(&ts->uses_mt_slots)) /* Type A protocol */
		{
			finger_num = report_typeA_event(ts);
		}
		else /* Does not support MT protocol */
		{
		}
		cur_tch->n_active_fingers = finger_num;

		/* Report the ST events if there is atleast one touch */
#if defined(FTX_SUPPORT_ST)
		if(finger_num > 0)
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting ST touch event\n", dev_name(&(ts->client->dev)), __func__);
			/* Just report the last touch point */
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting ST PRESSURE event: pressure=%u\n",
					dev_name(&(ts->client->dev)), __func__,	cur_tch->fingers[cur_tch->n_fingers - 1].pressure);
			input_report_abs(input_dev, ABS_PRESSURE, cur_tch->fingers[cur_tch->n_fingers - 1].pressure);
			input_report_abs(ts->input, ABS_TOOL_WIDTH, FT_SMALL_TOOL_WIDTH);

			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reporting MT touch events X and Y: x=%u, y=%u\n", dev_name(&(ts->client->dev)), __func__,
											((cur_tch->fingers[cur_tch->n_fingers - 1].x_h << 8) | cur_tch->fingers[cur_tch->n_fingers - 1].x_l),
											((cur_tch->fingers[cur_tch->n_fingers - 1].y_h << 8) | cur_tch->fingers[cur_tch->n_fingers - 1].y_l));
			input_report_abs(input_dev, ABS_X, ((cur_tch->fingers[cur_tch->n_fingers - 1].x_h << 8) | cur_tch->fingers[cur_tch->n_fingers - 1].x_l));
			input_report_abs(input_dev, ABS_Y, ((cur_tch->fingers[cur_tch->n_fingers - 1].y_h << 8) | cur_tch->fingers[cur_tch->n_fingers - 1].y_l));
		}
#endif /* defined(FTX_SUPPORT_ST) */

		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: %d fingers reported\n", dev_name(&(ts->client->dev)), __func__, finger_num);

		input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

		/* handle gestures */
		if (ts->platform_data->use_gestures)
		{
			if(cur_tch->n_fingers != 0)
			{
				gest_id = tempbuffer[0];
				#if 0
				ret = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_GEST_ID, sizeof(u8), &gest_id);
				if(ret < 0)
				{
					DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read gesture id from the Touch Panel registers.\n", dev_name(&(ts->client->dev)), __func__);
					gest_id = GESTURE_NONE;
				}
				#endif
				if(ts->prev_gest == gest_id)
				{
					ts->gest_count++;
					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Gesture count incremented for gesture id 0x%02x: %d\n", dev_name(&(ts->client->dev)), __func__, gest_id, ts->gest_count);
				}
				else
				{
					ts->gest_count = 0;
					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: New gesture 0x%02x, gesture count set to 0\n", dev_name(&(ts->client->dev)), __func__, gest_id);
				}

				if(gest_id == GESTURE_NONE)
				{
					pending_gesture_none = 1;
					ts->gesture_in_progress = 0;
					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: no gesture triggered with %d events....ignoring\n", dev_name(&(ts->client->dev)), __func__, cur_tch->n_fingers);
				}
				else
				{
					ts->gesture_in_progress = 1;
					if((finger_num == 0) && (pending_gesture_none != 0)) /* No more touches and pending GESTURE_NONE */
					{
						/* Send the current gesture code and do a sync.
						 * After the sync userspace will not see any more touches.
						 * Then change the gest_id to GESTURE_NONE so that a GESTURE_NONE is sent to userspace.
						 */
						DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting HAT1X event: hat1x=%u\n", dev_name(&(ts->client->dev)), __func__, gest_id);
						input_report_abs(ts->input, ABS_HAT1X, gest_id);
						DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting HAT2X event: hat2x=%lu\n", dev_name(&(ts->client->dev)), __func__, finger_num);
						input_report_abs(ts->input, ABS_HAT2X, finger_num);
						DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting HAT2Y event: hat2y=%u\n", dev_name(&(ts->client->dev)), __func__, ts->gest_count);
						input_report_abs(ts->input, ABS_HAT2Y, ts->gest_count);

						/* signal the view motion event */
						input_sync(ts->input);

						ts->prev_gest = gest_id;
						gest_id = GESTURE_NONE;
						ts->gest_count = 0;
						pending_gesture_none = 0;
						ts->gesture_in_progress = 0;
					}
					// If we don't report a gesture code, the Linux input
					// event protocol means that the previous code is
					// repeated.  Hence we must report a code always; report
					// zero if there's no actual gesture code.
					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting HAT1X event: hat1x=%u\n", dev_name(&(ts->client->dev)), __func__, gest_id);
					input_report_abs(ts->input, ABS_HAT1X, gest_id);
					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting HAT2X event: hat2x=%lu\n", dev_name(&(ts->client->dev)), __func__, finger_num);
					input_report_abs(ts->input, ABS_HAT2X, finger_num);
					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reporting HAT2Y event: hat2y=%u\n", dev_name(&(ts->client->dev)), __func__, ts->gest_count);
					input_report_abs(ts->input, ABS_HAT2Y, ts->gest_count);
				}
			}
			else /* You cannot do a gesture with no events */
			{
				DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: false gesture triggered....ignoring\n", dev_name(&(ts->client->dev)), __func__);
				ts->gesture_in_progress = 0;
				ts->gest_count = 0;
				ts->prev_gest = 0;
			}
		}

		/* Swap the current and previous finger */
		SWAP(ts->cur_fingers.n_fingers, ts->prev_fingers.n_fingers);
		SWAP(ts->cur_fingers.fingers_mask, ts->prev_fingers.fingers_mask);
		iLoop = (unsigned long)ts->cur_fingers.fingers;
		ts->cur_fingers.fingers = ts->prev_fingers.fingers;
		ts->prev_fingers.fingers = (ft5x06_xydata_t *)iLoop;

		/* signal the view motion event */
		input_sync(ts->input);
	}
	else
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read  n_touches from the Touch Panel registers.\n", dev_name(&(ts->client->dev)), __func__);
	}

//err_cleanup:
	mutex_unlock(&ts->lock_fingers);

	return;
}

/*************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function
 ************************************************************************/
static irqreturn_t ft5x06_irq(int irq, void *handle)
{
	struct ft5x06 *ts = (struct ft5x06 *) handle;
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: got irq!\n", dev_name(&(ts->client->dev)), __func__);

	if(atomic_cmpxchg(&ts->power_on_ftx, 1, 0)) /* Check if powered on just now. If yes ignore the first interrupt and clear the power_on_ftx flag */
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: first irq after power on\n", dev_name(&(ts->client->dev)), __func__);
	}
	else
	{
		/* schedule motion signal handling */
		if(0 != queue_work(ts->ft5x06_ts_wq, &(ts->work)))
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: too many interrupts\n", dev_name(&(ts->client->dev)), __func__);
		}
	}
	return IRQ_HANDLED;
}


/*************************************************************************
 * GPIO Helper Functions
 ************************************************************************/

static void ft5x06_reset_panel_via_gpio(int reset_gpio)
{
    DBG_PRINT(dbg_level_debug, FTX_TAG ": %s(): DEBUG: Toggling gpio %d to reset touch panel...\n",  __func__, reset_gpio);

    gpio_set_value(reset_gpio, 1);
    msleep(20);
    gpio_set_value(reset_gpio, 0);
    msleep(20);
    gpio_set_value(reset_gpio, 1);
    msleep(300);
}


static bool ft5x06_poll_gpio(const int gpio_num, const int requested_gpio_val)
{
    const int poll_count_limit = 20;
    const int poll_delay_ms    = 5;
    int poll_count = 0;
    int gpio_val   = -1;

    DBG_PRINT(dbg_level_debug, FTX_TAG ": %s(): DEBUG: Waiting for gpio %d to go to %d...\n", __func__, gpio_num, requested_gpio_val);

    while ((poll_count_limit > poll_count++) && (requested_gpio_val != gpio_val))
    {
        msleep(poll_delay_ms);
        gpio_val = gpio_get_value(gpio_num);
    }

    return (requested_gpio_val == gpio_val);
}

/* Set the TP controller to Firmware update mode return 0 for success
 * caller needs to reset TP for exit update mode */
static int ft5x06_enter_fwupdate_mode(struct ft5x06 *ts)
{
	int  retval = 0;
	int  i      = 0;
	u8  temp_buffer[4]; /* Do we need 4 bytes */

        struct i2c_msg msg =
	{
		.addr   = ts->client->addr,
		.flags  = 0,
		.len    = 2,
		.buf    = temp_buffer,
	};

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Reset CTPM...\n", dev_name(&(ts->client->dev)), __func__);
	retval = i2c_smbus_write_byte_data(ts->client, FT5x06_WMREG_RESET, FT5x06_CMD_RESET_CTPM_H);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write the CTPM reset command (high byte)...\n", dev_name(&(ts->client->dev)), __func__);
		goto error_return;
	}
	msleep(ts->ftx_upgrade_info.upgrade_h_delay);

	retval = i2c_smbus_write_byte_data(ts->client, FT5x06_WMREG_RESET, FT5x06_CMD_RESET_CTPM_L);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write the CTPM reset command (low byte)...\n", dev_name(&(ts->client->dev)), __func__);
		goto error_return;
	}
	msleep(ts->ftx_upgrade_info.upgrade_l_delay);

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Put the CTPM in FW upgrade mode...\n", dev_name(&(ts->client->dev)), __func__);

	temp_buffer[0] = 0x55;
	temp_buffer[1] = 0xAA;
	i = 0;
	do
	{
		retval = i2c_transfer(ts->client->adapter, &msg, 1);
		msleep(10);
		if(0 < retval)
		{
			break;
		}
		i++;
	}while(i < 5);

	if ((0 >= retval))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the CTPM in FW update mode.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_return;
	}
	msleep(ts->ftx_upgrade_info.upgrade_enter_upgrade_mode_delay);
	return 0;

error_return:
	return -1;
}

/*************************************************************************
 * Factory Mode Helper Functions
 ************************************************************************/

static int ft5x06_enter_factory_mode(struct ft5x06 * ts)
{
	int retval = 0;
	u8  regval = 0;

	if (ts->client->irq)
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Disabling touch interrupt.\n", dev_name(&(ts->client->dev)), __func__);
		disable_irq(ts->client->irq);
	}

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Putting touch panel in factory mode.\n", dev_name(&(ts->client->dev)), __func__);
	regval = FT5x06_MODE_FACTORY;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_enable_irq;
	}

	msleep(100);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that touch panel is in factory mode.\n", dev_name(&(ts->client->dev)), __func__);
	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_enable_irq;
	}

	if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_FACTORY)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Touch panel is not in factory mode; DEVICE MODE register contains 0x%02x\n", dev_name(&(ts->client->dev)), __func__, regval);
		retval = FT5x06_ERR_NOT_FACTORY_MODE;
		goto error_enable_irq;
	}

	return 0;

error_enable_irq:
	if (ts->client->irq)
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Enabling touch interrupt.\n", dev_name(&(ts->client->dev)), __func__);
		enable_irq(ts->client->irq);
	}

	return retval;
}


static int ft5x06_exit_factory_mode(struct ft5x06 * ts)
{
	int retval = 0;
	u8  regval = 0;

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Putting touch panel in working mode.\n", dev_name(&(ts->client->dev)), __func__);

	regval = FT5x06_MODE_WORKING;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_enable_irq;
	}
	msleep(100);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that touch panel is in working mode.\n", dev_name(&(ts->client->dev)), __func__);
	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_enable_irq;
	}

	if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_WORKING)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Touch panel is not in working mode; DEVICE MODE register contains 0x%02x\n", dev_name(&(ts->client->dev)), __func__, regval);
		retval = FT5x06_ERR_NOT_WORKING_MODE;
		goto error_enable_irq;
	}

	retval = 0;

error_enable_irq:
	if (ts->client->irq)
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Enabling touch interrupt.\n", dev_name(&(ts->client->dev)), __func__);
		enable_irq(ts->client->irq);
	}

	return retval;
}

static int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

/* read rawdata and convert to string */
static int ft5x06_read_data(struct i2c_client * client, char * output_buffer, ssize_t output_buffer_size, ssize_t * p_num_read_chars)
{
	int i, j;
	int retval  = 0;
	int rawdata[FT5x06_NUM_TX][FT5x06_NUM_RX];
	retval = ft5x06_read_rawdata(client, rawdata);

	if (0 != retval)
	{
		dev_err(&client->dev, "%s() - ERROR: Could not read Raw Data from the Touch Panel.\n", __FUNCTION__);
		goto error_return;
	}

	for (i = 0; i < FT5x06_NUM_TX; i++) {
		for (j = 0; ((j < FT5x06_NUM_RX) && ((*p_num_read_chars + 11) < output_buffer_size)); j ++) {
			*p_num_read_chars += sprintf(&(output_buffer[*p_num_read_chars]), "%d ", rawdata[i][j] - 8000);
		}
		output_buffer[*p_num_read_chars-1] = '\n';
		if((*p_num_read_chars + 11) >= output_buffer_size)
		{
			break;
		}
	}

	retval = 0;
error_return:
	return retval;
}

/*************************************************************************
 * Firmware Update
 ************************************************************************/

static int ft5x06_perform_fw_upgrade(struct device *dev, const u8 * firmware_src_buffer, u32 firmware_size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	int  retval = 0;
	int  i      = 0;
	int  j      = 0;
	u32 num_packets = 0;
	u16 temp_val    = 0;
	u8  checksum    = 0;
	u8  write_buffer[4];
	u8  read_buffer[2];
	u8  packet_buffer[FT5x06_PACKET_LENGTH + 6];
	int suspend_enabled;

	/* Disable suspend during FW upgrade */
	suspend_enabled = atomic_cmpxchg(&ts->do_not_suspend_ftx, 0, 1); /* After this do_not_suspend_ftx will be 1, and old value will be in suspend_enabled */

	/* Disable FW upgrade if we are in suspend */
	mutex_lock(&ts->lock_ftx_suspend_state);
	if(ts->ftx_suspend_state == FTX_SUSPEND)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Controller is in suspend...Make sure the controller and display is on during FW upgrade.\n", dev_name(dev), __func__);
		retval = -EAGAIN;
		goto error_return;
	}
	mutex_unlock(&ts->lock_ftx_suspend_state);

	/* Disable FW upgrade if controller is powered OFF */
	mutex_lock(&ts->lock_ftx_power_supply_state);
	if(ts->ftx_power_supply_state == FTX_POWER_SUPPLY_OFF)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Controller is turned OFF...Make sure the controller and display is on during FW upgrade.\n", dev_name(dev), __func__);
		retval = -EAGAIN;
		goto error_return;
	}
	mutex_unlock(&ts->lock_ftx_power_supply_state);

	if (NULL == firmware_src_buffer)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Firmware source buffer pointer is NULL.\n", dev_name(dev), __func__);
		retval = -EINVAL;
		goto error_return;
	}

	if (0 == firmware_size)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Firmware source size is 0.\n", dev_name(dev), __func__);
		retval = -EINVAL;
		goto error_return;
	}

	for (i = 0; i < 3; i++) {
		retval = ft5x06_enter_fwupdate_mode(ts);
		if (0 != retval)
		{
			goto error_reset;
		}

		DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Read the CTPM ID.\n", dev_name(dev), __func__);
		/* Send the GET ID command to CTPM */
		write_buffer[0] = FT5x06_CMD_GET_ID;
		write_buffer[1] = FT5x06_CMD_GET_ID_P1;
		write_buffer[2] = FT5x06_CMD_GET_ID_P2;
		write_buffer[3] = FT5x06_CMD_GET_ID_P3;
		retval = i2c_master_send(ts->client, write_buffer, 4);
		if (0 > retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write the GET ID command to the CTPM.\n", dev_name(dev), __func__);
			goto error_reset;
		}

		/* Read the ID from CTPM */
		retval = i2c_master_recv(ts->client,read_buffer, 2);
		if (0 > retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read the ID from the CTPM.\n", dev_name(dev), __func__);
			goto error_reset;
		}

		if ((FT5x06_CPTM_ID_COMPANY != read_buffer[0]) || (FT5606_CPTM_ID_PRODUCT != read_buffer[1]))
		{
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Invalid CPTM ID. Expected 0x%02X%02X, got 0x%02X%02X.\n", dev_name(dev), __func__,
					ts->info.focaltech_id, ts->info.product_id, read_buffer[0], read_buffer[1]);
		}
		else
		{
			break;
		}
	}

	// if (i >= 3)
	// {
	// 	DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid CPTM ID. Expected 0x%02X%02X, got 0x%02X%02X.\n", dev_name(dev), __func__,
	// 		ts->info.focaltech_id, ts->info.product_id, read_buffer[0], read_buffer[1]);
	// 	retval = FT5x06_ERR_INVALID_ID;
	// 	goto error_reset;
	// }

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Erase old CTPM FW.\n", dev_name(dev), __func__);
	write_buffer[0] = FT5x06_CMD_ERASE_FW;
	retval = i2c_master_send(ts->client, write_buffer, 1);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write ERASE FW command to the CTPM.\n", dev_name(dev), __func__);
		goto error_reset;
	}
	msleep(ts->ftx_upgrade_info.upgrade_erase_delay);

	/*********************************************************************************************/
	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Write new FW to CTPM flash.\n", dev_name(dev), __func__);
	/* We write everything but the last 8 bytes in packets.
	 * The first 6 of the last 8 bytes will be written in the footer.
	 * The final 2 bytes (which seem to be always 0xFF and 0x00) don't get written.
	 */
	firmware_size = firmware_size - 8;
	num_packets   = (firmware_size) / FT5x06_PACKET_LENGTH;
	packet_buffer[0] = 0xBF;
	packet_buffer[1] = 0x00;
	/* Write whole packets */
	for (i = 0; i < num_packets; i++)
	{
		/* Target offset */
		temp_val = i * FT5x06_PACKET_LENGTH;
		packet_buffer[2] = (u8)(0x00FF & (temp_val >> 8));
		packet_buffer[3] = (u8)(0x00FF & (temp_val));

		/* Num bytes following header */
		temp_val = FT5x06_PACKET_LENGTH;
		packet_buffer[4] = (u8)(0x00FF & (temp_val >> 8));
		packet_buffer[5] = (u8)(0x00FF & (temp_val));

		for (j = 0; j < FT5x06_PACKET_LENGTH; j++)
		{
			/* Process byte j of packet i... */
			packet_buffer[6 + j] = firmware_src_buffer[(i * FT5x06_PACKET_LENGTH) + j];
			checksum ^= packet_buffer[6 + j];
		}

		retval = i2c_master_send(ts->client, packet_buffer, FT5x06_PACKET_LENGTH + 6);
		if (0 > retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write packet %u of %u to the CTPM.\n", dev_name(dev), __func__, i, num_packets);
			goto error_reset;
		}
		//msleep(FT5x06_PACKET_LENGTH/6 + 1);
		msleep(ts->ftx_upgrade_info.upgrade_packet_delay);

		if (0 == ((i * FT5x06_PACKET_LENGTH) % 1024))
		{
			DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Uploaded %6d of %6u bytes to the CTPM.\n", dev_name(dev), __func__, (i * FT5x06_PACKET_LENGTH), firmware_size);
		}
	}
	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Uploaded %6d of %6u bytes to the CTPM.\n", dev_name(dev), __func__, (i * FT5x06_PACKET_LENGTH), firmware_size);

	/* Write a partial packet if necessary */
	if (0 != (firmware_size % FT5x06_PACKET_LENGTH))
	{
		/* Target offset */
		temp_val = num_packets * FT5x06_PACKET_LENGTH;

		packet_buffer[2] = (u8)(0x00FF & (temp_val >> 8));
		packet_buffer[3] = (u8)(0x00FF & (temp_val));

		/* Num bytes following header */
		temp_val = (firmware_size % FT5x06_PACKET_LENGTH);

		packet_buffer[4] = (u8)(0x00FF & (temp_val >> 8));
		packet_buffer[5] = (u8)(0x00FF & (temp_val));

		for (j = 0; j < temp_val; j++)
		{
			packet_buffer[6 + j] = firmware_src_buffer[(num_packets * FT5x06_PACKET_LENGTH) + j];
			checksum ^= packet_buffer[6 + j];
		}

		retval = i2c_master_send(ts->client, packet_buffer, temp_val + 6);
		if (0 > retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write partial packet to the CTPM.\n", dev_name(dev), __func__);
			goto error_reset;
		}
		//msleep(20);
		msleep(ts->ftx_upgrade_info.upgrade_packet_delay);
	}
	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Uploaded %6d of %6u bytes to the CTPM.\n", dev_name(dev), __func__, (i * FT5x06_PACKET_LENGTH)+temp_val, firmware_size);

	/* Write the firmware footer */
	for (i = 0; i < 6; i++)
	{
		packet_buffer[2] = 0x6F;
		packet_buffer[3] = 0xFA + i;
		packet_buffer[4] = 0x00;
		packet_buffer[5] = 0x01;

		packet_buffer[6] = firmware_src_buffer[firmware_size + i];
		checksum ^= packet_buffer[6];

		retval = i2c_master_send(ts->client, packet_buffer, 7);
		if (0 > retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write FW footer to the CTPM.\n", dev_name(dev), __func__);
			goto error_reset;
		}
		//msleep(20);
		msleep(ts->ftx_upgrade_info.upgrade_packet_delay);
	}

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Checksum verification.\n", dev_name(dev), __func__);
	write_buffer[0] = FT5x06_CMD_GET_CHECKSUM;
	retval = i2c_master_send(ts->client, write_buffer, 1);
	if (0 > retval)
	{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write GET CHECKSUM command to the CTPM.\n", dev_name(dev), __func__);
			goto error_reset;
	}
	//msleep(10);
	msleep(ts->ftx_upgrade_info.upgrade_checksum_delay);

	retval = i2c_master_recv(ts->client, read_buffer, 1);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read the checksum from the CTPM.\n", dev_name(dev), __func__);
		goto error_reset;
	}

	if (checksum != read_buffer[0])
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Checksum(0x%02x) did not match calculated value(0x%02x).\n", dev_name(dev), __func__, read_buffer[0], checksum);
		retval = FT5x06_ERR_INVALID_CHECKSUM;
		goto error_reset;
	}

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Reset the CTPM FW.\n", dev_name(dev), __func__);
	write_buffer[0] = FT5x06_CMD_RESET_FW;
	retval = i2c_master_send(ts->client, write_buffer, 1);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write the RESET FW command to the CTPM.\n", dev_name(dev), __func__);
		goto error_reset;
	}
	msleep(ts->ftx_upgrade_info.upgrade_reset_fw_delay);
	retval = 0;

error_reset:
	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Reset the CTPM.\n", dev_name(dev), __func__);
	ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);
	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: FW update complete; update %s.\n", dev_name(dev), __func__, ((0 == retval) ? "Succeeded" : "Failed"));
error_return:
	/* Restore the do_not_suspend_ftx */
	atomic_cmpxchg(&ts->do_not_suspend_ftx, !suspend_enabled, suspend_enabled); /* After this do_not_suspend_ftx will be suspend_enabled */

	return retval;
}


/*************************************************************************
 * SYSFS Store and Show functions
 ************************************************************************/

static ssize_t ft5x06_driver_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%04X\n", FT5x06_DRIVER_VERSION);
}


static ssize_t ft5x06_rawbase_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	int     retval = 0;

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	retval = ft5x06_read_data(ts->client, buf, PAGE_SIZE, &num_read_chars);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read raw data from the touch panel.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
		goto error_return;
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_crosstalk_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	const u8 rx_offset = ((FT5x06_CROSSTALK_TEST_TYPE_ODD == ts->crosstalk_test_type) ? 1 : 0);
	ssize_t  num_read_chars = 0;
	u8  num_processed_rx_cac_values = 0x00;
	u8  i        = 0x00;
	u8  regval   = 0x00;
	int retval   = 0;
	u8  original_rx_cac[FT5x06_NUM_RX];
	u8 testmode = 0x00;
	u8 m = 1;

	mutex_lock(&ts->device_mode_mutex);
	memset(original_rx_cac, 0x00, FT5x06_NUM_RX);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	/* Preserve the original values of the even or odd RXn_CAC registers and then set them to 0x00. */
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Modifying the RXn_CAC register values....\n", dev_name(dev), __func__);
	m = 1;
	for (i = rx_offset; i < FT5x06_NUM_RX; i += 2)
	{
		if (i >= FT5606_TEST_MODE_RX) {
			if (i >= FT5606_TEST_MODE_RX*2)
				m = 3;
			else if (i >= FT5606_TEST_MODE_RX*3)
				m = 4;
			else
				m = 2;
			testmode = ft5606_factory_mode[m-1] << 4;
			retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (0 != retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
				goto error_restore_cac_values;
			}
		}

		retval = i2c_smbus_read_i2c_block_data(ts->client, (FT5x06_FMREG_RX_0_CAC + i - FT5606_TEST_MODE_RX*(m-1)), sizeof(u8), &regval);
		if (0 > retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from the RX%u_CAC register.\n", dev_name(dev), __func__, i);
			goto error_restore_cac_values;
		}

		original_rx_cac[num_processed_rx_cac_values] = regval;
		num_processed_rx_cac_values++;

		regval = 0x00;
		retval = i2c_smbus_write_i2c_block_data(ts->client, (FT5x06_FMREG_RX_0_CAC + i - FT5606_TEST_MODE_RX*(m-1)), sizeof(u8), &regval);
		if (0 != retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x00 to the RX%u_CAC register.\n", dev_name(dev), __func__, i);
			goto error_restore_cac_values;
		}

		if (i >= FT5606_TEST_MODE_RX) {
			testmode = FACTORY_MODE0 <<4;
			retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (retval != 0) {
				dev_err(dev, "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n", __FUNCTION__, i);
				goto error_restore_cac_values;
			}
		}
	}
	msleep(100);

	retval = ft5x06_read_data(ts->client, buf, PAGE_SIZE, &num_read_chars);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read raw data from the touch panel.\n", dev_name(dev), __func__);
		goto error_restore_cac_values;
	}

error_restore_cac_values:
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Restoring the RXn_CAC register values....\n", dev_name(dev), __func__);
	m = 1;
	for (i = 0; i < num_processed_rx_cac_values; i++)
	{
		if (((2 * i) + rx_offset) >= FT5606_TEST_MODE_RX) {
			if (i >= FT5606_TEST_MODE_RX*2)
				m = 3;
			else if (i >= FT5606_TEST_MODE_RX*3)
				m = 4;
			else
				m = 2;
			testmode = ft5606_factory_mode[m-1] << 4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (0 != retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
				goto error_restore_mode;
			}
		}

		retval = i2c_smbus_write_i2c_block_data(client, (FT5x06_FMREG_RX_0_CAC + (2 * i) + rx_offset - (FT5606_TEST_MODE_RX*(m-1))), sizeof(u8), &(original_rx_cac[i]) );
		if (0 != retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not restore the oroginal value of the RX%u_CAC register.\n", dev_name(dev), __func__, (2 * i) + rx_offset);
			goto error_restore_mode;
		}

		testmode = FACTORY_MODE0 <<4;
		retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
		if (retval != 0) {
			dev_err(dev, "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n", __FUNCTION__, i);
		}
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}


static ssize_t ft5x06_crosstalk_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	unsigned long value = 0;
	int retval = 0;

	mutex_lock(&ts->device_mode_mutex);

	retval = strict_strtoul(buf, 10, &value);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Could not convert the given input(\"%s\") to a number.\n", dev_name(dev), __func__, buf);
		goto error_return;
	}

	switch (value)
	{
	case 0:
		{
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Crosstalk Test Type is now EVEN.\n", dev_name(dev), __func__);
			ts->crosstalk_test_type = FT5x06_CROSSTALK_TEST_TYPE_EVEN;
		}
		break;
	case 1:
		{
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Crosstalk Test Type is now ODD.\n", dev_name(dev), __func__);
			ts->crosstalk_test_type = FT5x06_CROSSTALK_TEST_TYPE_ODD;
		}
		break;
	default:
		{
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Invalid input specified: %lu.\n", dev_name(dev), __func__, value);
			goto error_return;
		}
		break;
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
return count;
}


static ssize_t ft5x06_icsupplier_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	int     retval = 0;
	u8      regval = 0x00;


	mutex_lock(&ts->device_mode_mutex);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reading IC supplier value.\n", dev_name(dev), __func__);
	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_FOCALTECH_ID, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read IC supplier value.\n", dev_name(dev), __func__);
		goto error_return;
	}
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: IC supplier value: 0x%02x\n", dev_name(dev), __func__, regval);

	num_read_chars += snprintf(buf, PAGE_SIZE, "0x%02x\n", regval);

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}


static ssize_t ft5x06_icpartno_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	int     retval = 0;
	u8      regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reading IC part no: value.\n", dev_name(dev), __func__);

	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_IC_PARTNO, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read IC part no: value.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: IC part no: value: 0x%02x\n", dev_name(dev), __func__, regval);

	num_read_chars += snprintf(buf, PAGE_SIZE, "0x%02X\n", regval);

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);

	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_storecalibrateflash_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	int retval = 0;
	u8  regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	msleep(500);
	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Configuring the CALIBRATE register.\n", dev_name(dev), __func__);
	regval = FT5x06_CALIBRATE_SAVE_TO_FLASH;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_CALIBRATE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to the CALIBRATE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	msleep(2000);
	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from the DEVICE MODE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	if (FT5x06_MODE_FACTORY != regval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: The DEVICE MODE register contained 0x%02x (expected 0x%02x).\n", dev_name(dev), __func__, regval, FT5x06_MODE_FACTORY);
		goto error_restore_mode;
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
	}

	mutex_unlock(&ts->device_mode_mutex);
return 0;
}

static ssize_t ft5x06_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	int     fwver = 0;

	mutex_lock(&ts->device_mode_mutex);
	fwver = i2c_smbus_read_byte_data(ts->client, FT5x06_WMREG_FW_VER);
	if (0 > fwver)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read the FW version number.\n", dev_name(dev), __func__);
		goto error_return;
	}

	num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", (fwver & 0x000000FF));

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

/* Read the vendorID from firmware protected area, need to enter into firmware mode */
static ssize_t ft5x06_vendorid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	u8 vendorid = 0;
	int retval   = 0;
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
	if (0 >= retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read the UPGRADE VERSION from the CTPM.\n", dev_name(dev), __func__);
		goto error_reset;
	}

	num_read_chars = snprintf(buf, PAGE_SIZE, "0x%02X\n", vendorid);
error_reset:
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reset the touch panel.\n", dev_name(dev), __func__);
	ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static int ft5606_read_rx_offset(struct i2c_client *client, u8 offset[], int rx)
{
	int retval = 0;
	u8 j = 0, m = 1;
	int err = 0;
	u8 testmode = 0x00;
	u8 regaddr = 0x00;
//	u8 writebuf[2] = {0};
	u8 read_buffer[FT5x06_CALIBRATION_NUM_BYTES];

	/*get rx offset*/
	/* Read the data*/
	//for (j = 0; j < (rx/FT5606_TEST_MODE_RX + 1); j++) {
	for (j = 0; j < rx; j++) {
		if (j >= FT5606_TEST_MODE_RX) {
			if (j >= FT5606_TEST_MODE_RX*2)
				m = 3;
			else if (j >= FT5606_TEST_MODE_RX*3)
				m = 4;
			else
				m = 2;
			testmode = ft5606_factory_mode[m-1] << 4;
			err = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (0 != retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
				return err;
			}
		}

		regaddr = FT5x06_FMREG_RX_0_1_OFFSET + j - (FT5606_TEST_MODE_RX*(m-1));

		err = i2c_smbus_read_i2c_block_data(client, regaddr, sizeof(u8), read_buffer + j);
		if (0 > retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from the DEVICE MODE register.\n", dev_name(&(client->dev)), __func__);
			return err;
		}

		if (j >= FT5606_TEST_MODE_RX) {
			testmode = FACTORY_MODE0 <<4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (retval != 0) {
				dev_err(&client->dev, "%s() - return test mode 0 failed.\n",
						__func__);
				return err;
			}
		}
	}
	for (j = 0; j < rx; j++)
		offset[j] = read_buffer[j];
	/*return test mode 0*/
	testmode = FACTORY_MODE0 <<4;
	retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
	if (retval != 0) {
		dev_err(&client->dev, "%s() - return test mode 0 failed.\n",
				__func__);
		return err;
	}

	return 0;
}

static ssize_t ft5x06_rxoffset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	int     retval  = 0;
	int i = 0;
	u8  calibration_data[FT5x06_CALIBRATION_NUM_BYTES];

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_return;
	}

	retval = ft5606_read_rx_offset(client, calibration_data, FT5x06_CALIBRATION_NUM_BYTES);

	for (i = 0; ((i < FT5x06_CALIBRATION_NUM_BYTES) && ((num_read_chars + 6) < PAGE_SIZE)); i++)
	{
		num_read_chars += sprintf(&(buf[num_read_chars]), "%u ", calibration_data[i]);
	}

	buf[num_read_chars-1] = '\n';

error_return:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

/*Calibreate in working mode for 0x15 firmware needed */
static ssize_t ft5x06_calibrate_show2(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	int     i       = 0;
	int     retval  = 0;
	u8      regval  = 0x00;
	u8      devmode = 0x00;
	u8  calibration_data[FT5x06_CALIBRATION_NUM_BYTES];

	mutex_lock(&ts->device_mode_mutex);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Putting touch panel in working mode.\n", dev_name(&(ts->client->dev)), __func__);
	msleep(200);
	regval = FT5x06_MODE_WORKING;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_restore_mode;
	}
	msleep(100);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that touch panel is in working mode.\n", dev_name(&(ts->client->dev)), __func__);

	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
		goto error_restore_mode;
	}

	if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_WORKING)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Touch panel is not in working mode; DEVICE MODE register contains 0x%02x\n", dev_name(&(ts->client->dev)), __func__, regval);
		retval = FT5x06_ERR_NOT_WORKING_MODE;
		goto error_restore_mode;
	}

	retval = 0;

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Configuring the CALIBRATE register.\n", dev_name(dev), __func__);
	regval = FT5x06_CALIBRATE_START;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_STAT_CFG, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to the CALIBRATE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	msleep(2000);

	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_STAT_CFG, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from the DEVICE MODE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	if (0x1 != regval) {
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): stat cfg error %d.\n", dev_name(dev), __func__, regval);
		goto error_restore_mode;
	}

	/* Go back to Factory Mode, but don't call ft5x06_enter_factory_mode()
	 * since that function will disable the IRQ a second time without
	 * re-enabling the IRQ first.
	 */
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Putting the touch panel in factory mode.\n", dev_name(dev), __func__);
	msleep(300);
	regval = FT5x06_MODE_FACTORY;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to the DEVICE MODE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	msleep(100);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that the touch panel is in factory mode.\n", dev_name(dev), __func__);
	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from the DEVICE MODE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_FACTORY)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: The DEVICE MODE register contains 0x%02x (expected 0x%02x).\n", dev_name(dev), __func__, regval, FT5x06_MODE_FACTORY);
		retval = FT5x06_ERR_NOT_FACTORY_MODE;
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reading calibration data...\n", dev_name(dev), __func__);
	retval = ft5606_read_rx_offset(client, calibration_data, FT5x06_CALIBRATION_NUM_BYTES);

	msleep(100);

	/* Each calibration byte holds two offsets that can each be up to 2 chars long. Add two spaces and we need 6 chars to store it. */
	for (i = 0; ((i < FT5x06_CALIBRATION_NUM_BYTES) && ((num_read_chars + 6) < PAGE_SIZE)); i++)
	{
		num_read_chars += sprintf(&(buf[num_read_chars]), "%u ", calibration_data[i]);
	}

	buf[num_read_chars-1] = '\n';

error_restore_mode:
	/* go to working mode */
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Putting touch panel in working mode.\n", dev_name(&(ts->client->dev)), __func__);

	regval = FT5x06_MODE_WORKING;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
	}
	msleep(100);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that touch panel is in working mode.\n", dev_name(&(ts->client->dev)), __func__);

	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from DEVICE MODE register.\n", dev_name(&(ts->client->dev)), __func__);
	}

	if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_WORKING)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Touch panel is not in working mode; DEVICE MODE register contains 0x%02x\n", dev_name(&(ts->client->dev)), __func__, regval);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}
static ssize_t ft5x06_calibrate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	int     i       = 0;
	int     retval  = 0;
	u8      regval  = 0x00;
	u8      devmode = 0x00;
	u8  calibration_data[FT5x06_CALIBRATION_NUM_BYTES];

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Configuring the CALIBRATE register.\n", dev_name(dev), __func__);

	regval = FT5x06_CALIBRATE_START;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_CALIBRATE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to the CALIBRATE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	msleep(1000);

	/* Wait for finish and back to working mode */
	for (i = 0; i < 100; i++) {
		msleep(10);
		/* Configuring the Calibration register should put us back in Working Mode. */
		retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE, sizeof(u8), &devmode);
		if (0 > retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from the DEVICE MODE register.\n", dev_name(dev), __func__);
			goto error_restore_mode;
		}
		printk("i=%d devmode=0x%x\n", i, devmode);
		if (FT5x06_MODE_WORKING == devmode)
		{
			break;
		}
	}

	/* Go back to Factory Mode, but don't call ft5x06_enter_factory_mode()
	 * since that function will disable the IRQ a second time without
	 * re-enabling the IRQ first.
	 */
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Putting the touch panel in factory mode.\n", dev_name(dev), __func__);

	regval = FT5x06_MODE_FACTORY;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to the DEVICE MODE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	msleep(100);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that the touch panel is in factory mode.\n", dev_name(dev), __func__);
	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from the DEVICE MODE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}


	if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_FACTORY)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: The DEVICE MODE register contains 0x%02x (expected 0x%02x).\n", dev_name(dev), __func__, regval, FT5x06_MODE_FACTORY);
		retval = FT5x06_ERR_NOT_FACTORY_MODE;
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reading calibration data...\n", dev_name(dev), __func__);
	retval = ft5606_read_rx_offset(client, calibration_data, FT5x06_CALIBRATION_NUM_BYTES);

	msleep(100);

	/* Each calibration byte holds two offsets that can each be up to 2 chars long. Add two spaces and we need 6 chars to store it. */
	for (i = 0; ((i < FT5x06_CALIBRATION_NUM_BYTES) && ((num_read_chars + 6) < PAGE_SIZE)); i++)
	{
		num_read_chars += sprintf(&(buf[num_read_chars]), "%u ", calibration_data[i]);
	}

	buf[num_read_chars-1] = '\n';

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}


static ssize_t ft5x06_voltage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	int     retval  = 0;
	u8      voltage = 0;

	mutex_lock(&ts->device_mode_mutex);
	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reading device voltage...\n", dev_name(dev), __func__);

	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DRIVER_VOLTAGE, sizeof(u8), &voltage);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read the value of the DEVICE VOLTAGE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	num_read_chars += snprintf(buf, PAGE_SIZE, "%u\n", (FT5x06_VOLTAGE_MASK & voltage));

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_voltage_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);

	unsigned long voltage = 0;

	int retval = 0;
	u8  regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	retval = strict_strtoul(buf, 16, &voltage);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid input specified (\"%s\").\n", dev_name(dev), __func__, buf);
		goto error_return;
	}

	if (0x07 < voltage)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid input specified (%lu).\n", dev_name(dev), __func__, voltage);
		goto error_return;
	}

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	regval = (u8)(voltage & 0x000000FF);

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Writing 0x%02x to the DEVICE VOLTAGE register.\n", dev_name(dev), __func__, regval);
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_DRIVER_VOLTAGE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the DEVICE VOLTAGE register.\n", dev_name(dev), __func__, regval);
		goto error_restore_mode;
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}

static ssize_t ft5x06_interrupttest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	const int gpio_num = irq_to_gpio(ts->client->irq);
	int test_result = 0;
	int retval   = 0;
	u8  regval   = 0x00;

	mutex_lock(&ts->device_mode_mutex);
	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that the interrupt gpio is HIGH.\n", dev_name(dev), __func__);

	if (!ft5x06_poll_gpio(gpio_num, 1))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: The interrupt gpio is LOW when it should be HIGH.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}
	else
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: The interrupt gpio is HIGH.\n", dev_name(dev), __func__);
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Toggling the interrupt gpio line.\n", dev_name(dev), __func__);

	/* Note that the interrupt line can be toggled by writing any value to the INTERRRUPT_TOGGLE register. */
	regval = 0x00;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_INTERRUPT_TOGGLE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to the INTERRUPT TOGGLE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that the interrupt gpio is LOW.\n", dev_name(dev), __func__);

	if (!ft5x06_poll_gpio(gpio_num, 0))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: The interrupt gpio is HIGH when it should be LOW.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}
	else
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: The interrupt gpio is LOW.\n", dev_name(dev), __func__);
	}

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Toggling the interrupt gpio line.\n", dev_name(dev), __func__);

	regval = 0x00;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_INTERRUPT_TOGGLE, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write to the INTERRUPT TOGGLE register.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Verifying that the interrupt gpio is HIGH.\n", dev_name(dev), __func__);

	if (!ft5x06_poll_gpio(gpio_num, 1))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: The interrupt gpio is LOW when it should be HIGH.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}
	else
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: The interrupt gpio is HIGH.\n", dev_name(dev), __func__);
	}

	test_result = 1;

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
	}
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Test %s\n", dev_name(dev), __func__, (test_result ? "Passed" : "Failed"));
	mutex_unlock(&ts->device_mode_mutex);
	return snprintf(buf, PAGE_SIZE, "%d\n", test_result);
}


static ssize_t ft5x06_tpreset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);

	mutex_lock(&ts->device_mode_mutex);
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Resetting the touch panel.\n", dev_name(dev), __func__);

	ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);
	mutex_unlock(&ts->device_mode_mutex);
	return 0;
}


static ssize_t ft5x06_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	const struct firmware * fw_blob = NULL;
	char fw_name[256];

	mutex_lock(&ts->device_mode_mutex);

	// buf contains the newline-terminated firmware file name - remove the newline.
	strncpy(fw_name, buf, size - 1);
	fw_name[size - 1] = '\0';

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Processing input file %s.\n", dev_name(dev), __func__, fw_name);
	if (request_firmware(&fw_blob, fw_name, dev))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: request_firmware failed.\n", dev_name(dev), __func__);
		goto error_return;
	}

	if (ft5x06_perform_fw_upgrade(dev, fw_blob->data, fw_blob->size))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not update the firmware using %s.\n", dev_name(dev), __func__, fw_name);
		goto error_return;
	}

error_return:
	if (fw_blob)
	{
		release_firmware(fw_blob);
	}

	mutex_unlock(&ts->device_mode_mutex);
	return size;
}


static ssize_t ft5x06_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    /* place holder for future use */
    return -EPERM;
}


static ssize_t ft5x06_fmreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	unsigned long fmreg = 0;
	int retval = 0;

	mutex_lock(&ts->device_mode_mutex);

	retval = strict_strtoul(buf, 16, &fmreg);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid input specified (\"%s\").\n", dev_name(dev), __func__, buf);
		goto error_return;
	}

	if ((FT5x06_FMREG_MAX < fmreg) ||
			(FT5x06_FMREG_RESERVED == fmreg) ||
			((FT5x06_FMREG_RESERVEDBLK_START <= fmreg) && (FT5x06_FMREG_RESERVEDBLK_END >= fmreg)))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid register specified (%lu).\n", dev_name(dev), __func__, fmreg);
		goto error_return;
	}

	ts->factory_mode_register = (u8)(fmreg & 0x000000FF);

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}


static ssize_t ft5x06_fmreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	return snprintf(buf, PAGE_SIZE, "0x%02X\n", ts->factory_mode_register);
}


static ssize_t ft5x06_fmval_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	unsigned long fmval = 0;
	int retval = 0;
	u8  regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);

	retval = strict_strtoul(buf, 16, &fmval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid input specified (\"%s\").\n", dev_name(dev), __func__, buf);
		goto error_return;
	}

	if (0xFF < fmval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid value specified (%lu).\n", dev_name(dev), __func__, fmval);
		goto error_return;
	}

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	regval = (u8)(fmval & 0x000000FF);

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Writing 0x%02x to the register at offset 0x%02x.\n", dev_name(dev), __func__, regval, ts->factory_mode_register);

	retval = i2c_smbus_write_i2c_block_data(ts->client, ts->factory_mode_register, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the register at offset 0x%02x.\n", dev_name(dev), __func__, regval, ts->factory_mode_register);
		goto error_restore_mode;
	}

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}


static ssize_t ft5x06_fmval_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	int     retval = 0;
	u8      regval = 0;

	mutex_lock(&ts->device_mode_mutex);

	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not put the touch panel in factory mode.\n", dev_name(dev), __func__);
		goto error_restore_mode;
	}

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reading the register at offset 0x%02x.\n", dev_name(dev), __func__, ts->factory_mode_register);
	retval = i2c_smbus_read_i2c_block_data(ts->client, ts->factory_mode_register, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read the register at offset 0x%02x.\n", dev_name(dev), __func__, ts->factory_mode_register);
		goto error_restore_mode;
	}
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: The register at offset 0x%02x contains value 0x%02x.\n", dev_name(dev), __func__, ts->factory_mode_register, regval);

	num_read_chars += snprintf(buf, PAGE_SIZE, "0x%02X\n", regval);

error_restore_mode:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not return the touch panel to working mode.\n", dev_name(dev), __func__);
		}

	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_wmreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	unsigned long wmreg = 0;
	int retval = 0;

	mutex_lock(&ts->device_mode_mutex);
	retval = strict_strtoul(buf, 16, &wmreg);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid input specified (\"%s\").\n", dev_name(dev), __func__, buf);
		goto error_return;
	}

	if((wmreg >= 0x00) && (wmreg <= 0xFF))
	{
		ts->working_mode_register = (u8)(wmreg & 0x000000FF);
	}
	else
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid register specified (%lu).\n", dev_name(dev), __func__, wmreg);
		goto error_return;
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}


static ssize_t ft5x06_wmreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	return snprintf(buf, PAGE_SIZE, "0x%02X\n", ts->working_mode_register);
}


static ssize_t ft5x06_wmval_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	unsigned long wmval = 0;
	int retval = 0;
	u8  regval = 0x00;

	mutex_lock(&ts->device_mode_mutex);
	retval = strict_strtoul(buf, 16, &wmval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid input specified (\"%s\").\n", dev_name(dev), __func__, buf);
		goto error_return;
	}

	if (0xFF < wmval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Invalid value specified (%lu).\n", dev_name(dev), __func__, wmval);
		goto error_return;
	}

	regval = (u8)(wmval & 0x000000FF);

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Writing 0x%02x to the register at offset 0x%02x.\n", dev_name(dev), __func__, regval, ts->working_mode_register);
	retval = i2c_smbus_write_i2c_block_data(ts->client, ts->working_mode_register, sizeof(u8), &regval);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the register at offset 0x%02x.\n", dev_name(dev), __func__, regval, ts->working_mode_register);
		goto error_return;
	}

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return size;
}


static ssize_t ft5x06_wmval_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int     retval = 0;
	u8      regval = 0;

	mutex_lock(&ts->device_mode_mutex);

	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Reading the register at offset 0x%02x\n", dev_name(dev), __func__, ts->working_mode_register);
	retval = i2c_smbus_read_i2c_block_data(ts->client, ts->working_mode_register, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read the value of the register at offset 0x%02x\n", dev_name(dev), __func__, ts->working_mode_register);
		goto error_return;
	}
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: The register at offset 0x%02x contains value 0x%02x\n", dev_name(dev), __func__, ts->working_mode_register, regval);

	num_read_chars += snprintf(buf, PAGE_SIZE, "0x%02X\n", regval);

error_return:
	mutex_unlock(&ts->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ftx_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ft5x06 *ts = (struct ft5x06 *)dev_get_drvdata(dev);
	int bytes_read = 0;
	int retval;

	mutex_lock(&ts->device_mode_mutex);
	retval = ftx_read_info(ts);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read info from the device.\n", dev_name(dev), __func__);
		return 0;
	}

	bytes_read += sprintf(buf + bytes_read, "FW Ver      : 0x%02x\n", ts->info.fw_ver);
	bytes_read += sprintf(buf + bytes_read, "FocalTech ID: 0x%02x\n", ts->info.focaltech_id);
	bytes_read += sprintf(buf + bytes_read, "Product ID  : 0x%02x\n", ts->info.product_id);
	bytes_read += sprintf(buf + bytes_read, "Lib Version : 0x%02x 0x%02x\n", ts->info.lib_ver_h, ts->info.lib_ver_l);
	bytes_read += sprintf(buf + bytes_read, "\n");
	mutex_unlock(&ts->device_mode_mutex);

	return bytes_read;
}
static DEVICE_ATTR(info, 0444, ftx_info_show, NULL);

static ssize_t ftx_report_rate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ft5x06 *ts = (struct ft5x06 *)dev_get_drvdata(dev);
	int retval;
	u8 regval;

	mutex_lock(&ts->device_mode_mutex);
	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_RPT_RATE, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read the value of the register at offset 0x%02x.\n", dev_name(dev), __func__, FT5x06_WMREG_RPT_RATE);
		goto err_return;
	}
	else
	{
		retval = sprintf(buf, "0x%02x\n", regval);
	}
err_return:
	mutex_unlock(&ts->device_mode_mutex);
	return retval;
}
static ssize_t ftx_report_rate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ft5x06 *ts = (struct ft5x06 *)dev_get_drvdata(dev);
	int retval;
	unsigned int regval;

	mutex_lock(&ts->device_mode_mutex);
	if(sscanf(buf, "%u", &regval) >= 1)
	{
		retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_RPT_RATE, sizeof(u8), (u8 *) &regval);
		if (0 != retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the register at offset 0x%02x.\n", dev_name(dev), __func__, regval, FT5x06_WMREG_RPT_RATE);
		}
	}
	else
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: report_rate write error.\n", dev_name(dev), __func__);
	}
	mutex_unlock(&ts->device_mode_mutex);
	return count;
}
static DEVICE_ATTR(report_rate, 0664, ftx_report_rate_show, ftx_report_rate_store);

static ssize_t ftx_fingers_supported_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ft5x06 *ts = (struct ft5x06 *)dev_get_drvdata(dev);
	int retval;
	u8 regval;

	mutex_lock(&ts->device_mode_mutex);
	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_MAX_TOUCHES, sizeof(u8), &regval);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read the value of the register at offset 0x%02x.\n", dev_name(dev), __func__, FT5x06_WMREG_MAX_TOUCHES);
		goto err_return;
	}
	retval = sprintf(buf, "0x%02x\n", regval);
err_return:
	mutex_unlock(&ts->device_mode_mutex);
	return retval;
}
static ssize_t ftx_fingers_supported_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ft5x06 *ts = (struct ft5x06 *)dev_get_drvdata(dev);
	int retval;
	unsigned int regval;

	mutex_lock(&ts->device_mode_mutex);
	if(sscanf(buf, "%u", &regval) >= 1)
	{
		retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_MAX_TOUCHES, sizeof(u8), (u8 *) &regval);
		if (0 != retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the register at offset 0x%02x.\n", dev_name(dev), __func__, regval, FT5x06_WMREG_MAX_TOUCHES);
		}
	}
	else
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: n_touches write error.\n", dev_name(dev), __func__);
	}
	mutex_unlock(&ts->device_mode_mutex);
	return count;
}
static DEVICE_ATTR(fingers_supported, 0664, ftx_fingers_supported_show, ftx_fingers_supported_store);

static ssize_t ftx_dbg_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	int bytes_read = 0;

	mutex_lock(&ts->device_mode_mutex);
	bytes_read += sprintf(buf + bytes_read, "current debug level = %u\n", cur_dbg_level);
	bytes_read += sprintf(buf + bytes_read, "highest level = %u\n", dbg_level_highest);
	bytes_read += sprintf(buf + bytes_read, "lowest level = %u\n", dbg_level_lowest);
	mutex_unlock(&ts->device_mode_mutex);
	return bytes_read;
}
static ssize_t ftx_dbg_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);
	unsigned int dbg_val;

	mutex_lock(&ts->device_mode_mutex);
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: entering...input = %s\n", dev_name(dev), __func__, buf);
	if(sscanf(buf, "%u", &dbg_val) >= 1)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: scanned input = %u\n", dev_name(dev), __func__, dbg_val);
		if((dbg_val >= dbg_level_highest) && (dbg_val <= dbg_level_lowest))
		{
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: changing current debug level from %u to %u\n", dev_name(dev), __func__, cur_dbg_level, dbg_val);
			cur_dbg_level = dbg_val;
		}
	}
	mutex_unlock(&ts->device_mode_mutex);
	return count;
}
static DEVICE_ATTR(dbg_level, 0664, ftx_dbg_level_show, ftx_dbg_level_store);

static ssize_t ftx_driver_buildid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);
	int bytes_read = 0;

	mutex_lock(&ts->device_mode_mutex);
	bytes_read += sprintf(buf + bytes_read, "%s:%s\n", __DATE__, __TIME__);
	mutex_unlock(&ts->device_mode_mutex);

	return bytes_read;
}
static DEVICE_ATTR(build_id, 0444, ftx_driver_buildid_show, NULL);

static ssize_t ftx_suspend_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06 *ts = (struct ft5x06 *)i2c_get_clientdata(client);
	int bytes_read = 0;

	mutex_lock(&ts->device_mode_mutex);
	mutex_lock(&ts->lock_ftx_suspend_state);
	bytes_read += sprintf(buf + bytes_read, "%d\n", ts->ftx_suspend_state);
	mutex_unlock(&ts->lock_ftx_suspend_state);
	mutex_unlock(&ts->device_mode_mutex);

	return bytes_read;
}
static DEVICE_ATTR(power_state, 0444, ftx_suspend_state_show, NULL);

#ifdef CONFIG_TOUCHSCREEN_FT5x06_TEST
// Add for factory testing
#define MAX_READ_DATA 16
#define LWJ_FAILED	-1
#define LWJ_DEBUG
int g_testfilenum = 0;
char szstation[32] = "final";
u8 g_ftsbaselineresult=1;
u8 g_ftsvoltageresult=1;
u8 g_ftscrosstalkoddresult=1;
u8 g_ftsfullpanelresult = 1;
u8 g_ftsfullpaneltest = 0;
int baseline[FT5x06_NUM_TX][FT5x06_NUM_RX];
int voltage[FT5x06_NUM_TX][FT5x06_NUM_RX];
int crosstalk[FT5x06_NUM_TX][FT5x06_NUM_RX];

int threshold_mean_Tx = 100;
int threshold_V = 50;
int N = 4;
double c = 0.5;
double G = 0.8;

// short/open 
int threshold_mean_Rx = 275;
int threshold_very_small = 50;
int threshold_small = 275;
int threshold_cross_talk_Rx = 500;
int threshold_min_Rx = 100;
int threshold_Rx = 275;
int threshold_TX_OPEN = 50; //TX line open threshold
int threshold_RX_Short = 50;

static ssize_t ft5x06_resettest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);

    int test_result = 0;
    int retval = 0;
    u8  regval = 0x00;

    mutex_lock(&ts->device_mode_mutex);

    retval = ft5x06_enter_factory_mode(ts);
    if (0 != retval)
    {
        dev_err(dev, "%s() - ERROR: Could not put the device in Factory Mode.\n", __FUNCTION__);
        goto error_restore_mode;
    }

    dev_dbg(dev, "%s() - Resetting Touch Panel\n", __FUNCTION__);

    ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);

    dev_dbg(dev, "%s() - Verifying that the Touch Panel has been returned to Working Mode...\n", __FUNCTION__);

    retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &regval);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not read from the Device Mode register.\n", __FUNCTION__);
        goto error_restore_mode;
    }

    if ((regval & FT5x06_MODE_MASK) != FT5x06_MODE_WORKING)
    {
        dev_err(dev, "%s() - ERROR: The Touch Panel was not returned to Working Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
        retval = FT5x06_ERR_NOT_FACTORY_MODE;
        goto error_restore_mode;
    }

    test_result = 1;

error_restore_mode:
    retval = ft5x06_exit_factory_mode(ts);
    if (0 != retval)
    {
        dev_err(dev, "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n", __FUNCTION__);
    }

    dev_dbg(dev, "%s() - Reset Test Result: %s\n", __FUNCTION__, (test_result ? "PASS" : "FAIL"));

    mutex_unlock(&ts->device_mode_mutex);

    return snprintf(buf, PAGE_SIZE, "%d\n", test_result);
}


static ssize_t ft5x06_dump_rxcac_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);

    int i      = 0;
    int retval = 0;
    u8  regval = 0x00;
    u8 testmode = 0x00;
    u8 m = 1;

    ssize_t num_read_chars = 0;

    mutex_lock(&ts->device_mode_mutex);

    retval = ft5x06_enter_factory_mode(ts);
    if (0 != retval)
    {
        dev_err(dev, "%s() - ERROR: Could not put the device in Factory Mode.\n", __FUNCTION__);
        goto error_restore_mode;
    }

	m = 1;
	for (i = 0; ((i < FT5x06_NUM_RX) && ((num_read_chars + 16) < PAGE_SIZE)); i++) {
		if (i >= FT5606_TEST_MODE_RX) {
			if (i >= FT5606_TEST_MODE_RX*2)
				m = 3;
			else if (i >= FT5606_TEST_MODE_RX*3)
				m = 4;
			else
				m = 2;
			testmode = ft5606_factory_mode[m-1] << 4;
			retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (0 != retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
				goto error_restore_mode;
			}
		}
		retval = i2c_smbus_read_i2c_block_data(ts->client, (FT5x06_FMREG_RX_0_CAC + i - FT5606_TEST_MODE_RX*(m-1)), sizeof(u8), &regval);
		if (0 > retval)
		{
			dev_err(dev, "%s() - ERROR: Could not read from the RX%u_CAC register.\n", __FUNCTION__, i);
			goto error_restore_mode;
		}
		num_read_chars += sprintf(&(buf[num_read_chars]), "RX%02u_CAC = 0x%02X\n", i, regval);

		if (i >= FT5606_TEST_MODE_RX) {
			testmode = FACTORY_MODE0 <<4;
			retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (retval != 0) {
				dev_err(dev, "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n", __FUNCTION__, i);
				goto error_restore_mode;
			}
		}
	}

error_restore_mode:
    retval = ft5x06_exit_factory_mode(ts);
    if (0 != retval)
    {
        dev_err(dev, "%s() - ERROR: Could not return the Touch Panel to Working Mode.\n", __FUNCTION__);
    }

    mutex_unlock(&ts->device_mode_mutex);

    return num_read_chars;
}

static int readCfg(char *filepath, int *pNum, int *pDelta) {
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	int i;
	off_t fsize;
	char databuf[MAX_READ_DATA];
	//int baseline[FT5x06_NUM_TX][FT5x06_NUM_RX];
	char tmpbuf;
	loff_t pos;
	//int filelen = 0;
	int index=0;int tx=0;int rx=0;
	mm_segment_t old_fs;

	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
		}
	if(IS_ERR(pfile)){
		printk("error occured while opening file %s.\n", filepath);
		return -1;
		}
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	memset(databuf, 0, sizeof(databuf));
	vfs_read(pfile, databuf, sizeof(databuf), &pos);
	printk("read data from %s size = %d\n", filepath, fsize);

	i = sscanf(databuf, "%d %d", pNum, pDelta);
	if (i <= 0) printk("sscanf fail!\n");

	printk("maxnum=%d  maxdelta=%d \n", *pNum, *pDelta);
	
	printk(KERN_DEBUG"close file and resume the fs\n");
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

static int readTXRXdatafromfile(char * filepath, int data[][FT5x06_NUM_RX])
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	int i;
	off_t fsize;
	char databuf[MAX_READ_DATA];
	//int baseline[FT5x06_NUM_TX][FT5x06_NUM_RX];
	char tmpbuf;
	loff_t pos;
	//int filelen = 0;
	int index=0;int tx=0;int rx=0;
	mm_segment_t old_fs;

	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
		}
	if(IS_ERR(pfile)){
		printk("error occured while opening file %s.\n", filepath);
		return -1;
		}
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	char * buf;
	buf=(char *) kmalloc(fsize+1,GFP_ATOMIC);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	memset(databuf, 0, sizeof(databuf));
	vfs_read(pfile, buf, fsize, &pos);
	//printk("read data from file size = %d\n", fsize);
	for(i=0; i<fsize; i++){
		tmpbuf = buf[i];
		if(tmpbuf == ' '){
			if(strlen(databuf)==0)
				continue;
			strict_strtoul(databuf, 10, (unsigned long *)&data[tx][rx]);
			//printk("%03d ", data[tx][rx]);
			index = 0;
			rx++;
			memset(databuf, 0, sizeof(databuf));
			}
		else if(tmpbuf == '\n'){
			if(strlen(databuf)==0)
				continue;
			strict_strtoul(databuf, 10, (unsigned long *)&data[tx][rx]);
			//printk("%03d ", data[tx][rx]);
			//printk("\n");
			rx = 0;
			tx++;
			index = 0;
			memset(databuf, 0, sizeof(databuf));
			}
		else if(tmpbuf == '\r'){
			if(strlen(databuf)==0)
				continue;
			strict_strtoul(databuf, 10, (unsigned long *)&data[tx][rx]);
			//printk("%03d ", data[tx][rx]);
			//printk("\n");
			rx = 0;
			tx++;
			index = 0;
			i++;
			memset(databuf, 0, sizeof(databuf));
			}
		else{
			databuf[index] = tmpbuf;
			index++;
			}
		}
	kfree(buf);
	//printk("close file and resume the fs\n");
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

static int writeTXRXdatatofile(char * filepath, int data[][FT5x06_NUM_RX])
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	int i, j;
	off_t fsize;
	char databuf[MAX_READ_DATA];
	//int baseline[FT5x06_NUM_TX][FT5x06_NUM_RX];
	char tmpbuf;
	loff_t pos;
	//int filelen = 0;
	int index=0;int tx=0;int rx=0;
	mm_segment_t old_fs;

	printk("%s %s\n", __FUNCTION__, filepath);

	if(NULL == pfile){
		pfile = filp_open(filepath, O_WRONLY, 0);
	}
	if(IS_ERR(pfile)){
		printk("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	char * buf;
	buf=(char *) kmalloc(fsize+1,GFP_ATOMIC);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	for (i = 0; i < FT5x06_NUM_TX; i++) {
		for (j = 0; j < FT5x06_NUM_RX; j++) {
			memset(databuf, 0, sizeof(databuf));			
			snprintf(databuf, sizeof(databuf),  "%d ", data[i][j]);
			//printk("%s ", databuf);
			vfs_write(pfile, databuf, strlen(databuf), &pos);
		}
		vfs_write(pfile, "\n", 1, &pos);
		//printk("\n");
	}

	//printk("read data from file size = %d\n", fsize);
	kfree(buf);
	//printk("close file and resume the fs\n");
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}	

static int writebuftofile(char * filepath, char *buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	int i, j;
	off_t fsize;
	char databuf[MAX_READ_DATA];
	//int baseline[FT5x06_NUM_TX][FT5x06_NUM_RX];
	char tmpbuf;
	loff_t pos;
	//int filelen = 0;
	int index=0;int tx=0;int rx=0;
	mm_segment_t old_fs;

	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDWR|O_CREAT, 0);
	}
	if(IS_ERR(pfile)){
		printk("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_write(pfile, buf, strlen(buf), &pos);


	printk("write  size = %d\n", strlen(buf));
	//printk("close file and resume the fs\n");
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}	

/*Same as ft5x06_read_data */
static int ft5x06_read_rawdata(struct i2c_client *client, int data[][FT5x06_NUM_RX])
{
	u8 testmode = 0x00;
	u8 readlen = 0;
	int retval  = 0;
	u8 regaddr = 0x00;
	int i       = 0;
	int j       = 0;
	int rx = 0;
	u16 dataval = 0x0000;
	u8  devmode = 0x00;
	u8  rownum  = 0x00;
	u8 read_buffer[FT5x06_NUM_RX * 2];

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Verifying that touch panel is in factory mode.\n", dev_name(&(client->dev)), __func__);
	retval = i2c_smbus_read_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &devmode);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read from DEVICE MODE register.\n", dev_name(&(client->dev)), __func__);
		goto error_return;
	}

	if (FT5x06_MODE_FACTORY != (devmode & FT5x06_MODE_MASK))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Touch panel is not in factory mode\n", dev_name(&(client->dev)), __func__);
		retval = FT5x06_ERR_NOT_FACTORY_MODE;
		goto error_return;
	}

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Initiating a scan for raw data...\n", dev_name(&(client->dev)), __func__);

	devmode |=  FT5x06_SCAN_START;
	retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &devmode);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
		goto error_return;
	}

	msleep(FT5x06_SCAN_DELAY);

	retval = i2c_smbus_read_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &devmode);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read updated value of DEVICE MODE register.\n", dev_name(&(client->dev)), __func__);
		goto error_return;
	}

	if (FT5x06_SCAN_DONE != (devmode & FT5x06_SCAN_MASK))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Raw data did not complete after %u ms.\n", dev_name(&(client->dev)), __func__, FT5x06_SCAN_DELAY);
		retval = FT5x06_ERR_SCAN_NOT_DONE;
		goto error_return;
	}

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Reading raw data...\n", dev_name(&(client->dev)), __func__);

	for(i=0; i<FT5x06_NUM_TX; i++) {
		memset(read_buffer, 0x00, (FT5x06_NUM_RX * 2));
		rownum = i;
		retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_ROW_ADDR, sizeof(u8), &rownum);
		if (0 != retval) {
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write the row number %u\n", dev_name(&(client->dev)), __func__, rownum);
			goto error_return;
		}

		msleep(1);

		/* Read the data for this row FT5606_TEST_MODE_RX=20*/
		for (j = 0; j < (FT5x06_NUM_RX/FT5606_TEST_MODE_RX + ((FT5x06_NUM_RX % FT5606_TEST_MODE_RX)?1:0)); j++) {
			if (j > 0) {
				testmode = ft5606_factory_mode[j] << 4;
				retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
				if (0 != retval)
				{
					DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
					goto error_return;
				}
			}

			readlen = (FT5x06_NUM_RX - FT5606_TEST_MODE_RX*j) * 2;

			if (0 == readlen) {
				readlen = FT5606_TEST_MODE_RX * 2;
			}

			if (readlen > FT5606_TEST_MODE_RX*2)
				readlen = FT5606_TEST_MODE_RX * 2;

			regaddr = FT5x06_FMREG_RAWDATA_0_H;
			//retval = i2c_smbus_read_i2c_block_data(client, FT5x06_FMREG_RAWDATA_0_H, readlen, read_buffer + j*FT5606_TEST_MODE_RX*2);
			retval = ft5x0x_i2c_Read(client, &regaddr, 1,  read_buffer + j*FT5606_TEST_MODE_RX*2, readlen);
			if (0 > retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not read row %u raw data.\n", dev_name(&(client->dev)), __func__, rownum);
				goto error_return;
			}

			//printk("j=%d  real=%d except=%d\n", j, retval, readlen);
		}

		/*return test mode 0*/
		testmode = FACTORY_MODE0 <<4;
		retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
		if (0 != retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
			goto error_return;
		}

		rx = 0;
		//printk("read_data %d\n", *p_num_read_chars);
		/* Each data value can be up to 5 chars long. Add a space and we need 6 chars to store it. */
		for (j = 0; j < FT5x06_NUM_RX * 2; j += 2)
		{
			dataval  = read_buffer[j];
			dataval  = (dataval << 8);
			dataval |= read_buffer[j+1];
			data[i][rx] = dataval;
			rx++;
			//*p_num_read_chars += sprintf(&(output_buffer[*p_num_read_chars]), "%u ", dataval);
		}
	}
	retval = 0;
error_return:
	return retval;
}

int baseline_full[FT5x06_NUM_TX][FT5x06_NUM_RX];
int baseline_file[FT5x06_NUM_TX][FT5x06_NUM_RX];
static ssize_t ft5x06_ftsfullpanel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int 	retval = 0;
	int i, j;
//	u8		regval = 0x00;
	int flag = 1;
	int totalfailed = 0;
	int maxnum = 10;
	int maxdelta = 10;
	bool limitchanged = false;

	int differ[FT5x06_NUM_TX][FT5x06_NUM_RX];
	char filepath_fullpanel[64];
	char cfgfilepath[64];
	
	sprintf(cfgfilepath, "/config/limitconfig_fullpaneltouch_%s.txt", szstation);
	readCfg(cfgfilepath, &maxnum, &maxdelta);
	
	sprintf(filepath_fullpanel, "/config/fullpaneltouch_%s.txt", szstation);
	g_ftsfullpaneltest = 1;
	mutex_lock(&ts->device_mode_mutex);
	g_ftsfullpanelresult=1;
	retval = ft5x06_enter_factory_mode(ts);
	   if (0 != retval)
	   {
		   dev_err(dev, "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n", __FUNCTION__);
		   goto error_return;
	   }

	  retval = ft5x06_read_rawdata(client, baseline_full);

	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not read Raw Data from the Touch Panel.\n", __FUNCTION__);
		goto error_return;
	}
	//read baseline form file
	dev_dbg(dev, "baseline test:read baseline form file\n");
	memset(baseline_file, 0xffff, sizeof(baseline_file));	
	if(readTXRXdatafromfile(filepath_fullpanel, baseline_file) <0){
		dev_err(dev, "%s() - ERROR: read baseline_max from file error\n", __FUNCTION__);
        	goto error_return;
	}

	//compare
	dev_dbg(dev, "baseline test:compare\n");
	for(i=0; i<FT5x06_NUM_TX; i++){
		for(j=0; j<FT5x06_NUM_RX; j++) {
			differ[i][j] = baseline_full[i][j] - baseline[i][j];
			num_read_chars += sprintf(&(buf[num_read_chars]), "%d ", differ[i][j]);
			if(differ[i][j] < baseline_file[i][j]){
				dev_err(dev, "baseline error. tx=%d rx=%d value=%d\n", i+1, j+1, baseline[i][j]);
				flag = 0;
				totalfailed++;
				if ((baseline_file[i][j] - differ[i][j]) < maxdelta) {
					printk("update %d,%d %d\n", i, j, maxdelta);
					baseline_file[i][j] = differ[i][j];
					limitchanged = true;
				}
			}
		}
			buf[num_read_chars-1] = '\n';
	}
	if(flag){
		g_ftsfullpanelresult = 0;
	}

	if (totalfailed < maxnum) {
		if (limitchanged) {
			printk("update %d < %d\n", totalfailed, maxnum);
			writeTXRXdatatofile(filepath_fullpanel, baseline_file);
		}
	}
	
error_return:
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		 dev_err(dev, "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n", __FUNCTION__);
	}
	mutex_unlock(&ts->device_mode_mutex);

	return num_read_chars;
}

int baseline_max[FT5x06_NUM_TX][FT5x06_NUM_RX];
int baseline_min[FT5x06_NUM_TX][FT5x06_NUM_RX];
static ssize_t ft5x06_ftsbaseline_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int retval = 0;
	int i, j;
	u8		regval = 0x00;
	int flag = 1;
	int num = 10;
	int delta = 10;
	int totalfailed = 0;
	//ssize_t testresult = LWJ_FAILED;
	int maxnum = 10;
	int maxdelta = 10;
	bool minchanged = false;
	bool maxchanged = false;

	char filepath_max[64];
	char filepath_min[64];

	char cfgfilepath[64];
	sprintf(cfgfilepath, "/config/limitconfig_baseline_%s.txt", szstation);
	readCfg(cfgfilepath, &maxnum, &maxdelta);
	
	sprintf(filepath_max, "/config/baseline_%s_max.txt", szstation);
	sprintf(filepath_min, "/config/baseline_%s_min.txt", szstation);
	g_ftsbaselineresult = 1;
	mutex_lock(&ts->device_mode_mutex);
	retval = ft5x06_enter_factory_mode(ts);
	   if (0 != retval)
	   {
		   dev_err(dev, "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n", __FUNCTION__);
		   goto error_restore_baseline;
	   }
/*
//	add by zhangjk @20110915; read raw data, not the baseline
	regval = FT5x06_BASELINE_ENABLE;
	retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_BASELINE_ENABLE, sizeof(u8), &regval);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not write to the Baseline register.\n", __FUNCTION__);
		goto error_return;
	}

	msleep(FT5x06_VERIFICATION_DELAY_MS);

	regval = FT5x06_BASELINE_DISABLE;
	retval = i2c_smbus_read_i2c_block_data(client, FT5x06_FMREG_BASELINE_ENABLE, sizeof(u8), &regval);
	if (0 > retval)
	{
		dev_err(dev, "%s() - ERROR: Could not read the updated value of the Baseline register.\n", __FUNCTION__);
		goto error_restore_baseline;
	}

	if (FT5x06_BASELINE_ENABLE != (FT5x06_BASELINE_MASK & regval))
	{
		dev_err(dev,  "%s() - ERROR: The Baseline register contained 0x%02X (expected 0x%02X).\n", __FUNCTION__, (FT5x06_BASELINE_MASK & regval), FT5x06_BASELINE_ENABLE);
		goto error_restore_baseline;
	}
*/
	/* Read raw data */
	dev_dbg(dev, "baseline test:read raw data\n");
	retval = ft5x06_read_rawdata(client, baseline);

	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not read Raw Data from the Touch Panel.\n", __FUNCTION__);
		goto error_restore_baseline;
	}
	//read baseline form file
	dev_dbg(dev, "baseline test:read baseline form file\n");
	memset(baseline_max, 0, sizeof(baseline_max));
	if(readTXRXdatafromfile(filepath_max, baseline_max) <0){
		dev_err(dev, "%s() - ERROR: read baseline_max from file error\n", __FUNCTION__);
            	goto error_restore_baseline;
	}

	memset(baseline_min, 0xffff, sizeof(baseline_min));
	if(readTXRXdatafromfile(filepath_min, baseline_min)<0){
		dev_err(dev, "%s() - ERROR: read baseline_min from file error\n", __FUNCTION__);
            	goto error_restore_baseline;
	}
	//compare
	dev_dbg(dev, "baseline test:compare\n");
	maxchanged = false;
	minchanged = false;
	for(i=0; i<FT5x06_NUM_TX; i++){
		for(j=0; j<FT5x06_NUM_RX; j++){
			num_read_chars += sprintf(&(buf[num_read_chars]), "%d ", baseline[i][j] - 8000);
			if(baseline[i][j] > baseline_max[i][j]){
				dev_err(dev, "baseline max error. tx=%d rx=%d value=%d\n", i+1, j+1, baseline[i][j]);
				flag = 0;
				totalfailed++;
				if ((baseline[i][j] - baseline_max[i][j]) < maxdelta) {
					printk("update %d,%d %d\n", i,j, maxdelta);
					baseline_max[i][j] = baseline[i][j];
					maxchanged = true;
				}
			}
			if(baseline[i][j] < baseline_min[i][j]){
				dev_err(dev, "baseline min error. tx=%d rx=%d value=%d\n", i+1, j+1, baseline[i][j]);
				flag = 0;
				totalfailed++;
				if ((baseline_min[i][j] - baseline[i][j]) < maxdelta) {
					printk("update %d,%d %d\n", i,j, maxdelta);
					baseline_min[i][j] = baseline[i][j];
					minchanged = true;
				}				
			}
		}
		buf[num_read_chars-1] = '\n';
	}
	if(flag){
		g_ftsbaselineresult = 0;
	}

	if (totalfailed < maxnum) {
		if (maxchanged) {
			writeTXRXdatatofile(filepath_max, baseline_max);
			printk("update max files %d < %d\n", totalfailed, maxnum);
		}

		if (minchanged) {
			printk("update min files %d < %d\n", totalfailed, maxnum);
			writeTXRXdatatofile(filepath_min, baseline_min);
		}
	} else {
		printk("failed to much %d > %d\n", totalfailed, maxnum);
	}	
error_restore_baseline:
	dev_dbg(dev,  "%s() - Restoring the Baseline register...\n", __FUNCTION__);
/*
//	add by zhangjk @20110915; read raw data, not the baseline
	regval = FT5x06_BASELINE_DISABLE;
	retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_BASELINE_ENABLE, sizeof(u8), &regval);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not write to the Baseline register.\n", __FUNCTION__);
		goto error_return;
	}

	msleep(FT5x06_VERIFICATION_DELAY_MS);

	regval = FT5x06_BASELINE_ENABLE;
	retval = i2c_smbus_read_i2c_block_data(client, FT5x06_FMREG_BASELINE_ENABLE, sizeof(u8), &regval);
	if (0 > retval)
	{
		dev_err(dev, "%s() - ERROR: Could not read the updated value of the Baseline register.\n", __FUNCTION__);
		goto error_return;
	}

	if (FT5x06_BASELINE_DISABLE != (FT5x06_BASELINE_MASK & regval))
	{
		dev_err(dev, "%s() - ERROR: The Baseline register contained 0x%02X (expected 0x%02X).\n", __FUNCTION__, (FT5x06_BASELINE_MASK & regval), FT5x06_BASELINE_DISABLE);
		//goto error_restore_mode;
	}
*/
	retval = ft5x06_exit_factory_mode(ts);
	if (0 != retval)
	{
		 dev_err(dev, "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n", __FUNCTION__);
	}
	mutex_unlock(&ts->device_mode_mutex);

	return num_read_chars;
}

int rawbase_before[FT5x06_NUM_TX][FT5x06_NUM_RX];
int rawbase_after[FT5x06_NUM_TX][FT5x06_NUM_RX];
int voltage_max[FT5x06_NUM_TX][FT5x06_NUM_RX];
int voltage_min[FT5x06_NUM_TX][FT5x06_NUM_RX];
static ssize_t ft5x06_ftsvoltage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;
	int flag = 1;
	//int testresult = LWJ_FAILED;

	char filepath_max[64];
	char filepath_min[64];
	u8 original_voltage;
	int     retval  = 0;
	u8      currvoltage = 0;
	int i,j,k;
	char cfgfilepath[64];
	int maxnum = 10;
	int maxdelta = 10;
	int totalfailed = 0;
	bool maxchanged, minchanged;
	
	sprintf(cfgfilepath, "/config/limitconfig_voltage_%s.txt", szstation);
	readCfg(cfgfilepath, &maxnum, &maxdelta);

	g_ftsvoltageresult = 1;
	sprintf(filepath_max, "/config/voltage_%s_max.txt", szstation);
	sprintf(filepath_min, "/config/voltage_%s_min.txt", szstation);

	mutex_lock(&ts->device_mode_mutex);
	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n", __FUNCTION__);
		goto error_restore_voltage;
	}

	//step 1: get original voltage
    retval = i2c_smbus_read_i2c_block_data(client, FT5x06_FMREG_DRIVER_VOLTAGE, sizeof(u8), &original_voltage);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not read the value of the Device Voltage register.\n", __FUNCTION__);
        goto error_retrun;
    }

	//step 2: get rawbase_before
	retval = ft5x06_read_rawdata(client, rawbase_before);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not read Raw Data from the Touch Panel.\n", __FUNCTION__);
		goto error_restore_voltage;
	}

	//step 3: set voltage=1
	currvoltage = 1;
	retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DRIVER_VOLTAGE, sizeof(u8), &currvoltage);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not write the value of the Device Voltage register.\n", __FUNCTION__);
        goto error_restore_voltage;
    }

	//step 4: get rawbase_after
	for(i=0; i<3; i++){
		msleep(100);
	retval = ft5x06_read_rawdata(client, rawbase_after);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not read Raw Data from the Touch Panel.\n", __FUNCTION__);
		goto error_restore_voltage;
	}
	//step 5:set voltage=original_voltage
	}
	retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DRIVER_VOLTAGE, sizeof(u8), &original_voltage);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not write the value of the Device Voltage register.\n", __FUNCTION__);
        goto error_restore_voltage;
    }

	//step 6: computer differ and compare to voltage that from file
	//differ >= voltage
	memset(voltage_max, 0, sizeof(voltage_max));
	memset(voltage_min, 0xffff, sizeof(voltage_min));

	if(readTXRXdatafromfile(filepath_max, voltage_max)<0){
		dev_err(dev,  "%s() - ERROR: read voltage_max from file error\n", __FUNCTION__);
        	goto error_restore_voltage;
	}

	if(readTXRXdatafromfile(filepath_min, voltage_min)<0){
		dev_err(dev,  "%s() - ERROR: read voltage_min from file error\n", __FUNCTION__);
        	goto error_restore_voltage;
	}
	maxchanged = false;
	minchanged = false;
	for(i=0; i<FT5x06_NUM_TX; i++){
		for(j=0; j<FT5x06_NUM_RX; j++){
			voltage[i][j] = rawbase_after[i][j] - rawbase_before[i][j];
			num_read_chars += sprintf(&(buf[num_read_chars]), "%d ", voltage[i][j]);
			if(voltage[i][j] < voltage_min[i][j]){
				flag = 0;
				dev_err(dev, "voltage min test error. tx=%d rx=%d value=%d.\n", i+1, j+1, voltage[i][j]);
				totalfailed++;
				 if ((voltage_min[i][j] - voltage[i][j]) < maxdelta) {
					printk("update %d,%d %d\n", i, j, maxdelta);
					voltage_min[i][j] = voltage[i][j];
					minchanged = true;
				 }
			}

			if(voltage[i][j] > voltage_max[i][j]){
				flag = 0;
				dev_err(dev, "voltage max test error. tx=%d rx=%d value=%d.\n", i+1, j+1, voltage[i][j]);
				totalfailed++;
				 if ((voltage_max[i][j] - voltage[i][j]) < maxdelta) {
					printk("update %d,%d %d\n", i, j, maxdelta);
					voltage_max[i][j] = voltage[i][j];
					maxchanged = true;
				 }
			}
		}
		buf[num_read_chars-1] = '\n';
	}
#if 0
	//step: search "V" shape in tx and rx
	if(flag){
		//search "V" in tx
		for(i=0; i<FT5x06_NUM_TX; i++){
			for(j=0; j<FT5x06_NUM_RX; j++){
				for(k=(j-1); k>=0; k--){
					if(voltage[i][j]voltage[i][k]
					}
				}
			}
		}
#endif
	if(flag)
		g_ftsvoltageresult = 0;

	if (totalfailed < maxnum) {
		if (maxchanged) {
			printk("update voltage max file %d < %d\n", totalfailed, maxnum);
			writeTXRXdatatofile(filepath_max, voltage_max);
		}

		if (minchanged) {
			printk("update voltage min file %d < %d\n", totalfailed, maxnum);
			writeTXRXdatatofile(filepath_min, voltage_min);
		}
	}
	
error_restore_voltage:
	retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DRIVER_VOLTAGE, sizeof(u8), &original_voltage);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not write the value of the Device Voltage register.\n", __FUNCTION__);
    }

error_retrun:
	retval = ft5x06_exit_factory_mode(ts);
		if (0 != retval)
		{
			 dev_err(dev, "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n", __FUNCTION__);
		}
		mutex_unlock(&ts->device_mode_mutex);

	return num_read_chars;
}

static ssize_t ft5x06_ftscrosstalkodd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);

	ssize_t num_read_chars = 0;

	int crosstalk_file[FT5x06_NUM_TX][FT5x06_NUM_RX];
	char filepath_crosstalk[64];
	//int	 retval  = 0;
	int 	flag = 1;
	//int testresult = LWJ_FAILED;
	int j;
	const u8 rx_offset = 1;//((FT5x06_CROSSTALK_TEST_TYPE_ODD == ts->crosstalk_test_type) ? 1 : 0);
	//ssize_t  num_read_chars = 0;
	u8	num_processed_rx_cac_values = 0x00;
	u8	i		 = 0x00;
	u8	regval	 = 0x00;
	int retval	 = 0;
	u8 testmode = 0x00;
	u8 m = 1;

	u8	original_rx_cac[FT5x06_NUM_RX];
#ifdef CONFIG_MACH_OMAP_HUMMINGBIRD
#else
	u8	original_tx_cac[FT5x06_NUM_TX];
#endif

	int maxnum = 10;
	int maxdelta = 10;
	int totalfailed = 0;
	char cfgfilepath[64];
	bool limitchanged = false;
	
	sprintf(cfgfilepath, "/config/limitconfig_crosstalk_odd_%s.txt", szstation);
	readCfg(cfgfilepath, &maxnum, &maxdelta);
	
	g_ftscrosstalkoddresult = 1;
	sprintf(filepath_crosstalk, "/config/crosstalk_odd_%s.txt", szstation);
	memset(original_rx_cac, 0x00, FT5x06_NUM_RX);

	mutex_lock(&ts->device_mode_mutex);
	retval = ft5x06_enter_factory_mode(ts);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n", __FUNCTION__);
		goto error_return;
	}

#ifdef CONFIG_MACH_OMAP_HUMMINGBIRD
#else
	// adjust the TX CAC for FT56
	m = 1;
	for (i = 0; i < FT5x06_NUM_TX; i ++) {
		if (i >= FT5606_TEST_MODE_RX) {
			if (i >= FT5606_TEST_MODE_RX*2)
				m = 3;
			else if (i >= FT5606_TEST_MODE_RX*3)
				m = 4;
			else
				m = 2;
			testmode = ft5606_factory_mode[m-1] << 4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (0 != retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
				goto error_return;
			}
		}

		retval = i2c_smbus_read_i2c_block_data(client, (FT5x06_FMREG_TX_0_CAC + i - FT5606_TEST_MODE_RX*(m-1)), sizeof(u8), &regval);
		if (0 > retval)
		{
			dev_err(dev, "%s() - ERROR: Could not read from the RX%u_CAC register.\n", __FUNCTION__, i);
			goto error_restore_cac_values;
		}
		original_tx_cac[i] = regval;

		if (i >= FT5606_TEST_MODE_RX) {
			testmode = FACTORY_MODE0 <<4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (retval != 0) {
				dev_err(dev, "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n", __FUNCTION__, i);
				goto error_restore_cac_values;
			}
		}
	}

	m = 1;
	for (i = 0; i < FT5x06_NUM_TX; i ++) {
		if (i >= FT5606_TEST_MODE_RX) {
			if (i >= FT5606_TEST_MODE_RX*2)
				m = 3;
			else if (i >= FT5606_TEST_MODE_RX*3)
				m = 4;
			else
				m = 2;
			testmode = ft5606_factory_mode[m-1] << 4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (0 != retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
				goto error_return;
			}
		}

		regval = original_tx_cac[i] -40;
		retval = i2c_smbus_write_i2c_block_data(client, (FT5x06_FMREG_TX_0_CAC + i - FT5606_TEST_MODE_RX*(m-1)), sizeof(u8), &regval);
		if (0 > retval)
		{
			dev_err(dev, "%s() - ERROR: Could not read from the RX%u_CAC register.\n", __FUNCTION__, i);
			goto error_restore_cac_values;
		}

		if (i >= FT5606_TEST_MODE_RX) {
			testmode = FACTORY_MODE0 <<4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (retval != 0) {
				dev_err(dev, "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n", __FUNCTION__, i);
				goto error_restore_cac_values;
			}
		}
	}
#endif

	for (i = 0; i < 4; i++) {
		msleep(100);
		//step 1: get rawbase
		retval = ft5x06_read_rawdata(client, rawbase_before);
		if (0 != retval)
		{
			dev_err(dev, "%s() - ERROR: Could not put the device in Factory Mode.\n", __FUNCTION__);
			goto error_restore_cac_values;
		}
	}

	/* Preserve the original values of the even or odd RXn_CAC registers and then set them to 0x00. */
	dev_dbg(dev, "%s() - Modifying the RXn_CAC register values...\n", __FUNCTION__);
	//step 2: get crosstalk
	m = 1;
	for (i = rx_offset; i < FT5x06_NUM_RX; i += 2)
	{
		if (i >= FT5606_TEST_MODE_RX) {
			if (i >= FT5606_TEST_MODE_RX*2)
				m = 3;
			else if (i >= FT5606_TEST_MODE_RX*3)
				m = 4;
			else
				m = 2;
			testmode = ft5606_factory_mode[m-1] << 4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (0 != retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
				goto error_return;
			}
		}
		retval = i2c_smbus_read_i2c_block_data(client, (FT5x06_FMREG_RX_0_CAC + i - FT5606_TEST_MODE_RX*(m-1)), sizeof(u8), &regval);
		if (0 > retval)
		{
			dev_err(dev, "%s() - ERROR: Could not read from the RX%u_CAC register.\n", __FUNCTION__, i);
			goto error_restore_cac_values;
		}

		original_rx_cac[num_processed_rx_cac_values] = regval;
		num_processed_rx_cac_values++;
#ifdef CONFIG_MACH_OMAP_HUMMINGBIRD
		regval = 0x00;
#else
		regval = 220;
#endif
		retval = i2c_smbus_write_i2c_block_data(client, (FT5x06_FMREG_RX_0_CAC + i - FT5606_TEST_MODE_RX*(m-1)), sizeof(u8), &regval);
		if (0 != retval)
		{
			dev_err(dev, "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n", __FUNCTION__, i);
			goto error_restore_cac_values;
		}
		if (i >= FT5606_TEST_MODE_RX) {
			testmode = FACTORY_MODE0 <<4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (retval != 0) {
				dev_err(dev, "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n", __FUNCTION__, i);
				goto error_restore_cac_values;
			}
		}
	}

	for (i = 0; i < 4; i++)
	{
		msleep(100);
		retval = ft5x06_read_rawdata(client, rawbase_after);
		if (0 != retval)
		{
			dev_err(dev, "%s() - ERROR: Could not read Raw Data from the Touch Panel.\n", __FUNCTION__);
			goto error_restore_cac_values;
		}
	}

	memset(crosstalk_file, 0, sizeof(crosstalk_file));
	//step 3: differ = rawbase - crosstalk and compare to crosstalk that from file
	if(readTXRXdatafromfile(filepath_crosstalk, crosstalk_file)<0){
		dev_err(dev, "%s() - ERROR: read voltage from file error\n", __FUNCTION__);
		goto error_restore_cac_values;
	}
	for(i=0; i<FT5x06_NUM_TX; i++){
		for(j=0; j<FT5x06_NUM_RX; j++){
			crosstalk[i][j] = abs(rawbase_before[i][j] - rawbase_after[i][j]);
			num_read_chars += sprintf(&(buf[num_read_chars]), "%02d ", crosstalk[i][j]);

			if (0 == ( j % 2)) {// odd should be < limit
				if(crosstalk[i][j] > crosstalk_file[i][j]){
					flag = 0;
					dev_err(dev, "crosstalk odd tx=%d rx=%d value=%d\n", i+1, j+1, crosstalk[i][j]);
					if ((crosstalk[i][j] - crosstalk_file[i][j]) < maxdelta) {
						printk("update %d,%d %d\n", i, j, maxdelta);
						crosstalk_file[i][j] = crosstalk[i][j];
						limitchanged = true;
					}
					totalfailed++;
				}
			} else { // even should be > limit
				if(crosstalk[i][j] < crosstalk_file[i][j]){
					flag = 0;
					dev_err(dev, "crosstalk even tx=%d rx=%d value=%d\n", i+1, j+1, crosstalk[i][j]);
					if ((crosstalk_file[i][j] - crosstalk[i][j]) < maxdelta) {
						printk("update %d,%d %d\n", i, j, maxdelta);
						crosstalk_file[i][j] = crosstalk[i][j];
						limitchanged = true;
					}
					totalfailed++;
				}
			}
		}
		buf[num_read_chars-1] = '\n';
	}
	if(flag)
		g_ftscrosstalkoddresult = 0;
	
	if (totalfailed < maxnum) {
		if (limitchanged) {
			printk("update max files %d < %d\n", totalfailed, maxnum);
			writeTXRXdatatofile(filepath_crosstalk, crosstalk_file);
		}
	}
error_restore_cac_values:
	dev_dbg(dev, "%s() - Restoring the RXn_CAC register values...\n", __FUNCTION__);

	m = 1;
	for (i = 0; i < num_processed_rx_cac_values; i++)
	{
		if (((2 * i) + rx_offset) >= FT5606_TEST_MODE_RX) {
			if (i >= FT5606_TEST_MODE_RX*2)
				m = 3;
			else if (i >= FT5606_TEST_MODE_RX*3)
				m = 4;
			else
				m = 2;
			testmode = ft5606_factory_mode[m-1] << 4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (0 != retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
				goto error_return;
			}
		}
		retval = i2c_smbus_write_i2c_block_data(client, (FT5x06_FMREG_RX_0_CAC + (2 * i) + rx_offset - (FT5606_TEST_MODE_RX*(m-1))), sizeof(u8), &(original_rx_cac[i]) );
		if (0 != retval)
		{
			dev_err(dev, "%s() - ERROR: Could not restore the original value of the RX%u_CAC register.\n", __FUNCTION__, (2 * i) + rx_offset);
			//goto error_restore_mode;
		}
		testmode = FACTORY_MODE0 <<4;
		retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
		if (retval != 0) {
			dev_err(dev, "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n", __FUNCTION__, i);
		}
	}

#ifdef CONFIG_MACH_OMAP_HUMMINGBIRD
#else
	// restore the org TX CAP
	m = 1;
	for (i = 0; i < FT5x06_NUM_TX; i ++) {
		if (i >= FT5606_TEST_MODE_RX) {
			if (i >= FT5606_TEST_MODE_RX*2)
				m = 3;
			else if (i >= FT5606_TEST_MODE_RX*3)
				m = 4;
			else
				m = 2;
			testmode = ft5606_factory_mode[m-1] << 4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (0 != retval)
			{
				DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not initiate scan for raw data...\n", dev_name(&(client->dev)), __func__);
				goto error_return;
			}
		}

		regval = original_tx_cac[i];
		retval = i2c_smbus_write_i2c_block_data(client, (FT5x06_FMREG_TX_0_CAC + i - FT5606_TEST_MODE_RX*(m-1)), sizeof(u8), &regval);
		if (0 > retval)
		{
			dev_err(dev, "%s() - ERROR: Could not read from the RX%u_CAC register.\n", __FUNCTION__, i);
			goto error_restore_cac_values;
		}

		if (i >= FT5606_TEST_MODE_RX) {
			testmode = FACTORY_MODE0 <<4;
			retval = i2c_smbus_write_i2c_block_data(client, FT5x06_FMREG_DEVICE_MODE, sizeof(u8), &testmode);
			if (retval != 0) {
				dev_err(dev, "%s() - ERROR: Could not write 0x00 to the RX%u_CAC register.\n", __FUNCTION__, i);
				goto error_restore_cac_values;
			}
		}
	}
#endif
error_return:
	retval = ft5x06_exit_factory_mode(ts);
		if (0 != retval)
		{
			 dev_err(dev, "%s() - ERROR: Could not put the Touch Panel in Factory Mode.\n", __FUNCTION__);
		}
		mutex_unlock(&ts->device_mode_mutex);

	return num_read_chars;
}

//get correlation coefficient
static int getcorrelationcoefficient(int x[], int y[], int len)
{
/*
	double c = 0.0;
	int sum_xy = 0;
	int sum_x = 0;
	int sum_y = 0;
	int sum_xx = 0;
	int sum_yy = 0;
*/
	int similarity = 0;
	int sum = 0;int avg = 0;
	int delta[len];int min = 10000;int max = 0;
	int maxindex,minindex;
	int sum_delta = 0;
	int threshold = 20;
	int tmp = 0;
	int i;
	for(i=0; i<len; i++){
		delta[i] = x[i] - y[i];
		sum += delta[i];
		/*
		sum_x += x[i];
		sum_y += y[i];
		sum_xy += x[i] * y[i];
		sum_xx += x[i] * x[i];
		sum_yy += y[i] * y[i];
		*/
		}
	avg = sum/len;
	for(i=0; i<len; i++){
		tmp = delta[i]-avg;
		if(tmp<0)
			tmp = 0-tmp;
		sum_delta += tmp;//����ֵ
		delta[i] -= avg;
		if(delta[i] > max){
			max = delta[i];
			maxindex = i;
			}
		if(delta[i] < min){
			min = delta[i];
			minindex = i;
			}
		}
	//printk("%s:sum_delta=%d max_delta=%d min_delta=%d\n", __FUNCTION__, sum_delta, max, min);
	if(sum_delta > threshold)
		similarity = 0;
	else
		similarity = 1;
	//c = (len*sum_xy - sum_x*sum_y)/sqrt(len*sum_xx - sum_x*sum_x*sqrt(len*sum_yy - sum_y*sum_y));

	return similarity;
}
static ssize_t fx5x06_analyse_short_open(char * output_buffer, ssize_t * p_num_read_chars)
{
	//int mean_crosstalk_TX[FT5x06_NUM_TX-2];
	int mean_crosstalk_RX[FT5x06_NUM_RX-2];
	int mean_voltage_TX[FT5x06_NUM_TX];
	int mean_voltage_RX[FT5x06_NUM_RX];
	//int delta_voltage_TX[FT5x06_NUM_RX];
	//int avg_voltage_TX[FT5x06_NUM_RX];
	int max_crosstalk_RX=0, min_crosstalk_RX=10000;

	int flag_voltage_TX[FT5x06_NUM_TX];
	int flag_voltage_RX[FT5x06_NUM_RX];
	int abnormalindex = 0;int abnormal = 0;// 0-normal  1-abnormal
	//int abnormalindex2 = 0;
	int sum_crosstalk_RX;
	int sum_voltage_TX, sum_voltage_RX;
	int i, j, m,n,k;int rxflag=0;int rxopen1=0, rxopen2=0,txopen1=0, txopen2=0;
	//int avg;
	int tmp;
	int flag_small[FT5x06_NUM_TX][FT5x06_NUM_RX];
	int very_small_num = 0;int very_small_tx=-1,very_small_rx=-1;
	int small_tx = -1,small_rx=-1;
	int min_rx = 0;int min_rx_index = 0;
	int small_num = 0;//int min_small = 10000;int min_small_index = 0;
	//int flag_small[FT5x06_NUM_TX][FT5x06_NUM_RX];

	for(i=0; i<FT5x06_NUM_TX; i++){
		sum_voltage_TX = 0;
		for(j=0; j<FT5x06_NUM_RX; j++){
			sum_voltage_TX += voltage[i][j];
			flag_small[i][j] = 0;
			if(voltage[i][j] <= threshold_very_small){
				flag_small[i][j] = 1;
				very_small_num++;
				if(very_small_tx<0){
					very_small_tx = i;
					very_small_rx = j;
					}
				}
			if(voltage[i][j]>threshold_very_small &&
				voltage[i][j] <= threshold_small){
				flag_small[i][j] = 2;
				small_num++;
				if(small_tx<0){
					small_tx = i;
					small_rx = j;
					}
				}
			}
		mean_voltage_TX[i] = sum_voltage_TX/FT5x06_NUM_RX;
		if(mean_voltage_TX[i] < threshold_mean_Tx)
			flag_voltage_TX[i] = 1;
		else
			flag_voltage_TX[i] = 0;
		}
	for(i=0; i<FT5x06_NUM_RX; i++){
		sum_voltage_RX = 0;
		for(j=0; j<FT5x06_NUM_TX; j++){
			sum_voltage_RX += voltage[j][i];
			}
		mean_voltage_RX[i] = sum_voltage_RX / FT5x06_NUM_TX;
		if(mean_voltage_RX[i] < threshold_mean_Rx)
			flag_voltage_RX[i] = 1;
		else
			flag_voltage_RX[i] = 0;
		}

	for(i=1; i<FT5x06_NUM_RX-1; i++){
		sum_crosstalk_RX = 0;
		for(j=1; j<FT5x06_NUM_TX-1; j++){
			sum_crosstalk_RX += crosstalk[j][i];
			}
		mean_crosstalk_RX[i-1] = sum_crosstalk_RX / (FT5x06_NUM_TX-2);
		}
#if 0
	printk("\nflag_voltage_RX:\n");
	for(i=0; i<FT5x06_NUM_RX; i++)
		printk("%d ", flag_voltage_RX[i]);
	printk("\nflag_voltage_TX:\n");
	for(i=0; i<FT5x06_NUM_TX; i++)
		printk("%d ", flag_voltage_TX[i]);
	printk("\n");
	printk("\nmean_voltage_RX:\n");
	for(i=0; i<FT5x06_NUM_RX; i++)
		printk("%d ", mean_voltage_RX[i]);
	printk("\nmean_voltage_TX:\n");
	for(i=0; i<FT5x06_NUM_TX; i++)
		printk("%d ", mean_voltage_TX[i]);
	printk("\n");
#endif

	//TX line
	abnormalindex = -1;abnormal = 0;
	for(i=0; i<FT5x06_NUM_TX; i++){
		if(flag_voltage_TX[i]){
			abnormal++;
			if(abnormalindex<0)
				abnormalindex = i;
			}
		else{
			if(abnormal>0)
				break;
			}
		}
	sum_voltage_RX = 0;
	if(abnormal==1){
		rxflag = 1;
		/*
		for(i=0; i<FT5x06_NUM_RX; i++){
			sum_voltage_RX += voltage[abnormalindex][i];
			avg = sum_crosstalk_RX/(i+1);
			if(voltage[abnormalindex][i] > (avg*N)){
				//tx open
				// *p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "open. tx=%d\n", (abnormalindex+1));
				printk("line;%d open. tx=%d rx=%d\n", __LINE__, (abnormalindex+1), (i+1));
				return 0;
				}
			}*/
		for(i=0; i<FT5x06_NUM_RX; i++){
			if(voltage[abnormalindex][i]>threshold_TX_OPEN)
			{
				rxflag=0;
				break;
			}
		}
		if(rxflag){
			//�ж������Ƿ���·
			#ifdef LWJ_DEBUG
			*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "tx=%d single open at bonding point. \n", (abnormalindex+1));
			#else
			printk("line;%d tx=%d single open at bonding point. \n", __LINE__, (abnormalindex+1));
			#endif
			return 0;
		}
		for(i=FT5x06_NUM_RX; i>=0; i--){
			if(voltage[abnormalindex][i]<threshold_Rx){
				rxopen2 = i;
				break;
			}
			}
			#ifdef LWJ_DEBUG
			*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "tx=%d single open at rx=%d\n", (abnormalindex+1), (rxopen2+1));
			#else
			printk("line;%d tx=%d single open at rx=%d\n\n", __LINE__, (abnormalindex+1), (rxopen2+1));
			#endif
			return 0;
		}
	else if(abnormal >= 2){
		rxflag=0;
		#if 0
		for(i=abnormalindex+1; i<(abnormalindex+2); i++){
			for(j=0; j<FT5x06_NUM_RX; j++){
				if(voltage[i][j] > threshold_Rx){
					printk("%d %d > threshold_Rx\n", __LINE__, voltage[i][j]);
					rxflag = 1;
					abnormalindex=i;
					break;
					}
				}
			if(rxflag)
				break;
			}
		#endif
			if(flag_voltage_TX[abnormalindex])
				rxflag = 1;
			if(rxflag==0){
				for(i=(FT5x06_NUM_RX-1); i>=0; i--){
					if(voltage[abnormalindex][i]<threshold_Rx)
						rxopen2 = i;
				}
				#ifdef LWJ_DEBUG
				*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "tx=%d single open at rx=%d\n", (abnormalindex+1), (rxopen2+1));
				#else
				printk("line;%d tx=%d single open at rx=%d\n\n", __LINE__, (abnormalindex+1), (rxopen2+1));
				#endif
				return 0;
				}
			else{
			if(getcorrelationcoefficient(voltage[abnormalindex], voltage[abnormalindex+1], FT5x06_NUM_RX)){
				#ifdef LWJ_DEBUG
				*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "tx=%d and tx=%d both single open at bonding point.\n", (abnormalindex+1),(abnormalindex+2));
				#else
				printk("line;%d similarity. tx=%d and tx=%d both single open at bonding point. \n", __LINE__, (abnormalindex+1),(abnormalindex+2));
				#endif
				}
			else{
				#ifdef LWJ_DEBUG
				*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "tx=%d and tx=%d short. \n", (abnormalindex+1), (abnormalindex+2));
				#else
				printk("line;%d no similarity. tx=%d and tx=%d short. \n", __LINE__, (abnormalindex+1), (abnormalindex+2));
				#endif
				}
			return 0;
			}
		}
	//���з���(RX)
	abnormalindex = -1;abnormal = 0;
	for(i=0; i<FT5x06_NUM_RX; i++){
		if(flag_voltage_RX[i]){
			abnormal++;
			if(abnormalindex<0)
				abnormalindex = i;
			}
		else{
			if(abnormal>0)
				break;
			}
		}
	if(abnormal==1){
		rxflag = 1;
		for(i=0; i<FT5x06_NUM_TX; i++){
			if(voltage[i][abnormalindex] < threshold_RX_Short)
			{
				rxflag = 0;
				break;
			}
		}
		if(rxflag){
			//���е㶼С��threshold_RX_Short,ֱ���жϿ�·
			#ifdef LWJ_DEBUG
			*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d single open at bonding point. \n", (abnormalindex+1));
			#else
			printk("line;%d rx=%d single open at bonding point.\n", __LINE__, (abnormalindex+1));
			#endif
			return 0;
		}
		else{
			for(i=0; i<FT5x06_NUM_TX; i++){
				if(voltage[i][abnormalindex] < threshold_Rx){
					if(voltage[i][abnormalindex] > threshold_RX_Short){
						for(j=i; j<FT5x06_NUM_TX; j++){
							if(voltage[j][abnormalindex] > threshold_Rx){
								break;
							}
						}
						if(j==FT5x06_NUM_TX){
							#ifdef LWJ_DEBUG
							*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d double open at tx=%d and tx=%d\n", (abnormalindex+1), i+1, FT5x06_NUM_TX);
							#else
							printk("line;%d rx=%d double open at tx=%d and tx=%d\n", __LINE__, (abnormalindex+1), i+1, FT5x06_NUM_TX);
							#endif
							return 0;
						}
						else{
							#ifdef LWJ_DEBUG
							*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d double open at tx=%d and tx=%d\n", (abnormalindex+1), i+1, j+1);
							#else
							printk("line;%d rx=%d double open at tx=%d and tx=%d\n", __LINE__, (abnormalindex+1), i+1, j+1);
							#endif
							return 0;
						}
					}
					else{
						min_rx = 4000;
						min_rx_index = i;
						for(j=i; j<FT5x06_NUM_TX; j++){
							if(voltage[j][abnormalindex] > threshold_Rx){
								break;
							}
							if(voltage[j][abnormalindex] < min_rx){
								min_rx = voltage[j][abnormalindex];
								min_rx_index = j;
							}
						}
						#if 0
						if(j==FT5x06_NUM_TX){
							#ifdef LWJ_DEBUG
							*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "short. rx=%d tx=%d and tx=%d\n", (abnormalindex+1), i+1, FT5x06_NUM_TX);
							#else
							printk("line;%d rx=%d double short. rx=%d tx=%d and tx=%d\n", __LINE__, (abnormalindex+1), i+1, FT5x06_NUM_TX);
							#endif
							return 0;
						}
						else
						#endif
						{
							#ifdef LWJ_DEBUG
							*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d single open at tx=%d\n", (abnormalindex+1), min_rx_index+1);
							#else
							printk("line;%d rx=%d single open at tx=%d\n", __LINE__, (abnormalindex+1), min_rx_index+1);
							#endif
							return 0;
						}
					}
				}
			}
		}
			{
			//Ѱ�ҵ�һ��������һ��С����ֵΪ275��TX��
			//�ж�Ϊrx=? Tx=? And Tx=?
			/*if(flag_voltage_RX[abnormalindex])
				rxflag=1;
			if(rxflag)
				{
				for(j=0; j<FT5x06_NUM_TX; j++){
					if(voltage[j][abnormalindex] < threshold_Rx){
						txopen1 = j;
						break;
						}
					}
				for(j=(FT5x06_NUM_TX-1); j>=0; j--){
					if(voltage[j][abnormalindex] < threshold_Rx){
						txopen2 = j;
						break;
						}
					}
				if(txopen1 == txopen2){
					#ifdef LWJ_DEBUG
					*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "open. rx=%d tx=%d\n", (abnormalindex+1), (txopen1+1));
					#else
					printk("line;%d open. rx=%d tx=%d\n", __LINE__, (abnormalindex+1), (txopen1+1));
					#endif
					}
				else{
					#ifdef LWJ_DEBUG
					*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "open. rx=%d tx=%d and tx=%d\n", (abnormalindex+1), (txopen1+1), (txopen2+1));
					#else
					printk("line;%d open. rx=%d tx=%d and tx=%d\n", __LINE__, (abnormalindex+1), (txopen1+1), (txopen2+1));
					#endif
					}
				}
			else{
				#ifdef LWJ_DEBUG
				*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "short. rx=%d\n", (i+1));
				#else
				printk("line;%d short. rx=%d\n", __LINE__, (i+1));
				#endif
				}
			return 0;
			*/
		}
		}
	else if(abnormal>=2){
		for(i=abnormalindex+1; i<FT5x06_NUM_RX; i++){
			if(flag_voltage_RX[i]){
				if((i-abnormalindex)==1){
					#ifdef LWJ_DEBUG
					*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d and rx=%d short. \n", i, (i+1));
					#else
					printk("line:%d rx=%d and rx=%d short. \n",__LINE__,  i, (i+1));
					#endif
					return 0;
					}
				else
					abnormalindex = i;
				}
			}
		#ifdef LWJ_DEBUG
		*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d and rx=%d short. \n", i, (abnormalindex+1));
		#else
		printk("line=%d rx=%d and rx=%d short. \n",__LINE__,  i, (abnormalindex+1));
		#endif
		return 0;
		}

	//Ѱ���쳣�ĵ���
	if(very_small_num==1){
	// add by zhangjk: only when no small data can it be said that TX single open
	      if (small_num == 0)
	      {
			#ifdef LWJ_DEBUG
			*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "tx=%d single open at rx=%d.  \n", (very_small_tx+1), (very_small_rx+1));
			#else
			printk("line=%d tx=%d single open at rx=%d. \n", __LINE__, (very_small_tx+1), (very_small_rx+1));
			#endif
			return 0;
		}
		else // small_num > 0
		{
			tmp = 1;
			for(i=0; i<FT5x06_NUM_TX; i++)
			{
				for(j=0; j<FT5x06_NUM_RX; j++)
				{
					if(flag_small[i][j]==1)
					{
						if(j<(FT5x06_NUM_RX-1))
						{
							//if(flag_small[i][j+1]==1)
							{
								//tmp++;
								for(m=j+1; m<FT5x06_NUM_RX; m++)
								{
									if(flag_small[i][m]==2)	// 2 is for "small", 1 is for "very small"
									{
										tmp++;
									}
								}
								#ifdef LWJ_DEBUG
								*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "tx=%d single open at rx=%d.\n", (i+1),  (j+tmp+1));
								#else
								printk("line;%d  tx=%d single open at rx=%d. \n", __LINE__, (i+1), (j+tmp+1));
								#endif
								return 0;
							}
						}
						if(i<(FT5x06_NUM_TX-1))
						{
							//if(flag_small[i+1][j]==1)	// comment this because there is only one very small point
							{
								for(m=i+1; m<FT5x06_NUM_TX; m++)
								{
									if(flag_small[m][j]==2)	// 2 is for "small", 1 is for "very small"
									{
										tmp++;
									}
								}
								#ifdef LWJ_DEBUG
								*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d double open at tx=%d and tx=%d\n", (j+1), (i+1), (i+tmp+1));
								#else
								printk("line;%d  rx=%d double open at tx=%d and tx=%d\n", __LINE__, (j+1), (i+1), (i+tmp+1));
								#endif
								return 0;
							}
						}
					}
				}
			}
		}
	}
	else if(very_small_num >=2){
		tmp = 1;
		for(i=0; i<FT5x06_NUM_TX; i++){
			for(j=0; j<FT5x06_NUM_RX; j++){
				if(flag_small[i][j]==1){
					if(j<(FT5x06_NUM_RX-1)){
						if(flag_small[i][j+1]==1){
							//tmp++;
							for(m=j+1; m<FT5x06_NUM_RX; m++){
								if(flag_small[i][m]==1){
									tmp++;
									}
								}
							#ifdef LWJ_DEBUG
							*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "tx=%d single open at rx=%d.\n", (i+1),  (j+tmp+1));
							#else
							printk("line;%d  tx=%d single open at rx=%d. \n", __LINE__, (i+1), (j+tmp+1));
							#endif
							return 0;
							}
						}
					if(i<(FT5x06_NUM_TX-1)){
						if(flag_small[i+1][j]==1){
							for(m=i+1; m<FT5x06_NUM_TX; m++){
								if(flag_small[m][j]==1){
									tmp++;
									}
								}
							#ifdef LWJ_DEBUG
							*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d double open at tx=%d and tx=%d\n", (j+1), (i+1), (i+tmp+1));
							#else
							printk("line;%d  rx=%d double open at tx=%d and tx=%d\n", __LINE__, (j+1), (i+1), (i+tmp+1));
							#endif
							return 0;
							}
						}
					}
				}
			}
		}
	else if(small_num==1)
	{
		#ifdef LWJ_DEBUG
		*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d single open at tx=%d.  \n", (small_rx+1), (small_tx+1));
		#else
		printk("line=%d rx=%d single open at tx=%d.  \n very_small=, small=", __LINE__, (small_rx+1), (small_tx+1), very_small_num, small_num);
		#endif
		return 0;
	}
	else
	{
		if (small_num > 1)
		{	// search for the min of differ data
			tmp = voltage[0][0];
			m=0;
			n=0;
			for(i=0; i<FT5x06_NUM_TX; i++)
			{
				for(j=0; j<FT5x06_NUM_RX; j++)
				{
					if (voltage[i][j] < tmp)
					{
						m=i;
						n=j;
						tmp = voltage[i][j];
					}
				}
			}
			tmp = 0; 		// this is not single open
			for (i=0; i<FT5x06_NUM_TX; i++)		// |max-min| on that RX should be larger than a threshold for single open.
			{
				if ((voltage[i][n] - voltage[m][n]) > threshold_very_small)
				{
					tmp = 1;			// this is single open
					break;
				}
			}

			if (tmp == 1)
			{
				#ifdef LWJ_DEBUG
				*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "line;%d rx=%d single open at tx=%d. \n", __LINE__, (n+1), (m+1));
				#else
				printk("line;%d rx=%d single open at tx=%d. \n very_small=%d,small=%d\n", __LINE__, (n+1), (m+1), very_small_num, small_num);
				#endif
				return 0;
			}
		}
#if 0		//deleted by zhangjk: change the algorithm.
		if(small_num>1)
		{
			for(i=0; i<FT5x06_NUM_TX; i++)
			{
				for(j=0; j<FT5x06_NUM_RX; j++)
				{
					if(flag_small[i][j]==2)
					{
						small_num = 1;
						for(m=i; m<FT5x06_NUM_TX; m++)
						{
							if(flag_small[m][j] == 2)
							{
								/*
								if(flag_small[m][j] < min_small){
									min_small = flag_small[m][j];
									min_small_index = m;
									}*/
								if(flag_small[m][j]==2)
								{
									tmp++;
								}
								//small_num++;
							}
							else break;
						}
						if(small_num > 1)
						{
							#ifdef LWJ_DEBUG
							*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "rx=%d double open at tx=%d and tx=%d. \n", __LINE__, (j+1), (i+1), (i+tmp+1));
							#else
							printk("line;%d rx=%d double open at tx=%d and tx=%d. \n", __LINE__, (j+1), (i+1), (i+tmp+1));
							#endif
							return 0;
						}
					}
				}
			}
		}
#endif // 0
	}

	//No shielding���ж�
	for(i=0; i<FT5x06_NUM_RX-2; i++){
		if(mean_crosstalk_RX[i] < threshold_cross_talk_Rx){
			if(mean_crosstalk_RX[i] > max_crosstalk_RX)
				max_crosstalk_RX = mean_crosstalk_RX[i];
			if(mean_crosstalk_RX[i] < min_crosstalk_RX)
				min_crosstalk_RX = mean_crosstalk_RX[i];
			}
		}
#if 0
	printk("\nmean_crosstalk_RX:\n");
	for(i=0; i<FT5x06_NUM_RX-2; i++)
		printk("%d ", mean_crosstalk_RX[i]);
	printk("\nmax_crosstalk_RX=%d min_crosstalk_RX=%d \n", max_crosstalk_RX, min_crosstalk_RX);
#endif
	if(max_crosstalk_RX > threshold_min_Rx){
		if(min_crosstalk_RX*2 > max_crosstalk_RX){
			//No shielding
			#ifdef LWJ_DEBUG
			*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "Shielding open from GND. \n");
			#else
			printk("line;%d Shielding open from GND.\n", __LINE__);
			#endif
			return 0;
			}
		}

	#ifdef LWJ_DEBUG
	*p_num_read_chars = snprintf(output_buffer, PAGE_SIZE, "Unclear defect. \n");
	#else
	printk("Unclear defect. \n");
	#endif
	//*p_num_read_chars = 0;
	return 0;
}

static ssize_t ft5x06_ftssetstation_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
//	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
//	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);
	//int len=0;
	int i;
	memset(szstation, 0, sizeof(szstation));
	sprintf(szstation, "%s", buf);
	szstation[size-1]='\0';
	//strlwr(szstation);
	for(i=0; i<size-1; i++){
		if(szstation[i]>='A' && szstation[i]<='Z'){
			szstation[i] = szstation[i] - 'A' + 'a';
			}
		}
#if 1
	if(strcmp(szstation, "postbond") == 0)
		g_testfilenum = 20;
	else if(strcmp(szstation, "fpc") == 0)
		g_testfilenum = 50;
	if(strcmp(szstation, "coverglass")==0 ||
		strcmp(szstation, "final") == 0 ||
		strcmp(szstation,"ieciqc") == 0 ){
		threshold_mean_Tx = 100;
		threshold_mean_Rx = 50;
		threshold_very_small = 50;
		threshold_small = 275;
		threshold_cross_talk_Rx = 500;
		threshold_min_Rx = 100;
		threshold_Rx = 275;
		}
	else{
		threshold_mean_Tx = threshold_mean_Tx*4/5;
		threshold_mean_Rx = (int)(threshold_mean_Rx*4/5);
		threshold_very_small = (int)(threshold_very_small*4/5);
		threshold_small = (int)(threshold_small*4/5);
		threshold_cross_talk_Rx = (int)(threshold_cross_talk_Rx*4/5);
		threshold_min_Rx = (int)(threshold_min_Rx*4/5);
		threshold_Rx = (int)(threshold_Rx*4/5);

		}
	printk("threshold_mean_Tx=%d\n"
		"threshold_mean_Rx=%d\n"
		"threshold_very_small=%d\n"
		"threshold_small=%d\n"
		"threshold_cross_talk_Rx=%d\n"
		"threshold_min_Rx=%d\n"
		"threshold_Rx=%d\n",
		threshold_mean_Tx,
		threshold_mean_Rx,
		threshold_very_small,
		threshold_small,
		threshold_cross_talk_Rx,
		threshold_min_Rx,
		threshold_Rx);
#endif
	return size;
}

static ssize_t ft5x06_ftssetstation_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
//	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
	num_read_chars = snprintf(buf, PAGE_SIZE, "test station:%s\n", szstation);

	return num_read_chars;
}

static ssize_t ft5x06_ftstestresult_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
//	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;
#ifndef TEST_SIMULATION		// for the real test
	if(g_ftsbaselineresult==1 ||
		g_ftsvoltageresult == 1 ||
		g_ftscrosstalkoddresult == 1)
	fx5x06_analyse_short_open(buf, &num_read_chars);
	if(g_ftsfullpaneltest==1){
		if(g_ftsfullpanelresult==1){
			num_read_chars += snprintf(&(buf[num_read_chars]), PAGE_SIZE, "full panel test failed\n");
			}
		}
	g_ftsfullpaneltest = 0;
#else		// for a simulation test, using sim files of like voltage test data
	not used!
	int i=0; //int j,k;
	char testfilepath[64];
	for(i=0; i<=g_testfilenum; i++){
		memset(testfilepath, 0, sizeof(testfilepath));
		sprintf(testfilepath, "/var/test_%s_file/test_sample%02d", szstation, i);
		printk("\ntest file:%s\n", testfilepath);
		if(readTXRXdatafromfile(testfilepath, voltage)<0){
			printk("read file error\n");
			continue;
			}
		fx5x06_analyse_short_open(buf, &num_read_chars);
		}
	#if 0
	memset(testfilepath, 0, sizeof(testfilepath));
	sprintf(testfilepath, "/var/test_file/test_sample_crosstalk");
	printk("\ntest file:%s\n", testfilepath);
	if(readTXRXdatafromfile(testfilepath, crosstalk)<0){
		printk("read file error\n");
		}
	#if 0
	for(j=0; j<FT5x06_NUM_TX; j++){
			for(k=0; k<FT5x06_NUM_RX; k++){
				printk("%d ", crosstalk[j][k]);
				}
			printk("\n");
			}
	#endif
	memset(testfilepath, 0, sizeof(testfilepath));
	sprintf(testfilepath, "/var/test_file/test_sampleNoShielding");
	printk("\ntest file:%s\n", testfilepath);
	if(readTXRXdatafromfile(testfilepath, voltage)<0){
		printk("read file error\n");
		}
	#if 0
	for(j=0; j<FT5x06_NUM_TX; j++){
			for(k=0; k<FT5x06_NUM_RX; k++){
				printk("%d ", voltage[j][k]);
				}
			printk("\n");
			}
	#endif
	fx5x06_analyse_short_open(buf, &num_read_chars);
	#endif
#endif
	return num_read_chars;
}

static ssize_t ft5x06_ftsbaselineresult_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
//	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "%d", g_ftsbaselineresult);

	return num_read_chars;
}

static ssize_t ft5x06_ftsfullpanelresult_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
//	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "%d", g_ftsfullpanelresult);

	return num_read_chars;
}

static ssize_t ft5x06_ftsvoltageresult_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
//	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "%d", g_ftsvoltageresult);

	return num_read_chars;
}

static ssize_t ft5x06_ftscrosstalkoddresult_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
//	struct ft5x06	  *ts	  = (struct ft5x06 *)i2c_get_clientdata(client);
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "%d", g_ftscrosstalkoddresult);

	return num_read_chars;
}

static int ft5x06_GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "/var/firmware/%s", firmware_name);
	printk("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
		}
	if(IS_ERR(pfile)){
		printk("error occured while opening file %s.\n", filepath);
		return -1;
		}
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}
static int ft5x06_ReadFirmware(char * firmware_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;

	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "/var/firmware/%s", firmware_name);
	printk("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
		}
	if(IS_ERR(pfile)){
		printk("error occured while opening file %s.\n", filepath);
		return -1;
		}
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	//char * buf;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

static int ft5x06_ftsperform_fw_upgrade(struct device *dev)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);

    int  retval = 0;
    int  i      = 0;
    int  j      = 0;

    u32 num_packets = 0;
    u16 temp_val    = 0;
    u8  checksum    = 0;

    u8  temp_buffer[4];
    u8  packet_buffer[FT5x06_PACKET_LENGTH + 6];
	//const struct firmware * fw_blob = NULL;
    char fw_name[256];
    u8 * firmware_src_buffer = NULL;
	u32 firmware_size;
	u8 FWID = 0x00;

    dev_info(dev, "%s() - Initiating Firmware Update...\n", __FUNCTION__);

/*******************Get FW ID, if ID is 0x79, then panel belong to TPK ,or belong to WinTek******/
	temp_buffer[0] = FT5x06_FWID_REG;
	retval = i2c_master_send(ts->client, temp_buffer, 1);
	if (0 > retval)
	{
		printk(KERN_ERR "%s() - ERROR: Could not write the Get FW ID to the CTPM.\n", __FUNCTION__);
	}

	 retval = i2c_master_recv(ts->client,temp_buffer, 1);
	if (0 > retval)
	 {
		printk(KERN_ERR "%s() - ERROR: Could not read the FW ID from the CTPM.\n", __FUNCTION__);
	}
	else
		FWID = temp_buffer[0];
	pr_info("%s() - FW ID = 0x%0x\n",  __FUNCTION__, FWID);
/******************************************************************************/
    /*********************************************************************************************/
    dev_dbg(dev, "%s() - Step 1: Reset the CTPM\n", __FUNCTION__);

    retval = i2c_smbus_write_byte_data(ts->client, FT5x06_WMREG_RESET, FT5x06_CMD_RESET_CTPM_H);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not write the CTPM Reset command (high byte) to the CTPM.\n", __FUNCTION__);
        //goto error_reset;
    }

    msleep(50);

    retval = i2c_smbus_write_byte_data(ts->client, FT5x06_WMREG_RESET, FT5x06_CMD_RESET_CTPM_L);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not write the CTPM Reset command (low byte) to the CTPM.\n", __FUNCTION__);
        //goto error_reset;
    }

    msleep(30);

    /*********************************************************************************************/
    dev_dbg(dev, "%s() - Step 2: Put the CTPM in Firmware Upgrade mode\n", __FUNCTION__);

    temp_buffer[0] = 0x55;
    temp_buffer[1] = 0xAA;

    for (i = 0; ((i < 5) && (0 >= retval)); i++)
    {
        struct i2c_msg msg =    {
                                    .addr   = ts->client->addr,
                                    .flags  = 0,
                                    .len    = 2,
                                    .buf    = temp_buffer,
                                };
        msleep(5);

        retval = i2c_transfer(ts->client->adapter, &msg, 1);
    }

    if (0 >= retval)
    {
        dev_err(dev, "%s() - ERROR: Could not put the CTPM in Firmware Update mode.\n", __FUNCTION__);
        //goto error_reset;
    }

    /*********************************************************************************************/
    dev_dbg(dev, "%s() - Step 3: Read the CTPM ID\n", __FUNCTION__);

    temp_buffer[0] = FT5x06_CMD_GET_ID;
    temp_buffer[1] = FT5x06_CMD_GET_ID_P1;
    temp_buffer[2] = FT5x06_CMD_GET_ID_P2;
    temp_buffer[3] = FT5x06_CMD_GET_ID_P3;

    retval = i2c_master_send(ts->client, temp_buffer, 4);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not write the Get ID command to the CTPM.\n", __FUNCTION__);
        goto error_reset;
    }

    msleep(10);

    retval = i2c_master_recv(ts->client,temp_buffer, 2);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not read the ID from the CTPM.\n", __FUNCTION__);
        goto error_reset;
    }

    if ((FT5x06_CPTM_ID_COMPANY != temp_buffer[0]) || (FT5x06_CPTM_ID_PRODUCT != temp_buffer[1]))
    {
        dev_err(dev, "%s() - ERROR: Invalid CPTM ID. Expected 0x%02X%02X, got 0x%02X%02X.\n", __FUNCTION__, FT5x06_CPTM_ID_COMPANY, FT5x06_CPTM_ID_PRODUCT, temp_buffer[0], temp_buffer[1]);
        retval = FT5x06_ERR_INVALID_ID;
        goto error_reset;
    }
/****************check the panel that belong to TPK or WinTek******************************/
		temp_buffer[0] = FT5x06_UPGRADEVER_REG;
		retval = i2c_master_send(ts->client, temp_buffer, 1);
		if (0 > retval)
		{
			printk(KERN_ERR "%s() - ERROR: Could not write the Get Upgrad Version to the CTPM.\n", __FUNCTION__);
			goto error_reset;
		}

		 retval = i2c_master_recv(ts->client,temp_buffer, 1);
		if (0 > retval)
		 {
			printk(KERN_ERR "%s() - ERROR: Could not read the Upgrad Version from the CTPM.\n", __FUNCTION__);
			goto error_reset;
		}
		if(FT5x06_TPK_UPGRADEVER1==temp_buffer[0])
		{
			//TPK's Panel and set the path
			//strncpy(fw_name, buf, size - 1);
			sprintf(fw_name, "%s", "FTS0019U700_TPK_app.bin");
			//fw_name[size - 1] = '\0';
		}
		else if(FT5x06_WINTEK_UPGRADEVER ==temp_buffer[0])
		{
			//WinTek's Panel and set the path
			//strncpy(fw_name, buf, size - 1);
			sprintf(fw_name, "%s", "FTS0019U700_WinTek_app.bin");
			//fw_name[size - 1] = '\0';
		}
		else
		{
			//printk(KERN_INFO "%s() - Upgrade version i.\n", __FUNCTION__);
			if(FT5x06_NONE_UPGRADEVER == temp_buffer[0])
			{
				//judge panel by FW ID
				if(FT5x06_TPK_FWID == FWID)
				{
					//TPK's Panel
					sprintf(fw_name, "%s", "FTS0019U700_TPK_app.bin");
				}
				else if(FT5x06_WINTEK_FWID == FWID)
				{
					//WinTek's Panel
					sprintf(fw_name, "%s", "FTS0019U700_WinTek_app.bin");
				}
				else
				{
					//default
					printk(KERN_INFO "%s() -unknown FW ID and Default panel is TPK.\n", __FUNCTION__);
					sprintf(fw_name, "%s", "FTS0019U700_TPK_app.bin");
				}
			}
			else
			{
				//default
				printk(KERN_INFO "%s() -unknown upgrade version and Default panel is TPK.\n", __FUNCTION__);
				sprintf(fw_name, "%s", "FTS0019U700_TPK_app.bin");
			}
			//goto error_reset;
		}
		printk(KERN_DEBUG "%s() - Processing input file: '%s'\n", __FUNCTION__, fw_name);
		//Get FW data
		//if (request_firmware(&fw_blob, fw_name, dev))
		firmware_size = ft5x06_GetFirmwareSize(fw_name);
		if(firmware_size>0)
		{
			firmware_src_buffer=(unsigned char *) kmalloc(firmware_size+1,GFP_ATOMIC);
			if(ft5x06_ReadFirmware(fw_name, firmware_src_buffer))
			{
				printk(KERN_ERR "%s() - ERROR: request_firmware failed\n", __FUNCTION__);
				goto error_reset;
			}
		}
		else
		{
			printk(KERN_ERR "%s() - ERROR: read firmware size\n", __FUNCTION__);
			goto error_reset;
		}
		printk(KERN_INFO "%s() - request_firmware sucessful sizelen=%d\n", __FUNCTION__, firmware_size);
/********************************************************/

    /*********************************************************************************************/
    printk(KERN_INFO "%s() - Step 4: Erase the old CTPM firmware\n", __FUNCTION__);

    temp_buffer[0] = FT5x06_CMD_ERASE_FW;

    retval = i2c_master_send(ts->client, temp_buffer, 1);
    if (0 > retval)
    {
       printk(KERN_INFO "%s() - ERROR: Could not write the Erase Firmware command to the CTPM.\n", __FUNCTION__);
        goto error_reset;
    }

    msleep(1500);

    /*********************************************************************************************/
    printk(KERN_INFO "%s() - Step 5: Write the new CTPM firmware to CTPM flash\n", __FUNCTION__);

    /* We write everything but the last 8 bytes in packets.
     * The first 6 of the last 8 bytes will be written in the footer.
     * The final 2 bytes (which seem to be always 0xFF and 0x00) don't get written.
     */
    firmware_size = firmware_size - 8;
    num_packets   = (firmware_size) / FT5x06_PACKET_LENGTH;

    packet_buffer[0] = 0xBF;
    packet_buffer[1] = 0x00;

    /* Write whole packets */
    for (i = 0; i < num_packets; i++)
    {
        /* Target offset */
        temp_val = i * FT5x06_PACKET_LENGTH;

        packet_buffer[2] = (u8)(0x00FF & (temp_val >> 8));
        packet_buffer[3] = (u8)(0x00FF & (temp_val));

        /* Num bytes following header */
        temp_val = FT5x06_PACKET_LENGTH;

        packet_buffer[4] = (u8)(0x00FF & (temp_val >> 8));
        packet_buffer[5] = (u8)(0x00FF & (temp_val));
	//printk(KERN_INFO "send %d bytes to FW\n", i*FT5x06_PACKET_LENGTH);
        for (j = 0; j < FT5x06_PACKET_LENGTH; j++)
        {
            /* Process byte j of packet i... */
            packet_buffer[6 + j] = firmware_src_buffer[(i * FT5x06_PACKET_LENGTH) + j];
            checksum ^= packet_buffer[6 + j];
        }

        retval = i2c_master_send(ts->client, packet_buffer, FT5x06_PACKET_LENGTH + 6);
        if (0 > retval)
        {
            dev_err(dev, "%s() - ERROR: Could not write packet %u of %u to the CTPM.\n", __FUNCTION__, i, num_packets);
            goto error_reset;
        }

        msleep(20);

        if (0 == ((i * FT5x06_PACKET_LENGTH) % 1024))
        {
            printk(KERN_INFO "%s() - Uploaded %6d of %6u bytes.\n", __FUNCTION__, (i * FT5x06_PACKET_LENGTH), firmware_size);
        }
    }

    printk(KERN_INFO "%s() - Uploaded %6d of %6u bytes.\n", __FUNCTION__, (i * FT5x06_PACKET_LENGTH), firmware_size);

    /* Write a partial packet if necessary */
    if (0 != (firmware_size % FT5x06_PACKET_LENGTH))
    {
        /* Target offset */
        temp_val = num_packets * FT5x06_PACKET_LENGTH;

        packet_buffer[2] = (u8)(0x00FF & (temp_val >> 8));
        packet_buffer[3] = (u8)(0x00FF & (temp_val));

        /* Num bytes following header */
        temp_val = (firmware_size % FT5x06_PACKET_LENGTH);

        packet_buffer[4] = (u8)(0x00FF & (temp_val >> 8));
        packet_buffer[5] = (u8)(0x00FF & (temp_val));

        for (j = 0; j < temp_val; j++)
        {
            packet_buffer[6 + j] = firmware_src_buffer[(num_packets * FT5x06_PACKET_LENGTH) + j];
            checksum ^= packet_buffer[6 + j];
        }

        retval = i2c_master_send(ts->client, packet_buffer, temp_val + 6);
        if (0 > retval)
        {
            dev_err(dev, "%s() - ERROR: Could not write partial packet to the CTPM.\n", __FUNCTION__);
            goto error_reset;
        }

        msleep(20);
    }

    printk(KERN_INFO "%s() - Uploaded %6d of %6u bytes.\n", __FUNCTION__, (i * FT5x06_PACKET_LENGTH) + temp_val, firmware_size);

    /* Write the firmware footer */
    for (i = 0; i < 6; i++)
    {
        packet_buffer[2] = 0x6F;
        packet_buffer[3] = 0xFA + i;
        packet_buffer[4] = 0x00;
        packet_buffer[5] = 0x01;

        packet_buffer[6] = firmware_src_buffer[firmware_size + i];
        checksum ^= packet_buffer[6];

        retval = i2c_master_send(ts->client, packet_buffer, 7);
        if (0 > retval)
        {
            dev_err(dev, "%s() - ERROR: Could not write partial packet to the CTPM.\n", __FUNCTION__);
            goto error_reset;
        }

        msleep(20);
    }

    /*********************************************************************************************/
    printk(KERN_INFO "%s() - Step 6: Checksum verification\n", __FUNCTION__);

    temp_buffer[0] = FT5x06_CMD_GET_CHECKSUM;

    retval = i2c_master_send(ts->client, temp_buffer, 1);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not write the Get Checksum command to the CTPM.\n", __FUNCTION__);
        goto error_reset;
    }

    msleep(10);

    retval = i2c_master_recv(ts->client,temp_buffer, 1);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not read the Checksum from the CTPM.\n", __FUNCTION__);
        goto error_reset;
    }

    if (checksum != temp_buffer[0])
    {
        dev_err(dev, "%s() - ERROR: Checksum (0x%02X) did not match calculated value (0x%02X).\n", __FUNCTION__, temp_buffer[0], checksum);
        retval = FT5x06_ERR_INVALID_CHECKSUM;
        goto error_reset;
    }

    /*********************************************************************************************/
    printk(KERN_INFO "%s() - Step 7: Reset the CTPM firmware\n", __FUNCTION__);

    temp_buffer[0] = FT5x06_CMD_RESET_FW;

    retval = i2c_master_send(ts->client, temp_buffer, 1);
    if (0 > retval)
    {
        dev_err(dev, "%s() - ERROR: Could not write the Reset Firmware command to the CTPM.\n", __FUNCTION__);
        goto error_reset;
    }

    retval = 0;

error_reset:
    /*********************************************************************************************/
    printk(KERN_INFO "%s() - Step 8: Reset the CTPM\n", __FUNCTION__);

    ft5x06_reset_panel_via_gpio(ts->platform_data->reset_gpio);

    dev_info(dev, "%s() - FIRMWARE UPDATE COMPLETE - Update %s\n", __FUNCTION__, ((0 == retval) ? "Succeeded" : "Failed"));

	if (firmware_src_buffer)
    {
        //release_firmware(fw_blob);
        kfree(firmware_src_buffer);
	firmware_src_buffer = NULL;
    }
	printk(KERN_INFO "upgrade end!\n");

    return retval;
}

static ssize_t ft5x06_ftsfwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct ft5x06     *ts     = (struct ft5x06 *)i2c_get_clientdata(client);

    mutex_lock(&ts->device_mode_mutex);

    if (ft5x06_ftsperform_fw_upgrade(dev))
    {
        dev_err(dev, "%s() - ERROR: Could not update the firmware using \n", __FUNCTION__);
        goto error_return;
    }

error_return:


    mutex_unlock(&ts->device_mode_mutex);

	return size;
}

static ssize_t ft5x06_ftsfwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    /* place holder for future use */
    return -EPERM;
}

#endif

/* sysfs */
static DEVICE_ATTR(driver_version,      S_IRUGO,           ft5x06_driver_version_show,      NULL);
static DEVICE_ATTR(rawbase,             S_IRUGO,           ft5x06_rawbase_show,             NULL);
static DEVICE_ATTR(crosstalk,           S_IRUGO | S_IWUSR, ft5x06_crosstalk_show,           ft5x06_crosstalk_store);
static DEVICE_ATTR(icsupplier,          S_IRUGO,           ft5x06_icsupplier_show,          NULL);
static DEVICE_ATTR(icpartno,            S_IRUGO,           ft5x06_icpartno_show,            NULL);
static DEVICE_ATTR(storecalibrateflash, S_IRUGO,           ft5x06_storecalibrateflash_show, NULL);
static DEVICE_ATTR(tpfwver,             S_IRUGO,           ft5x06_tpfwver_show,             NULL);
static DEVICE_ATTR(vendorid,            S_IRUGO,           ft5x06_vendorid_show,            NULL);
static DEVICE_ATTR(voltage,             S_IRUGO | S_IWUSR, ft5x06_voltage_show,             ft5x06_voltage_store);
static DEVICE_ATTR(calibrate,           S_IRUGO,           ft5x06_calibrate_show2,           NULL);
static DEVICE_ATTR(interrupttest,       S_IRUGO,           ft5x06_interrupttest_show,       NULL);
static DEVICE_ATTR(tpreset,             S_IRUGO,           ft5x06_tpreset_show,             NULL);
static DEVICE_ATTR(fwupdate,            S_IRUGO | S_IWUSR, ft5x06_fwupdate_show,            ft5x06_fwupdate_store);
static DEVICE_ATTR(fmreg,               S_IRUGO | S_IWUSR, ft5x06_fmreg_show,               ft5x06_fmreg_store);
static DEVICE_ATTR(fmval,               S_IRUGO | S_IWUSR, ft5x06_fmval_show,               ft5x06_fmval_store);
static DEVICE_ATTR(wmreg,               S_IRUGO | S_IWUSR, ft5x06_wmreg_show,               ft5x06_wmreg_store);
static DEVICE_ATTR(wmval,               S_IRUGO | S_IWUSR, ft5x06_wmval_show,               ft5x06_wmval_store);

#ifdef CONFIG_TOUCHSCREEN_FT5x06_TEST
static DEVICE_ATTR(resettest,           S_IRUGO,           ft5x06_resettest_show,           NULL);
static DEVICE_ATTR(dump_rxcac,          S_IRUGO,           ft5x06_dump_rxcac_show,          NULL);
static DEVICE_ATTR(ftsbaseline,        S_IRUGO | S_IWUSR, ft5x06_ftsbaseline_show, NULL);
static DEVICE_ATTR(ftsvoltage,        S_IRUGO | S_IWUSR, ft5x06_ftsvoltage_show, NULL);
static DEVICE_ATTR(ftscrosstalkodd,        S_IRUGO, ft5x06_ftscrosstalkodd_show, NULL);
static DEVICE_ATTR(ftssetstation,        S_IRUGO | S_IWUSR, ft5x06_ftssetstation_show, ft5x06_ftssetstation_store);
static DEVICE_ATTR(ftsbaselineresult,          S_IRUGO,           ft5x06_ftsbaselineresult_show,          NULL);
static DEVICE_ATTR(ftsvoltageresult,          S_IRUGO,           ft5x06_ftsvoltageresult_show,          NULL);
static DEVICE_ATTR(ftscrosstalkoddresult,          S_IRUGO,       ft5x06_ftscrosstalkoddresult_show,          NULL);
static DEVICE_ATTR(ftstestresult,          S_IRUGO,       ft5x06_ftstestresult_show,          NULL);
static DEVICE_ATTR(ftsfullpanel,          S_IRUGO,       ft5x06_ftsfullpanel_show,          NULL);
static DEVICE_ATTR(ftsfullpanelresult,          S_IRUGO,       ft5x06_ftsfullpanelresult_show,          NULL);
static DEVICE_ATTR(ftstpupdate,        S_IRUGO | S_IWUSR, ft5x06_ftsfwupdate_show, ft5x06_ftsfwupdate_store);
static DEVICE_ATTR(ftsrxoffset,        S_IRUGO, ft5x06_rxoffset_show, NULL);
#endif

static struct attribute *ft5x06_attributes[] = {
    &dev_attr_irq_enabled.attr,
    &dev_attr_driver_version.attr,
    &dev_attr_rawbase.attr,
    &dev_attr_crosstalk.attr,
    &dev_attr_icsupplier.attr,
    &dev_attr_icpartno.attr,
    &dev_attr_storecalibrateflash.attr,
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
    &dev_attr_info.attr,
    &dev_attr_report_rate.attr,
    &dev_attr_fingers_supported.attr,
    &dev_attr_dbg_level.attr,
    &dev_attr_build_id.attr,
    &dev_attr_power_state.attr,
#ifdef CONFIG_TOUCHSCREEN_FT5x06_TEST
    &dev_attr_resettest.attr,
    &dev_attr_dump_rxcac.attr,
    &dev_attr_ftsbaseline.attr,
    &dev_attr_ftsvoltage.attr,
    &dev_attr_ftscrosstalkodd.attr,
    &dev_attr_ftssetstation.attr,
    &dev_attr_ftsbaselineresult.attr,
    &dev_attr_ftsvoltageresult.attr,
    &dev_attr_ftscrosstalkoddresult.attr,
    &dev_attr_ftstestresult.attr,
    &dev_attr_ftsfullpanel.attr,
    &dev_attr_ftsfullpanelresult.attr,
    &dev_attr_ftstpupdate.attr,
    &dev_attr_ftsrxoffset.attr,
#endif
    NULL
};

static struct attribute_group ft5x06_attribute_group = {
    .attrs = ft5x06_attributes
};


static int ftx_input_open(struct input_dev *dev)
{
	struct ft5x06 *ts = input_get_drvdata(dev);
	int force_resume;

	/* Check if the controller is ON, if not turn it ON */
	force_resume = atomic_cmpxchg(&ts->force_resume_ftx, 0, 1); /* After this force_resume_ftx will be 1, and old value will be in force_resume */
	ft5x06_resume(&ts->client->dev);
	atomic_cmpxchg(&ts->force_resume_ftx, !force_resume, force_resume); /* After this force_resume_ftx will be force_resume */

	return 0;
}
static void ftx_input_close(struct input_dev *dev)
{
	struct ft5x06 *ts = input_get_drvdata(dev);

	ft5x06_suspend(&ts->client->dev);
}
/* ************************************************************************
 * Probe and Initialization functions
 * ***********************************************************************/

static void ftx_input_device_destroy(struct ft5x06 *ts)
{
	struct input_dev *input_device = ts->input;

	if (NULL != ts->input)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Freeing input device\n", dev_name(&ts->client->dev), __func__);
		input_free_device(ts->input);
		kfree(ts->input);
		ts->input = NULL;
	}
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Deallocating the fingers\n", dev_name(&ts->client->dev), __func__);
	ftx_allocate_fingers(ts, 0);
	atomic_set(&ts->uses_events, 0);
}

static int ftx_configure_volatile_settings(struct ft5x06 *ts)
{
	u8 reg_val;
	int retval;

	reg_val = (ts->platform_data->maxx >> 8) & 0xFF;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_MAX_X_HIGH , sizeof(u8), &reg_val);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the register at offset 0x%02x.\n", dev_name(&ts->client->dev), __func__, reg_val, FT5x06_WMREG_MAX_X_HIGH);
	}
	reg_val = (ts->platform_data->maxx) & 0xFF;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_MAX_X_LOW , sizeof(u8), &reg_val);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the register at offset 0x%02x.\n", dev_name(&ts->client->dev), __func__, reg_val, FT5x06_WMREG_MAX_X_LOW);
	}
	reg_val = (ts->platform_data->maxy >> 8) & 0xFF;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_MAX_Y_HIGH , sizeof(u8), &reg_val);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the register at offset 0x%02x.\n", dev_name(&ts->client->dev), __func__, reg_val, FT5x06_WMREG_MAX_Y_HIGH);
	}
	reg_val = (ts->platform_data->maxy) & 0xFF;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_MAX_Y_LOW , sizeof(u8), &reg_val);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the register at offset 0x%02x.\n", dev_name(&ts->client->dev), __func__, reg_val, FT5x06_WMREG_MAX_Y_LOW);
	}

	reg_val = FTX_MAX_FINGERS;
	retval = i2c_smbus_write_i2c_block_data(ts->client, FT5x06_WMREG_MAX_TOUCHES, sizeof(u8), &reg_val);
	if (0 != retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not write 0x%02x to the register at offset 0x%02x.\n", dev_name(&ts->client->dev), __func__, reg_val, FT5x06_WMREG_MAX_TOUCHES);
	}

	return 0;
}
static int ftx_input_device_initialize(struct ft5x06 *ts)
{
	struct input_dev *input_device;
	int use_mt_protocol;
	int retval;

	if(NULL != ts->input)
	{
		ftx_input_device_destroy(ts);
	}

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not allocate input device.\n", dev_name(&ts->client->dev), __func__);
		return -ENOMEM;
	}

	atomic_set(&ts->uses_events, 1);

	if(ftx_allocate_fingers(ts, FTX_MAX_FINGERS) == FTX_MAX_FINGERS)
	{
		use_mt_protocol = FTX_MAX_FINGERS;
	}
	else
	{
		use_mt_protocol = 0;
	}

	/* Set the common parameters */
	input_device->name = ts->device_name;
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &ts->client->dev;
	input_device->open = ftx_input_open;
	input_device->close = ftx_input_close;
	input_set_drvdata(input_device, ts);
	set_bit(EV_SYN,    input_device->evbit);
	set_bit(EV_KEY,    input_device->evbit);
	set_bit(EV_ABS,    input_device->evbit);

	/* Support ST events */
#if defined(FTX_SUPPORT_ST)
	input_set_abs_params(input_device, ABS_X,          0, ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_Y,          0, ts->platform_data->maxy, 0, 0);
	input_set_abs_params(input_device, ABS_TOOL_WIDTH, 0, FT_LARGE_TOOL_WIDTH,     0, 0);
	input_set_abs_params(input_device, ABS_PRESSURE,    0, FT_MAXZ,                 0, 0);
#endif /* defined(FTX_SUPPORT_ST) */

	set_bit(BTN_TOUCH, input_device->keybit);

	if (ts->platform_data->use_gestures)
	{
		//set_bit(BTN_3, input_device->keybit);
		input_set_abs_params(input_device, ABS_HAT1X, 0, 255, 0, 0);  // Gesture code.
		input_set_abs_params(input_device, ABS_HAT2X, 0, 255, 0, 0);  // Gesture touch count.
		input_set_abs_params(input_device, ABS_HAT2Y, 0, 255, 0, 0);  // Gesture occur count.
	}

	if(use_mt_protocol != 0)
	{
		/* Initialize the slot handling. If it is  already initialized, this function just returns 0 */
		if(1) //(input_mt_init_slots(input_device, atomic_read(&ts->n_fingers_supported)) != 0)
		{
			/*
			 * Slots were not initialized. It is difficult to support Type B protocol without the slots.
			 * Use Type A protocol
			 */
			ftx_input_switch_protocol(ts, FTX_MT_PROTOCOL_A);
			DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Failed to initialze slots; using Type A protocol.\n", dev_name(&ts->client->dev), __func__);
		}
		else
		{
			ftx_input_switch_protocol(ts, FTX_MT_PROTOCOL_B);
			DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Successfully initialzed slots; using Type B protocol.\n", dev_name(&ts->client->dev), __func__);
		}

		/* Set the parameters for multi touch only if we can support MT */
		input_set_abs_params(input_device, ABS_MT_POSITION_X,  0, ts->platform_data->maxx - 1, 0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y,  0, ts->platform_data->maxy - 1, 0, 0);
		//input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0, FT_LARGE_TOOL_WIDTH,     0, 0);
		input_set_abs_params(input_device, ABS_MT_PRESSURE,    0, FT_MAXZ,                 0, 0);
		input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0, FT_MAXZ,                 0, 0);
	}
	/* Support ST events */
#if defined(FTX_SUPPORT_ST)
	else
	{
		/*
		 * We will not be able to support multi touches without the buffers for fingers.
		 * Use only ST protocol
		 */
		ftx_input_switch_protocol(ts, FTX_MT_PROTOCOL_NONE);
		DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Could not setup multi-finger support; using ST protocol.\n", dev_name(&ts->client->dev), __func__);
	}
#endif /* defined(FTX_SUPPORT_ST) */

	/* Register the device */
	if( 0 != input_register_device(input_device))
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not register input device.\n", dev_name(&ts->client->dev), __func__);
		input_free_device(input_device);
		kfree(input_device);
		input_device = NULL;
		return -EBUSY;
	}

	ts->input = input_device;

	return 0;
}

/* ft5x06_initialize: Driver Initialization. This function takes
 * care of the following tasks:
 * 1. Create and register an input device with input layer
 * 2. Take FT5x06 device out of bootloader mode; go operational
 * 3. Start any timers/Work queues.
 * Note that validation has already been performed on the inputs.
 */
static int ft5x06_initialize(struct ft5x06 *ts)
{
	int retval = 0;

	ts->crosstalk_test_type   = FT5x06_CROSSTALK_TEST_TYPE_EVEN;
	ts->factory_mode_register = FT5x06_FMREG_DEVICE_MODE;
	ts->working_mode_register = FT5x06_WMREG_DEVICE_MODE;

	ts->ft5x06_ts_wq = create_singlethread_workqueue("ft5x06_touch");
	if (NULL == ts->ft5x06_ts_wq)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not create the work queue\n", dev_name(&ts->client->dev), __func__);
		retval = -ENOMEM;
		goto success;
	}

	/* Prepare our worker structure prior to setting up the ISR */
	INIT_WORK(&ts->work, ft5x06_process_touch);

	/* Interrupt setup */
	if (ts->client->irq)
	{
		/* request_irq() will call enable_irq() */
		retval = request_irq(ts->client->irq, ft5x06_irq, IRQF_TRIGGER_FALLING, ts->device_name, ts);
		if (retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not acquire irq\n", dev_name(&ts->client->dev), __func__);
			goto error_destroy_wq;
		}
		atomic_set(&ts->irq_enabled, 1);
	}

	mutex_lock(&ts->lock_ftx_suspend_state);
	/* Disable the interrupt so that no more touches are reported */
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Disabling irq.\n", dev_name(&ts->client->dev), __func__);
	disable_irq(ts->client->irq);
	ts->ftx_suspend_state = FTX_SUSPEND;
	mutex_unlock(&ts->lock_ftx_suspend_state);

	retval = sysfs_create_group(&ts->client->dev.kobj, &ft5x06_attribute_group);
	if (retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not create sysfs entries\n", dev_name(&ts->client->dev), __func__);
		goto error_free_irq;
	}
	else
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Create sysfs entries\n", dev_name(&ts->client->dev), __func__);
	}

	retval =  ftx_input_device_initialize(ts);
	if(retval != 0)
	{
		goto error_free_irq;
	}

	retval = 0;
	goto success;

error_destroy_device:
	ftx_input_device_destroy(ts);
error_free_irq:
	free_irq(ts->client->irq, ts);
	atomic_set(&ts->irq_enabled, 0);
error_destroy_wq:
	if(ts->ft5x06_ts_wq)
	{
		destroy_workqueue(ts->ft5x06_ts_wq);
	}
success:
    return retval;
}

static void ft5x06_deinitialize(struct ft5x06 *ts)
{
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Releasing irq.\n", dev_name(&ts->client->dev), __func__);
	free_irq(ts->client->irq, ts);
	atomic_set(&ts->irq_enabled, 0);

	/* Removing any queued work */
	if (cancel_work_sync(&ts->work) < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not remove all the work from work queue.\n", dev_name(&ts->client->dev), __func__);
	}

	if(ts->ft5x06_ts_wq)
	{
		/* Flush the workqueue so that all the pending events are reported */
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Flushing workqueue.\n", dev_name(&ts->client->dev), __func__);
		flush_workqueue(ts->ft5x06_ts_wq);
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Destroying workqueue.\n", dev_name(&ts->client->dev), __func__);
		destroy_workqueue(ts->ft5x06_ts_wq);
	}

	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Destroying input device.\n", dev_name(&ts->client->dev), __func__);
	ftx_input_device_destroy(ts);
}

/* I2C driver probe function */
struct ft5x06 *factory_test_ts = NULL;
static int __devinit ft5x06_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	const struct ft5x06_platform_data *pdata = client->dev.platform_data;
	struct ft5x06 *ts;
	int retval = 0;

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: probing for %s @ %s .\n", dev_name(&client->dev), __func__, id->name, dev_name(&client->dev));

	if (!pdata)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: No platform data supplied...exiting.\n", dev_name(&client->dev), __func__);
		return -EINVAL;
	}
	if(pdata->request_resources)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Calling platform specific function to request resources.\n", dev_name(&client->dev), __func__);
		retval = pdata->request_resources(&client->dev);
		if(retval != 0)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Failed to acquire resources...exiting.\n", dev_name(&client->dev), __func__);
			goto err_request_resources;
		}
	}

	ts = kzalloc(sizeof(struct ft5x06), GFP_KERNEL);
	if (NULL == ts)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not allocate %d bytes of memory for private structure...exiting.\n", dev_name(&client->dev), __func__, sizeof(struct ft5x06));
		retval = -ENOMEM;
		goto err_data_alloc;
	}

	ts->client = client;
	ts->platform_data = client->dev.platform_data;
	ts->device_name = id->name;
	i2c_set_clientdata(client, ts);

	atomic_set(&ts->do_not_suspend_ftx, 0);
	atomic_set(&ts->force_resume_ftx, 0);
	atomic_set(&ts->power_on_ftx, 0);

	mutex_init(&ts->lock_fingers);
	mutex_init(&ts->lock_ftx_power_supply_state);
	mutex_init(&ts->lock_ftx_suspend_state);

	mutex_lock(&ts->lock_ftx_power_supply_state);
	if(pdata->power_on)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Calling platform specific function to power ON the device.\n", dev_name(&client->dev), __func__);
		retval = pdata->power_on(&client->dev);
		if(retval != 0)
		{
			DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Failed to power ON the controller...exiting.\n", dev_name(&client->dev), __func__);
			mutex_unlock(&ts->lock_ftx_power_supply_state);
			goto err_power_on;
		}
	}
	ts->ftx_power_supply_state = FTX_POWER_SUPPLY_ON;
	mutex_unlock(&ts->lock_ftx_power_supply_state);

	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_CONTROLLER_ID, sizeof(u8), &ts->info.product_id);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: I2C bus read error.\n", dev_name(&(ts->client->dev)), __func__);
		goto err_i2c_verify;
	}
	if((ts->info.product_id != FT5606_CPTM_ID_PRODUCT) && (ts->info.product_id != FT5506_CPTM_ID_PRODUCT)) /* Default to FT5606 */
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Unsupported Product ID 0x%02x...defaulting to 5606.\n", dev_name(&(ts->client->dev)), __func__, ts->info.product_id);
		ts->info.product_id = FT5606_CPTM_ID_PRODUCT;
	}

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: focaltech controller with Product ID=0x%02x found @ 0x%02x.\n", dev_name(&client->dev), __func__, ts->info.product_id, client->addr);

	switch(ts->info.product_id)
	{
	case 0x05:
		{
			/* TODO add the parameters and verify the product id */
		}
		/* Fall through as of now */
	case 0x06:
		{
			/* TODO add the parameters and verify the product id */
			ts->ftx_upgrade_info.upgrade_h_delay = FT5606_UPGRADE_H_DELAY;
			ts->ftx_upgrade_info.upgrade_l_delay = FT5606_UPGRADE_L_DELAY;
			ts->ftx_upgrade_info.upgrade_enter_upgrade_mode_delay = FT5606_UPGRADE_ENTER_UPGRADE_MODE_DELAY;
			ts->ftx_upgrade_info.upgrade_erase_delay = FT5606_UPGRADE_ERASE_DELAY;
			ts->ftx_upgrade_info.upgrade_packet_delay = FT5606_UPGRADE_PACKET_DELAY;
			ts->ftx_upgrade_info.upgrade_checksum_delay = FT5606_UPGRADE_CHECKSUM_DELAY;
			ts->ftx_upgrade_info.upgrade_reset_fw_delay = FT5606_UPGRADE_RESET_FW_DELAY;
		}
		break;
	default:
		{
			/* TODO change the parameters */
			ts->ftx_upgrade_info.upgrade_h_delay = FT5606_UPGRADE_H_DELAY;
			ts->ftx_upgrade_info.upgrade_l_delay = FT5606_UPGRADE_L_DELAY;
			ts->ftx_upgrade_info.upgrade_enter_upgrade_mode_delay = FT5606_UPGRADE_ENTER_UPGRADE_MODE_DELAY;
			ts->ftx_upgrade_info.upgrade_erase_delay = FT5606_UPGRADE_ERASE_DELAY;
			ts->ftx_upgrade_info.upgrade_packet_delay = FT5606_UPGRADE_PACKET_DELAY;
			ts->ftx_upgrade_info.upgrade_checksum_delay = FT5606_UPGRADE_CHECKSUM_DELAY;
			ts->ftx_upgrade_info.upgrade_reset_fw_delay = FT5606_UPGRADE_RESET_FW_DELAY;
		}
		break;
	}

	retval = i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_FW_VER, sizeof(u8), &ts->info.fw_ver);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: I2C bus read error.\n", dev_name(&(ts->client->dev)), __func__);
	}

	switch(ts->info.fw_ver)
	{
	case 0x0b:
		{
			ts->platform_data->flags = FLIP_DATA_FLAG | REVERSE_X_FLAG;
			ts->platform_data->maxx = 768;
		}
		break;
	}

	/* Switch off the controller till someone starts using it */
	mutex_lock(&ts->lock_ftx_power_supply_state);
	if(ts->ftx_power_supply_state != FTX_POWER_SUPPLY_OFF)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: power supply is not OFF.....switching OFF.\n", dev_name(&client->dev), __func__);
		if (ts->platform_data->power_off)
		{
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Calling platform specific function to power OFF the device.\n", dev_name(&client->dev), __func__);
			ts->platform_data->power_off(&client->dev);
		}
		ts->ftx_power_supply_state = FTX_POWER_SUPPLY_OFF;
	}
	mutex_unlock(&ts->lock_ftx_power_supply_state);

	// Need to initialize the SYSFS mutex before creating the SYSFS entries in ft5x06_initialize().
	mutex_init(&ts->device_mode_mutex);
	retval = ft5x06_initialize(ts);
	if (0 > retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Controller could not be initialized...exiting.\n", dev_name(&client->dev), __func__);
		goto error_device_mode_mutex_destroy;
	}

#if defined(CONFIG_DEBUG_FS)
	mutex_init(&ts->ftx_raw_data_mutex);
	ts->ftx_raw_data_tx_start = 0;
	ts->ftx_raw_data_rx_start = 0;
	ts->ftx_raw_data_tx_end = ts->platform_data->max_tx_lines - 1;
	ts->ftx_raw_data_rx_end = ts->platform_data->max_rx_lines - 1;
	ts->ftx_raw_data_offset = 7500;
	ts->ftx_raw_data = kzalloc(ts->platform_data->max_tx_lines * ts->platform_data->max_rx_lines * 2, GFP_KERNEL);
	if(ts->ftx_raw_data == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: No memory for raw data buffer.\n", dev_name(&ts->client->dev), __func__);
	}

	ts->ftx_dbgfs_entries = ftx_dbgfs_files;
	ftx_dbgfs_create(ts);
#endif /* defined(CONFIG_DEBUG_FS) */

#ifdef CONFIG_HAS_EARLYSUSPEND
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Registering for early suspend.\n", dev_name(&client->dev), __func__);
	ts->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ft5x06_early_suspend;
	ts->early_suspend.resume  = ft5x06_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	factory_test_ts = ts;

	goto error_return;

#if defined(CONFIG_DEBUG_FS)
error_raw_data_mutex_destroy:
	mutex_destroy(&ts->ftx_raw_data_mutex);
#endif /* defined(CONFIG_DEBUG_FS) */
error_device_mode_mutex_destroy:
	mutex_destroy(&ts->device_mode_mutex);
err_i2c_verify:
	mutex_lock(&ts->lock_ftx_power_supply_state);
	if(ts->ftx_power_supply_state != FTX_POWER_SUPPLY_OFF)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: power supply is not OFF.....switching OFF.\n", dev_name(&client->dev), __func__);
		if(pdata->power_off)
		{
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Calling platform specific function to power OFF the device.\n", dev_name(&client->dev), __func__);
			pdata->power_off(&client->dev);
		}
		ts->ftx_power_supply_state = FTX_POWER_SUPPLY_OFF;
	}
	mutex_unlock(&ts->lock_ftx_power_supply_state);
err_power_on:
	mutex_destroy(&ts->lock_ftx_suspend_state);
	mutex_destroy(&ts->lock_ftx_power_supply_state);
	mutex_destroy(&ts->lock_fingers);
	i2c_set_clientdata(client, NULL);
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Freeing private structure.\n", dev_name(&client->dev), __func__);
	kfree(ts);
	ts = NULL;
err_data_alloc:
	if(pdata->release_resources)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Calling platform specific function to release resources.\n", dev_name(&client->dev), __func__);
		pdata->release_resources(&client->dev);
	}
err_request_resources:
error_return:
    return retval;
}

/* registered in driver struct */
static int ft5x06_remove(struct i2c_client *client)
{
	struct ft5x06 *ts;

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Driver is being removed.\n", dev_name(&client->dev), __func__);

	/* clientdata registered on probe */
	factory_test_ts = NULL;
	ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Unregistering from early suspend.\n", dev_name(&client->dev), __func__);
	unregister_early_suspend(&ts->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	/* TODO: We need to clear all existing touch before removing the driver */
	ftx_clear_all_touches(ts);

#if defined(CONFIG_DEBUG_FS)
	ftx_dbgfs_destroy(ts);
	if(ts->ftx_raw_data != NULL)
	{
		kfree(ts->ftx_raw_data);
		ts->ftx_raw_data = NULL;
	}
	mutex_destroy(&ts->ftx_raw_data_mutex);
#endif /* defined(CONFIG_DEBUG_FS) */

	/*
	 * Wait until any outstanding SYSFS transaction has finished,
	 * and prevent any new ones from starting.
	 */
	mutex_lock(&ts->device_mode_mutex);
	/* Remove the SYSFS entries */
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Removing sysfs entries.\n", dev_name(&client->dev), __func__);
	sysfs_remove_group(&client->dev.kobj, &ft5x06_attribute_group);
	mutex_unlock(&ts->device_mode_mutex);
	mutex_destroy(&ts->device_mode_mutex);

	/* Deinitialize the private structure */
	ft5x06_deinitialize(ts);

	mutex_lock(&ts->lock_ftx_power_supply_state);
	if(ts->ftx_power_supply_state != FTX_POWER_SUPPLY_OFF)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: power supply is not OFF.....switching OFF.\n", dev_name(&client->dev), __func__);
		if(ts->platform_data->power_off)
		{
			DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Calling platform specific function to power OFF the device.\n", dev_name(&client->dev), __func__);
			ts->platform_data->power_off(&client->dev);
		}
		ts->ftx_power_supply_state = FTX_POWER_SUPPLY_OFF;
	}
	mutex_unlock(&ts->lock_ftx_power_supply_state);

	if(ts->platform_data->release_resources)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Calling platform specific function to release resources.\n", dev_name(&client->dev), __func__);
		ts->platform_data->release_resources(&client->dev);
	}

	if (NULL != ts)
	{
		DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: Freeing private structure.\n", dev_name(&client->dev), __func__);
		kfree(ts);
		ts = NULL;
	}
	i2c_set_clientdata(client, NULL);

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: Driver removal is complete.\n", dev_name(&client->dev), __func__);

	return 0;
}

#if defined(CONFIG_PM)
/* Function to manage power-on resume */
static int ft5x06_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int retval = 0;
	struct ft5x06 *ts = NULL;
	u8 dummy_val;


	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: driver is resuming.\n", dev_name(dev), __func__);
	ts = (struct ft5x06 *) i2c_get_clientdata(client);

	if(atomic_read(&ts->force_resume_ftx) == 0)
	{
		mutex_lock(&ts->input->mutex);
		if(ts->input->users) /* Power on only if someone is using the device */
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: device opened by %u users.\n", dev_name(dev), __func__, ts->input->users);
			mutex_lock(&ts->lock_ftx_power_supply_state);
			if(ts->ftx_power_supply_state != FTX_POWER_SUPPLY_ON)
			{
				DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: power supply is not ON.....switching ON.\n", dev_name(dev), __func__);
				if (ts->platform_data->power_on)
				{
					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Calling platform specific function to power ON the device.\n", dev_name(dev), __func__);
					ts->platform_data->power_on(&client->dev);
					atomic_set(&ts->power_on_ftx, 1);
				}
				ts->ftx_power_supply_state = FTX_POWER_SUPPLY_ON;
			}
			mutex_unlock(&ts->lock_ftx_power_supply_state);

			/*
			 * We will get one interrupt after power on to indicate that the controller is ready.
			 * Consume that interrupt in the interrupt handler.
			 */

			mutex_lock(&ts->lock_ftx_suspend_state);
			if(ts->ftx_suspend_state != FTX_ACTIVE)
			{
				ts->ftx_suspend_state = FTX_ACTIVE;
				DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Enabling irq.\n", dev_name(dev), __func__);
				enable_irq(ts->client->irq);
			}
			mutex_unlock(&ts->lock_ftx_suspend_state);

			ftx_configure_volatile_settings(ts);
		}
		mutex_unlock(&ts->input->mutex);
	}
	else
	{
			mutex_lock(&ts->lock_ftx_power_supply_state);
			if(ts->ftx_power_supply_state != FTX_POWER_SUPPLY_ON)
			{
				DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: power supply is not ON.....switching ON.\n", dev_name(dev), __func__);
				if (ts->platform_data->power_on)
				{
					DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Calling platform specific function to power ON the device.\n", dev_name(dev), __func__);
					ts->platform_data->power_on(&client->dev);
					atomic_set(&ts->power_on_ftx, 1);
				}
				ts->ftx_power_supply_state = FTX_POWER_SUPPLY_ON;
			}
			mutex_unlock(&ts->lock_ftx_power_supply_state);

			/*
			 * We will get one interrupt after power on to indicate that the controller is ready.
			 * Consume that interrupt by a dummy read.
			 */
			//i2c_smbus_read_i2c_block_data(ts->client, FT5x06_WMREG_TD_STATUS, sizeof(u8), &dummy_val);

			mutex_lock(&ts->lock_ftx_suspend_state);
			if(ts->ftx_suspend_state != FTX_ACTIVE)
			{
				ts->ftx_suspend_state = FTX_ACTIVE;
				DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Enabling irq.\n", dev_name(dev), __func__);
				enable_irq(ts->client->irq);
			}
			mutex_unlock(&ts->lock_ftx_suspend_state);
			ftx_configure_volatile_settings(ts);
	}

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: driver resumed.\n", dev_name(dev), __func__);
	return retval;
}

/* Function to manage low power suspend */
static int ft5x06_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x06 *ts = (struct ft5x06 *) i2c_get_clientdata(client);

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: driver is suspending.\n", dev_name(dev), __func__);

	if(atomic_read(&ts->do_not_suspend_ftx) != 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: suspend is disabled....try again.\n", dev_name(dev), __func__);
		return -EAGAIN;
	}

	mutex_lock(&ts->lock_ftx_suspend_state);
	if(ts->ftx_suspend_state != FTX_SUSPEND)
	{
		/* We are setting suspend to make sure that the irq is not reenabled in case there is pending irq work which reeanbles the irq*/
		ts->ftx_suspend_state = FTX_SUSPEND;
		/* Disable the interrupt so that no more touches are reported */
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Disabling irq.\n", dev_name(dev), __func__);
		disable_irq(ts->client->irq);
	}
	mutex_unlock(&ts->lock_ftx_suspend_state);

	/* Remove any queued work */
	if (cancel_work_sync(&ts->work) < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " FTX_TAG ": %s(): ERROR: Could not remove all the work from work queue.\n", dev_name(&ts->client->dev), __func__);
	}

	/* Clear all existing touch before suspending the controller */
	ftx_clear_all_touches(ts);

	/* Power off the controller */
	mutex_lock(&ts->lock_ftx_power_supply_state);
	if(ts->ftx_power_supply_state != FTX_POWER_SUPPLY_OFF)
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: power supply is not OFF.....switching OFF.\n", dev_name(dev), __func__);
		if (ts->platform_data->power_off)
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Calling platform specific function to power OFF the device.\n", dev_name(dev), __func__);
			ts->platform_data->power_off(&client->dev);
		}
		ts->ftx_power_supply_state = FTX_POWER_SUPPLY_OFF;
	}
	mutex_unlock(&ts->lock_ftx_power_supply_state);

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": %s(): INFO: driver suspended.\n", dev_name(dev), __func__);

	return 0;
}

static const struct dev_pm_ops ft5x06_pm_ops = {
	.suspend	= ft5x06_suspend,
	.resume		= ft5x06_resume,
};
#else
static int ft5x06_suspend(struct device *dev)
{
	/* Do Nothing */
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: suspend not defined.\n", dev_name(dev), __func__);
}
static int ft5x06_resume(struct device *dev)
{
	/* Do Nothing */
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: resume not defined.\n", dev_name(dev), __func__);
}
#endif /* defined(CONFIG_PM) */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x06_early_suspend(struct early_suspend *handler)
{
	struct ft5x06 *ts;
	ts = container_of(handler, struct ft5x06, early_suspend);
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Suspending touch.\n", dev_name(&(ts->client->dev)), __func__);
	ft5x06_suspend(&ts->client->dev);
}

static void ft5x06_late_resume(struct early_suspend *handler)
{
	struct ft5x06 *ts;
	ts = container_of(handler, struct ft5x06, early_suspend);
	DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Resuming touch.\n", dev_name(&(ts->client->dev)), __func__);
	ft5x06_resume(&ts->client->dev);
}
#else  /* CONFIG_HAS_EARLYSUSPEND */
static void ft5x06_early_suspend(struct early_suspend *handler)
{
	/* Do Nothing */
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: early suspend not defined.\n", dev_name(&(ts->client->dev)), __func__);
}

static void ft5x06_late_resume(struct early_suspend *handler)
{
	/* Dio Nothing */
	DBG_PRINT(dbg_level_debug, "%s: " FTX_TAG ": %s(): DEBUG: late resume not defined.\n", dev_name(&(ts->client->dev)), __func__);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id ft5x06_id[] = {
	{ FT_DEVICE_5x06_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x06_id);

static struct i2c_driver ft5x06_driver = {
	.driver = {
		.name = FT_DRIVER_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_PM)
#if !defined(CONFIG_HAS_EARLYSUSPEND)
		.pm	= &ft5x06_pm_ops,
#endif /* !defined(CONFIG_HAS_EARLYSUSPEND) */
#endif /* defined(CONFIG_PM) */
	},
	.probe = ft5x06_probe,
	.remove = ft5x06_remove,
	.shutdown = ft5x06_shutdown,
	.id_table = ft5x06_id,
};

static void  ft5x06_shutdown(struct i2c_client *client)
{
	struct ft5x06 *ts = (struct ft5x06 *) i2c_get_clientdata(client);

	DBG_PRINT(dbg_level_info, "%s: " FTX_TAG ": INFO: Shutdown.\n", dev_name(&ts->client->dev), __func__);

	/* Disable the irq */
	if(ts->client->irq)
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Disabling touch interrupt.\n", dev_name(&(ts->client->dev)), __func__);
		disable_irq(ts->client->irq);
	}

	/* Power off the controller */
	mutex_lock(&ts->lock_ftx_power_supply_state);
	if(ts->ftx_power_supply_state != FTX_POWER_SUPPLY_OFF)
	{
		DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: power supply is not OFF.....switching OFF.\n", dev_name(&(ts->client->dev)), __func__);
		if (ts->platform_data->power_off)
		{
			DBG_PRINT(dbg_level_verbose, "%s: " FTX_TAG ": %s(): VERBOSE: Calling platform specific function to power OFF the device.\n", dev_name(&(ts->client->dev)), __func__);
			ts->platform_data->power_off(&client->dev);
		}
		ts->ftx_power_supply_state = FTX_POWER_SUPPLY_OFF;
	}
	mutex_unlock(&ts->lock_ftx_power_supply_state);
}

static int load_count = 0;
static int ft5x06_module_init()
{
	int ret = 0;

	if (load_count == 0)
	{
		/* cur_dbg_level is initialized to dbg_level_info statically */
		///* Initialize the cur_dbg_level to error */
		//cur_dbg_level = dbg_level_info;

		DBG_PRINT(dbg_level_critical, FTX_TAG ": %s(): INFO: Initializing FT I2C Touchscreen Driver (Built %s @ %s)\n", __func__, __DATE__, __TIME__);
		ret = i2c_add_driver(&ft5x06_driver);
	}

	load_count = 1;

	return ret;
}
static void ft5x06_module_remove()
{
	if (load_count == 1)
	{
		DBG_PRINT(dbg_level_info, FTX_TAG ": %s(): INFO: FT I2C Touchscreen Driver exiting (Built %s @ %s)\n", __func__, __DATE__, __TIME__);
		i2c_del_driver(&ft5x06_driver);
	}

	load_count = 0;
}

/*Add a interface to hot remove/insert touchpanel module for TP factory test*/
static ssize_t ft5x06_module_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
	       const char *buf, size_t n)
{
	int val;

	sscanf(buf, "%d", &val);
	DBG_PRINT(dbg_level_debug, FTX_TAG ": %s(): DEBUG: %s ft5x06 touch panel-------------\n", __func__, val ? "Enable" : "Disable");
	if (val == 1)
	{
		ft5x06_module_init();
		if (factory_test_ts != NULL && factory_test_ts->client != NULL)
		{
			atomic_set(&factory_test_ts->force_resume_ftx, 1);
			ft5x06_resume(&(factory_test_ts->client->dev));
		}
	}
	else
	{
		if (factory_test_ts != NULL && factory_test_ts->client != NULL)
			ft5x06_suspend(&(factory_test_ts->client->dev));
		ft5x06_module_remove();
	}

	return n;
}

static struct kobj_attribute module_enable_attribute =
      __ATTR(module_enable, 0666, NULL, ft5x06_module_enable_store);

static struct attribute * touchpanel_attributes[] = {
	&module_enable_attribute.attr,
	NULL,
};

static struct attribute_group touchpanel_attr_group = {
	.attrs = touchpanel_attributes,
};

static struct kobject *touchpanel_kobj;

static int ft5x06_init(void)
{
	int ret = 0;

	touchpanel_kobj = kobject_create_and_add("ft5x06_driver", NULL);
	if (touchpanel_kobj)
		sysfs_create_group(touchpanel_kobj, &touchpanel_attr_group);

	ret = ft5x06_module_init();

	return ret;
}

static void ft5x06_exit(void)
{
	ft5x06_module_remove();

	sysfs_remove_group(touchpanel_kobj, &touchpanel_attr_group);
}

module_init(ft5x06_init);
module_exit(ft5x06_exit);

MODULE_DESCRIPTION("Focaltech 5x06 touchscreen driver");
MODULE_AUTHOR("B&N");
MODULE_LICENSE("GPL");

