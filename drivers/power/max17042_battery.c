/*
 * linux/drivers/power/max17042.c
 *
 * Copyright (C) 2010 Barnes & Noble, Inc.
 * Intrinsyc Software International, Inc. on behalf of Barnes & Noble, Inc.
 *
 * Max17042 Gas Gauge driver for Linux
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pm.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/max17042.h>
//#include <mach/board-boxer.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#define MODULE_TAG "MAX17042"

#define MAX8903_DEBUG 0
#if MAX8903_DEBUG
#define DBGPRINT(x...)   printk(MODULE_TAG ": " x)
#else
#define DBGPRINT(x...)
#endif // MAX8903_DEBUG
#define ENGPRINT(x...)  printk(MODULE_TAG ": " x)

#define NAME				"max17042"
#define DRIVER_VERSION			"1.0.0"
#define RESUME_ENTRIES			10
#define MONITOR_PERIOD  		30*HZ		// put it 30s or update on significant change
#define MONITOR_RESUME_PERIOD  		1*HZ		// put it 1s afer resume
#define LOWER_THRESHOLD_VOL     	3250000		// 3.25 Volt

#define DUMP_FMT_NONE 0
#define DUMP_FMT_SHORT 1
#define DUMP_FMT_LONG 2

// refresh history data whenever rsoc changes by this % : might be written to file less frequently
#define SAVE_STORE_SOC_THRESHOLD	1

// refresh history at least this often: might be written to file less frequently
#define HISTORY_REFRESH_INTERVAL	60

#define HISTORY_MAGIC		0x1234

/*leave the debug msg on until later release TBD*/
#define DEBUG(x...) 			printk(x)

// DEBUG_NO_BATTERY is used for configs without a real battery (bench supply)
// this produces no Alert signal on faults (like temperature/capacity)

//#define DEBUG_NO_BATTERY

static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct max17042_data {
	struct device *dev;
	int id;
	int voltage_uV;
	int current_uA;
	int temp_C;
	int charge_rsoc;
	int charge_status;
	int health;
	int saved_designcap;
	int saved_mixedsoc;
	int saved_dq_acc;
	int saved_dp_acc;
	int vfsoc;
	int fullcap0;
	int remcap;
	int repcap;
	int dq_acc;
	
	struct power_supply bat;

	struct i2c_client *client;
	struct delayed_work monitor_work;
	struct max17042_platform_data *pdata;
	struct work_struct irq_work;

	int hw_initialized;
	atomic_t enabled;
	int res_interval;
	int irq;
	uint32_t max17042_reg;

	int low_thr;
	//cached parameters to reduce I2C overhead
	int status;
	int volt_cached;
	int volt_average_cached;
	int curr_cached;
	int rsoc_cached;
	int rsoc_base;
	int temp_cached;
	int tte_cached;
	int vendor_cached;
	//end of cached parameters
	int turn;
	atomic_t interval;
	int is_power_supply;

	int dumpFormat;
	unsigned long next_save; // jiffies at which history should next be written
#if defined(CONFIG_DEBUG_FS)
	struct dentry *dbg_dir;
	struct dentry *dbg_active_de;
	struct dentry *dbg_capacity_de;
	struct dentry *dbg_voltage_de;
	struct dentry *dbg_health_de;
	struct dentry *dbg_charge_status_de;
	struct dentry *dbg_trigger_de;
	u32 dbg_active;
	u8 dbg_rsoc_cached;
	u32 dbg_volt_cached;
	u8 dbg_health;
	u8 dbg_charge_status;
#endif
};

static enum power_supply_property max17042_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
};

static int max17042_battery_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val);

static int max17042_battery_rsoc(struct max17042_data*);
static struct max17042_data* globe_max17042_data;

static int max17042_i2c_read(struct max17042_data *max17042, u8 addr, u8 *data, int len)
{
	int err;
	
	struct i2c_msg msgs[] = {
		{
			.addr = max17042->client->addr,
			.flags = max17042->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = max17042->client->addr,
			.flags = (max17042->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};
	err = i2c_transfer(max17042->client->adapter, msgs, 2);

	if (err != 2)
	{
		dev_err(&max17042->client->dev, "read transfer error\n");
	}
	return err;
}

static int max17042_i2c_write(struct max17042_data *max17042, u8 addr, u8 *data, int len)
{
	int err;
	int i;
	u8 buf[len + 1];

	struct i2c_msg msgs[] = {
		{
			.addr = max17042->client->addr,
			.flags = max17042->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	buf[0] = addr;
	for (i = 0; i < len; i++)
	{
		buf[i + 1] = data[i];
	}

	err = i2c_transfer(max17042->client->adapter, msgs, 1);

	if (err != 1)
	{
		dev_err(&max17042->client->dev, "write transfer error\n");
	}
	return err;
}

//wrapper of i2c read
static int max17042_read(u8 addr, u16 *data, struct max17042_data *max17042)
{
	int err;

	err = max17042_i2c_read(max17042, addr, (u8*)data, 2);
	*data = le16_to_cpu (*data);
	return err;
}

//wrapper of i2c write
static int max17042_write(u8 addr, const u16 *data, struct max17042_data *max17042)
{
	u16 buf;

	buf = cpu_to_le16 (*data);
	return max17042_i2c_write(max17042, addr, (u8*)&buf, 2);
}

//wrapper of i2c read 16 words
static int max17042_read16(u8 addr, u16 *data, struct max17042_data *max17042)
{
	int i, err;

	err = max17042_i2c_read(max17042, addr, (u8*)data, 32);

	for (i = 0; i < 16; i++)
	{
		*data = le16_to_cpu (*data);
		data++;
	}

	return err;
}

static int max17042_verify(struct max17042_data *max17042)
{
	int err;
	u16 buf;

	DBGPRINT("%s: max17042_verify ...\n", __func__);

	err =max17042_read(MAX17042_Version, &buf, max17042);
	/*** DEBUG OUTPUT - REMOVE ***/
	dev_info(&max17042->client->dev, "%s: Version = 0x%04x\n", __func__, buf);
	/*** <end> DEBUG OUTPUT - REMOVE ***/
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err int source\n", __func__);
	}

	return err;
}

static int max17042_get_status(struct max17042_data *max17042)
{
	int err;
	u16 buf;

	err =max17042_read(MAX17042_STATUS, &buf, max17042);

	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err STATUS \n", __func__);
	}
	else
	{
		max17042->status = buf;
	}

	return err;
}

static int max17042_set_config( struct max17042_data *max17042)
{
	int err;

	static const u16 config = (MAX17042_CONFIG_Ts |
#ifndef DEBUG_NO_BATTERY
	                           MAX17042_CONFIG_Aen |
#endif
	                           MAX17042_CONFIG_Ten |
	                           MAX17042_CONFIG_Etherm);

	err =max17042_write(MAX17042_CONFIG, &config, max17042);
	DEBUG("%s: config = 0x%04x\n", __func__, config);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: write err CONFIG \n", __func__);
		return err;
	}

	// most config is already set by bootloader

	return 0;	//to verfy it with le16_to_cpu()
}

static int max17042_set_thresholds( struct max17042_data *max17042)
{
	int err;

	/* voltage Valert max is disabled, min is 3.1V */
	static const u16 Valert = 0xFF9B;  // 20 mV resolution

	/* capacity Salert max is disabled, min is 5% */
	static const u16 Salert = 0xFF05; //1% resolution

	err =max17042_write(MAX17042_ValertThreshold, &Valert, max17042);
	DEBUG("%s: Valert = 0x%04x\n", __func__, Valert);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: write err Valert \n", __func__);
		goto ERR;
	}

	err =max17042_write(MAX17042_SOCalertThreshold, &Salert, max17042);
	DEBUG("%s: Salert = 0x%04x\n", __func__, Salert);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: write err Salert \n", __func__);
	}

ERR:
	return err;
}

static int max17042_set_shdntimer( struct max17042_data *max17042)
{
	int err;
	static const u16 shdntimer = 0<<14 | 0x02;	// 45s*2=90s

	err =max17042_write(MAX17042_SHDNTIMER, &shdntimer, max17042);
	DEBUG("%s: shdntimer = 0x%04x\n", __func__, shdntimer);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: write err SHDNTIMER \n", __func__);
	}
	return err;
}


static int max17042_enable_Ten ( struct max17042_data *max17042,
                                 int iEnabled )
{
	int err;
	u16 config;

	err =max17042_read(MAX17042_CONFIG, &config, max17042);
	DEBUG("%s: config = 0x%04x\n", __func__, config);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err CONFIG \n", __func__);
		return err;
	}

	// toggle the Ten bit (temperature reading enable)
	if (iEnabled)
	{
		config |= MAX17042_CONFIG_Ten;
	}
	else
	{
		config &= ~MAX17042_CONFIG_Ten;
	}

	err =max17042_write(MAX17042_CONFIG, &config, max17042);
	DEBUG("%s: config = 0x%04x\n", __func__, config);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: write err CONFIG \n", __func__);
		return err;
	}

	return 0;
}

static int max17042_enable_Aen ( struct max17042_data *max17042)
{
	u16 config;
	int err;

	err =max17042_read(MAX17042_CONFIG, &config, max17042);
	DEBUG("%s: config = 0x%04x\n", __func__, config);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err CONFIG \n", __func__);
		return err;
	}

	config |= MAX17042_CONFIG_Aen;	//enable Aen bit of CONFIG

	err =max17042_write(MAX17042_CONFIG, &config, max17042);
	DEBUG("%s: config = 0x%04x\n", __func__, config);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: write err CONFIG \n", __func__);
		return err;
	}

	return 0;
}

static int max17042_hw_init(struct max17042_data *max17042)
{
	int err = -1;

	DEBUG("%s: enter ...\n", __func__);

	err = max17042_get_status ( max17042 );

	if (err < 0)
	{
		printk("%s: get_status failed\n", __func__ );
		return err;
	}

	max17042->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	/*check to see if we are using power supply*/
	if(((max17042->status)&(1<<15))!=0)
	{
		if(((max17042->status)&(1<<11))!=0)
		{
			max17042->is_power_supply =0;
			max17042->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
			DEBUG("%s: CHARGER + BATTERY Detected!\n", __func__);
		}
		else
		{
			max17042->is_power_supply =1;
			DEBUG("%s:  POWER SUPPLY Detected!\n", __func__);
		}
	}
	else
	{
		max17042->is_power_supply =0;
		DEBUG("%s:  BATTERY      Detected!\n", __func__);
		max17042->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	err = max17042_set_config(max17042);
	if (err < 0)
	{
		return err;
	}

	//write all memory locations to starting app values
	err = max17042_set_thresholds( max17042 );	//set thesholds before Aen bit is enabled
	if (err < 0)
	{
		return err;
	}

#ifndef DEBUG_NO_BATTERY
	err = max17042_enable_Aen( max17042 );	//enable Alert pin
	if (err < 0)
	{
		return err;
	}
#endif

	err = max17042_set_shdntimer(max17042);
	if (err < 0)
	{
		return err;
	}
	max17042->hw_initialized = 1;

	return 0;
}


#if 0
static void max17042_device_power_off(struct max17042_data *max17042)
{
	int err;
	u8 buf;	//=PC1_OFF;

	DEBUG("max17042: max17042_device_power_off ...\n");

	//err = max17042_i2c_write(max17042, CTRL_REG1, &buf, 1);
	if (err < 0)
		dev_err(&max17042->client->dev, "soft power off failed\n");
	disable_irq(max17042->irq);
	if (max17042->pdata->power_off)
		max17042->pdata->power_off();
	max17042->hw_initialized = 0;
}
#endif


static int max17042_device_power_on(struct max17042_data *max17042)
{
	int err;

	DEBUG("%s: enter ...\n", __func__);

	if (max17042->pdata->power_on)
	{
		err = max17042->pdata->power_on();
		if (err < 0)
		{
			return err;
		}
		DEBUG("%s: max17042_device_power_on > power_on Ok!\n", __func__);
	}
	enable_irq(max17042->irq);

	if (!max17042->hw_initialized)
	{
		mdelay(100);
		err = max17042_hw_init(max17042);
		if (err < 0)
		{
			DEBUG("%s: max17042_device_power_on > max17042_hw_init Failed! %d\n", __func__, err);
			//max17042_device_power_off(max17042);
			return err;
		}
	}

	return 0;
}


static irqreturn_t max17042_isr(int irq, void *dev)
{
	struct max17042_data *max17042 = dev;

	disable_irq_nosync(irq);
	schedule_work(&max17042->irq_work);

	return IRQ_HANDLED;
}


//Alert pin will trigger wakeup to OMAP, so not much to be done here
static void max17042_irq_work_func(struct work_struct *work)
{
	int err;
	u16 buf =0xFFFF;
	//to prevent multiple interrupts on violation of temperature threshold

	struct max17042_data *max17042 = container_of(work, struct max17042_data, irq_work);

	//DEBUG("max17042: max17042_irq_work_func ...\n");
	/*battery presence is false and we are running, must be power supply, no need to do anything*/
	err = max17042_get_status(max17042);
	if (err < 0)
	{
		//DEBUG("max17042: max17042_irq_work_func ... read err status %d\n", err);
		return;
	}
	buf= max17042->status;		//save it

	//clear Br, Bi
	buf &= ~(1<<15 | 1<<11);

	//clear the bits
	if (max17042->status & (1<<8))
	{
		DEBUG("%s: Min Vol threshold exceeded!\n", __func__);
		buf &= ~(1<<8);		//Vmn
	}

	if (max17042->status & (1<<9))
	{
		DEBUG("%s: Min Temp threshold exceeded!\n", __func__);
		buf &= ~(1<<9);		//Tmn
	}

	if (max17042->status & (1<<10))
	{
		DEBUG("%s: Min SOC threshold exceeded!\n", __func__);
		buf &= ~(1<<10);		//Smn
		//update cached SOC right away in case SOC is low
		if (max17042_battery_rsoc(max17042) < 0)
			max17042->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		//kick the monitor to send event out
		cancel_delayed_work_sync(&max17042->monitor_work);
		schedule_delayed_work(&max17042->monitor_work, 0);
	}

	if (max17042->status & (1<<12))
	{
		DEBUG("%s: Max Vol threshold exceeded!\n", __func__);
		buf &= ~(1<<12);		//Vmx
	}

	if (max17042->status & (1<<14))
	{
		DEBUG("%s: Max SOC threshold exceeded!\n", __func__);
		buf &= ~(1<<14);		//Smx
	}

	err =max17042_write(MAX17042_STATUS, &buf, max17042);
	DEBUG("%s: status = 0x%04x\n", __func__, buf);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: write err STATUS \n", __func__);
	}
	else
	{
		max17042->status = buf;
	}

	//send out uevent and inform charger driver to take action
	cancel_delayed_work_sync(&max17042->monitor_work);
	schedule_delayed_work(&max17042->monitor_work, 0);

	enable_irq(max17042->irq);
}

static int max17042_enable(struct max17042_data *max17042)
{

	DEBUG("%s: enter ...\n", __func__);

	//note: look into possible race condition here.
	if (!atomic_cmpxchg(&max17042->enabled, 0, 1)) {
#if 0
		err = max17042_device_power_on(max17042);
		if (err < 0) {
			dev_err(&max17042->client->dev,
			        "error clearing interrupt: %d\n", err);
			atomic_set(&max17042->enabled, 0);
			return err;
		}
#endif
		schedule_delayed_work(&max17042->monitor_work, MONITOR_RESUME_PERIOD);
	}

	return 0;
}

static int max17042_disable(struct max17042_data *max17042)
{
	DEBUG("%s: enter ...\n", __func__);

	if (atomic_cmpxchg(&max17042->enabled, 1, 0)) {
		cancel_delayed_work_sync(&max17042->monitor_work);
		//max17042_device_power_off(max17042);
	}

	return 0;
}

static void max17042_powersupply_init(struct max17042_data *max17042)
{
	max17042->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	max17042->bat.properties = max17042_battery_props;
	max17042->bat.num_properties = ARRAY_SIZE(max17042_battery_props);
	max17042->bat.get_property = max17042_battery_get_property;
	max17042->bat.external_power_changed = NULL;
}
#if defined(CONFIG_DEBUG_FS)
#define MAKE_REG(r) dbg_ ## r
#define DBG_PROPERTY(r, m) (m->dbg_active ? m->MAKE_REG(r) : m->r)
#else
#define DBG_PROPERTY(r, m) (m->r)
#endif

static int max17042_battery_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	//DEBUG(KERN_NOTICE "bq27x00_battery_get_property()\n");

	struct max17042_data *max17042 = container_of((psy), struct max17042_data, bat);

	//NOTE: sort out first round of data update below, force to read and cache data.
	switch (psp)
	{
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = DBG_PROPERTY(charge_status, max17042);
		dev_dbg(max17042->dev, "%s: Status of max17042 is %x\n", __func__, max17042->charge_status);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = DBG_PROPERTY(health, max17042);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = DBG_PROPERTY(volt_cached, max17042);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
#if defined(CONFIG_DEBUG_FS)
		if (max17042->dbg_active)
			val->intval = max17042->dbg_volt_cached;
		else
#endif
		val->intval = max17042->volt_average_cached;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		//Bst(0) of Reg_00h inidcates battery presence
		if(!(max17042->status & (1<<3) && max17042->health != POWER_SUPPLY_HEALTH_UNKNOWN))
		{
			val->intval = 1;
		}
		else
		{
			val->intval = 0;
		}
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = max17042->curr_cached;
		//DEBUG(KERN_NOTICE "bq27x00_battery_get_property(), curr(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (max17042->rsoc_cached > 100)
		{
			// DEBUG(KERN_NOTICE "bq27x00_battery_get_property: capacity (%d) capped at 100%%\n", max17042->rsoc_cached );
			max17042->rsoc_cached = 100;
		}
		val->intval = DBG_PROPERTY(rsoc_cached, max17042);
		//DEBUG(KERN_NOTICE "bq27x00_battery_get_property(), rsoc(%d)\n", val->intval);
		
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = (max17042->temp_cached/100000);
		//DEBUG(KERN_NOTICE "bq27x00_battery_get_property(), temp(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = max17042->tte_cached;
		//DEBUG(KERN_NOTICE "bq27x00_battery_get_property(), tte(%d)\n", val->intval);
		break;
	default:
		dev_dbg(max17042->dev, "%s: Unknown psp number is %x\n", __func__, psp);
		return -EINVAL;
	}

	return 0;
}

//Vcell is 5.12v~0v with step of 0.625mV, interval 178.5ms
static int max17042_battery_voltage(struct max17042_data* max17042)
{
	int err;
	u16 volt = 0;

	err =max17042_read(MAX17042_Vcell, &volt, max17042);
	//DEBUG("volt = 0x%04x\n", volt);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err Vcell \n", __func__);
	}
	else
	{
		max17042->volt_cached = (volt>>3)*VOLT_RESOLUTION;	//in unit of uV
	}

	return err;
}

// AverageVcell is 5.12v~0v with step of 0.625mV, interval 178.5ms
static int max17042_battery_voltage_average(struct max17042_data* max17042)
{
	int err;
	u16 volt = 0;

	err = max17042_read(MAX17042_AverageVcell, &volt, max17042);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err AverageVcell \n", __func__);
	}
	else
	{
		max17042->volt_average_cached = (volt>>3)*VOLT_RESOLUTION;	//in unit of uV
	}

	return err;
}

#define COMPLEMENT_VAL(x, y, z)		( ( ((((~(x)) & 0x7FFF) + 1) * (y))  / (z) ) * (-1) )
#define NON_COMPLEMENT_VAL(x, y, z)	( ((((x) & 0x7FFF)) * (y))  / (z) )

//sense resistor is 0.01ohm, current register resolution is 156.25uA
static int max17042_battery_current(struct max17042_data* max17042)
{
	int err;
	u16 curr = 0;

	err =max17042_read(MAX17042_Current, &curr, max17042);
	//DEBUG("curr = 0x%04x\n", curr);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err Current \n", __func__);
	}
	else
	{
		//in unit of uA,
		if (curr & (1<<15))
	       	{
			//negtive value
			max17042->curr_cached = COMPLEMENT_VAL(curr, CURR_RESOLUTION, 100);
		}
		else
		{
			//positive value
			max17042->curr_cached = NON_COMPLEMENT_VAL(curr, CURR_RESOLUTION, 100);
		}
	}

	return err;
}

//resolution 0.0039-degree, or 3900uC
static int max17042_battery_temperature(struct max17042_data* max17042)
{
	int err = 0;
	u16 temp = 0;
	err =max17042_read(MAX17042_Temperature, &temp, max17042);
	//DEBUG("temp = 0x%04x\n", temp);

	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err Temperature \n", __func__);
	}
	else
	{
		// in unit of micro-degree,
		if (temp & (1<<15))
		{
			//negtive value
			max17042->temp_cached = COMPLEMENT_VAL(temp, TEMP_RESOLUTION, 1);
			//(~(temp & 0x7FFF) + 1) * TEMP_RESOLUTION * (-1);
		}
		else
		{
			//positive value
			max17042->temp_cached = NON_COMPLEMENT_VAL(temp, TEMP_RESOLUTION, 1);
			//(temp & 0x7FFF) * TEMP_RESOLUTION;
		}
	}

	return err;
}

//take upper byte only for resulotion 1%
static int max17042_battery_rsoc(struct max17042_data* max17042)
{
	int err;
	u16 soc, socrep;

	soc = socrep = 0;

	err =max17042_read(MAX17042_SOCREP, &socrep, max17042);
	//DEBUG("soc = 0x%04x\n", soc);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err SOCREP \n", __func__);
	}
	else
	{
		soc = socrep >> 8; // take upper byte (bit 8 is 1%)

		// round up if xx.5% or greater (bit 7 is 0.5%)
		if (socrep & (1<<7))
		{
			soc++;
		}

		if (soc>100)
		{
			soc = 100;
		}
		max17042->rsoc_cached = soc;
	}

	return err;
}

//resolution 5.625s
static int max17042_battery_tte(struct max17042_data* max17042)
{
	int err;
	u16 tte = 0;

	err =max17042_read(MAX17042_TTE, &tte, max17042);
	//DEBUG("tte = 0x%04x\n", tte);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err TTE \n", __func__);
	}
	else
	{
		max17042->tte_cached = (tte * TTE_RESOLUTION) /1000;
	}

	return err;
}

#define	NUM_SAVE_RESTORE 20

//TODO: refactor the array below
static u16 max_buf[NUM_SAVE_RESTORE] = {
					0x0, 0x2206, 0x0001,0x2206, 0x0000,0x0100, 0x006C,0x1420, 0x0652,
                                        0x0000, 0x0000,0x3200, 0x1224,0x0520, 0xA053, 0x87A4, 0x1400,
                                        0x2008, 0x2008,0x2200
                                       };

static int max17042_save_reg(struct max17042_data* max17042)
{
	int err;
	static const u16 tag = HISTORY_MAGIC;
	int i=0;

	max_buf[i++] = tag;

	err =max17042_read(MAX17042_FullCAP, &max_buf[i++], max17042);
	//DEBUG("fullcap = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err FullCAP \n", __func__);
		return err;
	}

	err =max17042_read(MAX17042_Cycles, &max_buf[i++], max17042);
	//DEBUG("Cycles = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err Cycles \n", __func__);
		return err;
	}

	err =max17042_read(MAX17042_FullCAPNom, &max_buf[i++], max17042);
	//DEBUG("fullcapnom = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err FullCAPNom \n", __func__);
		return err;
	}

	err =max17042_read(MAX17042_SOCempty, &max_buf[i++], max17042);
	//DEBUG("socempty = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err SOCempty \n", __func__);
		return err;
	}

	err =max17042_read(MAX17042_Iavg_empty, &max_buf[i++], max17042);
	//DEBUG("Iavgempty = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err Iavg_empty \n", __func__);
		return err;
	}

	err =max17042_read(MAX17042_RCOMP0, &max_buf[i++], max17042);
	//DEBUG("rcomp0 = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err RCOMP0 \n", __func__);
		return err;
	}

	err =max17042_read(MAX17042_TempCo, &max_buf[i++], max17042);
	//DEBUG("tempco = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err TempCo \n", __func__);
		return err;
	}

	err =max17042_read(MAX17042_k_empty0, &max_buf[i++], max17042);
	//DEBUG("kempty0 = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err k_empty0 \n", __func__);
		return err;
	}

	max_buf[i++] = 0x0000;	//dQacc, don't save them as we calculate them on restore
	max_buf[i++] = 0x0000;	//dPacc,  don't save them as we calculate them on restore

	//this is for step 23 and step 3.
	err =max17042_read(MAX17042_SOCmix, &max_buf[i++], max17042);
	//DEBUG("socmix = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err SOCmix \n", __func__);
		return err;
	}

	//this is for step10, ETC
	err =max17042_read(MAX17042_Empty_TempCo, &max_buf[i++], max17042);
	//DEBUG("ETC = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err ETC \n", __func__);
		return err;
	}

	//this is for step 10, ICHGTerm
	err =max17042_read(MAX17042_ICHGTerm, &max_buf[i++], max17042);
	//DEBUG("ichgterm = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err ICHGTerm \n", __func__);
		return err;
	}

	//this is for step 10
	err =max17042_read(MAX17042_Vempty, &max_buf[i++], max17042);
	//DEBUG("vempty = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err Vempty \n", __func__);
		return err;
	}

	//this is for step 10
	err =max17042_read(MAX17042_FilterCFG, &max_buf[i++], max17042);
	//DEBUG("filtercfg = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err FilterCFG \n", __func__);
		return err;
	}

	//this is for step 10
	err =max17042_read(MAX17042_TempNom, &max_buf[i++], max17042);
	//DEBUG("tempnom = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err TempNom \n", __func__);
		return err;
	}

	//this is for step3, 11 and 16
	err =max17042_read(MAX17042_DesignCap, &max_buf[i++], max17042);
	//DEBUG("designcap = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err DesignCap \n", __func__);
		return err;
	}

	//this is for Capacity in all steps
	max_buf[i] = max_buf[i-1]; 	//copy DesignCap to Capacity
	i++;

	err =max17042_read(MAX17042_SOCREP, &max_buf[i++], max17042);
	//DEBUG("socrep = 0x%04x\n", max_buf[i-1]);
	if (err < 0)
	{
		dev_err(&max17042->client->dev, "%s: read err SOCREP \n", __func__);
		return err;
	}

	//for (i=0; i < NUM_SAVE_RESTORE; i++ ) {
	//	DEBUG(KERN_INFO " item %d val %04x\n", i, max_buf[i]);
	//}

	return 0;
}

void max17042_update_callback(u8 charger_in)
{

	if(!globe_max17042_data)
	{
		return;
	}
	/*cahrge status is updated ONLY by charger when there is an AC/USB event*/
	globe_max17042_data->charge_status = (charger_in)? POWER_SUPPLY_STATUS_CHARGING:POWER_SUPPLY_STATUS_DISCHARGING;
	power_supply_changed(&globe_max17042_data->bat);
	return;
}

EXPORT_SYMBOL(max17042_update_callback);

static void max17042_battery_status_monitor(struct work_struct *work)
{
	struct max17042_data *max17042 = container_of(work, struct max17042_data, monitor_work.work);
	int err;
	u8 is_save = 0;
	int i;

	/*battery presence is false and we are running,
	 must be power supply, no need to do anything*/
	if (max17042->is_power_supply)
	{
		DEBUG("%s: no need to monitor.Power Supply is being used!\n", __func__);
		return;
	}

	max17042->health = POWER_SUPPLY_HEALTH_GOOD;
	err = max17042_get_status ( max17042 );
	if (err < 0)
	{
		//DEBUG("max17042: status monitor ... read err status %d\n", err);
		max17042->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		return;
	}
	else
	{
		u16 buf;
		buf= max17042->status;		//save it

		//since we set Temperature Ts bit of Reg_1Dh as sticky,
		//we need to monitor Tmn/Tmx to clear them once the threshold is not violated
		buf &= ~ ( (1<<15) | (1<<11) | (1<<9) | (1<<13) );	//Br|Bi|Tmn|Tmx

		if (max17042->status & ((1<<15) | (1<<11) | (1<<9) | (1<<13)))
		{
			err =max17042_write(MAX17042_STATUS, &buf, max17042);
			DEBUG("%s: monitor upate status = 0x%04x\n", __func__, buf);
			if (err < 0)
			{
				dev_err(&max17042->client->dev, "%s: write err STATUS \n", __func__);
			}
			else
			{
				max17042->status = buf;
			}
		}
	}

	if (max17042_battery_rsoc(max17042) < 0)
	{
		max17042->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		printk( KERN_ERR "%s: failed to read rsoc\n", __func__);
	}

	if (abs(max17042->rsoc_cached - max17042->rsoc_base ) >= SAVE_STORE_SOC_THRESHOLD)
	{
		is_save = 1;
		max17042->rsoc_base = max17042->rsoc_cached; //rebase
	}

	if ((max17042_battery_voltage(max17042) < 0)
			|| (max17042_battery_voltage_average(max17042) < 0))
	{
		max17042->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	else if (max17042->volt_average_cached < LOWER_THRESHOLD_VOL)
	{
		// Force framework to shutdown.
		max17042->health = POWER_SUPPLY_HEALTH_DEAD;
		max17042->rsoc_cached = 0;
	}

	// read all the parameters
	if (max17042_battery_temperature(max17042) < 0)
	{
		max17042->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		printk( KERN_ERR "%s: failed to read temperature\n", __func__);
	}

	if (max17042_battery_tte(max17042) < 0)
	{
		max17042->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		printk( KERN_ERR "%s: failed to read tte\n", __func__);
	}

	if (max17042_battery_current(max17042) < 0)
	{
		max17042->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		printk( KERN_ERR "%s: failed to read current\n", __func__);
	}

	if ((max17042->dumpFormat == DUMP_FMT_SHORT) || (max17042->dumpFormat == DUMP_FMT_LONG))
	{
		DEBUG("%s: rsoc:%d  volt:%d  curr:%d  temp:%d  tte:%d chg:%s\n",
				__func__,
	  			max17042->rsoc_cached, max17042->volt_cached,
	  			max17042->curr_cached, max17042->temp_cached,
	  			max17042->tte_cached,
	  			(max17042->charge_status == POWER_SUPPLY_STATUS_CHARGING) ? "charging" : "discharging"
		     );
	}

	if (max17042->dumpFormat == DUMP_FMT_LONG)
	{
		/* read and dump regs from 0x00 to 0x50 */
		int i,j;
#define DUMPSZ 16
		u16 buf[DUMPSZ];
		char szbuf[DUMPSZ*7+10], *pSz;

		i = 0;
		while (i < 0x50)
		{
			if (max17042_read16( i, buf, max17042 ) == 2)
			{
				pSz = szbuf;
				for (j=0; j<DUMPSZ; j++)
				{
					pSz += sprintf(pSz, ",0x%04x", buf[j]);
				}
				DEBUG("%s: max17042_dump,%d%s\n", __func__, i,szbuf);
			}
			else
			{
				DEBUG(KERN_ERR "%s: read 16 failed at %d\n", __func__, i);
			}
			i+= DUMPSZ;
		}
	}

	if (time_after(jiffies,max17042->next_save))
	{
		// DEBUG("max17042_monitor: save history interval expired (%lu > %lu)\n", jiffies, max17042->next_save);
		is_save = 1;
	}

	if (is_save)
	{
		max17042_save_reg(max17042);	//save register values into buffer
		max17042->next_save = jiffies + HISTORY_REFRESH_INTERVAL*HZ;

		// DEBUG("max17042_monitor: next history @ %lu\n", max17042->next_save);
	}

	power_supply_changed(&max17042->bat);
	i = atomic_read(&max17042->interval);

	if (max17042->dumpFormat == DUMP_FMT_SHORT)
	{
		// DEBUG("MAX17042 scheduling next poll in %d HZ\n",(i/HZ));
	}

	schedule_delayed_work(&max17042->monitor_work,i);
}

static ssize_t max17042_r_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 rtval = 0x0;
	int err = 0;
	struct max17042_data *max17042 = dev_get_drvdata(dev);

	err = max17042_read(max17042->max17042_reg, &rtval, max17042);
	if (err < 0)
	{
		DEBUG(KERN_ERR "%s: max17042 reg value read errror.", __func__);
	}

	return sprintf(buf, "0x%x\n",  rtval);
}

static ssize_t max17042_w_val(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u16 val = (u16)simple_strtoul(buf, NULL, 16);
	int err;
	struct max17042_data *max17042 = dev_get_drvdata(dev);

	if(val > 0xffff)
	{
		return -EINVAL;
	}

	err = max17042_write(max17042->max17042_reg, &val, max17042);
	if (err < 0)
	{
		DEBUG(KERN_ERR "%s: max17042 reg value write errror.", __func__);
	}

	return count;
}

static DEVICE_ATTR(val, S_IWUSR | S_IRUGO, max17042_r_val, max17042_w_val);

static ssize_t max17042_r_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max17042_data *max17042 = dev_get_drvdata(dev);

	return sprintf(buf, "0x%.2x\n", max17042->max17042_reg);
}

static ssize_t max17042_w_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 16);
	struct max17042_data *max17042 = dev_get_drvdata(dev);

	if(val > 0xffff)
	{
		return -EINVAL;
	}

	max17042->max17042_reg = val;

	return count;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, max17042_r_reg, max17042_w_reg);
/****************************************/
/* addition of polling interval control */
/****************************************/

/*writing the interval to sysfs*/
static ssize_t max17042_w_interval(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	struct max17042_data *max17042 = dev_get_drvdata(dev);

	if(val > 0xffff)
	{
		return -EINVAL;
	}

	atomic_set(&max17042->interval,val*HZ);
	printk("%s: polling interval writes %lu HZ \n", __func__, val);
	return count;
}
/*getting the polling interval from sysfs*/
static ssize_t max17042_r_interval(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max17042_data *max17042 = dev_get_drvdata(dev);
	unsigned long interval;

	interval=(atomic_read(&max17042->interval)/HZ);

	return sprintf(buf, "MAX17042: polling interval reads %lu HZ\n", interval);
}
static DEVICE_ATTR(interval, S_IWUSR | S_IRUGO, max17042_r_interval, max17042_w_interval);

static ssize_t max17042_w_dumpformat(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct max17042_data *max17042 = dev_get_drvdata(dev);

	if (!count)
	{
		return -EINVAL;
	}

	switch (toupper(buf[0]))
	{
	case 'N':
		printk("%s: dump disabled\n", __func__);
		max17042->dumpFormat = DUMP_FMT_NONE;
		break;
	case 'S':
		printk("%s: set to short dump format\n", __func__);
		max17042->dumpFormat = DUMP_FMT_SHORT;
		break;
	case 'L':
		printk("%s: set to long dump format\n", __func__);
		max17042->dumpFormat = DUMP_FMT_LONG;
		break;
	default:
		printk(KERN_ERR "%s: dumpformat [ long | short | none ]\n", __func__);
		return -EINVAL;
	}

	return count;
}
static ssize_t max17042_r_dumpformat(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	struct max17042_data *max17042 = dev_get_drvdata(dev);

	switch (max17042->dumpFormat)
	{
	case DUMP_FMT_NONE:
		count = sprintf(buf,"none");
		break;
	case DUMP_FMT_SHORT:
		count = sprintf(buf,"short");
		break;
	case DUMP_FMT_LONG:
		count = sprintf(buf,"long");
		break;
	default:
		count = sprintf(buf,"unknown");
	}

	return count;
}
static DEVICE_ATTR(dumpformat, S_IWUSR | S_IRUGO, max17042_r_dumpformat, max17042_w_dumpformat);

/****************************************/
/* history data published to sysfs      */
/****************************************/

/*getting the polling interval from sysfs*/
static ssize_t max17042_r_histdata(struct device *dev, struct device_attribute *attr, char *buf)
{
	if ( max_buf[0] != HISTORY_MAGIC ) {
		printk("max17042: error: history data uninitialized\n");
		return 0;
	}

	memcpy(buf, (const char *)max_buf, sizeof(max_buf));

	return sizeof(max_buf);
}
static DEVICE_ATTR(histdata, S_IRUGO, max17042_r_histdata, NULL);

static struct attribute *max17042_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_val.attr,
	&dev_attr_interval.attr,
	&dev_attr_dumpformat.attr,
	&dev_attr_histdata.attr,
	NULL
};

static struct attribute_group max17042_attribute_group = {
	.attrs = max17042_attributes
};

/* /sysfs */

#if defined(CONFIG_DEBUG_FS)
static int max17042_dbg_trigger(void *data, u64 val)
{
	struct max17042_data *max17042 = (struct max17042_data *) data;
	if (!max17042)
	{
		return -EINVAL;
	}

	power_supply_changed(&max17042->bat);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(max17042_dbg_fops, NULL, max17042_dbg_trigger, "%llu\n");
#endif


//NOTE: sort out battery_mutex usage below
static int __devinit max17042_probe(struct i2c_client *client,
                                    const struct i2c_device_id *id)
{
	char *name;
	int num = 0;
	int retval = 0;
	int err = -1;
	struct max17042_data *max17042;

	DEBUG("%s: enter ...\n", __func__);

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
	{
		return -ENOMEM;
	}
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
	{
		return retval;
	}

	name = kasprintf(GFP_KERNEL, "max17042-%d", num);
	if (!name)
	{
		dev_err(&client->dev, "%s: failed to allocate device name\n", __func__);
		retval = -ENOMEM;
		goto err0;
	}
	//DEBUG("max17042_probe: max17042->bat.name(%s)\n", name);

	max17042 = kzalloc(sizeof(*max17042), GFP_KERNEL);
	if (max17042 == NULL)
	{
		dev_err(&client->dev, "%s: failed to allocate memory for module data\n", __func__);
		err = -ENOMEM;
		goto err1;
	}
	if (client->dev.platform_data == NULL)
	{
		dev_err(&client->dev, "%s: platform data is NULL; exiting\n", __func__);
		err = -ENODEV;
		goto err2;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dev_err(&client->dev, "%s: client not i2c capable\n", __func__);
		err = -ENODEV;
		goto err2;
	}
	//mutex_lock(&battery_mutex);
	max17042->client = client;
	max17042->dev = &client->dev;
	max17042->bat.name = name;
	max17042->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	max17042->rsoc_base = max17042->rsoc_cached = 100;
	max17042->volt_average_cached = max17042->volt_cached = 4000000;
	max17042->curr_cached = 300000;
	max17042->temp_cached = 25000000;
	max17042->dumpFormat = DUMP_FMT_NONE;
	max17042->next_save = jiffies + HISTORY_REFRESH_INTERVAL*HZ;

	DEBUG("%s: next history save at %lu\n", __func__, max17042->next_save);

	i2c_set_clientdata(client, max17042);
	atomic_set(&max17042->interval,MONITOR_PERIOD);

	//DEBUG("%s: max17042_probe > init irq work function ...\n", __func__);
	INIT_WORK(&max17042->irq_work, max17042_irq_work_func);
	max17042->pdata = kmalloc(sizeof(*max17042->pdata), GFP_KERNEL);
	if (max17042->pdata == NULL)
	{
		goto err2;
	}

	err = sysfs_create_group(&client->dev.kobj, &max17042_attribute_group);
	if (err)
	{
		goto err3;
	}
	//DEBUG("max17042: max17042_probe > Create sysfs_group successfully!\n");
	memcpy(max17042->pdata, client->dev.platform_data, sizeof(*max17042->pdata));
	if (max17042->pdata->init)
	{
		err = max17042->pdata->init();
		if (err < 0)
		{
			goto err4;
		}
	}
	//DEBUG("max17042: max17042_probe > Copy platform_data successfully!\n");

	max17042->irq = gpio_to_irq(max17042->pdata->gpio);
	DEBUG("%s: IRQ is %d\n", __func__, max17042->irq);
	DEBUG("%s: max17042 GPIO pin read %d\n", __func__, gpio_get_value(max17042->pdata->gpio));

	INIT_DELAYED_WORK(&max17042->monitor_work, max17042_battery_status_monitor);
	schedule_delayed_work(&max17042->monitor_work, 5*HZ);

	err = max17042_device_power_on(max17042);
	if (err < 0)
	{
		goto err4;
	}
	atomic_set(&max17042->enabled, 1);
	//DEBUG("max17042: max17042_probe > max17042 enabled!\n");

	err = max17042_verify(max17042);
	if (err < 0)
	{
		dev_err(&client->dev, "%s: unresolved i2c client\n", __func__);
		goto err5;
	}

	//max17042_device_power_off(max17042);
	atomic_set(&max17042->enabled, 0);
	//DEBUG("max17042: max17042_probe > max17042 disabled!\n");

	err = request_irq(max17042->irq, max17042_isr,
	                  IRQF_TRIGGER_FALLING | IRQF_DISABLED, "max17042-irq", max17042);
	//DEBUG("max17042: max17042_probe > Request IRQ successfully!\n");

	if (err < 0)
	{
		pr_err("%s: request irq failed: %d\n", __func__, err);
		goto err5;
	}

	max17042_powersupply_init(max17042);

	retval = power_supply_register(&client->dev, &max17042->bat);
	if (retval)
	{
		dev_err(&client->dev, "%s: failed to register battery\n", __func__);
		goto err5;
	}

	dev_info(&client->dev, "%s: support ver. %s enabled\n", __func__, DRIVER_VERSION);

	//mutex_unlock(&battery_mutex);

	max17042_save_reg(max17042);	//save register values into buffer

	globe_max17042_data = max17042; //used only for charger update
#if defined(CONFIG_DEBUG_FS)
	max17042->dbg_dir = debugfs_create_dir("max17042", NULL);

	if (!IS_ERR(max17042->dbg_dir))
	{
		max17042->dbg_rsoc_cached = 90;
		max17042->dbg_volt_cached = 4000000;
		max17042->dbg_health = POWER_SUPPLY_HEALTH_GOOD;
		max17042->dbg_charge_status = POWER_SUPPLY_STATUS_DISCHARGING;

		max17042->dbg_active_de = debugfs_create_bool("active", 0600, max17042->dbg_dir, &max17042->dbg_active);
		max17042->dbg_capacity_de = debugfs_create_u8("capacity", 0600, max17042->dbg_dir, &max17042->dbg_rsoc_cached);
		max17042->dbg_voltage_de = debugfs_create_u32("voltage", 0600, max17042->dbg_dir, &max17042->dbg_volt_cached);
		max17042->dbg_health_de = debugfs_create_u8("health", 0600, max17042->dbg_dir, &max17042->dbg_health);
		max17042->dbg_charge_status_de = debugfs_create_u8("charge_status", 0600, max17042->dbg_dir, &max17042->dbg_charge_status);
		max17042->dbg_trigger_de = debugfs_create_file("trigger", 0600, max17042->dbg_dir,max17042, &max17042_dbg_fops);

		if (IS_ERR(max17042->dbg_active_de) ||
		IS_ERR(max17042->dbg_capacity_de) ||
		IS_ERR(max17042->dbg_voltage_de) ||
		IS_ERR(max17042->dbg_health_de) ||
		IS_ERR(max17042->dbg_charge_status_de) ||
		IS_ERR(max17042->dbg_trigger_de)) {
			dev_warn(&client->dev, "failed to create debug entries");
		}
	}
	else
	{
		dev_warn(&client->dev, "failed to create debug dir: %ld", PTR_ERR(max17042->dbg_dir));
	}

#endif
	return 0;

//TODO: refactor error handling code here
err5:
	//max17042_device_power_off(max17042);
err4:
	if (max17042->pdata->exit)
	{
		max17042->pdata->exit();
	}
	sysfs_remove_group(&client->dev.kobj, &max17042_attribute_group);
err3:
	kfree(max17042->pdata);
err2:
	//mutex_unlock(&battery_mutex);
	kfree(max17042);
err1:
	kfree(name);
err0:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return err;
}

static int __devexit max17042_remove(struct i2c_client *client)
{
	struct max17042_data *max17042 = i2c_get_clientdata(client);

#if defined(CONFIG_DEBUG_FS)
	if (!IS_ERR(max17042->dbg_dir))
	{
		debugfs_remove_recursive(max17042->dbg_dir);
	}
#endif
	power_supply_unregister(&max17042->bat);
	sysfs_remove_group(&client->dev.kobj, &max17042_attribute_group);

	//NOTE: refactor chage_status as it may not needed
	max17042->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	cancel_delayed_work(&max17042->monitor_work);
	kfree(max17042->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, max17042->id);
	mutex_unlock(&battery_mutex);

	free_irq(max17042->irq, max17042);
	gpio_free(max17042->pdata->gpio);
	//max17042_device_power_off(max17042);
	if (max17042->pdata->exit)
	{
		max17042->pdata->exit();
	}
	kfree(max17042->pdata);
	kfree(max17042);

	return 0;
}

static int max17042_resume(struct i2c_client *client)
{
	struct max17042_data *max17042 = i2c_get_clientdata(client);

	// re-enable temperature monitoring
	max17042_enable_Ten(max17042, 1);

	return max17042_enable(max17042);
}

static int max17042_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct max17042_data *max17042 = i2c_get_clientdata(client);

	// turn off temperature monitoring during suspend

	max17042_enable_Ten(max17042, 0);

	return max17042_disable(max17042);
}

static void max17042_shutdown(struct i2c_client *client)
{
	struct max17042_data *max17042 = i2c_get_clientdata(client);

	disable_irq(max17042->irq);
	cancel_work_sync(&max17042->irq_work);
	// In case interrupts got enabled again by the irq_work
	disable_irq(max17042->irq);

	max17042_disable(max17042);
}

static const struct i2c_device_id max17042_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, max17042_id);

static struct i2c_driver max17042_driver = {
	.driver = {
		.name = NAME,
	},
	.probe = max17042_probe,
	.remove = __devexit_p(max17042_remove),
	.resume = max17042_resume,
	.suspend = max17042_suspend,
	.shutdown = max17042_shutdown,
	.id_table = max17042_id,
      };

static int __init max17042_init(void)
{
	DEBUG("max17042: max17042_init ...\n");
	return i2c_add_driver(&max17042_driver);
}

static void __exit max17042_exit(void)
{
	i2c_del_driver(&max17042_driver);
}

module_init(max17042_init);
module_exit(max17042_exit);

MODULE_AUTHOR("Intrinsyc Software Inc., <support@intrinsyc.com>");
MODULE_DESCRIPTION("MAX17042 Battery Gas Gauge");
MODULE_LICENSE("GPL");

