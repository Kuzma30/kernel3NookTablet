/*
 * linux/drivers/power/max8903_charger.c
 *
 * Copyright (C) 2010 Barnes & Noble, Inc.
 * 
 * Lei Cao
 * lcao@book.com
 *
 * Max8903 PMIC driver for Linux
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/max8903.h>
#include <linux/i2c/twl.h>
#include <linux/usb/otg.h>

#define DEBUG(x...)   printk(x)
//#define DEBUG(x...)

/*amperage def*/
#define USB_CURRENT_LIMIT_LOW    100   /* mA */
#define USB_CURRENT_LIMIT_HIGH   500   /* mA */
#define AC_CURRENT_LIMIT        1900   /* mA */

#define CHG_ILM_SELECT_AC   1
#define CHG_ILM_SELECT_USB  0

#define CHG_IUSB_SELECT_500mA   1
#define CHG_IUSB_SELECT_100mA   0

/*polarity def*/
#define ENABLED   0
#define DISABLED  1

extern void max17042_update_callback(u8 charger_in);

DEFINE_MUTEX(charger_mutex);

struct max8903_charger {
    struct delayed_work max8903_charger_detect_work;
    struct notifier_block nb;
    struct otg_transceiver *otg;
    unsigned int detected_charger_current;
    unsigned long detected_event;
    unsigned int detected_charger;
    unsigned int usb_max_power;
    int adapter_active;
    int adapter_online;
    int adapter_curr_limit;
    int usb_active;
    int usb_online;
    u8 temperature_ok;
    u8 ftm;                /*Factory Test Mode, in this mode charger on/off is controlled by sysfs entry "ctrl_charge"*/
    int flt_irq;
    // Init from resources
    unsigned int max8903_gpio_chg_en;
    unsigned int max8903_gpio_chg_status;
    unsigned int max8903_gpio_chg_flt;
    unsigned int max8903_gpio_chg_iusb;
    unsigned int max8903_gpio_chg_usus;
    unsigned int max8903_gpio_chg_ilm;
    unsigned int max8903_gpio_chg_uok;
    unsigned int max8903_gpio_chg_dok;
    struct power_supply usb;
    struct power_supply adapter;
    struct delayed_work usb_work;
};

static inline void reset_charger_detect_fsm(void)
{
}

/*~CEN control*/
void max8903_enable_charge(struct max8903_charger *charger, u8 enable)
{
    int prev_status;

    /*this code will be triggered for FTM only*/
    if (charger->ftm)
    {
        if (enable) {
                printk("MAX8903-TESTING: Charger is now On !\n");
                gpio_set_value(charger->max8903_gpio_chg_en,ENABLED);
        }
        else {
                printk("MAX8903-TESTING: Charger is now Off !\n");
                gpio_set_value(charger->max8903_gpio_chg_en,DISABLED);
        }
        return;
    }

    /*probe previous charging status*/
    mutex_lock(&charger_mutex);
    prev_status = gpio_get_value(charger->max8903_gpio_chg_en);

    switch(prev_status)
    {
        case ENABLED:
            if(!enable) {
                gpio_set_value(charger->max8903_gpio_chg_en,DISABLED);
                printk("MAX8903: Charging is now disabled!\n");
            }
            break;
        default:
        case DISABLED:
            if (enable) {
                printk("MAX8903: Charging is now enabled!\n");
                gpio_set_value(charger->max8903_gpio_chg_en,ENABLED);
            }
            break;
    }

    mutex_unlock(&charger_mutex);
}

/*charger differentiation*/
/*                             
  5  types of charger:
   
  I.   DC charger:  DC-CHG only, we charge via DC input limit 2A   ( development use ONLY)
  II.  BN charger:  VBUS/DC signal,D+/D- shorted, we charge via DC input,limit 2A
  III. USB charger: VBUS/DC signal,D+/D- not shorted,charge via VBUS,limit 500mA
  IV.  USB charger: VBUS only,D+/D- not shorted,charge via VBUS,limit 500mA 
  V.   USB charger: VBUS only,D+/D- shorted,charge via VBUS,limit 500mA
*/

void max8903_charger_enable(struct max8903_charger *charger, int current_val)
{
	int uok = 0;

#ifdef TRUST_UOK
	uok = gpio_get_value(charger->max8903_gpio_chg_uok);
#endif

	/* only enable charging at all, if VBUS is present (uok low) */
	if (uok != 0)
	{
		current_val = 0;
		printk("MAX8903: No VBUS Detected through UOK.\n");
	}

	if (current_val >= AC_CURRENT_LIMIT) {
		int dok = gpio_get_value(charger->max8903_gpio_chg_dok);

		gpio_direction_output(charger->max8903_gpio_chg_usus, 0);
		charger->adapter_online = 1;
		charger->adapter_active = 1;

		// If standared USB cable detected, limit current to 500mA
		if (dok == 1 && uok == 0 && current_val >= USB_CURRENT_LIMIT_HIGH)
		{
			charger->adapter_curr_limit = USB_CURRENT_LIMIT_HIGH;
			gpio_set_value(charger->max8903_gpio_chg_iusb, CHG_IUSB_SELECT_500mA);
			gpio_set_value(charger->max8903_gpio_chg_ilm, CHG_ILM_SELECT_USB);
		}
		else
		{
			charger->adapter_curr_limit = AC_CURRENT_LIMIT;
			gpio_set_value(charger->max8903_gpio_chg_ilm, CHG_ILM_SELECT_AC);
		}
		max17042_update_callback(1);
		max8903_enable_charge(charger, 1);
		power_supply_changed(&charger->adapter);
		printk("MAX8903: AC Charging at %d mA \n", charger->adapter_curr_limit);
	} else if (current_val >= USB_CURRENT_LIMIT_HIGH) {
		charger->usb_online = 1;
		charger->usb_active = 1;
		charger->adapter_curr_limit = USB_CURRENT_LIMIT_HIGH;
		gpio_set_value(charger->max8903_gpio_chg_iusb, CHG_IUSB_SELECT_500mA);
		gpio_set_value(charger->max8903_gpio_chg_ilm, CHG_ILM_SELECT_USB);
		max17042_update_callback(1);
		max8903_enable_charge(charger, 1);
		power_supply_changed(&charger->usb);
		gpio_direction_output(charger->max8903_gpio_chg_usus, 0);
		printk("MAX8903: USB Charging at %d mA \n", charger->adapter_curr_limit);
	} else {		/* no VBUS, no DC */
		max8903_enable_charge(charger, 0);
		gpio_direction_output(charger->max8903_gpio_chg_usus, 1); // ensure that we will not draw current
		charger->adapter_online = 0;
		charger->adapter_active = 0;
		gpio_set_value(charger->max8903_gpio_chg_ilm, CHG_ILM_SELECT_USB);
		charger->adapter_curr_limit = USB_CURRENT_LIMIT_HIGH; /* back to high limit by default */
		charger->usb_online = 0;
		charger->usb_active = 0;
		power_supply_changed(&charger->usb);
		power_supply_changed(&charger->adapter);
		max17042_update_callback(0);
		printk("MAX8903: Charger Unplugged!\n");
	}
}

static int twl6030_usb_notifier_call(struct notifier_block *nblock,
		unsigned long event, void *data)
{
	struct max8903_charger *di = container_of(nblock, struct max8903_charger, nb);

	if (event == USB_EVENT_ENUMERATED)
		return NOTIFY_OK;

	di->detected_event = event;
	switch (event) {
		case USB_EVENT_VBUS:
			di->detected_charger = *((unsigned int *) data);
			di->detected_charger_current = USB_CURRENT_LIMIT_HIGH;
			DEBUG("MAX8903: USB_EVENT_VBUS: Charger=%d\n", di->detected_charger);
			break;

		case USB_EVENT_ENUMERATED:
			di->detected_charger = POWER_SUPPLY_TYPE_USB;
			di->detected_charger_current = USB_CURRENT_LIMIT_HIGH; //*((unsigned int *) data);
			DEBUG("MAX8903: USB_EVENT_ENUMERATED: %u mA\n", di->detected_charger_current);
			break;

		case USB_EVENT_CHARGER:
			di->detected_charger = *((unsigned int *) data);
			di->detected_charger_current = AC_CURRENT_LIMIT;
			DEBUG("MAX8903: USB_EVENT_CHARGER: Charger=%d\n", di->detected_charger);
			break;

		case USB_EVENT_NONE:
			DEBUG("MAX8903: USB_EVENT_NONE\n");
			di->detected_charger = POWER_SUPPLY_TYPE_BATTERY;
			di->usb_online = 0;
			di->detected_charger_current = 0;
			break;

		case USB_EVENT_ID:
			DEBUG("MAX8903: USB_EVENT_ID\n");
			break;

		default:
			return NOTIFY_OK;
	}

	cancel_delayed_work_sync(&di->usb_work);
	schedule_delayed_work(&di->usb_work, HZ*2);

	return NOTIFY_OK;
}

static void max8903_usb_charger_work(struct work_struct *work)
{
	struct max8903_charger	*di =
		container_of(work, struct max8903_charger, usb_work.work);

	max8903_charger_enable(di, di->detected_charger_current);
}

static irqreturn_t max8903_fault_interrupt(int irq, void *_di)
{
	// struct max8903_charger *di=_di;
	disable_irq(irq);
	printk("Fault Status Changed!\n");
	enable_irq(irq);
	return IRQ_HANDLED;
}

/*WALL online status*/
static int adapter_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max8903_charger *mc= container_of(psy, struct max8903_charger, adapter);
	int ret = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval =  mc->adapter_online;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			val->intval = mc->adapter_curr_limit;
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}
/*USB online status*/
static int usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max8903_charger *mc= container_of(psy, struct max8903_charger, usb);
	int ret = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = mc->usb_online;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			val->intval = USB_CURRENT_LIMIT_HIGH;
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

/*only interested in ONLINE property*/
static enum power_supply_property power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_AVG,
};

/****************************************
  For manufacturing test purpose only
 *****************************************/
static ssize_t set_charge(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max8903_charger *charger = dev_get_drvdata(dev);

	static const char* const command[]={"LOW","HIGH"};
	unsigned long index=0;
	if(!charger)
		return -EINVAL;

	index=simple_strtoul(buf,0,10);
	if(index>1){
		printk("MAX8903-TESTING: INVALID COMMAND only 1 and 0 are acceptable!!\n");
		return -EINVAL;
	}

	printk("MAX8903-TESTING:  == %s!\n",command[index]);
	/*set the FTM mode*/
	if(charger->ftm!=1)
		charger->ftm=1;
	/*for evt1b and older HW, pulling low on ~CEN means enable charge, EVT2 and higher will reverse this logic 
	  a board HW rev detection will be put in u-boot for differentiation*/
	max8903_enable_charge(charger, !index);

	if(index){
		printk("MAX8903-TESTING: PULL HIGH PIN(~CEN)!\n");
	}
	else{   
		printk("MAX8903-TESTING: PULL LOW PIN(~CEN)!\n");
	}
	return count;
}

/*no need to read entry, write only for controlling the charging circuit*/
static DEVICE_ATTR(ctrl_charge, S_IRUGO | S_IWUSR, NULL, set_charge);

static struct attribute *max8903_control_sysfs_entries[] = {
	&dev_attr_ctrl_charge.attr,
	NULL,
};

static struct attribute_group max8903_attr_group = {
	.name   = NULL,         /* put in device directory */
	.attrs  = max8903_control_sysfs_entries,
};

static void max8903_usb_charger_atboot(struct max8903_charger *di)
{
	di->detected_event = otg_get_link_status(di->otg);
	printk("MAX8903: Charger detected at boot = %ld\n", di->detected_event);
		switch (di->detected_event) {
		case USB_EVENT_VBUS:
			di->detected_charger = POWER_SUPPLY_TYPE_USB_CDP;
			di->detected_charger_current = USB_CURRENT_LIMIT_HIGH;
			DEBUG("MAX8903: USB_EVENT_VBUS: Charger=%d\n", di->detected_charger);
			break;

		case USB_EVENT_CHARGER:
			di->detected_charger = POWER_SUPPLY_TYPE_USB_DCP;
			di->detected_charger_current = AC_CURRENT_LIMIT;
			DEBUG("MAX8903: USB_EVENT_CHARGER: Charger=%d\n", di->detected_charger);
			break;

		case USB_EVENT_NONE:
			DEBUG("MAX8903: USB_EVENT_NONE\n");
			di->detected_charger = POWER_SUPPLY_TYPE_BATTERY;
			di->usb_online = 0;
			di->detected_charger_current = 0;
			break;

		case USB_EVENT_ID:
			DEBUG("MAX8903: USB_EVENT_ID\n");
			break;

		default:
			return;
	}
	schedule_delayed_work(&di->usb_work, HZ*5);
}


/*Charger Module Initialization*/
static int max8903_charger_probe(struct platform_device *pdev)
{

	struct max8903_charger *mc;
	int ret;
	struct resource *r;

	/*allocate mem*/
	mc = kzalloc(sizeof(*mc), GFP_KERNEL);
	if (!mc)
		return -ENOMEM;

	platform_set_drvdata(pdev, mc);

	printk("MAX8903: Charger Initializing...\n");
	//INIT_DELAYED_WORK(&mc->max8903_charger_detect_work, max8903_charger_detect);
	/* Create power supplies for WALL/USB which are the only 2 ext supplies*/
	mc->adapter.name            = "ac";
	mc->adapter.type            = POWER_SUPPLY_TYPE_MAINS;
	mc->adapter.properties      = power_props;
	mc->adapter.num_properties  = ARRAY_SIZE(power_props);
	mc->adapter.get_property    = &adapter_get_property;

	mc->usb.name            = "usb";
	mc->usb.type            = POWER_SUPPLY_TYPE_USB;
	mc->usb.properties      = power_props;
	mc->usb.num_properties  = ARRAY_SIZE(power_props);
	mc->usb.get_property    = usb_get_property;

	mc->adapter_online = 0;
	mc->adapter_active = 0;
	mc->adapter_curr_limit = USB_CURRENT_LIMIT_HIGH; /* default limit is high limit */
	mc->usb_online = 0;
	mc->usb_active = 0;

	ret = power_supply_register(&pdev->dev, &mc->adapter);
	if (ret) {
		printk("MAX8903: Failed to register WALL CHARGER\n");
		goto exit0;
	}

	ret = power_supply_register(&pdev->dev, &mc->usb);
	if (ret) {
		printk("MAX8903: Failed to register USB CHARGER\n");
		goto exit1;
	}

	/*****************/
	/* Get resources */
	/*****************/
	r = platform_get_resource_byname(pdev, IORESOURCE_IO, MAX8903_TOKEN_GPIO_CHG_EN);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource: %s\n", MAX8903_TOKEN_GPIO_CHG_EN);
		goto exit1;
	}
	mc->max8903_gpio_chg_en = r->start;
	r = platform_get_resource_byname(pdev, IORESOURCE_IO, MAX8903_TOKEN_GPIO_CHG_FLT);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource: %s\n", MAX8903_TOKEN_GPIO_CHG_FLT);
		goto exit1;
	}
	mc->max8903_gpio_chg_flt = r->start;
	r = platform_get_resource_byname(pdev, IORESOURCE_IO, MAX8903_TOKEN_GPIO_CHG_IUSB);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource: %s\n", MAX8903_TOKEN_GPIO_CHG_IUSB);
		goto exit1;
	}
	mc->max8903_gpio_chg_iusb = r->start;
	r = platform_get_resource_byname(pdev, IORESOURCE_IO, MAX8903_TOKEN_GPIO_CHG_USUS);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource: %s\n", MAX8903_TOKEN_GPIO_CHG_USUS);
		goto exit1;
	}
	mc->max8903_gpio_chg_usus = r->start;
	r = platform_get_resource_byname(pdev, IORESOURCE_IO, MAX8903_TOKEN_GPIO_CHG_ILM);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource: %s\n", MAX8903_TOKEN_GPIO_CHG_ILM);
		goto exit1;
	}
	mc->max8903_gpio_chg_ilm = r->start;
	r = platform_get_resource_byname(pdev, IORESOURCE_IO, MAX8903_TOKEN_GPIO_CHG_UOK);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource: %s\n", MAX8903_TOKEN_GPIO_CHG_UOK);
		goto exit1;
	}
	mc->max8903_gpio_chg_uok = r->start;
	r = platform_get_resource_byname(pdev, IORESOURCE_IO, MAX8903_TOKEN_GPIO_CHG_DOK);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource: %s\n", MAX8903_TOKEN_GPIO_CHG_DOK);
		goto exit1;
	}
	mc->max8903_gpio_chg_dok = r->start;

	/******************************/
	/* Control pins configuration */
	/******************************/

	/*~DOK Status*/
	if (gpio_request(mc->max8903_gpio_chg_dok, MAX8903_TOKEN_GPIO_CHG_DOK) < 0) {
		printk(KERN_ERR "Can't get GPIO for max8903 chg_dok\n");
		goto exit2;
	}
	gpio_direction_input(mc->max8903_gpio_chg_dok);
	/*~UOK Status*/
	if (gpio_request(mc->max8903_gpio_chg_uok, MAX8903_TOKEN_GPIO_CHG_UOK) < 0) {
		printk(KERN_ERR "Can't get GPIO for max8903 chg_uok\n");
		goto exit3;
	}
	gpio_direction_input(mc->max8903_gpio_chg_uok);
	/*IUSB control*/
	if (gpio_request(mc->max8903_gpio_chg_iusb,  MAX8903_TOKEN_GPIO_CHG_IUSB) < 0) {
		printk(KERN_ERR "Can't get GPIO for max8903 chg_iusb\n");
		goto exit5;
	}
	gpio_direction_output(mc->max8903_gpio_chg_iusb, CHG_IUSB_SELECT_500mA);

	/*USUS control*/
	if (gpio_request(mc->max8903_gpio_chg_usus, MAX8903_TOKEN_GPIO_CHG_USUS) < 0) {
		printk(KERN_ERR "Can't get GPIO for max8903 chg_usus\n");
		goto exit6;
	}
	gpio_direction_output(mc->max8903_gpio_chg_usus, DISABLED); // leave USUS disabled until we connect

	/*~CEN control */
	if (gpio_request(mc->max8903_gpio_chg_en, MAX8903_TOKEN_GPIO_CHG_EN) < 0) {
		printk(KERN_ERR "Can't get GPIO for max8903 chg_en\n");
		goto exit7;
	}
	gpio_direction_output(mc->max8903_gpio_chg_en, DISABLED);

	if (gpio_request(mc->max8903_gpio_chg_ilm, MAX8903_TOKEN_GPIO_CHG_ILM) < 0) {
		printk(KERN_ERR "Can't get GPIO for max8903 chg_ilm\n");
		goto exit8;
	}
	gpio_direction_output(mc->max8903_gpio_chg_ilm, CHG_ILM_SELECT_USB);    /* set to USB  current limit by default */

	if (gpio_request(mc->max8903_gpio_chg_flt, MAX8903_TOKEN_GPIO_CHG_FLT) < 0) {
		printk(KERN_ERR "Can't get GPIO for max8903 chg_flt\n");
		goto exit9;
	}
	gpio_direction_input(mc->max8903_gpio_chg_flt);

	/*~FLT status*/
	mc->flt_irq= gpio_to_irq(mc->max8903_gpio_chg_flt) ;
	ret  = request_irq( mc->flt_irq,
			max8903_fault_interrupt,
			IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_DISABLED,
			"max8903-fault-irq",
			mc);

	printk("MAX8903: Request CHARGER FLT IRQ successfully!\n");
	if (ret < 0) {
		printk(KERN_ERR "MAX8903: Can't Request IRQ for max8903 flt_irq\n");
		goto exita;
	}

	/*
	   create sysfs for manufacture testing coverage on charging
	   the operator should be able to write 1 to turn on the charging and 0 to
	   turn off the charging to verify the charging circuit is functioning
	   */
	ret = sysfs_create_group(&pdev->dev.kobj, &max8903_attr_group);

	if (ret){
		printk(KERN_ERR "MAX8903: Can't Create Sysfs Entry for FTM\n");
		goto exitd;
	}

	// Register charger work and notification callback.
	INIT_DELAYED_WORK_DEFERRABLE(&mc->usb_work, max8903_usb_charger_work);
	mc->nb.notifier_call = twl6030_usb_notifier_call;
	mc->otg = otg_get_transceiver();
	if (!mc->otg) {
		dev_err(&pdev->dev, "otg_get_transceiver() failed\n");
		goto exitn;
	}
	ret = otg_register_notifier(mc->otg, &mc->nb);
	if (ret) {
		dev_err(&pdev->dev, "otg register notifier failed %d\n", ret);
		goto exitn;
	}

	max8903_usb_charger_atboot(mc);

	return 0;

exitn:
	cancel_delayed_work_sync(&mc->usb_work);
	sysfs_remove_group(&pdev->dev.kobj, &max8903_attr_group);

exitd:
	free_irq(mc->flt_irq,mc);
exita:
	gpio_free(mc->max8903_gpio_chg_flt);
exit9:
	gpio_free(mc->max8903_gpio_chg_ilm);
exit8:
	gpio_free(mc->max8903_gpio_chg_en);
exit7:
	gpio_free(mc->max8903_gpio_chg_usus);
exit6:
	gpio_free(mc->max8903_gpio_chg_iusb);
exit5:
	gpio_free(mc->max8903_gpio_chg_uok);
exit3:
	gpio_free(mc->max8903_gpio_chg_dok);
exit2:
	power_supply_unregister(&mc->usb);
exit1:
	power_supply_unregister(&mc->adapter);
exit0:
	kfree(mc);
	return ret;
}

/*Charger Module DeInitialization*/
static int __exit max8903_charger_remove(struct platform_device *pdev)
{
	struct max8903_charger *mc = platform_get_drvdata(pdev);
	/*unregister,clean up*/
	if(mc)
	{
		power_supply_unregister(&mc->usb);
		power_supply_unregister(&mc->adapter);
	}

	free_irq(mc->flt_irq,mc);
	sysfs_remove_group(&pdev->dev.kobj, &max8903_attr_group);
	otg_unregister_notifier(mc->otg, &mc->nb);
	cancel_delayed_work_sync(&mc->usb_work);

	gpio_free(mc->max8903_gpio_chg_uok);
	gpio_free(mc->max8903_gpio_chg_dok);
	gpio_free(mc->max8903_gpio_chg_iusb);
	gpio_free(mc->max8903_gpio_chg_usus);
	gpio_free(mc->max8903_gpio_chg_flt);
	gpio_free(mc->max8903_gpio_chg_en);
	gpio_free(mc->max8903_gpio_chg_ilm);
	if(mc)
		kfree(mc);
	return 0;
}

static struct platform_driver max8903_charger_driver = {
	.driver = {
		.name = "max8903_charger",
	},
	.probe = max8903_charger_probe,
	.remove =  __exit_p(max8903_charger_remove),
};

static int __init max8903_charger_init(void)
{
	printk("MAX8903: Charger registering!\n");
	return platform_driver_register(&max8903_charger_driver);
}

static void __exit max8903_charger_exit(void)
{
	platform_driver_unregister(&max8903_charger_driver);
}

module_init(max8903_charger_init);
module_exit(max8903_charger_exit);

MODULE_AUTHOR("Lei Cao <lcao@book.com>");
MODULE_DESCRIPTION("MAXIM 8903 PMIC driver");
MODULE_LICENSE("GPL");
