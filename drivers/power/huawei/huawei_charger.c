/*
 * drivers/power/huawei_charger.c
 *
 *huawei charger driver
 *
 * Copyright (C) 2012-2015 HUAWEI, Inc.
 * Author: HUAWEI, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/spmi.h>
#include <linux/sysfs.h>
#include <linux/power/huawei_charger.h>

#include <dsm/dsm_pub.h>

#define DEFAULT_FCP_TEST_DELAY	6000
#define DEFAULT_IIN_CURRENT	1000
#define MAX_CURRENT	3000
#define MIN_CURRENT	100


typedef enum {
	CHARGER_UNKNOWN = 0,
	STANDARD_HOST,		/* USB : 450mA */
	CHARGING_HOST,
	NONSTANDARD_CHARGER,	/* AC : 450mA~1A */
	STANDARD_CHARGER,	/* AC : ~1A */
	APPLE_2_1A_CHARGER,	/* 2.1A apple charger */
	APPLE_1_0A_CHARGER,	/* 1A apple charger */
	APPLE_0_5A_CHARGER,	/* 0.5A apple charger */
	WIRELESS_CHARGER,
} CHARGER_TYPE;

static struct dsm_client *dsm_chargemonitor_dclient = NULL;
static struct dsm_dev dsm_charge_monitor =
{
	.name = "dsm_charge_monitor",
	.fops = NULL,
	.buff_size = 4096,
};

struct class *power_class = NULL;
struct device *charge_dev = NULL;
struct charge_device_info *g_charger_device_para = NULL;

static int get_property_from_psy(struct power_supply *psy,
		enum power_supply_property prop)
{
	int rc;
	int val = 0;
	union power_supply_propval ret = {0, };

	rc = psy->get_property(psy, prop, &ret);
	if (rc) {
		pr_err("psy doesn't support reading prop %d rc = %d\n",
				prop, rc);
		return rc;
	}
	val = ret.intval;
	return val;
}

void get_log_info_from_psy(struct power_supply *psy,
			   enum power_supply_property prop, char *buf)
{
	int rc;
	union power_supply_propval val = {0, };

	val.strval = buf;
	rc = psy->get_property(psy, prop, &val);
	if (rc) {
		pr_err("psy does not allow get prop %d rc = %d\n", prop, rc);
	}
}

static struct kobject *g_sysfs_poll = NULL;

static ssize_t get_poll_charge_start_event(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct charge_device_info *chip = g_charger_device_para;

	if (chip) {
		return snprintf(buf, PAGE_SIZE, "%d\n", chip->input_event);
	} else {
		return 0;
	}
}

static ssize_t set_poll_charge_event(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct charge_device_info *chip = g_charger_device_para;
	long val = 0;

	if (chip) {
		if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 3000)) {
			return -EINVAL;
		}
		chip->input_event = val;
		sysfs_notify(g_sysfs_poll, NULL, "poll_charge_start_event");
	}
	return count;
}

static DEVICE_ATTR(poll_charge_start_event, (S_IWUSR | S_IRUGO),
				get_poll_charge_start_event,
				set_poll_charge_event);

static int charge_event_poll_register(struct device *dev)
{
	int ret;

	ret = sysfs_create_file(&dev->kobj, &dev_attr_poll_charge_start_event.attr);
	if (ret) {
		pr_err("fail to create poll node for %s\n", dev->kobj.name);
		return ret;
	}
	g_sysfs_poll = &dev->kobj;
	return ret;
}

static void charge_event_notify(unsigned int event)
{
	struct charge_device_info *chip = g_charger_device_para;

	if (!chip) {
		pr_info("smb device is not init, do nothing!\n");
		return;
	}
	/* avoid notify charge stop event continuously without charger inserted */
	if ((chip->input_event != event) || (event == SMB_START_CHARGING)) {
		chip->input_event = event;
		if (g_sysfs_poll) {
			sysfs_notify(g_sysfs_poll, NULL, "poll_charge_start_event");
		}
	}
}

static void smb_update_status(struct charge_device_info *di)
{
	unsigned int events = 0;
	int charging_enabled = 0;
	int battery_present = 0;

	charging_enabled = get_property_from_psy(di->batt_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED);
	battery_present = get_property_from_psy(di->batt_psy,
					POWER_SUPPLY_PROP_PRESENT);
	if (!battery_present) {
		events = SMB_STOP_CHARGING;
	}
	if (!events) {
		if (charging_enabled && battery_present) {
			events = SMB_START_CHARGING;
		}
	}
	charge_event_notify(events);
}

static void smb_charger_work(struct work_struct *work)
{
	struct charge_device_info *chip = container_of(work,
				struct charge_device_info, smb_charger_work.work);

	smb_update_status(chip);
	schedule_delayed_work(&chip->smb_charger_work,
				msecs_to_jiffies(QPNP_SMBCHARGER_TIMEOUT));
}


#define CHARGE_SYSFS_FIELD(_name, n, m, store)	\
{	\
	.attr = __ATTR(_name, m, charge_sysfs_show, store),	\
	.name = CHARGE_SYSFS_##n,	\
}

/*#define CHARGE_SYSFS_FIELD_RW(_name, n)	\
	CHARGE_SYSFS_FIELD(_name, n, S_IWUSR | S_IRUGO,	\
		charge_sysfs_store)
*/
#define CHARGE_SYSFS_FIELD_RO(_name, n)	\
	CHARGE_SYSFS_FIELD(_name, n, S_IRUGO, NULL)

static ssize_t charge_sysfs_show(struct device *dev,
struct device_attribute *attr, char *buf);

struct charge_sysfs_field_info
{
	char name;
	struct device_attribute    attr;
};


static struct charge_sysfs_field_info charge_sysfs_field_tbl[] =
{
	CHARGE_SYSFS_FIELD_RO(chargelog_head,    CHARGELOG_HEAD),
	CHARGE_SYSFS_FIELD_RO(chargelog,    CHARGELOG),
	CHARGE_SYSFS_FIELD_RO(ibus,    IBUS),
	CHARGE_SYSFS_FIELD_RO(chargerType,    CHARGE_TYPE)
};

static struct attribute *charge_sysfs_attrs[ARRAY_SIZE(charge_sysfs_field_tbl) + 1];

static const struct attribute_group charge_sysfs_attr_group =
{
	.attrs = charge_sysfs_attrs,
};

 /* initialize charge_sysfs_attrs[] for charge attribute */
static void charge_sysfs_init_attrs(void)
{
	int i, limit = ARRAY_SIZE(charge_sysfs_field_tbl);

	for (i = 0; i < limit; i++) {
		charge_sysfs_attrs[i] = &charge_sysfs_field_tbl[i].attr.attr;
	}

	charge_sysfs_attrs[limit] = NULL; /* Has additional entry for this */
}

/*
 * get the current device_attribute from charge_sysfs_field_tbl
 * by attr's name
 */
static struct charge_sysfs_field_info *charge_sysfs_field_lookup(const char *name)
{
	int i, limit = ARRAY_SIZE(charge_sysfs_field_tbl);

	for (i = 0; i < limit; i++) {
		if (!strcmp(name, charge_sysfs_field_tbl[i].attr.attr.name)) {
			break;
		}
	}

	if (i >= limit)	{
		return NULL;
	}

	return &charge_sysfs_field_tbl[i];
}

int get_loginfo_int(struct power_supply *psy, int propery)
{
	int rc = 0;
	union power_supply_propval ret = {0, };

	if (!psy) {
		pr_err("get input source power supply node failed!\n");
		return -EINVAL;
	}

	rc = psy->get_property(psy, propery, &ret);
	if (rc) {
		//pr_err("Couldn't get type rc = %d\n", rc);
		ret.intval = -EINVAL;
	}

	return ret.intval;
}
EXPORT_SYMBOL_GPL(get_loginfo_int);

void strncat_length_protect(char *dest, char *src)
{
	int str_length = 0;

	if (NULL == dest || NULL == src) {
		pr_err("the dest or src is NULL");
		return;
	}
	if (strlen(dest) >= CHARGELOG_SIZE) {
		pr_err("strncat dest is full!\n");
		return;
	}

	str_length = min(CHARGELOG_SIZE - strlen(dest), strlen(src));
	if (str_length > 0) {
		strncat(dest, src, str_length);
	}
}
EXPORT_SYMBOL_GPL(strncat_length_protect);

static void conver_usbtype(int val, char *p_str)
{
	if (NULL == p_str) {
		pr_err("the p_str is NULL\n");
		return;
	}

	switch (val) {
	case CHARGER_UNKNOWN:
		strncpy(p_str, "UNKNOWN", sizeof("UNKNOWN"));
		break;
	case STANDARD_HOST:
		strncpy(p_str, "STANDARD_HOST", sizeof("STANDARD_HOST"));
		break;
	case CHARGING_HOST:
		strncpy(p_str, "CHARGING_HOST", sizeof("CHARGING_HOST"));
		break;
	case NONSTANDARD_CHARGER:
		strncpy(p_str, "NONSTANDARD_CHARGER", sizeof("NONSTANDARD_CHARGER"));
		break;
	case STANDARD_CHARGER:
		strncpy(p_str, "STANDARD_CHARGER", sizeof("STANDARD_CHARGER"));
		break;
	case APPLE_2_1A_CHARGER:
		strncpy(p_str, "APPLE_2_1A_CHARGER", sizeof("APPLE_2_1A_CHARGER"));
		break;
	case APPLE_1_0A_CHARGER:
		strncpy(p_str, "APPLE_1_0A_CHARGER", sizeof("APPLE_1_0A_CHARGER"));
		break;
	case APPLE_0_5A_CHARGER:
		strncpy(p_str, "APPLE_0_5A_CHARGER", sizeof("APPLE_0_5A_CHARGER"));
		break;
	case WIRELESS_CHARGER:
		strncpy(p_str, "WIRELESS_CHARGER", sizeof("WIRELESS_CHARGER"));
		break;
	default:
		strncpy(p_str, "UNSTANDARD", sizeof("UNSTANDARD"));
		break;
	}
}

static void conver_charging_status(int val, char *p_str)
{
	if (NULL == p_str) {
		pr_err("the p_str is NULL\n");
		return;
	}

	switch (val) {
	case POWER_SUPPLY_STATUS_UNKNOWN:
		strncpy(p_str, "UNKNOWN", sizeof("UNKNOWN"));
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		strncpy(p_str, "CHARGING", sizeof("CHARGING"));
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		strncpy(p_str, "DISCHARGING", sizeof("DISCHARGING"));
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		strncpy(p_str, "NOTCHARGING", sizeof("NOTCHARGING"));
		break;
	case POWER_SUPPLY_STATUS_FULL:
		strncpy(p_str, "FULL", sizeof("FULL"));
		break;
	default:
		break;
	}
}

static void conver_charger_health(int val, char *p_str)
{
	if (NULL == p_str) {
		pr_err("the p_str is NULL\n");
		return;
	}

	switch (val) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		strncpy(p_str, "OVERHEAT", sizeof("OVERHEAT"));
		break;
	case POWER_SUPPLY_HEALTH_COLD:
		strncpy(p_str, "COLD", sizeof("COLD"));
		break;
	case POWER_SUPPLY_HEALTH_WARM:
		strncpy(p_str, "WARM", sizeof("WARM"));
		break;
	case POWER_SUPPLY_HEALTH_COOL:
		strncpy(p_str, "COOL", sizeof("COOL"));
		break;
	case POWER_SUPPLY_HEALTH_GOOD:
		strncpy(p_str, "GOOD", sizeof("GOOD"));
		break;
	default:
		break;
	}
}

static bool charger_shutdown_flag;
static int __init early_parse_shutdown_flag(char *p)
{
	if (p) {
		if (!strcmp(p, "charger")) {
			charger_shutdown_flag = true;
		}
	}
	return 0;
}
early_param("androidboot.mode", early_parse_shutdown_flag);

static void get_charger_shutdown_flag(bool flag, char *p_str)
{
	if (NULL == p_str) {
		pr_err("the p_str is NULL\n");
		return;
	}
	if (flag) {
		strncpy(p_str, "OFF", sizeof("OFF"));
	} else {
		strncpy(p_str, "ON", sizeof("ON"));
	}
}

/* show the value for all charge device's node */
static ssize_t charge_sysfs_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct charge_sysfs_field_info *info = NULL;
	struct charge_device_info *di = dev_get_drvdata(dev);
	int online = 0, in_type = 0, ch_en = 0, status = 0, health = 0, bat_present = 0;
	int temp = 0, vol = 0, cur = 0, capacity = 0, ibus = 0, usb_vol = 0;
	char cType[30] = {0}, cStatus[30] = {0}, cHealth[30] = {0}, cOn[30] = {0};
	//int count = 0;

	info = charge_sysfs_field_lookup(attr->attr.name);
	if (!info) {
		return -EINVAL;
	}

	switch (info->name) {
	case CHARGE_SYSFS_CHARGELOG_HEAD:
		mutex_lock(&di->sysfs_data.dump_reg_head_lock);
		get_log_info_from_psy(di->batt_psy,
				POWER_SUPPLY_PROP_REGISTER_HEAD,
				di->sysfs_data.reg_head);
		ret = snprintf(buf, MAX_SIZE, " online   in_type     usb_vol     iin_thl     ch_en   status         health    bat_present   temp    vol       cur       capacity   ibus       %s Mode\n",
		di->sysfs_data.reg_head);
		mutex_unlock(&di->sysfs_data.dump_reg_head_lock);
		break;
	case CHARGE_SYSFS_CHARGELOG:
		online = get_loginfo_int(di->usb_psy, POWER_SUPPLY_PROP_ONLINE);
		in_type = get_loginfo_int(di->usb_psy, POWER_SUPPLY_PROP_TYPE);
		conver_usbtype(in_type, cType);
		usb_vol = get_loginfo_int(di->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		ch_en = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_CHARGING_ENABLED);
		status = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_STATUS);
		conver_charging_status(status, cStatus);
		health = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_HEALTH);
		conver_charger_health(health, cHealth);
		bat_present = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_PRESENT);
		temp = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_TEMP);
		vol = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		cur = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
		capacity = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_CAPACITY);
		ibus = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW);
		get_charger_shutdown_flag(charger_shutdown_flag, cOn);
		mutex_lock(&di->sysfs_data.dump_reg_lock);
		get_log_info_from_psy(di->batt_psy,
				POWER_SUPPLY_PROP_DUMP_REGISTER,
				di->sysfs_data.reg_value);
		ret = snprintf(buf, MAX_SIZE, " %-8d %-11s %-11d %-11d %-7d %-14s %-9s %-13d %-7d %-9d %-9d %-10d %-10d %-16s %s\n",
				online, cType, usb_vol, di->sysfs_data.iin_thl,
				ch_en, cStatus, cHealth, bat_present, temp, vol,
				cur, capacity, ibus, di->sysfs_data.reg_value, cOn);
		mutex_unlock(&di->sysfs_data.dump_reg_lock);
		break;
	case CHARGE_SYSFS_IBUS:
		ibus = get_loginfo_int(di->batt_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW);
		return snprintf(buf, MAX_SIZE, "%d\n", ibus);
		break;
	case CHARGE_SYSFS_CHARGE_TYPE:
		in_type = get_loginfo_int(di->usb_psy, POWER_SUPPLY_PROP_TYPE);
		conver_usbtype(in_type, cType);
		return snprintf(buf, MAX_SIZE, "%s\n", cType);
		break;
	default:
		pr_err("(%s)NODE ERR!!HAVE NO THIS NODE:(%d)\n", __func__, info->name);
		break;
	}

	return ret;
}

/* create the charge device sysfs group */
static int charge_sysfs_create_group(struct charge_device_info *di)
{
	charge_sysfs_init_attrs();
	return sysfs_create_group(&di->dev->kobj, &charge_sysfs_attr_group);
}

/* remove the charge device sysfs group */
static inline void charge_sysfs_remove_group(struct charge_device_info *di)
{
	sysfs_remove_group(&di->dev->kobj, &charge_sysfs_attr_group);
}

static struct mutex mutex_of_hw_power_class;
void mutex_init_of_hw_power_class(void)
{
	mutex_init(&mutex_of_hw_power_class);
	return ;
}
EXPORT_SYMBOL_GPL(mutex_init_of_hw_power_class);

static struct class *hw_power_class;
struct class *hw_power_get_class(void)
{
	mutex_lock(&mutex_of_hw_power_class);
	if (NULL == hw_power_class) {
		hw_power_class = class_create(THIS_MODULE, "hw_power");
		if (IS_ERR(hw_power_class)) {
			pr_err("hw_power_class create fail");
			mutex_unlock(&mutex_of_hw_power_class);
			return NULL;
		}
	}
	mutex_unlock(&mutex_of_hw_power_class);
	return hw_power_class;
}
EXPORT_SYMBOL_GPL(hw_power_get_class);

static int charge_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct charge_device_info *di = NULL;
	struct class *power_class = NULL;
	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	//struct power_supply *bms_psy;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("batt supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}
	/*bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		pr_err("bms supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}*/
	di = devm_kzalloc(&pdev->dev, sizeof(struct charge_device_info), GFP_KERNEL);
	if (!di) {
		pr_err("memory allocation failed.\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "huawei,fcp-test-delay", &di->fcp_test_delay);
	if (ret) {
		pr_info("There is no fcp test delay setting, use default time: 1s\n");
		di->fcp_test_delay = DEFAULT_FCP_TEST_DELAY;
	}

	INIT_DELAYED_WORK(&di->smb_charger_work, smb_charger_work);
	di->dev = &(pdev->dev);
	dev_set_drvdata(&(pdev->dev), di);
	di->usb_psy = usb_psy;
	di->batt_psy = batt_psy;
	//di->bms_psy = bms_psy;
	di->running_test_settled_status = POWER_SUPPLY_STATUS_CHARGING;

	di->sysfs_data.iin_thl = 1500;
	di->sysfs_data.iin_rt = 1;
	di->sysfs_data.iin_rt_curr = DEFAULT_IIN_CURRENT;
	di->sysfs_data.hiz_mode = 0;
	di->chrg_config = 1;
	di->factory_diag = 1;

	mutex_init(&di->sysfs_data.dump_reg_lock);
	mutex_init(&di->sysfs_data.dump_reg_head_lock);
	ret = charge_sysfs_create_group(di);
	if (ret) {
		pr_err("can't create charge sysfs entries\n");
		goto charge_fail_0;
	}

	power_class = hw_power_get_class();
	if (power_class) 
	{
		if (charge_dev == NULL) 
		{
			charge_dev = device_create(power_class, NULL, 0, NULL, "charger");
		}
		if(NULL != charge_dev)
		{
			ret = sysfs_create_link(&charge_dev->kobj, &di->dev->kobj, "charge_data");
			if(ret) 
			{
				pr_err("create link to charge_data fail.\n");
			}
		}
	}

	if((NULL == power_class) || (NULL == charge_dev))
	{
		pr_err("power_class or charge_dev is NULL\n");
		goto charge_fail_0;
	}
	charge_event_poll_register(charge_dev);
	dsm_register_client(&dsm_charge_monitor);
	g_charger_device_para = di;
	pr_info("huawei charger probe ok!\n");
	return 0;

charge_fail_0:
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(di);
	di = NULL;

	return 0;
}

static void charge_event_poll_unregister(struct device *dev)
{
	sysfs_remove_file(&dev->kobj, &dev_attr_poll_charge_start_event.attr);
	g_sysfs_poll = NULL;
}

static int charge_remove(struct platform_device *pdev)
{
	struct charge_device_info *di = dev_get_drvdata(&pdev->dev);

	cancel_delayed_work_sync(&di->smb_charger_work);
	charge_event_poll_unregister(charge_dev);
	dsm_unregister_client(dsm_chargemonitor_dclient, &dsm_charge_monitor);
	charge_sysfs_remove_group(di);
	kfree(di);
	di = NULL;

	return 0;
}

static void charge_shutdown(struct platform_device  *pdev)
{
	return;
}

#ifdef CONFIG_PM

static int charge_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int charge_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static struct of_device_id charge_match_table[] =
{
	{
		.compatible = "huawei,charger",
		.data = NULL,
	},
	{
	},
};

static struct platform_driver charge_driver =
{
	.probe = charge_probe,
	.remove = charge_remove,
	.suspend = charge_suspend,
	.resume = charge_resume,
	.shutdown = charge_shutdown,
	.driver =
	{
		.name = "huawei,charger",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(charge_match_table),
	},
};

int charge_init(void)
{
    int ret;
    ret = platform_driver_register(&charge_driver);
    if(ret)
    {
        pr_info("register platform_driver_register failed!\n");
        return ret;
    }
    return 0;
}

static void __exit charge_exit(void)
{
	platform_driver_unregister(&charge_driver);
}
//late_initcall(charge_init);
module_exit(charge_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("huawei charger module driver");
MODULE_AUTHOR("HUAWEI Inc");
