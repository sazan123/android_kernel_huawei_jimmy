/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
//+add by ontim
#include <mt_gpio.h>
#include "mach/gpio_const.h" 

#include <linux/switch.h>
//-add by ontim
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)


/*#define DEBUG_LEDS_STROBE*/
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif


//+add by ontim
#ifndef GPIO_FLASH_LED_EN
#define GPIO_FLASH_LED_EN	(GPIO8 | 0x80000000)
#endif
#ifndef GPIO_TORCH_EN
#define GPIO_TORCH_EN 		(GPIO9 | 0x80000000)
#endif
//-add by ontim


/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);


#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */

//static int g_bLtVersion;

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *RT4505_i2c_client;

//+add by ontim
extern int touch_flash_enable;// use for bootinfo.c 20170310 byyzm

struct switch_dev touch_flash;
//-add by ontim

struct RT4505_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct RT4505_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct RT4505_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};
/*
static int RT4505_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writing at 0x%02x\n", reg);
	return ret;
}
*/
static int RT4505_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}




static int RT4505_chip_init(struct RT4505_chip_data *chip)
{


	return 0;
}

static int RT4505_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct RT4505_chip_data *chip;
	struct RT4505_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("RT4505_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("RT4505 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct RT4505_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("RT4505 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct RT4505_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (RT4505_chip_init(chip) < 0)
		goto err_chip_init;

	RT4505_i2c_client = client;
	PK_DBG("RT4505 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("RT4505 probe is failed\n");
	return -ENODEV;
}

static int RT4505_remove(struct i2c_client *client)
{
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define RT4505_NAME "leds-RT4505"
static const struct i2c_device_id RT4505_id[] = {
	{RT4505_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id RT4505_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver RT4505_i2c_driver = {
	.driver = {
		   .name = RT4505_NAME,
#ifdef CONFIG_OF
		   .of_match_table = RT4505_of_match,
#endif
		   },
	.probe = RT4505_probe,
	.remove = RT4505_remove,
	.id_table = RT4505_id,
};
static int __init RT4505_init(void)
{
	PK_DBG("RT4505_init\n");
	return i2c_add_driver(&RT4505_i2c_driver);
}

static void __exit RT4505_exit(void)
{
	i2c_del_driver(&RT4505_i2c_driver);
}


module_init(RT4505_init);
module_exit(RT4505_exit);

MODULE_DESCRIPTION("Flash driver for RT4505");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

	int val;

	val = RT4505_read_reg(RT4505_i2c_client, reg);
	return (int)val;
}

int FL_Enable(void)
{
#if 0   //modify by ontim
	char buf[2];
/* char bufR[2]; */
	if (g_duty < 0)
		g_duty = 0;
	else if (g_duty > 16)
		g_duty = 16;
	if (g_duty <= 2) {
		int val;

		if (g_bLtVersion == 1) {
			if (g_duty == 0)
				val = 3;
			else if (g_duty == 1)
				val = 5;
			else	/* if(g_duty==2) */
				val = 7;
		} else {
			if (g_duty == 0)
				val = 1;
			else if (g_duty == 1)
				val = 2;
			else	/* if(g_duty==2) */
				val = 3;
		}
		buf[0] = 9;
		buf[1] = val << 4;
		/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
		RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);

		buf[0] = 10;
		buf[1] = 0x02;
		/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
		RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	} else {
		int val;

		val = (g_duty - 1);
		buf[0] = 9;
		buf[1] = val;
		/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
		RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);

		buf[0] = 10;
		buf[1] = 0x03;
		/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
		RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	}
	PK_DBG(" FL_Enable line=%d\n", __LINE__);

	readReg(0);
	readReg(1);
	readReg(6);
	readReg(8);
	readReg(9);
	readReg(0xa);
	readReg(0xb);
#else
	if (g_duty == 1)
{
   printk("%s(%d) camera flashlight on  g_duty=%d byyzm\n",__FUNCTION__,__LINE__,g_duty);
   mt_set_gpio_mode(GPIO_FLASH_LED_EN, GPIO_MODE_00);//GPIO_FLASH_LED_EN_M_GPIO);
   mt_set_gpio_dir(GPIO_FLASH_LED_EN, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO_FLASH_LED_EN, GPIO_OUT_ONE);//camera flash
}
else
{
   printk("%s(%d) torch on  g_duty=%d byyzm\n",__FUNCTION__,__LINE__,g_duty);
   mt_set_gpio_mode(GPIO_TORCH_EN, GPIO_MODE_00);//GPIO_TORCH_EN_M_GPIO);
   mt_set_gpio_dir(GPIO_TORCH_EN, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO_TORCH_EN, GPIO_OUT_ONE);//touch flash
}
#endif
	return 0;
}



int FL_Disable(void)
{
#if 0  //modify by ontim
	char buf[2];

/* ///////////////////// */
	buf[0] = 10;
	buf[1] = 0x00;
	/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	PK_DBG(" FL_Disable line=%d\n", __LINE__);
#else
   if (g_duty == 1)
{
   printk("%s(%d) camera flashlight off  g_duty=%d byyzm\n",__FUNCTION__,__LINE__,g_duty);
   mt_set_gpio_mode(GPIO_FLASH_LED_EN, GPIO_MODE_00);//GPIO_FLASH_LED_EN_M_GPIO);
   mt_set_gpio_dir(GPIO_FLASH_LED_EN, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO_FLASH_LED_EN, GPIO_OUT_ZERO);//camera flash
}
else
{
   printk("%s(%d) torch off  g_duty=%d byyzm\n",__FUNCTION__,__LINE__,g_duty);
   mt_set_gpio_mode(GPIO_TORCH_EN, GPIO_MODE_00);//GPIO_TORCH_EN_M_GPIO);
   mt_set_gpio_dir(GPIO_TORCH_EN, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO_TORCH_EN, GPIO_OUT_ZERO);//touch flash

}
	g_duty=0;
#endif
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	printk("%s(%d)  byyzm\n",__FUNCTION__,__LINE__);
	g_duty = duty;
#if 1  //add by ontim
if (g_duty == 1)
{
//printk(" wwwwwww FL_dim_duty 0 line=%d,duty:%d\n",__LINE__,duty);
   mt_set_gpio_mode(GPIO_FLASH_LED_EN, GPIO_MODE_00);//GPIO_FLASH_LED_EN_M_GPIO);
   mt_set_gpio_dir(GPIO_FLASH_LED_EN, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO_FLASH_LED_EN, GPIO_OUT_ONE);//camera flash

}
else
{
//printk(" wwwwwww FL_dim_duty 0 line=%d,duty:%d\n",__LINE__,duty);
   mt_set_gpio_mode(GPIO_TORCH_EN, GPIO_MODE_00);//GPIO_TORCH_EN_M_GPIO);
   mt_set_gpio_dir(GPIO_TORCH_EN, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO_TORCH_EN, GPIO_OUT_ONE);//touch flash

}
#endif
	return 0;
}




int FL_Init(void)
{
#if 0  //modify by ontim
	int regVal0;
	char buf[2];

	buf[0] = 0xa;
	buf[1] = 0x0;
	/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);

	buf[0] = 0x8;
	buf[1] = 0x47;
	/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);

	buf[0] = 9;
	buf[1] = 0x35;
	/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);




	/* static int RT4505_read_reg(struct i2c_client *client, u8 reg) */
	/* regVal0 = readReg(0); */
	regVal0 = RT4505_read_reg(RT4505_i2c_client, 0);

	if (regVal0 == 1)
		g_bLtVersion = 1;
	else
		g_bLtVersion = 0;


	PK_DBG(" FL_Init regVal0=%d isLtVer=%d\n", regVal0, g_bLtVersion);


/*
	if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!!\n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!!\n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!!\n");}

	if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!!\n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!!\n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!!\n");}
    */




/*	PK_DBG(" FL_Init line=%d\n", __LINE__); */
#else
   mt_set_gpio_mode(GPIO_TORCH_EN, GPIO_MODE_00);//GPIO_TORCH_EN_M_GPIO);
   mt_set_gpio_dir(GPIO_TORCH_EN, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO_TORCH_EN, GPIO_OUT_ZERO);//touch flash

   mt_set_gpio_mode(GPIO_FLASH_LED_EN, GPIO_MODE_00);//GPIO_FLASH_LED_EN_M_GPIO);
   mt_set_gpio_dir(GPIO_FLASH_LED_EN, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO_FLASH_LED_EN, GPIO_OUT_ZERO);//camera flash

   touch_flash_enable=0;
   switch_set_state((struct switch_dev *)&touch_flash, touch_flash_enable);
#endif
	return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}


static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/	printk("%s(%d)  byyzm\n",__FUNCTION__,__LINE__);
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;
	printk("%s(%d)  byyzm\n",__FUNCTION__,__LINE__);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");
	printk("%s(%d)  byyzm\n",__FUNCTION__,__LINE__);
	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}
EXPORT_SYMBOL(constantFlashlightInit);


/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
