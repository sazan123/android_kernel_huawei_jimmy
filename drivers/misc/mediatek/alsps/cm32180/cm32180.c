/*
 * Author: yucong xiong <yucong.xion@mediatek.com>
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
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
/*#include <asm/uaccess.h>*/
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
/*#include <asm/atomic.h>*/

/* #include <mach/mt_typedefs.h> */
/* #include <mach/mt_gpio.h> */
/* #include <mach/mt_pm_ldo.h> */

#include <alsps.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif

/* #define POWER_NONE_MACRO MT65XX_POWER_NONE */

/* #include <linux/hwmsensor.h> */
/* #include <linux/hwmsen_dev.h> */
/* #include <linux/sensors_io.h> */

/*#include <asm/io.h>*/
/* #include <cust_eint.h> */
#include <cust_alsps.h>
#include "cm32180.h"
#include <linux/sched.h>
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define CM32180_DEV_NAME     "cm32180"
/*----------------------------------------------------------------------------*/
#define CM32180_DEBUG 0
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               pr_debug(APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)    pr_err(APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#if CM32180_DEBUG
#define APS_LOG(fmt, args...)    pr_debug(APS_TAG fmt, ##args)
#else
#define APS_LOG(fmt, args...)
#endif

#define I2C_FLAG_WRITE		0
#define I2C_FLAG_READ		1
#define I2C_FLAG_READ_ARA	2

#define CAPELLA_ORIGINAL

/*----------------------------------------------------------------------------*/
static int cm32180_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cm32180_i2c_remove(struct i2c_client *client);
/* static int cm32180_i2c_detect(struct i2c_client *client, struct i2c_board_info *info); */
static int cm32180_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int cm32180_i2c_resume(struct i2c_client *client);

static int cm32180_local_init(void);
static int cm32180_remove(void);
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id cm32180_i2c_id[] = { {CM32180_DEV_NAME, 0}, {} };

/* static struct i2c_board_info __initdata i2c_cm32180={ I2C_BOARD_INFO(CM32180_DEV_NAME, 0x60)}; */
static unsigned long long int_top_time;

/* Maintain alsps cust info here */
/* For alsp driver get cust info */
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
/*
struct alsps_hw *get_cust_alsps(void)
{
	return &alsps_cust;
}
*/
/* struct platform_device *alspsPltFmDev; */

/*----------------------------------------------------------------------------*/
struct cm32180_priv {
	struct alsps_hw *hw;
	struct i2c_client *client;
	struct work_struct eint_work;
#ifdef CUSTOM_KERNEL_SENSORHUB
	struct work_struct init_done_work;
#endif

	/*misc */
	u16 als_modulus;
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t als_debounce;	/*debounce time after enabling als */
	atomic_t als_deb_on;	/*indicates if the debounce is on */
	atomic_t als_deb_end;	/*the jiffies representing the end of debounce */
	atomic_t ps_mask;	/*mask ps: always return far away */
	atomic_t ps_debounce;	/*debounce time after enabling ps */
	atomic_t ps_deb_on;	/*indicates if the debounce is on */
	atomic_t ps_deb_end;	/*the jiffies representing the end of debounce */
	atomic_t ps_suspend;
	atomic_t trace;

	atomic_t init_done;
	struct device_node *irq_node;
	int irq;

	/*data */
	u32 als;
	u32 ps;
	/* u8                    _align; */
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL - 1];
	u32 als_value[C_CUST_ALS_LEVEL];
	int ps_cali;

	atomic_t als_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_high;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_low;	/*the cmd value can't be read, stored in ram */
	atomic_t als_thd_val_high;	/*the cmd value can't be read, stored in ram */
	atomic_t als_thd_val_low;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val;
	ulong enable;		/*enable mask */
	ulong pending_intr;	/*pending interrupt */

	/*early suspend */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

static struct i2c_driver cm32180_i2c_driver = {
	.probe = cm32180_i2c_probe,
	.remove = cm32180_i2c_remove,
	/* .detect     = cm32180_i2c_detect, */
	.suspend = cm32180_i2c_suspend,
	.resume = cm32180_i2c_resume,
	.id_table = cm32180_i2c_id,
	.driver = {
		   .name = CM32180_DEV_NAME,
#ifdef CONFIG_OF
		   .of_match_table = alsps_of_match,
#endif
		   },
};

/*----------------------------------------------------------------------------*/
struct PS_CALI_DATA_STRUCT {
	int close;
	int far_away;
	int valid;
};

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct i2c_client *cm32180_i2c_client;
static struct cm32180_priv *cm32180_obj;

static int cm32180_init_flag = -1;	/* 0<==>OK -1 <==> fail */

static struct alsps_init_info cm32180_init_info = {
	.name = "cm32180",
	.init = cm32180_local_init,
	.uninit = cm32180_remove,

};

/*----------------------------------------------------------------------------*/

static DEFINE_MUTEX(cm32180_mutex);


/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS = 1,
	CMC_BIT_PS = 2,
} CMC_BIT;
/*-----------------------------CMC for debugging-------------------------------*/
typedef enum {
	CMC_TRC_ALS_DATA = 0x0001,
	CMC_TRC_PS_DATA = 0x0002,
	CMC_TRC_EINT = 0x0004,
	CMC_TRC_IOCTL = 0x0008,
	CMC_TRC_I2C = 0x0010,
	CMC_TRC_CVT_ALS = 0x0020,
	CMC_TRC_CVT_PS = 0x0040,
	CMC_TRC_DEBUG = 0x8000,
} CMC_TRC;
/*-----------------------------------------------------------------------------*/

#if 1/* ndef CAPELLA_ORIGINAL */
static int i2c_write(struct i2c_client *client, uint8_t regaddr, uint8_t txbyte, uint8_t *data)
{
	int ret = 0;
	unsigned char *buffer = NULL;
	struct i2c_msg msg;

	memset(&msg, 0, sizeof(struct i2c_msg));
	buffer = kzalloc(txbyte + 1, GFP_KERNEL);
	if (buffer == NULL)
		return -1;
	buffer[0] = regaddr;
	memcpy((buffer + 1), data, txbyte);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = txbyte + 1;
	msg.buf = buffer;

	ret = i2c_transfer(client->adapter, &msg, 1);

	kfree(buffer);
	return ret;
}

#ifdef CONFIG_MTK_I2C_EXTENSION
static int i2c_read(struct i2c_client *client, uint8_t regaddr, uint8_t rxbyte, uint8_t *data)
{
	int ret = 0;
	struct i2c_msg msg;

	memset(&msg, 0, sizeof(struct i2c_msg));

	data[0] = regaddr;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ((rxbyte & 0x1f) << 8) | 1;
	msg.buf = data;
	msg.ext_flag = I2C_WR_FLAG | I2C_RS_FLAG;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret;
}

#else
static int i2c_read(struct i2c_client *client, uint8_t regaddr, uint8_t rxbyte, uint8_t *data)
{
	int err;
	struct i2c_msg msgs[2] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &regaddr},
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = rxbyte,
		 .buf = data,
		 }
	};
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	return err;
}
#endif

int CM32180_i2c_master_operate(struct i2c_client *client, char *data, int len, int i2c_flag)
{
	int res = 0;
	struct i2c_msg msg;

	memset(&msg, 0, sizeof(struct i2c_msg));

	if (!client)
		return -EINVAL;

	mutex_lock(&cm32180_mutex);
	switch (i2c_flag) {
	case I2C_FLAG_WRITE:
		res = i2c_write(client, data[0], len, &data[1]);
		break;
	case I2C_FLAG_READ:
		res = i2c_read(client, data[0], len, data);
		break;
	case I2C_FLAG_READ_ARA:
		msg.addr = data[0],
		msg.flags = I2C_M_RD,
		msg.len = len,
		msg.buf = data,
		res = i2c_transfer(client->adapter, &msg, 1);
		res = i2c_transfer(client->adapter, &msg, 1);
		break;
	default:
		APS_LOG("CM32180_i2c_master_operate i2c_flag command not support!\n");
		break;
	}
	if (res < 0)
		goto EXIT_ERR;
	mutex_unlock(&cm32180_mutex);
	return res;
EXIT_ERR:
	mutex_unlock(&cm32180_mutex);
	APS_ERR("CM32180_i2c_master_operate fail(%d)\n", i2c_flag);
	return res;
}

#else

int CM32180_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	struct i2c_msg msg;
	int res = 0;

	mutex_lock(&cm32180_mutex);
	switch (i2c_flag) {
	case I2C_FLAG_WRITE:
		client->addr &= I2C_MASK_FLAG;
		/* res = i2c_master_send(client, buf, count); */
		res = i2c_master_send(client, buf, count + 1);
		client->addr &= I2C_MASK_FLAG;
		break;
	case I2C_FLAG_READ:
		client->addr &= I2C_MASK_FLAG;
		client->addr |= I2C_WR_FLAG;
		client->addr |= I2C_RS_FLAG;
		/* res = i2c_master_send(client, buf, count); */
		count = 0x200;
		res = i2c_master_send(client, buf, count + 1);
		client->addr &= I2C_MASK_FLAG;
		break;
	case I2C_FLAG_READ_ARA:
		msg.addr = 0x0C;
		msg.flags = client->flags & I2C_M_TEN;
		msg.flags |= I2C_M_RD;
		msg.timing = client->timing;
		/* msg.len = count; */
		msg.len = 1;
		msg.buf = (uint8_t *) buf;
		res = i2c_transfer(client->adapter, &msg, 1);
		res = i2c_transfer(client->adapter, &msg, 1);
		break;
	default:
		APS_LOG("CM32180_i2c_master_operate i2c_flag command not support!\n");
		break;
	}
	if (res < 0)
		goto EXIT_ERR;
	mutex_unlock(&cm32180_mutex);
	return res;
EXIT_ERR:
	mutex_unlock(&cm32180_mutex);
	APS_ERR("CM32180_i2c_master_operate fail\n");
	return res;
}
#endif

/*----------------------------------------------------------------------------*/
static void cm32180_power(struct alsps_hw *hw, unsigned int on)
{

}

/********************************************************************/
int cm32180_enable_ps(struct i2c_client *client, int enable)
{
	struct cm32180_priv *obj = i2c_get_clientdata(client);
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else
	/* u8 databuf[3]; */
#endif

	if (client) {
		APS_LOG("cm32180 don't support ps!\n");
		return 0;
	} else
		return -EINVAL;

	if (enable == 1) {
		APS_LOG("cm32180_enable_ps enable_ps\n");
#ifdef CUSTOM_KERNEL_SENSORHUB
		req.activate_req.sensorType = ID_PROXIMITY;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
		if (res) {
			APS_ERR("SCP_sensorHub_req_send!\n");
			goto ENABLE_PS_EXIT_ERR;
		}
#else
		/* res = cm32180_enable_eint(client); */
		enable_irq(cm32180_obj->irq);
		if (res != 0) {
			APS_ERR("disable eint fail: %d\n", res);
			return res;
		}
/*
	databuf[0]= CM32180_REG_PS_CONF3_MS;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_READ);
	if(res < 0)
	{
	    APS_ERR("i2c_master_send function err\n");
	    goto ENABLE_PS_EXIT_ERR;
	}
	APS_LOG("CM32180_REG_PS_CONF3_MS value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

	databuf[0]= CM32180_REG_PS_CANC;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_READ);
	if(res < 0)
	{
	    APS_ERR("i2c_master_send function err\n");
	    goto ENABLE_PS_EXIT_ERR;
	}
	APS_LOG("CM32180_REG_PS_CANC value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

	databuf[0]= CM32180_REG_PS_CONF1_2;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_READ);
	if(res < 0)
	{
	    APS_ERR("i2c_master_send function err\n");
	    goto ENABLE_PS_EXIT_ERR;
	}
	APS_LOG("CM32180_REG_PS_CONF1_2 value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

	databuf[2] = databuf[1];
	databuf[1] = databuf[0]&0xFE;

	databuf[0]= CM32180_REG_PS_CONF1_2;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res < 0)
	{
	    APS_ERR("i2c_master_send function err\n");
	    goto ENABLE_PS_EXIT_ERR;
	}
*/
#endif
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end,
			   jiffies + atomic_read(&obj->ps_debounce) / (1000 / HZ));
	} else {
		APS_LOG("cm32180_enable_ps disable_ps\n");
#ifdef CUSTOM_KERNEL_SENSORHUB
		req.activate_req.sensorType = ID_PROXIMITY;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
		if (res) {
			APS_ERR("SCP_sensorHub_req_send!\n");
			goto ENABLE_PS_EXIT_ERR;
		}
#else
/*
	databuf[0]= CM32180_REG_PS_CONF1_2;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_READ);
	if(res < 0)
	{
	    APS_ERR("i2c_master_send function err\n");
	    goto ENABLE_PS_EXIT_ERR;
	}

	APS_LOG("CM32180_REG_PS_CONF1_2 value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

	databuf[2] = databuf[1];
	databuf[1] = databuf[0]|0x01;
	databuf[0]= CM32180_REG_PS_CONF1_2;

	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res < 0)
	{
	    APS_ERR("i2c_master_send function err\n");
	    goto ENABLE_PS_EXIT_ERR;
	}
*/
		/* res = cm32180_disable_eint(client); */
		disable_irq_nosync(cm32180_obj->irq);
		if (res != 0) {
			APS_ERR("disable eint fail: %d\n", res);
			return res;
		}
#endif
		atomic_set(&obj->ps_deb_on, 0);
	}

	return 0;
	/* ENABLE_PS_EXIT_ERR: */
	return res;
}

/********************************************************************/
int cm32180_enable_als(struct i2c_client *client, int enable)
{
	struct cm32180_priv *obj = i2c_get_clientdata(client);
	int res;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else
	u8 databuf[3];
#endif

	if (enable == 1) {
		APS_LOG("cm32180_enable_als enable_als\n");
#ifdef CUSTOM_KERNEL_SENSORHUB
		req.activate_req.sensorType = ID_PROXIMITY;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
		if (res) {
			APS_ERR("SCP_sensorHub_req_send!\n");
			goto ENABLE_ALS_EXIT_ERR;
		}
#else
		databuf[0] = CM32180_REG_ALS_CONF;
		if (1 == obj->hw->polling_mode_als) {
			databuf[1] = CM32180_ALS_CONF_CONFIG_ON & 0x00FF;
			databuf[2] = CM32180_ALS_CONF_CONFIG_ON >> 8;
		} else {
			databuf[1] = CM32180_ALS_CONF_CONFIG_ON_INT & 0x00FF;
			databuf[2] = CM32180_ALS_CONF_CONFIG_ON_INT >> 8;
		}

		res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res < 0) {
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_ALS_EXIT_ERR;
		}
#endif
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end,
			   jiffies + atomic_read(&obj->als_debounce) / (1000 / HZ));
	} else {
		APS_LOG("cm32180_enable_als disable_als\n");
#ifdef CUSTOM_KERNEL_SENSORHUB
		req.activate_req.sensorType = ID_PROXIMITY;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
		if (res) {
			APS_ERR("SCP_sensorHub_req_send!\n");
			goto ENABLE_ALS_EXIT_ERR;
		}
#else
		databuf[0] = CM32180_REG_ALS_CONF;
		if (1 == obj->hw->polling_mode_als) {
			databuf[1] = CM32180_ALS_CONF_CONFIG_OFF & 0x00FF;
			databuf[2] = CM32180_ALS_CONF_CONFIG_OFF >> 8;
		} else {
			databuf[1] = CM32180_ALS_CONF_CONFIG_OFF_INT & 0x00FF;
			databuf[2] = CM32180_ALS_CONF_CONFIG_OFF_INT >> 8;
		}
		res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res < 0) {
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_ALS_EXIT_ERR;
		}
#endif
		atomic_set(&obj->als_deb_on, 0);
	}
	return 0;
ENABLE_ALS_EXIT_ERR:
	return res;
}

/********************************************************************/
long cm32180_read_ps(struct i2c_client *client, u32 *data)
{
	long res;
	struct cm32180_priv *obj = i2c_get_clientdata(client);
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else
	u8 databuf[2] = { 0 };
#endif
	/* APS_FUN(f); */
	if (client) {
		APS_LOG("cm32180 don't support ps!\n");
		return 0;
	} else
		return -EINVAL;

#ifdef CUSTOM_KERNEL_SENSORHUB
	req.get_data_req.sensorType = ID_PROXIMITY;
	req.get_data_req.action = SENSOR_HUB_GET_DATA;
	len = sizeof(req.get_data_req);
	res = SCP_sensorHub_req_send(&req, &len, 1);
	if (res) {
		APS_ERR("SCP_sensorHub_req_send!\n");
		goto READ_PS_EXIT_ERR;
	}

	len -= offsetof(SCP_SENSOR_HUB_GET_DATA_RSP, u8Data);

	if (len >= 0 && len < 4)
		memcpy(data, &(req.get_data_rsp.u8Data), len);
	else
		APS_ERR("data length fail : %d\n", len);
/*
	if (atomic_read(&obj->trace) & CMC_TRC_PS_DATA) {
		 show data
	}
*/
#else
/*
	databuf[0] = CM32180_REG_PS_DATA;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_READ);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_PS_EXIT_ERR;
	}

	//APS_LOG("CM32180_REG_PS_DATA value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
*/
	if (databuf[0] < obj->ps_cali)
		*data = 0;
	else
		*data = databuf[0] - obj->ps_cali;
#endif

	return 0;
	/* READ_PS_EXIT_ERR: */
	return res;
}

/********************************************************************/
long cm32180_read_als(struct i2c_client *client, u32 *data)
{
	long res;
#ifdef CUSTOM_KERNEL_SENSORHUB
	struct cm32180_priv *obj = i2c_get_clientdata(client);
	SCP_SENSOR_HUB_DATA req;
	int len;
#else
	u8 databuf[3] = { 0 };
#endif

	/* APS_FUN(f); */

#ifdef CUSTOM_KERNEL_SENSORHUB
	req.get_data_req.sensorType = ID_LIGHT;
	req.get_data_req.action = SENSOR_HUB_GET_DATA;
	len = sizeof(req.get_data_req);
	res = SCP_sensorHub_req_send(&req, &len, 1);
	if (res) {
		APS_ERR("SCP_sensorHub_req_send!\n");
		goto READ_ALS_EXIT_ERR;
	}

	len -= offsetof(SCP_SENSOR_HUB_GET_DATA_RSP, u8Data);

	if (len >= 0 && len < 4)
		memcpy(data, &(req.get_data_rsp.u8Data), len);
	else
		APS_ERR("data length fail : %d\n", len);
/*
	if (atomic_read(&obj->trace) & CMC_TRC_ALS_DATA) {
		show data
	}
*/
#else
	databuf[0] = CM32180_REG_ALS_DATA;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_READ);
	if (res < 0) {
		APS_ERR("i2c_master_send function err\n");
		goto READ_ALS_EXIT_ERR;
	}
	APS_LOG("CM32180_REG_ALS_DATA value value_low = %x, value_high = %x\n", databuf[0],
		databuf[1]);

	data[0] = ((databuf[1] << 8) | databuf[0]);
#endif

	return 0;
READ_ALS_EXIT_ERR:
	return res;
}

/********************************************************************/
static int cm32180_get_ps_value(struct cm32180_priv *obj, u32 ps)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
	return ps;
#else
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;

	val = 0;

	if (ps > atomic_read(&obj->ps_thd_val_high))
		val = 0;	/*close */
	else if (ps < atomic_read(&obj->ps_thd_val_low))
		val = 1;	/*far away */

	if (atomic_read(&obj->ps_suspend))
		invalid = 1;
	else if (1 == atomic_read(&obj->ps_deb_on)) {
		unsigned long endt = atomic_read(&obj->ps_deb_end);

		if (time_after(jiffies, endt))
			atomic_set(&obj->ps_deb_on, 0);

		if (1 == atomic_read(&obj->ps_deb_on))
			invalid = 1;
	}

	if (!invalid) {
		if (unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS)) {
			if (mask)
				APS_LOG("PS:  %05d => %05d [M]\n", ps, val);
			else
				APS_LOG("PS:  %05d => %05d\n", ps, val);
		}
		if (0 == test_bit(CMC_BIT_PS, &obj->enable)) {
			/* if ps is disable do not report value */
			APS_LOG("PS: not enable and do not report this value\n");
			return -1;
		} else
			return val;
	} else {
		if (unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
			APS_LOG("PS:  %05d => %05d (-1)\n", ps, val);
		return -1;
	}
#endif
}

/********************************************************************/
static int cm32180_get_als_value(struct cm32180_priv *obj, u32 als)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
	return als;
#else
	int idx;
	int invalid = 0;

	for (idx = 0; idx < obj->als_level_num; idx++) {
		if (als < obj->hw->als_level[idx])
			break;
	}
	if (idx >= obj->als_value_num) {
		APS_ERR("exceed range\n");
		idx = obj->als_value_num - 1;
	}

	if (1 == atomic_read(&obj->als_deb_on)) {
		unsigned long endt = atomic_read(&obj->als_deb_end);

		if (time_after(jiffies, endt))
			atomic_set(&obj->als_deb_on, 0);

		if (1 == atomic_read(&obj->als_deb_on))
			invalid = 1;
	}

	if (!invalid) {
#if defined(CONFIG_MTK_AAL_SUPPORT)
		int level_high = obj->hw->als_level[idx];
		int level_low = (idx > 0) ? obj->hw->als_level[idx - 1] : 0;
		int level_diff = level_high - level_low;
		int value_high = obj->hw->als_value[idx];
		int value_low = (idx > 0) ? obj->hw->als_value[idx - 1] : 0;
		int value_diff = value_high - value_low;
		int value = 0;

		if ((level_low >= level_high) || (value_low >= value_high))
			value = value_low;
		else
			value =
			    (level_diff * value_low + (als - level_low) * value_diff +
			     ((level_diff + 1) >> 1)) / level_diff;

		APS_LOG("ALS: %d [%d, %d] => %d [%d, %d]\n", als, level_low, level_high, value,
			value_low, value_high);
		return value;
#endif
		if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
			APS_LOG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);

		return obj->hw->als_value[idx];
	}
	if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
		APS_LOG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
	return -1;
#endif
}


/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t cm32180_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n",
		       atomic_read(&cm32180_obj->i2c_retry),
		       atomic_read(&cm32180_obj->als_debounce), atomic_read(&cm32180_obj->ps_mask),
		       atomic_read(&cm32180_obj->ps_thd_val),
		       atomic_read(&cm32180_obj->ps_debounce));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	}

	if (5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb)) {
		atomic_set(&cm32180_obj->i2c_retry, retry);
		atomic_set(&cm32180_obj->als_debounce, als_deb);
		atomic_set(&cm32180_obj->ps_mask, mask);
		atomic_set(&cm32180_obj->ps_thd_val, thres);
		atomic_set(&cm32180_obj->ps_debounce, ps_deb);
	} else {
		APS_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&cm32180_obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&cm32180_obj->trace, trace);
	else
		APS_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_show_als(struct device_driver *ddri, char *buf)
{
	int res;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	}
	res = cm32180_read_als(cm32180_obj->client, &cm32180_obj->als);
	if (res)
		res = snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	else
		res = snprintf(buf, PAGE_SIZE, "0x%04X\n", cm32180_obj->als);
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_show_ps(struct device_driver *ddri, char *buf)
{
	int res;

	if (!cm32180_obj) {
		APS_ERR("cm3623_obj is null!!\n");
		return 0;
	}
	res = cm32180_read_ps(cm32180_obj->client, &cm32180_obj->ps);
	if (res)
		res = snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	else
		res = snprintf(buf, PAGE_SIZE, "0x%04X\n", cm32180_obj->ps);
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_show_reg(struct device_driver *ddri, char *buf)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_show_send(struct device_driver *ddri, char *buf)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	} else if (2 != sscanf(buf, "%x %x", &addr, &cmd)) {
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8) cmd;
	/* **************************** */
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_show_recv(struct device_driver *ddri, char *buf)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	int rc;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	}
	rc = kstrtoint(buf, 0, &addr);
	if (rc != 0) {
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	}

	if (cm32180_obj->hw) {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d, (%d %d)\n",
				cm32180_obj->hw->i2c_num, cm32180_obj->hw->power_id,
				cm32180_obj->hw->power_vol);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "REGS: %02X %02X %02X %02lX %02lX\n",
			atomic_read(&cm32180_obj->als_cmd_val),
			atomic_read(&cm32180_obj->ps_cmd_val),
			atomic_read(&cm32180_obj->ps_thd_val), cm32180_obj->enable,
			cm32180_obj->pending_intr);

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "MISC: %d %d\n",
		     atomic_read(&cm32180_obj->als_suspend), atomic_read(&cm32180_obj->ps_suspend));

	return len;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct cm32180_priv *obj, const char *buf, size_t count, u32 data[],
			     int len)
{
	int idx = 0;
	int rc;
	char *cur = (char *)buf, *end = (char *)(buf + count);

	while (idx < len) {
		while ((cur < end) && IS_SPACE(*cur))
			cur++;

		rc = kstrtoint(cur, 0, &data[idx]);
		if (rc != 0)
			break;

		idx++;
		while ((cur < end) && !IS_SPACE(*cur))
			cur++;
	}
	return idx;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	}

	for (idx = 0; idx < cm32180_obj->als_level_num; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%d ", cm32180_obj->hw->als_level[idx]);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	} else if (!strcmp(buf, "def")) {
		memcpy(cm32180_obj->als_level, cm32180_obj->hw->als_level,
		       sizeof(cm32180_obj->als_level));
	} else if (cm32180_obj->als_level_num !=
		   read_int_from_buf(cm32180_obj, buf, count, cm32180_obj->hw->als_level,
				     cm32180_obj->als_level_num)) {
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	}

	for (idx = 0; idx < cm32180_obj->als_value_num; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%d ", cm32180_obj->hw->als_value[idx]);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm32180_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return 0;
	} else if (!strcmp(buf, "def")) {
		memcpy(cm32180_obj->als_value, cm32180_obj->hw->als_value,
		       sizeof(cm32180_obj->als_value));
	} else if (cm32180_obj->als_value_num !=
		   read_int_from_buf(cm32180_obj, buf, count, cm32180_obj->hw->als_value,
				     cm32180_obj->als_value_num)) {
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}

/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als, S_IWUSR | S_IRUGO, cm32180_show_als, NULL);
static DRIVER_ATTR(ps, S_IWUSR | S_IRUGO, cm32180_show_ps, NULL);
static DRIVER_ATTR(config, S_IWUSR | S_IRUGO, cm32180_show_config, cm32180_store_config);
static DRIVER_ATTR(alslv, S_IWUSR | S_IRUGO, cm32180_show_alslv, cm32180_store_alslv);
static DRIVER_ATTR(alsval, S_IWUSR | S_IRUGO, cm32180_show_alsval, cm32180_store_alsval);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, cm32180_show_trace, cm32180_store_trace);
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, cm32180_show_status, NULL);
static DRIVER_ATTR(send, S_IWUSR | S_IRUGO, cm32180_show_send, cm32180_store_send);
static DRIVER_ATTR(recv, S_IWUSR | S_IRUGO, cm32180_show_recv, cm32180_store_recv);
static DRIVER_ATTR(reg, S_IWUSR | S_IRUGO, cm32180_show_reg, NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *cm32180_attr_list[] = {
	&driver_attr_als,
	&driver_attr_ps,
	&driver_attr_trace,	/*trace log */
	&driver_attr_config,
	&driver_attr_alslv,
	&driver_attr_alsval,
	&driver_attr_status,
	&driver_attr_send,
	&driver_attr_recv,
	&driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int cm32180_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(cm32180_attr_list) / sizeof(cm32180_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, cm32180_attr_list[idx]);
		if (err) {
			APS_ERR("driver_create_file (%s) = %d\n",
					cm32180_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int cm32180_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(cm32180_attr_list) / sizeof(cm32180_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, cm32180_attr_list[idx]);

	return err;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------interrupt functions--------------------------------*/
static int intr_flag;
/*----------------------------------------------------------------------------*/
#ifndef CUSTOM_KERNEL_SENSORHUB
static int cm32180_check_intr(struct i2c_client *client)
{
	return 0;
/*
	int res = 0;
	u8 databuf[2] = {0};
	//u8 intr;
*/
/*
	databuf[0] = CM32180_REG_PS_DATA;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_READ);
	if(res<0)
	{
		APS_ERR("i2c_master_send function err res = %d\n",res);
		goto EXIT_ERR;
	}

	APS_LOG("CM32180_REG_PS_DATA value value_low = %x, value_reserve = %x\n",databuf[0],databuf[1]);

	databuf[0] = CM32180_REG_INT_FLAG;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_READ);
	if(res<0)
	{
		APS_ERR("i2c_master_send function err res = %d\n",res);
		goto EXIT_ERR;
	}

	APS_LOG("CM32180_REG_INT_FLAG value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
*/
/*
	if(databuf[1]&0x02)
	{
		intr_flag = 0;//for close
	}else if(databuf[1]&0x01)
	{
		intr_flag = 1;//for away
	}else{
		res = -1;
		APS_ERR("cm32180_check_intr fail databuf[1]&0x01: %d\n", res);
		goto EXIT_ERR;
	}

	return 0;
	EXIT_ERR:
	APS_ERR("cm32180_check_intr dev: %d\n", res);
	return res;
*/
}
#endif				/* #ifndef CUSTOM_KERNEL_SENSORHUB */
/*----------------------------------------------------------------------------*/
static void cm32180_eint_work(struct work_struct *work)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
#else
	struct cm32180_priv *obj =
	    (struct cm32180_priv *)container_of(work, struct cm32180_priv, eint_work);
#endif				/* #ifdef CUSTOM_KERNEL_SENSORHUB */

	APS_LOG("cm32180 int top half time = %lld\n", int_top_time);

#ifdef CUSTOM_KERNEL_SENSORHUB
	res = ps_report_interrupt_data(intr_flag);
	if (res != 0)
		APS_ERR("cm32180_eint_work err: %d\n", res);
#else
	res = cm32180_check_intr(obj->client);
	if (res != 0)
		goto EXIT_INTR_ERR;
	else {
		APS_LOG("cm32180 interrupt value = %d\n", intr_flag);
		res = ps_report_interrupt_data(intr_flag);
	}


	enable_irq(obj->irq);
	return;
EXIT_INTR_ERR:
	enable_irq(obj->irq);
	APS_ERR("cm32180_eint_work err: %d\n", res);
#endif				/* #ifdef CUSTOM_KERNEL_SENSORHUB */
}

/*----------------------------------------------------------------------------*/
#ifdef CUSTOM_KERNEL_SENSORHUB
static void cm32180_init_done_work(struct work_struct *work)
{
	struct cm32180_priv *obj = cm32180_obj;
	CM32180_CUST_DATA *p_cust_data;
	SCP_SENSOR_HUB_DATA data;
	int max_cust_data_size_per_packet;
	int i;
	uint sizeOfCustData;
	uint len;
	char *p = (char *)obj->hw;

	p_cust_data = (CM32180_CUST_DATA *) data.set_cust_req.custData;

	data.set_cust_req.sensorType = ID_LIGHT;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	sizeOfCustData = sizeof(*(obj->hw));
	p_cust_data->setCust.action = CM32180_CUST_ACTION_SET_CUST;
	max_cust_data_size_per_packet =
	    sizeof(data.set_cust_req.custData) - offsetof(CM32180_SET_CUST, data);

	for (i = 0; sizeOfCustData > 0; i++) {
		p_cust_data->setCust.part = i;
		if (sizeOfCustData > max_cust_data_size_per_packet)
			len = max_cust_data_size_per_packet;
		else
			len = sizeOfCustData;

		memcpy(p_cust_data->setCust.data, p, len);
		sizeOfCustData -= len;
		p += len;

		len +=
		    offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(CM32180_SET_CUST,
									       data);
		SCP_sensorHub_req_send(&data, &len, 1);
	}
}
#endif

/*----------------------------------------------------------------------------*/
#ifndef CUSTOM_KERNEL_SENSORHUB
static void cm32180_eint_func(void)
{
	struct cm32180_priv *obj = cm32180_obj;

	if (!obj)
		return;
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}

#if defined(CONFIG_OF)
static irqreturn_t cm32180_eint_handler(int irq, void *desc)
{
	cm32180_eint_func();
	disable_irq_nosync(cm32180_obj->irq);

	return IRQ_HANDLED;
}
#endif

#else
static int cm32180_irq_handler(void *data, uint len)
{
	struct cm32180_priv *obj = cm32180_obj;
	SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P) data;

	if (!obj)
		return -1;

	switch (rsp->rsp.action) {
	case SENSOR_HUB_NOTIFY:
		switch (rsp->notify_rsp.event) {
		case SCP_INIT_DONE:
			schedule_work(&obj->init_done_work);
			break;
		case SCP_NOTIFY:
			if (CM32180_NOTIFY_PROXIMITY_CHANGE == rsp->notify_rsp.data[0]) {
				intr_flag = rsp->notify_rsp.data[1];
				schedule_work(&obj->eint_work);
			} else {
				APS_ERR("Unknown notify");
			}
			break;
		default:
			APS_ERR("Error sensor hub notify");
			break;
		}
		break;
	default:
		APS_ERR("Error sensor hub action");
		break;
	}

	return 0;
}
#endif				/* #ifdef CUSTOM_KERNEL_SENSORHUB */
/*----------------------------------------------------------------------------*/
int cm32180_setup_eint(struct i2c_client *client)
{
#if defined(CONFIG_OF)
	u32 ints[2] = { 0, 0 };
#endif

	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ret = SCP_sensorHub_rsp_registration(ID_PROXIMITY, cm32180_irq_handler);
	return ret;
#else
	struct platform_device *alspsPltFmDev = NULL;

	alspsPltFmDev = get_alsps_platformdev();

	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");

	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

	}
	pinctrl_select_state(pinctrl, pins_cfg);

	if (cm32180_obj->irq_node) {
		of_property_read_u32_array(cm32180_obj->irq_node, "debounce", ints,
					   ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		cm32180_obj->irq = irq_of_parse_and_map(cm32180_obj->irq_node, 0);
		APS_LOG("cm32180_obj->irq = %d\n", cm32180_obj->irq);
		if (!cm32180_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}

		if (request_irq
		    (cm32180_obj->irq, cm32180_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}

		enable_irq(cm32180_obj->irq);
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}
	return 0;
#endif
}

/*-------------------------------MISC device related------------------------------------------*/



/************************************************************/
static int cm32180_open(struct inode *inode, struct file *file)
{
	file->private_data = cm32180_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/************************************************************/
static int cm32180_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/************************************************************/
#ifndef CUSTOM_KERNEL_SENSORHUB
static int set_psensor_threshold(struct i2c_client *client)
{
	return 0;
/*
	struct cm32180_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];
	int res = 0;
	APS_ERR("set_psensor_threshold function high: 0x%x, low:0x%x\n",
			atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));
*/
/*
	databuf[0] = CM32180_REG_PS_THD;
	databuf[1] = atomic_read(&obj->ps_thd_val_low);
	databuf[2] = atomic_read(&obj->ps_thd_val_high);//threshold value need to confirm
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		return -1;
	}
*/
	return 0;
}
#endif
static long cm32180_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct cm32180_priv *obj = i2c_get_clientdata(client);
	long err = 0;
	void __user *ptr = (void __user *)arg;
	int dat;
	uint32_t enable;
	int ps_result;
	int ps_cali;
	int threshold[2];
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	CM32180_CUST_DATA *pCustData;
	int len;

	data.set_cust_req.sensorType = ID_PROXIMITY;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	pCustData = (CM32180_CUST_DATA *) (&data.set_cust_req.custData);
#endif
	switch (cmd) {
	case ALSPS_SET_PS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		if (enable) {
			err = cm32180_enable_ps(obj->client, 1);
			if (err) {
				APS_ERR("enable ps fail: %ld\n", err);
				goto err_out;
			}

			set_bit(CMC_BIT_PS, &obj->enable);
		} else {
			err = cm32180_enable_ps(obj->client, 0);
			if (err) {
				APS_ERR("disable ps fail: %ld\n", err);
				goto err_out;
			}
			clear_bit(CMC_BIT_PS, &obj->enable);
		}
		break;

	case ALSPS_GET_PS_MODE:
		enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_DATA:
		err = cm32180_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;

		dat = cm32180_get_ps_value(obj, obj->ps);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_RAW_DATA:
		err = cm32180_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;

		dat = obj->ps;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_SET_ALS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		if (enable) {
			err = cm32180_enable_als(obj->client, 1);
			if (err) {
				APS_ERR("enable als fail: %ld\n", err);
				goto err_out;
			}
			set_bit(CMC_BIT_ALS, &obj->enable);
		} else {
			err = cm32180_enable_als(obj->client, 0);
			if (err) {
				APS_ERR("disable als fail: %ld\n", err);
				goto err_out;
			}
			clear_bit(CMC_BIT_ALS, &obj->enable);
		}
		break;

	case ALSPS_GET_ALS_MODE:
		enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_DATA:
		err = cm32180_read_als(obj->client, &obj->als);
		if (err)
			goto err_out;

		dat = cm32180_get_als_value(obj, obj->als);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_RAW_DATA:
		err = cm32180_read_als(obj->client, &obj->als);
		if (err)
			goto err_out;

		dat = obj->als;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	/*-------for factory mode test------*/
	case ALSPS_GET_PS_TEST_RESULT:
		err = cm32180_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;
		if (obj->ps > atomic_read(&obj->ps_thd_val_high))
			ps_result = 0;
		else
			ps_result = 1;

		if (copy_to_user(ptr, &ps_result, sizeof(ps_result))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_IOCTL_CLR_CALI:
		if (copy_from_user(&dat, ptr, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		if (dat == 0)
			obj->ps_cali = 0;

#ifdef CUSTOM_KERNEL_SENSORHUB
		pCustData->clearCali.action = CM32180_CUST_ACTION_CLR_CALI;
		len =
		    offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->clearCali);

		err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

		break;

	case ALSPS_IOCTL_GET_CALI:
		ps_cali = obj->ps_cali;
		if (copy_to_user(ptr, &ps_cali, sizeof(ps_cali))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_IOCTL_SET_CALI:
		if (copy_from_user(&ps_cali, ptr, sizeof(ps_cali))) {
			err = -EFAULT;
			goto err_out;
		}

		obj->ps_cali = ps_cali;

#ifdef CUSTOM_KERNEL_SENSORHUB
		pCustData->setCali.action = CM32180_CUST_ACTION_SET_CALI;
		pCustData->setCali.cali = ps_cali;
		len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);

		err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

		break;

	case ALSPS_SET_PS_THRESHOLD:
		if (copy_from_user(threshold, ptr, sizeof(threshold))) {
			err = -EFAULT;
			goto err_out;
		}
		APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],
			threshold[1]);
		atomic_set(&obj->ps_thd_val_high, (threshold[0] + obj->ps_cali));
		atomic_set(&obj->ps_thd_val_low, (threshold[1] + obj->ps_cali));	/* need to confirm */

#ifdef CUSTOM_KERNEL_SENSORHUB
		pCustData->setPSThreshold.action = CM32180_CUST_ACTION_SET_PS_THRESHODL;
		pCustData->setPSThreshold.threshold[0] = threshold[0];
		pCustData->setPSThreshold.threshold[1] = threshold[1];
		len =
		    offsetof(SCP_SENSOR_HUB_SET_CUST_REQ,
			     custData) + sizeof(pCustData->setPSThreshold);

		err = SCP_sensorHub_req_send(&data, &len, 1);
#else
		set_psensor_threshold(obj->client);
#endif

		break;

	case ALSPS_GET_PS_THRESHOLD_HIGH:
		threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
		APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
		if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_THRESHOLD_LOW:
		threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
		APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
		if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
			err = -EFAULT;
			goto err_out;
		}
		break;
			/*------------------------------------------------------------------------------------------*/

	default:
		APS_ERR("%s not supported = 0x%04x", __func__, cmd);
		err = -ENOIOCTLCMD;
		break;
	}

err_out:
	return err;
}

#ifdef CONFIG_COMPAT
static long compat_cm32180_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	APS_FUN();

	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		APS_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_ALSPS_SET_PS_MODE:
	case COMPAT_ALSPS_GET_PS_MODE:
	case COMPAT_ALSPS_GET_PS_DATA:
	case COMPAT_ALSPS_GET_PS_RAW_DATA:
	case COMPAT_ALSPS_SET_ALS_MODE:
	case COMPAT_ALSPS_GET_ALS_MODE:
	case COMPAT_ALSPS_GET_ALS_DATA:
	case COMPAT_ALSPS_GET_ALS_RAW_DATA:
	case COMPAT_ALSPS_GET_PS_TEST_RESULT:
	case COMPAT_ALSPS_GET_ALS_TEST_RESULT:
	case COMPAT_ALSPS_GET_PS_THRESHOLD_HIGH:
	case COMPAT_ALSPS_GET_PS_THRESHOLD_LOW:
	case COMPAT_ALSPS_GET_ALS_THRESHOLD_HIGH:
	case COMPAT_ALSPS_GET_ALS_THRESHOLD_LOW:
	case COMPAT_ALSPS_IOCTL_CLR_CALI:
	case COMPAT_ALSPS_IOCTL_GET_CALI:
	case COMPAT_ALSPS_IOCTL_SET_CALI:
	case COMPAT_ALSPS_SET_PS_THRESHOLD:
	case COMPAT_ALSPS_SET_ALS_THRESHOLD:
	case COMPAT_AAL_SET_ALS_MODE:
	case COMPAT_AAL_GET_ALS_MODE:
	case COMPAT_AAL_GET_ALS_DATA:
		return filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
	default:
		APS_ERR("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif
/********************************************************************/

/*------------------------------misc device related operation functions------------------------------------*/
static const struct file_operations cm32180_fops = {
	.owner = THIS_MODULE,
	.open = cm32180_open,
	.release = cm32180_release,
	.unlocked_ioctl = cm32180_unlocked_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_cm32180_unlocked_ioctl,
#endif
};

static struct miscdevice cm32180_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &cm32180_fops,
};

/*--------------------------------------------------------------------------------------*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void cm32180_early_suspend(struct early_suspend *h)
{
	struct cm32180_priv *obj = container_of(h, struct cm32180_priv, early_drv);
	int err;

	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 1);
	err = cm32180_enable_als(obj->client, 0);
	if (err)
		APS_ERR("disable als fail: %d\n", err);
}

static void cm32180_late_resume(struct early_suspend *h)
{
	struct cm32180_priv *obj = container_of(h, struct cm32180_priv, early_drv);
	int err;
	hwm_sensor_data sensor_data;

	memset(&sensor_data, 0, sizeof(sensor_data));
	APS_FUN();
	if (!obj) {
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if (test_bit(CMC_BIT_ALS, &obj->enable)) {
		err = cm32180_enable_als(obj->client, 1);
		if (err)
			APS_ERR("enable als fail: %d\n", err);
	}
}
#endif
/*--------------------------------------------------------------------------------*/
static int cm32180_init_client(struct i2c_client *client)
{
	struct cm32180_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];
	int res = 0;

	/* Clear any stale interrupts */
	databuf[0] = 0x0C; /* address */
	CM32180_i2c_master_operate(client, databuf, 0x1, I2C_FLAG_READ_ARA);

	/* Initialize 0x03 register */
	databuf[0] = 0x03;
	databuf[1] = 0x0;
	databuf[2] = 0x0;
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}

	databuf[0] = CM32180_REG_ALS_CONF;
	if (1 == obj->hw->polling_mode_als) {
		databuf[1] = CM32180_ALS_CONF_CONFIG_ON & 0x00FF;
		databuf[2] = CM32180_ALS_CONF_CONFIG_ON >> 8;
	} else {
		databuf[1] = CM32180_ALS_CONF_CONFIG_ON_INT & 0x00FF;
		databuf[2] = CM32180_ALS_CONF_CONFIG_ON_INT >> 8;
	}
	res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}

	if (0 == obj->hw->polling_mode_als) {
		databuf[0] = CM32180_REG_ALS_THDH;
		databuf[1] = atomic_read(&obj->als_thd_val_high) & 0xFF;
		databuf[2] = atomic_read(&obj->als_thd_val_high) >> 8;
		res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			APS_ERR("i2c_master_send function err\n");
			goto EXIT_ERR;
		}
		databuf[0] = CM32180_REG_ALS_THDL;
		databuf[1] = atomic_read(&obj->als_thd_val_low) & 0xFF;
		databuf[2] = atomic_read(&obj->als_thd_val_low) >> 8;	/* threshold value need to confirm */
		res = CM32180_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			APS_ERR("i2c_master_send function err\n");
			goto EXIT_ERR;
		}
	}

	res = cm32180_setup_eint(client);
	if (res != 0) {
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
#ifndef CUSTOM_KERNEL_SENSORHUB
	disable_irq(cm32180_obj->irq);
	/* res = cm32180_disable_eint(client); */
	if (res != 0) {
		APS_ERR("disable eint fail: %d\n", res);
		return res;
	}
#endif				/* #ifndef CUSTOM_KERNEL_SENSORHUB */

	return CM32180_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/*--------------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int als_enable_nodata(int en)
{
	int res = 0;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return -1;
	}
	APS_LOG("cm32180_obj als enable value = %d\n", en);

	if (en) {
		res = cm32180_enable_als(cm32180_obj->client, 1);
		if (res) {
			APS_ERR("enable als fail: %d\n", res);
			return -1;
		}
		set_bit(CMC_BIT_ALS, &cm32180_obj->enable);
	} else {
		res = cm32180_enable_als(cm32180_obj->client, 0);
		if (res) {
			APS_ERR("disable als fail: %d\n", res);
			return -1;
		}
		clear_bit(CMC_BIT_ALS, &cm32180_obj->enable);
	}

	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}

/*--------------------------------------------------------------------------------*/
static int als_get_data(int *value, int *status)
{
	int err = 0;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return -1;
	}
	err = cm32180_read_als(cm32180_obj->client, &cm32180_obj->als);
	if (err)
		err = -1;
	else {
		*value = cm32180_get_als_value(cm32180_obj, cm32180_obj->als);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

/*--------------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/*--------------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int ps_enable_nodata(int en)
{
	int res = 0;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return -1;
	}
	APS_LOG("cm32180_obj als enable value = %d\n", en);

	if (en) {
		res = cm32180_enable_ps(cm32180_obj->client, 1);
		if (res) {
			APS_ERR("enable ps fail: %d\n", res);
			return -1;
		}
		set_bit(CMC_BIT_PS, &cm32180_obj->enable);
	} else {
		res = cm32180_enable_ps(cm32180_obj->client, 0);
		if (res) {
			APS_ERR("disable ps fail: %d\n", res);
			return -1;
		}
		clear_bit(CMC_BIT_PS, &cm32180_obj->enable);
	}

	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	return 0;

}

/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}

/*--------------------------------------------------------------------------------*/
static int ps_get_data(int *value, int *status)
{
	int err = 0;

	if (!cm32180_obj) {
		APS_ERR("cm32180_obj is null!!\n");
		return -1;
	}
	err = cm32180_read_ps(cm32180_obj->client, &cm32180_obj->ps);
	if (err)
		err = -1;
	else {
		*value = cm32180_get_ps_value(cm32180_obj, cm32180_obj->ps);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

/*-----------------------------------i2c operations----------------------------------*/
static int cm32180_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cm32180_priv *obj;
	struct als_control_path als_ctl = { 0 };
	struct als_data_path als_data = { 0 };
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = { 0 };
	int err = 0;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(*obj));
	cm32180_obj = obj;

	obj->hw = hw;		/* get custom file data struct */

	INIT_WORK(&obj->eint_work, cm32180_eint_work);
#ifdef CUSTOM_KERNEL_SENSORHUB
	INIT_WORK(&obj->init_done_work, cm32180_init_done_work);
#endif
	obj->client = client;
	i2c_set_clientdata(client, obj);

	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val, 0xC1);
	atomic_set(&obj->ps_thd_val_high, obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low, obj->hw->ps_threshold_low);
	atomic_set(&obj->als_thd_val_high, obj->hw->als_threshold_high);
	atomic_set(&obj->als_thd_val_low, obj->hw->als_threshold_low);
	atomic_set(&obj->init_done, 0);
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");

	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->als_level_num = sizeof(obj->hw->als_level) / sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value) / sizeof(obj->hw->als_value[0]);
	/*-----------value need to be confirmed-----------*/

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	cm32180_i2c_client = client;
	err = cm32180_init_client(client);
	if (err)
		goto exit_init_failed;

	APS_LOG("cm32180_init_client() OK!\n");
	err = misc_register(&cm32180_device);
	if (err) {
		APS_ERR("cm32180_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	APS_LOG("cm32180_device misc_register OK!\n");

	/*-----------cm32180 attribute file for debug-----------*/
	err = cm32180_create_attr(&cm32180_init_info.platform_diver_addr->driver);
	if (err) {
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*-----------cm32180 attribute file for debug-----------*/

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = true;
#else
	als_ctl.is_support_batch = false;
#endif

	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}


	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
#else
	ps_ctl.is_support_batch = false;
#endif

	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
#if 0				/* defined(CONFIG_HAS_EARLYSUSPEND) */
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	    obj->early_drv.suspend = cm32180_early_suspend,
	    obj->early_drv.resume = cm32180_late_resume, register_early_suspend(&obj->early_drv);
#endif

	err = batch_register_support_info(ID_LIGHT, als_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register light batch support err = %d\n", err);

	err = batch_register_support_info(ID_PROXIMITY, ps_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register proximity batch support err = %d\n", err);

	cm32180_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
	misc_deregister(&cm32180_device);
exit_init_failed:
	kfree(obj);
exit:
	cm32180_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	cm32180_init_flag = -1;
	return err;
}

static int cm32180_i2c_remove(struct i2c_client *client)
{
	int err;
	/*--------------cm32180 attribute file for debug-------------*/
	err = cm32180_delete_attr(&cm32180_init_info.platform_diver_addr->driver);
	if (err)
		APS_ERR("cm32180_delete_attr fail: %d\n", err);
	/*-----------------------------------*/
	err = misc_deregister(&cm32180_device);
	if (err)
		APS_ERR("misc_deregister fail: %d\n", err);

	cm32180_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;

}

/*
static int cm32180_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, CM32180_DEV_NAME);
	return 0;

}
*/
static int cm32180_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}

static int cm32180_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/
static int cm32180_local_init(void)
{
	/* printk("fwq loccal init+++\n"); */

	cm32180_power(hw, 1);
	if (i2c_add_driver(&cm32180_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}
	if (-1 == cm32180_init_flag)
		return -1;
	/* printk("fwq loccal init---\n"); */
	return 0;
}

/*----------------------------------------------------------------------------*/
static int cm32180_remove(void)
{
	/* APS_FUN(); */
	cm32180_power(hw, 0);	/* ***************** */
	i2c_del_driver(&cm32180_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init cm32180_init(void)
{
	const char *name = "mediatek,cm32180";

	APS_FUN();
	hw = get_alsps_dts_func(name, hw);
	if (!hw) {
		APS_ERR("get_alsps_dts_func fail\n");
		return 0;
	}
	/* i2c_register_board_info(hw->i2c_num, &i2c_cm32180, 1); */
	alsps_driver_add(&cm32180_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit cm32180_exit(void)
{
	APS_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(cm32180_init);
module_exit(cm32180_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yucong xiong");
MODULE_DESCRIPTION("cm32180 driver");
MODULE_LICENSE("GPL");
