/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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

 /*******************************************************************************
*
* File Name: focaltech_core.c
*
*    Author: Tsai HsiangYu
*
*   Created: 2015-03-02
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
/*#include <linux/rtpm_prio.h>*/
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/dma-mapping.h>
#include "tpd_custom_ft5x26.h"

#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include "mt_boot_common.h"
/* PROXIMITY */
#ifdef CONFIG_TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

/*******************************************************************************
* 2.Private constant and macro definitions using #define
*******************************************************************************/
/*register define*/
#define FTS_RESET_PIN										GPIO_CTP_RST_PIN
#define TPD_OK												0
#define DEVICE_MODE										0x00
#define GEST_ID												0x01
#define TD_STATUS											0x02
#define TOUCH1_XH											0x03
#define TOUCH1_XL											0x04
#define TOUCH1_YH											0x05
#define TOUCH1_YL											0x06
#define TOUCH2_XH											0x09
#define TOUCH2_XL											0x0A
#define TOUCH2_YH											0x0B
#define TOUCH2_YL											0x0C
#define TOUCH3_XH											0x0F
#define TOUCH3_XL											0x10
#define TOUCH3_YH											0x11
#define TOUCH3_YL											0x12
#define TPD_MAX_RESET_COUNT								3

unsigned int fts_tp_irq = 0;
/*for tp esd check*/

#if FT_ESD_PROTECT
#define TPD_ESD_CHECK_CIRCLE							200
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue;
static int count_irq;
static u8 run_check_91_register;
static unsigned long esd_check_circle = TPD_ESD_CHECK_CIRCLE;
static int gtp_esd_check_func(struct work_struct *);
#endif
#ifdef __MSG_DMA_MODE__
static void msg_dma_release(void);
#endif

#if FT_ESD_PROTECT
int apk_debug_flag = 0;
#endif

#ifdef CONFIG_TPD_PROXIMITY
#define APS_ERR(fmt, arg...)				pr_err("<<proximity>> "fmt"\n", ##arg)
#define TPD_PROXIMITY_DEBUG(fmt, arg...)	pr_debug("<<proximity>> "fmt"\n", ##arg)
#define TPD_PROXIMITY_DMESG(fmt, arg...)	pr_debug("<<proximity>> "fmt"\n", ##arg)
static u8 tpd_proximity_flag;
static u8 tpd_proximity_flag_one;	/* add for tpd_proximity by wangdongfang */
static u8 tpd_proximity_detect = 1;	/* 0-->close ; 1--> far away */
#endif
/*dma declare, allocate and release*/
#define __MSG_DMA_MODE__
#ifdef __MSG_DMA_MODE__
u8 *g_dma_buff_va_fts = NULL;
u8 *g_dma_buff_pa_fts = NULL;
#endif

#ifdef __MSG_DMA_MODE__
static void msg_dma_release(void)
{
	if (g_dma_buff_va_fts) {
		dma_free_coherent(NULL, 128, g_dma_buff_va_fts, (dma_addr_t) g_dma_buff_pa_fts);
		g_dma_buff_va_fts = NULL;
		g_dma_buff_pa_fts = NULL;
		FTS_DBG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	}
}
#endif
#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT] = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX_ROTATION;
#endif
/*******************************************************************************
* 3.Private enumerations, structures and unions using typedef
*******************************************************************************/

/* touch info */
#if A_TYPE == 0
struct touch_info {
	int y[10];
	int x[10];
	int p[10];
	int id[10];
	int pressure[10];
	int count;
};
#else
struct touch_info {
	int y[10];
	int x[10];
	int p[10];
	int id[10];
	int count;
};
#endif

/*******************************************************************************
* 4.Static variables
*******************************************************************************/
struct i2c_client *fts_i2c_client = NULL;
struct input_dev *fts_input_dev = NULL;
struct task_struct *fts_thread = NULL;
int up_flag = 0;
int up_count = 0;
static int tpd_flag;
static int tpd_halt;
static int point_num;
static int total_point;
/*******************************************************************************
* 5.Global variable or extern global variabls/functions
*******************************************************************************/



/*******************************************************************************
* 6.Static function prototypes
*******************************************************************************/
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);
static irqreturn_t tpd_eint_interrupt_handler(void);
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static const struct i2c_device_id ft5x26_tpd_id[] = { {"ft5x26", 0}, {} };

static const struct of_device_id ft5x26_dt_match[] = {
	{.compatible = "mediatek,ft5x26_touch"},
	{},
};

MODULE_DEVICE_TABLE(of, ft5x0x_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		   .name = "ft5x26",
		   .of_match_table = ft5x26_dt_match,
		   /* .owner     = THIS_MODULE, */
		   },
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft5x26_tpd_id,
	.detect = tpd_detect,
};



/*
* open/release/(I/O) control tpd device
*
*/
/* #define VELOCITY_CUSTOM_fts */
#ifdef VELOCITY_CUSTOM_fts
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

/* for magnify velocity */
#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X							10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y							10
#endif

#define TOUCH_IOC_MAGIC									'A'
#define TPD_GET_VELOCITY_CUSTOM_X						_IO(TOUCH_IOC_MAGIC, 0)
#define TPD_GET_VELOCITY_CUSTOM_Y						_IO(TOUCH_IOC_MAGIC, 1)

int g_v_magnify_x = TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y = TPD_VELOCITY_CUSTOM_Y;


/************************************************************************
* Name: tpd_misc_open
* Brief: open node
* Input: node, file point
* Output: no
* Return: fail <0
***********************************************************************/
static int tpd_misc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

/************************************************************************
* Name: tpd_misc_release
* Brief: release node
* Input: node, file point
* Output: no
* Return: 0
***********************************************************************/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

/************************************************************************
* Name: tpd_unlocked_ioctl
* Brief: I/O control for apk
* Input: file point, command
* Output: no
* Return: fail <0
***********************************************************************/

static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	void __user *data;

	long err = 0;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		FTS_ERROR("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd),
			  _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case TPD_GET_VELOCITY_CUSTOM_X:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x))) {
			err = -EFAULT;
			break;
		}
		break;

	case TPD_GET_VELOCITY_CUSTOM_Y:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y))) {
			err = -EFAULT;
			break;
		}
		break;


	default:
		FTS_ERROR("tpd: unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}


static const struct file_operations tpd_fops = {
	/* .owner = THIS_MODULE, */
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};

static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};
#endif


/************************************************************************
* Name: fts_i2c_read
* Brief: i2c read
* Input: i2c info, write buf, write len, read buf, read len
* Output: get data in the 3rd buf
* Return: fail <0
***********************************************************************/
int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf,
		 int readlen)
{
	int ret = 0;

	/* for DMA I2c transfer */

	mutex_lock(&i2c_rw_access);

	if ((NULL != client) && (writelen > 0) && (writelen <= 128)) {
		/* DMA Write */
		memcpy(g_dma_buff_va_fts, writebuf, writelen);
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		ret = i2c_master_send(client, (unsigned char *)g_dma_buff_pa_fts, writelen);

		if (ret != writelen)
			FTS_ERROR("i2c write failed\n");
		client->addr = (client->addr & I2C_MASK_FLAG) & (~I2C_DMA_FLAG);
	}
	/* DMA Read */
	if ((NULL != client) && (readlen > 0) && (readlen <= 128)) {
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;

		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa_fts, readlen);

		memcpy(readbuf, g_dma_buff_va_fts, readlen);

		client->addr = (client->addr & I2C_MASK_FLAG) & (~I2C_DMA_FLAG);
	}

	mutex_unlock(&i2c_rw_access);

	return ret;
}

/************************************************************************
* Name: fts_i2c_write
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);

	/* client->addr = client->addr & I2C_MASK_FLAG; */

	/* ret = i2c_master_send(client, writebuf, writelen); */
	if ((NULL != client) && (writelen > 0) && (writelen <= 128)) {
		memcpy(g_dma_buff_va_fts, writebuf, writelen);

		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		ret = i2c_master_send(client, (unsigned char *)g_dma_buff_pa_fts, writelen);

		if (ret != writelen)
			FTS_ERROR("i2c write failed\n");
		client->addr = (client->addr & I2C_MASK_FLAG) & (~I2C_DMA_FLAG);
	}
	mutex_unlock(&i2c_rw_access);

	return ret;
}

/************************************************************************
* Name: fts_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = { 0 };

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_write(client, buf, sizeof(buf));
}

/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{

	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);

}

#ifndef MT_PROTOCOL_B
/************************************************************************
* Name: tpd_down
* Brief: down info
* Input: x pos, y pos, id number
* Output: no
* Return: no
***********************************************************************/
#if A_TYPE == 0
static int tpd_history_x = 0, tpd_history_y;
static void tpd_down(int x, int y, int size, int id)
{
	if ((!size) && (!id)) {
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
		/* track id Start 0 */
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	}

	input_report_key(tpd->dev, BTN_TOUCH, 1);

	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
	TPD_DEBUG_SET_TIME;
	TPD_EM_PRINT(x, y, x, y, id, 1);
	tpd_history_x = x;
	tpd_history_y = y;
#ifdef TPD_HAVE_BUTTON
	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
		tpd_button(x, y, 1);
#endif
	TPD_DOWN_DEBUG_TRACK(x, y);
}
#else				/*(A_TYPE == 1) || (A_TYPE == 2) */
static void tpd_down(int x, int y, int p)
{

	if (x > TPD_RES_X) {
		FTS_DBG("warning: IC have sampled wrong value.\n");
		return;
	}
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0x3f);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	/* FTS_DBG("tpd:D[%4d %4d %4d] ", x, y, p); */
	/* track id Start 0 */
	/* input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); */
	input_mt_sync(tpd->dev);
	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
		tpd_button(x, y, 1);
	/* virtual key debounce to avoid android ANR issue */
	if (y > TPD_RES_Y)
		FTS_DBG("D virtual key\n");
	TPD_EM_PRINT(x, y, x, y, p - 1, 1);

}
#endif

 /************************************************************************
* Name: tpd_up
* Brief: up info
* Input: x pos, y pos, count
* Output: no
* Return: no
***********************************************************************/
#if A_TYPE == 0
static void tpd_up(int x, int y, int id)
{
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	/* FTS_DBG("U[%4d %4d %4d] ", x, y, 0); */
	input_mt_sync(tpd->dev);
	TPD_DEBUG_SET_TIME;
	TPD_EM_PRINT(tpd_history_x, tpd_history_y, tpd_history_x, tpd_history_y, id, 0);
	tpd_history_x = 0;
	tpd_history_y = 0;
#ifdef TPD_HAVE_BUTTON
	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
		tpd_button(x, y, 0);
#endif
}
#else				/* A_TYPE == 1 ||  A_TYPE == 2 */
static void tpd_up(int x, int y, int *count)
{

	input_report_key(tpd->dev, BTN_TOUCH, 0);
	/* FTS_DBG("U[%4d %4d %4d] ", x, y, 0); */
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, 0, 0);

	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
		tpd_button(x, y, 0);
}
#endif
 /************************************************************************
* Name: tpd_touchinfo
* Brief: touch info
* Input: touch info point, no use
* Output: no
* Return: success nonzero
***********************************************************************/
#if A_TYPE == 0
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo,
			 struct touch_info *ptest)
{
	int i = 0;
	char data[128] = { 0 };
	u16 high_byte, low_byte, reg;
	u8 pointid = FTS_MAX_ID;

	if (tpd_halt) {
		FTS_DBG("tpd_touchinfo return ..\n");
		return false;
	}

	/* mutex_lock(&i2c_access); */
	reg = 0x00;
	fts_i2c_read(fts_i2c_client, (char *)(&reg), 1, data, 64);
	/* mutex_unlock(&i2c_access); */

	point_num = data[2] & 0x0f;
#if REPORT_TOUCH_DEBUG
	FTS_DBG(" zax point_num=%d\n", point_num);
#endif
	memset(cinfo, 0, sizeof(struct touch_info));
	total_point = 0;
	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++) {
		pointid = (data[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;

		total_point++;
		cinfo->p[i] = data[3 + 6 * i] >> 6;	/* event flag */
		cinfo->id[i] = data[3 + 6 * i + 2] >> 4;	/* touch id */

		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3 + 6 * i + 1];
		cinfo->x[i] = high_byte | low_byte;
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3 + 6 * i + 3];
		cinfo->y[i] = high_byte | low_byte;

		cinfo->pressure[i] = (data[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]);	/* cannot constant value */

#if REPORT_TOUCH_DEBUG
		FTS_DBG(" zax tpd i=%d,  x= (0x%02x), y= (0x%02x), evt= %d,id=%d,pre=%d\n", i,
			cinfo->x[i], cinfo->y[i], cinfo->p[i], cinfo->id[i], cinfo->pressure[i]);
#endif
	}

	return true;
}
#elif A_TYPE == 1
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo,
			 struct touch_info *ptest)
{
	int i = 0;
	char data[128] = { 0 };
	u16 high_byte, low_byte, reg;
	u8 report_rate = 0;

	if (tpd_halt) {
		FTS_DBG("tpd_touchinfo return ..\n");
		return false;
	}

	/* mutex_lock(&i2c_access); */
	reg = 0x00;
	fts_i2c_read(fts_i2c_client, &reg, 1, data, 64);
	/* mutex_unlock(&i2c_access); */

	/* get the number of the touch points */
	point_num = data[2] & 0x0f;
	if (up_flag == 2) {
		up_flag = 0;
		for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++) {
			cinfo->p[i] = data[3 + 6 * i] >> 6;	/* event flag */
			cinfo->id[i] = data[3 + 6 * i + 2] >> 4;	/* touch id */
			/* get the X coordinate, 2 bytes */
			high_byte = data[3 + 6 * i];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3 + 6 * i + 1];
			cinfo->x[i] = high_byte | low_byte;
			high_byte = data[3 + 6 * i + 2];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3 + 6 * i + 3];
			cinfo->y[i] = high_byte | low_byte;

			if (point_num >= i + 1)
				continue;
			if (up_count == 0)
				continue;
			cinfo->p[i] = ptest->p[i - point_num];	/* event flag */
			cinfo->id[i] = ptest->id[i - point_num];	/* touch id */
			cinfo->x[i] = ptest->x[i - point_num];
			cinfo->y[i] = ptest->y[i - point_num];
			up_count--;
		}

		return true;
	}

	up_count = 0;
	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++) {
		cinfo->p[i] = data[3 + 6 * i] >> 6;	/* event flag */

		if (0 == cinfo->p[i]) {
			up_flag = 1;
		cinfo->id[i] = data[3 + 6 * i + 2] >> 4;	/* touch id */
		/* get the X coordinate, 2 bytes */
		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3 + 6 * i + 1];
		cinfo->x[i] = high_byte | low_byte;
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3 + 6 * i + 3];
		cinfo->y[i] = high_byte | low_byte;

		if (up_flag == 1 && 1 == cinfo->p[i]) {
			up_flag = 2;
			point_num++;
			ptest->x[up_count] = cinfo->x[i];
			ptest->y[up_count] = cinfo->y[i];
			ptest->id[up_count] = cinfo->id[i];
			ptest->p[up_count] = cinfo->p[i];
			cinfo->p[i] = 2;
			up_count++;
		}
	}
	if (up_flag == 1)
		up_flag = 0;
	return true;

}
#elif A_TYPE == 2
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[128] = { 0 };
	u16 high_byte, low_byte, reg;
	u8 report_rate = 0;

	if (tpd_halt) {
		FTS_DBG("tpd_touchinfo return ..\n");
		return false;
	}

	/* mutex_lock(&i2c_access); */
	reg = 0x00;
	fts_i2c_read(fts_i2c_client, &reg, 1, data, 64);
	/* mutex_unlock(&i2c_access); */

	/* get the number of the touch points */
	point_num = data[2] & 0x0f;

	for (i = 0; i < point_num; i++) {
		cinfo->p[i] = data[3 + 6 * i] >> 6;	/* event flag */
		cinfo->id[i] = data[3 + 6 * i + 2] >> 4;	/* touch id */
		/* get the X coordinate, 2 bytes */
		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3 + 6 * i + 1];
		cinfo->x[i] = high_byte | low_byte;
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3 + 6 * i + 3];
		cinfo->y[i] = high_byte | low_byte;
	}

	/* FTS_DBG(" tpd cinfo->x[0] = %d, cinfo->y[0] = %d,
		cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]); */
	return true;

}
#endif
#endif

#ifdef MT_PROTOCOL_B
 /************************************************************************
* Name: fts_read_Touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
static int fts_read_Touchdata(struct ts_event *data)
{
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FTS_MAX_ID;

	if (tpd_halt) {
		FTS_DBG("tpd_touchinfo return ..\n");
		return false;
	}

	mutex_lock(&i2c_access);
	ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		FTS_ERROR("%s read touchdata failed.\n", __func__);
		mutex_unlock(&i2c_access);
		return ret;
	}
#if REPORT_TOUCH_DEBUG
	for (i = 0; i < POINT_READ_BUF; i++)
		FTS_ERROR("zax buf[%d] =(0x%02x)\n", i, buf[i]);
#endif
	mutex_unlock(&i2c_access);
	memset(data, 0, sizeof(struct ts_event));
	data->touch_point = 0;
	data->touch_point_num = buf[FT_TOUCH_POINT_NUM] & 0x0F;
	/*FTS_DBG("tpd  fts_updateinfo_curr.TPD_MAX_POINTS=%d fts_updateinfo_curr.chihID=%d\n",
	   fts_updateinfo_curr.TPD_MAX_POINTS,fts_updateinfo_curr.CHIP_ID); */
	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++) {
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;

		data->touch_point++;
		data->au16_x[i] =
		    (s16)(buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i] & 0x0F) << 8 |
		    (s16)buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i];
		data->au16_y[i] =
		    (s16)(buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i] & 0x0F) << 8 |
		    (s16)buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i];
		data->au8_touch_event[i] = buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;
		data->au8_finger_id[i] = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;

		data->pressure[i] = (buf[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]);
		data->area[i] = (buf[FTS_TOUCH_MISC + FTS_TOUCH_STEP * i]) >> 4;
#if REPORT_TOUCH_DEBUG
		FTS_ERROR("zax data 1  (id= %d ,x=(0x%02x),y= (0x%02x)),point_num=%d, event=%d\n ",
			  data->au8_finger_id[i], data->au16_x[i], data->au16_y[i],
			  data->touch_point, data->au8_touch_event[i]);
#endif
		if ((data->au8_touch_event[i] == 0 || data->au8_touch_event[i] == 2)
		    && ((data->touch_point_num == 0)))
			return 1;
#if REPORT_TOUCH_DEBUG
		FTS_ERROR("zax data  2 (id= %d ,x=(0x%02x),y= (0x%02x)),point_num=%d, event=%d\n ",
			  data->au8_finger_id[i], data->au16_x[i], data->au16_y[i],
			  data->touch_point, data->au8_touch_event[i]);
#endif
	}

	return 0;
}

 /************************************************************************
* Name: fts_report_value
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int fts_report_value(struct ts_event *data)
{
	int i = 0, j = 0;
	int up_point = 0;
	int touchs = 0;

	for (i = 0; i < data->touch_point; i++) {
		input_mt_slot(tpd->dev, data->au8_finger_id[i]);

		if (data->au8_touch_event[i] == 0 || data->au8_touch_event[i] == 2) {
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, true);
			input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0x3f);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0x05);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X, data->au16_x[i]);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y, data->au16_y[i]);
			touchs |= BIT(data->au8_finger_id[i]);
			data->touchs |= BIT(data->au8_finger_id[i]);
#if REPORT_TOUCH_DEBUG
			FTS_ERROR("zax down (id=%d ,x=%d, y=%d, pres=%d, area=%d)\n",
				  data->au8_finger_id[i], data->au16_x[i], data->au16_y[i],
				  data->pressure[i], data->area[i]);
#endif
		} else {
#if REPORT_TOUCH_DEBUG
			FTS_ERROR("zax normal_up 1 (id=%d ,x=%d, y=%d, pres=%d, area=%d)\n",
				  data->au8_finger_id[i], data->au16_x[i], data->au16_y[i],
				  data->pressure[i], data->area[i]);
#endif
			up_point++;
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(data->au8_finger_id[i]);
		}
	}
	for (i = 0; i < 10; i++) {
		if (BIT(i) & (data->touchs ^ touchs)) {
#if REPORT_TOUCH_DEBUG
			FTS_ERROR("zax normal_up 2  id=%d\n", i);
#endif
			data->touchs &= ~BIT(i);
			input_mt_slot(tpd->dev, i);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
		}
	}
#if REPORT_TOUCH_DEBUG
	FTS_ERROR("zax 1 touchs=%d, data-touchs=%d, touch_point=%d, up_point=%d\n ",
		  touchs, data->touchs, data->touch_point, up_point);
#endif
	data->touchs = touchs;

	if (data->touch_point_num == 0) {
		for (j = 0; j < 10; j++) {
			input_mt_slot(tpd->dev, j);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
		}

		data->touchs = 0;
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_sync(tpd->dev);

#if REPORT_TOUCH_DEBUG
		FTS_ERROR
		    ("zax  normal_up 3 end 2 touchs=%d, data-touchs=%d, touch_point=%d, up_point=%d\n ",
		     touchs, data->touchs, data->touch_point, up_point);
#endif
		return 0;
	}
#if REPORT_TOUCH_DEBUG
	FTS_ERROR("zax 2 touchs=%d, data-touchs=%d, touch_point=%d, up_point=%d\n ", touchs,
		  data->touchs, data->touch_point, up_point);
#endif
	if (data->touch_point == up_point)
		input_report_key(tpd->dev, BTN_TOUCH, 0);
	else
		input_report_key(tpd->dev, BTN_TOUCH, 1);

	input_sync(tpd->dev);
#if REPORT_TOUCH_DEBUG
	FTS_DBG("zax end 1\n ");
#endif
	return 0;
}
#endif

#ifdef CONFIG_TPD_PROXIMITY
 /************************************************************************
* Name: tpd_read_ps
* Brief: read proximity value
* Input: no
* Output: no
* Return: 0
***********************************************************************/
int tpd_read_ps(void)
{
	tpd_proximity_detect;
	return 0;
}

 /************************************************************************
* Name: tpd_get_ps_value
* Brief: get proximity value
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

 /************************************************************************
* Name: tpd_enable_ps
* Brief: enable proximity
* Input: enable or not
* Output: no
* Return: 0
***********************************************************************/
static int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;

	/* i2c_smbus_read_i2c_block_data(fts_i2c_client, 0xB0, 1, &state); */

	ret = fts_read_reg(fts_i2c_client, 0xB0, &state);
	if (ret < 0) {
		FTS_ERROR("read value fail");
		/* return ret; */
	}

	FTS_DBG("[proxi_fts]read: 999 0xb0's value is 0x%02X\n", state);

	if (enable) {
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is on\n");
	} else {
		state &= 0x00;
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is off\n");
	}

	/* ret = i2c_smbus_write_i2c_block_data(fts_i2c_client, 0xB0, 1, &state); */
	ret = fts_write_reg(fts_i2c_client, 0xB0, state);
	if (ret < 0) {
		FTS_ERROR("write value fail");
		/* return ret; */
	}
	TPD_PROXIMITY_DEBUG("[proxi_fts]write: 0xB0's value is 0x%02X\n", state);
	return 0;
}

 /************************************************************************
* Name: tpd_ps_operate
* Brief: operate function for proximity
* Input: point, which operation, buf_in , buf_in len, buf_out , buf_out len, no use
* Output: buf_out
* Return: fail <0
***********************************************************************/
int tpd_ps_operate(void *self, uint32_t command, void *buff_in, int size_in,
		   void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;

	FTS_DBG("[proxi_fts]command = 0x%02X\n", command);

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			APS_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		}
		/* Do nothing */
		break;
	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			APS_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value) {
				if ((tpd_enable_ps(1) != 0)) {
					APS_ERR("enable ps fail: %d\n", err);
					return -1;
				}
			} else {
				if ((tpd_enable_ps(0) != 0)) {
					APS_ERR("disable ps fail: %d\n", err);
					return -1;
				}
			}
		}
		break;
	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data))) {
			APS_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			sensor_data = (hwm_sensor_data *) buff_out;
			err = tpd_read_ps();
			if (err)
				err = -1;
			else {
				sensor_data->values[0] = tpd_get_ps_value();
				TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n",
						    sensor_data->values[0]);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
		}
		break;
	default:
		APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}
	return err;
}
#endif
#if FT_ESD_PROTECT
void esd_switch(s32 on)
{
	if (1 == on)		/* switch on esd */
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
	else			/* switch off esd */
		cancel_delayed_work(&gtp_esd_check_work);
}

/************************************************************************
* Name: force_reset_guitar
* Brief: reset
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void force_reset_guitar(void)
{
	s32 i;
	s32 ret;

	disable_irq(fts_tp_irq);
	tpd_gpio_output(GTP_RST_PORT, 0);
	mdelay(10);
	FTS_DBG("force_reset_guitar\n");

	err = regulator_disable(tpd->reg);
	if (err)
		FTS_ERROR("regulator_enable() failed!\n");

	msleep(200);

	err = regulator_enable(tpd->reg);
	if (err)
		FTS_ERROR("regulator_enable() failed!\n");

	mdelay(10);
	FTS_DBG(" fts ic reset\n");
	tpd_gpio_output(GTP_RST_PORT, 1);
	tpd_gpio_as_int(GTP_INT_PORT);

	msleep(300);

#ifdef CONFIG_TPD_PROXIMITY
	if (FT_PROXIMITY_ENABLE == tpd_proximity_flag)
		tpd_enable_ps(FT_PROXIMITY_ENABLE);
#endif
	enable_irq(fts_tp_irq);
}


#define A3_REG_VALUE								0x54
#define RESET_91_REGVALUE_SAMECOUNT				5
static u8 g_old_91_Reg_Value = 0x00;
static u8 g_first_read_91 = 0x01;
static u8 g_91value_same_count;
/************************************************************************
* Name: gtp_esd_check_func
* Brief: esd check function
* Input: struct work_struct
* Output: no
* Return: 0
***********************************************************************/
static int gtp_esd_check_func(struct work_struct *work)
{
	int i;
	int ret = -1;
	u8 data, data_old;
	u8 flag_error = 0;
	int reset_flag = 0;
	u8 check_91_reg_flag = 0;

	if (tpd_halt)
		return 0;
	if (is_update)
		return 0;
	if (apk_debug_flag)
		return 0;

	run_check_91_register = 0;
	for (i = 0; i < 3; i++) {
		ret = fts_read_reg(fts_i2c_client, 0xA3, &data);
		if (ret < 0)
			FTS_ERROR("read value fail");
		if (ret == 1 && A3_REG_VALUE == data)
			break;
	}

	if (i >= 3) {
		force_reset_guitar();
		FTS_ERROR("tpd reset fail,ret: %d, A3_Reg_Value = 0x%02x\n ", ret, data);
		reset_flag = 1;
		goto FOCAL_RESET_A3_REGISTER;
	}
	/* esd check for count */
	ret = fts_read_reg(fts_i2c_client, 0x8F, &data);
	if (ret < 0)
		FTS_ERROR("read value fail");
	FTS_DBG("0x8F:%d, count_irq is %d\n", data, count_irq);

	flag_error = 0;
	if ((count_irq - data) > 10) {
		if ((data + 200) > (count_irq + 10))
			flag_error = 1;
	}

	if ((data - count_irq) > 10)
		flag_error = 1;

	if (1 == flag_error) {
		FTS_ERROR("tpd reset.1 flag_error, data=%d count_irq\n ", data,
			  count_irq);
		force_reset_guitar();
		reset_flag = 1;
		goto FOCAL_RESET_INT;
	}

	run_check_91_register = 1;
	ret = fts_read_reg(fts_i2c_client, 0x91, &data);
	if (ret < 0)
		FTS_ERROR("read value fail");
	FTS_DBG("91 register value = 0x%02x, old value = 0x%02x\n", data,
		g_old_91_Reg_Value);
	if (0x01 == g_first_read_91) {
		g_old_91_Reg_Value = data;
		g_first_read_91 = 0x00;
	} else {
		if (g_old_91_Reg_Value == data) {
			g_91value_same_count++;
			FTS_DBG("g_91value_same_count=%d\n",
				g_91value_same_count);
			if (RESET_91_REGVALUE_SAMECOUNT == g_91value_same_count) {
				force_reset_guitar();
				FTS_DBG("reset. g_91value_same_count = 5\n");
				g_91value_same_count = 0;
				reset_flag = 1;
			}
			/* run_check_91_register = 1; */
			esd_check_circle = TPD_ESD_CHECK_CIRCLE / 2;
			g_old_91_Reg_Value = data;
		} else {
			g_old_91_Reg_Value = data;
			g_91value_same_count = 0;
			/* run_check_91_register = 0; */
			esd_check_circle = TPD_ESD_CHECK_CIRCLE;
		}
	}
FOCAL_RESET_INT:
FOCAL_RESET_A3_REGISTER:
	count_irq = 0;
	data = 0;
	ret = fts_write_reg(fts_i2c_client, 0x8F, data);
	if (ret < 0)
		FTS_ERROR("write value fail");
	if (0 == run_check_91_register)
		g_91value_same_count = 0;
#ifdef CONFIG_TPD_PROXIMITY
	if ((1 == reset_flag) && (FT_PROXIMITY_ENABLE == tpd_proximity_flag)) {
		if ((tpd_enable_ps(FT_PROXIMITY_ENABLE) != 0)) {
			APS_ERR("FTS enable ps fail\n");
			return -1;
		}
	}
#endif
	/* end esd check for count */

	if (!tpd_halt)
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);

	return 0;
}
#endif

/*Coordination mapping*/
static void tpd_calibrate_driver(int *x, int *y)
{
	int tx;

	FTS_DBG("Call tpd_calibrate of this driver ..\n");
	tx = ((tpd_def_calmat[0] * (*x)) + (tpd_def_calmat[1] * (*y))
		+ (tpd_def_calmat[2])) >> 12;
	*y = ((tpd_def_calmat[3] * (*x)) + (tpd_def_calmat[4] * (*y))
		+ (tpd_def_calmat[5])) >> 12;
	*x = tx;
}

 /************************************************************************
* Name: touch_event_handler
* Brief: interrupt event from TP, and read/report data to Android system
* Input: no use
* Output: no
* Return: 0
***********************************************************************/
static int touch_event_handler(void *unused)
{
#ifdef MT_PROTOCOL_B
	struct ts_event pevent;
#endif

#ifndef MT_PROTOCOL_B
#if A_TYPE == 0
	struct touch_info cinfo, pinfo, ptest;
	int i = 0;
	static u8 pre_touch;
#endif
#endif
#if defined(CONFIG_TPD_PROXIMITY) || defined(CONFIG_FTS_GESTRUE_EN)
	int ret = 0;
#endif
	struct sched_param param = {.sched_priority = 4 };

#ifdef CONFIG_TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
	u8 proximity_status;
#endif
#if defined(CONFIG_FTS_GESTRUE_EN)
	u8 state;
#endif
	int input_x = 0;
	int input_y = 0;


	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);
		/* FTS_DBG("tpd touch_event_handler\n"); */
#if defined(CONFIG_FTS_GESTRUE_EN)
		/* i2c_smbus_read_i2c_block_data(fts_i2c_client, 0xd0, 1, &state); */
		ret = fts_read_reg(fts_i2c_client, 0xd0, &state);
		if (ret < 0)
			FTS_ERROR("read value fail");
		/* FTS_DBG("tpd fts_read_Gestruedata state=%d\n",state); */
		if (state == 1) {
			fts_read_Gestruedata();
			continue;
		}
#endif

#ifdef CONFIG_TPD_PROXIMITY
		if (tpd_proximity_flag == 1) {
			ret = fts_read_reg(fts_i2c_client, 0xB0, &state);
			if (ret < 0)
				FTS_ERROR("read value fail");
			TPD_PROXIMITY_DEBUG("proxi_fts 0xB0 state value is 1131 0x%02X\n", state);
			if (!(state & 0x01))
				tpd_enable_ps(1);
			/* i2c_smbus_read_i2c_block_data(fts_i2c_client, 0x01, 1, &proximity_status); */
			ret = fts_read_reg(fts_i2c_client, 0x01, &proximity_status);
			if (ret < 0)
				FTS_ERROR("read value fail");
			TPD_PROXIMITY_DEBUG("proxi_fts 0x01 value is 1139 0x%02X\n",
					    proximity_status);
			if (proximity_status == 0xC0)
				tpd_proximity_detect = 0;
			else if (proximity_status == 0xE0)
				tpd_proximity_detect = 1;

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n",
					    tpd_proximity_detect);
			err = tpd_read_ps();
			if (err)
				TPD_PROXIMITY_DMESG("proxi_fts read ps data 1156: %d\n", err);
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		}
#endif
#if FT_ESD_PROTECT
		esd_switch(0);
		apk_debug_flag = 1;
#endif
#ifdef MT_PROTOCOL_B
		ret = fts_read_Touchdata(&pevent);
		if (ret == 0)
			fts_report_value(&pevent);
#else
#if A_TYPE == 0
		if (tpd_touchinfo(&cinfo, &pinfo, &ptest)) {
			/* FTS_DBG("zax 0 tpd point_num = %d\n",point_num); */
			/* TPD_DEBUG_SET_TIME; */
			if (point_num > 0) {
				for (i = 0; i < total_point; i++) {
					if ((0 == cinfo.p[i]) || (2 == cinfo.p[i])) {
						input_x = cinfo.x[i];
						input_y = cinfo.y[i];
						FTS_DBG("Original touch point : [X:%04d, Y:%04d]",
							input_x, input_y);
						tpd_calibrate_driver(&input_x, &input_y);
						FTS_DBG
						    ("Touch point after calibration: [X:%04d, Y:%04d]",
						     input_x, input_y);

						tpd_down(input_x, input_y, cinfo.pressure[i],
							 cinfo.id[i]);
					}
				}
				input_sync(tpd->dev);
			} else if (pre_touch) {
				tpd_up(0, 0, 0);
				FTS_DBG("release --->\n");
				input_sync(tpd->dev);
			} else
				FTS_DBG("Additional Eint!");
			pre_touch = point_num;
		}
#elif A_TYPE == 1
		if (tpd_touchinfo(&cinfo, &pinfo, &ptest)) {
			/* FTS_DBG("zax 1 tpd point_num = %d\n",point_num); */
			/*TPD_DEBUG_SET_TIME; */
			if (point_num > 0) {
				for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++) {
					if ((0 == cinfo.p[i]) || (2 == cinfo.p[i]))
						tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
				}
				input_sync(tpd->dev);
			} else {
				tpd_up(cinfo.x[0], cinfo.y[0], &cinfo.id[0]);
				/* FTS_DBG("release --->\n"); */
				input_sync(tpd->dev);
			}
		}
#elif A_TYPE == 2
		if (tpd_touchinfo(&cinfo, &pinfo)) {
			/* FTS_DBG("zax 2 tpd point_num = %d\n",point_num); */
			TPD_DEBUG_SET_TIME;
			if (point_num > 0) {
				for (i = 0; i < point_num; i++)	/* only support 3 point */
					tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
				input_sync(tpd->dev);
			} else {
				tpd_up(cinfo.x[0], cinfo.y[0], &cinfo.id[0]);
				input_sync(tpd->dev);
			}
		}
#endif
#endif
#if FT_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
#endif
	} while (!kthread_should_stop());
	return 0;
}

  /************************************************************************
* Name: fts_reset_tp
* Brief: reset TP
* Input: pull low or high
* Output: no
* Return: 0
***********************************************************************/
void fts_reset_tp(int HighOrLow)
{

	if (HighOrLow)
		tpd_gpio_output(GTP_RST_PORT, 1);
	else
		tpd_gpio_output(GTP_RST_PORT, 0);
}

   /************************************************************************
* Name: tpd_detect
* Brief: copy device name
* Input: i2c info, board info
* Output: no
* Return: 0
***********************************************************************/
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);
	return 0;
}

/************************************************************************
* Name: tpd_eint_interrupt_handler
* Brief: deal with the interrupt event
* Input: no
* Output: no
* Return: no
***********************************************************************/
static irqreturn_t tpd_eint_interrupt_handler(void)
{
	/* FTS_DBG("TPD interrupt has been triggered\n"); */
	tpd_flag = 1;
#if FT_ESD_PROTECT
	count_irq++;
#endif
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

/************************************************************************
* Name: fts_init_gpio_hw
* Brief: initial gpio
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static int fts_init_gpio_hw(void)
{

	int ret = 0;

	tpd_gpio_output(GTP_RST_PORT, 1);
	return ret;
}

/************************************************************************
* Name: tpd_probe
* Brief: driver entrance function for initial/power on/create channel
* Input: i2c info, device id
* Output: no
* Return: 0
***********************************************************************/
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = TPD_OK;
	char data;
	u8 err = 0;
	/* int err=0; */
	int reset_count = 0;
#ifdef CONFIG_TPD_PROXIMITY
	int err;
	struct hwmsen_object obj_ps;
#endif
	struct device_node *node;
	int ret;

	FTS_INFO("tpd_probe\n");
reset_proc:
	fts_i2c_client = client;
	fts_input_dev = tpd->dev;
#ifdef CONFIG_TPD_CLOSE_POWER_IN_SLEEP

#else
	tpd_gpio_output(GTP_RST_PORT, 0);
	mdelay(10);
#endif

/* power on, need confirm with SA */
	err = regulator_enable(tpd->reg);
	if (err) {
		FTS_ERROR("regulator_enable() failed!\n");
		return -1;
	}

#ifdef CONFIG_TPD_CLOSE_POWER_IN_SLEEP
#else
	mdelay(10);
	FTS_DBG(" fts reset\n");
	tpd_gpio_output(GTP_RST_PORT, 1);
#endif
	tpd_gpio_as_int(GTP_INT_PORT);
	msleep(200);

	err = i2c_smbus_read_i2c_block_data(fts_i2c_client, 0x00, 1, &data);
	/* if auto upgrade fail, it will not read right value next upgrade. */
	/* reg0 data running state is 0; other state is not 0 */
	if (err < 0 || data != 0) {
		FTS_ERROR("I2C transfer error, err: %d, data: %d\n", err, data);

		if (++reset_count < TPD_MAX_RESET_COUNT)
			goto reset_proc;
		err = regulator_disable(tpd->reg);
		if (err)
			FTS_ERROR("regulator_disable() failed!\n");
		return -1;
	}
	/*msg_dma_alloct(); */
#ifdef __MSG_DMA_MODE__
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	client->dev.coherent_dma_mask = DMA_BIT_MASK(64);
#else
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
#endif
	client->dev.dma_mask = &client->dev.coherent_dma_mask;
	g_dma_buff_va_fts = (u8 *) dma_alloc_coherent(&client->dev,
		128, (dma_addr_t *) (&g_dma_buff_pa_fts), GFP_KERNEL | GFP_DMA);	/* DMA size 4096 for customer */
	if (!g_dma_buff_va_fts) {
		FTS_ERROR("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
		return -1;
	}
#endif
	fts_init_gpio_hw();

	/*tpd_gpio_as_int(GTP_INT_PORT);
	   msleep(50); */

	node = of_find_matching_node(NULL, touch_of_match);
	if (node) {
		/*fts_tp_irq = gpio_to_irq(tpd_int_gpio_number); */
		fts_tp_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(fts_tp_irq, (irq_handler_t) tpd_eint_interrupt_handler,
				  IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
		if (ret > 0) {
			TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			goto err_irq_fail;
		}
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
		goto err_irq_fail;
	}
	disable_irq(fts_tp_irq);

#ifdef VELOCITY_CUSTOM_fts
	err = misc_register(&tpd_misc_device);
	if (err)
		FTS_ERROR("mtk_tpd: tpd_misc_device register failed\n");
#endif

	fts_thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(fts_thread)) {
		retval = PTR_ERR(fts_thread);
		FTS_ERROR(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		goto err_thread_fail;
	}
#ifdef SYSFS_DEBUG
	fts_create_sysfs(fts_i2c_client);
#endif
	HidI2c_To_StdI2c(fts_i2c_client);
	fts_get_upgrade_array();
#ifdef FTS_CTL_IIC
	if (fts_rw_iic_drv_init(fts_i2c_client) < 0)
		FTS_ERROR("%s:create fts control iic driver failed\n", __func__);
#endif

#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(fts_i2c_client);
#endif

#ifdef CONFIG_TPD_AUTO_UPGRADE
	FTS_DBG("*********Enter CTP Auto Upgrade*************\n");
	is_update = true;
	fts_ctpm_auto_upgrade(fts_i2c_client);
	is_update = false;
#endif

#ifdef CONFIG_TPD_PROXIMITY
	{
		obj_ps.polling = 1;	/* 0--interrupt mode;1--polling mode; */
		obj_ps.sensor_operate = tpd_ps_operate;
		err = hwmsen_attach(ID_PROXIMITY, &obj_ps);
		if (err)
			FTS_DBG("hwmsen attach fail, return:%d.", err);
	}
#endif
#if FT_ESD_PROTECT
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
	queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif

#if defined(CONFIG_FTS_GESTRUE_EN)
	fts_Gesture_init(tpd->dev);
#endif
#ifdef MT_PROTOCOL_B
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif

	FTS_INFO("fts Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

	enable_irq(fts_tp_irq);
	tpd_load_status = 1;

	return 0;

err_thread_fail:
#ifdef VELOCITY_CUSTOM_fts
	misc_deregister(&tpd_misc_device);
#endif

err_irq_fail:
#ifdef __MSG_DMA_MODE__
	msg_dma_release();
#endif
	return -1;
}

/************************************************************************
* Name: tpd_remove
* Brief: remove driver/channel
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
static int tpd_remove(struct i2c_client *client)
{
#ifdef __MSG_DMA_MODE__
	msg_dma_release();
#endif

#ifdef FTS_CTL_IIC
	fts_rw_iic_drv_exit();
#endif

#ifdef SYSFS_DEBUG
	fts_remove_sysfs(client);
#endif

#if FT_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

#ifdef FTS_APK_DEBUG
	fts_release_apk_debug_channel();
#endif

	FTS_INFO("TPD removed\n");

	return 0;
}


 /************************************************************************
* Name: tpd_local_init
* Brief: add driver info
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
static int tpd_local_init(void)
{
	int retval = TPD_OK;

	FTS_DBG("Focaltech fts I2C Touchscreen Driver\n");
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	if (retval != 0) {
		FTS_ERROR("Failed to set reg-vgp6 voltage: %d\n", retval);
		return -1;
	}
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		FTS_ERROR("fts unable to add i2c driver.\n");
		return -1;
	}
	if (tpd_load_status == 0) {
		FTS_ERROR("fts add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	/* TINNO_TOUCH_TRACK_IDS <--- finger number */
	/* TINNO_TOUCH_TRACK_IDS        5 */
#ifdef MT_PROTOCOL_B
#else
	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (TPD_MAX_POINTS_10 - 1), 0, 0);
#endif


#ifdef TPD_HAVE_BUTTON
	/* initialize tpd button data */
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif

	FTS_DBG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;
	return 0;
}

static void fts_release_all_finger(void)
{
#ifdef MT_PROTOCOL_B
	unsigned int finger_count = 0;
#endif

#ifndef MT_PROTOCOL_B
	input_mt_sync(tpd->dev);
#else
	for (finger_count = 0; finger_count < MT_MAX_TOUCH_POINTS; finger_count++) {
		input_mt_slot(tpd->dev, finger_count);
		input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
	}
	input_report_key(tpd->dev, BTN_TOUCH, 0);
#endif
	input_sync(tpd->dev);
}

 /************************************************************************
* Name: tpd_resume
* Brief: system wake up
* Input: no use
* Output: no
* Return: no
***********************************************************************/
static void tpd_resume(struct device *h)
{
#ifdef CONFIG_TPD_CLOSE_POWER_IN_SLEEP
	int ret;
#endif
	FTS_DBG("TPD wake up\n");

#ifdef CONFIG_TPD_PROXIMITY
	if (tpd_proximity_flag == 1) {
		if (tpd_proximity_flag_one == 1) {
			tpd_proximity_flag_one = 0;
			FTS_DBG(TPD_DEVICE " tpd_proximity_flag_one\n");
			return;
		}
	}
#endif

#if defined(CONFIG_FTS_GESTRUE_EN)
	fts_write_reg(fts_i2c_client, 0xD0, 0x00);
#endif
#ifdef CONFIG_TPD_CLOSE_POWER_IN_SLEEP
	ret = regulator_enable(tpd->reg);
	if (ret)
		FTS_ERROR("regulator_enable() failed!\n");
#else
	tpd_gpio_output(GTP_RST_PORT, 0);
	mdelay(1);
	tpd_gpio_output(GTP_RST_PORT, 1);
#endif
	enable_irq(fts_tp_irq);
	msleep(30);
	fts_release_all_finger();
	tpd_halt = 0;

#if FT_ESD_PROTECT
	count_irq = 0;
	queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif

	FTS_DBG("TPD wake up done\n");

}

 /************************************************************************
* Name: tpd_suspend
* Brief: system sleep
* Input: no use
* Output: no
* Return: no
***********************************************************************/

static void tpd_suspend(struct device *h)
{
#ifndef CONFIG_TPD_CLOSE_POWER_IN_SLEEP
	static char data = 0x3;
#endif
	int ret = 0;

	FTS_DBG("TPD enter sleep\n");
#ifdef CONFIG_TPD_PROXIMITY
	if (tpd_proximity_flag == 1) {
		tpd_proximity_flag_one = 1;
		return;
	}
#endif

#if defined(CONFIG_FTS_GESTRUE_EN)
	if (fts_updateinfo_curr.CHIP_ID == 0x54 || fts_updateinfo_curr.CHIP_ID == 0x58
	    || fts_updateinfo_curr.CHIP_ID == 0x86 || fts_updateinfo_curr.CHIP_ID == 0x87) {
		fts_write_reg(fts_i2c_client, 0xd1, 0xff);
		fts_write_reg(fts_i2c_client, 0xd2, 0xff);
		fts_write_reg(fts_i2c_client, 0xd5, 0xff);
		fts_write_reg(fts_i2c_client, 0xd6, 0xff);
		fts_write_reg(fts_i2c_client, 0xd7, 0xff);
		fts_write_reg(fts_i2c_client, 0xd8, 0xff);
	}
	fts_write_reg(fts_i2c_client, 0xd0, 0x01);
	return;
#endif

#if FT_ESD_PROTECT
	cancel_delayed_work_sync(&gtp_esd_check_work);
#endif
	tpd_halt = 1;

	disable_irq(fts_tp_irq);

#ifdef CONFIG_TPD_CLOSE_POWER_IN_SLEEP
	ret = regulator_disable(tpd->reg);
	if (ret)
		FTS_ERROR("regulator_disable() failed!\n");
#else
	mutex_lock(&i2c_access);
	if ((fts_updateinfo_curr.CHIP_ID == 0x59)) {
		data = 0x02;
		ret = fts_write_reg(fts_i2c_client, 0xA5, data);

		if (ret < 0)
			FTS_ERROR("write value fail");
	} else {
		ret = fts_write_reg(fts_i2c_client, 0xA5, data);
		if (ret < 0)
			FTS_ERROR("write value fail");
	}
	mutex_unlock(&i2c_access);
	mdelay(10);
#endif
	fts_release_all_finger();

	FTS_INFO("TPD enter sleep done\n");
}


static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "ft5x26",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,

#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif

};

  /************************************************************************
* Name: tpd_suspend
* Brief:  called when loaded into kernel
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static int __init tpd_driver_init(void)
{
	FTS_INFO("MediaTek fts touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		FTS_ERROR("add fts driver failed\n");
	return 0;
}


/************************************************************************
* Name: tpd_driver_exit
* Brief:  should never be called
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void __exit tpd_driver_exit(void)
{
	FTS_DBG("MediaTek fts touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
