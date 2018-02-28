/* drivers/input/touchscreen/ektf.c - ELAN EKTF verions of driver
*
* Copyright (C) 2011 Elan Microelectronics Corporation.
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
* 2014/0/28: The first release, version 0x0006
*             Integrated 2 ,5 ,and 10 fingers driver code together
*             auto-mapping resolution.
*             Please change following parameters
*                 1. For 5 fingers protocol, please enable ELAN_PROTOCOL.
*                    The packet size is 18 or 24 bytes.
*                 2. For 10 fingers, please enable both ELAN_PROTOCOL and ELAN_TEN_FINGERS.
*                    The packet size is 40 or 4+40+40+40 (Buffer mode) bytes.
*                 3. Please enable the ELAN_BUTTON configuration to support button.
*		  4. For ektf3k serial, Add Re-Calibration Machanism
*                    So, please enable the define of RE_CALIBRATION.
*		  5. Please enable the define of ESD_CHECK, if your firmware support
*		     "I am live" packet(0x78 0x78 0x78 0x78).
*
*
*/
#include "tpd.h"
#include "include/tpd_custom_elan3915.h"

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>

/* for linux 2.6.36.3 */
#include <linux/cdev.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/regulator/machine.h>
#endif

#ifdef PROTOCOL_B
#include <linux/input/mt.h>
#endif
#ifdef PROTOCOL_A
#include <linux/input.h>
#endif

/*dma declare, allocate and release*/
#ifdef __MSG_DMA_MODE__
#include <linux/dma-mapping.h>
#endif

/********* declare function start*******************/
static irqreturn_t tpd_eint_interrupt_handler(void);
static int touch_event_handler(void *unused);
static DECLARE_WAIT_QUEUE_HEAD(waiter);
#if defined(ESD_CHECK)
static void elan_touch_esd_func(struct work_struct *work);
#endif
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf_ts_calibrate(struct i2c_client *client);
/* static int elan_ktf_ts_resume(struct i2c_client *client); */
/* static int elan_ktf_ts_suspend(struct i2c_client *client); */
static void elan_ktf_ts_hw_reset(void);
static int __hello_packet_handler(struct i2c_client *client);
static int elan_ktf_ts_get_data(struct i2c_client *, uint8_t *, uint8_t *, size_t, size_t);

#ifdef IAP_PORTION
/* static int Update_FW_One(int source); */
static int Update_FW_One(void *unused);
#endif

#ifdef ELAN_2WIREICE
static int elan_TWO_WIRE_ICE(struct i2c_client *client);
#endif

/************declare function end****************/

static int tpd_flag;
struct task_struct *elan_thread;

#ifdef __MSG_DMA_MODE__
static u8 *g_dma_buff_va_elan;
static u8 *g_dma_buff_pa_elan;
#endif

#ifdef PROTOCOL_B
static int mTouchStatus[10] = { 0 };	/* finger_num=10 */
#endif

unsigned int elan_tp_irq = 0;	/*for MTK */

#if defined(ESD_CHECK)
static int have_interrupts;
static struct workqueue_struct *esd_wq;
static struct delayed_work esd_work;
static unsigned long delay = 2 * HZ;
#endif

static unsigned int gPrint_point;
static int debug = DEBUG_ERROR;

static uint8_t recovery;
static int fw_ver;
static int bc_ver;
static int x_res = 3216;	/* (68 - 1) * 48; */
static int y_res = 1968;	/* (42 - 1) * 48; */
static int fw_id;
static int work_lock;
static int power_lock;
static int circuit_ver = 0x01;
/*++++i2c transfer start+++++++*/
/*++++i2c transfer end+++++++*/
static struct mutex ktf_mutex;
#ifdef ELAN_BUTTON
static int button_state;
#endif

#ifdef IAP_PORTION
static int update_progree;
static uint8_t i2c_addr[3] = { 0x10, 0x20, 0x21 };	/*I2C devices address */

static unsigned char firmware[52800];
static unsigned char elan_fw_local_file[30];
static struct task_struct *fw_update_thread;


#ifdef SENSOR_OPTION
/* fw update in driver */
static uint8_t file_fw_data_TG_AUO[] = {
#include "fw_data_tg_auo.i"
};

static uint8_t file_fw_data_TG_INX[] = {
#include "fw_data_tg_inx.i"
};

static uint8_t file_fw_data_HLT_AUO[] = {
#include "fw_data_hlt_auo.i"
};

static uint8_t file_fw_data_HLT_INX[] = {
#include "fw_data_hlt_inx.i"
};
static uint8_t *file_fw_data = file_fw_data_TG_AUO;

static int path_test;
static int FW_ID_Check = 1;		/* ok = 1, fail = 0 */
#endif

enum {
	PageSize = 132,
	ACK_Fail = 0x00,
	ACK_OK = 0xAA,
	ACK_REWRITE = 0x55,
};

static int PageNum;

enum {
	E_FD = -1,
};
#endif

#ifdef _ENABLE_DBG_LEVEL
#define PROC_FS_NAME    "ektf_dbg"
#define PROC_FS_MAX_LEN 8
static struct proc_dir_entry *dbgProcFile;
#endif

struct elan_ktf_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	int intr_gpio;
	int rst_gpio;
	int power_gpio;
	/* Firmware Information */
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;
	/* For Firmare Update */
	struct miscdevice firmware;
	struct wake_lock wakelock;
};

static struct elan_ktf_ts_data *private_ts;	/*for global use */

#ifdef IAP_PORTION
static int fw_source;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX_ROTATION;
#endif

/* +++++ for DMA */
static int elants_i2c_send(struct i2c_client *client, const void *data, size_t size)
{
	int ret;

#ifdef __MSG_DMA_MODE__
	/* mutex_lock(&ts->sysfs_mutex); */
	memcpy(g_dma_buff_va_elan, data, size);
	client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;

	ret = i2c_master_send(client, (unsigned char *)g_dma_buff_pa_elan, size);

	client->addr = client->addr & I2C_MASK_FLAG & (~I2C_DMA_FLAG);
	/* mutex_unlock(&ts->sysfs_mutex); */
#else
	ret = i2c_master_send(client, data, size);
#endif

	if (ret == size)
		return size;

	if (ret >= 0)
		ret = -EIO;

	ETP_INFO("%s failed (%*ph): %d\n", __func__, (int)size, data, ret);

	return ret;
}

static int elants_i2c_read(struct i2c_client *client, void *data, size_t size)
{
	int ret;

#ifdef __MSG_DMA_MODE__
	/* mutex_lock(&ts->sysfs_mutex); */
	client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
	ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa_elan, size);
	memcpy(data, g_dma_buff_va_elan, size);
	client->addr = client->addr & I2C_MASK_FLAG & (~I2C_DMA_FLAG);
	/* mutex_unlock(&ts->sysfs_mutex); */
#else
	ret = i2c_master_recv(client, data, size);
#endif

	if (ret == size)
		return size;

	if (ret >= 0)
		ret = -EIO;

	ETP_ERROR("%s failed: %d\n", __func__, ret);

	return ret;
}

/* -----for DMA */

static int __elan_ktf_ts_poll(struct i2c_client *client)
{
	int status = 0, retry = 10;

	do {
		status = gpio_get_value(GTP_INT_PORT);
		if (status == 0)
			break;
		ETP_DEBUG("%s: status = %d\n", __func__, status);
		retry--;
		mdelay(50);
	} while (status == 1 && retry > 0);

	ETP_INFO("%s: poll interrupt status %s\n", __func__,
		    status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf_ts_poll(struct i2c_client *client)
{
	return __elan_ktf_ts_poll(client);
}

/************************************
* Restet TP
*************************************/
static void elan_ktf_ts_hw_reset(void)
{
	ETP_DEBUG(" set GTP_RST_PORT to 1\n");
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(20);
	ETP_DEBUG(" set GTP_RST_PORT to 0\n");
	tpd_gpio_output(GTP_RST_PORT, 0);
}

static int elants_i2c_power_on(struct elan_ktf_ts_data *ts)
{
	int error;

	ETP_INFO("i2c_power_on, set GTP_RST_PORT to 0\n");
	tpd_gpio_output(GTP_RST_PORT, 0);

	error = regulator_enable(tpd->reg);
	if (error) {
		ETP_ERROR("regulator_enable() failed!\n");
		return error;
	}
		ETP_INFO("i2c_power_on, regulator_enable ok\n");
	/*
	 * We need to wait a bit after powering on controller before
	 * we are allowed to release reset GPIO.
	 */
	udelay(ELAN_POWERON_DELAY_USEC);
	return 0;
}

/* For Firmware Update */
int elan_iap_open(struct inode *inode, struct file *filp)
{
	ETP_DEBUG("into elan_iap_open\n");
	if (private_ts == NULL)
		ETP_ERROR("private_ts is NULL\n");

	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
	int ret;
	char *tmp;

	ETP_DEBUG("into elan_iap_write\n");
	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
		return -ENOMEM;

	if (copy_from_user(tmp, buff, count))
		return -EFAULT;

	/*++++i2c transfer start+++++++ */
	ret = elants_i2c_send(private_ts->client, tmp, count);
	/*++++i2c transfer end+++++++ */

	kfree(tmp);
	return (ret == 1) ? count : ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	char *tmp;
	int ret;
	long rc;

	ETP_DEBUG("into elan_iap_read\n");

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
		return -ENOMEM;
	ret = elants_i2c_read(private_ts->client, tmp, count);
	if (ret >= 0)
		rc = copy_to_user(buff, tmp, count);

	kfree(tmp);

	return (ret == 1) ? count : ret;

}

static long elan_iap_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

	int __user *ip = (int __user *)arg;

	ETP_DEBUG("into elan_iap_ioctl\n");
	ETP_DEBUG("cmd value %x\n", cmd);

	switch (cmd) {
	case IOCTL_I2C_SLAVE:
		private_ts->client->addr = (int __user)arg;
		break;
	case IOCTL_FW_INFO:
		__fw_packet_handler(private_ts->client);
		break;
	case IOCTL_MINOR_FW_VER:
		break;
	case IOCTL_RESET:
		/* modify */
		elan_ktf_ts_hw_reset();
		break;
	case IOCTL_IAP_MODE_LOCK:
		if (work_lock == 0) {
			work_lock = 1;
			disable_irq(elan_tp_irq);
			cancel_work_sync(&private_ts->work);
#if defined(ESD_CHECK)
			cancel_delayed_work_sync(&esd_work);
#endif
		}
		break;
	case IOCTL_IAP_MODE_UNLOCK:
		if (work_lock == 1) {
			work_lock = 0;
			enable_irq(elan_tp_irq);
#if defined(ESD_CHECK)		/* 0604 */
			queue_delayed_work(esd_wq, &esd_work, delay);
#endif
		}
		break;
	case IOCTL_CHECK_RECOVERY_MODE:
		return recovery;

	case IOCTL_FW_VER:
		/* __fw_packet_handler(private_ts->client); */
		return fw_ver;

	case IOCTL_X_RESOLUTION:
		/* __fw_packet_handler(private_ts->client); */
		return x_res;

	case IOCTL_Y_RESOLUTION:
		/* __fw_packet_handler(private_ts->client); */
		return y_res;

	case IOCTL_FW_ID:
		/* __fw_packet_handler(private_ts->client); */
		return fw_id;

	case IOCTL_BC_VER:
		return bc_ver;

	case IOCTL_ROUGH_CALIBRATE:
		return elan_ktf_ts_calibrate(private_ts->client);

	case IOCTL_I2C_INT:
		put_user(gpio_get_value(private_ts->intr_gpio), ip);
		break;
	case IOCTL_RESUME:
		/* elan_ktf_ts_resume(private_ts->client); */
		break;
	case IOCTL_POWER_LOCK:
		power_lock = 1;
		break;
	case IOCTL_POWER_UNLOCK:
		power_lock = 0;
		break;
#ifdef IAP_PORTION
	case IOCTL_GET_UPDATE_PROGREE:
		update_progree = (int __user)arg;
		break;
	case IOCTL_FW_UPDATE:
		fw_source = 0;
		Update_FW_One(&fw_source);
		break;
#endif
#ifdef ELAN_2WIREICE
	case IOCTL_2WIREICE:
		elan_TWO_WIRE_ICE(private_ts->client);
		break;
#endif
	case IOCTL_CIRCUIT_CHECK:
		return circuit_ver;

	default:
		ETP_ERROR(" Un-known IOCTL Command %d\n", cmd);
		break;
	}
	return 0;
}

static const struct file_operations elan_touch_fops = {
	.open = elan_iap_open,
	.write = elan_iap_write,
	.read = elan_iap_read,
	.release = elan_iap_release,
	.unlocked_ioctl = elan_iap_ioctl,
};



#ifdef IAP_PORTION
int HID_EnterISPMode(struct i2c_client *client)
{
	int len = 0;
	int j;
	uint8_t flash_key[37] = {
		0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0xc0, 0xe1, 0x5a
	};
	uint8_t isp_cmd[37] = { 0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x00, 0x12, 0x34 };
	uint8_t check_addr[37] = { 0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x01, 0x10 };
	uint8_t buff[67] = { 0 };


	len = i2c_master_send(private_ts->client, flash_key, 37);
	if (len != 37) {
		ETP_ERROR(" ERROR: Flash key fail! len=%d\r\n", len);
		return -1;
	}

	ETP_DEBUG(
		    " FLASH key write data successfully! cmd = [%2x, %2x, %2x, %2x]\n",
		    flash_key[7], flash_key[8], flash_key[9], flash_key[10]);

	mdelay(20);

	len = i2c_master_send(private_ts->client, isp_cmd, 37);
	if (len != 37) {
		ETP_ERROR(" ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}

	ETP_DEBUG(
		    " IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n",
		    isp_cmd[7], isp_cmd[8], isp_cmd[9], isp_cmd[10]);

	mdelay(20);
	len = i2c_master_send(private_ts->client, check_addr, sizeof(check_addr));
	if (len != sizeof(check_addr)) {
		ETP_ERROR(" ERROR: Check Address fail! len=%d\r\n", len);
		return -1;
	}

	ETP_DEBUG(
		    " Check Address write data successfully! cmd = [%2x, %2x, %2x, %2x]\n",
		    check_addr[7], check_addr[8], check_addr[9], check_addr[10]);

	mdelay(20);

	len = i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		ETP_ERROR(" ERROR: Check Address Read Data error. len=%d \r\n",
			    len);
		return -1;
	}

	ETP_DEBUG("[Check Addr]: ");
	for (j = 0; j < 37; j++)
		ETP_DEBUG("%x ", buff[j]);
	ETP_DEBUG("\n");

	return 0;
}

int EnterISPMode(struct i2c_client *client)
{
	int len = 0;
	uint8_t isp_cmd[] = { 0x45, 0x49, 0x41, 0x50 };	/* {0x45, 0x49, 0x41, 0x50}; */

	len = i2c_master_send(private_ts->client, isp_cmd, 4);
	if (len != 4) {
		ETP_ERROR(" ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}

	ETP_DEBUG(
		    " IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n",
		    isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

int ExtractPage(struct file *filp, uint8_t *szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage, byte, &filp->f_pos);
	if (len != byte) {
		ETP_ERROR(" %s: read page error, read error. len=%d\r\n",
			    __func__, len);
		return -1;
	}

	return 0;
}

int WritePage(const u8 *szPage, int byte)
{
	int len = 0;

	len = elants_i2c_send(private_ts->client, szPage, byte);
	if (len != byte) {
		ETP_ERROR(" %s: write page error, write error. len=%d\r\n",
			    __func__, len);
		return -1;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int rc = 0;

	uint8_t buff[67] = { 0 };
#ifdef ELAN_HID_I2C
	rc = elan_ktf_ts_poll(client);
#endif
	/* rc = i2c_master_recv(private_ts->client, buff, sizeof(buff)); */
	rc = elants_i2c_read(private_ts->client, buff, sizeof(buff));
	if (rc != sizeof(buff)) {
		ETP_ERROR(" %s: Read ACK Data error. rc=%d\r\n", __func__, rc);
		return -1;
	}

	ETP_DEBUG(" %s: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", __func__,
		    buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7], buff[8],
		    buff[9], buff[10], buff[11]);
#ifndef ELAN_HID_I2C
	if (buff[0] == 0xaa && buff[1] == 0xaa)
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;
#endif
	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent, page_tatol = 351, percent_tatol;
	char str[256];

	str[0] = '\0';
	for (i = 0; i < ((page) / 10); i++) {
		str[i] = '#';
		str[i + 1] = '\0';
	}

	percent = ((100 * page) / (PageNum));
	if ((page) == (PageNum))
		percent = 100;

	if ((page_tatol) == (PageNum * ic_num))
		percent_tatol = 100;

	ETP_INFO("progress %s| %d %d", str, percent, page);

	if (page == (PageNum))
		ETP_INFO("\n");

}

/*	fw_source = 0: update fw_data.h compile with driver code
	fw_source = 1: update fw at /system/etc/firmware/elan_fw.ekt
	fw_source = 2: update fw at /data/local/tmp/ElanFW.ekt       */
/* static int Update_FW_One(int fw_source) */
static int Update_FW_One(void *unused)
{				/* for kthread use */
	int res = 0, ic_num = 1;
	int iPage = 0, rewriteCnt = 0;	/* rewriteCnt for PAGE_REWRITE */
	int i = 0;
	uint8_t data;
	int pos = 0;
	struct file *firmware_fp;
	mm_segment_t oldfs;
	uint8_t boot_buffer[4] = { 0 };
	int byte_count = 0;
	const uint8_t *szBuff = NULL;
	int curIndex = 0;
	const struct firmware *p_fw_entry;
	const u8 *fw_data;
	int rc, fw_size;
	unsigned char fw_local_path[50];
	uint8_t cal_cmd[] = { 0x54, 0x29, 0x00, 0x01 };
	uint8_t flash_key[] = { 0x54, 0xC0, 0xE1, 0x5A };
#ifdef SENSOR_OPTION
	uint8_t cmd_id[] = { 0x53, 0xf0, 0x00, 0x01 };	/*Get firmware ID */
	int major, minor;
	uint8_t buf_recv[4] = { 0 };
	int sensor_FW_ID = 0x00;
#endif
	mutex_lock(&ktf_mutex);
	ETP_INFO(" %s: Update FW\n", __func__);

IAP_RESTART:

	curIndex = 0;
	data = i2c_addr[0];	/* Master 0x10 */
	ETP_INFO(" %s: address data=0x%x \r\n", __func__, data);

	if (recovery != 0x80)
		ETP_DEBUG(" Firmware upgrade normal mode !\n");
	else
		ETP_DEBUG(" Firmware upgrade recovery mode !\n");

	/*check where fw is coming from; 0 = fw_data.h; 1 = request_firmware */
	if (fw_source == 1) {	/* use request firmware */
#ifndef SENSOR_OPTION
		ETP_INFO(" request_firmware name = %s\n", ELAN_FW_FILENAME);
		rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME, &private_ts->client->dev);
#else
		ETP_ERROR(" ---request firmware.\n");
		/*sensor option start 20160622 */
		rc = elan_ktf_ts_get_data(private_ts->client, cmd_id, buf_recv, 4, 4);
		if (rc < 0) {
			ETP_ERROR("Get Firmware ID error, exit %s.\n", __func__);
			return -1;
		}
		major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
		minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
		sensor_FW_ID = major << 8 | minor;
		ETP_ERROR(" %s sensor_FW_ID = 0x%4.4x\n", __func__,
			    sensor_FW_ID);

		if (sensor_FW_ID == FWID_TG_AUO) {
			ETP_INFO("request_firmware name = %s\n",
				    ELAN_FW_FILENAME_TG_AUO);
			rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME_TG_AUO,
					      &private_ts->client->dev);
			path_test = 3;
		} else if (sensor_FW_ID == FWID_TG_INX) {
			ETP_INFO(" request_firmware name = %s\n",
				    ELAN_FW_FILENAME_TG_INX);
			rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME_TG_INX,
					      &private_ts->client->dev);
			path_test = 4;
		} else if (sensor_FW_ID == FWID_HLT_AUO) {
			ETP_INFO(" request_firmware name = %s\n",
				    ELAN_FW_FILENAME_HLT_AUO);
			rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME_HLT_AUO,
					      &private_ts->client->dev);
			path_test = 5;
		} else if (sensor_FW_ID == FWID_HLT_INX) {
			ETP_INFO(" request_firmware name = %s\n",
				    ELAN_FW_FILENAME_HLT_INX);
			rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME_HLT_INX,
					      &private_ts->client->dev);
			path_test = 6;
		} else {
			path_test = 7;
			ETP_INFO(
				    " %s: FW_ID_Check Not Match, Don't update fw!\n",
				    __func__);
			mutex_unlock(&ktf_mutex);
			return -1;
		}
		/* sensor option end 20160622 */
#endif
		if (rc != 0) {
			ETP_ERROR(" rc=%d, request_firmware fail\n", rc);
			mutex_unlock(&ktf_mutex);
			return -1;
		}
		ETP_DEBUG("Request Firmware Size=%zu\n", p_fw_entry->size);

		fw_data = p_fw_entry->data;
		fw_size = p_fw_entry->size;
		PageNum = (fw_size / sizeof(uint8_t) / PageSize);
	} else if (fw_source == 2) {	/* data/local/tmp */
		/* Note: If want fix file name, please replace <file name> */
		sprintf(fw_local_path, "%s%s", "/data/local/tmp/", elan_fw_local_file);
		ETP_DEBUG("Update Firmware from %s\n", fw_local_path);
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		firmware_fp = filp_open(fw_local_path, O_RDONLY, S_IRUSR | S_IRGRP);
		if (PTR_ERR(firmware_fp) == -ENOENT) {
			ETP_ERROR(" open file error.\n");
			set_fs(oldfs);
			mutex_unlock(&ktf_mutex);
			return -1;
		}

		ETP_ERROR(" open file success.\n");
		PageNum = 0;
		firmware_fp->f_pos = 0;
		/* ETP_ERROR("test1.\n"); */
		for (pos = 0; pos < 1000 * 132; pos += 132, PageNum++) {
			if (firmware_fp->
			    f_op->read(firmware_fp, firmware + pos, 132,
				       &firmware_fp->f_pos) != 132) {
				/* ETP_ERROR(" break!!!!\n"); */
				break;
			}
			/* ETP_ERROR("pos = %d, PageNum = %d\n", pos, PageNum); */
		}
		/* ETP_ERROR("test2.\n"); */
		fw_data = firmware;
		ETP_INFO("%s: %s = %d,\n", __func__,
			    elan_fw_local_file, PageNum);
		set_fs(oldfs);
		filp_close(firmware_fp, NULL);

		/* test */
		/* ETP_INFO("%s: test3, return 1\n",__func__); */
		/* mutex_unlock(&ktf_mutex); */
		/* return 1; */
	} else {		/* use fw_data.h */
		ETP_DEBUG("Update Firmware by fw_data.h\n");
#ifndef SENSOR_OPTION
		PageNum = (sizeof(file_fw_data) / sizeof(uint8_t) / PageSize);
		fw_data = file_fw_data;
#else
		if (FW_ID_Check) {	/* FW ID is checked OK */
			PageNum = (sizeof(file_fw_data_TG_AUO) / sizeof(uint8_t) / PageSize);
			fw_data = file_fw_data;
			path_test = path_test + 100;
		} else {	/* FW ID check fail */

			path_test = 8;
			ETP_ERROR(
				    " %s: FW_ID_Check Not Match, Don't update fw.\n",
				    __func__);
			mutex_unlock(&ktf_mutex);
			return -1;
		}
#endif
	}
	ETP_INFO("PageNum = %d.\n", PageNum);
	/*end check firmware */

	/*Send enter bootcode cmd */
	elan_ktf_ts_hw_reset();
	mdelay(50);
	res = EnterISPMode(private_ts->client);	/* enter ISP mode */
	mdelay(100);
	/*check enter bootcode cmd */
	res = i2c_master_recv(private_ts->client, boot_buffer, 4);	/* 55 aa 33 cc */
	ETP_DEBUG("%s : Receive %x,%x,%x,%x\n", __func__, boot_buffer[0],
		    boot_buffer[1], boot_buffer[2], boot_buffer[3]);

	/* Send Dummy Byte  */
	res = i2c_master_send(private_ts->client, &data, sizeof(data));
	if (res != sizeof(data))
		ETP_ERROR("dummy error code = %d\n", res);
	else
		ETP_DEBUG("Send Dummy Byte Success!!\n");

	/* Start IAP */
	for (iPage = 1; iPage <= PageNum; iPage++) {
		for (byte_count = 1; byte_count <= 17; byte_count++) {
			szBuff = fw_data + curIndex;
			if (byte_count != 17) {
				curIndex = curIndex + 8;
				res = WritePage(szBuff, 8);
			} else {
				curIndex = curIndex + 4;
				res = WritePage(szBuff, 4);
			}
		}
		if (iPage == PageNum || iPage == 1)
			mdelay(600);
		else
			mdelay(50);

		res = GetAckData(private_ts->client);

		if (ACK_OK != res) {
			mdelay(50);
			ETP_ERROR(" ERROR: GetAckData fail! res=%d\r\n", res);
			rewriteCnt = rewriteCnt + 1;
			if (rewriteCnt == PAGERETRY) {
				ETP_ERROR(
					    " %dth page ReWrite %d times fails!\n", iPage,
					    PAGERETRY);
				mutex_unlock(&ktf_mutex);
				return E_FD;
			}
			ETP_ERROR(" %d page ReWrite %d times!\n",
				    iPage, rewriteCnt);
			goto IAP_RESTART;
		} else {
			rewriteCnt = 0;
			print_progress(iPage, ic_num, i);
			if (iPage == PageNum)
				ETP_DEBUG("%s Firmware Update Successfully!\n", __func__);
		}
	}			/* end of for(iPage = 1; iPage <= PageNum; iPage++) */

	disable_irq(elan_tp_irq);	/* prevent recv by report */

	elan_ktf_ts_hw_reset();	/* 20160114 added, to reset ic */
	mdelay(150);
	recovery = 0;

	res = __hello_packet_handler(private_ts->client);
	if (res == 0x80)
		ETP_ERROR("Recovery mode! res = %02x.\n", res);
	else
		ETP_DEBUG("hello_packet_handler() res = %d.\n", res);

	res = __fw_packet_handler(private_ts->client);
	ETP_ERROR(" __fw_packet_handler() res = %d.\n", res);
	if (res < 0)
		ETP_ERROR(
			    " res = %d, Get Firmware information error, maybe received by system interrupt!\n",
			    res);

	enable_irq(elan_tp_irq);	/* prevent recv by report */
	/* Re-calibration start 20160315 */
	mdelay(600);
	ETP_INFO("%s: Re-calibration start\n", __func__);

	if ((i2c_master_send(private_ts->client, flash_key, sizeof(flash_key))) !=
	    sizeof(flash_key)) {
		ETP_ERROR("i2c_master_send flash_key failed\n");
	}

	if ((i2c_master_send(private_ts->client, cal_cmd, sizeof(cal_cmd))) != sizeof(cal_cmd))
		ETP_ERROR("i2c_master_send cal_cmd failed\n");

	/* Re calibration end */
	ETP_DEBUG(" fw_update end.\n");
#ifdef SENSOR_OPTION
	ETP_ERROR(" !! path_test = %d.\n", path_test);
	path_test = 0;
#endif
	mutex_unlock(&ktf_mutex);
	return res;		/* 0:successfully, 0x80: Recovery, -1: No response */
}

#endif
/* End Firmware Update */

/* Star 2wireIAP which used I2C to simulate JTAG function */
#ifdef ELAN_2WIREICE
static uint8_t file_bin_data[] = {
	/* #include "2wireice.i" */
};

int write_ice_status = 0;
int shift_out_16(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = { 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbf, 0xff };

	res = i2c_master_send(client, buff, sizeof(buff));
	return res;
}

int tms_reset(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = { 0xff, 0xff, 0xff, 0xbf };

	res = i2c_master_send(client, buff, sizeof(buff));
	return res;
}

int mode_gen(struct i2c_client *client)
{
	int res;
	int retry = 5;
	uint8_t buff[] = {
		0xff, 0xff, 0xff, 0x31, 0xb7, 0xb7, 0x7b, 0xb7, 0x7b, 0x7b, 0xb7, 0x7b, 0xf3, 0xbb,
		0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xf1
	};
	uint8_t buff_1[] = { 0x2a, 0x6a, 0xa6, 0xa6, 0x6e };
	char mode_buff[2] = { 0 };

	do {
		res = i2c_master_send(client, buff, sizeof(buff));
		if (res != sizeof(buff))
			ETP_ERROR(
				    " ERROR: mode_gen write buff error, write  error. res=%d\r\n", res);
		else {
			ETP_DEBUG(" mode_gen write buff successfully.\r\n");
			break;
		}
		mdelay(20);
		retry -= 1;
	} while (retry);
	res = i2c_master_recv(client, mode_buff, sizeof(mode_buff));
	if (res != sizeof(mode_buff)) {
		ETP_ERROR(
			    " ERROR: mode_gen read data error, write  error. res=%d\r\n", res);
		return -1;
	}
	ETP_DEBUG(
		    " mode gen read successfully(a6 59)! buff[0]=0x%x  buff[1]=0x%x \r\n",
		    mode_buff[0], mode_buff[1]);

	res = i2c_master_send(client, buff_1, sizeof(buff_1));
	if (res != sizeof(buff_1)) {
		ETP_ERROR(" ERROR: mode_gen write buff_1 error. res=%d\r\n", res);
		return -1;
	}
	return res;
}

int word_scan_out(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26, 0x66 };

	res = i2c_master_send(client, buff, sizeof(buff));
	return res;
}

int long_word_scan_out(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {
		0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
		0x22, 0x22, 0x26, 0x66
	};
	res = i2c_master_send(client, buff, sizeof(buff));
	return res;
}


int bit_manipulation(int TDI, int TMS, int TCK, int TDO, int TDI_1, int TMS_1, int TCK_1, int TDO_1)
{
	int res;

	res = ((TDI << 3 | TMS << 2 | TCK | TDO) << 4) | (TDI_1 << 3 | TMS_1 << 2 | TCK_1 | TDO_1);
	return res;
}

int ins_write(struct i2c_client *client, uint8_t buf)
{
	int res = 0;
	int length = 13;
	uint8_t write_buf[7] = { 0 };
	int TDI_bit[13] = { 0 };
	int TMS_bit[13] = { 0 };
	int i = 0;
	uint8_t buf_rev = 0;
	int TDI = 0, TMS = 0, TCK = 0, TDO = 0;
	int bit_tdi, bit_tms;
	int len;

	for (i = 0; i < 8; i++)
		buf_rev = buf_rev | (((buf >> i) & 0x01) << (7 - i));

	TDI = (0x7 << 10) | buf_rev << 2 | 0x00;
	TMS = 0x1007;
	TCK = 0x2;
	TDO = 1;

	for (len = 0; len <= length - 1; len++) {
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		TDI_bit[length - 1 - len] = bit_tdi;
		TMS_bit[length - 1 - len] = bit_tms;
		TDI = TDI >> 1;
		TMS = TMS >> 1;
	}

	for (len = 0; len <= length - 1; len = len + 2) {
		if (len == length - 1 && len % 2 == 0)
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0, 0, 0, 0);
		else
			res =
			    bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len + 1],
					     TMS_bit[len + 1], TCK, TDO);
		write_buf[len / 2] = res;
	}

	res = i2c_master_send(client, write_buf, sizeof(write_buf));
	return res;
}

int word_scan_in(struct i2c_client *client, uint16_t buf)
{
	int res = 0;
	uint8_t write_buf[10] = { 0 };
	int TDI_bit[20] = { 0 };
	int TMS_bit[20] = { 0 };


	int TDI = buf << 2 | 0x00;
	int TMS = 0x7;
	int TCK = 0x2;
	int TDO = 1;

	int bit_tdi, bit_tms;
	int len;

	for (len = 0; len <= 19; len++) {	/* length =20 */
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;

		TDI_bit[19 - len] = bit_tdi;
		TMS_bit[19 - len] = bit_tms;
		TDI = TDI >> 1;
		TMS = TMS >> 1;
	}

	for (len = 0; len <= 19; len = len + 2) {
		if (len == 19 && len % 2 == 0)
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0, 0, 0, 0);
		else
			res =
			    bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len + 1],
					     TMS_bit[len + 1], TCK, TDO);
		write_buf[len / 2] = res;
	}

	res = i2c_master_send(client, write_buf, sizeof(write_buf));
	return res;
}

int long_word_scan_in(struct i2c_client *client, int buf_1, int buf_2)
{
	uint8_t write_buf[18] = { 0 };
	uint8_t TDI_bit[36] = { 0 };
	uint8_t TMS_bit[36] = { 0 };

	int TDI_1 = buf_1;
	int TDI_2 = (buf_2 << 2) | 0x00;
	int TMS = 0x7;
	int TCK = 0x2;
	int TDO = 1;

	int bit_tdi, bit_tms;
	int len = 0;
	int res = 0;

	for (len = 0; len <= 35; len++) {	/* length =36 */

		if (len < 18)
			bit_tdi = TDI_2 & 0x1;
		else
			bit_tdi = TDI_1 & 0x1;

		bit_tms = TMS & 0x1;

		TDI_bit[35 - len] = bit_tdi;
		TMS_bit[35 - len] = bit_tms;
		if (len < 18)
			TDI_2 = TDI_2 >> 1;
		else
			TDI_1 = TDI_1 >> 1;

		TMS = TMS >> 1;
		bit_tdi = 0;
		bit_tms = 0;
	}

	for (len = 0; len <= 35; len = len + 2) {
		if (len == 35 && len % 2 == 0)
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0, 0, 0, 1);
		else
			res =
			    bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len + 1],
					     TMS_bit[len + 1], TCK, TDO);
		write_buf[len / 2] = res;
	}

	res = i2c_master_send(client, write_buf, sizeof(write_buf));
	return res;
}

uint16_t trimtable[8] = { 0 };

int Read_SFR(struct i2c_client *client, int open)
{
	uint8_t voltage_recv[2] = { 0 };

	int count, ret;
	/* uint16_t address_1[8]={0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007}; */

	ins_write(client, 0x6f);	/* IO write */
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	/* 0 */
	ins_write(client, 0x6f);	/* IO Write */
	long_word_scan_in(client, 0x007f, 0x9002);	/* TM=2h */
	ins_write(client, 0x68);	/* Program Memory Sequential Read */
	word_scan_in(client, 0x0000);	/* set Address 0x0000 */
	shift_out_16(client);	/* move data to I2C buf */

	mdelay(10);
	count = 0;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if (ret != sizeof(voltage_recv)) {
		ETP_ERROR(" %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}

	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
	ETP_DEBUG(
		    " voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n",
		    voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	/* 1 */
	ins_write(client, 0x6f);	/* IO write */
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0001);
	shift_out_16(client);

	mdelay(1);
	count = 1;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if (ret != sizeof(voltage_recv)) {
		ETP_ERROR(" %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}

	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
	ETP_DEBUG(
		    " read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n",
		    voltage_recv[0], voltage_recv[1], count, trimtable[count]);

	/* 2 */
	ins_write(client, 0x6f);	/* IO write */
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0002);
	shift_out_16(client);

	mdelay(1);
	count = 2;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if (ret != sizeof(voltage_recv)) {
		ETP_ERROR(" %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}

	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
	ETP_DEBUG(
		    " read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n",
		    voltage_recv[0], voltage_recv[1], count, trimtable[count]);


	/* 3 */
	ins_write(client, 0x6f);	/* IO write */
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0003);
	shift_out_16(client);

	mdelay(1);
	count = 3;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if (ret != sizeof(voltage_recv)) {
		ETP_ERROR(" %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}

	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
	ETP_DEBUG(
		    " read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n",
		    voltage_recv[0], voltage_recv[1], count, trimtable[count]);

	/* 4 */
	ins_write(client, 0x6f);	/* IO write */
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0004);
	shift_out_16(client);

	mdelay(1);
	count = 4;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if (ret != sizeof(voltage_recv)) {
		ETP_ERROR(" %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}

	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
	ETP_DEBUG(
		    " read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n",
		    voltage_recv[0], voltage_recv[1], count, trimtable[count]);

	/* 5 */
	ins_write(client, 0x6f);	/* IO write */
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0005);
	shift_out_16(client);

	mdelay(1);
	count = 5;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if (ret != sizeof(voltage_recv)) {
		ETP_ERROR(" %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}

	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
	ETP_DEBUG(
		    " read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n",
		    voltage_recv[0], voltage_recv[1], count, trimtable[count]);

	/* 6 */
	ins_write(client, 0x6f);	/* IO write */
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0006);
	shift_out_16(client);

	mdelay(1);
	count = 6;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if (ret != sizeof(voltage_recv)) {
		ETP_ERROR(" %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}

	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
	ETP_DEBUG(
		    " read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n",
		    voltage_recv[0], voltage_recv[1], count, trimtable[count]);

	/* 7 */
	ins_write(client, 0x6f);	/* IO write */
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0007);
	shift_out_16(client);

	mdelay(1);
	count = 7;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if (ret != sizeof(voltage_recv)) {
		ETP_ERROR(" %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	if (open == 1)
		trimtable[count] = voltage_recv[0] << 8 | (voltage_recv[1] & 0xbf);
	else
		trimtable[count] = voltage_recv[0] << 8 | (voltage_recv[1] | 0x40);
	ETP_ERROR(
		    " Open_High_Voltage recv  voltage_recv buff[0]=%x buff[1]=%x, trimtable[%d]=%x\n",
		    voltage_recv[0], voltage_recv[1], count, trimtable[count]);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);

	return 0;
}

int Write_SFR_2k(struct i2c_client *client, int open)
{

	/* set page 1 */
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x0001, 0x0100);
	if (open == 1) {
		/* set HV enable */
		ETP_DEBUG("%s set HV enable\n", __func__);
		ins_write(client, 0x6f);
		long_word_scan_in(client, 0x0050, 0xc041);
	} else {
		/* set HV disable */
		ETP_DEBUG("%s set HV disable\n", __func__);
		ins_write(client, 0x6f);
		long_word_scan_in(client, 0x0050, 0xc040);
	}
	return 0;
}

int Write_SFR(struct i2c_client *client)
{

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9001);


	ins_write(client, 0x66);	/* Program Memory Write */
	long_word_scan_in(client, 0x0000, trimtable[0]);
	ins_write(client, 0xfd);	/* Set up the initial addr for sequential access */
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0001, trimtable[1]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0002, trimtable[2]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0003, trimtable[3]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0004, trimtable[4]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0005, trimtable[5]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0006, trimtable[6]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0007, trimtable[7]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);


	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x7f, 0x8000);

	return 0;
}

int Enter_Mode(struct i2c_client *client)
{
	mode_gen(client);
	tms_reset(client);
	ins_write(client, 0xfc);	/* system reset */
	tms_reset(client);
	return 0;
}

int Open_High_Voltage(struct i2c_client *client, int open)
{
#ifdef EKTF3K_FLASH
	Read_SFR(client, open);
	Write_SFR(client);
	Read_SFR(client, open);

#endif
	Write_SFR_2k(client, open);
	return 0;
}

int Mass_Erase(struct i2c_client *client)
{
	char mass_buff[4] = { 0 };
	char mass_buff_1[2] = { 0 };
	int ret, finish = 0, i = 0;

	ETP_DEBUG(" Mass_Erase!!!!\n");
	ins_write(client, 0x01);	/* id code read */
	mdelay(2);
	long_word_scan_out(client);

	ret = i2c_master_recv(client, mass_buff, sizeof(mass_buff));
	ETP_DEBUG(" Mass_Erase mass_buff=%x %x %x %x(c0 08 01 00)\n",
			mass_buff[0], mass_buff[1], mass_buff[2], mass_buff[3]);	/* id: c0 08 01 00 */

	ins_write(client, 0x6f);	/* IO Write */
	/*add by herman */
	long_word_scan_in(client, 0x007e, 0x0020);

	long_word_scan_in(client, 0x007f, 0x4000);	/* orig 4000 */
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9040);
	ins_write(client, 0x66);	/* Program data Write */
	long_word_scan_in(client, 0x0000, 0x8765);	/* change by herman */
	ins_write(client, 0x6f);	/* IO Write */
	long_word_scan_in(client, 0x007f, 0x8000);	/* clear flash control PROG */

	ins_write(client, 0xf3);

	while (finish == 0) {
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
		if (ret != sizeof(mass_buff_1)) {
			ETP_ERROR(" ERROR: read data error. res=%d\r\n", ret);
			return -1;
		}

		finish = (mass_buff_1[1] >> 4) & 0x01;
		ETP_DEBUG(" mass_buff_1[0]=%x, mass_buff_1[1]=%x (80 10)!!!!!!!!!! finish=%d\n",
				mass_buff_1[0], mass_buff_1[1], finish);	/* 80 10: OK, 80 00: fail */

		if (mass_buff_1[1] != i2c_addr[0] && finish != 1 && i < 100) {
			mdelay(100);
			i++;
			if (i == 50)
				ETP_ERROR(" Mass_Erase fail !\n");
		}
	}
	return 0;
}

int Reset_ICE(struct i2c_client *client)
{
	/* struct elan_ktf_ts_data *ts = i2c_get_clientdata(client); */
	int res;

	ETP_INFO(" Reset ICE!!!!\n");
	ins_write(client, 0x94);
	ins_write(client, 0xd4);
	ins_write(client, 0x20);
	client->addr = i2c_addr[0];	/* //Modify address before 2-wire */
	elan_ktf_ts_hw_reset();
	mdelay(250);
	res = __hello_packet_handler(client);

	return 0;
}

int normal_write_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	/* char buff_check=0; */
	uint16_t szbuff = 0, szbuff_1 = 0;
	uint16_t sendbuff = 0;
	int write_byte, iw;

	ins_write(client, 0xfd);
	word_scan_in(client, j * 64);

	ins_write(client, 0x65);	/* Program data sequential write */

	write_byte = 64;

	for (iw = 0; iw < write_byte; iw++) {
		szbuff = *szBuff;
		szbuff_1 = *(szBuff + 1);
		sendbuff = szbuff_1 << 8 | szbuff;
		ETP_DEBUG("  Write Page sendbuff=0x%04x @@@\n", sendbuff);
		/* mdelay(1); */
		word_scan_in(client, sendbuff);	/* data????   buff_read_data */
		szBuff += 2;
	}
	return 0;
}

int fastmode_write_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	uint8_t szfwbuff = 0, szfwbuff_1 = 0;
	uint8_t sendfwbuff[130] = { 0 };
	uint8_t tmpbuff;
	int i = 0, len = 0;

	private_ts->client->addr = 0x76;

	sendfwbuff[0] = (j * 64) >> 8;
	tmpbuff = ((j * 64) << 8) >> 8;
	sendfwbuff[1] = tmpbuff;
	/* ETP_INFO("fastmode_write_func, sendfwbuff[0]=0x%x, sendfwbuff[1]=0x%x\n",
			sendfwbuff[0], sendfwbuff[1]); */

	for (i = 2; i < 129; i = i + 2) {	/* 1 Page = 64 word, 1 word=2Byte */

		szfwbuff = *szBuff;
		szfwbuff_1 = *(szBuff + 1);
		sendfwbuff[i] = szfwbuff_1;
		sendfwbuff[i + 1] = szfwbuff;
		szBuff += 2;
		/* ETP_INFO(" sendfwbuff[%d]=0x%x, sendfwbuff[%d]=0x%x\n",
				i, sendfwbuff[i], i+1, sendfwbuff[i+1]); */
	}


	len = i2c_master_send(private_ts->client, sendfwbuff, 130);
	if (len != 130) {	/* address+data(128) */
		ETP_ERROR(
			    " ERROR: fastmode write page error, len=%d, Page %d\r\n",
			    len, j);
		return -1;
	}
	/* ETP_INFO("fastmode_write_func, send len=%d (130), Page %d --\n", len, j); */

	private_ts->client->addr = 0x77;

	return 0;
}


int ektSize;
int lastpage_byte;
int lastpage_flag = 0;
int Write_Page(struct i2c_client *client, int j, uint8_t *szBuff)
{
	int len, finish = 0;
	char buff_read_data[2];
	int i = 0;

	ins_write(client, 0x6f);	/* IO Write */
	/* long_word_scan_in(client,0x007e,0x0023); */
	long_word_scan_in(client, 0x007f, 0x8000);
	long_word_scan_in(client, 0x007f, 0x9400);

	ins_write(client, 0x66);	/* Program Data Write */
	/* long_word_scan_in(client,0x0000,0x5a5a); */
	long_word_scan_in(client, j * 64, 0x0000);
	/* normal_write_func(client, j, szBuff); ////////////choose one : normal / fast mode */
	fastmode_write_func(client, j, szBuff);	/* //////// */

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9000);

	/* ins_write(client,0x6f); */
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0xf3);	/* Debug Reg Read */

	while (finish == 0) {
		word_scan_out(client);
		len = i2c_master_recv(client, buff_read_data, sizeof(buff_read_data));
		if (len != sizeof(buff_read_data)) {
			ETP_ERROR(
				    " ERROR: Write_Page read buff_read_data error, len=%d\r\n",
				    len);
			return E_FD;
		}

		finish = (buff_read_data[1] >> 4) & 0x01;
		/* ETP_INFO(" read data successfully! buff[0]=0x%x  buff[1]=0x%x  finish=%d\r\n",
				buff_read_data[0], buff_read_data[1], finish);  //80 10: ok */

		if (finish != 1) {
			mdelay(10);
			/* ETP_INFO(" Write_Page finish !=1\n"); */
			i++;
			if (i == 50) {
				ETP_ERROR(" Write_Page finish !=1, Page=%d\n",
					    j);
				write_ice_status = 1;
				return -1;
			}
		}
	}
	return 0;
}

int fastmode_read_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	uint8_t szfrbuff = 0, szfrbuff_1 = 0;
	uint8_t sendfrbuff[2] = { 0 };
	uint8_t recvfrbuff[130] = { 0 };
	uint16_t tmpbuff;
	int i = 0, len = 0, retry = 0;

	ins_write(client, 0x67);

	private_ts->client->addr = 0x76;

	sendfrbuff[0] = (j * 64) >> 8;
	tmpbuff = ((j * 64) << 8) >> 8;
	sendfrbuff[1] = tmpbuff;
	/* ETP_INFO("fastmode_write_func, sendfrbuff[0]=0x%x, sendfrbuff[1]=0x%x\n", sendfrbuff[0], sendfrbuff[1]); */
	len = i2c_master_send(private_ts->client, sendfrbuff, sizeof(sendfrbuff));

	len = i2c_master_recv(private_ts->client, recvfrbuff, sizeof(recvfrbuff));
	/* ETP_INFO("fastmode_read_func, recv len=%d (128)\n", len); */

	for (i = 2; i < 129; i = i + 2) {
		szfrbuff = *szBuff;
		szfrbuff_1 = *(szBuff + 1);
		szBuff += 2;
		if (recvfrbuff[i] != szfrbuff_1 || recvfrbuff[i + 1] != szfrbuff) {
			ETP_ERROR(
				    " @@@@Read Page Compare Fail. recvfrbuff[%d]=%x, recvfrbuff[i+1]=%x, szfrbuff_1=%x, szfrbuff=%x, ,j =%d@@@@@@@@@@@@@@@@\n\n",
				    i, recvfrbuff[i], recvfrbuff[i + 1], szfrbuff_1, szfrbuff, j);
			write_ice_status = 1;
			retry = 1;
		}
		break;		/* for test */
	}

	private_ts->client->addr = 0x77;
	if (retry == 1)
		return -1;
	else
		return 0;
}

int normal_read_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	char read_buff[2];
	int m, len, read_byte;
	uint16_t szbuff = 0, szbuff_1 = 0;

	ins_write(client, 0xfd);

	/* ETP_INFO(" Read_Page, j*64=0x%x\n", j*64); */
	word_scan_in(client, j * 64);
	ins_write(client, 0x67);

	word_scan_out(client);

	read_byte = 64;
	/* for(m=0;m<64;m++){ */
	for (m = 0; m < read_byte; m++) {
		/* compare...... */
		word_scan_out(client);
		len = i2c_master_recv(client, read_buff, sizeof(read_buff));

		szbuff = *szBuff;
		szbuff_1 = *(szBuff + 1);
		szBuff += 2;
		ETP_DEBUG(" Read Page: byte=%x%x, szbuff=%x%x\n",
			    read_buff[0], read_buff[1], szbuff, szbuff_1);

		if (read_buff[0] != szbuff_1 || read_buff[1] != szbuff) {
			ETP_ERROR(
				    " @@@@@@@@@@Read Page Compare Fail. j =%d. m=%d.@@@@@@@@@@@@@@@@\n\n",
				    j, m);
			write_ice_status = 1;
		}
	}
	return 0;
}

int Read_Page(struct i2c_client *client, int j, uint8_t *szBuff)
{
	int res = 0;

	ins_write(client, 0x6f);
	/* long_word_scan_in(client,0x007e,0x0023); */
	long_word_scan_in(client, 0x007f, 0x9000);
	ins_write(client, 0x68);

	/* mdelay(10); //for malata */
	/* normal_read_func(client, j,  szBuff); ////////////////choose one: normal / fastmode */
	fastmode_read_func(client, j, szBuff);

	/* Clear Flashce */
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x0000);
	if (res == -1)
		return -1;
	return 0;
}


int TWO_WIRE_ICE(struct i2c_client *client)
{
	int i;

	uint8_t *szBuff = NULL;
	/* char szBuff[128]={0}; */
	int curIndex = 0;
	int PageSize = 128;
	int res;
	/* int ektSize; */
	/* test */
	write_ice_status = 0;
	ektSize = sizeof(file_bin_data) / PageSize;
	client->addr = 0x77;	/* //Modify address before 2-wire */

	ETP_INFO(" ektSize=%d ,modify address = %x\n ", ektSize, client->addr);

	i = Enter_Mode(client);
	i = Open_High_Voltage(client, 1);
	if (i == -1) {
		ETP_ERROR(" Open High Voltage fail\n");
		return -1;
	}
	/* return 0; */

	i = Mass_Erase(client);	/* mark temp */
	if (i == -1) {
		ETP_ERROR(" Mass Erase fail\n");
		return -1;
	}
	/* for fastmode */
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0036);
	long_word_scan_in(client, 0x007f, 0x8000);
	long_word_scan_in(client, 0x007e, 0x0023);	/* add by herman */

	/* client->addr = 0x76;////Modify address before 2-wire */
	ETP_INFO("[elan-test] client->addr =%2x\n", client->addr);
	/* for fastmode */
	for (i = 0; i < ektSize; i++) {
		szBuff = file_bin_data + curIndex;
		curIndex = curIndex + PageSize;
		/* ETP_INFO(" Write_Page %d........................wait\n ", i); */

		res = Write_Page(client, i, szBuff);
		if (res == -1) {
			ETP_ERROR(" Write_Page %d fail\n ", i);
			break;
		}
		/* ETP_INFO(" Read_Page %d........................wait\n ", i); */
		mdelay(1);
		Read_Page(client, i, szBuff);
		/* ETP_INFO(" Finish  %d  Page!!!!!!!.........wait\n ", i); */
	}

	if (write_ice_status == 0)
		ETP_INFO(" Update_FW_Boot Finish!!!\n");
	else
		ETP_ERROR(" Update_FW_Boot fail!!!\n");

	i = Open_High_Voltage(client, 0);
	if (i == -1)
		return -1;	/* test */

	Reset_ICE(client);

	return 0;
}

static int elan_TWO_WIRE_ICE(struct i2c_client *client)
{				/* for driver internal 2-wire ice */
	work_lock = 1;
	disable_irq(elan_tp_irq);
	/* wake_lock(&private_ts->wakelock); */
	TWO_WIRE_ICE(client);
	work_lock = 0;
	enable_irq(elan_tp_irq);
	/* wake_unlock(&private_ts->wakelock); */
	return 0;
}

/* End 2WireICE */
#endif


int CheckISPstatus(struct i2c_client *client)
{
	int len = 0;
	int j;
	uint8_t checkstatus[37] = { 0x04, 0x00, 0x23, 0x00, 0x03, 0x18 };
	uint8_t buff[67] = { 0 };

	len = i2c_master_send(private_ts->client, checkstatus, sizeof(checkstatus));
	if (len != sizeof(checkstatus)) {
		ETP_ERROR(" ERROR: Flash key fail! len=%d\r\n", len);
		return -1;
	}

	ETP_DEBUG(
		    " check status write data successfully! cmd = [%x, %x, %x, %x, %x, %x]\n",
		    checkstatus[0], checkstatus[1], checkstatus[2], checkstatus[3],
		    checkstatus[4], checkstatus[5]);

	mdelay(10);
	/* len=i2c_master_recv(private_ts->client, buff, sizeof(buff)); */
	len = elants_i2c_read(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		ETP_ERROR(" ERROR: Check Address Read Data error. len=%d \r\n",
			    len);
		return -1;
	}

	ETP_INFO("[Check status]: ");
	for (j = 0; j < 37; j++)
		ETP_INFO("%x ", buff[j]);
	ETP_INFO("\n");
	if (buff[6] == 0x80)
		return 0x80;	/* return recovery mode 0x88 */

	return 0;
}

int HID_RecoveryISP(struct i2c_client *client)
{
	int len = 0;
	int j;
	uint8_t flash_key[37] = {
		0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0xc0, 0xe1, 0x5a
	};
	/* uint8_t isp_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x00, 0x12, 0x34}; */
	uint8_t check_addr[37] = { 0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x01, 0x10 };
	uint8_t buff[67] = { 0 };

	len = i2c_master_send(private_ts->client, flash_key, 37);
	if (len != 37) {
		ETP_ERROR("Flash key fail! len=%d\r\n", len);
		return -1;
	}

	ETP_DEBUG(
		    " FLASH key write data successfully! cmd = [%2x, %2x, %2x, %2x]\n",
		    flash_key[7], flash_key[8], flash_key[9], flash_key[10]);

	mdelay(20);
	/*
	   len = i2c_master_send(private_ts->client, isp_cmd,  37);
	   if (len != 37) {
	   ETP_ERROR(" ERROR: EnterISPMode fail! len=%d\r\n", len);
	   return -1;
	   }

	ETP_DEBUG(" IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n",
		isp_cmd[7], isp_cmd[8], isp_cmd[9], isp_cmd[10]);
	 */

	mdelay(20);
	len = i2c_master_send(private_ts->client, check_addr, sizeof(check_addr));
	if (len != sizeof(check_addr)) {
		ETP_ERROR("Check Address fail! len=%d\r\n", len);
		return -1;
	}

	ETP_DEBUG(
		    "Check Address write data succes! cmd = [%2x, %2x, %2x, %2x]\n",
		    check_addr[7], check_addr[8], check_addr[9], check_addr[10]);

	mdelay(20);
	/* len=i2c_master_recv(private_ts->client, buff, sizeof(buff)); */
	len = elants_i2c_read(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		ETP_ERROR("Check Address Read Data error. len=%d \r\n", len);
		return -1;
	}

	ETP_INFO("[Check Addr]: ");
	for (j = 0; j < 37; j++)
		ETP_INFO("%x ", buff[j]);
	ETP_INFO("\n");

	return 0;
}

int SendEndCmd(struct i2c_client *client)
{
	int len = 0;
	uint8_t send_cmd[37] = { 0x04, 0x00, 0x23, 0x00, 0x03, 0x1A };

	len = i2c_master_send(private_ts->client, send_cmd, sizeof(send_cmd));
	if (len != sizeof(send_cmd)) {
		ETP_ERROR(" ERROR: Send Cmd fail! len=%d\r\n", len);
		return -1;
	}

	ETP_DEBUG(
		    " check status write data successfully! cmd = [%x, %x, %x, %x, %x, %x]\n",
		    send_cmd[0], send_cmd[1], send_cmd[2], send_cmd[3], send_cmd[4],
		    send_cmd[5]);

	return 0;
}

/* start sysfs +++ */
static ssize_t store_debug_mesg(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	int level[SYSFS_MAX_LEN];
	int ret;

	ret =  kstrtoint(buf, 0, &level[0]);
	/* ret = sscanf(buf, "%x", &level[0]); */
	debug = level[0];
	ETP_INFO("debug level %d, size %d\n", debug, (int)count);
	return count;
}

static ssize_t show_debug_mesg(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Debug level %d\n", debug);
}

static ssize_t show_gpio_int(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(private_ts->intr_gpio));
}
static ssize_t show_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;

	elan_ktf_ts_hw_reset();
	ETP_DEBUG("Reset Touch Screen Controller!\n");
	count = sprintf(buf, "Reset Touch Screen Controller\n");

	return count;
}

static ssize_t store_reset(struct device *dev, struct device_attribute *attr, const char *buf,
			   size_t count)
{
	elan_ktf_ts_hw_reset();
	ETP_INFO("Reset Touch Screen Controller!\n");
	return count;
}
static ssize_t show_enable_irq(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;

	work_lock = 0;
	enable_irq(elan_tp_irq);
	wake_unlock(&private_ts->wakelock);
	ETP_DEBUG("Enable IRQ.\n");
	count = sprintf(buf, "Enable IRQ.\n");
	return count;
}

static ssize_t store_enable_irq(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	work_lock = 0;
	enable_irq(elan_tp_irq);
	wake_unlock(&private_ts->wakelock);
	ETP_INFO("Enable IRQ.\n");
	return count;
}
static ssize_t show_disable_irq(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;

	work_lock = 1;
	disable_irq(elan_tp_irq);
	wake_lock(&private_ts->wakelock);
	ETP_DEBUG("Disable IRQ.\n");
	count = sprintf(buf, "Disable IRQ.\n");

	return count;
}

static ssize_t store_disable_irq(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t count)
{
	work_lock = 1;
	disable_irq(elan_tp_irq);
	wake_lock(&private_ts->wakelock);

	ETP_INFO("Disable IRQ.\n");
	return count;
}

static ssize_t store_calibrate(struct device *dev, struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	ret = elan_ktf_ts_calibrate(client);

	if (ret == 0)
		ETP_INFO(" Calibrate Finish\n");
	else
		ETP_ERROR(" Calibrate Fail\n");
	return count;
}

static ssize_t store_fw_update_in_driver(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret;

	work_lock = 1;
	disable_irq(elan_tp_irq);
	wake_lock(&private_ts->wakelock);

	fw_source = 0;
	ret = Update_FW_One(&fw_source);

	work_lock = 0;
	enable_irq(elan_tp_irq);
	wake_unlock(&private_ts->wakelock);

	ETP_INFO(" Update Firmware in driver: fw_data.h\n");
	return count;
}

static ssize_t store_fw_update(struct device *dev, struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret;

	work_lock = 1;
	disable_irq(elan_tp_irq);
	wake_lock(&private_ts->wakelock);

	fw_source = 1;
	ret = Update_FW_One(&fw_source);	/* use request firmware 20160310 */

	work_lock = 0;
	enable_irq(elan_tp_irq);
	wake_unlock(&private_ts->wakelock);

	ETP_INFO("Update Firmware at /system/etc/firmware/.\n");
	return count;
}

static ssize_t store_fw_update_elan(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%s", elan_fw_local_file);
	ret = strlen(elan_fw_local_file);
	if (0 == ret) {
		ETP_ERROR(" ERROR: firmware name is empty\n");
		ETP_ERROR(" usage:\n");
		ETP_ERROR(" \techo <firmware name> > fw_update_elan\n");
		ETP_ERROR(" \tNote: <firmware file must push on /data/local/tmp\n");
		return -1;
	}
	fw_source = 2;
	work_lock = 1;
	disable_irq(elan_tp_irq);
	wake_lock(&private_ts->wakelock);
	ret = Update_FW_One(&fw_source);	/* firmware is located at /data/local/tmp */

	work_lock = 0;
	enable_irq(elan_tp_irq);
	wake_unlock(&private_ts->wakelock);

	ETP_INFO(" Update Firmware in /data/local/tmp/\n");
	return count;
}

#ifdef ELAN_2WIREICE
static ssize_t show_2wire(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	work_lock = 1;
	disable_irq(elan_tp_irq);
	wake_lock(&private_ts->wakelock);

	ret = TWO_WIRE_ICE(client);

	work_lock = 0;
	enable_irq(elan_tp_irq);
	wake_unlock(&private_ts->wakelock);

	return sprintf(buf, "Update Firmware by 2wire JTAG\n");
}
#endif

static ssize_t show_fw_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	work_lock = 1;
	disable_irq(elan_tp_irq);
	wake_lock(&private_ts->wakelock);

	__fw_packet_handler(private_ts->client);

	work_lock = 0;
	enable_irq(elan_tp_irq);
	wake_unlock(&private_ts->wakelock);

	return sprintf(buf, "FW VER = 0x%x, FW ID = 0x%x, BC VER = 0x%x\n", ts->fw_ver, ts->fw_id,
		       ts->bc_ver);
}

static ssize_t show_drv_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "Elan driver version 0x0006");
}

static ssize_t show_iap_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%s\n", (ts->fw_ver == 0) ? "Recovery" : "Normal");
}


static ssize_t elan_ktf_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->intr_gpio);
	ETP_DEBUG("GPIO_TP_INT_N=%d\n", ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static ssize_t elan_ktf_vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "elan_ktf", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}


static DEVICE_ATTR(debug_mesg, S_IRUGO | S_IWUSR, show_debug_mesg, store_debug_mesg);
static DEVICE_ATTR(gpio_int, S_IRUGO, show_gpio_int, NULL);
static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, show_reset, store_reset);
static DEVICE_ATTR(enable_irq, S_IRUGO | S_IWUSR, show_enable_irq, store_enable_irq);
static DEVICE_ATTR(disable_irq, S_IRUGO | S_IWUSR, show_disable_irq, store_disable_irq);
static DEVICE_ATTR(calibrate, S_IWUSR, NULL, store_calibrate);
static DEVICE_ATTR(fw_info, S_IRUGO, show_fw_info, NULL);
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);
static DEVICE_ATTR(fw_update_in_driver, S_IWUSR, NULL, store_fw_update_in_driver);
static DEVICE_ATTR(fw_update, S_IWUSR, NULL, store_fw_update);
static DEVICE_ATTR(fw_update_elan, S_IWUSR, NULL, store_fw_update_elan);
#ifdef ELAN_2WIREICE
static DEVICE_ATTR(2 wire, S_IRUGO, show_2wire, NULL);
#endif
static DEVICE_ATTR(iap_mode, S_IRUGO, show_iap_mode, NULL);
static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf_gpio_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf_vendor_show, NULL);

static struct attribute *elan_attributes[] = {
	&dev_attr_debug_mesg.attr,
	&dev_attr_gpio_int.attr,
	&dev_attr_reset.attr,
	&dev_attr_enable_irq.attr,
	&dev_attr_disable_irq.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_fw_info.attr,
	&dev_attr_drv_version.attr,
	&dev_attr_fw_update_in_driver.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_fw_update_elan.attr,
#ifdef ELAN_2WIREICE
	&dev_attr_2wire.attr,
#endif
	&dev_attr_iap_mode.attr,
	&dev_attr_gpio.attr,
	&dev_attr_vendor.attr,
	NULL
};

static struct attribute_group elan_attribute_group = {
	.name = DEVICE_NAME,
	.attrs = elan_attributes,
};

/* end sysfs -- */

static int elan_ktf_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf,
				size_t w_size, size_t r_size)
{
	int rc;
	/* ETP_INFO("%s: enter\n", __func__); */

	if (buf == NULL)
		return -EINVAL;

	/* if ((i2c_master_send(client, cmd, w_size)) != w_size) { */
	if ((elants_i2c_send(client, cmd, w_size)) != w_size) {
		ETP_ERROR("%s: i2c send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf_ts_poll(client);
	if (rc < 0)
		ETP_ERROR("%s: poll is high\n", __func__);

	if (r_size <= 0)
		r_size = w_size;

	if (elants_i2c_read(client, buf, r_size) != r_size)
		return -EINVAL;

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };

	rc = elan_ktf_ts_poll(client);
	if (rc < 0)
		ETP_ERROR(" %s: Int poll failed!\n", __func__);

	rc = elants_i2c_read(client, buf_recv, sizeof(buf_recv));
	if (rc < 0)
		goto err_exit;

	ETP_INFO(" %s: hello packet %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0],
	       buf_recv[1], buf_recv[2], buf_recv[3], buf_recv[4], buf_recv[5], buf_recv[6],
	       buf_recv[7]);

	if (buf_recv[0] == 0x55 && buf_recv[1] == 0x55 && buf_recv[2] == 0x80
	    && buf_recv[3] == 0x80) {
		recovery = 0x80;
		return recovery;
	}

err_exit:
	return rc;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = { CMD_R_PKT, 0x00, 0x00, 0x01 };	/* Get Firmware Version */
	uint8_t cmd_id[] = { 0x53, 0xf0, 0x00, 0x01 };	/*Get firmware ID */
	uint8_t cmd_bc[] = { CMD_R_PKT, 0x10, 0x00, 0x01 };	/* Get BootCode Version */
	/* uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01}; Get BootCode Version  */
	uint8_t buf_recv[4] = {
		0};

	/* Firmware version */
	rc = elan_ktf_ts_get_data(client, cmd, buf_recv, 4, 4);
	if (rc < 0) {
		ETP_ERROR("Get Firmware version error\n");
		goto exit;
	}

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	fw_ver = ts->fw_ver;
	/* Firmware ID */
	rc = elan_ktf_ts_get_data(client, cmd_id, buf_recv, 4, 4);
	if (rc < 0) {
		ETP_ERROR("Get Firmware ID error\n");
		goto exit;
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	fw_id = ts->fw_id;

#ifdef SENSOR_OPTION
	ETP_INFO(" !! FW_ID = 0x%4.4x\n", fw_id);
	/*sensor option start */
	if (fw_id == FWID_TG_AUO)
		file_fw_data = file_fw_data_TG_AUO;
	else if (fw_id == FWID_TG_INX)
		file_fw_data = file_fw_data_TG_INX;
	else if (fw_id == FWID_HLT_AUO)
		file_fw_data = file_fw_data_HLT_AUO;
	else if (fw_id == FWID_HLT_INX)
		file_fw_data = file_fw_data_HLT_INX;
	else {
		ETP_ERROR("%s: Firmware ID dismatch, set FW_ID_Check = 0.\n", __func__);
		FW_ID_Check = 0;	/* ok = 1, fail = 0 */
	}
	/*sensor option end */
#endif

	/* Bootcode version */
	rc = elan_ktf_ts_get_data(client, cmd_bc, buf_recv, 4, 4);
	if (rc < 0) {
		ETP_ERROR("Get Bootcode version error\n");
		goto exit;
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->bc_ver = major << 8 | minor;
	bc_ver = ts->bc_ver;

	/*Get XY info */
	ts->x_resolution = x_res;
	ts->y_resolution = y_res;

	ETP_INFO("%s: Firmware version: 0x%4.4x\n", __func__, ts->fw_ver);
	ETP_INFO("%s: Firmware ID: 0x%4.4x\n", __func__, ts->fw_id);
	ETP_INFO("%s: Bootcode Version: 0x%4.4x\n", __func__, ts->bc_ver);
	ETP_INFO("%s: x resolution: %d, y resolution: %d\n", __func__, x_res,
	       y_res);
exit:
	return rc;
}

static inline int elan_ktf_pen_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y, uint16_t *p)
{
	*x = *y = *p = 0;

	*x = data[3];
	*x <<= 8;
	*x |= data[2];

	*y = data[5];
	*y <<= 8;
	*y |= data[4];

	*p = data[7];
	*p <<= 8;
	*p |= data[6];

	return 0;
}

static inline int elan_ktf_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf_ts_setup(struct i2c_client *client)
{
	int rc;
#ifdef SENSOR_OPTION
	uint8_t cmd_id[] = { 0x53, 0xf0, 0x00, 0x01 };	/*Get firmware ID */
	uint8_t buf_recv[4] = { 0 };
	int major, minor;
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
#endif

	rc = __hello_packet_handler(client);

	if (rc < 0)
		goto exit;

	if (rc != 0x80) {
		rc = __fw_packet_handler(client);
		if (rc < 0) {
			ETP_ERROR(" %s, fw_packet_handler fail, rc = %d", __func__, rc);
			goto exit;
		} else
			ETP_DEBUG(" %s: firmware checking done.\n", __func__);

		/* Check for fw_ver, if 0x0000 means FW update fail! */
		if (fw_ver == 0x00) {
			rc = 0x80;
			ETP_ERROR(" fw_ver = 0x%4.4x, last FW update fail\n", fw_ver);
		}
	}
#ifdef SENSOR_OPTION
	else {
		ETP_INFO(" %s: rc = 0x%x, Recovery Mode\n", __func__, rc);
		/* Ask Firmware ID */
		rc = elan_ktf_ts_get_data(client, cmd_id, buf_recv, 4, 4);
		if (rc < 0)
			ETP_ERROR("Get Firmware ID error\n");

		major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
		minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
		ts->fw_id = major << 8 | minor;
		fw_id = ts->fw_id;
		ETP_INFO("%s: in Recovery mode, get fw_id = 0x%4.4x\n", __func__,
		       fw_id);

		if (fw_id == FWID_TG_AUO)
			file_fw_data = file_fw_data_TG_AUO;
		else if (fw_id == FWID_TG_INX)
			file_fw_data = file_fw_data_TG_INX;
		else if (fw_id == FWID_HLT_AUO)
			file_fw_data = file_fw_data_HLT_AUO;
		else if (fw_id == FWID_HLT_INX)
			file_fw_data = file_fw_data_HLT_INX;
		else {
			ETP_ERROR("%s: Firmware ID dismatch, set FW_ID_Check = 0\n", __func__);
			FW_ID_Check = 0;	/* ok = 1, fail = 0 */
		}
	}
#endif

exit:
	return rc;
}

static int elan_ktf_ts_calibrate(struct i2c_client *client)
{

#ifdef ELAN_HID_I2C
	uint8_t flash_key[37] = {
		0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, CMD_W_PKT, 0xc0, 0xe1, 0x5a
	};
	uint8_t cal_cmd[37] = {
		0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, CMD_W_PKT, 0x29, 0x00, 0x01
	};

	ETP_INFO(" %s: Flash Key cmd\n", __func__);
	if ((i2c_master_send(client, flash_key, sizeof(flash_key))) != sizeof(flash_key)) {
		ETP_ERROR(" %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
	ETP_INFO(" %s: Calibration cmd: %02x, %02x, %02x, %02x\n", __func__,
		 cal_cmd[7], cal_cmd[8], cal_cmd[9], cal_cmd[10]);
	if ((i2c_master_send(client, cal_cmd, sizeof(cal_cmd))) != sizeof(cal_cmd)) {
		ETP_ERROR(" %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
#else
	uint8_t flash_key[] = { CMD_W_PKT, 0xC0, 0xE1, 0x5A };
	uint8_t cmd[] = { CMD_W_PKT, 0x29, 0x00, 0x01 };

	/* ETP_INFO(" %s: enter\n", __func__); */
	ETP_INFO(" %s: enter\n", __func__);


	ETP_INFO(" dump flash_key: %02x, %02x, %02x, %02x\n", flash_key[0],
		 flash_key[1], flash_key[2], flash_key[3]);
	if ((i2c_master_send(client, flash_key, sizeof(flash_key))) != sizeof(flash_key)) {
		ETP_ERROR(" %s: (flash_key) i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	ETP_INFO(" dump cal_cmd: %02x, %02x, %02x, %02x\n", cmd[0], cmd[1],
		 cmd[2], cmd[3]);
	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		ETP_ERROR(" %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
#endif
	return 0;
}

#ifdef ELAN_POWER_SOURCE
static unsigned now_usb_cable_status;


void touch_callback(unsigned cable_status)
{
	now_usb_cable_status = cable_status;
	/* update_power_source(); */
}
#endif

/*Coordination mapping*/
static void elan_tpd_calibrate_driver(uint16_t *x, uint16_t *y)
{
	uint16_t tx;

	/* A=2445.3, E=2497.5, B=C=D=F=0;
	when LCM_X=1920, LCM_Y=1200, X_res = 3216, y_res = 1968 */
	tx = ((tpd_def_calmat[0] * (*x)) +
		 (tpd_def_calmat[1] * (*y)) +
		 (tpd_def_calmat[2])) >> 12;
	*y = ((tpd_def_calmat[3] * (*x)) +
		 (tpd_def_calmat[4] * (*y)) +
		 (tpd_def_calmat[5])) >> 12;
	*x = tx;
}

#ifdef PROTOCOL_B
/* Protocol B  */
/* static int mTouchStatus[FINGER_NUM] = {0};   finger_num=10  */
void force_release_pos(struct i2c_client *client)
{
	/* struct elan_ktf_ts_data *ts = i2c_get_clientdata(client); */
	struct input_dev *idev = tpd->dev;
	int i;

	for (i = 0; i < FINGER_NUM; i++) {
		if (mTouchStatus[i] == 0)
			continue;
		/* input_mt_slot(ts->input_dev, i); */
		input_mt_slot(idev, i);
		/* input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0); */
		mTouchStatus[i] = 0;
		input_mt_report_slot_state(idev, MT_TOOL_FINGER, 0);
		mTouchStatus[i] = 0;
	}
	/* input_sync(ts->input_dev); */
	input_sync(idev);
}

static inline int elan_ktf_hid_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[6]);
	*x <<= 8;
	*x |= data[5];

	*y = (data[10]);
	*y <<= 8;
	*y |= data[9];

	return 0;
}


/* Protocol B  */
static void elan_ktf_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	/* struct elan_ktf_ts_data *ts = i2c_get_clientdata(client); */
	/* struct input_dev *idev = ts->input_dev; */
	struct input_dev *idev = tpd->dev;
	uint16_t x = 0, y = 0, touch_size, pressure_size;
	uint16_t fbits = 0;
	uint8_t i, num;
	uint16_t active = 0;
	uint8_t idx, btn_idx;
	int finger_num;
	int finger_id;
	int pen_hover = 0;
	int pen_down = 0;
	uint16_t p = 0;

	/* for 10 fingers */
	if (buf[0] == TEN_FINGERS_PKT) {
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx = 3;
		btn_idx = 33;
	}
	/* for 5 fingers  */
	else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)) {
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >> 3;
		idx = 2;
		btn_idx = 17;
	} else {
		/* for 2 fingers */
		finger_num = 2;
		num = buf[7] & 0x03;	/* for elan old 5A protocol the finger ID is 0x06 */
		fbits = buf[7] & 0x03;
		/* fbits = (buf[7] & 0x03) >> 1; // for elan old 5A protocol the finger ID is 0x06 */
		idx = 1;
		btn_idx = 7;
	}

	switch (buf[0]) {
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:
	case TEN_FINGERS_PKT:
		for (i = 0; i < finger_num; i++) {
			active = fbits & 0x1;
			if (active || mTouchStatus[i]) {
				/* input_mt_slot(ts->input_dev, i); */
				input_mt_slot(idev, i);
				/* input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active); */
				input_mt_report_slot_state(idev, MT_TOOL_FINGER, active);
				if (active) {
					elan_ktf_ts_parse_xy(&buf[idx], &x, &y);

					ETP_INFO(" before: Finger id=%d x=%d y=%d\n", i, x, y);
					elan_tpd_calibrate_driver(&x, &y);
					ETP_INFO(" after: Finger id=%d x=%d y=%d\n", i, x, y);

					touch_size = buf[FW_POS_WIDTH + i];
					pressure_size = buf[FW_POS_PRESSURE + i];

					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, touch_size);
					input_report_abs(idev, ABS_MT_PRESSURE, pressure_size);
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
					if (unlikely(gPrint_point))
						ETP_INFO(
							    " finger id=%d x=%d y=%d size=%d press=%d\n",
							    i, x, y, touch_size, pressure_size);
				}
			}
			mTouchStatus[i] = active;
			fbits = fbits >> 1;
			idx += 3;
		}
		if (num == 0) {
			/* ETP_INFO(" ALL Finger Up\n"); */
			input_report_key(idev, BTN_TOUCH, 0);	/* for all finger up */
			force_release_pos(client);
		}
		input_sync(idev);
		break;
	case PEN_PKT:

		pen_hover = buf[1] & 0x1;
		pen_down = buf[1] & 0x03;
		/* input_mt_slot(ts->input_dev, 0); */
		input_mt_slot(idev, 0);
		/* input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, pen_hover); */
		input_mt_report_slot_state(idev, MT_TOOL_FINGER, pen_hover);
		if (pen_hover) {
			elan_ktf_pen_parse_xy(&buf[0], &x, &y, &p);
			if (pen_down == 0x01) {	/* report hover function  */
				input_report_abs(idev, ABS_MT_PRESSURE, 0);
				input_report_abs(idev, ABS_MT_DISTANCE, 15);
				ETP_INFO("[elan pen] Hover DISTANCE=15\n");
			} else {
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 20);
				input_report_abs(idev, ABS_MT_PRESSURE, p);
				ETP_INFO("[elan pen] PEN PRESSURE=%d\n", p);
			}
			input_report_abs(idev, ABS_MT_POSITION_X, x);
			input_report_abs(idev, ABS_MT_POSITION_Y, y);
		}
		if (unlikely(gPrint_point)) {
			ETP_INFO("[elan pen] %x %x %x %x %x %x %x %x\n", buf[0],
				    buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
			ETP_INFO(" x=%d y=%d p=%d\n", x, y, p);
		}
		if (pen_down == 0) {
			/* ETP_INFO(" ALL Finger Up\n"); */
			input_report_key(idev, BTN_TOUCH, 0);	/* for all finger up */
			force_release_pos(client);
		}
		input_sync(idev);
		break;
	case ELAN_HID_PKT:
		finger_num = buf[62];
		if (finger_num > 5)
			finger_num = 5;	/* support 5 fingers    */
		idx = 3;
		num = 5;
		for (i = 0; i < finger_num; i++) {
			if ((buf[idx] & 0x03) == 0x00)
				active = 0;	/* 0x03: finger down, 0x00 finger up  */
			else
				active = 1;

			if ((buf[idx] & 0x03) == 0)
				num--;
			finger_id = (buf[idx] & 0xfc) >> 2;
			/* input_mt_slot(ts->input_dev, finger_id); */
			input_mt_slot(idev, finger_id);
			/* input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active); */
			input_mt_report_slot_state(idev, MT_TOOL_FINGER, active);
			if (active) {
				elan_ktf_hid_parse_xy(&buf[idx], &x, &y);
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(idev, ABS_MT_PRESSURE, 100);
				input_report_abs(idev, ABS_MT_POSITION_X, x);
				input_report_abs(idev, ABS_MT_POSITION_Y, y);
				ETP_INFO(
					    "[elan hid] i=%d finger_id=%d x=%d y=%d Finger NO.=%d\n",
					    i, finger_id, x, y, finger_num);
			}
			mTouchStatus[i] = active;
			idx += 11;
		}
		if (num == 0) {
			ETP_INFO(" Release ALL Finger\n");
			input_report_key(idev, BTN_TOUCH, 0);	/* for all finger up */
			force_release_pos(client);
		}
		input_sync(idev);
		break;
	case IamAlive_PKT:
		ETP_INFO("%x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
		break;
	case HELLO_PKT:
		ETP_INFO(" %s: Report Hello packet: %x %x %x %x\n", __func__, buf[0], buf[1],
		       buf[2], buf[3]);
		/* __fw_packet_handler(private_ts->client); */
		break;
	case CALIB_PKT:
		ETP_INFO(" %s: Report Calibration packet: %x %x %x %x\n", __func__, buf[0],
		       buf[1], buf[2], buf[3]);
		break;
	default:
		ETP_INFO(" %s: unknown packet type: %x %x %x %x\n", __func__, buf[0], buf[1],
		       buf[2], buf[3]);
		break;
	}			/* end switch */
}

#endif
/* end #ifdef PROTOCOL_B */

#ifdef PROTOCOL_A
/* Protocol A  */
static void elan_ktf_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	/* struct elan_ktf_ts_data *ts = i2c_get_clientdata(client); */
	/* struct input_dev *idev = ts->input_dev; */
	struct input_dev *idev = tpd->dev;
	uint16_t x, y, touch_size, pressure_size;
	uint16_t fbits = 0;
	uint8_t i, num, reported = 0;
	uint8_t idx, btn_idx;
	int finger_num;

	/* for 10 fingers   */
	if (buf[0] == TEN_FINGERS_PKT) {
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx = 3;
		btn_idx = 33;
	}
	/* for 5 fingers    */
	else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)) {
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >> 3;
		idx = 2;
		btn_idx = 17;
	} else {
		/* for 2 fingers */
		finger_num = 2;
		num = buf[7] & 0x03;	/* for elan old 5A protocol the finger ID is 0x06 */
		fbits = buf[7] & 0x03;
		/* fbits = (buf[7] & 0x03) >> 1; // for elan old 5A protocol the finger ID is 0x06 */
		idx = 1;
		btn_idx = 7;
	}

	switch (buf[0]) {
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:
	case TEN_FINGERS_PKT:
		if (num == 0) {
			input_report_key(idev, BTN_TOUCH, 0);
#ifdef ELAN_BUTTON
			if (buf[btn_idx] == 0x21) {
				button_state = 0x21;
				input_report_key(idev, KEY_BACK, 1);
				input_report_key(idev, KEY_BACK, 0);
				ETP_INFO("[elan_debug] button %x\n", buf[btn_idx]);
			} else if (buf[btn_idx] == 0x41) {
				button_state = 0x41;
				input_report_key(idev, KEY_HOME, 1);
			} else if (buf[btn_idx] == 0x81) {
				button_state = 0x81;
				input_report_key(idev, KEY_MENU, 1);
			} else if (button_state == 0x21) {
				button_state = 0;
				input_report_key(idev, KEY_BACK, 0);
			} else if (button_state == 0x41) {
				button_state = 0;
				input_report_key(idev, KEY_HOME, 0);
			} else if (button_state == 0x81) {
				button_state = 0;
				input_report_key(idev, KEY_MENU, 0);
			} else {
				ETP_DEBUG("no press\n");
				input_mt_sync(idev);

			}
#endif
		} else {
			input_report_key(idev, BTN_TOUCH, 1);
			for (i = 0; i < finger_num; i++) {
				if ((fbits & 0x01)) {
					elan_ktf_ts_parse_xy(&buf[idx], &x, &y);
					x = x_res - x;
					y = y_res - y;

					/* ETP_INFO(" before: Finger id=%d x=%d y=%d\n", i, x, y); */
					elan_tpd_calibrate_driver(&x, &y);
					/* ETP_INFO(" after: Finger id=%d x=%d y=%d\n", i, x, y); */

					touch_size = buf[FW_POS_WIDTH + i];
					pressure_size = buf[FW_POS_PRESSURE + i];

					input_report_abs(idev, ABS_MT_TRACKING_ID, i);
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, touch_size);
					input_report_abs(idev, ABS_MT_PRESSURE, pressure_size);
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
					input_mt_sync(idev);
					reported++;
					if (unlikely(gPrint_point))
						ETP_INFO(
							    " finger id=%d x=%d y=%d\n", i,
							    x, y);

					ETP_INFO(" Finger id=%d x=%d y=%d size=%d press=%d\n",
					       i, x, y, touch_size, pressure_size);

				}	/* end if finger status */
				fbits = fbits >> 1;
				idx += 3;

			}	/* end for */
		}
		if (reported)
			input_sync(idev);
		else {
			input_mt_sync(idev);
			input_sync(idev);
		}
		break;
	case IamAlive_PKT:
		ETP_INFO("%x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
		break;
	case HELLO_PKT:
		ETP_INFO(" %s: Report Hello packet: %x %x %x %x\n", __func__, buf[0], buf[1],
		       buf[2], buf[3]);
		/* __fw_packet_handler(private_ts->client); */
		break;
	case CALIB_PKT:
		ETP_INFO(" %s: Report Calibration packet: %x %x %x %x\n", __func__, buf[0],
		       buf[1], buf[2], buf[3]);
		break;
	default:
		ETP_INFO(" %s: unknown packet type: %x %x %x %x\n", __func__, buf[0], buf[1],
		       buf[2], buf[3]);
		break;
	}			/* end switch */
}
#endif
/* end #ifdef PROTOCOL_A */


#ifdef _ENABLE_DBG_LEVEL
static int ektf_proc_read(char *buffer, char **buffer_location, off_t offset, int buffer_length,
			  int *eof, void *data)
{
	int ret;

	ETP_DEBUG("call proc_read\n");

	if (offset > 0)		/* we have finished to read, return 0 */
		ret = 0;
	else
		ret = sprintf(buffer, "Debug Level: Release Date: %s\n", "2011/10/05");

	return ret;
}


static int ektf_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char procfs_buffer_size = 0;
	int i, ret = 0;
	unsigned char procfs_buf[PROC_FS_MAX_LEN + 1] = { 0 };
	unsigned int command;

	procfs_buffer_size = count;
	if (procfs_buffer_size > PROC_FS_MAX_LEN)
		procfs_buffer_size = PROC_FS_MAX_LEN + 1;

	if (copy_from_user(procfs_buf, buffer, procfs_buffer_size)) {
		ETP_ERROR(" proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	command = 0;
	for (i = 0; i < procfs_buffer_size - 1; i++) {
		if (procfs_buf[i] >= '0' && procfs_buf[i] <= '9')
			command |= (procfs_buf[i] - '0');
		else if (procfs_buf[i] >= 'A' && procfs_buf[i] <= 'F')
			command |= (procfs_buf[i] - 'A' + 10);
		else if (procfs_buf[i] >= 'a' && procfs_buf[i] <= 'f')
			command |= (procfs_buf[i] - 'a' + 10);

		if (i != procfs_buffer_size - 2)
			command <<= 4;
	}

	command = command & 0xFFFFFFFF;
	switch (command) {
	case 0xF1:
		gPrint_point = 1;
		break;
	case 0xF2:
		gPrint_point = 0;
		break;
	case 0xFF:
		ret = elan_ktf_ts_calibrate(private_ts->client);
		break;
	}
	ETP_INFO("Run command: 0x%08X  result:%d\n", command, ret);

	return count;		/* procfs_buffer_size; */
}
#endif				/* #ifdef _ENABLE_DBG_LEV */


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct elan_ktf_ts_data *elan_dev_data =
	    container_of(self, struct elan_ktf_ts_data, fb_notif);
	ETP_INFO(" %s fb notifier callback\n", __func__);
	if (evdata && evdata->data && elan_dev_data && private_ts->client) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK) {
				ETP_INFO(" resume\n");
				/* elan_ktf_ts_resume(private_ts->client); */
				/* tpd_resume(struct device *h); */
			} else if (*blank == FB_BLANK_POWERDOWN) {
				ETP_INFO(" suspend\n");
				/* elan_ktf_ts_suspend(private_ts->client); */
				/* tpd_suspend(struct device *h); */
			}
		}
	}

	return 0;
}
#endif

#if defined(ESD_CHECK)
static void elan_touch_esd_func(struct work_struct *work)
{

	ETP_INFO("[elan esd] %s: enter.......\n", __func__);	/* elan_dlx */

	if ((have_interrupts == 1) || (work_lock == 1))
		ETP_INFO("[elan esd] : had interrupt not need check\n");
	else
		/* Reset TP, if touch controller no any response  */
		elan_ktf_ts_hw_reset();

	have_interrupts = 0;
	queue_delayed_work(esd_wq, &esd_work, delay);
	ETP_INFO("[elan esd] %s: exit.......\n", __func__);	/* elan_dlx */
}
#endif

static int touch_event_handler(void *unused)
{
#ifdef ELAN_BUFFER_MODE
	uint8_t buf[4 + PACKET_SIZE] = { 0 };
	uint8_t buf1[PACKET_SIZE] = { 0 };
#else
	uint8_t buf[PACKET_SIZE] = { 0 };
#endif

	/* uint8_t buf[169] = { 0 }; */
	int rc;
	/* struct sched_param param = { .sched_priority = RTPM_PRIO_TPD }; */
	struct sched_param param = {.sched_priority = 4 };
	/* ETP_INFO(" -----touch_event_handler-----#1\n"); */
	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		/* disable_irq(CUST_EINT_TOUCH_PANEL_NUM); */

		/* start to recv data */
#ifdef ELAN_BUFFER_MODE
		rc = elants_i2c_read(private_ts->client, buf, 4 + PACKET_SIZE);
#else
		rc = elants_i2c_read(private_ts->client, buf, PACKET_SIZE);
#endif
		if (rc < 0) {
			ETP_ERROR(" Received the packet Error1.\n");
			/* return IRQ_HANDLED; */
		}
#ifndef ELAN_BUFFER_MODE
		elan_ktf_ts_report_data(private_ts->client, buf);
#else
		/* first package */
		if ((buf[0] == HELLO_PKT) || (buf[1] == CALIB_PKT))
			elan_ktf_ts_report_data(private_ts->client, buf);
		else
			elan_ktf_ts_report_data(private_ts->client, buf + 4);

		/* Second package */
		if (((buf[0] == 0x63) || (buf[0] == 0x66)) && ((buf[1] == 2) || (buf[1] == 3))) {
			rc = elants_i2c_read(private_ts->client, buf1, PACKET_SIZE);
			if (rc < 0)
				ETP_ERROR(" Received the packet Error2.\n");

			elan_ktf_ts_report_data(private_ts->client, buf1);
			/* Final package */
			if (buf[1] == 3) {
				rc = elants_i2c_read(private_ts->client, buf1, PACKET_SIZE);
				if (rc < 0)
					ETP_ERROR(" Received the packet Error3.\n");

				elan_ktf_ts_report_data(private_ts->client, buf1);
			}
		}
#endif
	} while (!kthread_should_stop());

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(void)
{
	/* ETP_INFO(" test int1......\n"); */
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int elan_ktf_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *node;
	int err = 0;
	/* struct elan_ktf_i2c_platform_data *pdata; */
	struct elan_ktf_ts_data *ts;
	int New_FW_ID;
	int New_FW_VER;


	ETP_INFO(" enter probe....\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ETP_ERROR("i2c_check_functionality error\n");
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		ETP_ERROR(" %s: allocate elan_ktf_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	mutex_init(&ktf_mutex);

	ts->client = client;
	ts->intr_gpio = GTP_INT_PORT;

	i2c_set_clientdata(client, ts);
	private_ts = ts;

	err = elants_i2c_power_on(ts);
	if (err) {
		ETP_ERROR(" i2c_power_on fail\n");
		goto err_alloc_data_failed;
	}

	ETP_INFO(" reset IC ...\n");
	elan_ktf_ts_hw_reset();


/* DMA setting ++++ */
#ifdef __MSG_DMA_MODE__
	ETP_INFO(" allocate DMA settings\n");
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	client->dev.dma_mask = &client->dev.coherent_dma_mask;
	/* DMA size 4096 for customer */
	g_dma_buff_va_elan = (u8 *) dma_alloc_coherent(&client->dev, 1024,
			(dma_addr_t *) (&g_dma_buff_pa_elan), GFP_KERNEL | GFP_DMA);
	if (!g_dma_buff_va_elan) {
		ETP_ERROR("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
		goto err_dma_buff;
	}
#endif
/* DMA setting ---- */

	err = elan_ktf_ts_setup(client);
	if (err < 0) {
		ETP_ERROR(" No Elan chip inside\n");
		goto err_dma_buff;
	}
	wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "elan-touchscreen");

	/*mtk setup INT */
	tpd_gpio_as_int(GTP_INT_PORT);
	msleep(200);

	/* mtk test request irq handler */
	node = of_find_matching_node(NULL, touch_of_match);
	if (node) {
		elan_tp_irq = irq_of_parse_and_map(node, 0);
		err = request_irq(elan_tp_irq, (irq_handler_t) tpd_eint_interrupt_handler,
				  IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
		if (err > 0) {
			ETP_ERROR(" tpd request_irq IRQ LINE NOT AVAILABLE!.");
			goto err_dma_buff;
		} else
			ETP_INFO(" request irq successful, irq number:%d\n", elan_tp_irq);
	} else {
		ETP_ERROR("[%s] tpd request_irq can not find touch eint device node!.",
		       __func__);
		goto err_dma_buff;
	}

	disable_irq(elan_tp_irq);

	elan_thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(elan_thread)) {
		err = PTR_ERR(elan_thread);
		ETP_ERROR(" failed to create kernel thread: %d\n", err);
		goto err_dma_buff;
	}
	enable_irq(elan_tp_irq);	/*test earlier enable irq */

	/* elan_ktf_ts_register_interrupt(ts->client); */
	ETP_INFO(" Start touchscreen %s in interrupt mode\n", tpd->dev->name);

	/* register sysfs */
	if (sysfs_create_group(&client->dev.kobj, &elan_attribute_group))
		ETP_ERROR("sysfs create group error\n");
	else
		ETP_INFO(" sysfs_create_group ok!!\n");

	/* Firmware Update */
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG | S_IRWXUGO;

	if (misc_register(&ts->firmware) < 0)
		ETP_ERROR(" misc_register failed!!\n");
	else
		ETP_INFO(" misc_register finished!!\n");
	/* End Firmware Update */


#ifdef IAP_PORTION
	ETP_INFO("IAP_PORTION:\n");
	work_lock = 1;
	disable_irq(elan_tp_irq);
	/* cancel_work_sync(&ts->work); */

	power_lock = 1;
	/* FW ID & FW VER *//*IC 3915su */
	ETP_INFO("[850a]=0x%02x, [850b]=0x%02x, [871a]=0x%02x, [871b]=0x%02x\n",
	       file_fw_data[34058], file_fw_data[34059], file_fw_data[34586],
	       file_fw_data[34587]);
	New_FW_ID = file_fw_data[34587] << 8 | file_fw_data[34586];
	New_FW_VER = file_fw_data[34059] << 8 | file_fw_data[34058];

	/* New_FW_ID = 0x0db0; //test */
	/* New_FW_VER = 0x1132; //test */
	ETP_INFO(" FW_ID=0x%x, New_FW_ID=0x%x\n", fw_id, New_FW_ID);
	ETP_INFO(" fw_ver=0x%x, New_FW_VER=0x%x\n", fw_ver, New_FW_VER);

#ifndef SENSOR_OPTION
	if (recovery == 0x80) {
		ETP_INFO(" In Recovery mode. Update Firmware right away.\n");
		fw_update_thread = kthread_run(Update_FW_One, NULL, "elan_update");
	} else if (New_FW_ID == fw_id) {
		if (New_FW_VER > fw_ver) {
			ETP_INFO
			    ("normal mode. There is a newer FW, Update Firmware right away.\n");
			fw_update_thread = kthread_run(Update_FW_One, NULL, "elan_update");
			/* Update_FW_One(0); */
		} else
			ETP_INFO(" No need to update fw.\n");
	} else {
		ETP_INFO(" fw_id is different!\n");
	}
#else
	ETP_INFO(" SENSOR_OPTION is enabled.\n");
	if (New_FW_ID == fw_id) {	/* for firmware auto-upgrade */
		if (recovery != 0x80) {	/* normal mode */
			ETP_INFO(" %s: Normal mode\n", __func__);
			if (New_FW_VER > fw_ver) {
				path_test = 1;
				/* Update_FW_One(1); */
				ETP_INFO
				    ("normal mode. There is a newer FW, Update Firmware right away.\n");
				fw_update_thread =
				    kthread_run(Update_FW_One, NULL, "elan_update");
			} else
				ETP_INFO(" No need to update fw.\n");
		} else {
			/* recovery mode, note that suppose fw_id is readable in recovery mode in 3915su */
			path_test = 2;
			ETP_INFO(" %s: Recovery mode, update fw directly.\n", __func__);
			/* Update_FW_One(); */
			fw_update_thread = kthread_run(Update_FW_One, NULL, "elan_update");
		}
	} else
		ETP_INFO(" No auto update, fw_id is different!\n");
#endif

	power_lock = 0;
	work_lock = 0;
	enable_irq(elan_tp_irq);
#endif


#ifdef _ENABLE_DBG_LEVEL
	dbgProcFile = create_proc_entry(PROC_FS_NAME, 0600, NULL);
	if (dbgProcFile == NULL) {
		remove_proc_entry(PROC_FS_NAME, NULL);
		ETP_INFO(" Could not initialize /proc/%s\n", PROC_FS_NAME);
	} else {
		dbgProcFile->read_proc = ektf_proc_read;
		dbgProcFile->write_proc = ektf_proc_write;
		ETP_INFO(" /proc/%s created\n", PROC_FS_NAME);
	}
#endif				/* #ifdef _ENABLE_DBG_LEVEL */

#if defined(CONFIG_FB)
	/* ETP_INFO(" CONFIG_FB is enabled!!!~~~~~~~~~~~~~~~~~\n"); */
	private_ts->fb_notif.notifier_call = fb_notifier_callback;
	fb_register_client(&private_ts->fb_notif);
#endif

#if defined(ESD_CHECK)		/* 0604 */
	INIT_DELAYED_WORK(&esd_work, elan_touch_esd_func);
	esd_wq = create_singlethread_workqueue("esd_wq");
	if (!esd_wq)
		ETP_INFO(" failed to create esd thread\n");
	queue_delayed_work(esd_wq, &esd_work, delay);
#endif
	tpd_load_status = 1;

	return 0;

err_dma_buff:
#ifdef __MSG_DMA_MODE__
	if (g_dma_buff_va_elan) {
		dma_free_coherent(NULL, 1024, g_dma_buff_va_elan, (dma_addr_t) g_dma_buff_pa_elan);
		g_dma_buff_va_elan = NULL;
		g_dma_buff_pa_elan = NULL;
		ETP_INFO("[DMA][release] Allocate DMA I2C Buffer release!\n");
	}
#endif

err_alloc_data_failed:
err_check_functionality_failed:

	return -1;
}

static int elan_ktf_ts_remove(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	/* unregister_early_suspend(&ts->early_suspend); */
	free_irq(elan_tp_irq, ts);

	input_unregister_device(ts->input_dev);
	kfree(ts);

/* DMA ++++ */
#ifdef __MSG_DMA_MODE__
	if (g_dma_buff_va_elan) {
		dma_free_coherent(NULL, 1024, g_dma_buff_va_elan, (dma_addr_t) g_dma_buff_pa_elan);
		g_dma_buff_va_elan = NULL;
		g_dma_buff_pa_elan = NULL;
		ETP_INFO("[DMA][release] Allocate DMA I2C Buffer release!\n");
	}
#endif
/* DMA---- */
	return 0;
}

#ifndef POWER_OFF_IN_SUSPEND
static int elan_ktf_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = { CMD_W_PKT, 0x50, 0x00, 0x01 };

	ETP_DEBUG(" %s: enter, state = %d\n", __func__, state);

	cmd[1] |= (state << 3);

	ETP_DEBUG(" dump cmd: %02x, %02x, %02x, %02x\n", cmd[0], cmd[1], cmd[2],
		cmd[3]);

	/* if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) { */
	if ((elants_i2c_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		ETP_ERROR(" %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = { CMD_R_PKT, 0x50, 0x00, 0x01 };
	uint8_t buf[4], power_state;

	rc = elan_ktf_ts_get_data(client, cmd, buf, 4, 4);
	if (rc)
		return rc;

	power_state = buf[1];
	ETP_DEBUG(" dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	ETP_DEBUG(" power state = %0x, %s\n", power_state,
		power_state == PWR_STATE_DEEP_SLEEP ? "Deep Sleep" : "Normal/Idle");

	return power_state;
}
#endif

/* static int elan_ktf_ts_suspend(struct i2c_client *client) should use MTK interface */
static void tpd_suspend(struct device *h)
{
	int rc = 0;

	ETP_INFO(" %s: enter\n", __func__);
	if (0 == power_lock) {
#ifndef POWER_OFF_IN_SUSPEND
		rc = elan_ktf_ts_set_power_state(private_ts->client, PWR_STATE_DEEP_SLEEP);
#else
	/*power off IC's VDD and GPIO */
	disable_irq(elan_tp_irq);
	rc = regulator_disable(tpd->reg);
	if (rc)
		ETP_ERROR("regulator disable fail!\n");
	else
		ETP_DEBUG("regulator disable success!\n");
#endif
	}
}

/* static int elan_ktf_ts_resume(struct i2c_client *client)      should use MTK interface */
static void tpd_resume(struct device *h)
{
	int rc = 0;
#ifndef POWER_OFF_IN_SUSPEND
	int retry = 3;
#endif

	ETP_INFO(" %s: enter\n", __func__);
	if (0 == power_lock) {
#ifdef POWER_OFF_IN_SUSPEND
		rc = elants_i2c_power_on(private_ts);
		if (rc)
			ETP_ERROR("power on fail!\n");
		enable_irq(elan_tp_irq);
		elan_ktf_ts_hw_reset();
#else
		do {
			rc = elan_ktf_ts_set_power_state(private_ts->client, PWR_STATE_NORMAL);
			mdelay(200);
			rc = elan_ktf_ts_get_power_state(private_ts->client);
			if (rc != PWR_STATE_NORMAL)
				ETP_ERROR("wake up tp failed! err = %d\n", rc);
			else {
				ETP_DEBUG("state back to Normal.\n");
				break;
			}
		} while (--retry);
#endif
	}
}



MODULE_DEVICE_TABLE(i2c, elan_ktf_ts_id);

static const struct of_device_id elan_of_match[] = {
	/* { .compatible = "elan,ekth3260" }, */
	{.compatible = "elan_ktf"},
	{ /* sentinel */ }
};


static const struct i2c_device_id elants_i2c_id[] = {
	{DEVICE_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, elants_i2c_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id elants_acpi_id[] = {
	{"ELAN0001", 0},
	{}
};

MODULE_DEVICE_TABLE(acpi, elants_acpi_id);
#endif

#ifdef CONFIG_OF
static const struct of_device_id elants_of_match[] = {
	{.compatible = "mediatek,elan_touch"},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, elants_of_match);
#endif


/* try this */
static struct i2c_driver tpd_i2c_driver = {
	.probe = elan_ktf_ts_probe,
	.remove = elan_ktf_ts_remove,
	.id_table = elants_i2c_id,
	.driver = {
		   .name = DEVICE_NAME,
		   .owner = THIS_MODULE,
		   /* .acpi_match_table = ACPI_PTR(elants_acpi_id), */
		   .of_match_table = of_match_ptr(elants_of_match),
		   /* .async_probe = true, */
		   },
};

/************************************************************************
* Name: tpd_local_init
* Brief: add driver info
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
static int tpd_local_init(void)
{
	int retval = 0;

	ETP_INFO("ELAN touch I2C Touchscreen Driver\n");
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	if (retval != 0) {
		ETP_ERROR("Failed to set reg-vgp6 voltage: %d\n", retval);
		return -1;
	}
	ETP_INFO(" %s: OK to set reg-vgp6 voltage: %d\n", __func__, retval);
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		ETP_ERROR("fts unable to add i2c driver.\n");
		return -1;
	}
	if (tpd_load_status == 0) {
		ETP_ERROR(" add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	ETP_INFO(" %s: add i2c driver ok.\n", __func__);

#ifdef MT_PROTOCOL_B
#else
	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (10 - 1), 0, 0);
#endif


#ifdef TPD_HAVE_BUTTON
	/* initialize tpd button data */
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
	ETP_INFO(" have button = 1\n");
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

	ETP_INFO("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;
	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "elants",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

static int __init tpd_driver_init(void)
{
	ETP_INFO("MediaTek elan touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		ETP_ERROR(" add generic driver failed");
	return 0;
}

static void __exit tpd_driver_exit(void)
{
	ETP_INFO(" MediaTek elan touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);	/*for MTK interface */
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");
