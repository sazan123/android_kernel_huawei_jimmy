/*
 * goodix_ts.c - Main touch driver file of Goodix
 *
 * Copyright (C) 2015 - 2016 Goodix Technology Incorporated
 * Copyright (C) 2015 - 2016 Yulong Cai <caiyulong@goodix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "goodix_ts.h"
#include "goodix_dts.h"
#include <linux/kthread.h>
#include "gt1x_config.h"
#include "huawei_ts_kit.h"
#ifdef CONFIG_APP_INFO
#include <misc/app_info.h>
#endif
#ifdef CONFIG_HUAWEI_HW_I2C_DCT
#include <linux/hw_dev_dec.h>
#endif

#include <ontim/ontim_dev_dgb.h>
static char g_gtp_version[]="GT917D ver 1.0";
static char g_gtp_vendor[50]="trully-gt917d";
DEV_ATTR_DECLARE(touch_screen)
DEV_ATTR_DEFINE("version",g_gtp_version)
DEV_ATTR_DEFINE("vendor",g_gtp_vendor)
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(touch_screen,touch_screen,8);

struct goodix_ts_data *goodix_ts;
struct ts_kit_device_data *g_goodix_dev_data = NULL;
static struct mutex wrong_touch_lock;
int power_flag = 0;
/*Gesture register(0xd0) value*/
#define DOUBLE_CLICK_WAKEUP  			(0xcc)
#define SPECIFIC_LETTER_W  			('w')
#define SPECIFIC_LETTER_M			('m')
#define SPECIFIC_LETTER_E			('e')
#define SPECIFIC_LETTER_C 			('c')
#define LETTER_LOCUS_NUM 6
#define LINEAR_LOCUS_NUM 2
#define IS_APP_ENABLE_GESTURE(x)  		((u32)(1<<x))
bool update_from_sd = false;
/**
 * goodix_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 * Return: 0: success, otherwise: failed
 */
DECLARE_WORK(goodix_chip_reset_work,goodix_chip_reset);
DECLARE_WORK(goodix_chip_sleep_mode_work,goodix_sleep_mode_out);
DECLARE_WORK(goodix_chip_put_device_work,goodix_put_device_outof_easy_wakeup);
static int goodix_i2c_test(void);
u8 gt1x_config[GTP_CONFIG_MAX_LENGTH] = { 0 };

u32 gt1x_cfg_length = GTP_CONFIG_MAX_LENGTH;
s32 _do_i2c_write(struct i2c_msg *msg, u16 addr, u8 *buffer, s32 len)
{
	s32 ret = -1;
	s32 pos = 0;
	s32 data_length = len;
	s32 transfer_length = 0;
	u8 *data = NULL;
	u16 address = addr;

	data =
	    kmalloc(IIC_MAX_TRANSFER_SIZE <
			   (len + GTP_ADDR_LENGTH) ? IIC_MAX_TRANSFER_SIZE : (len + GTP_ADDR_LENGTH), GFP_KERNEL);
	if (data == NULL)
		return -ENODEV;

	msg->buf = data;

	while (pos != data_length) {
		if ((data_length - pos) > (IIC_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH))
			transfer_length = IIC_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH;
		else
			transfer_length = data_length - pos;

		msg->buf[0] = (address >> 8) & 0xFF;
		msg->buf[1] = address & 0xFF;
		msg->len = transfer_length + GTP_ADDR_LENGTH;
		memcpy(&msg->buf[GTP_ADDR_LENGTH], &buffer[pos], transfer_length);

		ret = i2c_transfer(goodix_ts->goodix_device_data->ts_platform_data->client->adapter, msg, 1);
		if (ret != 1) {
			GTP_INFO("I2c Transfer error! (%d)", ret);
			kfree(data);
			return -ENODEV;
		}
		pos += transfer_length;
		address += transfer_length;
	}

	kfree(data);
	return 0;
}
static int i2c_write_mtk(u16 addr, u8 *buffer, s32 len)
{
	s32 ret;

	struct i2c_msg msg = {
		.flags = 0,
#ifdef CONFIG_MTK_I2C_EXTENSION
		.addr = (goodix_ts->goodix_device_data->ts_platform_data->client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG),	/*remain*/
		.timing = I2C_MASTER_CLOCK,
#else
		.addr = goodix_ts->goodix_device_data->ts_platform_data->client->addr,  /*remain*/
#endif
	};

	ret = _do_i2c_write(&msg, addr, buffer, len);
	return ret;
}


int goodix_i2c_write(u16 addr, u8 * buffer, u16 len)
{
	int ret;
	u8 addr_buf[2] = { (addr >> 8) & 0xFF, addr & 0xFF };
	struct ts_kit_device_data *dev_data = goodix_ts->goodix_device_data;
	if ( !dev_data->ts_platform_data->bops->bus_write)
		return -ENODEV;
	ret = dev_data->ts_platform_data->bops->bus_write(addr_buf,2, buffer,len,3);
	if (ret < 0)
		GTP_ERROR("xlj i2c write error,addr:%04x bytes:%d ret:%d", addr, len,ret);

	return ret;
}

/**
 * goodix_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 * Return: 0: success, otherwise: failed
 */

int goodix_i2c_read(u16 addr, u8 * buffer, u16 len)
{
	struct ts_kit_device_data *dev_data = goodix_ts->goodix_device_data;
	int ret;
	u8 addr_buf[2] = { (addr >> 8) & 0xFF, addr & 0xFF };
	if ( !dev_data->ts_platform_data->bops->bus_read)
		return -ENODEV;

	//addr = cpu_to_be16(addr);
	ret = dev_data->ts_platform_data->bops->bus_read(addr_buf, 2, buffer, len,3);
	if (ret < 0)
		GTP_ERROR("i2c read error,addr:%04x bytes:%d", addr, len);

	return ret;
}

/**
 * goodix_i2c_read_dbl_check - read twice and double check
 * @addr: register address
 * @buffer: data buffer
 * @len: bytes to read
 * Return    <0: i2c error, 0: ok, 1:fail
 */
int goodix_i2c_read_dbl_check(u16 addr, u8 * buffer, u16 len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	int ret;

	if (len > 16) {
		GTP_ERROR("i2c_read_dbl_check length %d is too long, exceed %zu",
			len, sizeof(buf));
		return -EINVAL;
	}

	memset(buf, 0xAA, sizeof(buf));
	ret = goodix_i2c_read(addr, buf, len);
	if (ret < 0)
		return ret;

	msleep(5);
	memset(confirm_buf, 0, sizeof(confirm_buf));
	ret = goodix_i2c_read(addr, confirm_buf, len);
	if (ret < 0)
		return ret;

	if (!memcmp(buf, confirm_buf, len)) {
		memcpy(buffer, confirm_buf, len);
		return 0;
	}

	GTP_ERROR("i2c read 0x%04X, %d bytes, double check failed!", addr, len);
	return 1;
}

/**
 * goodix_send_cfg - Send config data to hw
 * @cfg_ptr: pointer to config data structure
 * Return 0--success,non-0--fail.
 */
int goodix_send_cfg(struct goodix_ts_config *cfg_ptr)
{
	//static DEFINE_MUTEX(mutex_cfg);
	u8 *config;
	int i, cfg_len;
	s32 ret = 0, retry = 0;
	u16 checksum = 0;
	if (!cfg_ptr || !cfg_ptr->initialized) {
		GTP_ERROR("Invalid config data");
		return -EINVAL;
	}

	config = &cfg_ptr->data[0];
	cfg_len = cfg_ptr->size;

	mutex_lock(&goodix_ts->mutex_cfg);
	GTP_INFO("Send %s,ver:%02x size:%d", cfg_ptr->name,
			config[0], cfg_len);

	if (cfg_len != GTP_CONFIG_ORG_LENGTH
		&& cfg_len != GTP_CONFIG_ORG_LENGTH + GTP_CONFIG_EXT_LENGTH) {
		GTP_ERROR("Invalid config size:%d", cfg_len);
		mutex_unlock(&goodix_ts->mutex_cfg);
		return -1;
	}

	for (i = 0, checksum = 0; i < cfg_len ; i ++)
		checksum += config[i] ;
	if (!checksum) {
		GTP_ERROR("Invalid config,all of the bytes is zero");
		mutex_unlock(&goodix_ts->mutex_cfg);
		return -1;
	}
	config[cfg_len] = (~checksum) + 1;
	retry = 0;
	while (retry++ < 3) {
		ret = goodix_i2c_write(GTP_REG_CONFIG_DATA, config, cfg_len);
		if (!ret) {
			if (cfg_ptr->delay_ms > 0)
				msleep(cfg_ptr->delay_ms);
			mutex_unlock(&goodix_ts->mutex_cfg);
			GTP_INFO("Send config successfully");
			return 0;
		}
	}

	GTP_ERROR("Send config failed");
	mutex_unlock(&goodix_ts->mutex_cfg);
	return ret;
}
int goodix_parse_cfg_data(struct goodix_ts_data *ts,
				char *cfg_type, u8 *cfg, int *cfg_len, u8 sid)
{
	struct device_node *self = ts->goodix_device_data->ts_platform_data->node;
	int correct_len;
	char project_id[20];
	struct property *prop;
	TS_LOG_ERR("goodix_parse_cfg_data");
	sprintf(project_id,  "gtp-%s", "DLI45210");
	self = of_find_compatible_node(ts->pdev->dev.of_node,
				NULL, project_id);
	if (!self) {
		TS_LOG_ERR("No chip specific dts:%s, need to parse", project_id);
		return -EINVAL;
	}

	TS_LOG_INFO("Parse [%s] data from dts[SENSORID%u]", cfg_type, sid);

	prop = of_find_property(self, cfg_type, cfg_len);
	if (!prop || !prop->value || *cfg_len == 0)
	{
		TS_LOG_ERR("xljadd get %s cfg_len = %d ",cfg_type,*cfg_len);
		return -EINVAL;/* fail */
	}
	memcpy(cfg, prop->value, *cfg_len);
	correct_len = GTP_CONFIG_ORG_LENGTH;
	if (*cfg_len != correct_len) {
		TS_LOG_ERR("Invalid config size:%d", *cfg_len);
		return -EINVAL;
	}

	return 0;
}
/**
 * goodix_send_cmd - seng cmd
 * must write data & checksum first
 * byte    content
 * 0       cmd
 * 1       data
 * 2       checksum
 * Returns 0 - succeed,non-0 - failed
 */
static int goodix_send_cmd(u16 addr, u8 cmd)
{
	s32 ret;
	u8 buffer[3] = { cmd, 0, 0 };
	TS_LOG_INFO("Send command:%u", cmd);
	mutex_lock(&goodix_ts->mutex_cmd);
	buffer[0] = cmd;
	ret = goodix_i2c_write(addr, &buffer[0], 1);
	mutex_unlock(&goodix_ts->mutex_cmd);

	return ret;
}
/**
 * goodix_init_watchdog - esd mechannism
 *
 * Returns  0--success,non-0--fail.
 */
static inline int goodix_init_watchdog(void)
{
	/* 0x8040 ~ 0x8043 */
	u8 value[] = {0xAA};

	GTP_DEBUG("Init watchdog");
	return goodix_i2c_write(GTP_REG_CMD + 1, &value[0], 4);
}


/**
 * goodix_switch_config - Switch config data.
 * @cfg_type: GOODIX_NORMAL_CFG - normal config data
 *			  GOODIX_GLOVE_CFG - glove config data
 *			  GOODIX_HOLSTER_CFG - holster config data
 * Returns  0--success,non-0--fail.
 */
static int goodix_switch_config(int cfg_type)
{
	struct goodix_ts_data *ts = goodix_ts;
	struct goodix_ts_config *config;
	int ret;

	if (!ts)
		return -EINVAL;

	switch (cfg_type) {
	case GOODIX_NORMAL_CFG:
		config = &ts->normal_config;
		break;
	case GOODIX_GLOVE_CFG:
		config = &ts->glove_config;
		break;
	case GOODIX_HOLSTER_CFG:
		config = &ts->holster_config;
		break;
	default:
		return -EINVAL;
	}

	ret = goodix_send_cfg(config);
	return ret;
}

#ifdef ROI
/**
 * goodix_ts_roi_init - initialize roi feature
 * @roi: roi data structure
 * return 0 OK, < 0 fail
 */
static int goodix_ts_roi_init(struct goodix_ts_roi *roi)
{
	unsigned int roi_bytes;

	if (!roi)
		return -EINVAL;

	if (!roi->roi_rows || !roi->roi_cols) {
		GTP_ERROR("Invalid roi config,rows:%d,cols:%d",
				roi->roi_rows, roi->roi_cols);
		return -EINVAL;
	}

	mutex_init(&roi->mutex);

	roi_bytes = (roi->roi_rows + roi->roi_cols) * 2;
	roi->rawdata = kmalloc(roi_bytes + ROI_HEAD_LEN, GFP_KERNEL);
	if (!roi->rawdata) {
		GTP_ERROR("Failed to alloc memory for roi");
		return -ENOMEM;
	}

	GTP_INFO("ROI init,rows:%d,cols:%d",
				roi->roi_rows, roi->roi_cols);

	return 0;
}

/**
 * goodix_cache_roidata - caching roi data
 * @roi: roi data structure
 * return 0 ok, < 0 fail
 */
static int goodix_cache_roidata(struct goodix_ts_roi *roi)
{
	unsigned roi_bytes,i;
	unsigned char status[ROI_HEAD_LEN];
	u16 checksum = 0;
	int ret;

	if ( !roi->enabled)
		return -EINVAL;
	ret = goodix_i2c_read(ROI_STA_REG, status, ROI_HEAD_LEN);
	if (ret < 0)
		return ret;

	for (i = 0; i < ROI_HEAD_LEN; i++)
		checksum += status[i];

	if ((u8)checksum != 0) { /* cast to 8bit checksum,*/
		GTP_ERROR("roi status checksum error");
		return -1;
	}

	if (status[0] & ROI_READY_MASK) /* roi data ready */
		roi->track_id = status[0] & ROI_TRACKID_MASK;
	else
		return -1; /* not ready */

	mutex_lock(&roi->mutex);
	roi->data_ready = false;
	roi_bytes = (roi->roi_rows * roi->roi_cols + 1) * 2;

	ret = goodix_i2c_read(ROI_DATA_REG,
				(u8 *)(roi->rawdata + ROI_HEAD_LEN), roi_bytes);
	if (ret < 0) {
		mutex_unlock(&roi->mutex);
		return ret;
	}

	for (i = ROI_HEAD_LEN, checksum = 0;
					i < roi_bytes / 2 + ROI_HEAD_LEN; i++) {
		/* 16bits */
		roi->rawdata[i] = be16_to_cpu(roi->rawdata[i]);
		checksum += roi->rawdata[i];
	}
	memcpy(&roi->rawdata[0], &status[0], ROI_HEAD_LEN);

	if (checksum != 0)
		GTP_ERROR("roi data checksum error");
	else
		roi->data_ready = true;

	mutex_unlock(&roi->mutex);

	status[0] = 0x00;
	ret = goodix_i2c_write(ROI_STA_REG, status, 1);

	return ret;
}
#endif
static int easy_wakeup_gesture_report_coordinate(
						 unsigned int
						 reprot_gesture_point_num,
						 struct ts_fingers *info)
{
	int retval = 0;
	u16 x = 0;
	u16 y = 0;
	int i = 0;
	u8 buf[64*4];
	u16 coordinate_x[10] = {0};
	u16 coordinate_y[10] = {0};
	u8 point_num;
	struct goodix_coordinate point[5];
	u16 top_x = 0;
	u16 top_y = 0;
	u16 bottom_x = 0;
	u16 bottom_y = 0;
	u16 left_x = 0;
	u16 left_y = 0;
	u16 right_x =0;
	u16 right_y =0;

	if (reprot_gesture_point_num != 0)
      {
		retval = goodix_i2c_read(0x8140, buf, 45);
		if(retval <0){
			TS_LOG_ERR("%s read gesture coordinate failed \n",__func__);
			return retval;
		}

		/*
		 *The most points num is 6,point from 1(lower address) to 6(higher address) means:
		 *1.beginning 2.end 3.top 4.leftmost 5.bottom 6.rightmost
		 */
		point_num = buf[1];
		TS_LOG_INFO("%s: point_num = %d\n", __func__, point_num);

		if(reprot_gesture_point_num == 2){
			TS_LOG_INFO("%s: Gesture Dobule Click \n", __func__);
			/*1.beginning 2.end */
			for (i = 0; i < 2; i++) {
					x =  (buf[14 + i*4] << 8) | (buf[13 + i*4] & 0xFF);
					y = (buf[16 + i*4]<<8 )| (buf[15 + i*4] & 0xFF);
					coordinate_x[i] = x;
					coordinate_y[i] = y;
					g_goodix_dev_data->easy_wakeup_info.easywake_position[i] = x << 16 | y;
					TS_LOG_DEBUG("%s: Gesture Repot Point %d, x = %d, y = %d\n", __func__, i, x, y);
					TS_LOG_DEBUG("easywake_position[%d] = 0x%08x\n", i,
						    g_goodix_dev_data->easy_wakeup_info.easywake_position[i]);
					TS_LOG_DEBUG("%s: Gesture Repot Point %d, coordinate_x = %d, coordinate_y = %d\n",
						__func__, i, coordinate_x[i], coordinate_y[i]);
			}
			return retval;
		}
/* fill gesture coordinate */
		else{
			retval = goodix_i2c_read(0xC0EA, buf, 64*4);
			if(retval <0){
				TS_LOG_ERR("%s read gesture coordinate failed \n",__func__);
				return retval;
			}

	            x =  ((buf[1] << 8) | (buf[0] & 0xFF));
	            y =  ((buf[3] << 8) | (buf[2] & 0xFF));
	            top_y = y;
	            bottom_y = y;
	            left_x = x;
	            right_x =x;

	            for (i = 0; i < 64; i++) {
			     x =  ( (buf[1 + (4 * i)] << 8) | (buf[0 + (4 * i)]& 0xFF));
			     y = ( (buf[3 + (4 * i)] <<8) | (buf[2 + (4 * i)] & 0xFF));

	                    if(x ==0 && y==0) {
					break;
	                    }

	                    if(top_y > y){
	                            top_y = y;
	                            top_x = x;
	                    }
	                    if(bottom_y < y){
	                            bottom_y = y;
	                            bottom_x = x;
	                    }
	                    if(left_x > x){
	                            left_x = x;
	                            left_y = y;
	                    }
	                    if(right_x < x){
	                            right_x = x;
	                            right_y = y;
	                    }
	                    TS_LOG_DEBUG("%s: [0xC0EA] Gesture Repot Point %d, x = %d, y = %d\n", __func__, i, x, y);
	            }

	            /*1.begin */
		     x =  ( (buf[1] << 8) | (buf[0]& 0xFF));
		     y = ( (buf[3] <<8) | (buf[2] & 0xFF));
		     g_goodix_dev_data->easy_wakeup_info.easywake_position[0] = x << 16 | y;
	                    TS_LOG_INFO("top = 0x%08x,  begin_x= %d , begin_y= %d \n",
					g_goodix_dev_data->easy_wakeup_info.easywake_position[0], x, y);

	            /*2.end */
		     x =  ( (buf[1 + (4 * (i-1))] << 8) | (buf[0 + (4 * (i-1))]& 0xFF));
		     y = ( (buf[3 + (4 * (i-1))] <<8) | (buf[2 + (4 * (i-1))] & 0xFF));
		     g_goodix_dev_data->easy_wakeup_info.easywake_position[1] = x << 16 | y;
	                    TS_LOG_INFO("top = 0x%08x,  end_x= %d , end_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[1], x, y);

	            /*3.top */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[2] = top_x << 16 | top_y;
	                    TS_LOG_INFO("top = 0x%08x,  top_x= %d , top_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[2], top_x, top_y);
	            /*4.leftmost */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[3] = left_x << 16 | left_y;
	                    TS_LOG_INFO("leftmost = 0x%08x,  left_x= %d , left_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[3], left_x, left_y);
	            /*5.bottom */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[4] = bottom_x << 16 | bottom_y;
	                    TS_LOG_INFO("bottom = 0x%08x,  bottom_x= %d , bottom_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[4], bottom_x, bottom_y);
	            /*6.rightmost */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[5] = right_x << 16 | right_y;
	                    TS_LOG_INFO("rightmost = 0x%08x,  right_x= %d , right_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[5], right_x, right_y);
		}

	}
	return retval;
}
/**
 * goodix_request_event_handler - firmware request
 * Return    <0: failed, 0: succeed
 */
static int goodix_check_key_gesture_report(
					     struct ts_fingers *info,
					     struct ts_easy_wakeup_info
					     *gesture_report_info,
					     unsigned char
					     get_gesture_wakeup_data)
{
	int retval = 0;
	unsigned int reprot_gesture_key_value = 0;
	unsigned int reprot_gesture_point_num = 0;

	TS_LOG_DEBUG("get_gesture_wakeup_data is %d \n",
		    get_gesture_wakeup_data);

	switch (get_gesture_wakeup_data) {
		case DOUBLE_CLICK_WAKEUP:
			if (IS_APP_ENABLE_GESTURE(GESTURE_DOUBLE_CLICK) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG("@@@DOUBLE_CLICK_WAKEUP detected!@@@\n");
				reprot_gesture_key_value = TS_DOUBLE_CLICK;
				//LOG_JANK_D(JLID_TP_GESTURE_KEY, "JL_TP_GESTURE_KEY");
				reprot_gesture_point_num = LINEAR_LOCUS_NUM;
		}
		break;
		case SPECIFIC_LETTER_C:
			if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_c) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG
				    ("@@@SPECIFIC_LETTER_c detected!@@@\n");
				reprot_gesture_key_value = TS_LETTER_c;
				reprot_gesture_point_num = LETTER_LOCUS_NUM;
			}
			break;
		case SPECIFIC_LETTER_E:
			if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_e) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG
				    ("@@@SPECIFIC_LETTER_e detected!@@@\n");
				reprot_gesture_key_value = TS_LETTER_e;
				reprot_gesture_point_num = LETTER_LOCUS_NUM;
			}
			break;
		case SPECIFIC_LETTER_M:
			if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_m) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG
				    ("@@@SPECIFIC_LETTER_m detected!@@@\n");
				reprot_gesture_key_value = TS_LETTER_m;
				reprot_gesture_point_num = LETTER_LOCUS_NUM;
			}
			break;
		case SPECIFIC_LETTER_W:
			if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_w) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG
				    ("@@@SPECIFIC_LETTER_w detected!@@@\n");
				reprot_gesture_key_value = TS_LETTER_w;
				reprot_gesture_point_num = LETTER_LOCUS_NUM;
			}
		break;
	default:
		TS_LOG_INFO("@@@unknow gesture detected!\n");
		return 1;
	}

	if (0 != reprot_gesture_key_value) {
		/*increase wake_lock time to avoid system suspend.*/
		wake_lock_timeout(&g_goodix_dev_data->ts_platform_data->ts_wake_lock, 5 * HZ);
		mutex_lock(&wrong_touch_lock);

		if (true == g_goodix_dev_data->easy_wakeup_info.off_motion_on) {
			//g_goodix_dev_data->easy_wakeup_info.off_motion_on = false;
			retval = easy_wakeup_gesture_report_coordinate(
								  reprot_gesture_point_num,
								  info);
			if (retval < 0) {
				mutex_unlock(&wrong_touch_lock);
				TS_LOG_ERR
				    ("%s: report line_coordinate error!retval = %d\n",
				     __func__, retval);
				return retval;
			}

			info->gesture_wakeup_value = reprot_gesture_key_value;
			TS_LOG_DEBUG
			    ("%s: info->gesture_wakeup_value = %d\n",
			     __func__, info->gesture_wakeup_value);
		}
		mutex_unlock(&wrong_touch_lock);
	}
	return NO_ERR;
}
static int goodix_request_event_handler(struct goodix_ts_data *ts)
{
	u8 rqst_data = 0;
	int ret;

	ret = goodix_i2c_read(GTP_REG_RQST, &rqst_data, 1);
	if (ret)
		return ret;

	GTP_DEBUG("Request state:0x%02x", rqst_data);
	switch (rqst_data & 0x0F) {
	case GTP_RQST_CONFIG:
		GTP_INFO("Request Config.");
		ret = goodix_send_cfg(&ts->normal_config);
		if (ret) {
			GTP_ERROR("Send config error");
		} else {
			GTP_INFO("Send config success");
			rqst_data = GTP_RQST_RESPONDED;
			goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		}
		break;
	case GTP_RQST_RESET:
		GTP_INFO("Request Reset.");
		goodix_chip_reset();
		rqst_data = GTP_RQST_RESPONDED;
		goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		break;
	default:
		break;
	}
	return 0;
}
static int goodix_feature_resume(struct goodix_ts_data *ts)
{
	int ret = 0;
	/*struct goodix_ts_config *config = NULL;

	if (ts->noise_env)
		config = &ts->normal_noise_config;
	else
		config = &ts->normal_config;*/
	//ret = goodix_send_cfg(config);
	ret = gt1x_init_panel();
	TS_LOG_INFO("goodix_send_cfg: %d", ret);

#if 0
	if (info->charger_info.charger_switch) {
		ret = goodix_send_cmd(GTP_CMD_CHARGER_ON, 0x00);
		TS_LOG_INFO("Charger switch on");
	}
#endif


	return ret;
}
int goodix_sleep_mode_out(void)
{
	struct goodix_ts_data *ts = goodix_ts;
	//gpio_direction_output(ts->ts_platform_data->irq_gpio, 1);
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
	msleep(6);

	disable_irq_nosync(ts->ts_platform_data->irq_id);
	goodix_chip_reset();
	enable_irq(ts->ts_platform_data->irq_id);

	msleep(2);
	goodix_feature_resume(ts);
	//enable_irq(ts->ts_platform_data->irq_id);
    int error = NO_ERR;
    error = ts_kit_power_control_notify(TS_AFTER_RESUME, NO_SYNC_TIMEOUT); 
    if (error)
    { TS_LOG_ERR("ts after resume err\n");}
	return 0;
}

int goodix_put_device_outof_easy_wakeup(void)
{
	int retval;
	struct goodix_ts_data *ts = goodix_ts;
	struct ts_easy_wakeup_info *info = &g_goodix_dev_data->easy_wakeup_info;

	TS_LOG_DEBUG
	    ("goodix_put_device_outof_easy_wakeup  info->easy_wakeup_flag =%d\n",
	     info->easy_wakeup_flag);

	if (false == info->easy_wakeup_flag) {
		return;
	}

	/*Wakeup Gesture Only bit(01) set 0 */
	//disable_irq_nosync(ts->ts_platform_data->irq_id);
	goodix_chip_reset();
	//enable_irq(ts->ts_platform_data->irq_id);

	retval = goodix_i2c_test();
	if (retval < 0)
	{
		TS_LOG_ERR("%s: goodix_i2c_test fail \n",__func__);
	}

	msleep(5);
	//goodix_chip_reset(20);
	//goodix_feature_resume(ts);
	//enable_irq(ts->ts_platform_data->irq_id);
	info->easy_wakeup_flag = false;
	g_goodix_dev_data->easy_wakeup_info.off_motion_on = false;

	msleep(2);
	goodix_feature_resume(ts);
    int error = NO_ERR;
    error = ts_kit_power_control_notify(TS_AFTER_RESUME, NO_SYNC_TIMEOUT);
    if (error)
    { TS_LOG_ERR("ts after resume err\n"); }
	return 0;
}
static int goodix_chip_resume(void)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return -ENODEV;

	TS_LOG_INFO("Resume start");
	switch (g_goodix_dev_data->easy_wakeup_info.sleep_mode) {
	case TS_POWER_OFF_MODE:
		//goodix_sleep_mode_out();	/*exit sleep mode*/
        schedule_work(&goodix_chip_sleep_mode_work);
		break;
	case TS_GESTURE_MODE:
		//goodix_put_device_outof_easy_wakeup();
         schedule_work(&goodix_chip_put_device_work);
		break;
	default:
		//goodix_sleep_mode_out();
		schedule_work(&goodix_chip_sleep_mode_work);

		break;
	}

	//goodix_feature_resume(ts);
	TS_LOG_INFO("Resume end");
	ts->enter_suspend = false;
	return 0;
}
static int goodix_read_gestrue_data(
					     struct ts_fingers *info,
					     struct ts_easy_wakeup_info
					     *gesture_report_info,
					     unsigned char gesture_id)
{
	int retval = NO_ERR;

	retval = goodix_check_key_gesture_report(info,
									gesture_report_info,
									gesture_id);
	return retval;
}
static int goodix_check_gesture(struct ts_fingers *info)
{
	int retval = NO_ERR;
	unsigned char gesture_id[2],doze_buf;
	struct ts_easy_wakeup_info *gesture_report_info = &g_goodix_dev_data->easy_wakeup_info;
	if (false == gesture_report_info->easy_wakeup_flag)
		return 1;

	retval = goodix_i2c_read(0x814B, gesture_id, 2);
	TS_LOG_INFO("gesture_id = 0x%02X, point_num : %d ", gesture_id[0], gesture_id[1]);
	retval = goodix_read_gestrue_data(info, gesture_report_info, gesture_id[0]);

	/*
	if(retval > 0){
		retval = goodix_read_gestrue_data(info, gesture_report_info, gesture_id);
		return 0;
	}
	else
	{
		TS_LOG_ERR("%s read state:%d \n",__func__);
		return 1;
	}*/

	/* Clear 0x814B */
	doze_buf = 0x00;
	goodix_i2c_write(0x814B, &doze_buf, 1);

	return retval;
}
/**
 * goodix_touch_evt_handler - handle touch event
 * (pen event, key event, finger touch envent)
 * Return    <0: failed, 0: succeed
 */
static int goodix_touch_evt_handler(struct goodix_ts_data  *ts,
				struct ts_fingers *info)
{
	u8 touch_data[1 + 8 * GTP_MAX_TOUCH + 1] = {0};
	static u16 pre_index = 0;
	u16 cur_index = 0;
	u8 touch_num;
	u8 *coor_data = NULL;
	int id, x, y, w, i;
	int ret = -1;
	static u16 pre_touch = 0;
	u8 sync_val = 0;
	ret = goodix_check_gesture(info);
	if (!ret) {
		TS_LOG_DEBUG
		    ("focal_gesture_report is called and report gesture\n");
		return ret;
	}
	ret = goodix_i2c_read(GTP_READ_COOR_ADDR, touch_data, 10);
	if (unlikely(ret))
		goto exit;

#if 0
	if (unlikely(!touch_data[0])) {
		/* hw request */
		goodix_request_event_handler(ts);
		ret = 1;
		goto exit;
	}

	if(touch_data[0]==0){
		return;
	}
#endif

	if((touch_data[0]&0x80)==0){
		TS_LOG_ERR("Illegal state!");
		return -1;
	}

	touch_num = touch_data[0] & 0x0f;
	TS_LOG_DEBUG("touch_num = %d ",touch_num);

	if (touch_num > GTP_MAX_TOUCH) {
		TS_LOG_ERR("Illegal finger number!");
		goto exit;
	}

	/* read the remaining coor data
		* 0x814E(touch status) +
		* 8(every coordinate consist of 8 bytes data) * touch num +
		* keycode
		*/
	if (touch_num > 1) {
		u8 buf[8 * GTP_MAX_TOUCH];
		ret = goodix_i2c_read((GTP_READ_COOR_ADDR),buf, (1+touch_num* 8+1));
		if (ret)
			goto exit;
		memcpy(&touch_data[0], &buf[0], (1+8 * touch_num +1));
	}



#if 0
	key_value = touch_data[1 + 8 * touch_num];
	/*  start check current event */
	if ((touch_data[0] & 0x10) && (key_value & 0x0F)) {
		//for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
//			input_report_key(dev, gt1x_touch_key_array[i],
//					key_value & (0x01 << i));
		// key
	}
#endif

	if(touch_num>0){
		for (i = 0; i < touch_num; i++) {
			coor_data = &touch_data[i * 8+1];
			id = coor_data[0] & 0x0f;
			x = coor_data[1] | (coor_data[2] << 8);
			y = coor_data[3] | (coor_data[4] << 8);
			w = coor_data[5] | (coor_data[6] << 8);

			if (unlikely(ts->flip_x))
				info->fingers[id].x = ts->max_x - x;
			else
				info->fingers[id].x = x;

			if (unlikely(ts->flip_y))
				info->fingers[id].y = ts->max_y - y;
			else
				info->fingers[id].y = y;

			info->fingers[id].major = w;
			info->fingers[id].minor = w;
			info->fingers[id].pressure = w;
			info->fingers[id].status = TP_FINGER;
			cur_index |= 1 << id;
			TS_LOG_DEBUG("%s:x = 0x%x; y = 0x%x; w = 0x%x\n",__func__, x, y, w);
		}

		info->cur_finger_number = touch_num;
	}
	else if (pre_touch){
		info->cur_finger_number = 0;
	}
	TS_LOG_DEBUG("info->cur_finger_number  = [%d],pre_touch is [%d] \n", info->cur_finger_number ,pre_touch);
	pre_touch = touch_num;
#ifdef ROI
	if (pre_index != cur_index && (cur_index & pre_index) != cur_index)
		goodix_cache_roidata(&ts->roi);
	pre_index = cur_index;
#endif

exit:
	TS_LOG_DEBUG("evt_handler ret= [%d] \n", ret);

/*sync_evt:*/
	if (!ts->rawdiff_mode)
		ret = goodix_i2c_write(GTP_READ_COOR_ADDR, &sync_val, 1);
	else
		TS_LOG_DEBUG("Firmware rawdiff mode");
	return ret;
}

/**
 * goodix_irq_bottom_half - Goodix touchscreen work function.
 */
static int goodix_irq_top_half(struct ts_cmd_node *cmd)
{
	cmd->command = TS_INT_PROCESS;
	TS_LOG_DEBUG("irq top half called\n");
	return NO_ERR;
}
#if 0
static int goodix_gesture_evt_handler(struct goodix_ts_data *ts,
				struct ts_fingers *info)
{
	u8 doze_buf[4] = {0}, ges_type;
	int len;
	int ret = 0;

	/** package: -head 4B + track points + extra info-
		* - head -
		*  doze_buf[0]: gesture type,
		*  doze_buf[1]: number of gesture points ,
		*  doze_buf[2]: protocol type,
		*  doze_buf[3]: gesture extra data length.
		*/
	ret = goodix_i2c_read(GTP_REG_WAKEUP_GESTURE, doze_buf, 4);
	if (ret < 0)
		return 0;

	ges_type = doze_buf[0];
	len = doze_buf[1];
/*	need_chk = doze_buf[2] & 0x80;
	extra_len = doze_buf[3];
*/

	TS_LOG_INFO("0x%x = 0x%02X,0x%02X,0x%02X,0x%02X", GTP_REG_WAKEUP_GESTURE,
		doze_buf[0], doze_buf[1], doze_buf[2], doze_buf[3]);

	/* check gesture type (if available?) */
	if (ges_type == 0) {
		TS_LOG_INFO("Invalid gesture");
		doze_buf[0] = 0x00;
		//goodix_i2c_write(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
		//gesture_enter_doze();
		info->gesture_wakeup_value = TS_GESTURE_INVALID;
		return 0;
	}

	switch (ges_type) {
	case 0xAA:
		break;
	case 0xCC:
		info->gesture_wakeup_value = TS_DOUBLE_CLICK;
		break;
	default:
		break;
	}
	input_report_key(goodix_ts->goodix_device_data->ts_platform_data->input_dev, info->gesture_wakeup_value , 1);
	input_sync(goodix_ts->goodix_device_data->ts_platform_data->input_dev);
	input_report_key(goodix_ts->goodix_device_data->ts_platform_data->input_dev, info->gesture_wakeup_value , 0);
	input_sync(goodix_ts->goodix_device_data->ts_platform_data->input_dev);
	ret = 1;
	doze_buf[0] = 0; // clear ges flag
	goodix_i2c_write(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
	return ret;
}
#endif
static int goodix_irq_bottom_half(struct ts_cmd_node *in_cmd,
				struct ts_cmd_node *out_cmd)
{
	struct goodix_ts_data *ts = goodix_ts;
	struct ts_fingers *ts_fingers;
	u8 sync_val = 0;
	int ret = 0;

	if (!ts)
		return -ENODEV;
	ts_fingers = &out_cmd->cmd_param.pub_params.algo_param.info;
	out_cmd->command = TS_INVAILD_CMD;
	out_cmd->cmd_param.pub_params.algo_param.algo_order =
			ts->goodix_device_data->algo_id;
	/* handle touch event
	 * return: <0 - error, 0 - touch event handled,
	 * 			1 - hw request event handledv */
	ret = goodix_touch_evt_handler(ts, ts_fingers);
	if (ret == 0)
		out_cmd->command = TS_INPUT_ALGO;
	//ret = goodix_i2c_write(GTP_READ_COOR_ADDR, &sync_val, 1);

	return ret;
}

static int goodix_i2c_test(void)
{
	u8 hw_info;
	int ret;
	ret = goodix_i2c_read(GTP_REG_CONFIG_DATA,&hw_info,sizeof(hw_info));
	TS_LOG_INFO("IIC test Info:%08X ret=%d", hw_info,ret);
	return ret;
}


/**
 * goodix_pinctrl_init - pinctrl init
 */
static int goodix_pinctrl_init(struct goodix_ts_data *ts)
{
      int ret = 0;
       /* Get pinctrl if target uses pinctrl */
      TS_LOG_INFO("ts->pdev->name=%s",ts->pdev->name);
      ts->pinctrl = devm_pinctrl_get(&(ts->pdev->dev));
      if (IS_ERR_OR_NULL(ts->pinctrl)) {
	  	TS_LOG_ERR("Target does not use pinctrl %d\n", ret);
		ret = -EINVAL;
		return ret;
	}
	ts->pinctrl_state_active= pinctrl_lookup_state(ts->pinctrl,PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(ts->pinctrl_state_active)) {
		TS_LOG_ERR("Can not lookup %s pinstate \n",PINCTRL_STATE_ACTIVE);
		ret = -EINVAL;
		goto err_pinctrl_lookup;
	}
	ts->pinctrl_state_suspend= pinctrl_lookup_state(ts->pinctrl,PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(ts->pinctrl_state_suspend)) {
		TS_LOG_ERR("Can not lookup %s pinstate \n",PINCTRL_STATE_SUSPEND);
		ret = -EINVAL;
		goto err_pinctrl_lookup;
	}
	ts->pinctrl_state_release= pinctrl_lookup_state(ts->pinctrl,PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(ts->pinctrl_state_release)) {
		TS_LOG_ERR("-Can not lookup %s pinstate \n",PINCTRL_STATE_RELEASE);
		ret = -EINVAL;
		goto err_pinctrl_lookup;
	}
	ts->pinctrl_state_as_int= pinctrl_lookup_state(ts->pinctrl,PINCTRL_STATE_AS_INT);
	if (IS_ERR_OR_NULL(ts->pinctrl_state_as_int)) {
		TS_LOG_ERR("-Can not lookup %s pinstate \n",PINCTRL_STATE_AS_INT);
		ret = -EINVAL;
		goto err_pinctrl_lookup;
	}
	ts->pinctrl_state_int_high= pinctrl_lookup_state(ts->pinctrl,PINCTRL_STATE_INT_HIGH);
	if (IS_ERR_OR_NULL(ts->pinctrl_state_int_high)) {
		TS_LOG_ERR("-Can not lookup %s pinstate \n",PINCTRL_STATE_RELEASE);
	}
	ts->pinctrl_state_int_low= pinctrl_lookup_state(ts->pinctrl,PINCTRL_STATE_INT_LOW);
	if (IS_ERR_OR_NULL(ts->pinctrl_state_int_low)) {
		TS_LOG_ERR("-Can not lookup %s pinstate \n", PINCTRL_STATE_RELEASE);
	}

	return 0;
err_pinctrl_lookup:
			devm_pinctrl_put(ts->pinctrl);
       return ret;

}

int goodix_pinctrl_select_normal(struct goodix_ts_data *ts)
{
	int ret = 0;
	if (goodix_ts->pinctrl && goodix_ts->pinctrl_state_active) {
	ret = pinctrl_select_state(goodix_ts->pinctrl, goodix_ts->pinctrl_state_active);
		if (ret < 0)
			TS_LOG_ERR("Set active pin state error:%d", ret);
	}
	return ret;
}
int goodix_pinctrl_select_suspend(struct goodix_ts_data *ts)
{
	int ret = 0;
	if (goodix_ts->pinctrl && goodix_ts->pinctrl_state_suspend) {
	ret = pinctrl_select_state(goodix_ts->pinctrl, goodix_ts->pinctrl_state_suspend);
		if (ret < 0)
			TS_LOG_ERR("Set suspend pin state error:%d", ret);
	}
	return ret;
}

static void goodix_pinctrl_release(struct goodix_ts_data *ts)
{
	if (ts->pinctrl)
		devm_pinctrl_put(ts->pinctrl);
	ts->pinctrl = NULL;
	ts->pinctrl_state_active = NULL;
	ts->pinctrl_state_suspend = NULL;
	ts->pinctrl_state_release = NULL;
}

/**
 * goodix_read_version - Read gt1x version info.
 * @hw_info: address to store version info
 * Return 0-succeed.
 */
int goodix_read_version(struct goodix_hw_info * hw_info)
{
	u8 buf[12] = { 0 };
	u32 mask_id;
	u16 patch_id;
	u8 product_id[5] = {0};
	u8 sensor_id, match_opt,config_ver;
	int retry = 3;
	unsigned int i;
	u8 checksum = 0;
	int ret = -1;
    u8 reg_val[1];
	while (retry--) {
		ret = goodix_i2c_read(GTP_REG_VERSION, buf, sizeof(buf));
              GTP_INFO("Invalid version info:%d%d%d  ret=%d", buf[0], buf[1], buf[2],ret);
		if (!ret) {
			for (i = 0, checksum = 0; i < sizeof(buf); i++)
				checksum += buf[i];

			if (checksum == 0 &&/* first 3 bytes must be number or char */
				IS_NUM_OR_CHAR(buf[0]) && IS_NUM_OR_CHAR(buf[1])
				&& IS_NUM_OR_CHAR(buf[2]) && buf[10] != 0xFF) {
				break;
			} else if (checksum == (u8)(buf[11] * 2) && buf[10] != 0xFF) {
				/* checksum calculated by boot code */
				break;
			} else{
			GTP_ERROR("Invalid version info:%c%c%c", buf[0], buf[1], buf[2]);
			break;}
		}

		GTP_DEBUG("Read version failed,retry: %d", retry);
		msleep(100);
	}

	if (retry <= 0)
		return -ENODEV;

	mask_id = (u32) ((buf[7] << 16) | (buf[8] << 8) | buf[9]);
	patch_id = (u32)(buf[4] + (buf[5] << 8));
	memcpy(product_id, buf, 4);
	sensor_id = buf[10] & 0x0F;
	match_opt = (buf[10] >> 4) & 0x0F;

	GTP_INFO("IC Version:GT%s_%04X(FW)_%04X(Boot)_%02X(SensorID)",
		product_id, patch_id, mask_id >> 8, sensor_id);
	ret = goodix_i2c_read(GTP_REG_CONFIG_DATA, &config_ver, 1);
	if(ret<0)
	{
		TS_LOG_ERR("Read goodix config version error\n");
	}
	if (hw_info != NULL) {
		hw_info->mask_id = mask_id;
		hw_info->patch_id = patch_id;
		memcpy(hw_info->product_id, product_id, 5);
		hw_info->sensor_id = sensor_id;
		hw_info->match_opt = match_opt;
		hw_info->config_ver = config_ver;
	}
	if(0x02 == hw_info->sensor_id)
	{
		strcpy(goodix_ts->hw_info.vendor_name,"Helitec");
	}
	else
	{
		strcpy(goodix_ts->hw_info.vendor_name,"Unknow");
	}
	goodix_ts->sensor_id_valid = true;
	return 0;
}

/**
 * goodix_init_configs - Prepare config data for touch ic,
 * don't call this function after initialization.
 *
 * Return 0--success,<0 --fail.
 */
int goodix_init_configs(struct goodix_ts_data *ts)
{
	u8 sensor_id, *cfg_data;
	int cfg_len = 0;
	int ret = 0;

	sensor_id = ts->hw_info.sensor_id;
	if (sensor_id > 5) {
		GTP_ERROR("Invalid sensor ID");
		return -EINVAL;
	}

	/* max config data length */
	cfg_len = sizeof(ts->normal_config.data);
	cfg_data = kzalloc(cfg_len, GFP_KERNEL);
	if (!cfg_data)
		return -ENOMEM;

	/* parse normal config data */
	ret = goodix_parse_cfg_data(ts, "normal_config", cfg_data,
				&cfg_len, sensor_id);
	TS_LOG_INFO("cfg_data[1]=0x%x\n",cfg_data[1]);
	if (ret < 0) {
		GTP_ERROR("Failed to parse normal_config data:%d", ret);
		goto exit_kfree;
	}

	cfg_data[0] &= 0x7F; /* mask config version */
	GTP_INFO("Normal config version:%d,size:%d", cfg_data[0], cfg_len);
	memcpy(&ts->normal_config.data[0], cfg_data, cfg_len);
	ts->normal_config.size = cfg_len;
	ts->normal_config.delay_ms = 200;
	ts->normal_config.name = "normal_config";
	ts->normal_config.initialized = true;

	/* parse glove config data */
	ret = goodix_parse_cfg_data(ts, "glove_config", cfg_data,
				&cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse glove_config data:%d", ret);
		ts->glove_config.initialized = false;
		ret = 0;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7F; /* mask config version */
		GTP_INFO("Glove config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->glove_config.data[0], cfg_data, cfg_len);
		ts->glove_config.size = cfg_len;
		ts->glove_config.delay_ms = 20;
		ts->glove_config.name = "glove_config";
		ts->glove_config.initialized = true;
	} else {
		ts->glove_config.initialized = false;
	}

	/* parse glove config data */
	ret = goodix_parse_cfg_data(ts, "holster_config", cfg_data,
				&cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse holster_config data:%d", ret);
		ts->holster_config.initialized = false;
		ret = 0;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7F; /* mask config version */
		GTP_INFO("Holster config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->holster_config.data[0], cfg_data, cfg_len);
		ts->holster_config.size = cfg_len;
		ts->holster_config.delay_ms = 20;
		ts->holster_config.name = "holster_config";
		ts->holster_config.initialized = true;
	} else {
		ts->holster_config.initialized = false;
	}

exit_kfree:
	kfree(cfg_data);
	return ret;
}

int gt1x_send_cfg(u8 *config, int cfg_len)
{
	static DEFINE_MUTEX(mutex_cfg);
	//u8 *config;
	int i;
	s32 ret = 0, retry = 0;
	u8 checksum = 0;
	TS_LOG_INFO("Goodix Send Cfg");
	//config = config;
	//cfg_len = cfg_len;

	mutex_lock(&mutex_cfg);
	//TS_LOG_INFO("Send %s,ver:%02x size:%d", cfg_ptr->name, config[0], cfg_len);
	//TS_LOG_INFO("write array:");
	GTP_DEBUG_ARRAY(config, cfg_len);

	if (cfg_len != GTP_CONFIG_ORG_LENGTH) {
		TS_LOG_ERR("Invalid config size:%d", cfg_len);
		mutex_unlock(&mutex_cfg);
		return -1;
	}

	for (i = 0, checksum = 0; i < cfg_len - 2; i++)
		checksum += config[i];
	if (!checksum) {
		TS_LOG_ERR("Invalid config,all of the bytes is zero");
		mutex_unlock(&mutex_cfg);
		return -1;
	}

	checksum = 0 - checksum;

	config[cfg_len - 2] = checksum & 0xFF;
	config[cfg_len - 1] = 0x01;
	retry = 0;
	while (retry++ < 3) {
		ret = goodix_i2c_write(GTP_REG_CONFIG_DATA, config, cfg_len);
		if (!ret) {
			//if (cfg_ptr->delay_ms > 0)
				//msleep(cfg_ptr->delay_ms);
			mutex_unlock(&mutex_cfg);
			TS_LOG_INFO("Send config successfully");
			return 0;
		}
	}

	TS_LOG_ERR("Send config failed");
	mutex_unlock(&mutex_cfg);
	return ret;
}
#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))
#define set_reg_bit(reg, index, val)	((reg) ^= (!(val) << (index)))
s32 gt1x_init_panel(void)
{
	s32 ret = 0;
	u8 cfg_len = 0,i=0;
	int tpd_resolution[2] = {720,1280};
	int  gt1x_abs_x_max;
	int  gt1x_abs_y_max;
	gt1x_abs_x_max = tpd_resolution[0];
	gt1x_abs_y_max = tpd_resolution[1];
	int  GTP_INT_TRIGGER = 1;
	u8 gt1x_wakeup_level = 0;
	u8 gt1x_int_type = 1;
#ifdef CONFIG_GTP_DRIVER_SEND_CFG
	u8 sensor_id = 3;
	const u8 cfg_grp0[] = GTP_CFG_GROUP0;
	const u8 cfg_grp1[] = GTP_CFG_GROUP1;
	const u8 cfg_grp2[] = GTP_CFG_GROUP2;
	const u8 cfg_grp3[] = GTP_CFG_GROUP3;
	const u8 cfg_grp4[] = GTP_CFG_GROUP4;
	const u8 cfg_grp5[] = GTP_CFG_GROUP5;
	const u8 *cfgs[] = {
		cfg_grp0, cfg_grp1, cfg_grp2,
		cfg_grp3, cfg_grp4, cfg_grp5
	};
	u8 cfg_lens[] = {
		CFG_GROUP_LEN(cfg_grp0),
		CFG_GROUP_LEN(cfg_grp1),
		CFG_GROUP_LEN(cfg_grp2),
		CFG_GROUP_LEN(cfg_grp3),
		CFG_GROUP_LEN(cfg_grp4),
		CFG_GROUP_LEN(cfg_grp5)
	};
#if 0
#ifdef CONFIG_GTP_CHARGER_SWITCH
	const u8 cfg_grp0_charger[] = GTP_CFG_GROUP0_CHARGER;
	const u8 cfg_grp1_charger[] = GTP_CFG_GROUP1_CHARGER;
	const u8 cfg_grp2_charger[] = GTP_CFG_GROUP2_CHARGER;
	const u8 cfg_grp3_charger[] = GTP_CFG_GROUP3_CHARGER;
	const u8 cfg_grp4_charger[] = GTP_CFG_GROUP4_CHARGER;
	const u8 cfg_grp5_charger[] = GTP_CFG_GROUP5_CHARGER;
	const u8 *cfgs_charger[] = {
		cfg_grp0_charger, cfg_grp1_charger, cfg_grp2_charger,
		cfg_grp3_charger, cfg_grp4_charger, cfg_grp5_charger
	};
	u8 cfg_lens_charger[] = {
		CFG_GROUP_LEN(cfg_grp0_charger),
		CFG_GROUP_LEN(cfg_grp1_charger),
		CFG_GROUP_LEN(cfg_grp2_charger),
		CFG_GROUP_LEN(cfg_grp3_charger),
		CFG_GROUP_LEN(cfg_grp4_charger),
		CFG_GROUP_LEN(cfg_grp5_charger)
	};
#endif				/* end  CONFIG_GTP_CHARGER_SWITCH */
#endif
	GTP_DEBUG("Config Groups Length: %d, %d, %d, %d, %d, %d", cfg_lens[0], cfg_lens[1], cfg_lens[2], cfg_lens[3],
		  cfg_lens[4], cfg_lens[5]);
      #if 0
	sensor_id = gt1x_version.sensor_id;
	if (sensor_id >= 6 || cfg_lens[sensor_id] < GTP_CONFIG_MIN_LENGTH
	    || cfg_lens[sensor_id] > GTP_CONFIG_MAX_LENGTH) {
		sensor_id = 0;
	}
       #endif
	cfg_len = cfg_lens[sensor_id];

	GTP_INFO("CTP_CONFIG_GROUP%d used, gt1x_config length: %d", sensor_id, cfg_len);

	if (cfg_len < GTP_CONFIG_MIN_LENGTH || cfg_len > GTP_CONFIG_MAX_LENGTH) {
		GTP_ERROR
		    ("CTP_CONFIG_GROUP%d is INVALID CONFIG GROUP! NO Config Sent! ;"
			"You need to check you header file CFG_GROUP section!",
		     sensor_id + 1);
		return -1;
	}

	memset(gt1x_config, 0, sizeof(gt1x_config));
	memcpy(gt1x_config, cfgs[sensor_id], cfg_len);

	/* clear the flag, avoid failure when send the_config of driver. */
	gt1x_config[0] &= 0x7F;

#ifdef CONFIG_GTP_CUSTOM_CFG
	/*gt1x_config[RESOLUTION_LOC] = (u8) tpd_resolution[0];
	gt1x_config[RESOLUTION_LOC + 1] = (u8) (tpd_resolution[0] >> 8);
	gt1x_config[RESOLUTION_LOC + 2] = (u8) tpd_resolution[1];
	gt1x_config[RESOLUTION_LOC + 3] = (u8) (tpd_resolution[1] >> 8);*/

	/*GTP_INFO("Res: %d * %d, trigger: %d", tpd_resolution[0],
		tpd_resolution[1], GTP_INT_TRIGGER);*/

      #if 0
	if (GTP_INT_TRIGGER == 0) {	/* RISING  */
		gt1x_config[TRIGGER_LOC] &= 0xfe;
	} else if (GTP_INT_TRIGGER == 1) {	/* FALLING */
		gt1x_config[TRIGGER_LOC] |= 0x01;
	}
	#endif
#endif				/* END CONFIG_GTP_CUSTOM_CFG */

#ifdef CONFIG_GTP_CHARGER_SWITCH
	GTP_DEBUG("Charger Config Groups Length: %d, %d, %d, %d, %d, %d", cfg_lens_charger[0],
		  cfg_lens_charger[1], cfg_lens_charger[2], cfg_lens_charger[3], cfg_lens_charger[4],
		  cfg_lens_charger[5]);

	memset(gt1x_config_charger, 0, sizeof(gt1x_config_charger));
	if (cfg_lens_charger[sensor_id] == cfg_len)
		memcpy(gt1x_config_charger, cfgs_charger[sensor_id], cfg_len);

	/* clear the flag, avoid failure when send the config of driver. */
	gt1x_config_charger[0] &= 0x7F;

#ifdef CONFIG_GTP_CUSTOM_CFG
	gt1x_config_charger[RESOLUTION_LOC] = (u8) tpd_dts_data.tpd_resolution[0];
	gt1x_config_charger[RESOLUTION_LOC + 1] = (u8) (tpd_dts_data.tpd_resolution[0] >> 8);
	gt1x_config_charger[RESOLUTION_LOC + 2] = (u8) tpd_dts_data.tpd_resolution[1];
	gt1x_config_charger[RESOLUTION_LOC + 3] = (u8) (tpd_dts_data.tpd_resolution[1] >> 8);

	if (GTP_INT_TRIGGER == 0) {	/* RISING  */
		gt1x_config_charger[TRIGGER_LOC] &= 0xfe;
	} else if (GTP_INT_TRIGGER == 1) {	/* FALLING */
		gt1x_config_charger[TRIGGER_LOC] |= 0x01;
	}
#endif				/* END CONFIG_GTP_CUSTOM_CFG */
	if (cfg_lens_charger[sensor_id] != cfg_len)
		memset(gt1x_config_charger, 0, sizeof(gt1x_config_charger));
#endif				/* END CONFIG_GTP_CHARGER_SWITCH */

#else				/* DRIVER NOT SEND CONFIG */
	cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret = goodix_i2c_read(GTP_REG_CONFIG_DATA, gt1x_config, cfg_len);
	if (ret < 0)
	{
	      TS_LOG_ERR("gt1x_i2c_read GTP_REG_CONFIG_DATA  ERROR");
		return ret;
	}
#endif				/* END CONFIG_GTP_DRIVER_SEND_CFG */
	GTP_DEBUG_FUNC();
	/* match resolution when gt1x_abs_x_max & gt1x_abs_y_max have been set already */
	/*if ((gt1x_abs_x_max == 0) && (gt1x_abs_y_max == 0)) {
		gt1x_abs_x_max = (gt1x_config[RESOLUTION_LOC + 1] << 8) + gt1x_config[RESOLUTION_LOC];
		gt1x_abs_y_max = (gt1x_config[RESOLUTION_LOC + 3] << 8) + gt1x_config[RESOLUTION_LOC + 2];
		gt1x_int_type = (gt1x_config[TRIGGER_LOC]) & 0x03;
		gt1x_wakeup_level = !(gt1x_config[MODULE_SWITCH3_LOC] & 0x20);
	} else {
		gt1x_config[RESOLUTION_LOC] = (u8) gt1x_abs_x_max;
		gt1x_config[RESOLUTION_LOC + 1] = (u8) (gt1x_abs_x_max >> 8);
		gt1x_config[RESOLUTION_LOC + 2] = (u8) gt1x_abs_y_max;
		gt1x_config[RESOLUTION_LOC + 3] = (u8) (gt1x_abs_y_max >> 8);
		set_reg_bit(gt1x_config[MODULE_SWITCH3_LOC], 5, !gt1x_wakeup_level);
		gt1x_config[TRIGGER_LOC] = (gt1x_config[TRIGGER_LOC] & 0xFC) | gt1x_int_type;*/
#ifdef CONFIG_GTP_CHARGER_SWITCH
		gt1x_config_charger[RESOLUTION_LOC] = (u8) gt1x_abs_x_max;
		gt1x_config_charger[RESOLUTION_LOC + 1] = (u8) (gt1x_abs_x_max >> 8);
		gt1x_config_charger[RESOLUTION_LOC + 2] = (u8) gt1x_abs_y_max;
		gt1x_config_charger[RESOLUTION_LOC + 3] = (u8) (gt1x_abs_y_max >> 8);
		set_reg_bit(gt1x_config[MODULE_SWITCH3_LOC], 5, !gt1x_wakeup_level);
		gt1x_config[TRIGGER_LOC] = (gt1x_config[TRIGGER_LOC] & 0xFC) | gt1x_int_type;
#endif
	ret = gt1x_send_cfg(gt1x_config, cfg_len);
	ret = goodix_i2c_read(GTP_REG_CONFIG_DATA,&goodix_ts->hw_info.config_ver, 1);
	if(ret<0)
	TS_LOG_ERR("Read goodix config version error\n");
	return ret;
}
static int ctp_proc_read_show (struct seq_file* m, void* data)
{
	char temp[40] = {0};
	char firmware_ver[4] = {0};
	sprintf(firmware_ver,"0x%x",goodix_ts->hw_info.config_ver);
	sprintf(temp, "[Vendor]%s,[Fw]%s,[IC]GT917\n",goodix_ts->hw_info.vendor_name,firmware_ver);
	seq_printf(m, "%s\n", temp);
	return 0;
}

static int ctp_proc_open (struct inode* inode, struct file* file)
{
    return single_open(file, ctp_proc_read_show, inode->i_private);
}

static const struct file_operations g_ctp_proc =
{
    .open = ctp_proc_open,
    .read = seq_read,
};

static void Goodix_set_tp_info(void)
{
	if(proc_create(CTP_PROC_FILE, 0444, NULL, &g_ctp_proc)== NULL)
	{
		TS_LOG_ERR("create_proc_entry tp_info failed\n");
	}
}
static void goodix_set_app_info(void)
{
	char touch_info[40] = {0};
	sprintf(touch_info,
				"Gd917D [vendor]%s [FW]0x%x",
				goodix_ts->hw_info.vendor_name,
				goodix_ts->hw_info.config_ver);
#ifdef CONFIG_APP_INFO
	app_info_set("touch_panel", touch_info);
#endif
}
static int goodix_chip_parse_config(struct device_node *device,
				struct ts_kit_device_data *chip_data)
{
	int ret = 0;

	GTP_INFO("Parse config");
	if (!device || !chip_data)
		return -ENODEV;
	ret = of_property_read_u32(device, GTP_IRQ_CFG,
						&chip_data->irq_config);
	if (ret) {
		TS_LOG_ERR("Get irq config failed");
		ret = -EINVAL;
		goto err;
	}

	/*ret = of_property_read_u32(device, GTP_ALGO_ID,
						&chip_data->algo_id);
	if (ret) {
		TS_LOG_ERR("Get algo id failed");
		ret = -EINVAL;
		goto err;
	}*/

	ret = of_property_read_u32(device, GTP_WD_CHECK,
						&chip_data->need_wd_check_status);
	if (ret) {
		GTP_ERROR("need_wd_check_status parmerter not exit use data");
		chip_data->need_wd_check_status = false;
	}

	ret = of_property_read_u32(device, GTP_WD_TIMEOUT,
						&chip_data->check_status_watchdog_timeout);
	if (ret) {
		GTP_ERROR("check_status_watchdog_timeout not exit use data");
		chip_data->check_status_watchdog_timeout = 0;
	}

	ret = of_property_read_string(device, GTP_TEST_TYPE,
				 &chip_data->tp_test_type);
	if (ret) {
		GTP_INFO
		    ("get device tp_test_type not exit,use default value");
		strncpy(chip_data->tp_test_type,
			"Normalize_type:judge_different_reslut",
			TS_CAP_TEST_TYPE_LEN);
		ret = 0;
	}
err:
	return ret;
}
static int goodix_chip_init(void)
{
	struct goodix_ts_data *ts = goodix_ts;
	int ret = -1;
	ret = goodix_chip_parse_config(g_goodix_dev_data->cnode, ts ->goodix_device_data);
	if (ret < 0)
	{
	      TS_LOG_ERR("goodix_chip_parse_config ERROR");
		return ret;
	}
	/* read version information. pid/vid/sensor id */
	ret = goodix_read_version(&ts->hw_info);
	Goodix_set_tp_info();//lc factory
	goodix_set_app_info();
	if (ret < 0)
		return ret;

	/* obtain goodix dt properties */
	ret = goodix_parse_dts(g_goodix_dev_data->cnode,goodix_ts);
	if (ret < 0)
		return ret;
	if (ts->tools_support)
		init_wr_node();

	/* init config data, normal/glove/hoslter config data */
	ret = gt1x_init_panel();
	if (ret != 0)
		GTP_ERROR("GTP init panel failed.");
#ifdef ROI_1
	goodix_ts_roi_init(&goodix_ts->roi);
#endif

	return 0;
}

static int goodix_input_config(struct input_dev *input_dev)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return -ENODEV;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);

	set_bit(TS_DOUBLE_CLICK, input_dev->keybit);
	set_bit(TS_SLIDE_L2R, input_dev->keybit);
	set_bit(TS_SLIDE_R2L, input_dev->keybit);
	set_bit(TS_SLIDE_T2B, input_dev->keybit);
	set_bit(TS_SLIDE_B2T, input_dev->keybit);
	set_bit(TS_CIRCLE_SLIDE, input_dev->keybit);
	set_bit(TS_LETTER_c, input_dev->keybit);
	set_bit(TS_LETTER_e, input_dev->keybit);
	set_bit(TS_LETTER_m, input_dev->keybit);
	set_bit(TS_LETTER_w, input_dev->keybit);
	set_bit(TS_PALM_COVERED, input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#ifdef INPUT_TYPE_B_PROTOCOL
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(input_dev, GTP_MAX_TOUCH, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, GTP_MAX_TOUCH);
#endif
#endif
	TS_LOG_DEBUG("x_max=%d y_max=%d\n",ts->goodix_device_data->x_max,ts->goodix_device_data->y_max);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->goodix_device_data->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->goodix_device_data->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, GTP_MAX_TOUCH, 0, 0);

	return 0;
}

static int goodix_pinctr_int_ouput_low(void)
{
	int ret = 0;
	if (goodix_ts->pinctrl && goodix_ts->pinctrl_state_int_low) {
	ret = pinctrl_select_state(goodix_ts->pinctrl, goodix_ts->pinctrl_state_int_low);
	if (ret < 0)
		TS_LOG_ERR("Set irq pin state error:%d", ret);
	}
	return ret;
}
/**
 * goodix_switch_wrokmode - Switch working mode.
 * @workmode: GTP_CMD_SLEEP - Sleep mode
 *			  GESTURE_MODE - gesture mode
 * Returns  0--success,non-0--fail.
 */
static int goodix_switch_wrokmode(int wrokmode)
{
	s32 retry = 0;
	u8 cmd;
	switch (wrokmode) {
	case SLEEP_MODE:
		cmd = GTP_CMD_SLEEP;
		//goodix_pinctr_int_ouput_low();//gpio_direction_output(irq_gpio, 0);
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
		msleep(5);
		break;
	case GESTURE_MODE:
		cmd = GTP_CMD_GESTURE_WAKEUP;
		break;
	default:
		return -EINVAL;
	}

	GTP_INFO("Switch working mode[%02X]", cmd);
	while (retry++ < 3) {
		if (!goodix_send_cmd(cmd, 0))
			return 0;
		msleep(20);
	}

	GTP_ERROR("Failed to switch working mode");
	return -1;
}
static void goodix_put_device_into_easy_wakeup(void)
{
	int retval;
	s8 retry = 0;
	struct ts_easy_wakeup_info *info = &g_goodix_dev_data->easy_wakeup_info;

	TS_LOG_DEBUG
	    ("goodix_put_device_into_easy_wakeup  info->easy_wakeup_flag =%x \n",
	     info->easy_wakeup_flag);
	/*if the sleep_gesture_flag is ture,it presents that  the tp is at sleep state*/

	if (true == info->easy_wakeup_flag) {
		TS_LOG_INFO
		    ("goodix_put_device_into_easy_wakeup  info->easy_wakeup_flag =%x \n",
		     info->easy_wakeup_flag);
		return;
	}

	/*Wakeup Gesture (d0) set 1 */
	TS_LOG_INFO("Entering gesture mode.");
	while(retry++ < 5)
	{
	    retval = goodix_send_cmd(0x8046, GTP_CMD_GESTURE_WAKEUP);
	    if (retval < 0)
	    {
	        TS_LOG_ERR("failed to set doze flag into 0x8046, %d", retval);
	        goto NEXT;
	    }

	    retval = goodix_send_cmd(0x8040, GTP_CMD_GESTURE_WAKEUP);
	    if (retval < 0)
	    {
	        TS_LOG_ERR("failed to set doze flag into 0x8040, %d", retval);
	        goto NEXT;
	    }

NEXT:
	   msleep(10);
	}

	info->easy_wakeup_flag = true;
}
static int goodix_before_suspend(void)
{
	return NO_ERR;
}
static void goodix_sleep_mode_in(void)
{
	s8 ret = -1;
	s8 retry = 0;
	struct goodix_ts_data *ts = goodix_ts;

	//goodix_pinctrl_select_suspend(ts);
	//gpio_direction_output(ts->ts_platform_data->irq_gpio, 0);
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(5);

	while(retry++ < 5)
	{
	    ret = goodix_send_cmd(0x8040, GTP_CMD_SLEEP);
	    if (ret == 0)
	    {
	        TS_LOG_INFO("GTP enter sleep!");
	        return;
	    }
	    msleep(10);
	}
	TS_LOG_ERR("GTP send sleep cmd failed.");
	return;
}
static int goodix_chip_suspend(void)
{
	struct goodix_ts_data *ts = goodix_ts;
	if (ts == NULL)
		return -ENODEV;

	switch (ts->goodix_device_data->easy_wakeup_info.sleep_mode){
	case TS_POWER_OFF_MODE:
		//goodix_switch_wrokmode(SLEEP_MODE);
		goodix_sleep_mode_in();
		break;
	case TS_GESTURE_MODE:
		//goodix_switch_wrokmode(GESTURE_MODE);
		TS_LOG_INFO("goodix_gesture \n");
		if (true == g_goodix_dev_data->easy_wakeup_info.palm_cover_flag)
			g_goodix_dev_data->easy_wakeup_info.palm_cover_flag = false;
		goodix_put_device_into_easy_wakeup();
		mutex_lock(&wrong_touch_lock);
		g_goodix_dev_data->easy_wakeup_info.off_motion_on = true;
		mutex_unlock(&wrong_touch_lock);
		break;
	default:
		//goodix_switch_wrokmode(SLEEP_MODE);
		goodix_sleep_mode_in();
		break;
	}
	ts->enter_suspend = true;
	GTP_INFO("Suspend end");
	return 0;
}

/**
 * goodix_fw_update_boot - update firmware while booting
 */
static int goodix_fw_update_boot(char *file_name)
{
	int ret;
	if (goodix_ts == NULL || !file_name) {
		GTP_ERROR("Invalid file name");
		return -ENODEV;
	}
	memset(goodix_ts->firmware_name,0,64);
	sprintf(goodix_ts->firmware_name,"%s",GT9XX_FW_NAME);
	ret = gup_update_proc(UPDATE_TYPE_HEADER);
	if (ret != SUCCESS) {
                 TS_LOG_ERR("%s:firmware update fail, ret=%d\n", __func__, ret);
#if defined (CONFIG_HUAWEI_DSM)
                         if (!dsm_client_ocuppy(ts_dclient)) {
                                         dsm_client_record(ts_dclient,
                                                           "goodix fw update result: failed.\nupdata_status is %d.\n",
                                                          goodix_ts->goodix_device_data->ts_platform_data->dsm_info.constraints_UPDATE_status);
                                         dsm_client_notify(ts_dclient,DSM_TP_FWUPDATE_ERROR_NO);
                         }
                         strncpy(goodix_ts->goodix_device_data->ts_platform_data->dsm_info.fw_update_result,
                                         "failed", strlen("failed"));
#endif
	}
	goodix_read_version(&goodix_ts->hw_info);
	sprintf(g_gtp_vendor,
	        "FW:%02X%02X-Cfg:0x%x",
	        (goodix_ts->hw_info.patch_id>>8)&0xFF,(goodix_ts->hw_info.patch_id)&0xFF,goodix_ts->hw_info.config_ver);
	return ret;
}

/**
 * goodix_fw_update_sd - update firmware from sdcard
 *  firmware path should be '/sdcard/_goodix_update_.bin'
 */
static int goodix_fw_update_sd(void)
{
	int ret;
	if (goodix_ts == NULL) {
		GTP_ERROR("Invalid file name");
		return -ENODEV;
	}
	update_from_sd = true;
	memset(goodix_ts->firmware_name,0,64);
	sprintf(goodix_ts->firmware_name,"%s", AUTO_TEST_FW_NAME);
	ret = gup_update_proc(UPDATE_TYPE_HEADER);
	if (ret != SUCCESS) {
                 TS_LOG_ERR("%s:firmware update fail, ret=%d\n", __func__, ret);
#if defined (CONFIG_HUAWEI_DSM)
                         if (!dsm_client_ocuppy(ts_dclient)) {
                                         dsm_client_record(ts_dclient,
                                                           "goodix fw update result: failed.\nupdata_status is %d.\n",
                                                          goodix_ts->goodix_device_data->ts_platform_data->dsm_info.constraints_UPDATE_status);
                                         dsm_client_notify(ts_dclient,DSM_TP_FWUPDATE_ERROR_NO);
                         }
                         strncpy(goodix_ts->goodix_device_data->ts_platform_data->dsm_info.fw_update_result,
                                         "failed", strlen("failed"));
#endif
	}
	goodix_read_version(&goodix_ts->hw_info);
	sprintf(g_gtp_vendor,
                "FW:%02X%02X-Cfg:0x%x",
                (goodix_ts->hw_info.patch_id>>8)&0xFF,(goodix_ts->hw_info.patch_id)&0xFF,goodix_ts->hw_info.config_ver);
	return ret;
}

static int goodix_chip_get_info(struct ts_chip_info_param *info)
{
	struct goodix_ts_data *ts = goodix_ts;
	if (!info || !ts)
		return -EINVAL;
	sprintf(info->ic_vendor, "GT%c%c%c%c", ts->hw_info.product_id[0],
	ts->hw_info.product_id[1],ts->hw_info.product_id[2],ts->hw_info.product_id[3]);
	sprintf(info->fw_vendor, "FW ver: %02X%02X-Cfg ver: 0x%x",(ts->hw_info.patch_id>>8)&0xFF,(ts->hw_info.patch_id)&0xFF,goodix_ts->hw_info.config_ver);
	sprintf(info->mod_vendor, "SensorID:%X",ts->hw_info.sensor_id);
	return 0;
}

 void goodix_int_sync(s32 ms)
{
	int irq_gpio;
	irq_gpio = goodix_ts->goodix_device_data->ts_platform_data->irq_gpio;
	goodix_pinctr_int_ouput_low();//gpio_direction_output(irq_gpio, 0);
	msleep(ms);
	gpio_direction_input(irq_gpio);
}

 /**
 * goodix_chip_reset - reset chip
 */
void gt1x_select_addr(void)
{
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(20);
	GTP_GPIO_OUTPUT(GTP_INT_PORT, goodix_ts->goodix_device_data->ts_platform_data->client->addr == 0x14);
	msleep(20);
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
}

int goodix_chip_reset(void)
{
	GTP_INFO("GTP RESET!\n");
	/* select i2c address */
	gt1x_select_addr();
    msleep(10);		/*must >= 6ms*/
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(58);
	GTP_GPIO_AS_INT(GTP_INT_PORT);
    return goodix_init_watchdog();
}
/**
 * goodix_glove_switch - switch to glove mode
 */
static int goodix_glove_switch(struct ts_glove_info *info)
{
	static bool glove_en = false;
	int ret = 0;
	u8 buf = 0;

	if (!info || !goodix_ts) {
		GTP_ERROR("info is Null");
		return -ENOMEM;
	}

	switch (info->op_action) {
	case TS_ACTION_READ:
		if (glove_en)
			info->glove_switch = 1;
		else
			info->glove_switch = 0;
		break;
	case TS_ACTION_WRITE:
		if (info->glove_switch) {
			/* enable glove feature */
			ret = goodix_switch_config(GOODIX_GLOVE_CFG);
			if (!ret)
				glove_en = true;
		} else {
			/* disable glove feature */
			ret = goodix_switch_config(GOODIX_NORMAL_CFG);
			if (!ret)
				glove_en = false;
		}

		if (ret < 0)
			GTP_ERROR("set glove switch(%d), failed : %d", buf, ret);
		break;
	default:
		GTP_ERROR("invalid switch status: %d", info->glove_switch);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void goodix_chip_shutdown(void)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return;
	return;
}

static int goodix_palm_switch(struct ts_palm_info *info)
{
	if (goodix_ts == NULL)
		return -ENODEV;

	return 0;
}

static int goodix_holster_switch(struct ts_holster_info *info)
{
	int ret = 0;

	if (!info || !goodix_ts) {
		GTP_ERROR("holster_switch: info is Null\n");
		ret = -ENOMEM;
		return ret;
	}

	switch (info->op_action) {
		case TS_ACTION_WRITE:
			if (info->holster_switch)
				ret = goodix_switch_config(GOODIX_HOLSTER_CFG);
			else
				ret = goodix_switch_config(GOODIX_NORMAL_CFG);
			if (ret < 0)
				GTP_ERROR("set holster switch(%d), failed: %d",
							info->holster_switch, ret);
			break;
		case TS_ACTION_READ:
			GTP_INFO("invalid holster switch(%d) action: TS_ACTION_READ",
							info->holster_switch);
			break;
		default:
			GTP_INFO("invalid holster switch(%d) action: %d\n",
							info->holster_switch, info->op_action);
			ret = -EINVAL;
			break;
	}

	return ret;
}

static int goodix_esdcheck_tp_reset(void)
{
	u8 esd_buf[5] = {0};
	struct goodix_ts_data *ts = goodix_ts;
	int ret = 0;

	esd_buf[0] = GTP_ESD_RESET_VALUE1;
	esd_buf[1] = GTP_ESD_RESET_VALUE2;
	esd_buf[2] = GTP_ESD_RESET_VALUE3;
	esd_buf[3] = GTP_ESD_RESET_VALUE3;
	esd_buf[4] = GTP_ESD_RESET_VALUE3;

	if (ts == NULL){
		TS_LOG_ERR("ts is NULL\n");
		return 0;
	}

	ret = goodix_i2c_write(GTP_REG_CMD, esd_buf, 5);
	if(ret < 0){
		TS_LOG_ERR("%s: goodix_i2c_write  fail\n",__func__);
	}
	msleep(50);

	ret = goodix_chip_reset();
	if(ret < 0){
		TS_LOG_ERR("%s: goodix_chip_reset  fail\n",__func__);
	}
	msleep(50);

	ret = goodix_send_cfg(&ts->normal_config);
	if(ret < 0){
		TS_LOG_ERR("%s: goodix_send_cfg  fail\n",__func__);
	}
	if(ret < 0)
	{
		#ifdef CONFIG_HUAWEI_DSM
		if (!dsm_client_ocuppy(ts_dclient)) {
			dsm_client_record(ts_dclient,"goodix esd result: failed.\esd_status is %d.\n",ret);
			dsm_client_notify(ts_dclient,DSM_TP_ESD_ERROR_NO);
		}
		#endif
	}
	return 0;
}

static int goodix_esdcheck_func(void)
{
	struct goodix_ts_data *ts =  goodix_ts;
	u8 esd_buf[4] = {0};
	u8 chk_buf[4] = {0};
	int ret = 0;
	int i = 0;

	esd_buf[0] = GTP_REG_CMD_ADDR1;
	esd_buf[1] = GTP_REG_CMD_ADDR2;
	chk_buf[0] = GTP_REG_CMD_ADDR1;
	chk_buf[1] = GTP_REG_CMD_ADDR2;

	if (ts == NULL){
		TS_LOG_ERR("%s: ts is NULL \n",__func__);
		return 0;
	}

	if (ts->enter_suspend || ts->enter_update || ts->enter_rawtest){
		TS_LOG_INFO("%s: Esd suspended \n",__func__);
		return ret;
	}

	for (i = 0; i < CHECK_HW_STATUS_RETRY; i++){
		ret = goodix_i2c_read(GTP_REG_CMD, esd_buf, 4);
		if (ret < 0){
			/* IIC communication problem */
			TS_LOG_ERR("%s: goodix_i2c_read  fail!\n",__func__);
			continue;
		}else{
			if ((esd_buf[0] == GTP_CMD_ESD_CHECK) || (esd_buf[1] != GTP_CMD_ESD_CHECK)){
				/* ESD check IC works abnormally */
				goodix_i2c_read(GTP_REG_CMD, chk_buf, 4);
				TS_LOG_ERR("%s,%d:[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X",__func__,__LINE__, chk_buf[0], chk_buf[1]);
				if ((chk_buf[0] == GTP_CMD_ESD_CHECK) || (chk_buf[1] != GTP_CMD_ESD_CHECK)){
				    i = CHECK_HW_STATUS_RETRY;
				    break;
				}else{
				    continue;
				}
			}else{
				/* IC works normally, Write 0x8040 0xAA, feed the dog */
				esd_buf[0] = GTP_CMD_ESD_CHECK;
				ret = goodix_i2c_write(GTP_REG_CMD, esd_buf, 3);
				if(ret < 0){
					TS_LOG_ERR("%s: goodix_i2c_write  fail!\n",__func__);
					continue;
				}
				break;
			}
		}
		TS_LOG_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X\n", esd_buf[0], esd_buf[1]);
	}

	if (i >= CHECK_HW_STATUS_RETRY){
		TS_LOG_INFO("%s: IC working abnormally! Process reset guitar\n", __func__);
		goodix_esdcheck_tp_reset();
	}

	return 0;
}
/*
 * goodix_check_hw_status - Hw exception checking
 */
static int goodix_check_hw_status(void)
{
	 return goodix_esdcheck_func();
}

#ifdef ROI
static int goodix_roi_switch(struct ts_roi_info *info)
{
	int ret = 0;

	if (!info || !goodix_ts) {
		GTP_ERROR("roi_switch: info is Null");
		ret = -ENOMEM;
		return ret;
	}

	switch (info->op_action) {
	case TS_ACTION_WRITE:
		if (info->roi_switch == 1) {
			goodix_ts->roi.enabled = true;
		} else if (info->roi_switch == 0) {
			goodix_ts->roi.enabled = false;
		} else {
			GTP_ERROR("Invalid roi switch value:%d", info->roi_switch);
			ret = -EINVAL;
		}
		break;
	case TS_ACTION_READ:
		break;
	default:
		break;
	}
	return ret;
}

static u8* goodix_roi_rawdata(void)
{
	u8 * rawdata_ptr = NULL;

	if (goodix_ts == NULL)
		return NULL;

	mutex_lock(&goodix_ts->roi.mutex);
	if (goodix_ts->roi.enabled && goodix_ts->roi.data_ready)
		rawdata_ptr = (u8 *)goodix_ts->roi.rawdata;
	mutex_unlock(&goodix_ts->roi.mutex);

	return rawdata_ptr;
}
#endif

static int goodix_chip_get_capacitance_test_type(
				struct ts_test_type_info *info)
{
	int ret = 0;

	if (!info) {
		GTP_ERROR("info is Null");
		ret = -ENOMEM;
		return ret;
	}

	switch (info->op_action) {
	case TS_ACTION_READ:
		memcpy(info->tp_test_type,
		       goodix_ts->goodix_device_data->tp_test_type,
		       TS_CAP_TEST_TYPE_LEN);
		GTP_INFO("test_type=%s",info->tp_test_type);
		break;
	case TS_ACTION_WRITE:
		break;
	default:
		GTP_ERROR("invalid status: %s", info->tp_test_type);
		ret = -EINVAL;
		break;
	}
	return ret;
}
struct ts_kit_device_data *goodix_get_device_data(void)
{
	return g_goodix_dev_data;
}

struct goodix_ts_data *goodix_get_platform_data(void)
{
	return goodix_ts;
}

static int
	goodix_chip_detect(struct ts_kit_platform_data *pdata)
{
	int ret = NO_ERR;
	int slave_addr = 0;

    if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
    {
        return -EIO;
    }
	if (!pdata){
		TS_LOG_ERR("%s device, ts_kit_platform_data *data or data->ts_dev is NULL \n", __func__);
		ret = -ENOMEM;
		goto exit;
	}
	GTP_INFO("Chip detect");
	goodix_ts->goodix_device_data = g_goodix_dev_data;
	goodix_ts->ts_platform_data = pdata;
	goodix_ts->pdev = pdata->ts_dev;
	g_goodix_dev_data->ts_platform_data = pdata;
	goodix_ts->pdev->dev.of_node = g_goodix_dev_data->cnode;

	ret = goodix_prase_ic_config_dts(g_goodix_dev_data->cnode, g_goodix_dev_data);
	if (ret) {
		TS_LOG_ERR("%s:parse ic config dts fail, ret=%d\n",
			__func__, ret);
	}

	pdata->client->addr =g_goodix_dev_data->slave_addr;

	if ((!pdata) &&(!pdata->ts_dev)){
		TS_LOG_ERR("%s device, ts_kit_platform_data *data or data->ts_dev is NULL \n", __func__);
		ret = -ENOMEM;
		//goto exit;
	}

	g_goodix_dev_data->is_i2c_one_byte = 0;
	g_goodix_dev_data->is_new_oem_structure= 0;
	g_goodix_dev_data->is_parade_solution= 0;
	goodix_ts->ops.i2c_read = goodix_i2c_read;
	goodix_ts->ops.i2c_write = goodix_i2c_write;
	goodix_ts->ops.chip_reset = goodix_chip_reset;
	goodix_ts->ops.send_cmd = goodix_send_cmd;
	goodix_ts->ops.send_cfg = goodix_send_cfg;
	goodix_ts->ops.i2c_read_dbl_check = goodix_i2c_read_dbl_check;
	goodix_ts->ops.read_version = goodix_read_version;
	goodix_ts->ops.parse_cfg_data = goodix_parse_cfg_data;
	goodix_ts->tools_support = true;
	mutex_init(&goodix_ts->mutex_cfg);
	mutex_init(&goodix_ts->mutex_cmd);
	mutex_init(&wrong_touch_lock);
	/* Do *NOT* remove these logs */
      GTP_INFO("Driver Version: %s", GTP_DRIVER_VERSION);
      GTP_INFO("pdata->client->addr =0x%x",pdata->client->addr);
	msleep(20);
	hw_ts_power_switch(SWITCH_ON);
	msleep(100);    //zhangle
#if 0
	goodix_parse_dts(g_goodix_dev_data->cnode,goodix_ts);
#endif
	ret = goodix_chip_reset();
	if (ret < 0)
		goto err_power_on;
	ret = goodix_i2c_test();
	if (ret < 0)
		goto err_power_on;
#ifdef CONFIG_HUAWEI_HW_I2C_DCT
	/* detect current device successful, set the flag as present */
	set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
#endif

	TS_LOG_INFO("%s:goodix chip detect success\n", __func__);
	REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
	return 0;

err_power_on:
	//hw_ts_power_switch(SWITCH_OFF);
exit:
	return ret;
}

struct ts_device_ops ts_goodix_ops = {
	.chip_detect = goodix_chip_detect,
	.chip_init = goodix_chip_init,
	.chip_irq_top_half = goodix_irq_top_half,
	.chip_parse_config = goodix_chip_parse_config,
	.chip_input_config = goodix_input_config,
	.chip_irq_bottom_half = goodix_irq_bottom_half,
	.chip_reset = goodix_chip_reset,
	.chip_fw_update_boot = goodix_fw_update_boot,
	.chip_fw_update_sd = goodix_fw_update_sd,
	.chip_get_info = goodix_chip_get_info,
//    .chip_set_info_flag = goodix_set_info_flag,
	.chip_before_suspend = goodix_before_suspend,
	.chip_suspend = goodix_chip_suspend,
	.chip_resume = goodix_chip_resume,
	.chip_get_rawdata = goodix_get_rawdata,
	.chip_glove_switch = goodix_glove_switch,
	.chip_shutdown = goodix_chip_shutdown,
	.chip_palm_switch = goodix_palm_switch,
	.chip_holster_switch = goodix_holster_switch,
#ifdef ROI
	.chip_roi_switch = goodix_roi_switch,
	.chip_roi_rawdata = goodix_roi_rawdata,
#endif
	.chip_check_status = goodix_check_hw_status,
	.chip_get_capacitance_test_type =
			goodix_chip_get_capacitance_test_type,
//    .chip_regs_operate = goodix_regs_operate,
#if defined (CONFIG_HUAWEI_DSM)
//    .chip_dsm_debug = goodix_dsm_debug,
#endif

#ifdef HUAWEI_TOUCHSCREEN_TEST
//    .chip_test = test_dbg_cmd_test,
#endif
//    .chip_wrong_touch=goodix_wrong_touch,
};

static int __init goodix_ts_module_init(void)
{
	int ret = NO_ERR;
	bool found = false;
	struct device_node *child = NULL;
	struct device_node *root = NULL;

	GTP_INFO("%s: called", __func__);

	goodix_ts =
		kzalloc(sizeof(struct goodix_ts_data), GFP_KERNEL);
	if (NULL == goodix_ts) {
		GTP_ERROR("%s:alloc mem for device data fail\n", __func__);
		ret = -ENOMEM;
		goto error_exit;
	}

	g_goodix_dev_data =
		kzalloc(sizeof(struct ts_kit_device_data), GFP_KERNEL);
	if (NULL == g_goodix_dev_data) {
		GTP_ERROR("%s:alloc mem for ts_kit_device data fail\n", __func__);
		ret = -ENOMEM;
		goto error_exit;
	}

	root = of_find_compatible_node(NULL, NULL, HUAWEI_TS_KIT);
	if (!root) {
		TS_LOG_ERR("%s:find_compatible_node error\n", __func__);
		ret = -EINVAL;
		goto error_exit;
	}

	for_each_child_of_node(root, child) {
		if (of_device_is_compatible(child, GTP_CHIP_NAME)) {
			found = true;
			break;
		}
	}

	if (!found) {
		TS_LOG_ERR("%s:device tree node not found, name=%s\n",
			__func__, GTP_CHIP_NAME);
		ret = -EINVAL;
		goto error_exit;
	}

	g_goodix_dev_data->cnode = child;
	g_goodix_dev_data->ops = &ts_goodix_ops;

	ret = huawei_ts_chip_register(g_goodix_dev_data);
	if (ret) {
		TS_LOG_ERR("%s:chip register fail, ret=%d\n", __func__, ret);
		goto error_exit;
	}
	tpd_load_status = 1;
	TS_LOG_INFO("%s:success\n", __func__);
	return 0;

error_exit:
	if(NULL != goodix_ts){
		kfree(goodix_ts);
	}	//kfree(g_goodix_dev_data);
	goodix_ts = NULL;
	kfree(g_goodix_dev_data);
	g_goodix_dev_data = NULL;
	TS_LOG_INFO("%s:fail\n", __func__);
	return ret;
}

static void __exit goodix_ts_module_exit(void)
{
	kfree(goodix_ts);
	goodix_ts = NULL;
	kfree(g_goodix_dev_data);
	g_goodix_dev_data = NULL;

	return;
}

late_initcall(goodix_ts_module_init);
module_exit(goodix_ts_module_exit);
MODULE_AUTHOR("Huawei Device Company");
MODULE_DESCRIPTION("Huawei TouchScreen Driver");
MODULE_LICENSE("GPL");
