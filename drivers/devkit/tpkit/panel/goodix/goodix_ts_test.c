/*
 * goodix_ts_test.c - TP O/S test
 *
 * Copyright (C) 2015 - 2016 Goodix Technology Incorporated
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
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include "goodix_ts.h"
#include "goodix_dts.h"
#include "gt1x_config.h"
#include <linux/syscalls.h>
#define DRV_NUM_OFFSET	(0x8062 - 0x8047)
#define SEN_NUM_OFFSET	(0x8064 - 0x8047)
#define KEY_NUM_OFFSET	(0x80A3 - 0x8047)
#define MODULE_SWITCH_DK_OFFSET	(0x804E - 0x8047)
#define MODULE_SWITCH_SK_OFFSET	(0x805A - 0x8047)

#define MAX_DRV_NUM		30
#define MAX_SEN_NUM		42
#define FLOAT_AMPLIFIER 1000

#define PACKAGE_SIZE    256

/*----------------------------------error_type--------------------------*/

#define INI_FILE_OPEN_ERR       ((0x1<<0)|0x80000000)

#define INI_FILE_READ_ERR       ((0x2<<0)|0x80000000)
#define INI_FILE_ILLEGAL        ((0x4<<0)|0x80000000)

		      // -2147483640
#define SHORT_TEST_ERROR        ((0x8<<0)|0x80000000)
#define CFG_FILE_OPEN_ERR       ((0x1<<4)|0x80000000)
#define CFG_FILE_READ_ERR       ((0x2<<4)|0x80000000)
#define CFG_FILE_ILLEGAL        ((0x4<<4)|0x80000000)
#define UPDATE_FILE_OPEN_ERR    ((0x1<<8)|0x80000000)
#define UPDATE_FILE_READ_ERR    ((0x2<<8)|0x80000000)
#define UPDATE_FILE_ILLEGAL     ((0x4<<8)|0x80000000)
#define FILE_OPEN_CREATE_ERR    ((0x1<<12)|0x80000000)
#define FILE_READ_WRITE_ERR     ((0x2<<12)|0x80000000)
#define FILE_ILLEGAL            ((0x4<<12)|0x80000000)
#define NODE_OPEN_ERR           ((0x1<<16)|0x80000000)
#define NODE_READ_ERR           ((0x2<<16)|0x80000000)
#define NODE_WRITE_ERR          ((0x4<<16)|0x80000000)
#define DRIVER_NUM_WRONG        ((0x1<<20)|0x80000000)
#define SENSOR_NUM_WRONG        ((0x2<<20)|0x80000000)
#define PARAMETERS_ILLEGL       ((0x4<<20)|0x80000000)
#define I2C_UNLOCKED            ((0x8<<20)|0x80000000)
#define MEMORY_ERR              ((0x1<<24)|0x80000000)
#define I2C_TRANS_ERR           ((0x2<<24)|0x80000000)
#define COMFIRM_ERR             ((0x4<<24)|0x80000000)
#define ENTER_UPDATE_MODE_ERR   ((0x8<<24)|0x80000000)
#define VENDOR_ID_ILLEGAL       ((0x1<<28)|0x80000000)
#define STORE_ERROR             ((0x2<<28)|0x80000000)
#define NO_SUCH_CHIP_TYPE       ((0x4<<28)|0x80000000)

#define FILE_NOT_EXIST          ((0x8<<28)|0x80000000)

#define _CHANNEL_PASS               0x0000
#define _BEYOND_MAX_LIMIT           0x0001
#define _BEYOND_MIN_LIMIT           0x0002
#define _BEYOND_ACCORD_LIMIT        0x0004
#define _BEYOND_OFFSET_LIMIT        0x0008
#define _BEYOND_JITTER_LIMIT        0x0010
#define _SENSOR_SHORT               0x0020
#define _DRIVER_SHORT               0x0040
#define _COF_FAIL                   0x0080
#define _I2C_ADDR_ERR               0x0100
#define _CONFIG_MSG_WRITE_ERR       0x0200
#define _GET_RAWDATA_TIMEOUT        0x0400
#define _GET_TEST_RESULT_TIMEOUT    0x0800
#define _KEY_BEYOND_MAX_LIMIT       0x1000
#define _KEY_BEYOND_MIN_LIMIT       0x2000
#define _INT_ERROR                  0x4000
#define _TEST_NG                    0x00008000
#define _VERSION_ERR                0x00010000
#define _RESET_ERR                  0x00020000
#define _CURRENT_BEYOND_MAX_LIMIT   0x00040000
#define _MODULE_TYPE_ERR            0x00080000
#define _MST_ERR                    0x00100000
#define _NVRAM_ERR                  0x00200000
#define _GT_SHORT                   0x00400000
#define _BEYOND_UNIFORMITY_LIMIT    0x00800000
#define _BETWEEN_ACCORD_AND_LINE    0x40000000

/*----------------------------------test_types--------------------------*/

#define _NEED_CHECK                     0x00
#define _NEED_NOT_CHECK                 0x01
#define _MAX_CHECK                      0x0001
#define _MIN_CHECK                      0x0002
#define _ACCORD_CHECK                   0x0004
#define _KEY_MAX_CHECK                  0x0100
#define _KEY_MIN_CHECK                  0x0200
#define _MODULE_SHORT_CHECK             0x00400000

/*------------------------------------ SHORT TEST PART--------------------------------------*/
#define _bRW_MISCTL__SRAM_BANK          0x4048
#define _bRW_MISCTL__PATCH_AREA_EN_  	0x404D
#define _bRW_MISCTL__MEM_CD_EN          0x4049
#define _bRW_MISCTL__CACHE_EN           0x404b
#define _bRW_MISCTL__TMR0_EN            0x40b0
#define _rRW_MISCTL__SWRST_B0_          0x4180
#define _bWO_MISCTL__CPU_SWRST_PULSE    0x4184
#define _rRW_MISCTL__BOOTCTL_B0_        0x4190
#define _rRW_MISCTL__BOOT_OPT_B0_       0x4218
#define _bRW_MISCTL__RG_OSC_CALIB       0x4268
#define _rRW_MISCTL__BOOT_CTL_          0x5094	//0x4283
#define _rRW_MISCTL__SHORT_BOOT_FLAG    0x5095	//0x4283

#define SHORT_FILE_PATH		"ts/goodix_shortcircut.bin"
#define _CHANNEL_TX_FLAG                0x80
#define _GT9_SHORT_TEST_ERR             88
unsigned short gt900_resistor_warn_threshold = 0;
struct firmware *tptest_limit = NULL;
/**
 * struct ts_test_params - test parameters
 * drv_num: touch panel tx(driver) number
 * sen_num: touch panel tx(sensor) number
 * sc_drv_num: screen tx number
 * sc_sen_num: screen rx number
 * key_num: key number
 * max_limits: max limits of rawdata
 * min_limits: min limits of rawdata
 * deviation_limits: channel deviation limits
 * short_threshold: short resistance threshold
 * r_drv_drv_threshold: resistance threshold between drv and drv
 * r_drv_sen_threshold: resistance threshold between drv and sen
 * r_sen_sen_threshold: resistance threshold between sen and sen
 * r_drv_gnd_threshold: resistance threshold between drv and gnd
 * r_sen_gnd_threshold: resistance threshold between sen and gnd
 * avdd_value: avdd voltage value
 */
struct ts_test_params {
	int test_types;
	u32 drv_num;
	u32 sen_num;
	u32 sc_drv_num;
	u32 sc_sen_num;
	u32 key_num;

	u32 max_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	u32 min_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	u32 key_max_limits[42];
	u32 key_min_limits[42];
	unsigned char channel_key_need_check[42];
	u32 deviation_limits[MAX_DRV_NUM * MAX_SEN_NUM];

	u16 short_threshold;
	u16 r_drv_drv_threshold;
	u16 r_drv_sen_threshold;
	u16 r_sen_sen_threshold;
	u16 r_drv_gnd_threshold;
	u16 r_sen_gnd_threshold;
	u16 avdd_value;
};

typedef struct {
	unsigned char master;	// Pin No.
	unsigned char position;	//If TX Pin No. > 26, position = master-1 as SE1.
	unsigned char slave;
	unsigned short short_code;
}strShortRecord;
static const u8 ChannelPackage_TX[42] =  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,
                                         20,21,22,23,24,25,/*26,*/27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42};


/**
 * struct ts_test_rawdata - rawdata structure
 * addr: rawdata address in chip ram
 * data: rawdata buffer, u16, big-endian
 * size: rawdata size
 */
struct ts_test_rawdata {
	u16 addr;
	u16 data[MAX_SEN_NUM * MAX_DRV_NUM];
	u32 size;
};

/**
 * struct goodix_ts_test - main data structrue
 * ts: goodix touch screen data
 * test_config: test mode config data
 * orig_config: original config data
 * test_param: test parameters
 * rawdata: raw data structure
 * product_id: product id infomation
 * test_result: test result string
 * failed_reason: test failed reason
 */
struct goodix_ts_test {
	struct goodix_ts_data *ts;
	struct goodix_ts_config test_config;
	struct goodix_ts_config orig_config;
	struct ts_test_params test_params;
	struct ts_test_rawdata rawdata;
	struct ts_test_rawdata basedata;
	const char *product_id;
	char test_result[TS_RAWDATA_RESULT_MAX];
	char failed_reason[TS_RAWDATA_RESULT_MAX];
};

static struct goodix_ts_test *gts_test;
const struct firmware *short_fw;
#define _GT9_UPLOAD_SHORT_TOTAL         15   //Max == 10
static unsigned char* upload_short_data;
static unsigned char* warn_short_data;
static unsigned short* self_data;
/*------------------------------------ FUNCTION DECLARE--------------------------------------*/
static int goodix_shortcircut_test(struct goodix_ts_test *ts_test);
static s32 gt9xx_disable_hopping(struct goodix_ts_test *ts_test,struct goodix_ts_config *cfg,int flag);
static int gt9x_short_test_end(struct goodix_ts_test *ts_test);

/**
 * goodix_hw_testmode - switch firmware to test mode
 */
static inline int goodix_hw_testmode(struct goodix_ts_test *ts_test)
{
	int ret = -ENODEV;
	u8 buf[1];
	buf[0] = 0x01;

	GTP_INFO("goodix_hw_testmode 0x8040 write 0x01");

	if(ts_test->ts->ops.i2c_write)
		ret = ts_test->ts->ops.i2c_write(GTP_REG_CMD, &buf[0], 1);

	msleep(1);
	GTP_INFO("goodix_hw_testmode 0x814E write 0x00");

	buf[0] = 0x00;
	ts_test->ts->ops.i2c_write(GTP_READ_COOR_ADDR, &buf[0], 1);

	GTP_INFO("clear 0x814e 00.");

	return ret;
}

/**
 * gt9xx_parse_config - resolve tx/rx number and key number
 */
static int gt9xx_parse_config(struct goodix_ts_config *test_config,
				struct ts_test_params *test_params)
{
	int i;
	GTP_ERROR("gt9xx_parse_config");
	test_params->drv_num = (test_config->data[DRV_NUM_OFFSET] & 0x1F)
					+ (test_config->data[DRV_NUM_OFFSET + 1] & 0x1F);
	test_params->sen_num = (test_config->data[SEN_NUM_OFFSET] & 0x0F)+((test_config->data[SEN_NUM_OFFSET] >>4)&0x0F);

	if(test_config->data[MODULE_SWITCH_DK_OFFSET] & 0x01){  /* has key */
		if ((test_config->data[MODULE_SWITCH_DK_OFFSET]>>6) & 0x01) {
			test_params->sc_drv_num = test_params->drv_num;
			test_params->sc_sen_num = test_params->sen_num - ((test_config->data[MODULE_SWITCH_DK_OFFSET]>>6) & 0x01);
		} else {
			test_params->sc_sen_num = test_params->sen_num;
			test_params->sc_drv_num = test_params->drv_num - ((test_config->data[MODULE_SWITCH_DK_OFFSET]>>6) & 0x01);
		}
	} else {
		test_params->sc_drv_num = test_params->drv_num;
		test_params->sc_sen_num = test_params->sen_num;
	}

	test_params->key_num = (test_params->drv_num* test_params->sen_num - test_params->sc_sen_num*test_params->sc_drv_num) ;

	GTP_INFO("tx_num:%d,rx_num:%d,sc_tx_num:%d,sc_rx_num:%d,key_num:%d",
		test_params->drv_num, test_params->sen_num,
		test_params->sc_drv_num, test_params->sc_sen_num,
		test_params->key_num);

	return 0;
}

 s32 _node_in_key_chn(u16 node, u8 * config, u16 keyoffest)
{
	int ret = -1;
	u8 i, tmp, chn;

	int n;
	if (node < 0) {
		return -1;
	}

	n = 4;

	chn = node;
	for (i = 0; i < n; i++) {
		tmp = config[keyoffest + i];
		if ((tmp != 0) && (tmp % 8 != 0)) {
			return 1;
		}

		if (tmp == (chn + 1) * 8) {
			ret = 1;
		}
	}

	return ret;
}


static unsigned char _check_key_min_value(struct ts_test_rawdata *rawdata,struct goodix_ts_test *ts_test)
{
	unsigned int i, offset, key_cnt = 0;;
	unsigned char test_result = 1;
	int max_key_test_cnt;

	if (ts_test->test_params.key_num == 0) {
		return test_result;
	}

	offset = ts_test->test_params.sc_sen_num * ts_test->test_params.sc_drv_num;

	max_key_test_cnt = ts_test->test_params.key_num > 4 ? 4 : ts_test->test_params.key_num;

	GTP_INFO("max_key_test_cnt is %d in key-min",max_key_test_cnt);
	for (i = 0; i < 42; i++) {
		// check nc node
		if (ts_test->test_params.channel_key_need_check[i] == _NEED_NOT_CHECK)
			continue;

		// is key ?
		if (_node_in_key_chn(i, &ts_test->test_config.data[0], 76) < 0)
			continue;

		if (rawdata->data[offset + i] < (ts_test->test_params.key_min_limits[i])) {
			test_result = -1;
		}

		if (++key_cnt == max_key_test_cnt)
			break;

	}

	return test_result;
}

static unsigned char _check_key_max_value(struct ts_test_rawdata *rawdata,struct goodix_ts_test *ts_test)
{
	int i, offset, key_cnt = 0;
	unsigned char test_result = 1;
	int max_key_test_cnt;

	if (ts_test->test_params.key_num == 0) {
		return test_result;
	}


	offset = ts_test->test_params.sc_sen_num * ts_test->test_params.sc_drv_num;

	max_key_test_cnt = ts_test->test_params.key_num > 4 ? 4 : ts_test->test_params.key_num;

	//DEBUG("max_key_test_cnt is %d in key-max",max_key_test_cnt);
	for (i = 0; i < 42; i++) {

		if (ts_test->test_params.channel_key_need_check[i] == _NEED_NOT_CHECK)
			continue;

		if (_node_in_key_chn(i, &ts_test->test_config.data[0], 76) < 0)
			continue;

		//DEBUG("key min chk:i=%d, j=%d", i, j);
		//DEBUG("key min chk: rawdata:%d, limit:%d" ,current_data_temp[offset + j], max_key_limit_value[i]);

		if (rawdata->data[offset + i] > ts_test->test_params.key_max_limits[i]) {
			test_result = -1;
			break;
		}

		if (++key_cnt == max_key_test_cnt)
			break;
	}

	return test_result;
}


/**
 * goodix_init_testlimits - get test limits(max/min/...) from dt
 */
static int goodix_init_testlimits(struct goodix_ts_test *ts_test)
{
	struct device_node *device;
	char project_id[20];
	int ret, i, size, reorder, row, column,offset,point,m;
	u32 value[8];
	char otp_id;
	u32 test_types[1];
	u32 key_test[2];
	u8 key_nc[42];
	u8 data;
	int index_key;
	GTP_DEBUG("Get test limits from dt");
	//sprintf(project_id, "gtp-%s","DLI45210");
	//sprintf(project_id, "gtp-%s",ts_test->ts->hw_info.sensor_id);
	/*device = of_find_compatible_node(NULL, NULL, project_id);
	if (!device) {
		GTP_ERROR("No chip specific dts %s, need to parse",project_id);
		return -EINVAL;
	}*/
    device = g_goodix_dev_data->cnode;

	ret = of_property_read_u32(device, "goodix,test-types",
					 &test_types[0]);
	if (ret < 0) {
		GTP_ERROR("Failed to read raw data DEVIATION limits from dt:%d", ret);
		return ret;
	}
	ts_test->test_params.test_types = test_types[0];

	if(test_types[0]& _MAX_CHECK){
		ret = of_property_read_u32(device, "goodix,panel-max",
						 &value[0]);
		if (ret < 0) {
			GTP_ERROR("Failed to read raw data MAX limits from dt:%d", ret);
			return ret;
		}
	}
	if(test_types[0]& _MIN_CHECK){
		ret = of_property_read_u32(device, "goodix,panel-min",
					 &value[1]);
		if (ret < 0) {
			GTP_ERROR("Failed to read raw data MIN limits from dt:%d", ret);
			return ret;
		}
	}
	if (test_types[0] & _ACCORD_CHECK) {
		ret = of_property_read_u32(device, "goodix,panel-deviation",
					 &value[2]);
		if (ret < 0) {
			GTP_ERROR("Failed to read raw data DEVIATION limits from dt:%d", ret);
			return ret;
		}
	}

	if (test_types[0] & _KEY_MAX_CHECK) {
		ret = of_property_read_u32(device, "goodix,key-max",
					 &key_test[0]);
		if (ret < 0) {
			GTP_ERROR("Failed to read raw data DEVIATION limits from dt:%d", ret);
			return ret;
		}
	}

	if (test_types[0] & _KEY_MIN_CHECK) {
		ret = of_property_read_u32(device, "goodix,key-min",
					 &key_test[1]);
		if (ret < 0) {
			GTP_ERROR("Failed to read raw data DEVIATION limits from dt:%d", ret);
			return ret;
		}
	}

	for (i = 0; i < MAX_SEN_NUM * MAX_DRV_NUM; i++) {
		ts_test->test_params.max_limits[i] = value[0];
		ts_test->test_params.min_limits[i] = value[1];
		ts_test->test_params.deviation_limits[i] = value[2];
	}
	GTP_DEBUG("raw_data_limit:%u %u %u", value[0],value[1], value[2]);
	for (i = 0; i < 42; i++) {
		ts_test->test_params.key_max_limits[i] = key_test[0];
		ts_test->test_params.key_min_limits[i] = key_test[1];
	}


	if ((test_types[0] & _KEY_MIN_CHECK)||(test_types[0] & _KEY_MIN_CHECK)) {
		ret = of_property_read_u32_array(device, "goodix,key-nc",
					 &key_nc[0],42);

		if (ret < 0) {
			GTP_ERROR("Failed to read raw data DEVIATION limits from dt:%d", ret);
			return ret;
		}

		memset(&ts_test->test_params.channel_key_need_check[0], 0, 42);

		point = 0;
	index_key = 0;
		for (i = 0; i < 10; i++) {

			data = key_nc[index_key];

			for (m = 0; m < 8; m++) {

				if (point >= 42) {

					return;

				}

				ts_test->test_params.channel_key_need_check[point] &= 0xfe;

				if ((data & 0x80) == 0x80) {

					ts_test->test_params.channel_key_need_check[point] |= 0x01;

				}

				data <<= 1;

				point++;

			}

			index_key++;

		}


		if(ts_test->test_params.key_num != 0) {

			offset = ts_test->test_params.sc_sen_num* ts_test->test_params.sc_drv_num;
			for (i = 0; i < 42; i++) {
				if (ts_test->test_params.channel_key_need_check[i] == _NEED_NOT_CHECK ||
						_node_in_key_chn(i, &ts_test->test_config.data[0], 76) < 0) {//76 is key offset
					ts_test->test_params.max_limits[offset + i] = 32767;
					ts_test->test_params.min_limits[offset + i] = 0;
				} else {
					ts_test->test_params.max_limits[offset + i] = ts_test->test_params.key_max_limits[i];
					ts_test->test_params.min_limits[offset + i] = ts_test->test_params.key_min_limits[i];
				}
			}
		}
	}

	size = of_property_count_u32_elems(device,
			"special_raw_data_limit");
	if (size > 0 && (size % 4) == 0) {
		int nodes = size / 4;
		u32 *array = kmalloc(size * sizeof(u32), GFP_KERNEL);

		if (!array) {
			GTP_ERROR("Failed to parse special_raw_data_limit");
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(device,
				"goodix,special_raw_data_limit", array, size);
		if (ret < 0) {
			GTP_ERROR("Failed to parse special_raw_data_limit");
			kfree(array);
			return ret;
		}

		for (i = 0; i < nodes; i++) {
			row = i / ts_test->test_params.sc_sen_num;
			column = i % ts_test->test_params.sc_sen_num;
			reorder = column * ts_test->test_params.sc_drv_num + row;
			if (array[reorder * 4]  < MAX_SEN_NUM * MAX_DRV_NUM) {
				ts_test->test_params.max_limits[i]=
						array[reorder * 4 + 1];
				ts_test->test_params.min_limits[i] =
						array[reorder * 4 + 2];
				ts_test->test_params.deviation_limits[i] =
						array[reorder * 4 + 3];
			} else {
				GTP_ERROR("Failed to reorder special_raw_data_limit");
				break;
			}
		}
		kfree(array);
	}
	if(test_types[0] & _MODULE_SHORT_CHECK){

		ret = of_property_read_u32_array(device,
						"goodix,shortcircut-threshold", &value[0], 7);
		if (ret < 0) {
			GTP_ERROR("Failed to read shortcircut threshold from dt:%d", ret);
			return ret;
		} else {
			ts_test->test_params.short_threshold = value[0];
			ts_test->test_params.r_drv_drv_threshold = value[1];
			ts_test->test_params.r_drv_sen_threshold = value[2];
			ts_test->test_params.r_sen_sen_threshold = value[3];
			ts_test->test_params.r_drv_gnd_threshold = value[4];
			ts_test->test_params.r_sen_gnd_threshold = value[5];
			ts_test->test_params.avdd_value = value[6];
			GTP_DEBUG("raw_data_limit:%u %u %u", value[0],
				value[1], value[2]);
		}
	}
	return 0;
}
static int gt9xx_get_testlimits_from_ini(struct goodix_ts_test *ts_test)
{
	int  i,j=0,retval = 0;
	struct device_node *device;
	u32 test_types[1];
	int buf_offset = strlen(RAWDATA_TEST_START_LABEL);
    device = g_goodix_dev_data->cnode;

  	retval = of_property_read_u32(device, "goodix,test-types",
					 &test_types[0]);
	if (retval < 0) {
		GTP_ERROR("Failed to read raw data DEVIATION limits from dt:%d", retval);
		return retval;
	}
	ts_test->test_params.test_types = test_types[0];
	retval = request_firmware(&tptest_limit, "ts/jimmy_goodix_rawdata_test.ini",&ts_test->ts->pdev->dev);
	if (retval < 0) {
		TS_LOG_ERR("%s: Fail request rawdata_test.ini\n", __func__);
		goto exit;
	}
	if (tptest_limit == NULL) {
		TS_LOG_ERR("%s: tptest_limit.ini == NULL\n", __func__);
		retval = -1;
		goto exit;
	}
	if (tptest_limit->data == NULL || tptest_limit->size == 0) {
		TS_LOG_ERR("%s: No tptest_limit received\n", __func__);
		retval = -2;
		goto exit;
	}
	TS_LOG_INFO("%s: tptest_limit->size = %zu\n", __func__, tptest_limit->size);
	for(i = 0; i < MAX_SEN_NUM * MAX_DRV_NUM; i++)
	{
		if(j<=144)
		{
			ts_test->test_params.max_limits[i] = (tptest_limit->data[buf_offset+2+j]&0x0F)*1000+(tptest_limit->data[buf_offset+3+j]&0x0F)*100+(tptest_limit->data[buf_offset+4+j]&0x0F)*10+(tptest_limit->data[buf_offset+5+j]&0x0F);
			ts_test->test_params.min_limits[i] = (tptest_limit->data[buf_offset+7+j]&0x0F)*1000+(tptest_limit->data[buf_offset+8+j]&0x0F)*100+(tptest_limit->data[buf_offset+9+j]&0x0F)*10+(tptest_limit->data[buf_offset+10+j]&0x0F);
			ts_test->test_params.deviation_limits[i] = 300;
			j+=16;
		}
		else if(j<=1673)
		{
			ts_test->test_params.max_limits[i] = (tptest_limit->data[buf_offset+3+j]&0x0F)*1000+(tptest_limit->data[buf_offset+4+j]&0x0F)*100+(tptest_limit->data[buf_offset+5+j]&0x0F)*10+(tptest_limit->data[buf_offset+6+j]&0x0F);
			ts_test->test_params.min_limits[i] = (tptest_limit->data[buf_offset+8+j]&0x0F)*1000+(tptest_limit->data[buf_offset+9+j]&0x0F)*100+(tptest_limit->data[buf_offset+10+j]&0x0F)*10+(tptest_limit->data[buf_offset+11+j]&0x0F);
			ts_test->test_params.deviation_limits[i] = 300;
			j+=17;
		}
		else if(j<=6172)
		{
			ts_test->test_params.max_limits[i] = (tptest_limit->data[buf_offset+4+j]&0x0F)*1000+(tptest_limit->data[buf_offset+5+j]&0x0F)*100+(tptest_limit->data[buf_offset+6+j]&0x0F)*10+(tptest_limit->data[buf_offset+7+j]&0x0F);
			ts_test->test_params.min_limits[i] = (tptest_limit->data[buf_offset+9+j]&0x0F)*1000+(tptest_limit->data[buf_offset+10+j]&0x0F)*100+(tptest_limit->data[buf_offset+11+j]&0x0F)*10+(tptest_limit->data[buf_offset+12+j]&0x0F);
			ts_test->test_params.deviation_limits[i] = 300;
			j+=18;
		}
		else
		{
			break;
		}
		TS_LOG_DEBUG("max_limits[%d]=%d min_limits[%d]=%d deviation_limits[%d]=%d\n",i,ts_test->test_params.max_limits[i],i,ts_test->test_params.min_limits[i],i,ts_test->test_params.deviation_limits[i]);
	}
	exit:
		release_firmware(tptest_limit);
		return retval;
}
/**
 * goodix_read_origconfig - read original config data
 */
static int goodix_read_origconfig(struct goodix_ts_test *ts_test)
{
	int ret = -ENODEV, cfg_size;

	GTP_DEBUG("Read original config data");

	cfg_size = GTP_CONFIG_ORG_LENGTH;
	if (ts_test->ts->ops.i2c_read) {
		ret = ts_test->ts->ops.i2c_read(GTP_REG_CONFIG_DATA,
				&ts_test->orig_config.data[0], cfg_size);
		if (ret < 0) {
			GTP_ERROR("Failed to read original config data");
			return ret;
		}

		ts_test->orig_config.size = cfg_size;
		ts_test->orig_config.delay_ms = 200;
		ts_test->orig_config.name = "original_config";
		ts_test->orig_config.initialized = true;
	}

	return ret;
}

/**
 * goodix_tptest_prepare - preparation before tp test
 */
static int goodix_tptest_prepare(struct goodix_ts_test *ts_test)
{
	struct goodix_ts_config *test_config;
	int ret, retry, i;
	u8 buf[1];
	const u8 cfg_raw_test[] = GTP_CFG_RAW_TEST;
	GTP_INFO("TP test preparation");
	ret = goodix_read_origconfig(ts_test);
	if (ret < 0)
	{
		GTP_ERROR("Get Normal Config data failed");
		return -EINVAL;
	}
	GTP_INFO("TP test preparation-->0");
	ts_test->product_id = "GT917D";
	GTP_INFO("TP test preparation-->1");
	test_config = &ts_test->test_config;
	/* parse test config data form dts */
	//ret = -ENODEV;
	/*if (ts_test->ts->ops.parse_cfg_data)
		ret = ts_test->ts->ops.parse_cfg_data(ts_test->ts,
			"tptest_config", &test_config->data[0], &test_config->size,
			ts_test->ts->hw_info.sensor_id);*/
	memcpy(test_config->data,cfg_raw_test,GTP_CONFIG_ORG_LENGTH);
	TS_LOG_INFO("xljadd tptest_config[1]=0x%x tptest_config[10]=0x%x\n",test_config->data[0],test_config->data[10]);
	test_config->delay_ms = 40;
	test_config->name = "tptest_config";
	test_config->initialized = true;

	if (ret < 0)
	{
		GTP_ERROR("Failed to parse tptest-config:%d",ret);

		memcpy(&test_config->data[0],&ts_test->orig_config.data[0],ts_test->test_config.size);
		gt9xx_disable_hopping(ts_test,test_config ,1);
	}
	else
	{
		GTP_ERROR("Success to parse tptest-config:%d",ret);
		gt9xx_disable_hopping(ts_test,test_config ,0);
	}

	/* parse sensor & driver number */
	if (!strcmp(ts_test->product_id, "GT917D"))
	{

		gt9xx_parse_config(&ts_test->test_config,
						&ts_test->test_params);
		ts_test->rawdata.addr = 0x9B60;
		ts_test->basedata.addr = 0x9560;//
	}
	else
	{
		return -ENODEV;
	}
	/* get test limits from dt */
	ret = gt9xx_get_testlimits_from_ini(ts_test);
	if (ret < 0)
	{
		GTP_ERROR("Failed to init testlimits from ini config:%d", ret);
	}
	else
	{
		ret = goodix_init_testlimits(ts_test);
		if (ret < 0)
		{
			GTP_ERROR("Failed to init testlimits from dt:%d", ret);
			return ret;
		}
	}
	return 0;
}

/**
 * goodix_tptest_finish - finish test
 */
static inline int goodix_tptest_finish(struct goodix_ts_test *ts_test)
{
	int ret = -ENODEV;

	GTP_INFO("TP test finish");

	if (ts_test->ts->ops.send_cfg)
		ret = ts_test->ts->ops.send_cfg(&ts_test->orig_config);
	if (ret < 0) {
		GTP_ERROR("Failed to send normal config:%d", ret);
		return ret;
	}

	return 0;
}

static s32 gt9x_be_normal(struct goodix_ts_test *ts_test) {
	s32 ret = -1;
	u8 buf[1];

	GTP_DEBUG("gt9x Leave Rawdiff..");
	buf[0] = 0x00;
	if  (ts_test->ts->ops.i2c_write)
		ret = ts_test->ts->ops.i2c_write(GTP_REG_CMD, &buf[0], 1);

	if (ret < 0)
	{
		return ret;
	}

	msleep(1);
	GTP_INFO("GT9x_be_normal");

	buf[0] = 0x00;
	ts_test->ts->ops.i2c_write(GTP_READ_COOR_ADDR, &buf[0], 1);

	GTP_INFO("clear 0x814e 00.");

	return ret;
}


/**
 * goodix_sample_rawdata - sample rawdata
 */
static void save_rawrata_file(int fd,u16 buf[],int rx,int tx)
{
	int len = rx*tx*5*sizeof(uint8_t)+tx*sizeof(uint8_t);
	uint8_t *buffer = kzalloc(len,GFP_KERNEL);
	memset(buffer,' ',len);
	int i=0,j=0;
	for(i = 0;i < len;i+=5)
	{
	  sprintf(buffer+i,"%d ",buf[j++]);
	  if(j%rx == 0)
	  {
	   buffer[i+5] = '\n';
	   i++;
	  }
	}
	sys_write(fd,buffer,strlen(buffer));
	kfree(buffer);
}
static int goodix_sample_rawdata(struct goodix_ts_test *ts_test)
{
	int ret = -1, i, retry = 40;
	u8 buf[1] = {0x00};
	u32 rawdata_size;
	u8	*sample_buf;
	int index;
	GTP_DEBUG("Sample Rawdata");
	rawdata_size = ts_test->test_params.sen_num*ts_test->test_params.drv_num;

	if (rawdata_size > MAX_DRV_NUM * MAX_SEN_NUM)
	{
		GTP_ERROR("Invalid rawdata size(%u)", rawdata_size);
		return -EINVAL;
	}
	sample_buf = (u8 *) kmalloc(rawdata_size*2,GFP_KERNEL);
	if (sample_buf == NULL)
	{
		GTP_ERROR("No memory");
			return -ENOMEM;
	}

	while (retry--)
	{
		ret = ts_test->ts->ops.i2c_read(GTP_READ_COOR_ADDR, &buf[0], 1);
		GTP_DEBUG("Sample Rawdata Retry No.%d,ret is %d,buf[0] is 0x%x",retry,ret,buf[0]);
		if (ret < 0 || (buf[0] & 0x80) == 0x80)
			break;
		usleep_range(5000, 5010);
	}

	if (!ret && (buf[0] & 0x80) == 0x80)
	{
		ret = ts_test->ts->ops.i2c_read(ts_test->rawdata.addr,sample_buf, rawdata_size * 2);
		if (ret < 0)
		{
			GTP_ERROR("Failed to read rawdata:%d", ret);
			return ret;
		}

		for (i = 0,index=0; i < rawdata_size*2; i+=2)
		{
			ts_test->rawdata.data[index++] =(sample_buf[i]<<8) + sample_buf[i+1];
		}

		ts_test->rawdata.size = rawdata_size;
		GTP_DEBUG("Rawdata ready");
	}
	else
	{
		GTP_ERROR("Read rawdata timeout");
		ret = -EFAULT;
	}

	buf[0] = 0x00;
	ts_test->ts->ops.i2c_write(GTP_READ_COOR_ADDR, &buf[0], 1);
	kfree(sample_buf);
	return ret;
}

/**
 * goodix_sample_rawdata - sample noise rawdata
 */
static int goodix_store_noisedata(struct goodix_ts_test *ts_test,
			struct ts_rawdata_info *info)
{
	int ret = -1, i, retry = 20;
	u8 buf[1] = {0x00};
	u32 data_size;

	data_size = ts_test->test_params.sc_sen_num *
			ts_test->test_params.sc_drv_num +
			ts_test->test_params.key_num;

	if (data_size > MAX_DRV_NUM * MAX_SEN_NUM) {
		GTP_ERROR("Invalid rawdata size(%u)", data_size);
		return -EINVAL;
	}

	while (retry--) {
		ret = ts_test->ts->ops.i2c_read(GTP_READ_COOR_ADDR, &buf[0], 1);
		if (ret < 0 || (buf[0] & 0x80) == 0x80)
			break;
		usleep_range(5000, 5010);
	}

	if (!ret && (buf[0] & 0x80) == 0x80) {
		ret = ts_test->ts->ops.i2c_read(ts_test->basedata.addr,
				(u8 *)&ts_test->basedata.data[0], data_size * 2);
		if (ret < 0) {
			GTP_ERROR("Failed to read basedata:%d", ret);
			return ret;
		}

		for (i = 0; i < data_size; i++)
			ts_test->basedata.data[i] =
				be16_to_cpu(ts_test->basedata.data[i]);
		ts_test->basedata.size = data_size;

		if (ts_test->basedata.size + info->used_size <=
				TS_RAWDATA_BUFF_MAX) {
			for (i = 0; i < ts_test->basedata.size; i++)
				info->buff[info->used_size + i ] =
					ts_test->basedata.data[i] -
					ts_test->rawdata.data[i];
			info->used_size += ts_test->basedata.size;
		} else {
			GTP_ERROR("Invalid base data size:%u",
					ts_test->basedata.size);
		}
	} else {
		GTP_ERROR("Read basedata timeout");
		ret = -EFAULT;
	}

	buf[0] = 0x00;
	ts_test->ts->ops.i2c_write(GTP_READ_COOR_ADDR, &buf[0], 1);
	return ret;
}

static int goodix_rawcapacitance_test(struct ts_test_rawdata *rawdata,
				struct ts_test_params *test_params)
{
	bool pass = true;
	int i;
	GTP_INFO("rawdata sc_sensor is [%d],sc_drv_num[%d] size is [%d]",test_params->sc_sen_num, test_params->sc_drv_num,rawdata->size);
	for (i = 0; i < rawdata->size; i++) {
		if (rawdata->data[i] > test_params->max_limits[i]) {
			GTP_ERROR("rawdata[%d][%d]:%u > max_limit:%u, NG",
				i / test_params->sc_sen_num, i % test_params->sc_sen_num,
				rawdata->data[i], test_params->max_limits[i]);
			pass = false;
		}

		if (rawdata->data[i] < test_params->min_limits[i]) {
			GTP_ERROR("rawdata[%d][%d]:%u < mix_limit:%u, NG",
				i / test_params->sc_sen_num, i % test_params->sc_sen_num,
				rawdata->data[i], test_params->min_limits[i]);
			pass = false;
		}
	}


	return pass ? 0 : -1;
}

static int goodix_deltacapacitance_test(struct ts_test_rawdata *rawdata,
				struct ts_test_params *test_params)
{
	u32 sc_data_num, max_val = 0, rawdata_val;
	u32 up = 0, down = 0, left = 0, right = 0;
	int cols = test_params->sc_sen_num;
	bool pass = true;
	int i;

	sc_data_num = test_params->sc_drv_num * test_params->sc_sen_num;

	for (i = 0; i < sc_data_num; i++) {
		rawdata_val = rawdata->data[i];
		max_val = 0;
		if (i - cols >= 0) {
			up = rawdata->data[i - cols];
			up = abs(rawdata_val - up);
			if (up > max_val)
				max_val = up;
		}

		if (i + cols < sc_data_num) {
			down = rawdata->data[i + cols];
			down = abs(rawdata_val - down);
			if (down > max_val)
				max_val = down;
		}

		if (i % cols) {
			left = rawdata->data[i - 1];
			left = abs(rawdata_val - left);
			if (left > max_val)
				max_val = left;
		}

		if ((i + 1) % cols) {
			right = rawdata->data[i + 1];
			right = abs(rawdata_val - right);
			if (right > max_val)
				max_val = right;
		}

		/* float to integer */
		max_val *= FLOAT_AMPLIFIER;
		max_val /= rawdata_val;
		TS_LOG_INFO("xljadd deltacapacitance max_val=%d",max_val);
		if (max_val > test_params->deviation_limits[i]) {
			GTP_ERROR("deviation[%d][%d]:%u > delta_limit:%u, NG",
			i / cols, i % cols, max_val,
			test_params->deviation_limits[i]);
			pass = false;
		}
	}

	return pass ? 0 : -1;
}

static int goodix_capacitance_test(struct goodix_ts_test *ts_test,
		struct ts_rawdata_info *info)
{
	int test_result0 = 0;
	int test_result1 = 0;
	int test_result2 = 0;
	int test_result3 = 0;
	int test_cnt = 0;
	int ret, i;
	int fd;
	mm_segment_t fs;

begin_test:
	if (test_cnt > 1)
		goto end_test;

	for (i = 0; i < 16; i++) {
		GTP_INFO("Rawcap Test:%d:%d", test_cnt, i);
		GTP_INFO("Rawcap Test_types is :0x%x", ts_test->test_params.test_types);

		ret = goodix_sample_rawdata(ts_test);
		if (ret < 0) {
			test_cnt ++;
			test_result0 = -1;
			test_result1 = -1;
			test_result2 = -1;
			test_result3 = -1;
			goto begin_test;
		}
		fs = get_fs();
		set_fs(KERNEL_DS);
		fd = sys_open(GOODIX_RAW_DUMP_FILE,O_CREAT | O_RDWR ,0666);
		if (fd>=0)
		{
			TS_LOG_INFO("%s: write rawdata file \n",__func__);
			save_rawrata_file(fd,ts_test->rawdata.data,ts_test->test_params.sen_num,ts_test->test_params.drv_num);
			sys_close(fd);
			set_fs(fs);
		}
		else
			TS_LOG_INFO("%s: open file fail fd=%d\n",__func__,fd);
		if ((ts_test->test_params.test_types &( _MAX_CHECK|_MIN_CHECK)) != 0) {
			test_result0 = goodix_rawcapacitance_test(&ts_test->rawdata,
				&ts_test->test_params);
		}

		if ((ts_test->test_params.test_types &_ACCORD_CHECK ) != 0) {
			test_result1 = goodix_deltacapacitance_test(&ts_test->rawdata,
				&ts_test->test_params);
		}

		if ((ts_test->test_params.test_types & _KEY_MAX_CHECK) != 0) {
			test_result2 = _check_key_max_value(&ts_test->rawdata,ts_test);
		}

		if ((ts_test->test_params.test_types & _KEY_MIN_CHECK) != 0) {
			test_result3 = _check_key_min_value(&ts_test->rawdata,ts_test);
		}

		if (test_result0 < 0 || test_result1 < 0||test_result2<0 ||test_result3<0) {
			test_cnt++;
			goto begin_test;
		}

	}
end_test:
	if ((ts_test->test_params.test_types & ( _MAX_CHECK|_MIN_CHECK)) != 0) {
		if (!test_result0) {
			strcat(ts_test->test_result, "-2P");
		} else {
			GTP_ERROR("Rawcap test:NG");
			strcat(ts_test->test_result, "-2F");
		}
	}

	if ((ts_test->test_params.test_types & _ACCORD_CHECK) != 0) {
		if (!test_result1) {
			strcat(ts_test->test_result, "-3P");
		} else {
			GTP_ERROR("Deltacap test:NG");
			strcat(ts_test->test_result, "-3F");
		}
	}

	if ((ts_test->test_params.test_types & _KEY_MAX_CHECK) != 0) {
		if (!test_result2) {
			strcat(ts_test->test_result, "-4P");
		} else {
			GTP_ERROR("key_max:NG");
			strcat(ts_test->test_result, "-4F");
		}
	}

	if ((ts_test->test_params.test_types & _KEY_MIN_CHECK) != 0) {
		if (!test_result3) {
			strcat(ts_test->test_result, "-5P");
		} else {
			GTP_ERROR("key_min:NG");
			strcat(ts_test->test_result, "-5F");
		}
	}

	if (ts_test->rawdata.size <= TS_RAWDATA_BUFF_MAX) {
		info->buff[0] = ts_test->test_params.sen_num;
		info->buff[1] = ts_test->test_params.drv_num;
		for (i = 0; i < ts_test->rawdata.size; i++)
			info->buff[i + 2] = ts_test->rawdata.data[i];
		info->used_size = ts_test->rawdata.size + 2;
	}
	else
	{
		GTP_ERROR("Invalid rawdata size");
	}

	return 0;
}

/* get short firmware */
static int goodix_get_short_firmware(void)
{
	int ret = -1;

	GTP_INFO("Get shortcircut firmware ...");

	ret = request_firmware(&short_fw, SHORT_FILE_PATH,&goodix_ts->pdev->dev);
	if (ret < 0) {
		GTP_ERROR("Request firmware failed - %s (%d)", SHORT_FILE_PATH, ret);
		return ret;
	}
	GTP_INFO("Firmware size: %zd", short_fw->size);
	GTP_INFO("dsp_short[0]: 0x%02x dsp_short[1]: 0x%02x dsp_short[2]: 0x%02x",
		short_fw->data[0], short_fw->data[1], short_fw->data[2]);

	return ret;
}

int gt9x_enter_update_mode_noreset(struct goodix_ts_test *ts_test)
{
	int ret = -1;
	int retry = 0;
	unsigned char buf[1];

	// disable watchdog
	buf[0] = 0x00;
    ts_test->ts->ops.i2c_write(_bRW_MISCTL__TMR0_EN, buf, 1);

	//clr bu
    buf[0] = 0x00;
	ret = ts_test->ts->ops.i2c_write(_bRW_MISCTL__CACHE_EN, buf, 1);
	if(ret < 0)
	{
		GTP_ERROR("clr bu fail.");
		return ret;
	}

    //boot from sram
	buf[0] = 0x02;
    ts_test->ts->ops.i2c_write(_rRW_MISCTL__BOOTCTL_B0_, buf, 1);

    //set bank0
	buf[0] = 0x00;
    ts_test->ts->ops.i2c_write( _bRW_MISCTL__SRAM_BANK, buf, 1);

	buf[0] = 0x01;
    ts_test->ts->ops.i2c_write(_bRW_MISCTL__MEM_CD_EN, buf, 1);

	return 1;
}

static int goodix_hold_ss51_dsp(struct goodix_ts_test *ts_test)
{
	int ret = -1;
	u8 buffer[1];
	int retry = 0;
	/* chip reset */
	ts_test->ts->ops.chip_reset();

	while (retry++ < 200) {
		/* Hold ss51 & dsp */
		buffer[0] = 0x0C;
		ret = ts_test->ts->ops.i2c_write(_rRW_MISCTL__SWRST_B0_, buffer, 1);
		if (ret != 0) {
			GTP_ERROR("Hold ss51 & dsp I2C error,retry:%d", retry);
			continue;
		}

		msleep(1);
		if(retry < 100)
			continue;
		/* Confirm hold */
		ret = ts_test->ts->ops.i2c_read(_rRW_MISCTL__SWRST_B0_, buffer, 1);
		if (ret != 0) {
			GTP_ERROR("Hold ss51 & dsp I2C error,retry:%d", retry);
			continue;
		}

		if (0x0C == buffer[0]) {
			GTP_INFO("Hold ss51 & dsp confirm SUCCESS");
			break;
		}

		GTP_ERROR("Hold ss51 & dsp confirm 0x4180 failed,value:%d", buffer[0]);
	}

	if (retry >= 200) {
		GTP_ERROR("Enter update Hold ss51 failed.");
		return -1;
	}
	return ret;
}

static s32 load_code_and_check(unsigned char *codes, int size, struct goodix_ts_test *ts_test)
{
	u8 i, count, packages;
	u8 *ram;
	u16 start_addr, tmp_addr;
	s32 len, code_len, ret = -1;

	ram = (u8 *) kmalloc(PACKAGE_SIZE,GFP_KERNEL);
	if (ram == NULL) {
		return MEMORY_ERR;
	}

	start_addr = 0xC000;
	len = PACKAGE_SIZE;
	tmp_addr = start_addr;
	count = 0;
	code_len = size;
	packages = code_len / PACKAGE_SIZE + 1;

	for (i = 0; i < packages; i++) {
		if (len > code_len) {
			len = code_len;
		}

		ts_test->ts->ops.i2c_write(tmp_addr, (u8 *) & codes[tmp_addr - start_addr], len);
		ts_test->ts->ops.i2c_read(tmp_addr, ram, len);
		ret = memcmp(&codes[tmp_addr - start_addr], ram, len);

		if (ret) {
			if (count++ > 5) {
				GTP_ERROR("equal error.");
				break;
			}
			continue;
		}

		tmp_addr += len;
		code_len -= len;

		if (code_len <= 0) {
			break;
		}
	}

	kfree(ram);
	if (count < 5) {
		GTP_INFO("Burn DSP code successfully!");
		return 1;
	}

	return -1;
}

static int dsp_fw_startup(struct goodix_ts_test *ts_test)
{
	unsigned char buf[8];
	int ret = -1;

	buf[0] = 0x00;
	ret = ts_test->ts->ops.i2c_write(_rRW_MISCTL__SHORT_BOOT_FLAG, buf, 1);
	if (ret < 0) {
		GTP_ERROR("_rRW_MISCTL__SHORT_BOOT_FLAG fail!");
		return ret;
	}

	buf[0] = 0x03;
    ret = ts_test->ts->ops.i2c_write(_rRW_MISCTL__BOOT_OPT_B0_,buf, 1);
	if (ret < 0) {
		GTP_ERROR("_rRW_MISCTL__BOOT_OPT_B0_ fail!");
		return ret;
	}

    buf[0] = 0x01;
    ret = ts_test->ts->ops.i2c_write(_bWO_MISCTL__CPU_SWRST_PULSE,buf, 1);
	if (ret < 0) {
		GTP_ERROR("_bWO_MISCTL__CPU_SWRST_PULSE fail!");
		return ret;
	}

	/* release ss51 */
	buf[0] = 0x08;
	ret = ts_test->ts->ops.i2c_write(_rRW_MISCTL__SWRST_B0_, buf, 1);
	if (ret < 0) {
		GTP_ERROR("release ss51 fail!");
		return ret;
	}

	return ret;
}

/* short test */
static int goodix_short_test_prepare(struct goodix_ts_test *ts_test)
{
	u8 data[MAX_SEN_NUM + MAX_DRV_NUM]; /* max sen_num and drv_num */
	int ret, retry = 2;
	u16 drv_offest, sen_offest;
	u8 chksum = 0x00;

	unsigned char i,j,tmp;

	GTP_INFO("Short test prepare+");

	ret = goodix_get_short_firmware();
	if (ret < 0)
	{
		GTP_ERROR("Request firmware failed, exit short test");
		return ret;
	}

	ret = goodix_hold_ss51_dsp(ts_test);
	if (ret < 0)
	{
		GTP_ERROR("Hold ss51 & dsp failed, exit short test");
		return ret;
	}

	ret = gt9x_enter_update_mode_noreset(ts_test);
	if (ret < 0) {
		return ret;
	}


	/* preparation of downloading the DSP code */
	ret = load_code_and_check((u8 *)short_fw->data, short_fw->size, ts_test);
	if (ret < 0)
	{
		GTP_ERROR("Load code failed, exit short test");
		return ret;
	}

	ret = dsp_fw_startup(ts_test);
	if (ret < 0)
	{
		GTP_ERROR("Dsp fw startup failed, exit short test");
		return ret;
	}

	msleep(1);


	for (i = 0; i < 100; i++) { /* wait short test fw funning */
		ts_test->ts->ops.i2c_read(_rRW_MISCTL__SHORT_BOOT_FLAG, data, 1);
		if (data[0] == 0xaa)
			break;

		GTP_INFO("buf[0]:0x%x", data[0]);
		msleep(1);
	}

	if (i >= 20) { // timeout
		GTP_INFO("Didn't get 0xaa at 0x%X", _rRW_MISCTL__SHORT_BOOT_FLAG);
		return -1;
	}

	GTP_INFO("Firmware in short test mode");

	data[0] = (ts_test->test_params.short_threshold >> 8) & 0xff;
	data[1] = ts_test->test_params.short_threshold & 0xff;
	ret = ts_test->ts->ops.i2c_write(0x8804, data, 2);
	if (ret < 0)
		return ret;

	//ADC Read Delay
    data[0] = (150 >> 8) & 0xff;
    data[1] = 150 & 0xff;
    ts_test->ts->ops.i2c_write(0x8806, data, 2);

	//DiffCode Short Threshold
    data[0] = (20 >> 8) & 0xff;
    data[1] = 20 & 0xff;
    ts_test->ts->ops.i2c_write(0x8851,data,2);

	sen_offest = 0x80B7 - 0X8047;
	memcpy(data,&ts_test->test_config.data[sen_offest],MAX_DRV_NUM + MAX_SEN_NUM);

	j = 0;
	for (i = 0;i < MAX_SEN_NUM;i++)
	{
		if (j == 1)
		{
			data[i] = 0xFF;
			continue;
		}
		if (data[i] == 0XFF)
		{
			j = 1;
		}
	}

	j = 0;
	for (i = 0 ;i < MAX_DRV_NUM; i++)
	{
		if (j == 1)
		{
			data[i + MAX_SEN_NUM] = 0xFF;
			continue;
		}
		if (data[i + MAX_SEN_NUM] == 0XFF)
		{
			j = 1;
		}
	}

	for (i = 0;i < MAX_DRV_NUM + MAX_SEN_NUM;i++)
	{
		chksum += data[i];
	}
	chksum = 0 - chksum;
	ts_test->ts->ops.i2c_write(0x8808,&data[MAX_SEN_NUM],MAX_DRV_NUM);

	ts_test->ts->ops.i2c_write(0x8832,data,MAX_SEN_NUM);

	ts_test->ts->ops.i2c_write( 0x8850, &chksum, 1);

	data[0] = 0x00;
	ts_test->ts->ops.i2c_write( 0x8853, data, 1);

	/* clr 5095, runing dsp */
	data[0] = 0x04;

	ret = ts_test->ts->ops.i2c_write(_rRW_MISCTL__SHORT_BOOT_FLAG, data, 1);
	if (ret < 0)
		return ret;
	GTP_INFO("Short test prepare-");
	return 0;

}

static inline long gt9xx_calc_resisitance(u16 self_data,u8 master,u8 slave,u16 short_code,u8 flag)
{
	long r = 0;

	if((((master >= (_CHANNEL_TX_FLAG|14)) && (master <= (_CHANNEL_TX_FLAG|28)))
	        && ((slave >= (_CHANNEL_TX_FLAG|14)) && (slave <= (_CHANNEL_TX_FLAG|28))))
	        ||(((master >= (_CHANNEL_TX_FLAG|29)) && (master <= (_CHANNEL_TX_FLAG|42)))
	        && ((slave >= (_CHANNEL_TX_FLAG|29)) && (slave <= (_CHANNEL_TX_FLAG|42)))))
	{
	    r = (long)self_data*40*FLOAT_AMPLIFIER/short_code - 40*FLOAT_AMPLIFIER;//ABIST
	}
	else if((slave&(_CHANNEL_TX_FLAG|0x01)) == 0x01)
	{
	    r = (long)self_data*60*FLOAT_AMPLIFIER/short_code - 40*FLOAT_AMPLIFIER;
	}
	else	//Others
	{
	    r = (long)self_data*60*FLOAT_AMPLIFIER/short_code - 60*FLOAT_AMPLIFIER;
	    if (flag == 1)   //GT950?∴??1：o??┷?DACA?┐?D???|：∴：a3：|ABIST
	    {
	        if ((master >= _CHANNEL_TX_FLAG) && (slave>= _CHANNEL_TX_FLAG))
	        {
			r = (float)self_data*40*FLOAT_AMPLIFIER/short_code - 40*FLOAT_AMPLIFIER;//ABIST
	        }
	    }
	}

	return r;
}


static int gt9xx_check_resistance_to_gnd(struct ts_test_params *test_params,
				u16 short_code, u8 pos)
{
	long long r;

	u16 r_th;

	u8 m, totals;
	//int max_drvs = 32;

	u16 avdd_value;

	avdd_value = test_params->avdd_value;

	totals = upload_short_data[0];

	if (short_code == 0 ||short_code == 0x8000) {

		short_code |= 1;

	}

	if ((short_code & (0x8000)) == 0)	//short to GND
		r = (long long)(52662850)*10 / (short_code & (~0x8000)) - 40 * FLOAT_AMPLIFIER * 10;	//52662.85/code-40
	else			//short to VDD
		r = (long long)40 * 9 * 1024 * (avdd_value - 9) * FLOAT_AMPLIFIER / ((short_code & (~0x8000)) * 7) - 40 * FLOAT_AMPLIFIER * 10;

	r = r / FLOAT_AMPLIFIER;
	if(r > 65535)
	{
		r = 65535;
	}
	short_code = (r >= 0 ? r : 0);

	m = pos >> 1;

	//DEBUG("Tx,Rx[0x%X] & VDD,GND resistance:%dK", pos, (int)r/10);
	if (pos < MAX_DRV_NUM*2) {

		m = ChannelPackage_TX[m] | _CHANNEL_TX_FLAG;
//		m = m | _CHANNEL_TX_FLAG;

		r_th = test_params->r_drv_gnd_threshold;

	} else {

		m -= MAX_DRV_NUM;

		r_th = test_params->r_drv_gnd_threshold;

	}

	if (short_code < (r_th*10)) {
		upload_short_data[totals * 4 + 1] = m;

		upload_short_data[totals * 4 + 2] = MAX_DRV_NUM + 1;	//short with GND
		upload_short_data[totals * 4 + 3] = (short_code >> 8) & 0xff;

		upload_short_data[totals * 4 + 4] = short_code & 0xff;

		{
			if (totals < _GT9_UPLOAD_SHORT_TOTAL) {

				upload_short_data[0] = totals + 1;

				return _GT_SHORT;
			}
		}

	}

	return _CHANNEL_PASS;
}
int save_short_data(u8* short_data,strShortRecord r_data,u16 short_code)
{
	u8 n,cnt;

    cnt = short_data[0];
    for (n = 0; n < cnt; n++)
    {
	if (short_data[n * 4 + 1] == r_data.master && short_data[n * 4 + 2] == r_data.slave)
		{
			break;
		}

		if(short_data[n * 4 + 1] == r_data.slave && short_data[n * 4 + 2] == r_data.master)
		{
			break;
		}
    }

	if (n >= cnt)         //??：?：?：2|━：：：?：22?┷??┐：a：o?：?D????
	{
		short_data[1 + cnt * 4 + 0] = r_data.master;
		short_data[1 + cnt * 4 + 1] = r_data.slave;
		short_data[1 + cnt * 4 + 2] = (short_code >> 8) & 0xff;
		short_data[1 + cnt * 4 + 3] = short_code & 0xff;

        if (cnt < _GT9_UPLOAD_SHORT_TOTAL)
        {
		cnt++;
		short_data[0] = cnt;
        }
		return _GT_SHORT;
    }

    return 0;
}

static int gt9xx_check_short_resistance(strShortRecord r_data,unsigned short short_r_th, unsigned char flag)
{
	unsigned short j;
	int max_drvs = MAX_DRV_NUM, max_sens  = MAX_SEN_NUM;
		unsigned short short_code, warn_threshold;

		long r;

		u16 totals = upload_short_data[0];

		int test_error_code = 0;

		if (totals >= _GT9_UPLOAD_SHORT_TOTAL)
			return _CHANNEL_PASS;
		//check whether calculate resistance
		j = r_data.position;

		if (j > (max_drvs + max_sens)) {
			upload_short_data[0] = _GT9_SHORT_TEST_ERR;
			return _CHANNEL_PASS;
		}

		if (self_data[j] == 0xffff) {
			return _CHANNEL_PASS;
		}

		if (self_data[j] == 0) {
			return _CHANNEL_PASS;
		}

		r = gt9xx_calc_resisitance(self_data[j], r_data.master, r_data.slave, r_data.short_code,flag);

		r = r * 10 / FLOAT_AMPLIFIER;

		if(r > 65535)
		{
			r = 65535;
		}
		short_code = (r >= 0 ? r : 0);

		//priority to upload the small resistance
		if (gt900_resistor_warn_threshold < short_r_th) {

			warn_threshold = short_r_th;

		} else {

			warn_threshold = gt900_resistor_warn_threshold;

		}

		if (short_code < (short_r_th * 10)) {

			test_error_code |= save_short_data(upload_short_data, r_data, short_code);

		} else if (short_code < (warn_threshold * 10)) {

			test_error_code |= save_short_data(warn_short_data, r_data, short_code);

		}

		return test_error_code;
}

static int goodix_shortcircut_analysis(struct goodix_ts_test *ts_test)
{
	u8 short_flag, *data_buf = NULL, short_status[3];
	int max_drvs = MAX_DRV_NUM, max_sens  = MAX_SEN_NUM;
	u16 self_capdata[max_drvs + max_sens], short_pin_num;
	u16 short_code, result_addr;
	u16 r_threshold;
	int size, i, j, length;
	int ret = 0, err = 0;
	int offest,test_error_code;
	u8 *tmp;

	strShortRecord r_data;

	ret = ts_test->ts->ops.i2c_read(0x8801,  &short_flag, 1);
	if (ret < 0)
	{
		return ret;
	} else if ((short_flag & 0x0F) == 0x00) {
		GTP_INFO("No shortcircut");
		return 0;
	}
	data_buf = kzalloc(1024, GFP_KERNEL);
	if (!data_buf)
	{
		GTP_ERROR("Failed to alloc memory");
		return -ENOMEM;
	}

	memset(data_buf, 0, 1024);

	offest = 0;

	upload_short_data = (u8 *)(&data_buf[offest]);

	offest += (_GT9_UPLOAD_SHORT_TOTAL +1 ) * 4 + 1;

	warn_short_data = (u8 *)(&data_buf[offest]);

	offest += (_GT9_UPLOAD_SHORT_TOTAL +1 ) * 4 + 1;

	self_data = (u8 *)(&data_buf[offest]);

	offest += (max_drvs + max_sens) * sizeof(unsigned short);

	tmp = (u8 *)(&data_buf[offest]);

	if((short_flag&0x08) == 0x08)
	{
		length = (max_drvs + max_sens) * 2;
		ret = ts_test->ts->ops.i2c_read(0xA531, tmp,length);
		if (ret < 0)
			goto exit_kfree;
		for(i = 0; i < length; i += 2)
		{
			short_code = (tmp[i] << 8) + tmp[i + 1];

			test_error_code |= gt9xx_check_resistance_to_gnd(&ts_test->test_params,short_code, i);
		}
	}

	length = (max_drvs + max_sens) * 2;
	ret = ts_test->ts->ops.i2c_read(0xA4A1, data_buf, size);
	if (ret < 0)
		goto exit_kfree;

	for (i = 0;i < (max_drvs + max_sens); i++)
	{
		self_data[i] = ((tmp[i*2]<<8) + tmp[i*2+1]) & 0x7fff;
	}

	ts_test->ts->ops.i2c_read(0x8802, short_status, 2);
	GTP_INFO("Tx&Tx:%d,Rx&Rx:%d,Tx&Rx:%d",short_status[0],short_status[1]);

	result_addr = 0x8800 + 0x60;
	for(i = 0; i < short_status[0]; i++)
	{
		length = 4 + (max_drvs + max_sens) * 2 + 2;
		ts_test->ts->ops.i2c_read(result_addr, tmp,length);
		//drv&drv
		r_threshold = ts_test->test_params.r_drv_drv_threshold;
		for(j = i + 1; j < max_drvs; j++)
		{
			short_code = (tmp[4 + j * 2] << 8) + tmp[4 + j * 2 + 1];
			if (short_code > ts_test->test_params.short_threshold)
			{
				r_data.master = tmp[0];
				r_data.position = j;
				r_data.slave	= ChannelPackage_TX[j] | _CHANNEL_TX_FLAG;//TX
				r_data.short_code= short_code;
				test_error_code |= gt9xx_check_short_resistance(r_data,r_threshold,0);
			}
		}

		//drv&sen
		r_threshold = ts_test->test_params.r_drv_sen_threshold;
		for(j = 0; j < max_sens; j++)
		{
			short_code = (tmp[4 + max_drvs*2 + j * 2] << 8) + tmp[4 + max_drvs*2 + j * 2 + 1];
			if (short_code > ts_test->test_params.short_threshold)
			{
				r_data.master = tmp[0];
				r_data.position = j + max_drvs;
				r_data.slave	= j;//TX
				r_data.short_code= short_code;
				test_error_code |= gt9xx_check_short_resistance(r_data,r_threshold,0);
			}
		}
		result_addr += length;
	}

	//sen&sen
	result_addr = 0xA0D2;
	r_threshold = ts_test->test_params.r_sen_sen_threshold;
	for (i = 0; i < short_status[1]; i++)//RX
	{
		length = 4 + max_sens * 2 + 2;
		ts_test->ts->ops.i2c_read(result_addr, tmp,length);
		for (j = 0; j < max_sens; j++)
		{
			if(j == i || (j < i && (j&0x01)==0))
				continue; // The first one have no last pin.

			short_code = (tmp[4 + j * 2] << 8) + tmp[4 + j * 2 + 1];
			if (short_code > ts_test->test_params.short_threshold)
			{
				r_data.master	= tmp[0];
				r_data.position = j + max_drvs;
				r_data.slave	= j;
				r_data.short_code= short_code;

				test_error_code |= gt9xx_check_short_resistance(r_data,r_threshold,0);
			}

		}
		result_addr += length;
	}

	exit_kfree:
		kfree(data_buf);
		return test_error_code ? -EFAULT : 0;

}

static int goodix_shortcircut_test(struct goodix_ts_test *ts_test)
{
	u16 reg_sta = 0x8800;
	u8 data[2];
	int ret, retry = 150;

	ret = goodix_short_test_prepare(ts_test);
	if (ret < 0)
		goto end_test;

	while (retry--)
	{
		ret = ts_test->ts->ops.i2c_read(reg_sta, data, 1);
		if (ret < 0)
			goto end_test;
		else if (data[0] == 0x88)
			break;

		msleep(1);
		GTP_DEBUG("waitting...:%d", retry);
	}

	if (retry <= 0)
	{
		GTP_ERROR("Wait short test finish timeout");
		ret = -1;
		goto end_test;
	}

	ret = goodix_shortcircut_analysis(ts_test);
	if (ret < 0)
	{
		GTP_ERROR("Shortcircut test failed");
		goto end_test;
	}

end_test:
	if (short_fw != NULL)
	{
		release_firmware(short_fw);
		short_fw = NULL;
	}

	gt9x_short_test_end(ts_test);
	unsigned char buf[1] = {0x00};
	ts_test->ts->ops.chip_reset();
	msleep(1);
	gt9xx_disable_hopping(ts_test,&ts_test->test_config,0);

	ts_test->ts->ops.i2c_write(0x814e, buf, 1);

	if (ret < 0)
	{
		GTP_ERROR("Short Test Fail.");
	}

	if (ret < 0)
	{
		GTP_ERROR("Short Test Fail.");
		strcat(ts_test->test_result, "-1F");
	}
	else
	{
		strcat(ts_test->test_result, "-1P");
	}

	return ret;
}

static int gt9x_short_test_end(struct goodix_ts_test *ts_test)
{
	unsigned char data;

	data = 0x99;
	ts_test->ts->ops.i2c_write(_rRW_MISCTL__BOOT_CTL_, &data, 1);

	data = 0x08;
	ts_test->ts->ops.i2c_write(_rRW_MISCTL__BOOTCTL_B0_, &data, 1);

	data = 0xFF;
	return ts_test->ts->ops.i2c_write(_rRW_MISCTL__BOOTCTL_B0_, &data, 1);
}

static s32 gt9xx_disable_hopping(struct goodix_ts_test *ts_test,struct goodix_ts_config *cfg,int flag)
{
    u16 hopping_switch = 54;
	unsigned char i = 0;

	u8* config;
	int len,ret;

	unsigned char *tmp_config;
	GTP_INFO(" Disable hopping Start!");
	len = cfg->size;

	tmp_config = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (!tmp_config)
	{
		GTP_ERROR("alloc memory failed.");
		return -1;
	}

      if(flag)
     {
		cfg->data[hopping_switch] &= 0x7F;
     }

    while(i++ < 5)
	{
		GTP_INFO(" Disable hopping config start to send!");
		ts_test->ts->ops.send_cfg(cfg);
		msleep(1);
		ret = ts_test->ts->ops.i2c_read(GTP_REG_CONFIG_DATA,tmp_config, len);
		if (ret < 0)
		{
			GTP_ERROR("Failed to read tmp config data");
			kfree(tmp_config);
			return ret;
		}

		if(memcmp(&cfg->data[1], &tmp_config[1], len - 3) == 0)
		{
			GTP_INFO("test config send success!");
			kfree(tmp_config);
			return 1;
		}
	}

	GTP_ERROR("Disable hopping send config failed!");
	kfree(tmp_config);
	return -1;
}

int goodix_get_rawdata(struct ts_rawdata_info *info,
				struct ts_cmd_node *out_cmd)
{
	int ret = 0;

	if (!goodix_ts)
		return -ENODEV;

	if (!gts_test)
	{
		gts_test = kzalloc(sizeof(struct goodix_ts_test),GFP_KERNEL);
		if (!gts_test)
		{
			GTP_ERROR("Failed to alloc mem");
			return -ENOMEM;
		}
		gts_test->ts = goodix_ts;
	}

	gts_test->test_config.size = GTP_CONFIG_ORG_LENGTH;
	GTP_INFO("Start to goodix_get_rawdata");

	ret = goodix_tptest_prepare(gts_test);
	if (ret < 0)
	{
		GTP_ERROR("TP test init error:%d", ret);
		strcpy(gts_test->test_result, "0F");
		if(gts_test->test_params.test_types & _MODULE_SHORT_CHECK)
		{
			strcat(gts_test->test_result, "-1F");
		}
		if ((gts_test->test_params.test_types & ( _MAX_CHECK|_MIN_CHECK)) != 0) {
			strcat(gts_test->test_result, "-2F");
		}

		if ((gts_test->test_params.test_types & _ACCORD_CHECK) != 0) {
			strcat(gts_test->test_result, "-3F");
		}

		if ((gts_test->test_params.test_types & _KEY_MAX_CHECK) != 0) {
			strcat(gts_test->test_result, "-4F");
		}

		if ((gts_test->test_params.test_types & _KEY_MIN_CHECK) != 0) {
			strcat(gts_test->test_result, "-5F");
		}

		goto exit_finish;
	}
	else
	{
		strcpy(gts_test->test_result, "0P");
	}

	if(gts_test->test_params.test_types & _MODULE_SHORT_CHECK)
	{
	       GTP_INFO("Start to short test");
		ret = goodix_shortcircut_test(gts_test);//short test
		if(ret < 0)
		{
			if ((gts_test->test_params.test_types & (_MAX_CHECK | _MIN_CHECK )) != 0) {
			strcat(gts_test->test_result, "-2F");
			}

			if ((gts_test->test_params.test_types & _ACCORD_CHECK) != 0) {
				strcat(gts_test->test_result, "-3F");
			}

			if ((gts_test->test_params.test_types & _KEY_MAX_CHECK) != 0) {
				strcat(gts_test->test_result, "-4F");
			}

			if ((gts_test->test_params.test_types & _KEY_MIN_CHECK) != 0) {
				strcat(gts_test->test_result, "-5F");
			}
			GTP_ERROR("Short Test Failed!!");
			goto exit_finish;
		}
	}

	if (gts_test->test_params.test_types & (_MAX_CHECK | _MIN_CHECK | _ACCORD_CHECK | _KEY_MAX_CHECK | _KEY_MIN_CHECK))
	{
		//open test
		/* i2c test and switch rawdata diff working mode */
		ret = goodix_hw_testmode(gts_test);
		if (ret < 0) {
			GTP_ERROR("Unable to set hw testmode:%d", ret);
			if ((gts_test->test_params.test_types & (_MAX_CHECK | _MIN_CHECK )) != 0) {
			strcat(gts_test->test_result, "-2F");
			}

			if ((gts_test->test_params.test_types & _ACCORD_CHECK) != 0) {
				strcat(gts_test->test_result, "-3F");
			}

			if ((gts_test->test_params.test_types & _KEY_MAX_CHECK) != 0) {
				strcat(gts_test->test_result, "-4F");
			}

			if ((gts_test->test_params.test_types & _KEY_MIN_CHECK) != 0) {
				strcat(gts_test->test_result, "-5F");
			}
			goto exit_finish;
		}

		goodix_capacitance_test(gts_test, info);
		goodix_store_noisedata(gts_test, info);
		/* i2c test and switch normal mode */
		gt9x_be_normal(gts_test);
	}

exit_finish:
	strcpy(info->result, gts_test->test_result);
	goodix_tptest_finish(gts_test);

	if(strstr(gts_test->test_result,"F")){
		strcat(info->result, "result=0");
		#ifdef CONFIG_HUAWEI_DSM
		if (!dsm_client_ocuppy(ts_dclient)) {
			dsm_client_record(ts_dclient,"goodix rawdata test result: failed.\rawdata_status is %d.\n",ret);
			dsm_client_notify(ts_dclient,DSM_TP_RAWDATA_ERROR_NO);
		}
		#endif
	}else{
		strcat(info->result, "result=1");
	}
	kfree(gts_test);
	gts_test = NULL;
	return 0;
}

