#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/of.h>

#include <linux/i2c.h>
#include <linux/delay.h>
#include "goodix_ts.h"
#include "goodix_dts.h"

int goodix_get_vendor_compatible_name(
	const char *project_id,
	char *comp_name,
	size_t size)
{
	int ret = 0;

	ret = snprintf(comp_name, size,	"%s-%s", GTP_CHIP_NAME, project_id);
	if (ret >= size) {
		TS_LOG_ERR("%s:%s, ret=%d, size=%lu\n", __func__,
			"compatible_name out of range", ret, size);
		return -EINVAL;
	}

	return 0;
}

int goodix_get_vendor_name_from_dts(
	const char *project_id,
	char *vendor_name,
	size_t size)
{
	int ret = 0;
	const char *producer = NULL;
	char comp_name[GTP_VENDOR_COMP_NAME_LEN] = {0};
	struct device_node *np = NULL;
	struct ts_kit_device_data* g_goodix_dev_data = goodix_get_device_data();
	ret = goodix_get_vendor_compatible_name(project_id,
		comp_name, GTP_VENDOR_COMP_NAME_LEN);
	if (ret) {
		TS_LOG_ERR("%s:get vendor compatible name fail\n", __func__);
		return ret;
	}

	TS_LOG_ERR("%s:compatible_name is: %s\n", __func__, comp_name);
	np = of_find_compatible_node(NULL, NULL, comp_name);
	if (!np) {
		TS_LOG_ERR("%s:find vendor node fail\n", __func__);
		return -ENODEV;
	}

	ret = of_property_read_string(np, "producer", &producer);
	if (!ret) {
		strncpy(vendor_name, producer, size);
	} else {
		TS_LOG_ERR("%s:find producer in dts fail, ret=%d\n",
			__func__, ret);
		return ret;
	}
	ret = of_property_read_u32(np, GTP_IC_TYPES, &g_goodix_dev_data->ic_type);
	if (ret) {
		TS_LOG_ERR("%s:get ic_type fail, ret=%d\n", __func__, ret);
		return -ENODATA;
	}
	TS_LOG_ERR("%s: is: producer is :%s ,ic_type is %d \n", __func__, producer,g_goodix_dev_data->ic_type);
	return 0;
}

static int goodix_parse_power_config_dts(
	struct device_node *np,
	struct ts_kit_device_data *dev_data)
{
	int ret = 0;
	return 0;
}

static int goodix_parse_report_config_dts(
	struct device_node *np,
	struct ts_kit_device_data *dev_data)
{
	int ret = 0;

	ret = of_property_read_u32(np, GTP_ALGO_ID, &dev_data->algo_id);
	if (ret) {
		TS_LOG_ERR("%s:get algo id failed\n", __func__);
		return -ENODATA;
	}

	ret = of_property_read_u32(np, GTP_X_MAX, &dev_data->x_max);
	if (ret) {
		TS_LOG_ERR("%s:get device x_max fail, ret=%d\n",
			__func__, ret);
		return -ENODATA;
	}

	ret = of_property_read_u32(np, GTP_Y_MAX, &dev_data->y_max);
	if (ret) {
		TS_LOG_ERR("%s:get device y_max fail, ret=%d\n",
			__func__, ret);
		return -ENODATA;
	}

	ret = of_property_read_u32(np, GTP_X_MAX_MT, &dev_data->x_max_mt);
	if (ret) {
		TS_LOG_ERR("%s:get device x_max fail, ret=%d\n",
			__func__, ret);
		return -ENODATA;
	}

	ret = of_property_read_u32(np, GTP_Y_MAX_MT, &dev_data->y_max_mt);
	if (ret) {
		TS_LOG_ERR("%s:get device y_max fail, ret=%d\n",
			__func__, ret);
		return -ENODATA;
	}

	TS_LOG_ERR("%s:%s=%d, %s=%d, %s=%d, %s=%d, %s=%d\n", __func__,
		"algo_id", dev_data->algo_id,
		"x_max", dev_data->x_max,
		"y_max", dev_data->y_max,
		"x_mt", dev_data->x_max_mt,
		"y_mt", dev_data->y_max_mt);

	return 0;
}

static void goodix_of_property_read_u32_default(
	struct device_node *np,
	char *prop_name,
	u32 *out_value,
	u32 default_value)
{
	int ret = 0;

	ret = of_property_read_u32(np, prop_name, out_value);
	if (ret) {
		TS_LOG_ERR("%s:%s not set in dts, use default\n",
			__func__, prop_name);
		*out_value = default_value;
	}
}

static void goodix_of_property_read_u16_default(
	struct device_node *np,
	char *prop_name,
	u16 *out_value,
	u16 default_value)
{
	int ret = 0;

	ret = of_property_read_u16(np, prop_name, out_value);
	if (ret) {
		TS_LOG_ERR("%s:%s not set in dts, use default\n",
			__func__, prop_name);
		*out_value = default_value;
	}
}


static void goodix_prase_delay_config_dts(
	struct device_node *np,
	struct goodix_ts_data *pdata)
{
	struct goodix_delay_time *delay_time = NULL;

	delay_time = pdata->delay_time;

	goodix_of_property_read_u32_default(np, GTP_HARD_RESET_DELAY,
		&delay_time->hard_reset_delay, 200);

	goodix_of_property_read_u32_default(np, GTP_ERASE_MIN_DELAY,
		&delay_time->erase_min_delay, 1350);

	goodix_of_property_read_u32_default(np, GTP_CALC_CRC_DELAY,
		&delay_time->calc_crc_delay, 300);

	goodix_of_property_read_u32_default(np, GTP_REBOOT_DELAY,
		&delay_time->reboot_delay, 200);

	goodix_of_property_read_u32_default(np, GTP_ERASE_QUERY_DELAY,
		&delay_time->erase_query_delay, 50);

	goodix_of_property_read_u32_default(np, GTP_WRITE_FLASH_QUERY_TIMES,
		&delay_time->write_flash_query_times, 30);

	goodix_of_property_read_u32_default(np, GTP_READ_ECC_QUERY_TIMES,
		&delay_time->read_ecc_query_times, 100);

	goodix_of_property_read_u32_default(np, GTP_ERASE_FLASH_QUERY_TIMES,
		&delay_time->erase_flash_query_times, 15);

	goodix_of_property_read_u32_default(np, GTP_UPGRADE_LOOP_TIMES,
		&delay_time->upgrade_loop_times, 30);

	TS_LOG_ERR("%s:%s=%d, %s=%d, %s=%d, %s=%d, %s=%d\n", __func__,
		"hard_reset_delay", delay_time->hard_reset_delay,
		"reboot_delay", delay_time->reboot_delay,
		"erase_min_delay", delay_time->erase_min_delay,
		"erase_query_delay", delay_time->erase_query_delay,
		"calc_crc_delay", delay_time->calc_crc_delay);

	TS_LOG_ERR("%s:%s=%d, %s=%d, %s=%d, %s=%d", __func__,
		"erase_flash_query_times", delay_time->erase_flash_query_times,
		"read_ecc_query_times", delay_time->read_ecc_query_times,
		"write_flash_query_times", delay_time->write_flash_query_times,
		"upgrade_loop_times", delay_time->upgrade_loop_times);
}

int goodix_prase_ic_config_dts(
	struct device_node *np,
	struct ts_kit_device_data *dev_data)
{


	goodix_of_property_read_u32_default(np, GTP_REBOOT_DELAY,
		&dev_data->reset_delay, 200);

	goodix_of_property_read_u32_default(np, GTP_SLAVE_ADDR,
		&dev_data->slave_addr, 0x14);

	TS_LOG_ERR("%s:%s=%d, %s=%d.\n", __func__,
		"reset_delay", dev_data->reset_delay,
		"slave_addr", dev_data->slave_addr);

	return 0;
}

int goodix_parse_dts(
	struct device_node *np,
	struct goodix_ts_data *goodix_pdata)
{
	int ret = 0;

	const char *str_value = NULL;
	struct ts_glove_info *glove_info = NULL;
	struct ts_holster_info *holster_info = NULL;
	struct ts_roi_info *roi_info = NULL;
	struct ts_kit_device_data *dev_data = NULL;
	dev_data = goodix_pdata->goodix_device_data->ts_platform_data->chip_data;
	ret = goodix_parse_power_config_dts(np, dev_data);
	if (ret) {
		TS_LOG_ERR("%s:parse power config fail\n", __func__);
		return ret;
	}
	ret = goodix_parse_report_config_dts(np, dev_data);
	if (ret) {
		TS_LOG_ERR("%s:parse report config fail\n", __func__);
		return ret;
	}

	ret = of_property_read_u32(np, GTP_IRQ_CFG, &dev_data->irq_config);
	if (ret) {
		TS_LOG_ERR("%s:get irq config fail, ret=%d\n", __func__, ret);
		return -ENODATA;
	}

	ret = of_property_read_u32(np, GTP_IC_TYPES, &dev_data->ic_type);
	if (ret) {
		TS_LOG_ERR("%s:get ic_type fail, ret=%d\n", __func__, ret);
		return -ENODATA;
	}
#if 0
	ret = of_property_read_string(np, GTP_TEST_TYPE, &str_value);
	if (ret) {
		TS_LOG_ERR("%s:get tp_test_type fail, ret=%d\n",
			__func__, ret);
		str_value = GTP_TEST_TYPE_DEFAULT;
	}
	strncpy(dev_data->tp_test_type, str_value, TS_CAP_TEST_TYPE_LEN);

	/*
	 * 0 is cover without glass,
	 * 1 is cover with glass that need glove mode
	 * if not define in dtsi,set 0 to disable it
	 */
	goodix_of_property_read_u32_default(np, GTP_COVER_FORCE_GLOVE,
		&dev_data->cover_force_glove, 0);
	glove_info = &(dev_data->ts_platform_data->feature_info.glove_info);
	goodix_of_property_read_u32_default(np, GTP_GLOVE_SUPPORTED,&glove_info->glove_supported, 1);
	if(glove_info->glove_supported){
		goodix_of_property_read_u32_default(np, GTP_GLOVE_SWITCH_ADDR,&glove_info->glove_switch_addr, 0);
	}
	holster_info = &dev_data->ts_platform_data->feature_info.holster_info;
	goodix_of_property_read_u32_default(np, GTP_HOLSTER_SUPPORTED,&holster_info->holster_supported, 0);
	if(holster_info->holster_supported){
		goodix_of_property_read_u32_default(np, GTP_HOSTLER_SWITCH_ADDR,&holster_info->holster_switch_addr, 0);
	}
	roi_info = &dev_data->ts_platform_data->feature_info.roi_info;
	goodix_of_property_read_u32_default(np, GTP_ROI_SUPPORTED,&roi_info->roi_supported, 0);
	if(roi_info->roi_supported){
		goodix_of_property_read_u32_default(np, GTP_ROI_SWITCH_ADDR,&roi_info->roi_control_addr, 0);
	}
	TS_LOG_ERR("%s:%s=%d, %s=%d\n", __func__,
		"irq_config", dev_data->irq_config,
		"ic_type",    dev_data->ic_type);

	TS_LOG_ERR("%s:glove_supported =%d  glove_switch_addr = 0x%04x,holster_supported = %d holster_switch_addr=0x%04x,\n",__func__,
		glove_info->glove_supported,glove_info->glove_switch_addr,
		holster_info->holster_supported,holster_info->holster_switch_addr);

	TS_LOG_ERR("%s:,roi_info=%d,roi_control_addr=0x%04x\n",__func__,
		roi_info->roi_supported,
		roi_info->roi_control_addr);
#endif
	ret = of_property_read_u32(np, GTP_IRQ_CFG, &dev_data->irq_config);
	if (ret){
		TS_LOG_ERR("%s:get irq config fail, ret=%d\n", __func__, ret);
		dev_data->irq_config = 2;
         }
	ret = of_property_read_u32(np,  "fw_upgrade_delay", &dev_data->fw_upgrade_delay);
	if (ret) {
		TS_LOG_INFO("%s, get device fw_upgrade_delay failed, will use default value: 0 \n ", __func__);
		dev_data->fw_upgrade_delay = 30000;
	}
	ret = of_property_read_u32(np,  "capacitance_test_config", &dev_data->capacitance_test_config);
	if (ret) {
		TS_LOG_INFO("%s, get device capacitance_test_config failed, will use default value: 1 \n ", __func__);
		dev_data->capacitance_test_config = 1;
	}
	return NO_ERR;
}

static void goodix_prase_test_threshold(
	struct device_node *np,
	struct goodix_test_threshold *threshold)
{

	goodix_of_property_read_u32_default(np, DTS_RAW_DATA_MIN,
		&threshold->raw_data_min, 0);

	goodix_of_property_read_u32_default(np, DTS_RAW_DATA_MAX,
		&threshold->raw_data_max, 0);

	goodix_of_property_read_u32_default(np, DTS_CB_TEST_MIN,
		&threshold->cb_test_min, 0);

	goodix_of_property_read_u32_default(np, DTS_CB_TEST_MAX,
		&threshold->cb_test_max, 0);

	goodix_of_property_read_u32_default(np, DTS_OPEN_TEST_CB_MIN,
		&threshold->open_test_cb_min, 0);

	/* short_circuit_test */
	goodix_of_property_read_u32_default(np, DTS_SHORT_CIRCUIT_RES_MIN,
		&threshold->short_circuit_min, 0);

	goodix_of_property_read_u32_default(np, DTS_LCD_NOISE_MAX,
		&threshold->lcd_noise_max, 0);

	TS_LOG_ERR("%s:%s:%s=%d, %s=%d, %s=%d, %s=%d, %s=%d, %s=%d\n",
		__func__, "cb test thresholds",
		"raw_data_min", threshold->raw_data_min,
		"raw_data_max", threshold->raw_data_max,
		"cb_test_min",  threshold->cb_test_min,
		"cb_test_max",  threshold->cb_test_max,
		"open_test_cb_min", threshold->open_test_cb_min,
		"lcd_noise_max", threshold->lcd_noise_max,
		"short_circuit_min", threshold->short_circuit_min);
}

int goodix_parse_cap_test_config(
	struct goodix_ts_data *pdata,
	struct goodix_test_params *params)
{
	int ret = 0;

	char comp_name[GTP_VENDOR_COMP_NAME_LEN] = {0};
	struct device_node *np = NULL;
	struct goodix_ts_data *gtp_pdata = goodix_get_platform_data();

	if (!gtp_pdata) {
		TS_LOG_ERR("%s:chip data null\n", __func__);
		return -EINVAL;
	}

	/*
	 * Deleted fts_read_project_id here, project_id has already readed
	 * in chip_init process
	 */

	ret = goodix_get_vendor_compatible_name(gtp_pdata->project_id, comp_name,
		GTP_VENDOR_COMP_NAME_LEN);
	if (ret) {
		TS_LOG_ERR("%s:get compatible name fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	np = of_find_compatible_node(NULL, NULL, comp_name);
	if (!np) {
		TS_LOG_ERR("%s:find dev node faile, compatible name:%s\n",
			__func__, comp_name);
		return -ENODEV;
	}

	goodix_prase_test_threshold(np, &params->threshold);

	return 0;
}

