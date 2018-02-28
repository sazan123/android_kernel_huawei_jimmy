

#ifndef __GOODIX_DTS_H__
#define __GOODIX_DTS_H__

#include <linux/i2c.h>


#define GTP_VENDOR_COMP_NAME_LEN	32

#define GTP_CHIP_NAME		"gtp"
#define HUAWEI_TS_KIT		"huawei,ts_kit"

#define GTP_IRQ_GPIO		"attn_gpio"
#define GTP_RST_GPIO		"reset_gpio"
#define GTP_VDDIO_GPIO_CTRL	"vddio_ctrl_gpio"
#define GTP_VCI_GPIO_CTRL	"vci_ctrl_gpio"
#define GTP_IRQ_CFG		"goodix_irq_config"
#define GTP_ALGO_ID		"algo_id"
#define GTP_VDD			"goodix-vdd"
#define GTP_VBUS		"goodix-io"
#define GTP_IC_TYPES		"ic_type"
#define GTP_WD_CHECK		"need_wd_check_status"
#define GTP_WD_TIMEOUT         "check_status_watchdog_timeout"
#define GTP_PRAM_PROJECTID_ADDR		"pram_projectid_addr"
#define GTP_X_MAX		"x_max"
#define GTP_Y_MAX		"y_max"
#define GTP_X_MAX_MT		"x_max_mt"
#define GTP_Y_MAX_MT		"y_max_mt"
#define GTP_VCI_GPIO_TYPE		"vci_gpio_type"
#define GTP_VCI_REGULATOR_TYPE		"vci_regulator_type"
#define GTP_VDDIO_GPIO_TYPE		"vddio_gpio_type"
#define GTP_VDDIO_REGULATOR_TYPE	"vddio_regulator_type"
#define GTP_COVER_FORCE_GLOVE		"force_glove_in_smart_cover"
#define GTP_TEST_TYPE			"tp_test_type"
#define GTP_HOLSTER_SUPPORTED			"holster_supported"
#define GTP_HOSTLER_SWITCH_ADDR		"holster_switch_addr"
#define GTP_HOSTLER_SWITCH_BIT		"holster_switch_bit"
#define GTP_GLOVE_SUPPORTED			"glove_supported"
#define GTP_GLOVE_SWITCH_ADDR		"glove_switch_addr"
#define GTP_GLOVE_SWITCH_BIT		"glove_switch_bit"
#define GTP_ROI_SUPPORTED			"roi_supported"
#define GTP_ROI_SWITCH_ADDR		"roi_switch_addr"

#define GTP_VCI_LDO_VALUE		"vci_value"
#define GTP_VDDIO_LDO_VALUE		"vddio_value"
#define GTP_NEED_SET_VDDIO_VALUE	"need_set_vddio_value"

#define GTP_FW_UPDATE_LOGIC		"fw_update_logic"

#define GTP_HARD_RESET_DELAY		"hard_reset_delay"
#define GTP_ERASE_MIN_DELAY		"erase_min_delay"
#define GTP_CALC_CRC_DELAY		"calc_crc_delay"
#define GTP_REBOOT_DELAY		"reboot_delay"
#define GTP_ERASE_QUERY_DELAY		"erase_query_delay"

#define GTP_WRITE_FLASH_QUERY_TIMES	"write_flash_query_times"
#define GTP_READ_ECC_QUERY_TIMES	"read_ecc_query_times"
#define GTP_ERASE_FLASH_QUERY_TIMES	"erase_flash_query_times"
#define GTP_UPGRADE_LOOP_TIMES		"upgrade_loop_times"
#define GTP_SLAVE_ADDR			"slave_address"

#define DTS_RAW_DATA_MIN		"threshold,raw_data_min"
#define DTS_RAW_DATA_MAX		"threshold,raw_data_max"
#define DTS_CB_TEST_MIN			"threshold,cb_test_min"
#define DTS_CB_TEST_MAX			"threshold,cb_test_max"

#define DTS_SHORT_CIRCUIT_RES_MIN	"threshold,short_circuit_min"
#define DTS_LCD_NOISE_MAX	"threshold,lcd_noise_max"
#define DTS_OPEN_TEST_CB_MIN		"threshold,open_test_cb_min"

#define GTP_TEST_TYPE_DEFAULT	"Normalize_type:judge_last_result"

#define GTP_BOOT_PROJ_CODE_ADDR2	 0x20


int goodix_get_vendor_name_from_dts(const char *project_id,
	char *vendor_name, size_t size);

int goodix_parse_dts(
	struct device_node *np,
	struct goodix_ts_data *goodix_pdata);


int goodix_prase_ic_config_dts(struct device_node *np,
	struct ts_kit_device_data *dev_data);

#endif
