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

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#include "lcdkit_panel.h"
#ifdef CONFIG_LCDKIT_DRIVER
#include "lcdkit_fb_util.h"
#endif
#if defined (CONFIG_HUAWEI_DSM)
#include <dsm/dsm_pub.h>
#include "lcdkit_dsm.h"
#endif
#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#else
#include "mt_gpio.h"
#include "mach/gpio_const.h"
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  printk(KERN_ERR "[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  printk(KERN_ERR "[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif


static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)       (lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
      lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)

#ifndef BUILD_LK

extern int NT50358A_write_byte(unsigned char addr, unsigned char value);

#endif


/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE                                   0
#define FRAME_WIDTH                                     (720)
#define FRAME_HEIGHT                                    (1280)

#define LCM_PHYSICAL_WIDTH									(64800)
#define LCM_PHYSICAL_HEIGHT									(115200)
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF

//static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
};
static struct LCM_setting_table init_setting[] = {
	{0xB9,0X03,{0xFF,0x83,0x94}},
	{0xBA,0X06,{0x63,0x03,0x68,0x6B,0xB2,0xC0}},
	{0xB1,0X0A,{0x48,0x0B,0x6B,0x09,0x32,0x44,0x71,0x31,0x4D,0x2F}},
	{0xB2,0X05,{0x00,0x80,0x64,0x05,0x07}},
	{0xB4,0X15,{0x26,0x76,0x26,0x76,0x26,0x26,0x05,0x10,0x86,0x75,0x00,0x3F,0x26,0x76,0x26,0x76,0x26,0x26,0x05,0x10,0x86}},
	{0xD3,0X21,{0x00,0x00,0x06,0x06,0x01,0x01,0x10,0x10,0x32,0x10,0x00,0x00,0x00,0x32,0x15,0x04,0x05,0x04,0x32,0x15,0x14,0x05,0x14,0x37,0x33,0x04,0x04,0x37,0x00,0x00,0x47,0x05,0x40}},
	{REGFLAG_DELAY, 5, {}},
	{0xD5,0X2C,{0x18,0x18,0x25,0x24,0x27,0x26,0x11,0x10,0x15,0x14,0x13,0x12,0x17,0x16,0x01,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x05,0x04,0x03,0x02,0x07,0x06,0x18,0x18,0x18,0x18,0x21,0x20,0x23,0x22,0x18,0x18,0x18,0x18}},
	{REGFLAG_DELAY, 5, {}},
	{0xD6,0X2C,{0x18,0x18,0x22,0x23,0x20,0x21,0x12,0x13,0x16,0x17,0x10,0x11,0x14,0x15,0x06,0x07,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x02,0x03,0x04,0x05,0x00,0x01,0x18,0x18,0x18,0x18,0x26,0x27,0x24,0x25,0x18,0x18,0x18,0x18}},
	{REGFLAG_DELAY, 5, {}},
	{0xE0,0X3A,{0x00,0x0E,0x1C,0x23,0x27,0x2B,0x2F,0x2E,0x5E,0x6C,0x7C,0x7B,0x83,0x94,0x99,0x9B,0xA8,0xAC,0xA9,0xB8,0xCA,0x64,0x63,0x66,0x6C,0x6F,0x79,0x7F,0x7F,0x00,0x0E,0x1C,0x23,0x27,0x2B,0x2F,0x2E,0x5E,0x6C,0x7C,0x7B,0x83,0x94,0x99,0x9B,0xA8,0xAC,0xA9,0xB8,0xCA,0x64,0x63,0x66,0x6C,0x6F,0x79,0x7F,0x7F}},
	{REGFLAG_DELAY, 5, {}},
	{0xC1,0X2B, {0x01,0x00,0x08,0x10,0x18,0x20,0x28,0x31,0x39,0x41,0x48,0x50,0x58,0x60,0x69,0x70,0x78,0x80,0x88,0x90,0x98,0x9F,0xA6,0xAE,0xB6,0xBE,0xC6,0xCE,0xD7,0xDE,0xE7,0xEF,0xF7,0xFF,0x00,0x01,0x27,0x4E,0xE4,0x26,0xC6,0x59,0x00}},
	{0xBD,0X01, {0x01}},
	{0xC1,0X2A, {0x00,0x08,0x10,0x18,0x20,0x29,0x31,0x39,0x41,0x49,0x51,0x59,0x61,0x69,0x71,0x79,0x81,0x88,0x90,0x98,0x9F,0xA7,0xAF,0xB7,0xBE,0xC7,0xCF,0xD7,0xDF,0xE7,0xEF,0xF7,0xFF,0x06,0xCB,0x94,0x55,0x65,0xE8,0xD9,0x9D,0x00}},
	{0xBD,0X01, {0x02}},
	{0xC1,0X2A, {0x00,0x08,0x10,0x19,0x21,0x2A,0x33,0x3B,0x43,0x4B,0x53,0x5B,0x63,0x6C,0x74,0x7C,0x84,0x8C,0x94,0x9C,0xA4,0xAB,0xB3,0xBB,0xC3,0xCC,0xD4,0xDC,0xE3,0xEC,0xF3,0xFA,0xFF,0x2D,0xD1,0x97,0xC6,0x50,0x39,0xDD,0xD6,0xC0}},
	{0xBD,0X01, {0x00}},
	{0xD2,0X01, {0x22}},
	{0xCC,0X01,{0x0B}},
	{0xC0,0X02,{0x1F,0x31}},
	{0xD4,0X01,{0x02}},
	{0xBD,0X01,{0x01}},
	{0xB1,0X01,{0x00}},
	{0xBD,0X01,{0x00}},
	{0xC6,0X01,{0xEF}},
	{0xC9,07,{0x13,0x00,0x14,0x1e,0xb1,0x1e,0x00}},
	{0x35,1,{0x00}},
	{0x51,1,{0x00}},
	{0x53,1,{0x24}},
	{0x55,1,{0x01}},
	{0x5E,1,{0x28}},
	{0x11,0,{}},
	{REGFLAG_DELAY, 120, {} },
	{0x29,0,{}},
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

#ifdef CONFIG_LCT_CABC_MODE_SUPPORT

#define  CABC_MODE_SETTING_DIS 0
#define  CABC_MODE_SETTING_UI 1
#define  CABC_MODE_SETTING_STILL 2
#define  CABC_MODE_SETTING_MV  3

static struct LCM_setting_table lcm_setting_dis[] = {
	{0x55,1,{0x00}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x24}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_ui[] = {
	{0x55,1,{0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x24}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_still[] = {
	{0x55,1,{0x02}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x24}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_mv[] = {
	{0x55,1,{0x03}},
        {REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x2c}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table_v22(void *handle, struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;
	for (i = 0; i < count; i++) {

		unsigned cmd;
		void *cmdq = handle;
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}

}

static void lcm_cabc_cmdq(void *handle, unsigned int mode)
{
	switch(mode){
		case CABC_MODE_SETTING_DIS:
			push_table_v22(handle,lcm_setting_dis, sizeof(lcm_setting_dis) /sizeof(struct LCM_setting_table), 1);
		        break;
		case CABC_MODE_SETTING_UI:
			push_table_v22(handle,lcm_setting_ui, sizeof(lcm_setting_ui) /sizeof(struct LCM_setting_table), 1);
		         break;
		case CABC_MODE_SETTING_STILL:
			 push_table_v22(handle,lcm_setting_still, sizeof(lcm_setting_still) /sizeof(struct LCM_setting_table), 1);
			break;
		case CABC_MODE_SETTING_MV:
			 push_table_v22(handle,lcm_setting_mv, sizeof(lcm_setting_mv) /sizeof(struct LCM_setting_table), 1);
		         break;
		default:
			 push_table_v22(handle,lcm_setting_ui, sizeof(lcm_setting_ui) /sizeof(struct LCM_setting_table), 1);
	}
}
#endif
#ifdef CONFIG_LCT_SCAN_MODE_SUPPORT

#define  NORMAL_SCAN 0
#define  REVERSE_SCAN 1

static struct LCM_setting_table lcm_setting_normalscan[] = {
	{0xCC,1,{0x0b}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_reversescan[] = {
	{0xCC,1,{0x07}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void lcm_scanmode_cmdq(void *handle, unsigned int mode)
{
	switch(mode){
		case NORMAL_SCAN:
			push_table_v22(handle,lcm_setting_normalscan, sizeof(lcm_setting_normalscan) /sizeof(struct LCM_setting_table), 1);
		        break;
		case REVERSE_SCAN:
			push_table_v22(handle,lcm_setting_reversescan, sizeof(lcm_setting_reversescan) /sizeof(struct LCM_setting_table), 1);
		         break;
		default:
			 push_table_v22(handle,lcm_setting_normalscan, sizeof(lcm_setting_normalscan) /sizeof(struct LCM_setting_table), 1);
	}
}
#endif

#ifdef CONFIG_LCT_INVERSION_MODE_SUPPORT

#define	COLUMN_INVERSION  0
#define	DOT_INVERSION_1  1
#define	DOT_INVERSION_2 2
#define	DOT_INVERSION_4 3
#define	DOT_INVERSION_8 4

static struct LCM_setting_table lcm_setting_column[] = {
	{0xB2,1,{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_1_dot[] = {
	{0xB2,1,{0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_2_dot[] = {
	{0xB2,1,{0x02}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_4_dot[] = {
	{0xB2,1,{0x03}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_8_dot[] = {
	{0xB2,1,{0x04}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_inversionmode_cmdq(void *handle, unsigned int mode)
{
	switch(mode){
		case COLUMN_INVERSION:
			push_table_v22(handle,lcm_setting_column, sizeof(lcm_setting_column) /sizeof(struct LCM_setting_table), 1);
		        break;
		case DOT_INVERSION_1:
			push_table_v22(handle,lcm_setting_1_dot, sizeof(lcm_setting_1_dot) /sizeof(struct LCM_setting_table), 1);
		         break;
		case DOT_INVERSION_2:
			push_table_v22(handle,lcm_setting_2_dot, sizeof(lcm_setting_2_dot) /sizeof(struct LCM_setting_table), 1);
		         break;
		case DOT_INVERSION_4:
			push_table_v22(handle,lcm_setting_4_dot, sizeof(lcm_setting_4_dot) /sizeof(struct LCM_setting_table), 1);
		         break;
		case DOT_INVERSION_8:
			push_table_v22(handle,lcm_setting_8_dot, sizeof(lcm_setting_8_dot) /sizeof(struct LCM_setting_table), 1);
		         break;
		default:
			 push_table_v22(handle,lcm_setting_column, sizeof(lcm_setting_column) /sizeof(struct LCM_setting_table), 1);
	}
}
#endif

static struct LCM_setting_table bl_level[] = {
	{0x51, 1, {0x7F} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				if (table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;

			case REGFLAG_UDELAY:
				UDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	lcm_dsi_mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif
	LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 8;
	params->dsi.vertical_frontporch = 15;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 50;
	params->dsi.horizontal_backporch = 50;
	params->dsi.horizontal_frontporch = 50;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 0;
	params->dsi.ssc_range = 3;
	params->dsi.HS_TRAIL = 5; //36;
	params->dsi.PLL_CLOCK = 218;    /* this value must be in MTK suggested table */
	params->dsi.PLL_CK_CMD = 218;
	params->dsi.PLL_CK_VDO = 218;
	params->dsi.CLK_HS_POST = 36;

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
    params->dsi.cont_clock =0; //wzx add clock is not continuos.
	params->dsi.lcm_esd_check_table[0].cmd = 0x09;
	params->dsi.lcm_esd_check_table[0].count = 3;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[0].para_list[2] = 0x06;
	params->dsi.lcm_esd_check_table[1].cmd = 0xD9;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;
	//printk("wzxlcm_get_params\n");


}

#ifdef BUILD_LK
static struct mt_i2c_t NT50358A_i2c;

static int NT50358A_write_byte(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0] = addr;
	write_data[1] = value;

	NT50358A_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL; /* I2C2; */
	/* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
	NT50358A_i2c.addr = LCD_BIAS_ADDR;
	NT50358A_i2c.mode = ST_MODE;
	NT50358A_i2c.speed = 100;
	len = 2;

	ret_code = i2c_write(&NT50358A_i2c, write_data, len);
	/* printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code); */

	return ret_code;
}

#endif

#define GPIO_LCD_RST_PIN         (GPIO158 | 0x80000000)
#define GPIO_LCD_BIAS_ENN_PIN         (GPIO11 | 0x80000000)
#define GPIO_LCD_BIAS_ENP_PIN         (GPIO12 | 0x80000000)

static void lcm_reset(void)
{
	//printf("[uboot]:lcm reset start.\n");
#ifdef GPIO_LCD_RST_PIN
	mt_set_gpio_mode(GPIO_LCD_RST_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ONE);
	MDELAY(1);
	mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ZERO);
	MDELAY(5);
	mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ONE);
	MDELAY(55);
#else
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(5);

	SET_RESET_PIN(1);
	MDELAY(55);

#endif
	printk("kernel:lcm reset end.\n");
}

static void lcm_init_power(void)
{
	/*#ifndef MACH_FPGA
	#ifdef BUILD_LK
	    mt6325_upmu_set_rg_vgp1_en(1);
	#else
	    LCM_LOGI("%s, begin\n", __func__);
	    hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");
	    LCM_LOGI("%s, end\n", __func__);
	#endif
	#endif*/
}

static void lcm_suspend_power(void)
{
	/*#ifndef MACH_FPGA
	#ifdef BUILD_LK
	    mt6325_upmu_set_rg_vgp1_en(0);
	#else
	    LCM_LOGI("%s, begin\n", __func__);
	    hwPowerDown(MT6325_POWER_LDO_VGP1, "LCM_DRV");
	    LCM_LOGI("%s, end\n", __func__);
	#endif
	#endif*/
}

static void lcm_resume_power(void)
{
	/*#ifndef MACH_FPGA
	#ifdef BUILD_LK
	    mt6325_upmu_set_rg_vgp1_en(1);
	#else
	    LCM_LOGI("%s, begin\n", __func__);
	    hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");
	    LCM_LOGI("%s, end\n", __func__);
	#endif
	#endif*/
}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0x0F;  //up to +/-5.5V
	int ret = 0;

	printk( "_____lcm_init%s: TDO_HX8394F\n",__func__);

#if defined(GPIO_LCD_BIAS_ENN_PIN)||defined(GPIO_LCD_BIAS_ENP_PIN)
#ifdef GPIO_LCD_BIAS_ENP_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
#endif

#ifdef GPIO_LCD_BIAS_ENN_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
#endif

#ifdef GPIO_LCD_BIAS_ENP_PIN
	//MDELAY(20);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#endif

#ifdef GPIO_LCD_BIAS_ENN_PIN
	//MDELAY(5);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
#endif
#else
	set_gpio_lcd_enp(1);
#endif

	ret = NT50358A_write_byte(cmd, data);

	if (ret < 0)
		LCM_LOGI("tdo_hx8394f----nt50358a----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("tdo_hx8394f----nt50358a----cmd=%0x--i2c write success----\n", cmd);

	cmd = 0x01;
	data = 0x0F;

	ret = NT50358A_write_byte(cmd, data);

	if (ret < 0)
		LCM_LOGI("tdo_hx8394f----nt50358a----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("tdo_hx8394f----nt50358a----cmd=%0x--i2c write success----\n", cmd);

	lcm_reset();

	push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
#ifndef MACH_FPGA

	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
	if(lcdkit_info.panel_infos.enable_PT_test == 0)
	{
#if defined(GPIO_LCD_BIAS_ENN_PIN)||defined(GPIO_LCD_BIAS_ENP_PIN)

#ifdef GPIO_LCD_BIAS_ENN_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
#endif

#ifdef GPIO_LCD_BIAS_ENP_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
#endif
#else
	set_gpio_lcd_enp(0);
#endif
	}

#endif
	/* SET_RESET_PIN(0);
	MDELAY(10); */
}

static void lcm_resume(void)
{
	lcm_init();
}

#define LCM_ID_TDO_HX8394F (0x8394)

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];

	unsigned int data_array[16];

	lcm_reset();

	data_array[0] = 0x00043902;
	data_array[1] = 0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0] = 0x00043902;
	data_array[1] = 0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, buffer, 3);
	id = (buffer[0] << 8) | buffer[1];	/* we only need ID */

	printk("%s,tdo_hx8394f id = 0x%08x\n", __func__, id);

	if (id == LCM_ID_TDO_HX8394F)
		return 1;
	else
		return 0;
}


#define LCDKIT_REG_CHECK_NUM    5
static ssize_t lcdkit_check_reg_show(void* pdata, char* buf)
{
    int ret = 0;
#if 1
    char expect_ptr[3] = {0x80,0x73,0x06};
    unsigned int count;
    unsigned char value[3];
#ifdef CONFIG_HUAWEI_DSM
    char *reg_name = "0x09";
#endif

    for (count = 0; count < LCDKIT_REG_CHECK_NUM; count++)
    {
        ret = 0;
         lcdkit_dsi_rx_block_data(0x09,value,3);

        if((value[0] != expect_ptr[0]) || (value[1] != expect_ptr[1]) || (value[2] != expect_ptr[2]))
        {
            ret = 1;
            LCM_LOGI("%s value[0]=0x%0x,value[1]=0x%0x,value[2]=0x%0x\n",__func__,value[0],value[1],value[2]);
            LCM_LOGI("%s expect_ptr[0]=0x%0x,expect_ptr[1]=0x%0x,expect_ptr=0x%0x\n",__func__,expect_ptr[0],expect_ptr[1],expect_ptr[2]);
            continue;
        }
            LCM_LOGI("%s value[0]=0x%0x,value[1]=0x%0x,value[2]=0x%0x\n",__func__,value[0],value[1],value[2]);
            LCM_LOGI("%s expect_ptr[0]=0x%0x,expect_ptr[1]=0x%0x,expect_ptr=0x%0x\n",__func__,expect_ptr[0],expect_ptr[1],expect_ptr[2]);
        if (0 == ret)
        {
            break;
        }
    }
#endif
    if (0 == ret)
    {
        ret = snprintf(buf, PAGE_SIZE, "OK\n");
    }
    else
    {
        ret = snprintf(buf, PAGE_SIZE, "ERROR\n");
#if 1
#ifdef CONFIG_HUAWEI_DSM
        lcdkit_report_dsm_err(DSM_LCD_STATUS_ERROR_NO, reg_name, 0, 0);
#endif
#endif
    }
    return ret;
}


static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	return ret;
#else
	return 0;
#endif
}



static void lcm_setbacklight(void *handle, unsigned int level)
{
	//if (level) level=100;//for test
	bl_level[0].para_list[0] = level;
	printk( "%s,tdo_lcm_setbacklight: level = %d\n", __func__, level);
	push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}

#ifdef CONFIG_LCDKIT_DRIVER
#include "lcdkit_fb_util.h"
#endif

static void  set_lcdkit_info(void)
{
           lcdkit_info.panel_infos.cabc_support = 1;
           lcdkit_info.panel_infos.cabc_mode = 0;
           lcdkit_info.panel_infos.inversion_support = 1;
           lcdkit_info.panel_infos.inversion_mode = 0;
           lcdkit_info.panel_infos.scan_support = 1;
           lcdkit_info.panel_infos.scan_mode = 0;
           lcdkit_info.panel_infos.esd_support = 1;
           lcdkit_info.panel_infos.mipi_detect_support = 1;
           lcdkit_info.panel_infos.lp2hs_mipi_check_support = 1;
           lcdkit_info.panel_infos.check_reg_support = 1;
           lcdkit_info.panel_infos.bias_power_ctrl_mode |= POWER_CTRL_BY_GPIO;
           lcdkit_info.panel_infos.gpio_lcd_vsn = 90;
           lcdkit_info.panel_infos.gpio_lcd_vsp = 17;
           lcdkit_info.panel_infos.enable_PT_test = 0;
           lcdkit_info.panel_infos.PT_test_support = 1;
           lcdkit_info.panel_infos.model_name = "TRULY_HX8394F 5.5' VIDEO TFT 720x1280";
           lcdkit_info.lcdkit_check_reg_show = lcdkit_check_reg_show;
           lcdkit_info.panel_infos.bl_level_max = 255;
           lcdkit_info.panel_infos.bl_level_min = 4;
           lcdkit_info.panel_infos.bl_max_nit = 0;
           lcdkit_info.panel_infos.panel_status_cmds = 0x0a;
           lcdkit_info.panel_infos.inversion_mode_cmds = 0xb2;
           lcdkit_info.panel_infos.scan_mode_cmds = 0xcc;
           lcdkit_info.panel_infos.scan_mode_normal_data = 0x0b;
           lcdkit_info.panel_infos.scan_mode_reverse_data = 0x07;
}

LCM_DRIVER ontim_hd720_hx8394_k50164_truly_lcm_drv = {
	.name = "k50164_truly_hx8394f_hd720",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
//	.esd_check = lcm_esd_check,
	.set_backlight_cmdq = lcm_setbacklight,
	.ata_check = lcm_ata_check,
//	.update = lcm_update,
//	.switch_mode = lcm_switch_mode,
	.set_lcm_panel_support = set_lcdkit_info,
#ifdef CONFIG_LCT_CABC_MODE_SUPPORT
	.set_cabc_cmdq = lcm_cabc_cmdq,
#endif
#ifdef CONFIG_LCT_SCAN_MODE_SUPPORT
	.set_scanmode_cmdq = lcm_scanmode_cmdq,
#endif
#ifdef CONFIG_LCT_INVERSION_MODE_SUPPORT
	.set_inversionmode_cmdq = lcm_inversionmode_cmdq,
#endif
};
