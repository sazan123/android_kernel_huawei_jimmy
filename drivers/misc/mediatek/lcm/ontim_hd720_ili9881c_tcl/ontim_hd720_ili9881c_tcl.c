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
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
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
           {0xFF,3,{0x98,0x81,0x03}},
           {0x01,1,{0x00}},
           {0x02,1,{0x00}},
           {0x03,1,{0x53}},
           {0x04,1,{0x14}},
           {0x05,1,{0x00}},
           {0x06,1,{0x06}},
           {0x07,1,{0x01}},
           {0x08,1,{0x00}},
           {0x09,1,{0x01}},
           {0x0a,1,{0x19}},
           {0x0b,1,{0x01}},
           {0x0c,1,{0x00}},
           {0x0d,1,{0x00}},
           {0x0e,1,{0x00}},
           {0x0f,1,{0x19}},
           {0x10,1,{0x19}},
           {0x11,1,{0x00}},
           {0x12,1,{0x00}},
           {0x13,1,{0x00}},
           {0x14,1,{0x00}},
           {0x15,1,{0x02}},
           {0x16,1,{0x00}}, 
           {0x17,1,{0x00}}, 
           {0x18,1,{0x00}},
           {0x19,1,{0x00}},
           {0x1a,1,{0x00}},
           {0x1b,1,{0x00}},
           {0x1c,1,{0x00}},
           {0x1d,1,{0x00}},
           {0x1e,1,{0x40}},
           {0x1f,1,{0x40}},
           {0x20,1,{0x02}},
           {0x21,1,{0x05}},
           {0x22,1,{0x02}},
           {0x23,1,{0x00}},
           {0x24,1,{0x87}},
           {0x25,1,{0x87}},
           {0x26,1,{0x00}},
           {0x27,1,{0x00}},
           {0x28,1,{0x3B}},
           {0x29,1,{0x03}},
           {0x2a,1,{0x00}},
           {0x2b,1,{0x00}},
           {0x2c,1,{0x00}},
           {0x2d,1,{0x00}},
           {0x2e,1,{0x00}}, 
           {0x2f,1,{0x00}},
           {0x30,1,{0x00}},
           {0x31,1,{0x00}},
           {0x32,1,{0x00}},
           {0x33,1,{0x00}},
           {0x34,1,{0x04}},
           {0x35,1,{0x00}},
           {0x36,1,{0x00}},
           {0x37,1,{0x00}},
           {0x38,1,{0x01}},
           {0x39,1,{0x01}},
           {0x3a,1,{0x40}}, 
           {0x3b,1,{0x40}},
           {0x3c,1,{0x00}},
           {0x3d,1,{0x00}},
           {0x3e,1,{0x00}},
           {0x3f,1,{0x00}},
           {0x40,1,{0x00}},
           {0x41,1,{0x88}},
           {0x42,1,{0x00}},
           {0x43,1,{0x00}},
           {0x44,1,{0x00}},
           {0x50,1,{0x01}},
           {0x51,1,{0x23}},
           {0x52,1,{0x45}},
           {0x53,1,{0x67}},
           {0x54,1,{0x89}},
           {0x55,1,{0xab}}, 
           {0x56,1,{0x01}},
           {0x57,1,{0x23}},
           {0x58,1,{0x45}},
           {0x59,1,{0x67}},
           {0x5a,1,{0x89}},
           {0x5b,1,{0xab}},
           {0x5c,1,{0xcd}},
           {0x5d,1,{0xef}},
           {0x5e,1,{0x11}},
           {0x5f,1,{0x08}},
           {0x60,1,{0x00}},
           {0x61,1,{0x01}},
           {0x62,1,{0x02}},
           {0x63,1,{0x02}},
           {0x64,1,{0x0F}},
           {0x65,1,{0x0D}},
           {0x66,1,{0x0E}},
           {0x67,1,{0x0C}},
           {0x68,1,{0x02}},
           {0x69,1,{0x02}},
           {0x6a,1,{0x02}},
           {0x6b,1,{0x02}},
           {0x6c,1,{0x02}},
           {0x6d,1,{0x02}},
           {0x6e,1,{0x06}},
           {0x6f,1,{0x02}},
           {0x70,1,{0x02}},
           {0x71,1,{0x02}},
           {0x72,1,{0x02}},
           {0x73,1,{0x02}},
           {0x74,1,{0x02}},
           {0x75,1,{0x06}},
           {0x76,1,{0x00}},
           {0x77,1,{0x01}},
           {0x78,1,{0x02}},
           {0x79,1,{0x02}},
           {0x7a,1,{0x0F}},
           {0x7b,1,{0x0D}},
           {0x7c,1,{0x0E}},
           {0x7d,1,{0x0C}},
           {0x7e,1,{0x02}},
           {0x7f,1,{0x02}},
           {0x80,1,{0x02}},
           {0x81,1,{0x02}},
           {0x82,1,{0x02}},
           {0x83,1,{0x02}},
           {0x84,1,{0x08}},
           {0x85,1,{0x02}},
           {0x86,1,{0x02}},
           {0x87,1,{0x02}},
           {0x88,1,{0x02}},
           {0x89,1,{0x02}},
           {0x8A,1,{0x02}},
           {0xFF,3,{0x98,0x81,0x04}},
           {0x6C,1,{0x15}},
           {0x6E,1,{0x2B}},
           {0x6F,1,{0x33}},
           {0x3A,1,{0x24}},
           {0x35,1,{0x1F}},
           {0x33,1,{0x00}},
           {0x7A,1,{0x0F}},
           {0x8D,1,{0x15}},
           {0x87,1,{0xBA}},
           {0x26,1,{0x76}},
           {0xB2,1,{0xD1}},
           {0xFF,3,{0x98,0x81,0x01}},
           {0x22,1,{0x0A}},
           {0x53,1,{0x82}},
           {0x55,1,{0x85}},
           {0x50,1,{0xA7}},
           {0x51,1,{0xA7}},
           {0x31,1,{0x00}},
           {0x60,1,{0x14}},
           {0x63,1,{0x00}},//VP243
    {0xA0, 1, {0x0F}}, //VP255Gamma P
    {0xA1, 1, {0x37}}, //VP251
    {0xA2, 1, {0x47}}, //VP247
    {0xA3, 1, {0x0F}}, //VP243
    {0xA4, 1, {0x12}}, //VP239
    {0xA5, 1, {0x25}}, //VP231
    {0xA6, 1, {0x19}}, //VP219
    {0xA7, 1, {0x1D}}, //VP203
    {0xA8, 1, {0xAA}}, //VP175
    {0xA9, 1, {0x1A}}, //VP144
    {0xAA, 1, {0x28}}, //VP111
    {0xAB, 1, {0x93}}, //VP80
    {0xAC, 1, {0x1D}}, //VP52
    {0xAD, 1, {0x1C}}, //VP36
    {0xAE, 1, {0x50}}, //VP24
    {0xAF, 1, {0x24}}, //VP16
    {0xB0, 1, {0x2A}}, //VP12
    {0xB1, 1, {0x57}}, //VP8
    {0xB2, 1, {0x65}}, //VP4
    {0xB3, 1, {0x39}}, //VP0
    {0xC0, 1, {0x00}}, //VN255 GAMMA N
    {0xC1, 1, {0x1D}}, //VN251
    {0xC2, 1, {0x29}}, //VN247
    {0xC3, 1, {0x16}}, //VN243
    {0xC4, 1, {0x19}}, //VN239
    {0xC5, 1, {0x2D}}, //VN231
    {0xC6, 1, {0x1E}}, //VN219
    {0xC7, 1, {0x1E}}, //VN203
    {0xC8, 1, {0x91}}, //VN175
    {0xC9, 1, {0x1C}}, //VN144
    {0xCA, 1, {0x29}}, //VN111
    {0xCB, 1, {0x85}}, //VN80
    {0xCC, 1, {0x1E}}, //VN52
    {0xCD, 1, {0x1D}}, //VN36
    {0xCE, 1, {0x4F}}, //VN24
    {0xCF, 1, {0x25}}, //VN16
    {0xD0, 1, {0x29}}, //VN12
    {0xD1, 1, {0x4E}}, //VN8
    {0xD2, 1, {0x5C}}, //VN4
           {0xD3,1,{0x39}},//VN0 
        {0xFF,3,{0x98,0x81,0x02}},
        {0x03,1,{0xDD}},
        {0x04,1,{0x45}},
        {0x05,1,{0x44}},
        {0x06,1,{0x20}},
        {0x07,1,{0x00}},
           {0xFF,3,{0x98,0x81,0x00}},
           {0x11,0,{}},
           {REGFLAG_DELAY, 120, {} },
           {0x29,0,{}},
           {REGFLAG_DELAY, 20, {} },
           {0x35,1,{0x00}},
           {0x53,1,{0x2c}},
           {0x51,2,{0x00,0x00}},
           {0x55,1,{0x01}},
           {0x68, 2, {0x03,0x01}},
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
	{0x53,1,{0x2c}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_ui[] = {
	{0x55,1,{0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x2c}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_still[] = {
	{0x55,1,{0x02}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x2c}},
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
	{0xFF,3,{0x98,0x81,01}},
	{REGFLAG_DELAY, 5, {}},
	{0x22,1,{0x0A}},
	{REGFLAG_DELAY, 5, {}},
	{0xFF,3,{0x98,0x81,00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_reversescan[] = {
	{0xFF,3,{0x98,0x81,01}},
	{REGFLAG_DELAY, 5, {}},
	{0x22,1,{0x09}},
	{REGFLAG_DELAY, 5, {}},
	{0xFF,3,{0x98,0x81,00}},
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
	{0x31,1,{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_1_dot[] = {
	{0x31,1,{0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_2_dot[] = {
	{0x31,1,{0x02}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_4_dot[] = {
	{0x31,1,{0x03}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_8_dot[] = {
	{0x31,1,{0x04}},
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
	{0x51, 2, {0x07,0xFF} },
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
	printk(KERN_ERR "%s: TCL_ILI9881C\n",__func__);
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
	params->dsi.vertical_sync_active = 2; //old is 2,now is 2
	params->dsi.vertical_backporch = 18; //old is 8,now is 18
	params->dsi.vertical_frontporch = 8; //old is 15,now is 24
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 30;
	params->dsi.horizontal_backporch = 32;
	params->dsi.horizontal_frontporch = 36;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 0;
	params->dsi.ssc_range = 3;
	params->dsi.HS_TRAIL = 5;
	params->dsi.PLL_CLOCK = 204;    /* this value must be in MTK suggested table */
	params->dsi.PLL_CK_CMD = 204;
	params->dsi.PLL_CK_VDO = 204;
	params->dsi.CLK_HS_POST = 36;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	params->dsi.lcm_esd_check_table[1].cmd = 0x0b;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
	params->dsi.lcm_esd_check_table[2].cmd = 0x0d;
	params->dsi.lcm_esd_check_table[2].count = 1;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0x00;
	params->dsi.lcm_esd_check_table[3].cmd = 0x54;
	params->dsi.lcm_esd_check_table[3].count = 1;
	params->dsi.lcm_esd_check_table[3].para_list[0] = 0x54;
	params->dsi.lcm_esd_check_table[3].para_list[1] = 0x24;
	params->dsi.lcm_esd_check_table[3].para_list[2] = 0x2C;
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
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ONE);
	MDELAY(10);
#else
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);

#endif
//	printf("[uboot]:lcm reset end.\n");
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

	printk(KERN_ERR "%s: TCL_ILI9881C\n",__func__);

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
	MDELAY(5);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
#endif
#else
	set_gpio_lcd_enp(1);
#endif

	ret = NT50358A_write_byte(cmd, data);

	if (ret < 0)
		LCM_LOGI("tcl_ili9881c----nt50358a----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("tcl_ili9881c----nt50358a----cmd=%0x--i2c write success----\n", cmd);

	cmd = 0x01;
	data = 0x0F;

	ret = NT50358A_write_byte(cmd, data);

	if (ret < 0)
		LCM_LOGI("tcl_ili9881c----nt50358a----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("tcl_ili9881c----nt50358a----cmd=%0x--i2c write success----\n", cmd);

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

#define LCM_ID_TCL_ILI9881C (0x9881)

static struct LCM_setting_table read_id[] = {
	{0xFF, 3, {0x98,0x81,0x01} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static unsigned int lcm_compare_id(void)
{
	unsigned char id1 = 0;
	unsigned char id2 = 0;
	unsigned int id= 0;
	
	lcm_reset();

	push_table(NULL, read_id, sizeof(read_id) / sizeof(struct LCM_setting_table), 1);
	read_reg_v2(0x00, &id1, 1); 
	read_reg_v2(0x01, &id2, 1); 

	id=(id1<<8)|id2;

	LCM_LOGI("%s,tcl_ili9881c debug: tcl_ili9881c id = 0x%08x\n", __func__, id);

	if (id == LCM_ID_TCL_ILI9881C)
		return 1;
	else
		return 0;

}

#define LCDKIT_REG_CHECK_NUM    5
static ssize_t lcdkit_check_reg_show(void* pdata, char* buf)
{
    int ret = 0;
#if 1
    char expect_ptr[1] = {0x9C};
    unsigned int count;
    unsigned char value[3];
#ifdef CONFIG_HUAWEI_DSM
    char *reg_name = "0x0A";
#endif

    for (count = 0; count < LCDKIT_REG_CHECK_NUM; count++)
    {
        ret = 0;
         lcdkit_dsi_rx_block_data(0x0A,value,1);

        if(value[0] != expect_ptr[0])
        {
            ret = 1;
            LCM_LOGI("%s value[0]=0x%0x\n",__func__,value[0]);
            LCM_LOGI("%s expect_ptr[0]=0x%0x\n",__func__,expect_ptr[0]);
            continue;
        }
            LCM_LOGI("%s value[0]=0x%0x\n",__func__,value[0]);
            LCM_LOGI("%s expect_ptr[0]=0x%0x\n",__func__,expect_ptr[0]);
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
        lcdkit_report_dsm_err(DSM_LCD_STATUS_ERROR_NO, reg_name, value[0], expect_ptr[0]);
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
	//int old_level=(bl_level[0].para_list[0]<<8)||bl_level[0].para_list[1];
	

	bl_level[0].para_list[0] = level>>4;
	bl_level[0].para_list[1] = ((level & 0x0F)<<4)|(level & 0x0F);
	printk(KERN_ERR "%s,tcl_ili9881c backlight: level = %d, set to 0x%x,0x%x \n", __func__, level,bl_level[0].para_list[0],bl_level[0].para_list[1]);
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
           lcdkit_info.panel_infos.model_name = "TCL_ILI9881C 5.5' VIDEO TFT 720x1280";
           lcdkit_info.lcdkit_check_reg_show = lcdkit_check_reg_show;
           lcdkit_info.panel_infos.bl_level_max = 255;
           lcdkit_info.panel_infos.bl_level_min = 4;
           lcdkit_info.panel_infos.bl_max_nit = 0;
           lcdkit_info.panel_infos.panel_status_cmds = 0x0a;
           lcdkit_info.panel_infos.inversion_mode_cmds = 0x31;
           lcdkit_info.panel_infos.scan_mode_cmds = 0x36;
           lcdkit_info.panel_infos.scan_mode_normal_data = 0x00;
           lcdkit_info.panel_infos.scan_mode_reverse_data = 0x07;
}

LCM_DRIVER ontim_hd720_ili9881c_tcl_lcm_drv = {
	.name = "tcl_ili9881c_hd720",
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
