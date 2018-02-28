/*
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
/*
 * Definitions for CM3218 als sensor chip.
 */
#ifndef __CM32180_H__
#define __CM32180_H__

#include <linux/ioctl.h>

/* CM32180 light sensor register related macro */
#define CM32180_REG_ALS_CONF		0X00
#define CM32180_REG_ALS_THDH		0X01
#define CM32180_REG_ALS_THDL		0X02
#define CM32180_REG_ALS_DATA			0X04

/* CM32180 command code 00 register bits */
#define CM32180_ALS_CONF_SD				0x0001
#define CM32180_ALS_CONF_INT_EN			0x0002
#define CM32180_ALS_CONF_PERS_MASK		0x0030
#define CM32180_ALS_CONF_PERS_1			0x0000
#define CM32180_ALS_CONF_PERS_2			0x0010
#define CM32180_ALS_CONF_PERS_4			0x0020
#define CM32180_ALS_CONF_PERS_8			0x0030
#define CM32180_ALS_CONF_IT_MASK			0x00C0
#define CM32180_ALS_CONF_IT_0_5T			0x0000
#define CM32180_ALS_CONF_IT_1T			0x0040
#define CM32180_ALS_CONF_IT_2T			0x0080
#define CM32180_ALS_CONF_IT_4T			0x00C0
#define CM32180_ALS_CONF_SM_MASK			0x1800
#define CM32180_ALS_CONF_SM_x1			0x0000
#define CM32180_ALS_CONF_SM_x2			0x0800
#define CM32180_ALS_CONF_SM_x0_5			0x1000
#define CM32180_ALS_CONF_DEFAULT			0x0004

#define CM32180_ALS_CONF_HI			CM32180_ALS_CONF_SM_x2
#define CM32180_ALS_CONF_LO			(CM32180_ALS_CONF_DEFAULT | CM32180_ALS_CONF_IT_1T)

#define CM32180_ALS_CONF_CONFIG_ON	(CM32180_ALS_CONF_HI | CM32180_ALS_CONF_LO)
#define CM32180_ALS_CONF_CONFIG_OFF	(CM32180_ALS_CONF_CONFIG_ON | CM32180_ALS_CONF_SD)

#define CM32180_ALS_CONF_CONFIG_ON_INT	(CM32180_ALS_CONF_CONFIG_ON | CM32180_ALS_CONF_INT_EN)
#define CM32180_ALS_CONF_CONFIG_OFF_INT	(CM32180_ALS_CONF_CONFIG_ON_INT | CM32180_ALS_CONF_SD)

/*CM32180 related driver tag macro*/
#define CM32180_SUCCESS						 0
#define CM32180_ERR_I2C						-1
#define CM32180_ERR_STATUS					-3
#define CM32180_ERR_SETUP_FAILURE			-4
#define CM32180_ERR_GETGSENSORDATA			-5
#define CM32180_ERR_IDENTIFICATION			-6

/*----------------------------------------------------------------------------*/
typedef enum {
	CM32180_NOTIFY_PROXIMITY_CHANGE = 1,
} CM32180_NOTIFY_TYPE;
/*----------------------------------------------------------------------------*/
typedef enum {
	CM32180_CUST_ACTION_SET_CUST = 1,
	CM32180_CUST_ACTION_CLR_CALI,
	CM32180_CUST_ACTION_SET_CALI,
	CM32180_CUST_ACTION_SET_PS_THRESHODL
} CM32180_CUST_ACTION;
/*----------------------------------------------------------------------------*/
typedef struct {
	uint16_t action;
} CM32180_CUST;
/*----------------------------------------------------------------------------*/
typedef struct {
	uint16_t action;
	uint16_t part;
	int32_t data[0];
} CM32180_SET_CUST;
/*----------------------------------------------------------------------------*/
typedef CM32180_CUST CM32180_CLR_CALI;
/*----------------------------------------------------------------------------*/
typedef struct {
	uint16_t action;
	int32_t cali;
} CM32180_SET_CALI;
/*----------------------------------------------------------------------------*/
typedef struct {
	uint16_t action;
	int32_t threshold[2];
} CM32180_SET_PS_THRESHOLD;
/*----------------------------------------------------------------------------*/
typedef union {
	uint32_t data[10];
	CM32180_CUST cust;
	CM32180_SET_CUST setCust;
	CM32180_CLR_CALI clearCali;
	CM32180_SET_CALI setCali;
	CM32180_SET_PS_THRESHOLD setPSThreshold;
} CM32180_CUST_DATA;
/*----------------------------------------------------------------------------*/

#endif
