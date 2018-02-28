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

#ifndef _LINUX_ELAN_KTF2K_H
#define _LINUX_ELAN_KTF2K_H

/* Touch firmware resolution. */
#define ELAN_X_MAX	 2496	/* 576//960//576 */
#define ELAN_Y_MAX	 1472	/* 1024//1856 //1792//960 */
/* Touch firmware resolution. */


#define LCM_X_MAX	1920	/* simple_strtoul(LCM_WIDTH, NULL, 0)//896 */
#define LCM_Y_MAX	1200	/* simple_strtoul(LCM_HEIGHT, NULL, 0)//1728 */

#define ELAN_KEY_BACK	0x81	/* Elan Key's define */
#define ELAN_KEY_HOME	0x41
#define ELAN_KEY_MENU	0x21
/* #define ELAN_KEY_SEARCH       0x11 */

#define ELAN_KTF2K_NAME "elan-ktf2k"

/**********function control start************/
#define PROTOCOL_A		/* multi-touch protocol  */
/* #define PROTOCOL_B */
#define FINGER_NUM 10
/* #define SENSOR_OPTION */

#define ELAN_PROTOCOL
#define ELAN_BUFFER_MODE
/* #define ELAN_BUTTON */
/* #define ELAN_2WIREICE */
#define ELAN_POWER_SOURCE
/*#define ELAN_RESUME_RST*/
#define POWER_OFF_IN_SUSPEND
#define DEVICE_NAME "elan_ktf"
#define EKTF3K_FLASH
/* #define ELAN_HID_I2C  for hid over i2c protocol  */
/*dma declare, allocate and release*/
#define __MSG_DMA_MODE__
/* #define ESD_CHECK */

/* #define _ENABLE_DBG_LEVEL */
#define TPD_HAVE_CALIBRATION
/**********function control end********/

#ifdef ELAN_HID_I2C
#define PACKET_SIZE		67	/* support 5 fingers elan hid packet */
#else
#define PACKET_SIZE		55	/* support 10 fingers packet for nexus7 55 */
#define FW_POS_PRESSURE		45
#define FW_POS_WIDTH		35
#endif
#define MAX_FINGER_SIZE			255
#define MAX_FINGER_PRESSURE		255
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK		BIT(3)
#define CMD_S_PKT		0x52
#define CMD_R_PKT		0x53
#define CMD_W_PKT		0x54
#define RESET_PKT		0x77
#define CALIB_PKT		0x66
#define IamAlive_PKT	0x78
#define PEN_PKT			0x71
#define HELLO_PKT			0x55
#define TWO_FINGERS_PKT		0x5A
#define FIVE_FINGERS_PKT	0x5D
#define MTK_FINGERS_PKT		0x6D
#define TEN_FINGERS_PKT		0x62
#define ELAN_HID_PKT		0x3F
#define BUFFER_PKT			0x63
#define BUFFER55_PKT		0x66
#define ELAN_POWERON_DELAY_USEC	500
#define ELAN_RESET_DELAY_MSEC	20
#define IAP_PORTION
#define PAGERETRY  30
#define IAPRESTART 5

/* For Firmware Update */
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_FW_INFO  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)

#define CUSTOMER_IOCTLID	0xA0
#define IOCTL_CIRCUIT_CHECK  _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)

/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_ERROR  1
#define DEBUG_INFO     2
#define DEBUG_MESSAGES 5
#define DEBUG_TRACE   10
#define SYSFS_MAX_LEN 100

/* request fw */
#ifdef SENSOR_OPTION
#define ELAN_FW_FILENAME_TG_AUO  "elan_fw_tg_auo.ekt"
#define ELAN_FW_FILENAME_TG_INX  "elan_fw_tg_inx.ekt"
#define ELAN_FW_FILENAME_HLT_AUO  "elan_fw_hlt_auo.ekt"
#define ELAN_FW_FILENAME_HLT_INX  "elan_fw_hlt_inx.ekt"
/* define fw id */
#define FWID_TG_AUO 0x0db0
#define FWID_TG_INX 0x0db1
#define FWID_HLT_AUO 0x0db2
#define FWID_HLT_INX 0x0db3
#else
#define ELAN_FW_FILENAME  "elan_fw.ekt"
#include "fw_data.h"
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
#define TPD_CALIBRATION_MATRIX_ROTATION {0, -2498, 4915200, 2445, 0, 0, 0, 0}
#endif

/* Log define */
#if 0
#define ETP_INFO(fmt, arg...)           pr_err("[elan_info] "fmt"\n", ##arg)
#else
#define ETP_INFO(fmt, arg...)
#endif

#define ETP_ERROR(fmt, arg...)          pr_err("[elan_err] "fmt"\n", ##arg)
#if 0
#define ETP_DEBUG(fmt, arg...) \
	pr_err("[elan_dbg] [%d]"fmt"\n", __LINE__, ##arg)
#else
#define ETP_DEBUG(fmt, arg...)
#endif

struct elan_ktf2k_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int (*power)(int on);
};

/* softkey is reported as AXIS */
/* #define SOFTKEY_AXIS_VER */

/* Orig. point at upper-right, reverse it. */
/* #define REVERSE_X_AXIS */
struct osd_offset {
	int left_x;
	int right_x;
	unsigned int key_event;
};

/* Elan add for OSD bar coordinate */
/*
static struct osd_offset OSD_mapping[] = {
  {35, 99, KEY_MENU},	//menu_left_x, menu_right_x, KEY_MENU
  {203, 267, KEY_HOME},	//home_left_x, home_right_x, KEY_HOME
  {373, 437, KEY_BACK},	//back_left_x, back_right_x, KEY_BACK
  {541, 605, KEY_SEARCH},	//search_left_x, search_right_x, KEY_SEARCH
};

static int key_pressed = -1;
*/
#endif				/* _LINUX_ELAN_KTF2K_H */
