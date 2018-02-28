/** \mainpage
*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* File : BMA4XY.h
*
* Date : 2016/05/18
*
* Revision : 1.1.0 $
*
* Usage: Sensor Driver for BMA4XY sensor
*
****************************************************************************
*
* \section Disclaimer
*
* Common:
* Bosch Sensortec products are developed for the consumer goods industry.
* They may only be used within the parameters of the respective valid
* product data sheet.  Bosch Sensortec products are provided with the
* express understanding that there is no warranty of fitness for a
* particular purpose.They are not fit for use in life-sustaining,
* safety or security sensitive systems or any system or device
* that may lead to bodily harm or property damage if the system
* or device malfunctions. In addition,Bosch Sensortec products are
* not fit for use in products which interact with motor vehicle systems.
* The resale and or use of products are at the purchasers own risk and
* his own responsibility. The examination of fitness for the intended use
* is the sole responsibility of the Purchaser.
*
* The purchaser shall indemnify Bosch Sensortec from all third party
* claims, including any claims for incidental, or consequential damages,
* arising from any product use not covered by the parameters of
* the respective valid product data sheet or not approved by
* Bosch Sensortec and reimburse Bosch Sensortec for all costs in
* connection with such claims.
*
* The purchaser must monitor the market for the purchased products,
* particularly with regard to product safety and inform Bosch Sensortec
* without delay of all security relevant incidents.
*
* Engineering Samples are marked with an asterisk (*) or (e).
* Samples may vary from the valid technical specifications of the product
* series. They are therefore not intended or fit for resale to third
* parties or for use in end products. Their sole purpose is internal
* client testing. The testing of an engineering sample may in no way
* replace the testing of a product series. Bosch Sensortec assumes
* no liability for the use of engineering samples.
* By accepting the engineering samples, the Purchaser agrees to indemnify
* Bosch Sensortec from all claims arising from the use of engineering
* samples.
*
* Special:
* This software module (hereinafter called "Software") and any information
* on application-sheets (hereinafter called "Information") is provided
* free of charge for the sole purpose to support your application work.
* The Software and Information is subject to the following
* terms and conditions:
*
* The Software is specifically designed for the exclusive use for
* Bosch Sensortec products by personnel who have special experience
* and training. Do not use this Software if you do not have the
* proper experience or training.
*
* This Software package is provided `` as is `` and without any expressed
* or implied warranties,including without limitation, the implied warranties
* of merchantability and fitness for a particular purpose.
*
* Bosch Sensortec and their representatives and agents deny any liability
* for the functional impairment
* of this Software in terms of fitness, performance and safety.
* Bosch Sensortec and their representatives and agents shall not be liable
* for any direct or indirect damages or injury, except as
* otherwise stipulated in mandatory applicable law.
*
* The Information provided is believed to be accurate and reliable.
* Bosch Sensortec assumes no responsibility for the consequences of use
* of such Information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of Bosch. Specifications mentioned in the Information are
* subject to change without notice.
**************************************************************************/
/*! \file BMA4XY.h
    \brief BMA4XY Sensor Driver Support Header File */
/* user defined code to be added here ... */
 #ifndef __BMA4XY_H__
 #define __BMA4XY_H__
#include <linux/types.h>
#include <stddef.h>
#define BMA421
#define CONFIG_STREAM_VERSION "1.5"





#define BMA4XY_MDELAY_DATA_TYPE                 u32
/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/


/*!
	@brief Define the calling convention of YOUR bus communication routine.
	@note This includes types of parameters. This example shows the
	configuration for an SPI bus link.

    If your communication function looks like this:

    write_my_bus_xy(u8 device_addr, u8 register_addr,
    u8 * data, u8 length);

    The BMA4XY_WR_FUNC_PTR would equal:

    BMA4XY_WR_FUNC_PTR s8 (* bus_write)(u8,
    u8, u8 *, u8)


    Parameters can be mixed as needed refer to the
    BMA4XY_BUS_WRITE_FUNC  macro.


*/
#define BMA4XY_WR_FUNC_PTR s8 (*bus_write)(u8, u8,\
u8 *, u8)
#define BMA4XY_BWR_FUNC_PTR s8 \
(*burst_write)(u8, u8 *, u8)
/**< link macro between API function calls and bus write function
	@note The bus write function can change since this is a
	system dependant issue.

    If the bus_write parameter calling order is like: reg_addr,
    reg_data, wr_len it would be as it is here.

    If the parameters are differently ordered or your communication
    function like I2C need to know the device address,
    you can change this macro accordingly.


    BMA4XY_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
    bus_write(dev_addr, reg_addr, reg_data, wr_len)

    This macro lets all API functions call YOUR communication routine in a
    way that equals your definition in the
    @ref BMA4XY_WR_FUNC_PTR definition.

*/
#define BMA4XY_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
				bus_write(dev_addr, reg_addr, reg_data, wr_len)
#define BMA4XY_BURST_WRITE_FUNC(reg_addr, reg_data, wr_len)\
				burst_write(reg_addr, reg_data, wr_len)
/**< Define the calling convention of YOUR bus communication routine.
	@note This includes types of parameters. This example shows the
	configuration for an SPI bus link.

    If your communication function looks like this:

    read_my_bus_xy(u8 device_addr, u8 register_addr,
    u8 * data, u8 length);

    The BMA4XY_RD_FUNC_PTR would equal:

    BMA4XY_RD_FUNC_PTR s8 (* bus_read)(u8,
    u8, u8 *, u8)

    Parameters can be mixed as needed refer to the
    refer BMA4XY_BUS_READ_FUNC  macro.

*/
#define BMA4XY_SPI_RD_MASK (0x80)   /* for spi read transactions on SPI the
			MSB has to be set */
#define BMA4XY_RD_FUNC_PTR s8 (*bus_read)(u8,\
			u8, u8 *, u16)
#define BMA4XY_BRD_FUNC_PTR s8 \
(*burst_read)(u8, u8, u8 *, u16)

/**< link macro between API function calls and bus read function
	@note The bus write function can change since this is a
	system dependant issue.

    If the bus_read parameter calling order is like: reg_addr,
    reg_data, wr_len it would be as it is here.

    If the parameters are differently ordered or your communication
    function like I2C need to know the device address,
    you can change this macro accordingly.


    BMA4XY_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
    bus_read(dev_addr, reg_addr, reg_data, wr_len)

    This macro lets all API functions call YOUR communication routine in a
    way that equals your definition in the
    refer BMA4XY_WR_FUNC_PTR definition.

    @note: this macro also includes the "MSB='1'
    for reading BMA4XY addresses.

*/
#define BMA4XY_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
				bus_read(dev_addr, reg_addr, reg_data, r_len)

#define BMA4XY_BURST_READ_FUNC(device_addr, \
register_addr, register_data, rd_len)\
burst_read(device_addr, register_addr, register_data, rd_len)

#if defined(BMA422) || defined(BMA455)
#define	BMA4XY_FEATURE_SIZE				(38)
#elif defined(BMA420)
#define	BMA4XY_FEATURE_SIZE				(30)
#define BMA4XY_SINGLE_TAP					(0X01)
#define BMA4XY_DOUBLE_TAP					(0x00)
#elif defined(BMA421)
#define	BMA4XY_FEATURE_SIZE       (24)
#endif

#define BMA4XY_SUCCESS					(0)
#define BMA4XY_FAIL							(-1)
#define BMA4XY_TITAN_INITIALIZED			(0x01)
#define BMA4XY_INVALID				(0xFFFF)

/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/
#define BMA4XY_I2C_ADDR1                   (0x68)
/**< I2C Address needs to be changed */
#define BMA4XY_I2C_ADDR2                    (0x69)
 /**< I2C Address needs to be changed */
#define BMA4XY_AUX_BMM150_I2C_ADDRESS       (0x10)
/**< I2C address of AKM09912*/
#define BMA4XY_AUX_AKM09916_I2C_ADDR			(0x0C)
/**< I2C address of AKM09916*/
/*******************************************/
/**\name	CONSTANTS        */
/******************************************/
#define  BMA4XY_INIT_VALUE			(0)
#define  BMA4XY_ASSIGN_DATA			(1)
#define  BMA4XY_READ_LENGTH			(1)
#define  BMA4XY_WRITE_LENGTH			(1)
#define  BMA4XY_MAXIMUM_TIMEOUT			(10)
#define  BMA4XY_GEN_READ_WRITE_DATA_LENGTH	(1)
#define  BMA4XY_FIFO_WM_LENGTH			(2)
#define	 BMA4XY_FIRMWARE_IMAGE_SIZE		(6144)
#define	 BMA4XY_FIRMWARE_BLOCK_SIZE		(32)

#define BMA4XY_NON_LATCH_MODE					(0)
#define BMA4XY_LATCH_MODE					(1)

#define BMA4XY_OPEN_DRAIN				(1)
#define BMA4XY_PUSH_PULL				(0)
#define BMA4XY_ACTIVE_HIGH				(1)
#define BMA4XY_ACTIVE_LOW				(0)
#define BMA4XY_EDGE_TRIGGER				(1)
#define BMA4XY_LEVEL_TRIGGER			(0)
#define BMA4XY_OUTPUT_ENABLE			(1)
#define BMA4XY_OUTPUT_DISABLE			(0)
#define BMA4XY_INPUT_ENABLE				(1)
#define BMA4XY_INPUT_DISABLE			(0)

#define  BMA4XY_ADV_POWER_SAVE_DISABLE	(0)
#define	 BMA4XY_UC_DISABLE			(0)
#define	 BMA4XY_UC_FIFO_ENABLE				(11)
/* output data rate condition check*/
#define  BMA4XY_MAG_MIN_OUTPUT_DATA_RATE	(1)
#define  BMA4XY_MAG_MAX_OUTPUT_DATA_RATE	(11)
#define  BMA4XY_OUTPUT_DATA_RATE1	(1)
#define  BMA4XY_OUTPUT_DATA_RATE2	(2)
#define  BMA4XY_OUTPUT_DATA_RATE3	(3)
#define  BMA4XY_OUTPUT_DATA_RATE4	(4)
#define  BMA4XY_OUTPUT_DATA_RATE5	(5)
#define  BMA4XY_OUTPUT_DATA_RATE6	(14)
#define  BMA4XY_OUTPUT_DATA_RATE7	(15)

/* accel range check*/
#define BMA4XY_ACCEL_RANGE_2G  (0)
#define BMA4XY_ACCEL_RANGE_4G  (1)
#define BMA4XY_ACCEL_RANGE_8G  (2)
#define BMA4XY_ACCEL_RANGE_16G (3)

/* check the status of registers*/
#define  BMA4XY_FOC_STAT_HIGH			(1)
#define  BMA4XY_SIG_MOTION_STAT_HIGH	(1)
#define  BMA4XY_STEP_DET_STAT_HIGH		(1)

/*condition check for reading and writing data*/
#define	BMA4XY_MAX_VALUE_SIGNIFICANT_MOTION      (1)
#define	BMA4XY_MAX_VALUE_FIFO_FILTER    (1)
#define	BMA4XY_MAX_VALUE_FIFO_TIME      (1)
#define	BMA4XY_MAX_VALUE_FIFO_INTR      (1)
#define	BMA4XY_MAX_VALUE_FIFO_HEADER    (1)
#define	BMA4XY_MAX_VALUE_FIFO_MAG       (1)
#define	BMA4XY_MAX_VALUE_FIFO_ACCEL     (1)
#define	BMA4XY_MAX_VALUE_SOURCE_INTR    (1)
#define	BMA4XY_MAX_VALUE_LOW_G_MODE     (1)
#define	BMA4XY_MAX_VALUE_NO_MOTION      (1)
#define	BMA4XY_MAX_VALUE_TAP_SHOCK      (1)
#define	BMA4XY_MAX_VALUE_TAP_QUIET      (1)
#define	BMA4XY_MAX_VALUE_ORIENT_UD      (1)
#define	BMA4XY_MAX_VALUE_ORIENT_AXES    (1)
#define	BMA4XY_MAX_VALUE_NVM_PROG       (1)
#define	BMA4XY_MAX_VALUE_SPI3           (1)
#define	BMA4XY_MAX_VALUE_PAGE           (1)
#define	BMA4XY_MAX_VALUE_I2C_WDT        (1)
#define	BMA4XY_MAX_VALUE_SLEEP_STATE    (1)
#define	BMA4XY_MAX_VALUE_WAKEUP_INTR    (1)
#define	BMA4XY_MAX_VALUE_SELFTEST_SIGN  (1)
#define	BMA4XY_MAX_VALUE_SELFTEST_AMP   (1)
#define	BMA4XY_MAX_VALUE_SELFTEST_START (1)

#define BMA4XY_MAX_ACCEL_SELFTEST_AXIS	    (3)
#define BMA4XY_MAX_ACCEL_BW                 (7)
#define BMA4XY_MAX_IF_MODE                  (3)
#define BMA4XY_MAX_TARGET_PAGE              (3)
#define BMA4XY_MAX_UNDER_SAMPLING           (1)
#define BMA4XY_MAX_ACCEL_OUTPUT_DATA_RATE   (12)

/* FIFO index definitions*/
#define BMA4XY_FIFO_X_LSB_DATA			(0)
#define BMA4XY_FIFO_X_MSB_DATA			(1)
#define BMA4XY_FIFO_Y_LSB_DATA			(2)
#define BMA4XY_FIFO_Y_MSB_DATA			(3)
#define BMA4XY_FIFO_Z_LSB_DATA			(4)
#define BMA4XY_FIFO_Z_MSB_DATA			(5)
#define BMA4XY_FIFO_R_LSB_DATA			(6)
#define BMA4XY_FIFO_R_MSB_DATA			(7)

/* FIFO mag/gyro/accel definition*/
#define BMA4XY_MGA_FIFO_M_X_LSB		(0)
#define BMA4XY_MGA_FIFO_M_X_MSB		(1)
#define BMA4XY_MGA_FIFO_M_Y_LSB		(2)
#define BMA4XY_MGA_FIFO_M_Y_MSB		(3)
#define BMA4XY_MGA_FIFO_M_Z_LSB		(4)
#define BMA4XY_MGA_FIFO_M_Z_MSB		(5)
#define BMA4XY_MGA_FIFO_M_R_LSB		(6)
#define BMA4XY_MGA_FIFO_M_R_MSB		(7)
#define BMA4XY_MGA_FIFO_G_X_LSB		(8)
#define BMA4XY_MGA_FIFO_G_X_MSB		(9)
#define BMA4XY_MGA_FIFO_G_Y_LSB		(10)
#define BMA4XY_MGA_FIFO_G_Y_MSB		(11)
#define BMA4XY_MGA_FIFO_G_Z_LSB		(12)
#define BMA4XY_MGA_FIFO_G_Z_MSB		(13)
#define BMA4XY_MGA_FIFO_A_X_LSB		(14)
#define BMA4XY_MGA_FIFO_A_X_MSB		(15)
#define BMA4XY_MGA_FIFO_A_Y_LSB		(16)
#define BMA4XY_MGA_FIFO_A_Y_MSB		(17)
#define BMA4XY_MGA_FIFO_A_Z_LSB		(18)
#define BMA4XY_MGA_FIFO_A_Z_MSB		(19)
/* FIFO mag definition*/
#define BMA4XY_MA_FIFO_M_X_LSB		(0)
#define BMA4XY_MA_FIFO_M_X_MSB		(1)
#define BMA4XY_MA_FIFO_M_Y_LSB		(2)
#define BMA4XY_MA_FIFO_M_Y_MSB		(3)
#define BMA4XY_MA_FIFO_M_Z_LSB		(4)
#define BMA4XY_MA_FIFO_M_Z_MSB		(5)
#define BMA4XY_MA_FIFO_M_R_LSB		(6)
#define BMA4XY_MA_FIFO_M_R_MSB		(7)
#define BMA4XY_MA_FIFO_A_X_LSB		(8)
#define BMA4XY_MA_FIFO_A_X_MSB		(9)
#define BMA4XY_MA_FIFO_A_Y_LSB		(10)
#define BMA4XY_MA_FIFO_A_Y_MSB		(11)
#define BMA4XY_MA_FIFO_A_Z_LSB		(12)
#define BMA4XY_MA_FIFO_A_Z_MSB		(13)

/* FIFO length definitions*/
#define BMA4XY_FIFO_SENSOR_TIME_LSB     (0)
#define BMA4XY_FIFO_SENSOR_TIME_XLSB    (1)
#define BMA4XY_FIFO_SENSOR_TIME_MSB     (2)
#define BMA4XY_FIFO_SENSOR_TIME_LENGTH  (3)
#define BMA4XY_FIFO_A_LENGTH            (6)
#define BMA4XY_FIFO_G_LENGTH            (6)
#define BMA4XY_FIFO_M_LENGTH            (8)
#define BMA4XY_FIFO_AG_LENGTH           (12)
#define BMA4XY_FIFO_AMG_LENGTH          (20)
#define BMA4XY_FIFO_MA_OR_MG_LENGTH     (14)

/* bus read and write length for mag and accel */
#define BMA4XY_MAG_X_DATA_LENGTH     (2)
#define BMA4XY_MAG_Y_DATA_LENGTH     (2)
#define BMA4XY_MAG_Z_DATA_LENGTH     (2)
#define BMA4XY_MAG_R_DATA_LENGTH     (2)
#define BMA4XY_MAG_XYZ_DATA_LENGTH	 (6)
#define BMA4XY_MAG_XYZR_DATA_LENGTH	 (8)
#define BMA4XY_ACCEL_DATA_LENGTH	 (6)

#define BMA4XY_TEMP_DATA_LENGTH		 (2)
#define BMA4XY_FIFO_DATA_LENGTH		 (2)
#define BMA4XY_STEP_COUNTER_LENGTH	 (2)
#define BMA4XY_SENSOR_TIME_LENGTH	 (3)

/* Delay definitions*/
#define BMA4XY_AUX_IF_DELAY				(5)
#define BMA4XY_BMM150_WAKEUP_DELAY1			(2)
#define BMA4XY_BMM150_WAKEUP_DELAY2			(3)
#define BMA4XY_BMM150_WAKEUP_DELAY3			(1)
#define BMA4XY_GEN_READ_WRITE_DELAY			(1)
#define BMA4XY_AKM_INIT_DELAY				(100)
/****************************************************/
/**\name	ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define	BMA4XY_ACCEL_X_DATA_SIZE   (2)
#define	BMA4XY_ACCEL_Y_DATA_SIZE   (2)
#define	BMA4XY_ACCEL_Z_DATA_SIZE   (2)
#define	BMA4XY_ACCEL_XYZ_DATA_SIZE (6)

#define	BMA4XY_MAG_X_DATA_SIZE      (2)
#define	BMA4XY_MAG_Y_DATA_SIZE      (2)
#define	BMA4XY_MAG_Z_DATA_SIZE      (2)
#define	BMA4XY_MAG_R_DATA_SIZE      (2)
#define	BMA4XY_MAG_XYZ_DATA_SIZE    (6)
#define	BMA4XY_MAG_XYZR_DATA_SIZE   (8)
#define	BMA4XY_MAG_TRIM_DATA_SIZE   (16)


#define	BMA4XY_TEMP_DATA_SIZE       (2)
#define	BMA4XY_FIFO_DATA_SIZE       (2)
#define	BMA4XY_STEP_COUNT_DATA_SIZE (2)
#define BMA4XY_INTERRUPT_DATA_SIZE	(2)

#define	BMA4XY_SENSOR_TIME_DATA_SIZE      (3)

/****************************************************/
/**\name	ARRAY PARAMETER DEFINITIONS      */
/***************************************************/
#define BMA4XY_SENSOR_TIME_MSB_BYTE   (2)
#define BMA4XY_SENSOR_TIME_XLSB_BYTE  (1)
#define BMA4XY_SENSOR_TIME_LSB_BYTE   (0)


#define BMA4XY_MAG_X_LSB_BYTE   (0)
#define BMA4XY_MAG_X_MSB_BYTE   (1)
#define BMA4XY_MAG_Y_LSB_BYTE   (2)
#define BMA4XY_MAG_Y_MSB_BYTE   (3)
#define BMA4XY_MAG_Z_LSB_BYTE   (4)
#define BMA4XY_MAG_Z_MSB_BYTE   (5)
#define BMA4XY_MAG_R_LSB_BYTE   (6)
#define BMA4XY_MAG_R_MSB_BYTE   (7)

#define BMA4XY_DATA_FRAME_ACCEL_X_LSB_BYTE   (0)
#define BMA4XY_DATA_FRAME_ACCEL_X_MSB_BYTE   (1)
#define BMA4XY_DATA_FRAME_ACCEL_Y_LSB_BYTE   (2)
#define BMA4XY_DATA_FRAME_ACCEL_Y_MSB_BYTE   (3)
#define BMA4XY_DATA_FRAME_ACCEL_Z_LSB_BYTE   (4)
#define BMA4XY_DATA_FRAME_ACCEL_Z_MSB_BYTE   (5)

#define	BMA4XY_TEMP_LSB_BYTE    (0)
#define	BMA4XY_TEMP_MSB_BYTE    (1)

#define	BMA4XY_FIFO_LENGTH_LSB_BYTE    (0)
#define	BMA4XY_FIFO_LENGTH_MSB_BYTE    (1)

#define	BMA4XY_STEP_COUNT_LSB_BYTE    (0)
#define	BMA4XY_STEP_COUNT_MSB_BYTE    (1)
/****************************************************/
/**\name	ERROR CODES       */
/***************************************************/

#define E_BMA4XY_NULL_PTR			((s8)-127)
#define E_BMA4XY_COMM_RES			((s8)-1)
#define E_BMA4XY_OUT_OF_RANGE		((s8)-2)
#define E_BMA4XY_BUSY				((s8)-3)
#define E_BMA4XY_COM_RES_OK			((u8)0)
#define	SUCCESS						((u8)0)
#define	ERROR						((s8)-1)

/* Constants */
#define BMA4XY_NULL						(0)
#define BMA4XY_DELAY_SETTLING_TIME		(5)
/*This refers BMA4XY return type as s8 */
#define BMA4XY_RETURN_TYPE		s8


#define BMA4XY_SET_LOW_BYTE      (0x00FF)
#define BMA4XY_SET_HIGH_BYTE		 (0xFF00)
#define BMA4XY_SET_LOW_NIBBLE      (0x0F)
#define BMA4XY_SET_HIGH_NIBBLE		 (0xF0)
/****************************************************/
/**\name	REGISTER DEFINITIONS       */
/***************************************************/
/*******************/
/**\name CHIP ID */
/*******************/
#define BMA4XY_CHIP_ID_ADDR				(0x00)
#define BMA4XY_CONFIG_STREAM_OFFSET			(0x00)

/*******************/
/**\name REV ID */
/*******************/
#define BMA4XY_REV_ID_ADDR				(0x01)
/*******************/
/**\name ERROR STATUS */
/*******************/
#define BMA4XY_ERROR_ADDR					(0X02)
/*******************/
/**\name POWER MODE STATUS */
/*******************/
#define BMA4XY_STATUS_ADDR				(0X1B)
/* below register is in page1*/
#define BMA4XY_PWR_STATUS_ADDR		(0x7D)
/*******************/
/**\name MAG DATA REGISTERS */
/*******************/
#define BMA4XY_DATA_0_ADDR					(0X0A)
#define BMA4XY_DATA_1_ADDR					(0X05)
#define BMA4XY_DATA_2_ADDR					(0X06)
#define BMA4XY_DATA_3_ADDR					(0X07)
#define BMA4XY_DATA_4_ADDR					(0X08)
#define BMA4XY_DATA_5_ADDR					(0X09)
#define BMA4XY_DATA_6_ADDR					(0X0A)
#define BMA4XY_DATA_7_ADDR					(0X0B)
#define BMA4XY_ACCEL_ADDR				(0X12)
/*******************/
/**\name ACCEL DATA REGISTERS */
/*******************/
#define BMA4XY_DATA_16_ADDR				(0X14)
#define BMA4XY_DATA_17_ADDR				(0X15)
#define BMA4XY_DATA_18_ADDR				(0X16)
#define BMA4XY_DATA_19_ADDR				(0X17)
/*******************/
/**\name SENSOR TIME REGISTERS */
/*******************/
#define BMA4XY_SENSORTIME_0_ADDR			(0X18)
#define BMA4XY_SENSORTIME_1_ADDR			(0X19)
#define BMA4XY_SENSORTIME_2_ADDR			(0X1A)
/*******************/
/**\name STATUS REGISTER FOR SENSOR STATUS FLAG */
/*******************/
#define BMA4XY_STAT_ADDR					(0X03)
/*******************/
/**\name INTERRUPY STATUS REGISTERS */
/*******************/
#define BMA4XY_INTR_STAT_0_ADDR			(0X1C)
#define BMA4XY_INTR_STAT_1_ADDR			(0X1D)
#define BMA4XY_INTR_STAT_2_ADDR			(0X1E)
#define BMA4XY_INTR_STAT_3_ADDR			(0X1F)
/*******************/
/**\name TEMPERATURE REGISTERS */
/*******************/
#define BMA4XY_TEMPERATURE_0_ADDR			(0X22)
#define BMA4XY_TEMPERATURE_1_ADDR			(0X23)
/*******************/
/**\name FIFO REGISTERS */
/*******************/
#define BMA4XY_FIFO_LENGTH_0_ADDR			(0X24)
#define BMA4XY_FIFO_LENGTH_1_ADDR			(0X25)
#define BMA4XY_FIFO_DATA_ADDR				(0X26)
/***************************************************/
/**\name ACCEL CONFIG REGISTERS  FOR ODR, BANDWIDTH AND UNDERSAMPLING*/
/******************************************************/
#define BMA4XY_ACCEL_CONFIG_ADDR			(0X40)
/*******************/
/**\name ACCEL RANGE */
/*******************/
#define BMA4XY_ACCEL_RANGE_ADDR            (0X41)

/***************************************************/
/**\name MAG CONFIG REGISTERS  FOR ODR*/
/******************************************************/
#define BMA4XY_MAG_CONFIG_ADDR				(0X44)
/***************************************************/
/**\name REGISTER FOR ACCEL DOWNSAMPLING RATES FOR FIFO*/
/******************************************************/
#define BMA4XY_FIFO_DOWN_ADDR              (0X45)
/***************************************************/
/**\name FIFO CONFIG REGISTERS*/
/******************************************************/
#define BMA4XY_FIFO_CONFIG_0_ADDR          (0X48)
#define BMA4XY_FIFO_CONFIG_1_ADDR          (0X49)
/***************************************************/
#define BMA4XY_FIFO_WTM_0_ADDR          (0X46)
#define BMA4XY_FIFO_WTM_1_ADDR          (0X47)
/**\name MAG INTERFACE REGISTERS*/
/******************************************************/
#define BMA4XY_MAG_IF_0_ADDR				(0X4B)
#define BMA4XY_MAG_IF_1_ADDR				(0X4C)
#define BMA4XY_MAG_IF_2_ADDR				(0X4D)
#define BMA4XY_MAG_IF_3_ADDR				(0X4E)
#define BMA4XY_MAG_IF_4_ADDR				(0X4F)
/***************************************************/
/**\name INTERRUPT ENABLE REGISTERS*/
/******************************************************/
#define BMA4XY_INTR_ENABLE_0_ADDR			(0X50)
#define BMA4XY_INTR_ENABLE_1_ADDR			(0X51)
#define BMA4XY_INTR_ENABLE_2_ADDR			(0X52)
#define BMA4XY_INTR1_OUT_CTRL_ADDR			(0X53)
#define BMA4XY_INTR2_OUT_CTRL_ADDR			(0X54)
/***************************************************/
/**\name LATCH DURATION REGISTERS*/
/******************************************************/
#define BMA4XY_INTR_LATCH_ADDR				(0X55)
/***************************************************/
/**\name MAP INTERRUPT 1 and 2 REGISTERS*/
/******************************************************/
#define BMA4XY_INTR_MAP_1_ADDR				(0X56)
#define BMA4XY_INTR_MAP_2_ADDR				(0X57)
#define BMA4XY_INTR_HW_ADDR				(0x58)
#define BMA4XY_UC_CONF_ADDR				(0x59)
/***************************************************/
/**\name DATA SOURCE REGISTERS*/
/******************************************************/
#define BMA4XY_UC_CONF			(0X59)
#define BMA4XY_INTR_DATA_1_ADDR			(0X59)
#define BMA4XY_UC_DMA_NEXT_LAST			(0x5B)
#define BMA4XY_UC_DMA_DATA_NEXT			(0x5C)
#define BMA4XY_UC_DMA_DATA				(0x5E)
#define BMA4XY_UC_STATUS				(0x5F)
#define BMA4XY_ACCEL_CONFIG_FOC			(0x37)
#define BMA4XY_RESOLUTION				(0x03)
#define BMA4XY_RANGE					(0x04)
#define BMA4XY_12_BIT_RESOLUTION		(0x00)
/*
 #define	BMA4XY_14_BIT_RESOLUTION	(0x01)
 #define BMA4XY_16_BIT_RESOLUTION		(0x02)
*/
/***************************************************/
/**\name FAST OFFSET CONFIGURATION REGISTER*/
/******************************************************/
#define BMA4XY_FOC_CONFIG_ADDR				(0X69)
/***************************************************/
/**\name MISCELLANEOUS CONFIGURATION REGISTER*/
/******************************************************/
#define BMA4XY_CONFIG_ADDR					(0X6A)
/***************************************************/
/**\name SERIAL INTERFACE SETTINGS REGISTER*/
/******************************************************/
#define BMA4XY_IF_CONFIG_ADDR				(0X6B)

/***************************************************/
/**\name SELF_TEST REGISTER*/
/******************************************************/
#define BMA4XY_SELF_TEST_ADDR				(0X6D)
/***************************************************/
/**\name SPI,I2C SELECTION REGISTER*/
/******************************************************/
#define BMA4XY_NV_CONFIG_ADDR				(0x70)
/***************************************************/
/**\name ACCEL OFFSET REGISTERS*/
/******************************************************/
#define BMA4XY_OFFSET_0_ADDR				(0X71)
#define BMA4XY_OFFSET_1_ADDR				(0X72)
#define BMA4XY_OFFSET_2_ADDR				(0X73)
#define BMA4XY_OFFSET_3_ADDR				(0X74)
#define BMA4XY_OFFSET_4_ADDR				(0X75)
#define BMA4XY_OFFSET_5_ADDR				(0X76)
#define BMA4XY_OFFSET_6_ADDR				(0X77)
/***************************************************/
/**\name STEP COUNTER INTERRUPT REGISTERS*/
/******************************************************/
#define BMA4XY_STEP_COUNT_0_ADDR			(0X78)
#define BMA4XY_STEP_COUNT_1_ADDR			(0X79)
/***************************************************/
/**\name STEP COUNTER CONFIGURATION REGISTERS*/
/******************************************************/
#define BMA4XY_STEP_CONFIG_0_ADDR			(0X7A)
#define BMA4XY_STEP_CONFIG_1_ADDR			(0X7B)
/***************************************************/
/**\name POWER_CTRL REGISTER*/
#define BMA4XY_POWER_CONF_ADDR					(0x7C)

#define BMA4XY_POWER_CTRL_ADDR					(0x7D)
/**\name COMMAND REGISTER*/
/******************************************************/
#define BMA4XY_CMD_COMMANDS_ADDR				(0X7E)
/***************************************************/
/**\name PAGE REGISTERS*/
/******************************************************/
#define BMA4XY_CMD_EXT_MODE_ADDR				(0X7F)
#define BMA4XY_COM_C_TRIM_REG_0_ADDR				(0X0B)

/****************************************************/
/**\name	SHIFT VALUE DEFINITION       */
/***************************************************/
#define BMA4XY_SHIFT_BIT_POSITION_BY_01_BIT      (1)
#define BMA4XY_SHIFT_BIT_POSITION_BY_02_BITS     (2)
#define BMA4XY_SHIFT_BIT_POSITION_BY_03_BITS     (3)
#define BMA4XY_SHIFT_BIT_POSITION_BY_04_BITS     (4)
#define BMA4XY_SHIFT_BIT_POSITION_BY_05_BITS     (5)
#define BMA4XY_SHIFT_BIT_POSITION_BY_06_BITS     (6)
#define BMA4XY_SHIFT_BIT_POSITION_BY_07_BITS     (7)
#define BMA4XY_SHIFT_BIT_POSITION_BY_08_BITS     (8)
#define BMA4XY_MSB_SHIFT_ADJUST					 (8)
#define BMA4XY_SHIFT_BIT_POSITION_BY_09_BITS     (9)
#define BMA4XY_SHIFT_BIT_POSITION_BY_12_BITS     (12)
#define BMA4XY_SHIFT_BIT_POSITION_BY_13_BITS     (13)
#define BMA4XY_SHIFT_BIT_POSITION_BY_14_BITS     (14)
#define BMA4XY_SHIFT_BIT_POSITION_BY_15_BITS     (15)
#define BMA4XY_SHIFT_BIT_POSITION_BY_16_BITS     (16)

#ifndef ABS
#define ABS(a)		((a) > 0 ? (a) : -(a)) /*!< Absolute value */
#endif


/****************************************************/
/**\name	AKM09911 AND AKM09912 DEFINITION */
/***************************************************/
#define AKM09912_SENSITIVITY_DIV	(256)
#define AKM09912_SENSITIVITY		(128)
#define AKM09911_SENSITIVITY_DIV	(128)
#define AKM_ASAX	(0)
#define AKM_ASAY	(1)
#define AKM_ASAZ	(2)
#define AKM_POWER_DOWN_MODE_DATA		(0x00)
#define AKM_FUSE_ROM_MODE				(0x1F)
#define AKM_POWER_MODE_REG				(0x31)
#define	AKM_SINGLE_MEASUREMENT_MODE		(0x01)
#define AKM_CONTINUOUS_MEASUREMENT_MODE1 (0x02)
#define AKM_DATA_REGISTER				(0x11)
/*! AKM09912 Register definition */
#define AKM_CHIP_ID_REG			(0x01)

/****************************************************/
/**\name	AKM09916 DEFINITION */
/***************************************************/
#define AKM_POWER_DOWN_MODE_DATA		(0x00)
#define AKM_POWER_MODE_REG				(0x31)
#define	AKM_SINGLE_MEASUREMENT_MODE		(0x01)
#define AKM_DATA_REGISTER				(0x11)
/*! AKM09912 Register definition */
#define AKM_CHIP_ID_REG			(0x01)
/****************************************************/
/**\name	BMM150 DEFINITION */
/***************************************************/
#define BMA4XY_BMM150_SET_POWER_CONTROL	(0x01)
#define BMA4XY_BMM150_MAX_RETRY_WAKEUP	(5)
#define BMA4XY_BMM150_POWER_ON			(0x01)
#define BMA4XY_BMM150_POWER_OFF			(0x00)
#define BMA4XY_BMM150_FORCE_MODE		(0x02)
#define BMA4XY_BMM150_POWER_ON_SUCCESS	(0)
#define BMA4XY_BMM150_POWER_ON_FAIL		((s8)-1)

#define	BMA4XY_BMM150_DIG_X1			(0)
#define	BMA4XY_BMM150_DIG_Y1			(1)
#define	BMA4XY_BMM150_DIG_X2			(2)
#define	BMA4XY_BMM150_DIG_Y3			(3)
#define	BMA4XY_BMM150_DIG_XY1			(4)
#define	BMA4XY_BMM150_DIG_XY2			(5)
#define	BMA4XY_BMM150_DIG_Z1_LSB		(6)
#define	BMA4XY_BMM150_DIG_Z1_MSB		(7)
#define	BMA4XY_BMM150_DIG_Z2_LSB		(8)
#define	BMA4XY_BMM150_DIG_Z2_MSB		(9)
#define	BMA4XY_BMM150_DIG_Z3_LSB	(10)
#define	BMA4XY_BMM150_DIG_Z3_MSB	(11)
#define	BMA4XY_BMM150_DIG_Z4_LSB	(12)
#define	BMA4XY_BMM150_DIG_Z4_MSB	(13)
#define	BMA4XY_BMM150_DIG_XYZ1_LSB	(14)
#define	BMA4XY_BMM150_DIG_XYZ1_MSB	(15)

#define BMA4XY_FIFO_FRAME_CNT			(146)
#define	BMA4XY_FRAME_COUNT				(1)

/* #define SPI */
#ifdef SPI_ENABLE
#define READ_EXTRA_BYTE			(1)
#else
#define READ_EXTRA_BYTE			(0)
#endif
/**************************************************************/
/**\name	STRUCTURE DEFINITIONS                         */
/**************************************************************/
/*!
*	@brief
*	This structure holds all relevant information about BMA4XY
*/
struct bma4xy_t {
u8 chip_id;/**< chip id of BMA4XY */
u8 rev_id;/**< rev id of BMA4XY */
u8 dev_addr;/**< device address of BMA4XY */
s8 mag_manual_enable;/**< used for check the mag manual/auto mode status */
u8 crc_check;/**< used to store the result of crc check status */
BMA4XY_WR_FUNC_PTR;/**< bus write function pointer */
BMA4XY_RD_FUNC_PTR;/**< bus read function pointer */
BMA4XY_BRD_FUNC_PTR;/**< burst write function pointer */
s8 (*burst_write)(u8, u8, u8 *, u8);/**< burst write function pointer */
void (*delay_msec)(BMA4XY_MDELAY_DATA_TYPE);/**< delay function pointer */
};

/*!
*	@brief Error Status structure */
struct bma4xy_err_reg_t {
	u8 fatal_err;/**<indicates fatal error*/
	u8 cmd_err;/**<indicates command error*/
	u8 err_code;/**<indicates error code*/
	u8 fifo_err;/**<indicates fifo error*/
	u8 aux_err;/**<indicates mag error*/

};
/*!
* @brief Status structure */
struct bma4xy_status_t {
	u8 aux_man_op;/**<indicates  manual Auxiliary
	sensor interface operation is ongoing*/
	u8 cmd_rdy;/**<indicates Command decoder is
	ready to accept a new command*/
	u8 drdy_aux;/**<Data ready for Auxiliary sensor*/
	u8 drdy_acc;/**<Data ready for Accelerometer*/

};

/*!
* @brief UC Status structure */
struct bma4xy_uc_status {
	u8 sleep;/**<uc is in sleep/halt state*/
	u8 irq_ovm;/**<Dedicated interrupt is set again
	before previous interrupt was acknowledged*/
	u8 wc_event;/**<Watchcell event detected
	(uC stopped)*/
	u8 dma_active;/**<DMA controller has started
	DMA and DMA transactions are ongoing*/
};

#if defined(BMA422) || defined(BMA455)
/*!
* @brief Interrupt Status structure */
struct bma4xy_int_status {
	u8 sig_motion;/**<UC sig_motion interrupt*/
	u8 step_counter;/**<UC step_counter interrupt*/
	u8 tilt;/**<UC tilt interrupt*/
	u8 pickup;/**<UC pickup interrupt*/
	u8 glance;/**<UC glance interrupt*/
	u8 wakeup;/**<UC wakeup interrupt*/
	u8 anymotion;/**<UC anymotion interrupt*/
	u8 error;/**<UC error interrupt*/
};
#endif
#ifdef BMA421
/*!
* @brief Interrupt Status structure */
struct bma4xy_int_status {

	u8 reserved_0;/**<Reserved for future*/
	u8 step_counter;/**<UC step_counter interrupt*/
	u8 reserved_2;/**<Reserved for future*/
	u8 reserved_3;/**<Reserved for future*/
	u8 reserved_4;/**<Reserved for future*/
	u8 reserved_5;/**<Reserved for future*/
	u8 anymotion;/**<UC anymotion interrupt*/
	u8 error;/**<error interrupt*/

};
#endif
#ifdef BMA420
/*!
* @brief Interrupt Status structure */
struct bma4xy_int_status {
	u8 anymotion;/**<UC anymotion interrupt*/
	u8 reserved_1;/**<Reserved for future*/
	u8 orientation;/**<UC orientation interrupt*/
	u8 flat;/**<UC orientation interrupt*/
	u8 tap;/**<UC flat interrupt*/
	u8 high_g;/**<high_g interrupt*/
	u8 low_g;/**<low_g interrupt*/
	u8 error;/**<error interrupt*/
};
#endif

/*!
* @brief Accelerometer Configuration structure */
struct bma4xy_acc_conf {
	u8 acc_odr;/**<output data rate*/
	u8 acc_bwp;/**<bandwidth*/
	u8 acc_perf_mode;/**<perf mode*/
	u8 acc_range;/**<Range*/

};

/*!
* @brief UC Config structure */
struct bma4xy_uc_config {
	u8 uc_en;/**<Enable/Disable UC Wake Up*/
	u8 fifo_mode_en;/**<Configure DMA/FIFO mode*/
	u8 mem_conf_ram1;/**<Mapping of instance RAM1*/
	u8 mem_conf_ram2;/**<Mapping of instance RAM2*/
	u8 mem_conf_ram3;/**<Mapping of instance RAM3*/

};

/*!
 * @brief bmm150 or akm09911
 *	magnetometer values structure
 */
struct bma4xy_mag_t {
s16 x;/**< BMM150 and AKM09911 and AKM09912 X raw data*/
s16 y;/**< BMM150 and AKM09911 and AKM09912 Y raw data*/
s16 z;/**< BMM150 and AKM09911 and AKM09912 Z raw data*/
};
/*!
 * @brief bmm150 xyz data structure
 */
struct bma4xy_mag_xyzr_t {
s16 x;/**< BMM150 X raw data*/
s16 y;/**< BMM150 Y raw data*/
s16 z;/**<BMM150 Z raw data*/
u16 r;/**<BMM150 R raw data*/
};

/*!
 * @brief accel xyz data structure
 */
struct bma4xy_accel_t {
s16 x;/**<accel X  data*/
s16 y;/**<accel Y  data*/
s16 z;/**<accel Z  data*/
};


/*!
 * @brief accel self test diff
 *	xyz data structure
 */
struct bma4xy_selftest_accel_t {
s32 x;/**<accel X  data*/
s32 y;/**<accel Y  data*/
s32 z;/**<accel Z  data*/
};

/*!
 * @brief accel difference value
 *	of axis.
 *
 */
struct diff {
		s16 val;/**<difference value*/
		u8 is_negative;/**<indicates negative value if set*/
};

/*!
 * @brief accel data deviation
 * from ideal value
 */
struct accel_data_diff {
	struct diff x;/**<accel x axis*/
	struct diff y;/**<accel y axis*/
	struct diff z;/**<accel z axis*/
};

/*!
 * @brief accel data ideal
 * values
 */
struct ideal_value_foc {

	u16 x;/**<accel x axis*/
	u16 y;/**<accel y axis*/
	u16 z;/**<accel z axis*/
};

/*!
 * @brief fifo config
 * param
 */
enum bma4xy_fifo_setup {
	BMA4XY_ACCEL = 1,/**<config for accel*/
	BMA4XY_MAG = 2,/**<config for mag*/
	BMA4XY_ALL = 3,/**<config for all*/
};

/*!
 * @brief axes remapping
 * for the sensor
 */
struct bma4xy_axes_remap {
	u8 map_x_axis;/**<x axis mapping*/
	u8 map_x_axis_sign;/**<x axis sign mapping*/
	u8 map_y_axis;/**<y axis mapping*/
	u8 map_y_axis_sign;/**<y axis sign mapping*/
	u8 map_z_axis;/**<z axis mapping*/
	u8 map_z_axis_sign;/**<z axis sign mapping*/

};

/*!
 * @brief for enabling
 * x,y,z or all axis
 */
enum bma4xy_axis_enable {

	BMA4XY_X_AXIS_ENABLE = 1,
	BMA4XY_Y_AXIS_ENABLE = 2,
	BMA4XY_Z_AXIS_ENABLE = 4,
	BMA4XY_ENABLE_ALL_AXIS = 7,
	BMA4XY_DISABLE_ALL_AXIS = 0,
};

/*!
 * @brief step counter settings
 * for android and stepcounter config
 * stream
 */
enum bma4xy_stepcounter_settings {

	BMA4XY_STEPCOUNTER_SETTING_2 = 2,
	BMA4XY_STEPCOUNTER_SETTING_3 = 3,
	BMA4XY_STEPCOUNTER_SETTING_4 = 4,
	BMA4XY_STEPCOUNTER_SETTING_5 = 5,
	BMA4XY_STEPCOUNTER_SETTING_6 = 6,
	BMA4XY_STEPCOUNTER_SETTING_7 = 7,
	BMA4XY_STEPCOUNTER_SETTING_8 = 8,
};
/*!
 * @brief firmware info.
 * of the image loaded
 */
struct bma4xy_config_stream_header {
	u8 major_version;/**<major version of config stream*/
	u8 minor_version;/**<minor version of config stream*/
	u8 image_type;/**<sample type of config stream*/
};
/*!
 * @brief accel offset xyz structure
 */
struct bma4xy_accel_offset_t {
u8 x;/**<accel offset X  data*/
u8 y;/**<accel offset Y  data*/
u8 z;/**<accel offset Z  data*/
};
/*!
 * @brief bmm150 mag compensated data structure
 */
struct bma4xy_mag_xyz_s32_t {
s16 x;/**<BMM150 X compensated data*/
s16 y;/**<BMM150 Y compensated data*/
s16 z;/**<BMM150 Z compensated data*/
};
/*!
 * @brief bmm150 mag trim data structure
 */
struct trim_data_t {
s8 dig_x1;/**<BMM150 trim x1 data*/
s8 dig_y1;/**<BMM150 trim y1 data*/

s8 dig_x2;/**<BMM150 trim x2 data*/
s8 dig_y2;/**<BMM150 trim y2 data*/

u16 dig_z1;/**<BMM150 trim z1 data*/
s16 dig_z2;/**<BMM150 trim z2 data*/
s16 dig_z3;/**<BMM150 trim z3 data*/
s16 dig_z4;/**<BMM150 trim z4 data*/

u8 dig_xy1;/**<BMM150 trim xy1 data*/
s8 dig_xy2;/**<BMM150 trim xy2 data*/

u16 dig_xyz1;/**<BMM150 trim xyz1 data*/
};
/*!
 *	@brief AKM compensated values structure
*/
struct bma4xy_bst_akm_xyz_t {
s32 x;/**<AKM09911 and AKM09912 X compensated data*/
s32 y;/**<AKM09911 and AKM09912 Y compensated data*/
s32 z;/**<AKM09911 and AKM09912 Z compensated data*/
};

/*!
* @brief FIFO header less data structure
*/
struct bma4xy_fifo_data_header_less_t {

struct bma4xy_accel_t accel_fifo[BMA4XY_FIFO_FRAME_CNT];/**<
Accel data of XYZ */
struct bma4xy_mag_t mag_fifo[BMA4XY_FIFO_FRAME_CNT];/**<
Mag data of XYZ */
u8 accel_frame_count;/**< The total number of accel frame stored
in the FIFO*/
u8 mag_frame_count;/**< The total number of mag frame stored
in the FIFO*/
};
/*!
* @brief FIFO header data structure
*/
struct bma4xy_fifo_data_header_t {
struct bma4xy_accel_t accel_fifo[BMA4XY_FIFO_FRAME_CNT];/**<
Accel data of XYZ */
struct bma4xy_mag_t mag_fifo[BMA4XY_FIFO_FRAME_CNT];/**<
Mag data of XYZ */
u32 fifo_time;/**< Value of fifo time*/
u8 skip_frame;/**< The value of skip frame information */
u8 fifo_input_config_info; /**< FIFO input config info*/
u8 accel_frame_count; /**< The total number of accel frame stored
in the FIFO*/
u8 mag_frame_count; /**< The total number of mag frame stored
in the FIFO*/
u8 fifo_header[BMA4XY_FIFO_FRAME_CNT]; /**< FIFO header info*/
};
/*!
* @brief FIFO mag data structure
*/
struct bma4xy_mag_fifo_data_t {
u8 mag_x_lsb;/**< The value of mag x LSB data*/
u8 mag_x_msb;/**< The value of mag x MSB data*/
u8 mag_y_lsb;/**< The value of mag y LSB data*/
u8 mag_y_msb;/**< The value of mag y MSB data*/
u8 mag_z_lsb;/**< The value of mag z LSB data*/
u8 mag_z_msb;/**< The value of mag z MSB data*/
u8 mag_r_y2_lsb;
/**< The value of mag r for BMM150 Y2 for YAMAHA LSB data*/
u8 mag_r_y2_msb;
/**< The value of mag r for BMM150 Y2 for YAMAHA MSB data*/
};

/**************************************************************/
/**\name	USER DATA REGISTERS DEFINITION START    */
/**************************************************************/

/**************************************************************/
/**\name	CHIP ID LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Chip ID Description - Reg Addr --> (0x00), Bit --> 0...7 */
#define BMA4XY_CHIP_ID_POS             (4)
#define BMA4XY_CHIP_ID_MSK            (0xF0)
#define BMA4XY_CHIP_ID_LEN             (4)
#define BMA4XY_CHIP_ID_REG             (BMA4XY_CHIP_ID_ADDR)

#define BMA4XY_CONFIG_STREAM_MINOR_VERSION_BYTE	0
#define BMA4XY_CONFIG_STREAM_MINOR_VERSION_POS	0
#define BMA4XY_CONFIG_STREAM_MINOR_VERSION_LEN	6
#define BMA4XY_CONFIG_STREAM_MINOR_VERSION_MSK	0x3F
#define BMA4XY_CONFIG_STREAM_MINOR_VERSION_REG	\
BMA4XY_CONFIG_STREAM_OFFSET

#define BMA4XY_CONFIG_STREAM_MAJOR_VERSION_BYTE	0
#define BMA4XY_CONFIG_STREAM_MAJOR_VERSION_POS	6
#define BMA4XY_CONFIG_STREAM_MAJOR_VERSION_LEN	4
#define BMA4XY_CONFIG_STREAM_MAJOR_VERSION_MSK	0x03C0
#define BMA4XY_CONFIG_STREAM_MAJOR_VERSION_REG	\
BMA4XY_CONFIG_STREAM_OFFSET

#define BMA4XY_CONFIG_STREAM_IMAGE_TYPE_BYTE	0
#define BMA4XY_CONFIG_STREAM_IMAGE_TYPE_POS		10
#define BMA4XY_CONFIG_STREAM_IMAGE_TYPE_LEN		6
#define BMA4XY_CONFIG_STREAM_IMAGE_TYPE_MSK		0xFC00
#define BMA4XY_CONFIG_STREAM_IMAGE_TYPE_REG	\
BMA4XY_CONFIG_STREAM_OFFSET
/**************************************************************/
/**\name	ERROR STATUS LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Error Description - Reg Addr --> (0x02), Bit --> 0 */
#define BMA4XY_ERR_STAT_POS               (0)
#define BMA4XY_ERR_STAT_LEN               (8)
#define BMA4XY_ERR_STAT_MSK               (0xFF)
#define BMA4XY_ERR_STAT_REG               (BMA4XY_ERROR_ADDR)

#define BMA4XY_FATAL_ERR_POS               (0)
#define BMA4XY_FATAL_ERR_LEN               (1)
#define BMA4XY_FATAL_ERR_MSK               (0x01)
#define BMA4XY_FATAL_ERR_REG               (BMA4XY_ERROR_ADDR)

#define BMA4XY_CMD_ERR_POS               (1)
#define BMA4XY_CMD_ERR_LEN               (1)
#define BMA4XY_CMD_ERR_MSK               (0x02)
#define BMA4XY_CMD_ERR_REG               (BMA4XY_ERROR_ADDR)

/* Error Description - Reg Addr --> (0x02), Bit --> 1...4 */
#define BMA4XY_ERR_CODE_POS               (2)
#define BMA4XY_ERR_CODE_LEN               (3)
#define BMA4XY_ERR_CODE_MSK               (0x1C)
#define BMA4XY_ERR_CODE_REG               (BMA4XY_ERROR_ADDR)

/* Error Description - Reg Addr --> (0x02), Bit --> 6 */
#define BMA4XY_FIFO_ERR_POS              (6)
#define BMA4XY_FIFO_ERR_LEN              (1)
#define BMA4XY_FIFO_ERR_MSK              (0x40)
#define BMA4XY_FIFO_ERR_REG              (BMA4XY_ERROR_ADDR)
/**************************************************************/
/**\name	MAG DATA READY LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Error Description - Reg Addr --> (0x02), Bit --> 7 */
#define BMA4XY_AUX_ERR_POS               (7)
#define BMA4XY_AUX_ERR_LEN               (1)
#define BMA4XY_AUX_ERR_MSK               (0x80)
#define BMA4XY_AUX_ERR_REG               (BMA4XY_ERROR_ADDR)
/**************************************************************/
/**\name	MAG POWER MODE LENGTH, POSITION AND MASK    */
/**************************************************************/
/* PMU_Status Description of MAG - Reg Addr --> (0x7D - page1), Bit --> 1..0 */
#define BMA4XY_MAG_POWER_MODE_STAT_POS		(0)
#define BMA4XY_MAG_POWER_MODE_STAT_LEN		(2)
#define BMA4XY_MAG_POWER_MODE_STAT_MSK		(0x03)
#define BMA4XY_MAG_POWER_MODE_STAT_REG		\
(BMA4XY_PWR_STATUS_ADDR)

/**************************************************************/
/**\name	ACCEL POWER MODE LENGTH, POSITION AND MASK    */
/**************************************************************/
/* PMU_Status Description of ACCEL - Reg Addr --> (0x7D), Bit --> 5...4 */
#define BMA4XY_ACCEL_POWER_MODE_STAT_POS               (4)
#define BMA4XY_ACCEL_POWER_MODE_STAT_LEN               (2)
#define BMA4XY_ACCEL_POWER_MODE_STAT_MSK               (0x30)
#define BMA4XY_ACCEL_POWER_MODE_STAT_REG		    \
(BMA4XY_PWR_STATUS_ADDR)
/**************************************************************/
/**\name	MAG STATUS LENGTH, POSITION AND MASK    */
/**************************************************************/
/* PMU_Status Description of MAG - Reg Addr --> (0x03), Bit --> 2 */
#define BMA4XY_AUX_MAN_OP_STATUS_POS               (2)
#define BMA4XY_AUX_MAN_OP_STATUS_LEN               (1)
#define BMA4XY_AUX_MAN_OP_STATUS_MSK               (0x04)
#define BMA4XY_AUX_MAN_OP_STATUS_REG		      \
(BMA4XY_STATUS_ADDR)
/**************************************************************/
/**\name	ACCEL POWER MODE LENGTH, POSITION AND MASK    */
/**************************************************************/
/* PMU_Status Description of ACCEL - Reg Addr --> (0x03), Bit --> 4 */
#define BMA4XY_CMD_RDY_STATUS_POS               (4)
#define BMA4XY_CMD_RDY_STATUS_LEN               (1)
#define BMA4XY_CMD_RDY_STATUS_MSK               (0x10)
#define BMA4XY_CMD_RDY_STATUS_REG		    \
(BMA4XY_STATUS_ADDR)

#define BMA4XY_DRDY_AUX_STATUS_POS               (5)
#define BMA4XY_DRDY_AUX_STATUS_LEN               (1)
#define BMA4XY_DRDY_AUX_STATUS_MSK               (0x20)
#define BMA4XY_DRDY_AUX_STATUS_REG		    \
(BMA4XY_STATUS_ADDR)

#define BMA4XY_DRDY_GYR_STATUS_POS               (6)
#define BMA4XY_DRDY_GYR_STATUS_LEN               (1)
#define BMA4XY_DRDY_GYR_STATUS_MSK               (0x40)
#define BMA4XY_DRDY_GYR_STATUS_REG		    \
(BMA4XY_STATUS_ADDR)

#define BMA4XY_DRDY_ACC_STATUS_POS               (7)
#define BMA4XY_DRDY_ACC_STATUS_LEN               (1)
#define BMA4XY_DRDY_ACC_STATUS_MSK               (0x80)
#define BMA4XY_DRDY_ACC_STATUS_REG		    \
(BMA4XY_STATUS_ADDR)

/**************************************************************/
/**\name	NV_CONFIG LENGTH, POSITION AND MASK*/
/**************************************************************/
/* NV_CONF Description - Reg Addr --> (0x70), Bit --> 3 */
#define BMA4XY_NV_ACCEL_OFFSET_POS               (3)
#define BMA4XY_NV_ACCEL_OFFSET_LEN               (1)
#define BMA4XY_NV_ACCEL_OFFSET_MSK               (0x08)
#define BMA4XY_NV_ACCEL_OFFSET_REG	(BMA4XY_NV_CONFIG_ADDR)

/**************************************************************/
/**\name	MAG DATA XYZ LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Mag_X(LSB) Description - Reg Addr --> (0x04), Bit --> 0...7 */
#define BMA4XY_DATA_0_MAG_X_LSB_POS           (0)
#define BMA4XY_DATA_0_MAG_X_LSB_LEN           (8)
#define BMA4XY_DATA_0_MAG_X_LSB_MSK          (0xFF)
#define BMA4XY_DATA_0_MAG_X_LSB_REG          (BMA4XY_DATA_0_ADDR)

/* Mag_X(LSB) Description - Reg Addr --> (0x04), Bit --> 3...7 */
#define BMA4XY_DATA_MAG_X_LSB_POS           (3)
#define BMA4XY_DATA_MAG_X_LSB_LEN           (5)
#define BMA4XY_DATA_MAG_X_LSB_MSK          (0xF8)
#define BMA4XY_DATA_MAG_X_LSB_REG          (BMA4XY_DATA_0_ADDR)

/* Mag_Y(LSB) Description - Reg Addr --> (0x06), Bit --> 3...7 */
#define BMA4XY_DATA_MAG_Y_LSB_POS           (3)
#define BMA4XY_DATA_MAG_Y_LSB_LEN           (5)
#define BMA4XY_DATA_MAG_Y_LSB_MSK          (0xF8)
#define BMA4XY_DATA_MAG_Y_LSB_REG          (BMA4XY_DATA_2_ADDR)

/* Mag_X(LSB) Description - Reg Addr --> (0x08), Bit --> 3...7 */
#define BMA4XY_DATA_MAG_Z_LSB_POS           (1)
#define BMA4XY_DATA_MAG_Z_LSB_LEN           (7)
#define BMA4XY_DATA_MAG_Z_LSB_MSK          (0xFE)
#define BMA4XY_DATA_MAG_Z_LSB_REG          (BMA4XY_DATA_4_ADDR)


/* Mag_R(LSB) Description - Reg Addr --> (0x0A), Bit --> 3...7 */
#define BMA4XY_DATA_MAG_R_LSB_POS           (2)
#define BMA4XY_DATA_MAG_R_LSB_LEN           (6)
#define BMA4XY_DATA_MAG_R_LSB_MSK          (0xFC)
#define BMA4XY_DATA_MAG_R_LSB_REG          (BMA4XY_DATA_6_ADDR)

/**************************************************************/
/**\name	ACCEL DATA XYZ LENGTH, POSITION AND MASK    */
/**************************************************************/
/* ACC_X (LSB) Description - Reg Addr --> (0x12), Bit --> 0...7 */
#define BMA4XY_ACCEL_X_LSB_POS           (0)
#define BMA4XY_ACCEL_X_LSB_LEN           (8)
#define BMA4XY_ACCEL_X_LSB_MSK          (0xFF)
#define BMA4XY_ACCEL_X_LSB_REG          (BMA4XY_ACCEL_ADDR)

/**************************************************************/
/**\name	SENSOR TIME LENGTH, POSITION AND MASK    */
/**************************************************************/
/* SENSORTIME_0 (LSB) Description - Reg Addr --> (0x18), Bit --> 0...7 */
#define BMA4XY_SENSOR_TIME_LSB_POS           (0)
#define BMA4XY_SENSOR_TIME_LSB_LEN           (8)
#define BMA4XY_SENSOR_TIME_LSB_MSK          (0xFF)
#define BMA4XY_SENSOR_TIME_LSB_REG          \
		(BMA4XY_SENSORTIME_0_ADDR)

/**************************************************************/
/**\name ACCEL DATA READY LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Status Description - Reg Addr --> 0x03, Bit --> 7 */
#define BMA4XY_STAT_DATA_RDY_ACCEL_POS           (7)
#define BMA4XY_STAT_DATA_RDY_ACCEL_LEN           (1)
#define BMA4XY_STAT_DATA_RDY_ACCEL_MSK           (0x80)
#define BMA4XY_STAT_DATA_RDY_ACCEL_REG           (BMA4XY_STAT_ADDR)

/**************************************************************/
/**\name MAG DATA READY LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Status Description - Reg Addr --> 0x03, Bit --> 5 */
#define BMA4XY_STAT_DATA_RDY_MAG_POS           (5)
#define BMA4XY_STAT_DATA_RDY_MAG_LEN           (1)
#define BMA4XY_STAT_DATA_RDY_MAG_MSK           (0x20)
#define BMA4XY_STAT_DATA_RDY_MAG_REG           (BMA4XY_STAT_ADDR)

/**************************************************************/
/**\name ADVANCE POWER SAVE LENGTH, POSITION AND MASK*/
/**************************************************************/
#define BMA4XY_ADVANCE_POWER_SAVE_POS			(0)
#define BMA4XY_ADVANCE_POWER_SAVE_LEN			(1)
#define BMA4XY_ADVANCE_POWER_SAVE_MSK			(0x01)
#define BMA4XY_ADVANCE_POWER_SAVE_REG			(BMA4XY_POWER_CONF_ADDR)

/**************************************************************/
/**\name FIFO SELF WAKE UP LENGTH, POSITION AND MASK*/
/**************************************************************/
#define BMA4XY_FIFO_SELF_WAKE_UP_POS			(1)
#define BMA4XY_FIFO_SELF_WAKE_UP_LEN			(1)
#define BMA4XY_FIFO_SELF_WAKE_UP_MSK			(0x02)
#define BMA4XY_FIFO_SELF_WAKE_UP_REG			(BMA4XY_POWER_CONF_ADDR)

/**************************************************************/
/**\name ACCELEROMETER ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
#define BMA4XY_ACCEL_ENABLE_POS			(2)
#define BMA4XY_ACCEL_ENABLE_LEN			(1)
#define BMA4XY_ACCEL_ENABLE_MSK			(0x04)
#define BMA4XY_ACCEL_ENABLE_REG			(BMA4XY_POWER_CTRL_ADDR)

/**************************************************************/
/**\name MAGNETOMETER ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
#define BMA4XY_MAG_ENABLE_POS			(0)
#define BMA4XY_MAG_ENABLE_LEN			(1)
#define BMA4XY_MAG_ENABLE_MSK			(0x01)
#define BMA4XY_MAG_ENABLE_REG			(BMA4XY_POWER_CTRL_ADDR)

/**************************************************************/
/**\name FIFO WATERMARK INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 1 */
#define BMA4XY_INTR_STAT_1_FIFO_WM_INTR_POS               (1)
#define BMA4XY_INTR_STAT_1_FIFO_WM_INTR_LEN               (1)
#define BMA4XY_INTR_STAT_1_FIFO_WM_INTR_MSK               (0x02)
#define BMA4XY_INTR_STAT_1_FIFO_WM_INTR_REG               \
		(BMA4XY_INTR_STAT_1_ADDR)

/**************************************************************/
/**\name FIFO FULL INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 0 */
#define BMA4XY_INTR_STAT_1_FIFO_FULL_INTR_POS               (0)
#define BMA4XY_INTR_STAT_1_FIFO_FULL_INTR_LEN               (1)
#define BMA4XY_INTR_STAT_1_FIFO_FULL_INTR_MSK               (0x01)
#define BMA4XY_INTR_STAT_1_FIFO_FULL_INTR_REG               \
		(BMA4XY_INTR_STAT_1_ADDR)

/**************************************************************/
/**\name	TEMPERATURE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Temperature Description - LSB Reg Addr --> (0x20), Bit --> 0...7 */
#define BMA4XY_TEMP_LSB_VALUE_POS               (0)
#define BMA4XY_TEMP_LSB_VALUE_LEN               (8)
#define BMA4XY_TEMP_LSB_VALUE_MSK               (0xFF)
#define BMA4XY_TEMP_LSB_VALUE_REG               \
		(BMA4XY_TEMPERATURE_0_ADDR)

/* Temperature Description - LSB Reg Addr --> 0x21, Bit --> 0...7 */
#define BMA4XY_TEMP_MSB_VALUE_POS               (0)
#define BMA4XY_TEMP_MSB_VALUE_LEN               (8)
#define BMA4XY_TEMP_MSB_VALUE_MSK               (0xFF)
#define BMA4XY_TEMP_MSB_VALUE_REG               \
		(BMA4XY_TEMPERATURE_1_ADDR)
/**************************************************************/
/**\name	FIFO BYTE COUNTER LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Length0 Description - Reg Addr --> 0x22, Bit --> 0...7 */
#define BMA4XY_FIFO_BYTE_COUNTER_LSB_POS           (0)
#define BMA4XY_FIFO_BYTE_COUNTER_LSB_LEN           (8)
#define BMA4XY_FIFO_BYTE_COUNTER_LSB_MSK          (0xFF)
#define BMA4XY_FIFO_BYTE_COUNTER_LSB_REG          \
		(BMA4XY_FIFO_LENGTH_0_ADDR)

/*Fifo_Length1 Description - Reg Addr --> 0x25, Bit --> 0...5 */
#define BMA4XY_FIFO_BYTE_COUNTER_MSB_POS           (0)
#define BMA4XY_FIFO_BYTE_COUNTER_MSB_LEN           (6)
#define BMA4XY_FIFO_BYTE_COUNTER_MSB_MSK          (0x3F)
#define BMA4XY_FIFO_BYTE_COUNTER_MSB_REG          \
		(BMA4XY_FIFO_LENGTH_1_ADDR)

/**************************************************************/
/**\name	FIFO DATA LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Data Description - Reg Addr --> 0x24, Bit --> 0...7 */
#define BMA4XY_FIFO_DATA_POS           (0)
#define BMA4XY_FIFO_DATA_LEN           (8)
#define BMA4XY_FIFO_DATA_MSK          (0xFF)
#define BMA4XY_FIFO_DATA_REG          (BMA4XY_FIFO_DATA_ADDR)

/**************************************************************/
/**\name	ACCEL CONFIGURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Acc_Conf Description - Reg Addr --> (0x40), Bit --> 0...3 */
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_POS               (0)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_LEN               (4)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_MSK               (0x0F)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_REG		       \
(BMA4XY_ACCEL_CONFIG_ADDR)

/* Acc_Conf Description - Reg Addr --> (0x40), Bit --> 4...6 */
#define BMA4XY_ACCEL_BW_POS               (4)
#define BMA4XY_ACCEL_BW_LEN               (3)
#define BMA4XY_ACCEL_BW_MSK               (0x70)
#define BMA4XY_ACCEL_BW_REG	(BMA4XY_ACCEL_CONFIG_ADDR)

/* Acc_Range Description - Reg Addr --> 0x41, Bit --> 0...3 */
#define BMA4XY_ACCEL_RANGE_POS               (0)
#define BMA4XY_ACCEL_RANGE_LEN               (2)
#define BMA4XY_ACCEL_RANGE_MSK               (0x03)
#define BMA4XY_ACCEL_RANGE_REG              \
(BMA4XY_ACCEL_RANGE_ADDR)

/* Acc_Conf Description - Reg Addr --> (0x40), Bit --> 7 */
#define BMA4XY_ACCEL_UNDER_SAMPLING_POS		(7)
#define BMA4XY_UNDER_SAMPLING_LEN		(1)
#define BMA4XY_ACCEL_UNDER_SAMPLING_MSK		(0x80)
#define BMA4XY_ACCEL_UNDER_SAMPLING_REG	\
(BMA4XY_ACCEL_CONFIG_ADDR)

/**************************************************************/
/**\name	MAG CONFIGURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Mag_Conf Description - Reg Addr --> (0x44), Bit --> 0...3 */
#define BMA4XY_MAG_CONFIG_OUTPUT_DATA_RATE_POS               (0)
#define BMA4XY_MAG_CONFIG_OUTPUT_DATA_RATE_LEN               (4)
#define BMA4XY_MAG_CONFIG_OUTPUT_DATA_RATE_MSK               (0x0F)
#define BMA4XY_MAG_CONFIG_OUTPUT_DATA_RATE_REG               \
(BMA4XY_MAG_CONFIG_ADDR)

/* Mag_Conf Description - Reg Addr --> (0x44), Bit --> 4...7 */
#define BMA4XY_MAG_CONFIG_OFFSET_POS               (4)
#define BMA4XY_MAG_CONFIG_OFFSET_LEN               (4)
#define BMA4XY_MAG_CONFIG_OFFSET_MSK               (0xF0)
#define BMA4XY_MAG_CONFIG_OFFSET_REG               \
(BMA4XY_MAG_CONFIG_ADDR)
/**************************************************************/
/**\name	FIFO DOWNS LENGTH, POSITION AND MASK*/
/**************************************************************/

/**************************************************************/
/**\name	FIFO FILTER FOR ACCEL  LENGTH, POSITION AND MASK*/
/**************************************************************/

/* Fifo_Downs Description - Reg Addr --> 0x45, Bit --> 4...6 */
#define BMA4XY_FIFO_DOWN_ACCEL_POS               (4)
#define BMA4XY_FIFO_DOWN_ACCEL_LEN               (3)
#define BMA4XY_FIFO_DOWN_ACCEL_MSK               (0x70)
#define BMA4XY_FIFO_DOWN_ACCEL_REG	(BMA4XY_FIFO_DOWN_ADDR)

/* Fifo_FILT Description - Reg Addr --> 0x45, Bit --> 7 */
#define BMA4XY_FIFO_FILTER_ACCEL_POS               (7)
#define BMA4XY_FIFO_FILTER_ACCEL_LEN               (1)
#define BMA4XY_FIFO_FILTER_ACCEL_MSK               (0x80)
#define BMA4XY_FIFO_FILTER_ACCEL_REG	(BMA4XY_FIFO_DOWN_ADDR)
/**************************************************************/
/**\name	FIFO WATER MARK LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_0 Description - Reg Addr --> 0x46, Bit --> 0...7 */
#define BMA4XY_FIFO_WM_POS               (0)
#define BMA4XY_FIFO_WM_LEN               (8)
#define BMA4XY_FIFO_WM_MSK               (0xFF)
#define BMA4XY_FIFO_WM_REG	(BMA4XY_FIFO_WTM_0_ADDR)
/**************************************************************/
/**\name	FIFO TIME LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x48, Bit --> 1 */
#define BMA4XY_FIFO_TIME_ENABLE_POS               (1)
#define BMA4XY_FIFO_TIME_ENABLE_LEN               (1)
#define BMA4XY_FIFO_TIME_ENABLE_MSK               (0x02)
#define BMA4XY_FIFO_TIME_ENABLE_REG	(BMA4XY_FIFO_CONFIG_0_ADDR)
/* Fifo_Config_1 Description - Reg Addr --> 0x48, Bit --> 0 */
#define BMA4XY_FIFO_STOP_ON_FULL_POS               (0)
#define BMA4XY_FIFO_STOP_ON_FULL_LEN               (1)
#define BMA4XY_FIFO_STOP_ON_FULL_MSK               (0x01)
#define BMA4XY_FIFO_STOP_ON_FULL_REG	(BMA4XY_FIFO_CONFIG_0_ADDR)
/**************************************************************/
/**\name	FIFO TAG INTERRUPT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x49, Bit --> 2 */
#define BMA4XY_FIFO_TAG_INTR2_ENABLE_POS               (2)
#define BMA4XY_FIFO_TAG_INTR2_ENABLE_LEN               (1)
#define BMA4XY_FIFO_TAG_INTR2_ENABLE_MSK               (0x04)
#define BMA4XY_FIFO_TAG_INTR2_ENABLE_REG	(BMA4XY_FIFO_CONFIG_1_ADDR)
/* Fifo_Config_1 Description - Reg Addr --> 0x48, Bit --> 3 */
#define BMA4XY_FIFO_TAG_INTR1_ENABLE_POS               (3)
#define BMA4XY_FIFO_TAG_INTR1_ENABLE_LEN               (1)
#define BMA4XY_FIFO_TAG_INTR1_ENABLE_MSK               (0x08)
#define BMA4XY_FIFO_TAG_INTR1_ENABLE_REG	(BMA4XY_FIFO_CONFIG_1_ADDR)
/**************************************************************/
/**\name	FIFO HEADER LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x49, Bit --> 4 */
#define BMA4XY_FIFO_HEADER_ENABLE_POS               (4)
#define BMA4XY_FIFO_HEADER_ENABLE_LEN               (1)
#define BMA4XY_FIFO_HEADER_ENABLE_MSK               (0x10)
#define BMA4XY_FIFO_HEADER_ENABLE_REG		         \
(BMA4XY_FIFO_CONFIG_1_ADDR)
/**************************************************************/
/**\name	FIFO MAG ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x49, Bit --> 5 */
#define BMA4XY_FIFO_MAG_ENABLE_POS               (5)
#define BMA4XY_FIFO_MAG_ENABLE_LEN               (1)
#define BMA4XY_FIFO_MAG_ENABLE_MSK               (0x20)
#define BMA4XY_FIFO_MAG_ENABLE_REG		     \
(BMA4XY_FIFO_CONFIG_1_ADDR)
/**************************************************************/
/**\name	FIFO ACCEL ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x49, Bit --> 6 */
#define BMA4XY_FIFO_ACCEL_ENABLE_POS               (6)
#define BMA4XY_FIFO_ACCEL_ENABLE_LEN               (1)
#define BMA4XY_FIFO_ACCEL_ENABLE_MSK               (0x40)
#define BMA4XY_FIFO_ACCEL_ENABLE_REG		        \
(BMA4XY_FIFO_CONFIG_1_ADDR)
/**************************************************************/
/**\name	MAG I2C ADDRESS SELECTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Mag_IF_0 Description - Reg Addr --> 0x4b, Bit --> 1...7 */
#define BMA4XY_I2C_DEVICE_ADDR_POS               (1)
#define BMA4XY_I2C_DEVICE_ADDR_LEN               (7)
#define BMA4XY_I2C_DEVICE_ADDR_MSK               (0xFE)
#define BMA4XY_I2C_DEVICE_ADDR_REG	(BMA4XY_MAG_IF_0_ADDR)
/**************************************************************/
/**\name MAG CONFIGURATION FOR SECONDARY
	INTERFACE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 0...1 */
#define BMA4XY_MAG_BURST_POS               (0)
#define BMA4XY_MAG_BURST_LEN               (2)
#define BMA4XY_MAG_BURST_MSK               (0x03)
#define BMA4XY_MAG_BURST_REG               (BMA4XY_MAG_IF_1_ADDR)
/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 7 */
#define BMA4XY_MAG_MANUAL_ENABLE_POS               (7)
#define BMA4XY_MAG_MANUAL_ENABLE_LEN               (1)
#define BMA4XY_MAG_MANUAL_ENABLE_MSK               (0x80)
#define BMA4XY_MAG_MANUAL_ENABLE_REG               \
(BMA4XY_MAG_IF_1_ADDR)
/* Mag_IF_2 Description - Reg Addr --> 0x4d, Bit -->0... 7 */
#define BMA4XY_READ_ADDR_POS               (0)
#define BMA4XY_READ_ADDR_LEN               (8)
#define BMA4XY_READ_ADDR_MSK               (0xFF)
#define BMA4XY_READ_ADDR_REG               (BMA4XY_MAG_IF_2_ADDR)
/* Mag_IF_3 Description - Reg Addr --> 0x4e, Bit -->0... 7 */
#define BMA4XY_WRITE_ADDR_POS               (0)
#define BMA4XY_WRITE_ADDR_LEN               (8)
#define BMA4XY_WRITE_ADDR_MSK               (0xFF)
#define BMA4XY_WRITE_ADDR_REG               (BMA4XY_MAG_IF_3_ADDR)
/* Mag_IF_4 Description - Reg Addr --> 0x4f, Bit -->0... 7 */
#define BMA4XY_WRITE_DATA_POS               (0)
#define BMA4XY_WRITE_DATA_LEN               (8)
#define BMA4XY_WRITE_DATA_MSK               (0xFF)
#define BMA4XY_WRITE_DATA_REG               (BMA4XY_MAG_IF_4_ADDR)
/**************************************************************/
/**\name	OUTPUT TYPE ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->3 */
#define BMA4XY_INTR1_OUTPUT_ENABLE_POS               (3)
#define BMA4XY_INTR1_OUTPUT_ENABLE_LEN               (1)
#define BMA4XY_INTR1_OUTPUT_ENABLE_MSK               (0x08)
#define BMA4XY_INTR1_OUTPUT_ENABLE_REG		\
(BMA4XY_INTR1_OUT_CTRL_ADDR)
/* Int_Out_Ctrl Description - Reg Addr --> 0x54, Bit -->3 */
#define BMA4XY_INTR2_OUTPUT_ENABLE_POS               (3)
#define BMA4XY_INTR2_OUTPUT_ENABLE_LEN               (1)
#define BMA4XY_INTR2_OUTPUT_ENABLE_MSK               (0x08)
#define BMA4XY_INTR2_OUTPUT_ENABLE_REG		\
(BMA4XY_INTR2_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	INPUT ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Latch Description - Reg Addr --> 0x53, Bit -->4 */
#define BMA4XY_INTR1_INPUT_ENABLE_POS               (4)
#define BMA4XY_INTR1_INPUT_ENABLE_LEN               (1)
#define BMA4XY_INTR1_INPUT_ENABLE_MSK               (0x10)
#define BMA4XY_INTR1_INPUT_ENABLE_REG               \
(BMA4XY_INTR1_OUT_CTRL_ADDR)
/* Int_Latch Description - Reg Addr --> 0x54, Bit -->4*/
#define BMA4XY_INTR2_INPUT_ENABLE_POS               (4)
#define BMA4XY_INTR2_INPUT_ENABLE_LEN               (1)
#define BMA4XY_INTR2_INPUT_ENABLE_MSK               (0x10)
#define BMA4XY_INTR2_INPUT_ENABLE_REG              \
(BMA4XY_INTR2_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	OUTPUT MODE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->2 */
#define BMA4XY_INTR1_OUTPUT_MODE_POS               (2)
#define BMA4XY_INTR1_OUTPUT_MODE_LEN               (1)
#define BMA4XY_INTR1_OUTPUT_MODE_MSK               (0x04)
#define BMA4XY_INTR1_OUTPUT_MODE_REG		\
(BMA4XY_INTR1_OUT_CTRL_ADDR)
/* Int_Out_Ctrl Description - Reg Addr --> 0x54, Bit -->2 */
#define BMA4XY_INTR2_OUTPUT_MODE_POS               (2)
#define BMA4XY_INTR2_OUTPUT_MODE_LEN               (1)
#define BMA4XY_INTR2_OUTPUT_MODE_MSK               (0x04)
#define BMA4XY_INTR2_OUTPUT_MODE_REG		\
(BMA4XY_INTR2_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	LEVEL LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->1 */
#define BMA4XY_INTR1_LEVEL_POS               (1)
#define BMA4XY_INTR1_LEVEL_LEN               (1)
#define BMA4XY_INTR1_LEVEL_MSK				 (0x02)
#define BMA4XY_INTR1_LEVEL_REG		\
(BMA4XY_INTR1_OUT_CTRL_ADDR)
/* Int_Out_Ctrl Description - Reg Addr --> 0x54, Bit -->1 */
#define BMA4XY_INTR2_LEVEL_POS               (1)
#define BMA4XY_INTR2_LEVEL_LEN               (1)
#define BMA4XY_INTR2_LEVEL_MSK               (0x02)
#define BMA4XY_INTR2_LEVEL_REG		\
(BMA4XY_INTR2_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	EDGE CONTROL LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->0 */
#define BMA4XY_INTR1_EDGE_CTRL_POS               (0)
#define BMA4XY_INTR1_EDGE_CTRL_LEN               (1)
#define BMA4XY_INTR1_EDGE_CTRL_MSK				 (0x01)
#define BMA4XY_INTR1_EDGE_CTRL_REG		\
(BMA4XY_INTR1_OUT_CTRL_ADDR)
/* Int_Out_Ctrl Description - Reg Addr --> 0x54, Bit -->0 */
#define BMA4XY_INTR2_EDGE_CTRL_POS               (0)
#define BMA4XY_INTR2_EDGE_CTRL_LEN               (1)
#define BMA4XY_INTR2_EDGE_CTRL_MSK               (0x01)
#define BMA4XY_INTR2_EDGE_CTRL_REG		\
(BMA4XY_INTR2_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF FIFO FULL AND
	WATER MARK LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->1 */
#define BMA4XY_INTR_MAP_1_INTR2_FIFO_FULL_POS               (0)
#define BMA4XY_INTR_MAP_1_INTR2_FIFO_FULL_LEN               (1)
#define BMA4XY_INTR_MAP_1_INTR2_FIFO_FULL_MSK               (0x01)
#define BMA4XY_INTR_MAP_1_INTR2_FIFO_FULL_REG	         \
(BMA4XY_INTR_MAP_1_ADDR)
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->2 */
#define BMA4XY_INTR_MAP_1_INTR2_FIFO_WM_POS               (1)
#define BMA4XY_INTR_MAP_1_INTR2_FIFO_WM_LEN               (1)
#define BMA4XY_INTR_MAP_1_INTR2_FIFO_WM_MSK               (0x02)
#define BMA4XY_INTR_MAP_1_INTR2_FIFO_WM_REG	         \
(BMA4XY_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF DATA READY LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->3 */
#define BMA4XY_INTR_MAP_1_INTR2_DATA_RDY_POS               (2)
#define BMA4XY_INTR_MAP_1_INTR2_DATA_RDY_LEN               (1)
#define BMA4XY_INTR_MAP_1_INTR2_DATA_RDY_MSK               (0x04)
#define BMA4XY_INTR_MAP_1_INTR2_DATA_RDY_REG	      \
(BMA4XY_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF FIFO FULL AND
	WATER MARK LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x56, Bit -->0 */
#define BMA4XY_INTR_MAP_1_A_POS               (0)
#define BMA4XY_INTR_MAP_1_A_LEN               (1)
#define BMA4XY_INTR_MAP_1_A_MSK               (0x01)
#define BMA4XY_INTR_MAP_1_A_REG	(BMA4XY_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF HIGH_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x56, Bit -->1 */
#define BMA4XY_INTR_MAP_1_B_POS               (1)
#define BMA4XY_INTR_MAP_1_B_LEN               (1)
#define BMA4XY_INTR_MAP_1_B_MSK               (0x02)
#define BMA4XY_INTR_MAP_1_B_REG	\
(BMA4XY_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF ANY MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x56, Bit -->2 */
#define BMA4XY_INTR_MAP_1_C_POS      (2)
#define BMA4XY_INTR_MAP_1_C_LEN      (1)
#define BMA4XY_INTR_MAP_1_C_MSK     (0x04)
#define BMA4XY_INTR_MAP_1_C_REG     \
(BMA4XY_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF NO MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x56, Bit -->3 */
#define BMA4XY_INTR_MAP_1_D_POS               (3)
#define BMA4XY_INTR_MAP_1_D_LEN               (1)
#define BMA4XY_INTR_MAP_1_D_MSK               (0x08)
#define BMA4XY_INTR_MAP_1_D_REG (BMA4XY_INTR_MAP_1_ADDR)
/* Int_Map_2 Description - Reg Addr --> 0x56, Bit -->4 */
#define BMA4XY_INTR_MAP_1_E_POS               (4)
#define BMA4XY_INTR_MAP_1_E_LEN               (1)
#define BMA4XY_INTR_MAP_1_E_MSK               (0x10)
#define BMA4XY_INTR_MAP_1_E_REG	(BMA4XY_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF HIGH_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x56, Bit -->5 */
#define BMA4XY_INTR_MAP_1_F_POS               (5)
#define BMA4XY_INTR_MAP_1_F_LEN               (1)
#define BMA4XY_INTR_MAP_1_F_MSK               (0x20)
#define BMA4XY_INTR_MAP_1_F_REG	\
(BMA4XY_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF ANY MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x56, Bit -->6 */
#define BMA4XY_INTR_MAP_1_G_POS      (6)
#define BMA4XY_INTR_MAP_1_G_LEN      (1)
#define BMA4XY_INTR_MAP_1_G_MSK     (0x40)
#define BMA4XY_INTR_MAP_1_G_REG     \
(BMA4XY_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF NO MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x56, Bit -->7 */
#define BMA4XY_INTR_MAP_1_H_POS               (7)
#define BMA4XY_INTR_MAP_1_H_LEN               (1)
#define BMA4XY_INTR_MAP_1_H_MSK               (0x80)
#define BMA4XY_INTR_MAP_1_H_REG (BMA4XY_INTR_MAP_1_ADDR)
/* Int_Map_1 Description - Reg Addr --> 0x58, Bit -->0 */
#define BMA4XY_INTR_MAP_1_FIFO_FULL_POS               (0)
#define BMA4XY_INTR_MAP_1_FIFO_FULL_LEN               (1)
#define BMA4XY_INTR_MAP_1_FIFO_FULL_MSK               (0x01)
#define BMA4XY_INTR_MAP_1_FIFO_FULL_REG	       \
(BMA4XY_INTR_HW_ADDR)
/* Int_Map_1 Description - Reg Addr --> 0x58, Bit -->1 */
#define BMA4XY_INTR_MAP_1_FIFO_WM_POS               (1)
#define BMA4XY_INTR_MAP_1_FIFO_WM_LEN               (1)
#define BMA4XY_INTR_MAP_1_FIFO_WM_MSK               (0x02)
#define BMA4XY_INTR_MAP_1_FIFO_WM_REG	\
(BMA4XY_INTR_HW_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF DATA READY LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x58, Bit -->2 */
#define BMA4XY_INTR_MAP_1_DATA_RDY_POS               (2)
#define BMA4XY_INTR_MAP_1_DATA_RDY_LEN               (1)
#define BMA4XY_INTR_MAP_1_DATA_RDY_MSK               (0x04)
#define BMA4XY_INTR_MAP_1_DATA_RDY_REG	\
(BMA4XY_INTR_HW_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF LOW_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->0 */
#define BMA4XY_INTR_MAP_2_A_POS               (0)
#define BMA4XY_INTR_MAP_2_A_LEN               (1)
#define BMA4XY_INTR_MAP_2_A_MSK               (0x01)
#define BMA4XY_INTR_MAP_2_A_REG	(BMA4XY_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF HIGH_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->1 */
#define BMA4XY_INTR_MAP_2_B_POS               (1)
#define BMA4XY_INTR_MAP_2_B_LEN               (1)
#define BMA4XY_INTR_MAP_2_B_MSK               (0x02)
#define BMA4XY_INTR_MAP_2_B_REG	\
(BMA4XY_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF ANY MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->2 */
#define BMA4XY_INTR_MAP_2_C_POS      (2)
#define BMA4XY_INTR_MAP_2_C_LEN      (1)
#define BMA4XY_INTR_MAP_2_C_MSK     (0x04)
#define BMA4XY_INTR_MAP_2_C_REG     \
(BMA4XY_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF NO MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->3 */
#define BMA4XY_INTR_MAP_2_D_POS               (3)
#define BMA4XY_INTR_MAP_2_D_LEN               (1)
#define BMA4XY_INTR_MAP_2_D_MSK               (0x08)
#define BMA4XY_INTR_MAP_2_D_REG (BMA4XY_INTR_MAP_2_ADDR)
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->4 */
#define BMA4XY_INTR_MAP_2_E_POS               (4)
#define BMA4XY_INTR_MAP_2_E_LEN               (1)
#define BMA4XY_INTR_MAP_2_E_MSK               (0x10)
#define BMA4XY_INTR_MAP_2_E_REG	(BMA4XY_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF HIGH_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->5 */
#define BMA4XY_INTR_MAP_2_F_POS               (5)
#define BMA4XY_INTR_MAP_2_F_LEN               (1)
#define BMA4XY_INTR_MAP_2_F_MSK               (0x20)
#define BMA4XY_INTR_MAP_2_F_REG	\
(BMA4XY_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF ANY MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->6 */
#define BMA4XY_INTR_MAP_2_G_POS      (6)
#define BMA4XY_INTR_MAP_2_G_LEN      (1)
#define BMA4XY_INTR_MAP_2_G_MSK     (0x40)
#define BMA4XY_INTR_MAP_2_G_REG     \
(BMA4XY_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF NO MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->7 */
#define BMA4XY_INTR_MAP_2_H_POS               (7)
#define BMA4XY_INTR_MAP_2_H_LEN               (1)
#define BMA4XY_INTR_MAP_2_H_MSK               (0x80)
#define BMA4XY_INTR_MAP_2_H_REG (BMA4XY_INTR_MAP_2_ADDR)
/* Int_Map_1 Description - Reg Addr --> 0x58, Bit -->4 */
#define BMA4XY_INTR_MAP_2_FIFO_FULL_POS               (4)
#define BMA4XY_INTR_MAP_2_FIFO_FULL_LEN               (1)
#define BMA4XY_INTR_MAP_2_FIFO_FULL_MSK               (0x10)
#define BMA4XY_INTR_MAP_2_FIFO_FULL_REG	       \
(BMA4XY_INTR_HW_ADDR)
/* Int_Map_1 Description - Reg Addr --> 0x58, Bit -->5 */
#define BMA4XY_INTR_MAP_2_FIFO_WM_POS               (5)
#define BMA4XY_INTR_MAP_2_FIFO_WM_LEN               (1)
#define BMA4XY_INTR_MAP_2_FIFO_WM_MSK               (0x20)
#define BMA4XY_INTR_MAP_2_FIFO_WM_REG	\
(BMA4XY_INTR_HW_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPING OF DATA READY LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->6 */
#define BMA4XY_INTR_MAP_2_DATA_RDY_POS               (6)
#define BMA4XY_INTR_MAP_2_DATA_RDY_LEN               (1)
#define BMA4XY_INTR_MAP_2_DATA_RDY_MSK               (0x04)
#define BMA4XY_INTR_MAP_2_DATA_RDY_REG	\
(BMA4XY_INTR_HW_ADDR)
/**************************************************************/
/**\name	FOR UC CONFIG LENGTH, POSITION AND MASK*/
/**************************************************************/
#define BMA4XY_UC_EN_POS               (0)
#define BMA4XY_UC_EN_LEN               (1)
#define BMA4XY_UC_EN_MSK               (0x01)
#define BMA4XY_UC_EN_REG	\
(BMA4XY_UC_CONF_ADDR)

#define BMA4XY_FIFO_MODE_EN_POS               (1)
#define BMA4XY_FIFO_MODE_EN_LEN               (1)
#define BMA4XY_FIFO_MODE_EN_MSK               (0x02)
#define BMA4XY_FIFO_MODE_EN_REG	\
(BMA4XY_UC_CONF_ADDR)

#define BMA4XY_MEM_CONF_RAM1_POS               (2)
#define BMA4XY_MEM_CONF_RAM1_LEN               (2)
#define BMA4XY_MEM_CONF_RAM1_MSK               (0x0C)
#define BMA4XY_MEM_CONF_RAM1_REG	\
(BMA4XY_UC_CONF_ADDR)

#define BMA4XY_MEM_CONF_RAM2_POS               (4)
#define BMA4XY_MEM_CONF_RAM2_LEN               (2)
#define BMA4XY_MEM_CONF_RAM2_MSK               (0x30)
#define BMA4XY_MEM_CONF_RAM2_REG	\
(BMA4XY_UC_CONF_ADDR)

#define BMA4XY_MEM_CONF_RAM3_POS               (6)
#define BMA4XY_MEM_CONF_RAM3_LEN               (2)
#define BMA4XY_MEM_CONF_RAM3_MSK               (0xC0)
#define BMA4XY_MEM_CONF_RAM3_REG	\
(BMA4XY_UC_CONF_ADDR)

/**************************************************************/
/**\name	FOC ACCEL XYZ LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Foc_Conf Description - Reg Addr --> (0x69), Bit --> 0...1 */
#define BMA4XY_FOC_ACCEL_Z_POS               (0)
#define BMA4XY_FOC_ACCEL_Z_LEN               (2)
#define BMA4XY_FOC_ACCEL_Z_MSK               (0x03)
#define BMA4XY_FOC_ACCEL_Z_REG               (BMA4XY_FOC_CONFIG_ADDR)

/* Foc_Conf Description - Reg Addr --> (0x69), Bit --> 2...3 */
#define BMA4XY_FOC_ACCEL_Y_POS               (2)
#define BMA4XY_FOC_ACCEL_Y_LEN               (2)
#define BMA4XY_FOC_ACCEL_Y_MSK               (0x0C)
#define BMA4XY_FOC_ACCEL_Y_REG               (BMA4XY_FOC_CONFIG_ADDR)

/* Foc_Conf Description - Reg Addr --> (0x69), Bit --> 4...5 */
#define BMA4XY_FOC_ACCEL_X_POS               (4)
#define BMA4XY_FOC_ACCEL_X_LEN               (2)
#define BMA4XY_FOC_ACCEL_X_MSK               (0x30)
#define BMA4XY_FOC_ACCEL_X_REG               (BMA4XY_FOC_CONFIG_ADDR)

/*IF_CONF Description - Reg Addr --> (0x6B), Bit --> 0 */
#define BMA4XY_CONFIG_SPI3_POS               (0)
#define BMA4XY_CONFIG_SPI3_LEN               (1)
#define BMA4XY_CONFIG_SPI3_MSK               (0x01)
#define BMA4XY_CONFIG_SPI3_REG               \
(BMA4XY_IF_CONFIG_ADDR)

/*IF_CONF Description - Reg Addr --> (0x6B), Bit --> 4 */
#define BMA4XY_IF_CONFIG_IF_MODE_POS               (4)
#define BMA4XY_IF_CONFIG_IF_MODE_LEN               (1)
#define BMA4XY_IF_CONFIG_IF_MODE_MSK               (0x10)
#define BMA4XY_IF_CONFIG_IF_MODE_REG		\
(BMA4XY_IF_CONFIG_ADDR)

/**************************************************************/
/**\name	ACCEL SELF TEST LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 0...1 */
#define BMA4XY_ACCEL_SELFTEST_ENABLE_POS               (0)
#define BMA4XY_ACCEL_SELFTEST_ENABLE_LEN               (1)
#define BMA4XY_ACCEL_SELFTEST_ENABLE_MSK               (0x01)
#define BMA4XY_ACCEL_SELFTEST_ENABLE_REG	(BMA4XY_SELF_TEST_ADDR)

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 2 */
#define BMA4XY_ACCEL_SELFTEST_SIGN_POS               (2)
#define BMA4XY_ACCEL_SELFTEST_SIGN_LEN               (1)
#define BMA4XY_ACCEL_SELFTEST_SIGN_MSK               (0x04)
#define BMA4XY_ACCEL_SELFTEST_SIGN_REG	(BMA4XY_SELF_TEST_ADDR)

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 3 */
#define BMA4XY_SELFTEST_AMP_POS               (3)
#define BMA4XY_SELFTEST_AMP_LEN               (1)
#define BMA4XY_SELFTEST_AMP_MSK               (0x08)
#define BMA4XY_SELFTEST_AMP_REG		(BMA4XY_SELF_TEST_ADDR)

/**************************************************************/
/**\name	ACCEL MANUAL OFFSET LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Offset_0 Description - Reg Addr --> (0x71), Bit --> 0...7 */
#define BMA4XY_OFFSET_0_ACCEL_OFF_X_POS               (0)
#define BMA4XY_OFFSET_0_ACCEL_OFF_X_LEN               (8)
#define BMA4XY_OFFSET_0_ACCEL_OFF_X_MSK               (0xFF)
#define BMA4XY_OFFSET_0_ACCEL_OFF_X_REG	(BMA4XY_OFFSET_0_ADDR)

/* Offset_1 Description - Reg Addr --> 0x72, Bit --> 0...7 */
#define BMA4XY_OFFSET_1_ACCEL_OFF_Y_POS               (0)
#define BMA4XY_OFFSET_1_ACCEL_OFF_Y_LEN               (8)
#define BMA4XY_OFFSET_1_ACCEL_OFF_Y_MSK               (0xFF)
#define BMA4XY_OFFSET_1_ACCEL_OFF_Y_REG	(BMA4XY_OFFSET_1_ADDR)

/* Offset_2 Description - Reg Addr --> 0x73, Bit --> 0...7 */
#define BMA4XY_OFFSET_2_ACCEL_OFF_Z_POS               (0)
#define BMA4XY_OFFSET_2_ACCEL_OFF_Z_LEN               (8)
#define BMA4XY_OFFSET_2_ACCEL_OFF_Z_MSK               (0xFF)
#define BMA4XY_OFFSET_2_ACCEL_OFF_Z_REG	(BMA4XY_OFFSET_2_ADDR)

/**************************************************************/
/**\name	ACCEL OFFSET  ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 6 */
#define BMA4XY_OFFSET_6_ACCEL_OFF_ENABLE_POS               (6)
#define BMA4XY_OFFSET_6_ACCEL_OFF_ENABLE_LEN               (1)
#define BMA4XY_OFFSET_6_ACCEL_OFF_ENABLE_MSK               (0x40)
#define BMA4XY_OFFSET_6_ACCEL_OFF_ENABLE_REG	 \
(BMA4XY_OFFSET_6_ADDR)

/* USER REGISTERS DEFINITION END */
/**************************************************************************/
/* CMD REGISTERS DEFINITION START */
/**************************************************************/
/**\name	COMMAND REGISTER LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Command description address - Reg Addr --> 0x7E, Bit -->  0....7 */
#define BMA4XY_CMD_COMMANDS_POS              (0)
#define BMA4XY_CMD_COMMANDS_LEN              (8)
#define BMA4XY_CMD_COMMANDS_MSK              (0xFF)
#define BMA4XY_CMD_COMMANDS_REG	 (BMA4XY_CMD_COMMANDS_ADDR)
/**************************************************************/
/**\name	PAGE ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Target page address - Reg Addr --> 0x7F, Bit -->  4...5 */
#define BMA4XY_CMD_TARGET_PAGE_POS           (4)
#define BMA4XY_CMD_TARGET_PAGE_LEN           (2)
#define BMA4XY_CMD_TARGET_PAGE_MSK           (0x30)
#define BMA4XY_CMD_TARGET_PAGE_REG	 (BMA4XY_CMD_EXT_MODE_ADDR)

/* Target page address - Reg Addr --> 0x7F, Bit -->  4....5 */
#define BMA4XY_CMD_PAGING_EN_POS           (7)
#define BMA4XY_CMD_PAGING_EN_LEN           (1)
#define BMA4XY_CMD_PAGING_EN_MSK           (0x80)
#define BMA4XY_CMD_PAGING_EN_REG		(BMA4XY_CMD_EXT_MODE_ADDR)

/* Target page address - Reg Addr --> 0x0B, Bit -->  0....1 */
#define BMA4XY_PULL_UP_POS           (0)
#define BMA4XY_PULL_UP_LEN           (2)
#define BMA4XY_PULL_UP_MSK           (0x03)
#define BMA4XY_PULL_UP_REG		(BMA4XY_COM_C_TRIM_REG_0_ADDR)

/* Target page address - Reg Addr --> 0x0B, Bit -->  2 */
#define BMA4XY_SEC_INTERFACE_ENABLE_POS           (2)
#define BMA4XY_SEC_INTERFACE_ENABLE_LEN           (1)
#define BMA4XY_SEC_INTERFACE_ENABLE_MSK           (0x04)
#define BMA4XY_SEC_INTERFACE_ENABLE_REG		(BMA4XY_COM_C_TRIM_REG_0_ADDR)

/**************************************************************************/
/* CMD REGISTERS DEFINITION END */

/**************************************************/
/**\name	FIFO FRAME COUNT DEFINITION           */
/*************************************************/
#define FIFO_FRAME				(1024)
#define FIFO_CONFIG_CHECK1		(0x00)
#define FIFO_CONFIG_CHECK2		(0x80)
/**************************************************/
/**\name	MAG SENSOR SELECT          */
/*************************************************/
#define BST_BMM		(0)
#define BST_AKM		(1)
#define BMA4XY_YAS537_I2C_ADDRESS	(0x2E)

/**************************************************/
/**\name	ACCEL ODR          */
/*************************************************/
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_RESERVED       (0x00)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_0_78HZ         (0x01)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_1_56HZ         (0x02)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_3_12HZ         (0x03)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_6_25HZ         (0x04)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_12_5HZ         (0x05)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_25HZ           (0x06)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_50HZ           (0x07)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_100HZ          (0x08)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_200HZ          (0x09)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_400HZ          (0x0A)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_800HZ          (0x0B)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_1600HZ         (0x0C)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_RESERVED0      (0x0D)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_RESERVED1      (0x0E)
#define BMA4XY_ACCEL_OUTPUT_DATA_RATE_RESERVED2      (0x0F)
/**************************************************/
/**\name	ACCEL BANDWIDTH PARAMETER         */
/*************************************************/
#define BMA4XY_ACCEL_OSR4_AVG1			(0)
#define BMA4XY_ACCEL_OSR2_AVG2			(1)
#define BMA4XY_ACCEL_NORMAL_AVG4		(2)
#define BMA4XY_ACCEL_CIC_AVG8			(3)
#define BMA4XY_ACCEL_RES_AVG2			(4)
#define BMA4XY_ACCEL_RES_AVG4			(5)
#define BMA4XY_ACCEL_RES_AVG8			(6)
#define BMA4XY_ACCEL_RES_AVG16			(7)
#define BMA4XY_ACCEL_RES_AVG32			(8)
#define BMA4XY_ACCEL_RES_AVG64			(9)
#define BMA4XY_ACCEL_RES_AVG128			(10)

#define BMA4XY_US_DISABLE				(0)
#define BMA4XY_US_ENABLE				(1)

/**************************************************/
/**\name	MAG ODR         */
/*************************************************/
#define BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED       (0x00)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_0_78HZ         (0x01)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_1_56HZ         (0x02)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_3_12HZ         (0x03)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_6_25HZ         (0x04)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_12_5HZ         (0x05)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_25HZ           (0x06)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_50HZ           (0x07)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_100HZ          (0x08)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_200HZ          (0x09)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_400HZ          (0x0A)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_800HZ          (0x0B)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_1600HZ         (0x0C)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED0      (0x0D)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED1      (0x0E)
#define BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED2      (0x0F)

/**************************************************/
/**\name	ENABLE/DISABLE SELECTIONS        */
/*************************************************/

/* Enable accel offset */
#define ACCEL_OFFSET_ENABLE		(0x01)
#define IDEAL_VALUE_X			0
#define IDEAL_VALUE_Y			0
#define BMA4XY_POSITIVE_1G_POSITION  (1)
#define	BMA4XY_NEGATIVE_1G_POSITION	 (-1)
#define BMA4XY_X_AXIS							0
#define BMA4XY_Y_AXIS							1
#define BMA4XY_Z_AXIS							2
#define IDEAL_VALUE_X_0G		0
#define IDEAL_VALUE_Y_0G		0
#define IDEAL_VALUE_Z_0G		0

/* FOC axis selection for accel*/
#define	FOC_X_AXIS		(0)
#define	FOC_Y_AXIS		(1)
#define	FOC_Z_AXIS		(2)

/* Self Test */
#define BMA4XY_SELFTEST_PASS			(0)
#define BMA4XY_SELFTEST_FAIL			(1)

/* IN OUT CONTROL */
#define BMA4XY_INTR1_EDGE_CTRL			(0)
#define BMA4XY_INTR2_EDGE_CTRL			(1)
#define BMA4XY_INTR1_LEVEL				(0)
#define BMA4XY_INTR2_LEVEL				(1)
#define BMA4XY_INTR1_OUTPUT_TYPE		(0)
#define BMA4XY_INTR2_OUTPUT_TYPE		(1)
#define BMA4XY_INTR1_OUTPUT_ENABLE		(0)
#define BMA4XY_INTR2_OUTPUT_ENABLE		(1)
#define BMA4XY_INTR1_INPUT_ENABLE	(0)
#define BMA4XY_INTR2_INPUT_ENABLE	(1)

/* GPIO REGISTERS */
#define BMA4XY_GPIO_0_REG				(0x1E)
#define BMA4XY_GPIO_1_REG				(0x1F)
#define BMA4XY_GPIO_2_REG				(0x20)
#define BMA4XY_GPIO_3_REG				(0x21)
#define BMA4XY_GPIO_4_REG				(0x27)
#define BMA4XY_GPIO_5_REG				(0x28)
#define BMA4XY_GPIO_6_REG				(0x29)
#define BMA4XY_GPIO_7_REG				(0x2A)
#define BMA4XY_STEP_COUNTER_DATA_SIZE	(0x04)

/*  INTERRUPT MAPS    */
#define BMA4XY_INTR1_MAP				(0)
#define BMA4XY_INTR2_MAP				(1)

/**************************************************/
/**\name	BMM150 TRIM DATA DEFINITIONS      */
/*************************************************/
#define BMA4XY_MAG_DIG_X1                      (0x5D)
#define BMA4XY_MAG_DIG_Y1                      (0x5E)
#define BMA4XY_MAG_DIG_Z4_LSB                  (0x62)
#define BMA4XY_MAG_DIG_Z4_MSB                  (0x63)
#define BMA4XY_MAG_DIG_X2                      (0x64)
#define BMA4XY_MAG_DIG_Y2                      (0x65)
#define BMA4XY_MAG_DIG_Z2_LSB                  (0x68)
#define BMA4XY_MAG_DIG_Z2_MSB                  (0x69)
#define BMA4XY_MAG_DIG_Z1_LSB                  (0x6A)
#define BMA4XY_MAG_DIG_Z1_MSB                  (0x6B)
#define BMA4XY_MAG_DIG_XYZ1_LSB                (0x6C)
#define BMA4XY_MAG_DIG_XYZ1_MSB                (0x6D)
#define BMA4XY_MAG_DIG_Z3_LSB                  (0x6E)
#define BMA4XY_MAG_DIG_Z3_MSB                  (0x6F)
#define BMA4XY_MAG_DIG_XY2                     (0x70)
#define BMA4XY_MAG_DIG_XY1                     (0x71)
/**************************************************/
/**\name	BMM150 PRE-SET MODE DEFINITIONS     */
/*************************************************/
#define BMA4XY_MAG_PRESETMODE_LOWPOWER                 (1)
#define BMA4XY_MAG_PRESETMODE_REGULAR                  (2)
#define BMA4XY_MAG_PRESETMODE_HIGHACCURACY             (3)
#define BMA4XY_MAG_PRESETMODE_ENHANCED                 (4)
/**************************************************/
/**\name	BMM150 PRESET MODES - DATA RATES    */
/*************************************************/
#define BMA4XY_MAG_LOWPOWER_DR                       (0x02)
#define BMA4XY_MAG_REGULAR_DR                        (0x02)
#define BMA4XY_MAG_HIGHACCURACY_DR                   (0x2A)
#define BMA4XY_MAG_ENHANCED_DR                       (0x02)
/**************************************************/
/**\name	BMM150 PRESET MODES - REPETITIONS-XY RATES */
/*************************************************/
#define BMA4XY_MAG_LOWPOWER_REPXY                    (1)
#define BMA4XY_MAG_REGULAR_REPXY                     (4)
#define BMA4XY_MAG_HIGHACCURACY_REPXY                (23)
#define BMA4XY_MAG_ENHANCED_REPXY                    (7)
/**************************************************/
/**\name	BMM150 PRESET MODES - REPETITIONS-Z RATES */
/*************************************************/
#define BMA4XY_MAG_LOWPOWER_REPZ                     (2)
#define BMA4XY_MAG_REGULAR_REPZ                      (14)
#define BMA4XY_MAG_HIGHACCURACY_REPZ                 (82)
#define BMA4XY_MAG_ENHANCED_REPZ                     (26)
#define BMA4XY_MAG_NOAMRL_SWITCH_TIMES               (5)
#define MAG_INTERFACE_PMU_ENABLE                     (1)
#define MAG_INTERFACE_PMU_DISABLE                    (0)
/**************************************************/
/**\name	USED FOR MAG OVERFLOW CHECK FOR BMM150  */
/*************************************************/
#define BMA4XY_MAG_OVERFLOW_OUTPUT			((s16)-32768)
#define BMA4XY_MAG_OVERFLOW_OUTPUT_S32		((s32)(-2147483647-1))
#define BMA4XY_MAG_NEGATIVE_SATURATION_Z   ((s16)-32767)
#define BMA4XY_MAG_POSITIVE_SATURATION_Z   ((u16)32767)
#define BMA4XY_MAG_FLIP_OVERFLOW_ADCVAL		((s16)-4096)
#define BMA4XY_MAG_HALL_OVERFLOW_ADCVAL		((s16)-16384)
/**************************************************/
/**\name	BMM150 REGISTER DEFINITION */
/*************************************************/
#define BMA4XY_BMM150_CHIP_ID           (0x40)
#define BMA4XY_BMM150_POWER_CONTROL_REG	(0x4B)
#define BMA4XY_BMM150_POWER_MODE_REG		(0x4C)
#define BMA4XY_BMM150_DATA_REG			(0x42)
#define BMA4XY_BMM150_XY_REP			(0x51)
#define BMA4XY_BMM150_Z_REP				(0x52)
/**************************************************/
/**\name	AKM COMPENSATING DATA REGISTERS     */
/*************************************************/
#define BMA4XY_BST_AKM_ASAX		(0x60)
#define BMA4XY_BST_AKM_ASAY		(0x61)
#define BMA4XY_BST_AKM_ASAZ		(0x62)
/**************************************************/
/**\name	AKM POWER MODE SELECTION     */
/*************************************************/
#define AKM_POWER_DOWN_MODE			(0)
#define AKM_SINGLE_MEAS_MODE		(1)
#define FUSE_ROM_MODE				(2)
/**************************************************/
/**\name	SECONDARY_MAG POWER MODE SELECTION    */
/*************************************************/
#define BMA4XY_MAG_FORCE_MODE		(0)
#define BMA4XY_MAG_SUSPEND_MODE		(1)
/**************************************************/
/**\name	MAG POWER MODE SELECTION    */
/*************************************************/
#define	FORCE_MODE		(0)
#define	SUSPEND_MODE	(1)
#define	NORMAL_MODE		(2)
#define MAG_SUSPEND_MODE (1)
/**************************************************/
/**\name	FIFO CONFIGURATIONS    */
/*************************************************/
#define FIFO_HEADER_ENABLE			(0x01)
#define FIFO_MAG_ENABLE				(0x01)
#define FIFO_ACCEL_ENABLE			(0x01)
#define FIFO_ACCEL_DISABLE			(0x00)
#define FIFO_ACCEL_FILTER_ENABLE	(0x01)
#define FIFO_ACCEL_FILTER_DISABLE	(0x00)

#define FIFO_TIME_ENABLE			(0x01)
#define FIFO_TIME_DISABLE			(0x00)
#define FIFO_STOPONFULL_ENABLE		(0x01)
#define FIFO_WM_INTERRUPT_ENABLE	(0x01)
#define	BMA4XY_FIFO_INDEX_LENGTH	(1)
#define	BMA4XY_FIFO_TAG_INTR_MASK	(0xFC)

/* FIFO definitions*/
#define FIFO_HEAD_A        (0x84)
#define FIFO_HEAD_G        (0x88)
#define FIFO_HEAD_M        (0x90)
#define FIFO_HEAD_G_A	(0x8C)
#define FIFO_HEAD_M_A   (0x94)
#define FIFO_HEAD_M_G   (0x98)
#define FIFO_HEAD_M_G_A		(0x9C)

#define FIFO_HEAD_SENSOR_TIME			(0x44)
#define FIFO_HEAD_INPUT_CONFIG			(0x48)
#define FIFO_HEAD_SKIP_FRAME			(0x40)
#define FIFO_HEAD_OVER_READ_LSB			(0x80)
#define FIFO_HEAD_OVER_READ_MSB			(0x00)


/* FIFO 1024 byte, max fifo frame count not over 150 */
#define	FIFO_INPUT_CONFIG_OVER_LEN  ((s8)-11)
#define	FIFO_OVER_READ_RETURN		((s8)-10)
#define	FIFO_SENSORTIME_RETURN		((s8)-9)
#define	FIFO_SKIP_OVER_LEN			((s8)-8)
#define	FIFO_M_G_A_OVER_LEN			((s8)-7)
#define	FIFO_M_G_OVER_LEN			((s8)-6)
#define	FIFO_M_A_OVER_LEN			((s8)-5)
#define	FIFO_G_A_OVER_LEN			((s8)-4)
#define	FIFO_M_OVER_LEN				((s8)-3)
#define	FIFO_G_OVER_LEN				((s8)-2)
#define	FIFO_A_OVER_LEN				((s8)-1)
/**************************************************/
/**\name	ACCEL POWER MODE    */
/*************************************************/
#define ACCEL_MODE_NORMAL	(0x11)
#define	ACCEL_LOWPOWER		(0X12)
#define	ACCEL_SUSPEND		(0X10)

/**************************************************/
/**\name	MAG POWER MODE    */
/*************************************************/
#define MAG_MODE_SUSPEND	(0x18)
#define MAG_MODE_NORMAL		(0x19)
#define MAG_MODE_LOWPOWER	(0x1A)
/**************************************************/
/**\name	ENABLE/DISABLE BIT VALUES    */
/*************************************************/
#define BMA4XY_ENABLE	(0x01)
#define BMA4XY_DISABLE	(0x00)
/**************************************************/
/**\name	INTERRUPT EDGE TRIGGER ENABLE    */
/*************************************************/
#define BMA4XY_EDGE		(0x01)
#define BMA4XY_LEVEL	(0x00)

/* interrupt output enable*/
#define BMA4XY_INPUT	(0x01)
#define BMA4XY_OUTPUT	(0x00)
#define BMA4XY_TRUE			(0x01)
#define BMA4XY_FALSE		(0x00)

/**************************************************/
/**\name	DEFINITION USED FOR DIFFERENT WRITE   */
/*************************************************/
#define	BMA4XY_WRITE_TARGET_PAGE0	(0x00)
#define	BMA4XY_WRITE_TARGET_PAGE1	(0x01)
#define	BMA4XY_WRITE_ENABLE_PAGE1	(0x01)
#define	BMA4XY_MANUAL_DISABLE	    (0x00)
#define	BMA4XY_MANUAL_ENABLE	    (0x01)
#define	BMA4XY_ENABLE_MAG_IF_MODE	(0x01)
#define	BMA4XY_MAG_DATA_READ_REG        (0x0A)
#define BMA4XY_BMM_POWER_MODE_REG		(0x06)
#define BMA4XY_PULL_UP_DATA             (0x03)
#define BMA4XY_FIFO_M_G_A_ENABLE        (0xE0)
#define BMA4XY_FIFO_M_G_ENABLE          (0xA0)
#define BMA4XY_FIFO_M_A_ENABLE          (0x60)
#define BMA4XY_FIFO_G_A_ENABLE          (0xC0)
#define BMA4XY_FIFO_A_ENABLE            (0x40)
#define BMA4XY_FIFO_M_ENABLE            (0x20)

#define BMA4XY_SEC_IF_BMM150	(0)
#define BMA4XY_SEC_IF_AKM09916	(1)

/**************************************************/
/**\name	MAG INIT DEFINITION  */
/*************************************************/
#define BMA4XY_COMMAND_REG_ONE		(0x37)
#define BMA4XY_COMMAND_REG_TWO		(0x9A)
#define BMA4XY_COMMAND_REG_THREE	(0xC0)
/**************************************************/
/**\name	BIT SLICE GET AND SET FUNCTIONS  */
/*************************************************/
#define BMA4XY_GET_BITSLICE(regvar, bitname)\
		((regvar & bitname##_MSK) >> bitname##_POS)
#define BMA4XY_SET_BITSLICE(regvar, bitname, val)\
		((regvar & ~bitname##_MSK) | \
		((val<<bitname##_POS)&bitname##_MSK))
#define BMA4XY_GET_DIFF(x, y) ((x) - (y))
/**************************************************/
/**\name	 FUNCTION DECLARATIONS  */
/*************************************************/

BMA4XY_RETURN_TYPE bma4xy_init(struct bma4xy_t *bma4xy);

BMA4XY_RETURN_TYPE bma4xy_config_stream_data(s8 *download_rslt);

BMA4XY_RETURN_TYPE bma4xy_get_config_stream_header(
		struct bma4xy_config_stream_header *config_stream_header);

BMA4XY_RETURN_TYPE bma4xy_write_reg(u8 addr_u8, u8 *data_u8, u8 len_u8);

BMA4XY_RETURN_TYPE bma4xy_read_reg(u8 addr_u8, u8 *data_u8, u8 len_u8);

BMA4XY_RETURN_TYPE bma4xy_get_error_status(struct bma4xy_err_reg_t  *err_reg);

BMA4XY_RETURN_TYPE bma4xy_get_status(struct bma4xy_status_t *status);

BMA4XY_RETURN_TYPE bma4xy_set_mag_interface_normal(void);

BMA4XY_RETURN_TYPE bma4xy_read_mag_xyz(struct bma4xy_mag_t *mag,
					u8 sensor_select_u8);

BMA4XY_RETURN_TYPE bma4xy_read_mag_xyzr(struct bma4xy_mag_xyzr_t *mag);

BMA4XY_RETURN_TYPE bma4xy_read_accel_xyz(struct bma4xy_accel_t *accel);

BMA4XY_RETURN_TYPE bma4xy_get_sensor_time(u32 *sensor_time_u32);

BMA4XY_RETURN_TYPE bma4xy_get_accel_data_rdy(u8 *drdy_acc);

BMA4XY_RETURN_TYPE bma4xy_get_mag_data_rdy(u8 *drdy_acc);

BMA4XY_RETURN_TYPE bma4xy_get_stat1_fifo_full_intr(u8 *fifo_full_intr_u8);

BMA4XY_RETURN_TYPE bma4xy_get_stat1_fifo_wm_intr(u8 *fifo_wm_intr_u8);

BMA4XY_RETURN_TYPE bma4xy_get_temperature(u16 *temp_s16);

BMA4XY_RETURN_TYPE bma4xy_read_interrupt_status(
					struct bma4xy_int_status *int_status);

BMA4XY_RETURN_TYPE bma4xy_get_accel_conf(struct bma4xy_acc_conf *acc_conf);

BMA4XY_RETURN_TYPE bma4xy_fifo_length(u16 *fifo_length_u16);

BMA4XY_RETURN_TYPE bma4xy_read_fifo_data(u8 *fifodata_u8, u16 fifo_length_u16);

BMA4XY_RETURN_TYPE bma4xy_get_accel_output_data_rate(u8 *output_data_rate_u8);

BMA4XY_RETURN_TYPE bma4xy_set_accel_output_data_rate(u8 output_data_rate_u8,
						u8 accel_bw_u8);

BMA4XY_RETURN_TYPE bma4xy_get_accel_bw(u8 *bw_u8);

BMA4XY_RETURN_TYPE bma4xy_set_accel_bw(u8 bw_u8);

BMA4XY_RETURN_TYPE bma4xy_get_accel_under_sampling_parameter(
						u8 *accel_under_sampling_u8);

BMA4XY_RETURN_TYPE bma4xy_set_accel_under_sampling_parameter(
						u8 accel_under_sampling_u8);

BMA4XY_RETURN_TYPE bma4xy_get_accel_range(u8 *range_u8);

BMA4XY_RETURN_TYPE bma4xy_set_accel_range(u8 range_u8);

BMA4XY_RETURN_TYPE bma4xy_get_mag_output_data_rate(u8 *output_data_rate_u8);

BMA4XY_RETURN_TYPE bma4xy_set_mag_output_data_rate(u8 odr);

BMA4XY_RETURN_TYPE bma4xy_set_mag_offset(u8 offset_u8);

BMA4XY_RETURN_TYPE bma4xy_get_mag_offset(u8 *offset_u8);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_down_accel(u8 *fifo_down_u8);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_down_accel(u8 fifo_down_u8);

BMA4XY_RETURN_TYPE bma4xy_get_accel_fifo_filter_data(u8 *accel_fifo_filter_u8);

BMA4XY_RETURN_TYPE bma4xy_set_accel_fifo_filter_data(u8 accel_fifo_filter_u8);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_wm(u16 *fifo_wm_u16);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_wm(u16 fifo_wm_u16);

BMA4XY_RETURN_TYPE bma4xy_setup_fifo(enum bma4xy_fifo_setup sensor_type,
				u8 header_enable);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_stop_on_full(u8 *fifo_stop_on_full_u8);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_stop_on_full(u8 fifo_stop_on_full_u8);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_time_enable(u8 *fifo_time_enable_u8);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_time_enable(u8 fifo_time_enable_u8);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_tag_intr2_enable(u8 *fifo_tag_intr2_u8);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_tag_intr2_enable(u8 fifo_tag_intr2_u8);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_tag_intr1_enable(u8 *fifo_tag_intr1_u8);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_tag_intr1_enable(u8 fifo_tag_intr1_u8);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_header_enable(u8 *fifo_header_u8);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_header_enable(u8 fifo_header_u8);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_mag_enable(u8 *fifo_mag_u8);

BMA4XY_RETURN_TYPE bma4xy_set_mag_manual_enable(u8 mag_manual_u8);

BMA4XY_RETURN_TYPE bma4xy_get_mag_read_addr(u8 *mag_read_addr_u8);

BMA4XY_RETURN_TYPE bma4xy_get_mag_write_addr(u8 *mag_write_addr_u8);

BMA4XY_RETURN_TYPE bma4xy_set_mag_write_addr(u8 mag_write_addr_u8);

BMA4XY_RETURN_TYPE bma4xy_get_mag_write_data(u8 *mag_write_data_u8);

BMA4XY_RETURN_TYPE bma4xy_set_mag_write_data(u8 mag_write_data_u8);

BMA4XY_RETURN_TYPE bma4xy_set_mag_read_addr(u8 mag_read_addr_u8);

BMA4XY_RETURN_TYPE bma4xy_set_aux_if_mode(u8 if_mode_u8);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_mag_enable(u8 fifo_mag_u8);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_accel_enable(u8 *fifo_accel_u8);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_accel_enable(u8 fifo_accel_u8);

BMA4XY_RETURN_TYPE bma4xy_get_uc_config(struct bma4xy_uc_config *uc_config);

BMA4XY_RETURN_TYPE bma4xy_set_uc_config(struct bma4xy_uc_config uc_config);

BMA4XY_RETURN_TYPE bma4xy_get_i2c_device_addr(u8 *i2c_device_addr_u8);

BMA4XY_RETURN_TYPE bma4xy_set_i2c_device_addr(u8 i2c_device_addr_u8);

BMA4XY_RETURN_TYPE bma4xy_get_uc_status(struct bma4xy_uc_status *uc_status);

BMA4XY_RETURN_TYPE bma4xy_get_output_enable(u8 channel_u8,
					u8 *output_enable_u8);

BMA4XY_RETURN_TYPE bma4xy_set_output_enable(u8 channel_u8, u8 output_enable_u8);

BMA4XY_RETURN_TYPE bma4xy_set_interrupt_trigger(u8 channel_u8, u8 intr_trigger);

BMA4XY_RETURN_TYPE bma4xy_get_interrupt_trigger(u8 channel_u8,
						u8 *intr_trigger);

BMA4XY_RETURN_TYPE bma4xy_set_interrupt_output(u8 channel_u8, u8 intr_output);

BMA4XY_RETURN_TYPE bma4xy_get_interrupt_output(u8 channel_u8, u8 *intr_output);

BMA4XY_RETURN_TYPE bma4xy_get_interrupt_level(u8 channel_u8, u8 *intr_level);

BMA4XY_RETURN_TYPE bma4xy_set_interrupt_level(u8 channel_u8, u8 intr_level);

BMA4XY_RETURN_TYPE bma4xy_get_input_enable(u8 channel_u8, u8 *input_en_u8);

BMA4XY_RETURN_TYPE bma4xy_set_input_enable(u8 channel_u8, u8 input_en_u8);

BMA4XY_RETURN_TYPE bma4xy_get_intr_fifo_full(u8 channel_u8,
					u8 *intr_fifo_full_u8);

BMA4XY_RETURN_TYPE bma4xy_set_intr_fifo_full(u8 channel_u8,
					u8 intr_fifo_full_u8);

BMA4XY_RETURN_TYPE bma4xy_get_intr_fifo_wm(u8 channel_u8, u8 *intr_fifo_wm_u8);

BMA4XY_RETURN_TYPE bma4xy_set_intr_fifo_wm(u8 channel_u8, u8 intr_fifo_wm_u8);

BMA4XY_RETURN_TYPE bma4xy_get_intr_data_rdy(u8 channel_u8,
					u8 *intr_data_rdy_u8);

BMA4XY_RETURN_TYPE bma4xy_set_intr_data_rdy(u8 channel_u8, u8 intr_data_rdy_u8);

BMA4XY_RETURN_TYPE bma4xy_get_foc_accel_z(u8 *foc_accel_z_u8);

BMA4XY_RETURN_TYPE bma4xy_set_foc_accel_z(u8 foc_accel_z_u8);

BMA4XY_RETURN_TYPE bma4xy_get_foc_accel_y(u8 *foc_accel_y_u8);

BMA4XY_RETURN_TYPE bma4xy_set_foc_accel_y(u8 foc_accel_y_u8);

BMA4XY_RETURN_TYPE bma4xy_get_foc_accel_x(u8 *foc_accel_x_u8);

BMA4XY_RETURN_TYPE bma4xy_set_foc_accel_x(u8 foc_accel_x_u8);

BMA4XY_RETURN_TYPE bma4xy_get_spi3(u8 *spi3_u8);

BMA4XY_RETURN_TYPE bma4xy_set_spi3(u8 spi3_u8);

BMA4XY_RETURN_TYPE bma4xy_get_if_mode(u8 *if_mode_u8);

BMA4XY_RETURN_TYPE bma4xy_set_if_mode(u8 if_mode_u8);

BMA4XY_RETURN_TYPE bma4xy_set_command_register(u8 command_reg_u8);

BMA4XY_RETURN_TYPE bma4xy_get_target_page(u8 *target_page_u8);

BMA4XY_RETURN_TYPE bma4xy_set_target_page(u8 target_page_u8);

BMA4XY_RETURN_TYPE bma4xy_get_paging_enable(u8 *page_enable_u8);

BMA4XY_RETURN_TYPE bma4xy_set_paging_enable(u8 page_enable_u8);

BMA4XY_RETURN_TYPE bma4xy_get_pullup_configuration(u8 *control_pullup_u8);

BMA4XY_RETURN_TYPE bma4xy_set_pullup_configuration(u8 control_pullup_u8);

BMA4XY_RETURN_TYPE bma4xy_set_offset_comp(u8 offset_en);

BMA4XY_RETURN_TYPE bma4xy_get_offset_comp(u8 *offset_en);

BMA4XY_RETURN_TYPE bma4xy_set_fifo_self_wakeup(u8 fifo_self_wakeup);

BMA4XY_RETURN_TYPE bma4xy_get_fifo_self_wakeup(u8 *fifo_self_wake_up);

BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_interface_init(u8 *chip_id_u8);

BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_wakeup(void);

BMA4XY_RETURN_TYPE bma4xy_read_bmm150_mag_trim(void);

BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_compensate_xyz(
struct bma4xy_mag_xyz_s32_t *mag_comp_xyz);

s32 bma4xy_bmm150_mag_compensate_X(s16 mag_data_x_s16, u16 data_r_u16);

s32 bma4xy_bmm150_mag_compensate_Y(s16 mag_data_y_s16, u16 data_r_u16);

s32 bma4xy_bmm150_mag_compensate_Z(s16 mag_data_z_s16, u16 data_r_u16);

BMA4XY_RETURN_TYPE bma4xy_set_bmm150_mag_presetmode(u8 mode);

BMA4XY_RETURN_TYPE bma4xy_get_mag_manual_enable(u8 *mag_manual_u8);

BMA4XY_RETURN_TYPE bma4xy_set_bmm150_mag_and_secondary_if_power_mode(
						u8 mag_sec_if_pow_mode_u8);

BMA4XY_RETURN_TYPE bma4xy_switch_page(u8 page);

BMA4XY_RETURN_TYPE bma4xy_enable_sec_interface(u8 sec_interface_enable);

BMA4XY_RETURN_TYPE bma4xy_enable_sec_interface(
u8 sec_interface_enable);



BMA4XY_RETURN_TYPE bma4xy_bst_akm_mag_interface_init(
u8 akm_i2c_address_u8, u8 *akm_chip_id);



BMA4XY_RETURN_TYPE bma4xy_bst_akm_set_powermode(u8 akm_pow_mode_u8);

BMA4XY_RETURN_TYPE bma4xy_set_bst_akm_and_secondary_if_powermode(
u8 mag_sec_if_pow_mode_u8);



BMA4XY_RETURN_TYPE bma4xy_read_fifo_headerless_mode(u8 mag_interface_u8);

BMA4XY_RETURN_TYPE
bma4xy_read_fifo_headerless_mode_user_defined_length(u16 fifo_user_length_u16,
			struct bma4xy_fifo_data_header_less_t *fifo_data,
			u8 mag_if_mag_u8);

BMA4XY_RETURN_TYPE bma4xy_read_fifo_header_data(u8 mag_if_u8,
			struct bma4xy_fifo_data_header_t *header_data);

BMA4XY_RETURN_TYPE bma4xy_read_fifo_header_data_user_defined_length(
			u16 fifo_user_length_u16, u8 mag_if_mag_u8,
			struct bma4xy_fifo_data_header_t *fifo_header_data);

struct bma4xy_t *bma4xy_get_ptr(void);

#endif

BMA4XY_RETURN_TYPE  bma4xy_dma_write(u8 *stream_data, u16 index);

s8 BMA4XY_SPI_burst_write(u8 reg_addr, u8 *reg_data, u8 cnt);

BMA4XY_RETURN_TYPE bma4xy_set_advance_power_save(u8 adv_pwr_save);

BMA4XY_RETURN_TYPE bma4xy_get_advance_power_save(u8 *adv_pwr_save);

BMA4XY_RETURN_TYPE bma4xy_get_accel_enable(u8 *accel_en);

BMA4XY_RETURN_TYPE bma4xy_set_accel_enable(u8 accel_en);

BMA4XY_RETURN_TYPE bma4xy_set_mag_enable(u8 mag_en);

BMA4XY_RETURN_TYPE bma4xy_get_mag_enable(u8 *mag_en);

BMA4XY_RETURN_TYPE bma4xy_get_mag_power_mode_stat(u8 *mag_power_mode_stat_u8);

BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_interface_init(u8 *chip_id_u8);

BMA4XY_RETURN_TYPE bma4xy_set_mag_burst(u8 mag_burst_u8);

BMA4XY_RETURN_TYPE bma4xy_get_mag_burst(u8 *mag_burst_u8);

BMA4XY_RETURN_TYPE bma4xy_second_if_mag_compensate_xyz(
			struct bma4xy_mag_fifo_data_t mag_fifo_data,
			u8 mag_second_if_u8,
			struct bma4xy_mag_xyz_s32_t *compensated_mag_data);

BMA4XY_RETURN_TYPE bma4xy_set_bmm150_mag_and_secondary_if_power_mode(
						u8 mag_sec_if_pow_mode_u8);

BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_set_power_mode(u8 mag_pow_mode_u8);

BMA4XY_RETURN_TYPE bma4xy_configure_accel_foc(int g_value[3]);

BMA4XY_RETURN_TYPE bma4xy_get_accel_selftest_enable(u8 *accel_selftest_axis_u8);

BMA4XY_RETURN_TYPE bma4xy_set_accel_selftest_enable(u8 accel_selftest_axis_u8);

BMA4XY_RETURN_TYPE bma4xy_get_accel_selftest_sign(u8 *accel_selftest_sign_u8);

BMA4XY_RETURN_TYPE bma4xy_set_accel_selftest_sign(u8 accel_selftest_sign_u8);

BMA4XY_RETURN_TYPE bma4xy_get_accel_selftest_amp(u8 *accel_selftest_amp_u8);

BMA4XY_RETURN_TYPE bma4xy_set_accel_selftest_amp(u8 accel_selftest_amp_u8);

BMA4XY_RETURN_TYPE bma4xy_perform_accel_selftest(u8 *result);

BMA4XY_RETURN_TYPE bma4xy_selftest_config(u8 sign);

BMA4XY_RETURN_TYPE bma4xy_set_accel_selftest_config(void);

BMA4XY_RETURN_TYPE bma4xy_validate_selftest(
				struct bma4xy_selftest_accel_t accel_data_diff);

BMA4XY_RETURN_TYPE bma4xy_set_interrupt_mode(u8 mode);

BMA4XY_RETURN_TYPE bma4xy_get_interrupt_mode(u8 *mode);

#if defined(BMA420) || defined(BMA421) || defined(BMA422) || defined(BMA455)
BMA4XY_RETURN_TYPE bma4xy_remap_axes(struct bma4xy_axes_remap *axis_remap_data);

BMA4XY_RETURN_TYPE bma4xy_anymotion_enable_axis(enum bma4xy_axis_enable axis);

BMA4XY_RETURN_TYPE bma4xy_anymotion_set_threshold(u16 anymotion_threshold);

BMA4XY_RETURN_TYPE bma4xy_anymotion_get_threshold(u16 *anymotion_threshold);

BMA4XY_RETURN_TYPE bma4xy_anymotion_set_duration(u16 anymotion_duration);

BMA4XY_RETURN_TYPE bma4xy_anymotion_get_duration(u16 *anymotion_duration);

BMA4XY_RETURN_TYPE bma4xy_anymotion_nomotion_selection(u8 selection);
#endif

#if defined(BMA422) || defined(BMA421) || defined(BMA455)
BMA4XY_RETURN_TYPE bma4xy_step_counter_enable(u8 enable);

BMA4XY_RETURN_TYPE bma4xy_step_detector_enable(u8 enable);

BMA4XY_RETURN_TYPE bma4xy_reset_step_counter(void);

BMA4XY_RETURN_TYPE bma4xy_step_counter_output(u32 *step_count);

BMA4XY_RETURN_TYPE bma4xy_step_counter_get_watermark(u16 *step_counter_wm);

BMA4XY_RETURN_TYPE bma4xy_step_counter_set_watermark(u16 step_counter_wm);

BMA4XY_RETURN_TYPE bma4xy_stepcounter_set_parameter(
				enum bma4xy_stepcounter_settings setting,
				u16 parameter);

BMA4XY_RETURN_TYPE bma4xy_stepcounter_get_parameter(
				enum bma4xy_stepcounter_settings setting,
				u16 *parameter);
#endif

#if defined(BMA420)
BMA4XY_RETURN_TYPE bma4xy_orientation_enable(u8 orientation_enable);

BMA4XY_RETURN_TYPE bma4xy_orientation_ud_enable(u8 ud_enable);

BMA4XY_RETURN_TYPE bma4xy_orientation_set_mode(u8 orientation_mode);

BMA4XY_RETURN_TYPE bma4xy_orientation_get_mode(u8 *orientation_mode);

BMA4XY_RETURN_TYPE bma4xy_orientation_set_blocking(u8 orientation_blocking);

BMA4XY_RETURN_TYPE bma4xy_orientation_get_blocking(u8 *orientation_blocking);

BMA4XY_RETURN_TYPE bma4xy_orientation_set_theta(u8 orientation_theta);

BMA4XY_RETURN_TYPE bma4xy_orientation_get_theta(u8 *orientation_theta);

BMA4XY_RETURN_TYPE bma4xy_orientation_set_hysteresis(
						u16 orientation_hysteresis);

BMA4XY_RETURN_TYPE bma4xy_orientation_get_hysteresis(
						u16 *orientation_hysteresis);

BMA4XY_RETURN_TYPE bma4xy_flat_enable(u8 flat_enable);

BMA4XY_RETURN_TYPE bma4xy_flat_set_theta(u8 flat_theta);

BMA4XY_RETURN_TYPE bma4xy_flat_get_theta(u8 *flat_theta);

BMA4XY_RETURN_TYPE bma4xy_flat_set_hysteresis(u8 flat_hysteresis);

BMA4XY_RETURN_TYPE bma4xy_flat_get_hysteresis(u8 *flat_hysteresis);

BMA4XY_RETURN_TYPE bma4xy_flat_set_holdtime(u8 falt_holdtime);

BMA4XY_RETURN_TYPE bma4xy_flat_get_holdtime(u8 *flat_holdtime);

BMA4XY_RETURN_TYPE bma4xy_flat_set_blocking(u8 flat_blocking);

BMA4XY_RETURN_TYPE bma4xy_flat_get_blocking(u8 *flat_blocking);

BMA4XY_RETURN_TYPE bma4xy_tap_enable(u8 tap_enable, u8 single_tap_en);

BMA4XY_RETURN_TYPE bma4xy_high_g_set_threshold(u16 high_g_threshold);

BMA4XY_RETURN_TYPE bma4xy_high_g_get_threshold(u16 *high_g_threshold);

BMA4XY_RETURN_TYPE bma4xy_high_g_set_hysteresis(u16 high_g_hysteresis);

BMA4XY_RETURN_TYPE bma4xy_high_g_get_hysteresis(u16 *high_g_hysteresis);

BMA4XY_RETURN_TYPE bma4xy_high_g_enable_axis(enum bma4xy_axis_enable axis);

BMA4XY_RETURN_TYPE bma4xy_high_g_set_duration(u16 high_g_duration);

BMA4XY_RETURN_TYPE bma4xy_high_g_get_duration(u16 *high_g_duration);

BMA4XY_RETURN_TYPE bma4xy_tap_set_sensitivity(u8 sensitivity);

BMA4XY_RETURN_TYPE bma4xy_tap_get_sensitivity(u8 *sensitivity);

BMA4XY_RETURN_TYPE bma4xy_low_g_set_threshold(u16 low_g_threshold);

BMA4XY_RETURN_TYPE bma4xy_low_g_get_threshold(u16 *low_g_threshold);

BMA4XY_RETURN_TYPE bma4xy_low_g_set_duration(u16 duration);

BMA4XY_RETURN_TYPE bma4xy_low_g_get_duration(u16 *low_g_duration);

BMA4XY_RETURN_TYPE bma4xy_low_g_set_hysteresis(u16 hysteresis);

BMA4XY_RETURN_TYPE bma4xy_low_g_get_hysteresis(u16 *hysteresis);

BMA4XY_RETURN_TYPE bma4xy_low_g_enable(u8 enable);

BMA4XY_RETURN_TYPE bma4xy_orientation_output(u8 *orientation_output);

BMA4XY_RETURN_TYPE bma4xy_high_g_detection_output(u8 *high_g_output);
#endif

#if defined(BMA422) || defined(BMA455)
BMA4XY_RETURN_TYPE bma4xy_tilt_get_threshold(u8 *threshold);

BMA4XY_RETURN_TYPE bma4xy_significant_motion_enable(u8 enable);

BMA4XY_RETURN_TYPE bma4xy_significant_motion_set_threshold(u16 threshold_time);

BMA4XY_RETURN_TYPE bma4xy_significant_motion_get_threshold(u16 *threshold_time);

BMA4XY_RETURN_TYPE bma4xy_significant_motion_set_prooftime(u8 proof_time);

BMA4XY_RETURN_TYPE bma4xy_significant_motion_get_prooftime(u8 *proof_time);

BMA4XY_RETURN_TYPE bma4xy_significant_motion_set_skiptime(u16 skip_time);

BMA4XY_RETURN_TYPE bma4xy_significant_motion_get_skiptime(u16 *skip_time);

BMA4XY_RETURN_TYPE bma4xy_tilt_enable(u8 enable);

BMA4XY_RETURN_TYPE bma4xy_tilt_set_threshold(u8 threshold);

BMA4XY_RETURN_TYPE bma4xy_glance_enable(u8 enable);

BMA4XY_RETURN_TYPE bma4xy_pickup_enable(u8 enable);

BMA4XY_RETURN_TYPE bma4xy_wakeup_enable(u8 enable);
#endif

BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_compensate_xyz_raw(
struct bma4xy_mag_xyz_s32_t *mag_comp_xyz, struct bma4xy_mag_xyzr_t mag_xyzr);
BMA4XY_RETURN_TYPE bma4xy_bst_akm09916_compensate_xyz(
struct bma4xy_mag_xyz_s32_t *bst_akm_xyz);

