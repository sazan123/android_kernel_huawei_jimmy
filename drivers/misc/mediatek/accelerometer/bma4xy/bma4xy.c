/*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bma4xy.c
* Date: 2016/05/18
* Revision: 1.1.0 $
*
* Usage: Sensor Driver for bma4xy sensor
*
****************************************************************************
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
/*! file <bma4xy >
    brief <Sensor driver for bma4xy> */
#include "bma4xy.h"
#include <linux/kernel.h>
/*Android Support*/

/*12bit Resolution*/
#if defined(BMA422)
#include "bma422_config.h"

/*14bit Resolution*/
#elif defined(BMA455)
#include "bma455_config.h"

/*Legacy Support*/

/*12bit Resolution*/
#elif defined(BMA420)
#include "bma420_config.h"

/*14bit Resolution*/
#elif defined(BMA456)
#include "bma456_config.h"

/*Step Counter Support*/

/*12bit Resolution*/
#elif defined(BMA421)

#include "bma421_config.h"

#endif



/* user defined code to be added here ... */
struct bma4xy_t *bma4xy_p;
/* used to read the mag trim values for compensation*/
struct trim_data_t bma4xy_mag_trim;
/* the following variable is used to avoid the selection of auto mode
when it is running in the manual mode of BMM150 mag interface*/
u8 bma4xy_V_bmm150_maual_auto_condition_u8 = BMA4XY_INIT_VALUE;
/* FIFO buffer used to store 1024 bytes of data */
#ifdef FIFO_ENABLE
u8 bma4xy_fifo_data_u8[FIFO_FRAME];
#endif
struct bma4xy_mag_fifo_data_t mag_data;


/*!
 *	@brief
 *	This function is used to read the chip id from the
 *	register 0x00 bit from 0 to 7
 *
 *	@param bma4xy : structure pointer in which all the parameters
 *	to be filled by the user except chip id.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *	@note
 *	While changing the parameter of the bma4xy_t
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
*/

BMA4XY_RETURN_TYPE bma4xy_init(struct bma4xy_t *bma4xy)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* assign bma4xy pointer */
	bma4xy_p = bma4xy;
	com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_CHIP_ID_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
	if (E_BMA4XY_COM_RES_OK == com_rslt)
		/* read Chip Id */
		bma4xy_p->chip_id = data_u8[READ_EXTRA_BYTE];

	return com_rslt;
}

/*!
 *	@brief This API is used to configure
 *	the stream data in the sensor using DMA operation.
 *
 *	@param config_rslt : pointer used to specify
 *	the result of configure operation
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_config_stream_data(s8 *config_rslt)
{
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	/* block size is 32*/
	u8 block_size = 32;
	/* disable advance power save*/
	u8 adv_power_save = 0;
	/* uc disable*/
	u8 uc_en = 0;
	u16 index;

	/* disable adv power save in 7C reg*/
	com_rslt = bma4xy_write_reg(BMA4XY_POWER_CONF_ADDR,
	&adv_power_save, BMA4XY_WRITE_LENGTH);

	if (BMA4XY_SUCCESS == com_rslt) {
		/* disable uc in 0x59 reg*/
		com_rslt += bma4xy_write_reg(BMA4XY_UC_CONF, &uc_en,
			BMA4XY_WRITE_LENGTH);

		for (index = 0; index < BMA4XY_FIRMWARE_IMAGE_SIZE;
		index += block_size)
			com_rslt += bma4xy_dma_write(
			(bma4xy_config_stream + index),
			index);
		/* enable uc and fifo mode in 0x59 reg*/
		uc_en = 0x01;
		com_rslt += bma4xy_write_reg(BMA4XY_UC_CONF, &uc_en,
					BMA4XY_WRITE_LENGTH);
		com_rslt += bma4xy_read_reg(BMA4XY_GPIO_7_REG,
					&bma4xy_p->crc_check,
					BMA4XY_READ_LENGTH);
		if (bma4xy_p->crc_check == BMA4XY_TITAN_INITIALIZED)
				*config_rslt = BMA4XY_SUCCESS;
		else
				*config_rslt = BMA4XY_FAIL;
	}
	return com_rslt;

}

/*!
 *	@brief This API is used to get the
 *  the config stream details like major version, minor version and image
 *	type.
 *
 *	@param config_stream_header : structure pointer used to store the
 *	config stream details.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_config_stream_header(
					struct bma4xy_config_stream_header
					*config_stream_header)
{

	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
						BMA4XY_UC_DMA_DATA,
						feature_config,
						BMA4XY_FEATURE_SIZE);
	if (BMA4XY_SUCCESS == com_rslt) {
		config_stream_header->minor_version =
				BMA4XY_GET_BITSLICE(feature_config[index],
					BMA4XY_CONFIG_STREAM_MINOR_VERSION);
		data_u16 = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));
		config_stream_header->major_version =
					BMA4XY_GET_BITSLICE(data_u16,
					BMA4XY_CONFIG_STREAM_MAJOR_VERSION);
		config_stream_header->image_type =
					BMA4XY_GET_BITSLICE(data_u16,
					BMA4XY_CONFIG_STREAM_IMAGE_TYPE);
	}


	return com_rslt;
}

/*!
 *	@brief This API is used to write the data using
 *	DMA.
 *
 *	@param stream_data : Pointer to store data of 32 bytes
 *	@param index : represents value in multiple of 32 bytes
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
*/
BMA4XY_RETURN_TYPE  bma4xy_dma_write(u8 *stream_data, u16 index)
{
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 dma_msb = (u8)((index & 0x1FE0) >> 5);
	u8 dma_lsb = (u8) ((index & 0x1E) >> 1);

	com_rslt = bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
						BMA4XY_UC_DMA_NEXT_LAST,
						&dma_lsb,
						BMA4XY_READ_LENGTH);
	if (BMA4XY_SUCCESS == com_rslt) {
		com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
							BMA4XY_UC_DMA_DATA_NEXT,
							&dma_msb,
							BMA4XY_READ_LENGTH);
		if (BMA4XY_SUCCESS == com_rslt)
			com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
							BMA4XY_UC_DMA_DATA,
							stream_data,
							32);
	}
	return com_rslt;
}

/*!
 *	@brief This API write the data to the given register
 *
 *	@param addr_u8 -> Address of the register
 *	@param data_u8 -> pointer of the data to be written
 *	@param len_u8 -> no of bytes to read
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_write_reg(u8 addr_u8, u8 *data_u8, u8 len_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write data from register*/
		com_rslt = bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
							addr_u8, data_u8,
							len_u8);
	}
	return com_rslt;
}

/*!
 *	@brief This API reads the data from the given register
 *
 *	@param addr_u8 -> Address of the register
 *	@param data_u8 -> The data from the register
 *	@param len_u8 -> no of bytes to read
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_read_reg(u8 addr_u8, u8 *data_u8, u8 len_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 temp[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* Read data from register*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
						addr_u8, temp,
						(len_u8+READ_EXTRA_BYTE));
		*data_u8 = temp[READ_EXTRA_BYTE];
	}
	return com_rslt;
}

/*!
 *	@brief This API reads the error status
 *	from the error register 0x02 bit 0 to 7
 *
 *  @param err_reg : bma4xy error status structure
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_error_status(struct bma4xy_err_reg_t *err_reg)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the error codes*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
			BMA4XY_ERR_STAT_REG, data_u8,
			(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt) {
			/* fatal error*/
			err_reg->fatal_err = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FATAL_ERR);
			/* cmd error*/
			err_reg->cmd_err = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_CMD_ERR);
			/* user error*/
			err_reg->err_code = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ERR_CODE);
			/* fifo error*/
			err_reg->fifo_err = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_ERR);
			/* mag data ready error*/
			err_reg->aux_err = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_AUX_ERR);
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads the interrupt status
 *	from the register 0x1B
 *
 *  @param status : pointer to interrupt status structure to hold
 *  interrupt data.
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_status(struct bma4xy_status_t *status)
{
	/* variable used to return the status of communication result*/
	s8 com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the error codes*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_STATUS_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt) {
			/* mag busy*/
			status->aux_man_op = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_AUX_MAN_OP_STATUS);
			/* command ready */
			status->cmd_rdy = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_CMD_RDY_STATUS);
			/* mag data ready*/
			status->drdy_aux = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_DRDY_AUX_STATUS);
			/* accel data ready */
			status->drdy_acc = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_DRDY_ACC_STATUS);
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads accelerometer data X,Y,Z values
 *	from the register 0x12 to 0x17
 *
 *  @param accel : Pointer which stores the value of accel data
 *
 *	@note For accel configuration use the following functions
 *	@note bma4xy_set_accel_output_data_rate()
 *	@note bma4xy_set_accel_bw()
 *	@note bma4xy_set_accel_under_sampling_parameter()
 *	@note bma4xy_set_accel_range()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_read_accel_xyz(struct bma4xy_accel_t *accel)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 index;
	/* Array contains the accel XYZ lSB and MSB data
	a_data_u8r[0] - X-LSB
	a_data_u8r[1] - X-MSB
	a_data_u8r[0] - Y-LSB
	a_data_u8r[1] - Y-MSB
	a_data_u8r[0] - Z-LSB
	a_data_u8r[1] - Z-MSB
	*/
	u8 a_data_u8r[BMA4XY_ACCEL_XYZ_DATA_SIZE+READ_EXTRA_BYTE];

	for (index = 0; index < BMA4XY_ACCEL_XYZ_DATA_SIZE+READ_EXTRA_BYTE;
	index++)
		a_data_u8r[index] = 0;

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_ACCEL_X_LSB_REG, a_data_u8r,
				(BMA4XY_ACCEL_DATA_LENGTH + READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt) {
			/* Data X */
			accel->x = ((s16)(((a_data_u8r[1+READ_EXTRA_BYTE]) << 8)
					| (a_data_u8r[0+READ_EXTRA_BYTE])));

			/* Data Y */
			accel->y = ((s16)(((a_data_u8r[3+READ_EXTRA_BYTE]) << 8)
					| (a_data_u8r[2+READ_EXTRA_BYTE])));

			/* Data Z */
			accel->z = ((s16)(((a_data_u8r[5+READ_EXTRA_BYTE]) << 8)
					| (a_data_u8r[4+READ_EXTRA_BYTE])));
			}
		}
	return com_rslt;
}

/*!
 *	@brief This API reads sensor_time from the register
 *	0x18 to 0x1A
 *
 *  @param sensor_time_u32 : The value of sensor time
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_sensor_time(u32 *sensor_time_u32)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 index;
	/* Array contains the sensor time it is 32 bit data
	a_data_u8r[0] - sensor time
	a_data_u8r[1] - sensor time
	a_data_u8r[0] - sensor time
	*/
	u8 a_data_u8r[BMA4XY_SENSOR_TIME_DATA_SIZE+READ_EXTRA_BYTE];

	for (index = 0; index < BMA4XY_SENSOR_TIME_DATA_SIZE+READ_EXTRA_BYTE;
									index++)
		a_data_u8r[index] = 0;

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_SENSOR_TIME_LSB_REG, a_data_u8r,
				(BMA4XY_SENSOR_TIME_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt)
			*sensor_time_u32 = (u32)
				((((u32)a_data_u8r[BMA4XY_SENSOR_TIME_MSB_BYTE+
				READ_EXTRA_BYTE]) << 16) |
				(((u32)a_data_u8r[BMA4XY_SENSOR_TIME_XLSB_BYTE+
				READ_EXTRA_BYTE]) << 8) |
				(a_data_u8r[BMA4XY_SENSOR_TIME_LSB_BYTE+
				READ_EXTRA_BYTE]));
		}
	return com_rslt;
}

/*!
 *	@brief This API reads the temperature of the sensor
 *	from the register 0x22 and 0x23 bit 0 to 15
 *
 *	@param temp_s16 : The value of temperature
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_temperature(u16 *temp_s16)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	/* Array contains the temperature lSB and MSB data
	data_u8[0] - LSB
	data_u8[1] - MSB*/
	u8 data_u8[BMA4XY_TEMP_DATA_SIZE+1] = {BMA4XY_INIT_VALUE,
						BMA4XY_INIT_VALUE,
						BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read temperature data */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_TEMP_LSB_VALUE_REG, data_u8,
				(BMA4XY_TEMP_DATA_SIZE+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt)
			*temp_s16 = ((s16)(((u16)((data_u8[BMA4XY_TEMP_MSB_BYTE+
				READ_EXTRA_BYTE]) << 8))|
				data_u8[BMA4XY_TEMP_LSB_BYTE+READ_EXTRA_BYTE]));
	}
	return com_rslt;
}

/*!
 *	@brief This API reads the feature interrupt status
 *	from the register 0x1C
 *
 *  @param intr_status : interrupt status structure to hold
 *  interrupt data.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_read_interrupt_status(
					struct bma4xy_int_status *int_status)
{
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 int_stat[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	u8 index = BMA4XY_INIT_VALUE;
	u8 *ptr = (u8 *)int_status;

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR_STAT_0_ADDR, int_stat,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			for (index = 0; index <= 7; index++) {
				*(ptr+index) = ((int_stat[READ_EXTRA_BYTE] &
						(0x01 << index)) ? 0x01 : 0x00);

			}

		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to get the accel
 *  configuration from the register 0x40
 *
 *  @param acc_conf : Pointer to structure to store
 *	the accel configuration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_conf(struct bma4xy_acc_conf *acc_conf)
{

	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_ACCEL_CONFIG_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt) {

			acc_conf->acc_odr =  BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_OUTPUT_DATA_RATE);
			acc_conf->acc_bwp = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_BW);
			acc_conf->acc_perf_mode = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_UNDER_SAMPLING);

			com_rslt += bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_ACCEL_CONFIG_ADDR+1, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
			if (BMA4XY_SUCCESS == com_rslt)
				acc_conf->acc_range = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_RANGE);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to get the
 *	accel output date rate from the register 0x40 bit 0 to 3
 *
 *  @param  output_data_rate_u8 :The value of accel output date rate
 *  value |  output data rate
 * -------|--------------------------
 *	 0    |	bma4xy_ACCEL_OUTPUT_DATA_RATE_RESERVED
 *	 1	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_0_78HZ
 *	 2	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_1_56HZ
 *	 3    |	bma4xy_ACCEL_OUTPUT_DATA_RATE_3_12HZ
 *	 4    | bma4xy_ACCEL_OUTPUT_DATA_RATE_6_25HZ
 *	 5	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_12_5HZ
 *	 6	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_25HZ
 *	 7	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_50HZ
 *	 8	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_100HZ
 *	 9	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_200HZ
 *	 10	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_400HZ
 *	 11	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_800HZ
 *	 12	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_1600HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_output_data_rate(u8 *output_data_rate_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the accel output data rate*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_ACCEL_OUTPUT_DATA_RATE_REG,
					data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*output_data_rate_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_OUTPUT_DATA_RATE);
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the
 *	accel output date rate from the register 0x40 bit 0 to 3
 *
 *
 *  @param  output_data_rate_u8 :The value of accel output date rate
 *  value |  output data rate
 * -------|--------------------------
 *	 0    |	bma4xy_ACCEL_OUTPUT_DATA_RATE_RESERVED
 *	 1	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_0_78HZ
 *	 2	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_1_56HZ
 *	 3    |	bma4xy_ACCEL_OUTPUT_DATA_RATE_3_12HZ
 *	 4    | bma4xy_ACCEL_OUTPUT_DATA_RATE_6_25HZ
 *	 5	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_12_5HZ
 *	 6	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_25HZ
 *	 7	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_50HZ
 *	 8	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_100HZ
 *	 9	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_200HZ
 *	 10	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_400HZ
 *	 11	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_800HZ
 *	 12	  |	bma4xy_ACCEL_OUTPUT_DATA_RATE_1600HZ
 *
 *  @param  accel_bw_u8 :The value of accel selected accel bandwidth
 *  value |  output data rate
 * -------|--------------------------
 *    0   |  bma4xy_ACCEL_OSR4_AVG1
 *    1   |  bma4xy_ACCEL_OSR2_AVG2
 *    2   |  bma4xy_ACCEL_NORMAL_AVG4
 *    3   |  bma4xy_ACCEL_CIC_AVG8
 *    4   |  bma4xy_ACCEL_RES_AVG2
 *    5   |  bma4xy_ACCEL_RES_AVG4
 *    6   |  bma4xy_ACCEL_RES_AVG8
 *    7   |  bma4xy_ACCEL_RES_AVG16
 *    8   |  bma4xy_ACCEL_RES_AVG32
 *    9   |  bma4xy_ACCEL_RES_AVG64
 *    10  |  bma4xy_ACCEL_RES_AVG128
 *
 *
 *	@note Verify the accel bandwidth before setting the
 *  output data rate
 *
 *  bandwidth  | output data rate |  under sampling
 *-------------|------------------|----------------
 *   OSR4      |  12.5 TO 1600    |   1
 *   OSR2      |  12.5 TO 1600    |   1
 *  NORMAL     |  12.5 TO 1600    |   1
 *   CIC       |  12.5 TO 1600    |   1
 *   AVG2      |  0.78 TO 400     |   0
 *   AVG4      |  0.78 TO 200     |   0
 *   AVG8      |  0.78 TO 100     |   0
 *   AVG16     |  0.78 TO 50      |   0
 *   AVG32     |  0.78 TO 25      |   0
 *   AVG64     |  0.78 TO 12.5    |   0
 *   AVG128    |  0.78 TO 6.25    |   0
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_output_data_rate(
						u8 output_data_rate_u8,
						u8 accel_bw_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	u8 odr_u8 = BMA4XY_INIT_VALUE;

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (((accel_bw_u8 > BMA4XY_ACCEL_RES_AVG2) &&
		(accel_bw_u8 <= BMA4XY_ACCEL_RES_AVG128)) ||
		(accel_bw_u8 == BMA4XY_ACCEL_RES_AVG2)) {

			/* disable the under sampling*/
			com_rslt = bma4xy_set_accel_under_sampling_parameter(
							BMA4XY_US_DISABLE);

	}  else if (((accel_bw_u8 > BMA4XY_ACCEL_OSR4_AVG1) &&
		(accel_bw_u8 <= BMA4XY_ACCEL_CIC_AVG8))
		|| (accel_bw_u8 == BMA4XY_ACCEL_OSR4_AVG1)) {

			/* enable the under sampling*/
			com_rslt = bma4xy_set_accel_under_sampling_parameter(
							BMA4XY_US_ENABLE);

	} else
		com_rslt = E_BMA4XY_OUT_OF_RANGE;

	if (BMA4XY_SUCCESS == com_rslt) {
		/* assign the output data rate*/
		switch (accel_bw_u8) {

		case BMA4XY_ACCEL_RES_AVG2:
			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_0_78HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_400HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		case BMA4XY_ACCEL_RES_AVG4:
			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_0_78HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_200HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		case BMA4XY_ACCEL_RES_AVG8:
			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_0_78HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_100HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		case BMA4XY_ACCEL_RES_AVG16:
			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_0_78HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_50HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		case BMA4XY_ACCEL_RES_AVG32:
			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_0_78HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_25HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		case BMA4XY_ACCEL_RES_AVG64:

			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_0_78HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_12_5HZ)) {
				odr_u8 = output_data_rate_u8;
			} else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			}
		break;

		case BMA4XY_ACCEL_RES_AVG128:

			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_0_78HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_6_25HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		case BMA4XY_ACCEL_OSR4_AVG1:

			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_12_5HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_1600HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		case BMA4XY_ACCEL_OSR2_AVG2:

			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_12_5HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_1600HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		case BMA4XY_ACCEL_NORMAL_AVG4:

			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_12_5HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_1600HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		case BMA4XY_ACCEL_CIC_AVG8:

			if ((output_data_rate_u8 >=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_12_5HZ) &&
			(output_data_rate_u8 <=
			BMA4XY_ACCEL_OUTPUT_DATA_RATE_1600HZ)) {
				odr_u8 = output_data_rate_u8;
			 } else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			 }
		break;

		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
	}
	if (com_rslt != E_BMA4XY_OUT_OF_RANGE) {
		/* write accel output data rate */
		com_rslt += bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_ACCEL_OUTPUT_DATA_RATE_REG,
					data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_OUTPUT_DATA_RATE,
						odr_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_ACCEL_OUTPUT_DATA_RATE_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_READ_LENGTH);
		}
	}
	}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the
 *	accel bandwidth from the register 0x40 bit 4 to 6
 *	@brief bandwidth parameter determines filter configuration(acc_us=0)
 *	and averaging for under sampling mode(acc_us=1)
 *
 *
 *  @param  bw_u8 : The value of accel bandwidth
 *
 *	@note accel bandwidth depends on under sampling parameter
 *	@note under sampling parameter can be set by the function
 *	"BMA4XY_SET_ACCEL_UNDER_SAMPLING_PARAMETER"
 *
 *	@note Filter configuration
 *  accel_us  | Filter configuration
 * -----------|---------------------
 *    0x00    |  OSR4 mode
 *    0x01    |  OSR2 mode
 *    0x02    |  normal mode
 *    0x03    |  CIC mode
 *    0x04    |  Reserved
 *    0x05    |  Reserved
 *    0x06    |  Reserved
 *    0x07    |  Reserved
 *
 *	@note accel under sampling mode
 *  accel_us  | Under sampling mode
 * -----------|---------------------
 *    0x00    |  no averaging
 *    0x01    |  average 2 samples
 *    0x02    |  average 4 samples
 *    0x03    |  average 8 samples
 *    0x04    |  average 16 samples
 *    0x05    |  average 32 samples
 *    0x06    |  average 64 samples
 *    0x07    |  average 128 samples
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_bw(u8 *bw_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the accel bandwidth */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_ACCEL_BW_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*bw_u8 = BMA4XY_GET_BITSLICE(data_u8[READ_EXTRA_BYTE],
							BMA4XY_ACCEL_BW);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the
 *	accel bandwidth from the register 0x40 bit 4 to 6
 *	@brief bandwidth parameter determines filter configuration(acc_us=0)
 *	and averaging for under sampling mode(acc_us=1)
 *
 *
 *  @param  bw_u8 : The value of accel bandwidth
 *
 *	@note accel bandwidth depends on under sampling parameter
 *	@note under sampling parameter can be set by the function
 *	"BMA4XY_SET_ACCEL_UNDER_SAMPLING_PARAMETER"
 *
 *	@note Filter configuration
 *  accel_us  | Filter configuration
 * -----------|---------------------
 *    0x00    |  OSR4 mode
 *    0x01    |  OSR2 mode
 *    0x02    |  normal mode
 *    0x03    |  CIC mode
 *    0x04    |  Reserved
 *    0x05    |  Reserved
 *    0x06    |  Reserved
 *    0x07    |  Reserved
 *
 *	@note accel under sampling mode
 *  accel_us  | Under sampling mode
 * -----------|---------------------
 *    0x00    |  no averaging
 *    0x01    |  average 2 samples
 *    0x02    |  average 4 samples
 *    0x03    |  average 8 samples
 *    0x04    |  average 16 samples
 *    0x05    |  average 32 samples
 *    0x06    |  average 64 samples
 *    0x07    |  average 128 samples
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_bw(u8 bw_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* select accel bandwidth*/
		if (bw_u8 <= BMA4XY_MAX_ACCEL_BW) {
			/* write accel bandwidth*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_ACCEL_BW_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_BW, bw_u8);
			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_ACCEL_BW_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the accel
 *	under sampling parameter form the register 0x40 bit 7
 *
 *	@param  accel_under_sampling_u8 : Pointer to store the value
 *	of accel under sampling
 *
 *	value    | under_sampling
 *	---------|---------------
 *	0x01     |  BMA4XY_ENABLE
 *	0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_under_sampling_parameter(
						u8 *accel_under_sampling_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the accel under sampling parameter */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_ACCEL_UNDER_SAMPLING_REG,
					data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*accel_under_sampling_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_UNDER_SAMPLING);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set the accel
 *	under sampling parameter form the register 0x40 bit 7
 *
 *	@param  accel_under_sampling_u8 : The value of accel under sampling
 *	value    | under_sampling
 * ----------|---------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_under_sampling_parameter(
						u8 accel_under_sampling_u8)
{
/* variable used to return the status of communication result*/
BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

/* check the bma4xy_p structure as NULL*/
if (bma4xy_p == BMA4XY_NULL) {
	com_rslt = E_BMA4XY_NULL_PTR;
} else {
	if (accel_under_sampling_u8 <= BMA4XY_MAX_UNDER_SAMPLING) {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_ACCEL_UNDER_SAMPLING_REG,
					data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
	if (BMA4XY_SUCCESS == com_rslt) {

		/* write the accel under sampling parameter */
		data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_UNDER_SAMPLING,
						accel_under_sampling_u8);
		com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
						BMA4XY_ACCEL_UNDER_SAMPLING_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}

	} else {
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	}
}
return com_rslt;
}

/*!
 *	@brief This API is used to get the ranges
 *	(g values) of the accel from the register 0x41 bit 0 to 1
 *
 *  @param range_u8 : Pointer to store the value of accel g range
 *	value	| g_range
 * ----------	|-----------
 *	0x00	| bma4xy_ACCEL_RANGE_2G
 *	0x01	| bma4xy_ACCEL_RANGE_4G
 *	0x02	| bma4xy_ACCEL_RANGE_8G
 *	0x03	| bma4xy_ACCEL_RANGE_16G
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_range(u8 *range_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the accel range*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_ACCEL_RANGE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS ==  com_rslt)
			*range_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_RANGE);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set the ranges
 *	(g values) of the accel from the register 0x41 bit 0 to 1
 *
 *  @param range_u8 : The value of accel g range
 *	value    | g_range
 * ----------|-----------
 *   0x00    | bma4xy_ACCEL_RANGE_2G
 *   0x01    | bma4xy_ACCEL_RANGE_4G
 *   0x02    | bma4xy_ACCEL_RANGE_8G
 *   0x03    | bma4xy_ACCEL_RANGE_16G
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_range(u8 range_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if ((range_u8 == BMA4XY_ACCEL_RANGE_2G) ||
		(range_u8 <= BMA4XY_ACCEL_RANGE_16G))  {
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_ACCEL_RANGE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE]  = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_RANGE,
						range_u8);
			/* write the accel range*/
			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_ACCEL_RANGE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
		} else {
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set
 *	advance power save mode from the register 0x7C bit 0
 *
 *	@param adv_pwr_save : The value of advance power save mode
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_advance_power_save(u8 adv_pwr_save)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_POWER_CONF_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ADVANCE_POWER_SAVE,
						adv_pwr_save);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_ADVANCE_POWER_SAVE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to get
 *	advance power save mode from the register 0x7C bit 0
 *
 *	@param adv_pwr_save : The value of advance power save mode
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_advance_power_save(u8 *adv_pwr_save)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_POWER_CONF_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*adv_pwr_save = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ADVANCE_POWER_SAVE);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set
 *	fifo self wake up from the register 0x7C bit 1
 *
 *  @param fifo_self_wakeup : variable used to enable or disable
 *	fifo self wake up feature
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_fifo_self_wakeup(u8 fifo_self_wakeup)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_SELF_WAKE_UP_REG,
					data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_SELF_WAKE_UP,
						fifo_self_wakeup);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_FIFO_SELF_WAKE_UP_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to get
 *	fifo self wake up from the register 0x7C bit 1
 *
 *	@param fifo_self_wake_up : Pointer used to store the
 *	fifo self wake up function in the sensor
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_fifo_self_wakeup(u8 *fifo_self_wake_up)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_SELF_WAKE_UP_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*fifo_self_wake_up = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_SELF_WAKE_UP);
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to enable or disable
 *	accel data by setting  the register 0x7D bit 2
 *
 *	@param accel_en : The value of accel enable
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_enable(u8 accel_en)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_POWER_CTRL_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_ENABLE,
						accel_en);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_ACCEL_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	}

	return com_rslt;
}
/*!
 *	@brief This API is used to enable or disable
 *	mag data by setting  the register 0x7D bit 0
 *
 *  @param mag_en : The value of mag enable
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_mag_enable(u8 mag_en)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_POWER_CTRL_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MAG_ENABLE, mag_en);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_MAG_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to get the whether mag
 *	sensor is enabled or not by reading from the
 *	register 0x7D bit 0
 *
 *	@param mag_en : pointer used to store the status whether
 *	mag sensor is enabled or not
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_mag_enable(u8 *mag_en)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_POWER_CTRL_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*mag_en = BMA4XY_GET_BITSLICE(data_u8[READ_EXTRA_BYTE],
							BMA4XY_MAG_ENABLE);

		}
	}

	return com_rslt;
}
/*!
 *	@brief This API is used to read the
 *	accel mode by getting from  the register 0x7D bit 2
 *
 *	@param accel_en : Pointer to store the  accel enable
 *	status
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_enable(u8 *accel_en)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_POWER_CTRL_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*accel_en = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_ACCEL_ENABLE);
		}
	}

	return com_rslt;
}
/*!
 *	@brief API used to get the pin output for interrupt1
 *	and interrupt2 pin from the register 0x53 and 0x54 bit 2
 *
 *  @param channel_u8: The value of output enable selection
 *   channel_u8		|   level selection
 *  ----------------|---------------
 *       0			| BMA4XY_INTR1_MAP
 *       1			| BMA4XY_INTR2_MAP
 *
 *	@param intr_output :
 *	Pointer used to get the value of interrupt output mode
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMA4XY_OPEN_DRAIN
 *  0x00     |  BMA4XY_PUSH_PULL
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_interrupt_output(u8 channel_u8, u8 *intr_output)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

	switch (channel_u8) {
	case BMA4XY_INTR1_MAP:
		/* write the output enable of interrupt1*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR1_OUTPUT_MODE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*intr_output = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_OUTPUT_MODE);
		}
	break;
	case BMA4XY_INTR2_MAP:
		/* write the output enable of interrupt2*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR2_OUTPUT_MODE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*intr_output = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_OUTPUT_MODE);

		}
	break;
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
	}
	return com_rslt;
}

/*!
 *	@brief API used to set the pin output for interrupt1
 *	and interrupt1 pin from the register 0x53 and 0x54 bit 2
 *
 *  @param channel_u8: The value of output enable selection
 *   channel_u8  |   level selection
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_output :
 *	The value of interrupt output mode
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMA4XY_OPEN_DRAIN
 *  0x00     |  BMA4XY_PUSH_PULL
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_interrupt_output(u8 channel_u8, u8 intr_output)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

	switch (channel_u8) {
	case BMA4XY_INTR1_MAP:

		/* write the output enable of interrupt1*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR1_OUTPUT_MODE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_OUTPUT_MODE,
						intr_output);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR1_OUTPUT_MODE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	break;
	case BMA4XY_INTR2_MAP:

		/* write the output enable of interrupt2*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR2_OUTPUT_MODE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_OUTPUT_MODE,
						intr_output);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR2_OUTPUT_MODE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	break;
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
	}
	return com_rslt;
}
 /*!
 *	@brief API used to get the level for interrupt1
 *	and interrupt1 pin from the register 0x53 and 0x54 bit 1
 *
 *  @param channel_u8: The value of output enable selection
 *   channel_u8  |   level selection
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_level :
 *	Pointer used to store the value of interrupt level
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMA4XY_ACTIVE_HIGH
 *  0x00     |  BMA4XY_ACTIVE_LOW
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_interrupt_level(u8 channel_u8, u8 *intr_level)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

	switch (channel_u8) {
	case BMA4XY_INTR1_MAP:

		/* write the output enable of interrupt1*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR1_LEVEL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*intr_level = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_LEVEL);

		}
	break;
	case BMA4XY_INTR2_MAP:

		/* write the output enable of interrupt2*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR2_LEVEL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt) {
			*intr_level = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_LEVEL);

		}
	break;
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
	}
	return com_rslt;
}

 /*!
 *	@brief API used to set the level for interrupt1
 *	and interrupt1 pin from the register 0x53 and 0x54 bit 1
 *
 *  @param channel_u8: The value of output enable selection
 *   channel_u8  |   level selection
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_level :
 *	The value of interrupt level
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMA4XY_ACTIVE_HIGH
 *  0x00     |  BMA4XY_ACTIVE_LOW
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_interrupt_level(u8 channel_u8, u8 intr_level)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

	switch (channel_u8) {
	case BMA4XY_INTR1_MAP:
		/* write the output enable of interrupt1*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR1_LEVEL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_LEVEL,
						intr_level);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR1_LEVEL_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	break;
	case BMA4XY_INTR2_MAP:

		/* write the output enable of interrupt2*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR2_LEVEL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_LEVEL,
						intr_level);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR2_LEVEL_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	break;
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
	}
	return com_rslt;
}

 /*!
 *	@brief API used to set the edge control for interrupt1
 *	and interrupt1 pin from the register 0x53 and 0x54 bit 0
 *
 *  @param channel_u8: The value of output enable selection
 *   channel_u8  |   level selection
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_trigger :
 *	The value of interrupt trigger
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMA4XY_EDGE_TRIGGER
 *  0x00     |  BMA4XY_LEVEL_TRIGGER
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_interrupt_trigger(u8 channel_u8, u8 intr_trigger)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

	switch (channel_u8) {
	case BMA4XY_INTR1_MAP:

		/* write the output enable of interrupt1*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR1_EDGE_CTRL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_EDGE_CTRL,
						intr_trigger);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR1_EDGE_CTRL_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	break;
	case BMA4XY_INTR2_MAP:

		/* write the output enable of interrupt2*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR2_EDGE_CTRL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_EDGE_CTRL,
						intr_trigger);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR2_EDGE_CTRL_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	break;
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
	}
	return com_rslt;
}

 /*!
 *	@brief API used to get the edge control for interrupt1
 *	and interrupt1 pin from the register 0x53 and 0x54 bit 0
 *
 *  @param channel_u8: The value of output enable selection
 *   channel_u8  |   level selection
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_trigger :
 *	Pointer used to store the value of interrupt trigger
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMA4XY_EDGE_TRIGGER
 *  0x00     |  BMA4XY_LEVEL_TRIGGER
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_interrupt_trigger(u8 channel_u8, u8 *intr_trigger)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

	switch (channel_u8) {
	case BMA4XY_INTR1_MAP:

		/* write the output enable of interrupt1*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR1_EDGE_CTRL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*intr_trigger = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_EDGE_CTRL);

		}
	break;
	case BMA4XY_INTR2_MAP:

		/* write the output enable of interrupt2*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR2_EDGE_CTRL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt) {
			*intr_trigger = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_EDGE_CTRL);
		}
	break;
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
	}
	return com_rslt;
}

 /*!
 *	@brief API used to set the Output enable for interrupt1
 *	and interrupt1 pin from the register 0x53 and 0x54 bit 3
 *
 *  @param channel_u8: The value of output enable selection
 *   channel_u8  |   level selection
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param output_enable_u8 :
 *	The value of output enable of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMA4XY_OUTPUT_ENABLE
 *  0x00     |  BMA4XY_OUTPUT_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_output_enable(u8 channel_u8, u8 output_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

	switch (channel_u8) {
	case BMA4XY_INTR1_MAP:

		/* write the output enable of interrupt1*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR1_OUTPUT_ENABLE_REG,
					data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_OUTPUT_ENABLE,
						output_enable_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR1_OUTPUT_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	break;
	case BMA4XY_INTR2_MAP:

		/* write the output enable of interrupt2*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR2_OUTPUT_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_OUTPUT_ENABLE,
						output_enable_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR2_OUTPUT_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	break;
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
	}
	return com_rslt;
}

 /*!
 *	@brief API used to get the Output enable for interrupt1
 *	and interrupt1 pin from the register 0x53 and 0x54 bit 3
 *
 *  @param channel_u8: The value of output enable selection
 *   channel_u8  |   level selection
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param output_enable_u8 :
 *	Pointer used to store the output enable status
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMA4XY_OUTPUT_ENABLE
 *  0x00     |  BMA4XY_OUTPUT_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_output_enable(u8 channel_u8, u8 *output_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

	switch (channel_u8) {
	case BMA4XY_INTR1_MAP:
		/* write the output enable of interrupt1*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR1_OUTPUT_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*output_enable_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_OUTPUT_ENABLE);
		}
	break;
	case BMA4XY_INTR2_MAP:
		/* write the output enable of interrupt2*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR2_OUTPUT_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*output_enable_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_OUTPUT_ENABLE);

		}
	break;
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
	}
	return com_rslt;
}

/*!
 *	@brief API used to get input enable for interrupt1
 *	and interrupt2 pin from the register 0x53 and 0x54
 *	bit 4
 *
 *  @param channel_u8: The value of input enable selection
 *
 *	channel_u8	|   input selection
 *	----------------|---------------
 *       0			| BMA4XY_INTR1_MAP
 *       1			| BMA4XY_INTR2_MAP
 *
 *	@param input_en_u8 :
 *	The value of input enable of interrupt enable
 *
 *	value	| Behaviour
 *	--------|-------------------
 *	0x01	|	BMA4XY_INPUT_ENABLE
 *	0x00	|	BMA4XY_OUTPUT_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_input_enable(u8 channel_u8, u8 *input_en_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		switch (channel_u8) {
		/* read input enable of interrup1 and interrupt2*/
		case BMA4XY_INTR1_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR1_INPUT_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
			if (BMA4XY_SUCCESS == com_rslt)
				*input_en_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_INPUT_ENABLE);
		break;
		case BMA4XY_INTR2_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR2_INPUT_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt)
				*input_en_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_INPUT_ENABLE);
		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}

/*!
 *	@brief API used to set input enable for interrupt1
 *	and interrupt2 pin from the register 0x53 and 0x54
 *	bit 4
 *
 *	@param channel_u8: The value of input enable selection
 *
 *	channel_u8	|   input selection
 *	----------------|-------------------
 *       0		| BMA4XY_INTR1_MAP
 *       1		| BMA4XY_INTR2_MAP
 *
 *	@param input_en_u8 : The value of input enable of interrupt enable
 *
 *	value	| Behaviour
 *	--------|-------------------
 *	0x01	|  BMA4XY_INPUT_ENABLE
 *	0x00	|  BMA4XY_OUTPUT_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_input_enable(u8 channel_u8, u8 input_en_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		switch (channel_u8) {
		/* write input enable of interrup1 and interrupt2*/
		case BMA4XY_INTR1_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR1_INPUT_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR1_INPUT_ENABLE,
						input_en_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR1_INPUT_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
		break;
		case BMA4XY_INTR2_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR2_INPUT_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR2_INPUT_ENABLE,
						input_en_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR2_INPUT_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}
return com_rslt;
}

/*!
 *	@brief Reads Data Ready interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x58 bit 2 and 6.
 *	@brief interrupt1 bit 2 in the register 0x58
 *	@brief interrupt2 bit 6 in the register 0x58
 *
 *	@param channel_u8: The value of data ready interrupt selection
 *   channel_u8  |   interrupt
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_data_rdy_u8 : The value of data ready interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_intr_data_rdy(u8 channel_u8, u8 *intr_data_rdy_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		switch (channel_u8) {
		/*Read Data Ready interrupt*/
		case BMA4XY_INTR1_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR_MAP_1_DATA_RDY_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
			if (BMA4XY_SUCCESS == com_rslt)
				*intr_data_rdy_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR_MAP_1_DATA_RDY);
		break;
		case BMA4XY_INTR2_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR_MAP_2_DATA_RDY_REG, data_u8,
					BMA4XY_READ_LENGTH+READ_EXTRA_BYTE);

			if (BMA4XY_SUCCESS == com_rslt)
				*intr_data_rdy_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR_MAP_2_DATA_RDY);
			break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief Write Data Ready interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x58 bit 2 and 6
 *	@brief interrupt1 bit 2 in the register 0x58
 *	@brief interrupt2 bit 6 in the register 0x58
 *
 *
 *	@param channel_u8: The value of data ready interrupt selection
 *   channel_u8  |   interrupt
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_data_rdy_u8 : The value of data ready interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_intr_data_rdy(
u8 channel_u8, u8 intr_data_rdy_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		switch (channel_u8) {
		/*Write Data Ready interrupt*/
		case BMA4XY_INTR1_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR_MAP_1_DATA_RDY_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR_MAP_1_DATA_RDY,
						intr_data_rdy_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_INTR_MAP_1_ADDR,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
		break;
		case BMA4XY_INTR2_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR_MAP_2_DATA_RDY_REG,
					data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR_MAP_2_DATA_RDY,
						intr_data_rdy_u8);
		}
		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
	}
}
return com_rslt;
}

/*!
 * @brief This API is used to set the microcontroller
 * configuration in address 0x59
 *
 *  @param uc_config : structure object which has the configuration
 *	settings for the microcontroller.
 *
 *	Value	|	uc_en
 * -------------|----------------------------------
 *	1	|	BMA4XY_WAKEUP_DISABLE
 *	0	|	BMA4XY_WAKEUP_ENABLE
 *	-1	|	BMA4XY_INVALID
 *
 *	Value	|	fifo_mode_en
 * -------------|-----------------------------------
 *	1	|	BMA4XY_DMA_MODE
 *	0	|	BMA4XY_FIFO_MODE
 *	-1	|	BMA4XY_INVALID
 *
 *	Value	|	mem_conf_ram1
 * -------------|-----------------------------------
 *	00	|	BMA4XY_ASSIGN_DATA_MEMORY
 *	01	|	BMA4XY_ASSIGN_PROGRAM_MEMORY
 *	10	|	BMA4XY_ASSIGN_FIFO
 *	-1	|	BMA4XY_INVALID
 *
 *	Value	|	mem_conf_ram2
 * -------------|-------------------------------------
 *	00	|	BMA4XY_ASSIGN_DATA_MEMORY
 *	01	|	BMA4XY_ASSIGN_PROGRAM_MEMORY
 *	10	|	BMA4XY_ASSIGN_FIFO
 *	-1	|	BMA4XY_INVALID
 *
 *	Value	|	mem_conf_ram3
 * -------------|-------------
 *	00	|	BMA4XY_ASSIGN_DATA_MEMORY
 *	01	|	BMA4XY_ASSIGN_PROGRAM_MEMORY
 *	10	|	BMA4XY_ASSIGN_FIFO
 *	-1	|	BMA4XY_INVALID
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMA4XY_RETURN_TYPE bma4xy_set_uc_config(
struct bma4xy_uc_config uc_config)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_CONF_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

	if (BMA4XY_SUCCESS == com_rslt) {

		if (uc_config.uc_en != BMA4XY_INVALID)
			data_u8[READ_EXTRA_BYTE] = (uc_config.uc_en & 0x01);
		if (uc_config.fifo_mode_en != BMA4XY_INVALID)
			data_u8[READ_EXTRA_BYTE] |=
						(uc_config.fifo_mode_en & 0x02);
		if (uc_config.mem_conf_ram1 != BMA4XY_INVALID)
			data_u8[READ_EXTRA_BYTE] |=
					(uc_config.mem_conf_ram1 & 0x0C);
		if (uc_config.mem_conf_ram2 != BMA4XY_INVALID)
			data_u8[READ_EXTRA_BYTE] |=
					(uc_config.mem_conf_ram2 & 0x30);
		if (uc_config.mem_conf_ram3 != BMA4XY_INVALID)
			data_u8[READ_EXTRA_BYTE] |=
					(uc_config.mem_conf_ram3 & 0xC0);

		com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
						BMA4XY_UC_CONF_ADDR,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_LENGTH);

	}

	return com_rslt;
}

/*!
 * @brief This API is used to get the micro controller
 * configuration in address 0x59
 *
 *  @param uc_config : structure pointer which is used to store the
 *	settings read.
 *
 *	Value	|	uc_en
 * -------------|------------------------------------
 *	1	|	BMA4XY_WAKEUP_DISABLE
 *	0	|	BMA4XY_WAKEUP_ENABLE
 *	-1	|	BMA4XY_INVALID
 *
 *	Value	|	fifo_mode_en
 * -------------|-------------------------------------
 *	1	|	BMA4XY_DMA_MODE
 *	0	|	BMA4XY_FIFO_MODE
 *	-1	|	BMA4XY_INVALID
 *
 *	Value	|	mem_conf_ram1
 * -------------|-------------------------------------
 *	00	|	BMA4XY_ASSIGN_DATA_MEMORY
 *	01	|	BMA4XY_ASSIGN_PROGRAM_MEMORY
 *	10	|	BMA4XY_ASSIGN_FIFO
 *	-1	|	BMA4XY_INVALID
 *
 *	Value	|	mem_conf_ram2
 * -------------|--------------------------------------
 *	00	|	BMA4XY_ASSIGN_DATA_MEMORY
 *	01	|	BMA4XY_ASSIGN_PROGRAM_MEMORY
 *	10	|	BMA4XY_ASSIGN_FIFO
 *	-1	|	BMA4XY_INVALID
 *
 *	Value	|	mem_conf_ram3
 * -------------|---------------------------------------
 *	00	|	BMA4XY_ASSIGN_DATA_MEMORY
 *	01	|	BMA4XY_ASSIGN_PROGRAM_MEMORY
 *	10	|	BMA4XY_ASSIGN_FIFO
 *	-1	|	BMA4XY_INVALID
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMA4XY_RETURN_TYPE bma4xy_get_uc_config(
struct bma4xy_uc_config *uc_config)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_CONF_ADDR, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

	if (BMA4XY_SUCCESS == com_rslt) {
		uc_config->uc_en = BMA4XY_GET_BITSLICE(data_u8[READ_EXTRA_BYTE],
							BMA4XY_UC_EN);

		uc_config->fifo_mode_en = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_MODE_EN);

		uc_config->mem_conf_ram1 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MEM_CONF_RAM1);

		uc_config->mem_conf_ram2 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MEM_CONF_RAM2);

		uc_config->mem_conf_ram3 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MEM_CONF_RAM3);

	}

	return com_rslt;
}

/*!
 * @brief This API read to configure SPI
 * Interface Mode for primary and OIS interface
 * from the register 0x6B bit 0
 *
 *  @param spi3_u8 : The value of SPI mode selection
 *  Value  |  Description
 * --------|-------------
 *   0     |  SPI 4-wire mode
 *   1     |  SPI 3-wire mode
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMA4XY_RETURN_TYPE bma4xy_get_spi3(
u8 *spi3_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt  = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read SPI mode*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_CONFIG_SPI3_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*spi3_u8 = BMA4XY_GET_BITSLICE(data_u8[READ_EXTRA_BYTE],
							BMA4XY_CONFIG_SPI3);
	}
	return com_rslt;
}

/*!
 * @brief This API write to configure SPI
 * Interface Mode for primary and OIS interface
 * from the register 0x6B bit 0
 *
 *  @param spi3_u8 : The value of SPI mode selection
 *  Value  |  Description
 * --------|-------------
 *   0     |  SPI 4-wire mode
 *   1     |  SPI 3-wire mode
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMA4XY_RETURN_TYPE bma4xy_set_spi3(
u8 spi3_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (spi3_u8 <= BMA4XY_MAX_VALUE_SPI3) {
			/* write SPI mode*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_CONFIG_SPI3_REG,	data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_CONFIG_SPI3, spi3_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_CONFIG_SPI3_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
			}
		} else {
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

 /*!
 *	@brief This API read target page from the register 0x7F bit 4 and 5
 *
 *  @param target_page_u8: The value of target page
 *  value   |  page
 * ---------|-----------
 *   0      |  User data/configure page
 *   1      |  Chip level trim/test page
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_target_page(u8 *target_page_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt  = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
			/* read the page*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_CMD_TARGET_PAGE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*target_page_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_CMD_TARGET_PAGE);
		}
	return com_rslt;
}

 /*!
 *	@brief This API writes target page from the register 0x7F bit 4 and 5
 *
 *  @param target_page_u8: The value of target page
 *  value   |  page
 * ---------|-----------
 *   0      |  User data/configure page
 *   1      |  Chip level trim/test page
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_target_page(u8 target_page_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (target_page_u8 <= BMA4XY_MAX_TARGET_PAGE) {

			/* write the page*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_CMD_TARGET_PAGE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_CMD_TARGET_PAGE,
						target_page_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_CMD_TARGET_PAGE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
 /*!
 *	@brief This API read page enable from the register 0x7F bit 7
 *
 *
 *
 *  @param page_enable_u8: The value of page enable
 *  value   |  page
 * ---------|-----------
 *   0      |  BMA4XY_DISABLE
 *   1      |  BMA4XY_ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_paging_enable(u8 *page_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt  = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the page enable */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_CMD_PAGING_EN_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*page_enable_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_CMD_PAGING_EN);
		}
	return com_rslt;
}

 /*!
 *	@brief This API write page enable from the register 0x7F bit 7
 *
 *  @param page_enable_u8: The value of page enable
 *  value   |  page
 * ---------|----------------
 *   0      |  BMA4XY_DISABLE
 *   1      |  BMA4XY_ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_paging_enable(
u8 page_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (page_enable_u8 <= BMA4XY_MAX_VALUE_PAGE) {
			/* write the page enable */
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_CMD_PAGING_EN_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_CMD_PAGING_EN, page_enable_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_CMD_PAGING_EN_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

 /*!
 *	@brief This API writes value to the register 0x7E bit 0 to 7
 *  @param  command_reg_u8 : The value to write command register
 *  value   |  Description
 * ---------|--------------------------------------------------------
 *	0xB6	|	Triggers a reset
 *	0x37	|	See extmode_en_last
 *	0x9A	|	See extmode_en_last
 *	0xC0	|	Enable the extended mode
 *  0xC4	|	Erase NVM cell
 *	0xC8	|	Load NVM cell
 *	0xF0	|	Reset acceleration data path
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_command_register(u8 command_reg_u8)
{
	BMA4XY_RETURN_TYPE com_rslt  = BMA4XY_INIT_VALUE;
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write command register */
		com_rslt = bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
						BMA4XY_CMD_COMMANDS_REG,
						&command_reg_u8,
						BMA4XY_READ_LENGTH);
	}
	return com_rslt;
}


/*!
 *	@brief This API write
 *	pull up configuration from the register 0X0B bit 0 an 1
 *
 *  @param control_pullup_u8: The value of pull up register
 *
 *  @note page1 should be set before calling this function
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_pullup_configuration(u8 control_pullup_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write  pull up value */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_COM_C_TRIM_REG_0_ADDR, data_u8,
					(BMA4XY_READ_LENGTH + READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_PULL_UP,
						control_pullup_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_COM_C_TRIM_REG_0_ADDR,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set
 *	I2C device address of auxiliary mag from the register 0x4B bit 1 to 7
 *
 *  @param i2c_device_addr_u8 : The value of mag I2C device address
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_i2c_device_addr(u8 i2c_device_addr_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write the mag I2C device address*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_I2C_DEVICE_ADDR_REG, data_u8,
					(BMA4XY_READ_LENGTH + READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_I2C_DEVICE_ADDR,
						i2c_device_addr_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_I2C_DEVICE_ADDR_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
			}
		}
	return com_rslt;
}

/*!
 *	@brief This API is used to set
 *	Enable register access on MAG_IF[2] or MAG_IF[3] writes.
 *	This implies that the DATA registers are not updated with
 *	magnetometer values. Accessing magnetometer requires
 *	the magnetometer in normal mode in PMU_STATUS.
 *	from the register 0x4C bit 7
 *
 *  @param mag_manual_u8 : The value of mag manual enable
 *	value    | mag manual
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_mag_manual_enable(u8 mag_manual_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write the mag manual*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_MAG_MANUAL_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

		if (BMA4XY_SUCCESS == com_rslt) {
			/* set the bit of mag manual enable*/
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MAG_MANUAL_ENABLE,
						mag_manual_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_MAG_MANUAL_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_LENGTH);

			if (BMA4XY_SUCCESS == com_rslt)
				bma4xy_p->mag_manual_enable = mag_manual_u8;
		} else {
			bma4xy_p->mag_manual_enable = BMA4XY_INIT_VALUE;
		}
	}
	return com_rslt;
}


/*!
 * @brief This API write I2C interface configuration(if) mode
 * from the register 0x6B bit 4
 *
 *  @param  if_mode_u8 : The value of interface configuration mode
 *	value			|		Description
 *	----------------|-------------------------------------------
 *		0	|	p_auto_s_off Auxiliary interface:off
 *		1	|	p_auto_s_mag Auxiliary interface:on
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_aux_if_mode(u8 if_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_IF_CONFIG_IF_MODE_REG, data_u8,
					(BMA4XY_READ_LENGTH + READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_IF_CONFIG_IF_MODE,
						if_mode_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_IF_CONFIG_IF_MODE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);

		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to read data
 *	magnetometer address to read from the register 0x4D bit 0 to 7
 *
 *	@param  mag_read_addr_u8 : The value of address need to be read
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_mag_read_addr(u8 *mag_read_addr_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the written address*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_READ_ADDR_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*mag_read_addr_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_ADDR);
		}
	return com_rslt;
}

/*!
 *	@brief This API is used to set
 *	magnetometer write address from the register 0x4D bit 0 to 7
 *
 *	@param mag_read_addr_u8:
 *	The data of auxiliary mag address to write data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_set_mag_read_addr(u8 mag_read_addr_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write the mag read address*/
		com_rslt = bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
						BMA4XY_READ_ADDR_REG,
						&mag_read_addr_u8,
						BMA4XY_READ_LENGTH);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to read
 *	magnetometer write address from the register 0x4E bit 0 to 7
 *	@brief mag write address writes the address of auxiliary mag to write
 *
 *  @param  mag_write_addr_u8:
 *	The data of auxiliary mag address to write data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_get_mag_write_addr(
u8 *mag_write_addr_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the address of last written */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_WRITE_ADDR_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*mag_write_addr_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_ADDR);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set
 *	magnetometer write address from the register 0x4E bit 0 to 7
 *
 *	@param  mag_write_addr_u8:
 *	The data of auxiliary mag address to write data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_set_mag_write_addr(
u8 mag_write_addr_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write the data of mag address to write data */
		com_rslt = bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
						BMA4XY_WRITE_ADDR_REG,
						&mag_write_addr_u8,
						BMA4XY_READ_LENGTH);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to read magnetometer write data
 *	form the resister 0x4F bit 0 to 7
 *	@brief This data is the data which is coming from
 *	from the magnetometer
 *
 *	@param  mag_write_data_u8: The pointer which stores the
 *	mag data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_mag_write_data(
u8 *mag_write_data_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_WRITE_DATA_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*mag_write_data_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_DATA);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set magnetometer write data
 *	form the resister 0x4F bit 0 to 7
 *	@brief This data will be written to magnetometer.
 *
 *	@param  mag_write_data_u8: The value of mag data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_mag_write_data(
u8 mag_write_data_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
							BMA4XY_WRITE_DATA_REG,
							&mag_write_data_u8,
							BMA4XY_READ_LENGTH);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set the
 *	output data rate of magnetometer from the register 0x44 bit 0 to 3
 *
 *  @param  output_data_rate_u8 : The value of mag output data rate
 *  value   |    mag output data rate
 * ---------|---------------------------
 *  0x00    |BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED
 *  0x01    |BMA4XY_MAG_OUTPUT_DATA_RATE_0_78HZ
 *  0x02    |BMA4XY_MAG_OUTPUT_DATA_RATE_1_56HZ
 *  0x03    |BMA4XY_MAG_OUTPUT_DATA_RATE_3_12HZ
 *  0x04    |BMA4XY_MAG_OUTPUT_DATA_RATE_6_25HZ
 *  0x05    |BMA4XY_MAG_OUTPUT_DATA_RATE_12_5HZ
 *  0x06    |BMA4XY_MAG_OUTPUT_DATA_RATE_25HZ
 *  0x07    |BMA4XY_MAG_OUTPUT_DATA_RATE_50HZ
 *  0x08    |BMA4XY_MAG_OUTPUT_DATA_RATE_100HZ
 *  0x09    |BMA4XY_MAG_OUTPUT_DATA_RATE_200HZ
 *  0x0A    |BMA4XY_MAG_OUTPUT_DATA_RATE_400HZ
 *  0x0B    |BMA4XY_MAG_OUTPUT_DATA_RATE_800HZ
 *  0x0C    |BMA4XY_MAG_OUTPUT_DATA_RATE_1600HZ
 *  0x0D    |BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED0
 *  0x0E    |BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED1
 *  0x0F    |BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_mag_output_data_rate(u8 output_data_rate_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* select the mag data output rate*/
		if ((output_data_rate_u8 >= BMA4XY_MAG_MIN_OUTPUT_DATA_RATE)
		&& (output_data_rate_u8 <= BMA4XY_MAG_MAX_OUTPUT_DATA_RATE)) {
			/* write the mag data output rate*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_MAG_CONFIG_OUTPUT_DATA_RATE_REG,
					data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_MAG_CONFIG_OUTPUT_DATA_RATE,
					output_data_rate_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_MAG_CONFIG_OUTPUT_DATA_RATE_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_READ_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set the offset of
 *	magnetometer from the register 0x44 bit 4 to 7
 *
 *	@param  offset_u8 : The value of mag offset to be set
 *
 *	@note trigger-readout offset in units of 2.5 ms. If set to zero,
 *	the offset is maximum, i.e. after readout a trigger is issued
 *	immediately
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_mag_offset(
u8 offset_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		/* write the mag data output rate*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_MAG_CONFIG_OFFSET_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MAG_CONFIG_OFFSET,
						offset_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_MAG_CONFIG_OFFSET_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);

		}
	}
	return com_rslt;
}

/*!
 *	@brief This function used to read the trim values of magnetometer
 *
 *	@note
 *	Before reading the mag trimming values
 *	make sure the following two points are addressed
 *	@note
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bma4xy_get_if_mode() function.
 *		If mag interface is not enabled, set the value of 0x10
 *		to the function bma4xy_get_if_mode(0x10)
 *	@note
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bma4xy_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bma4xy_set_command_register(0x19) function.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_read_bmm150_mag_trim(void)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	/* Array holding the bmm150 trim data
	*/
	u8 data_u8[BMA4XY_MAG_TRIM_DATA_SIZE] = {
					BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
					BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
					BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
					BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
					BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
					BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
					BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
					BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* read dig_x1 value */
	com_rslt = bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_X1);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_X1],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_x1 = data_u8[BMA4XY_BMM150_DIG_X1];

	/* read dig_y1 value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Y1);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_Y1],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_y1 = data_u8[BMA4XY_BMM150_DIG_Y1];

	/* read dig_x2 value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_X2);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_X2],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_x2 = data_u8[BMA4XY_BMM150_DIG_X2];

	/* read dig_y2 value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Y2);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
					&data_u8[BMA4XY_BMM150_DIG_Y3],
					BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_y2 = data_u8[BMA4XY_BMM150_DIG_Y3];

	/* read dig_xy1 value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_XY1);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_XY1],
				BMA4XY_READ_LENGTH);

	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_xy1 = data_u8[BMA4XY_BMM150_DIG_XY1];
	/* read dig_xy2 value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_XY2);
	bma4xy_p->delay_msec(1);

	/* 0x04 is mag_x_s16 ls register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_XY2],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_xy2 = data_u8[BMA4XY_BMM150_DIG_XY2];

	/* read dig_z1 lsb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Z1_LSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_Z1_LSB],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	/* read dig_z1 msb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Z1_MSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is mag_x_s16 msb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_Z1_MSB],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_z1 = (u16)(
		(((u32)((u8)data_u8[BMA4XY_BMM150_DIG_Z1_MSB]))
		<< 8) | (data_u8[BMA4XY_BMM150_DIG_Z1_LSB]));

	/* read dig_z2 lsb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Z2_LSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_Z2_LSB],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	/* read dig_z2 msb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Z2_MSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is mag_x_s16 msb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_Z2_MSB],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_z2 = (s16)(
		(((s32)((s8)data_u8[BMA4XY_BMM150_DIG_Z2_MSB]))
			<< 8) | (data_u8[BMA4XY_BMM150_DIG_Z2_LSB]));

	/* read dig_z3 lsb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Z3_LSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_Z3_LSB],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	/* read dig_z3 msb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Z3_MSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is mag_x_s16 msb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
				&data_u8[BMA4XY_BMM150_DIG_Z3_MSB],
				BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_z3 = (s16)(
		(((s32)((s8)data_u8[BMA4XY_BMM150_DIG_Z3_MSB]))
			<< 8) | (data_u8[BMA4XY_BMM150_DIG_Z3_LSB]));

	/* read dig_z4 lsb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Z4_LSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
					&data_u8[BMA4XY_BMM150_DIG_Z4_LSB],
					BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	/* read dig_z4 msb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_Z4_MSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is mag_x_s16 msb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
					&data_u8[BMA4XY_BMM150_DIG_Z4_MSB],
					BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_z4 = (s16)(
		(((s32)((s8)data_u8[BMA4XY_BMM150_DIG_Z4_MSB]))
			<< 8) | (data_u8[BMA4XY_BMM150_DIG_Z4_LSB]));

	/* read dig_xyz1 lsb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_XYZ1_LSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is secondary read mag x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
					&data_u8[BMA4XY_BMM150_DIG_XYZ1_LSB],
					BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	/* read dig_xyz1 msb value */
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_MAG_DIG_XYZ1_MSB);
	bma4xy_p->delay_msec(1);

	/* 0x04 is mag_x_s16 msb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
					&data_u8[BMA4XY_BMM150_DIG_XYZ1_MSB],
					BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(1);

	bma4xy_mag_trim.dig_xyz1 =
		(u16)((((u32)((u8)data_u8[BMA4XY_BMM150_DIG_XYZ1_MSB]))
		<< 8) | (data_u8[BMA4XY_BMM150_DIG_XYZ1_LSB]));

	return com_rslt;
}
/*!*
 *	@brief This API reads magnetometer data X,Y,Z,r
 *	values from the register 0x0A to 0x11
 *
 *	@brief The mag sensor data read from auxiliary mag
 *
 *  @param mag : The value of mag-BMM150 xyzr data
 *
 *	@note For mag data output rate configuration use the following function
 *	@note bma4xy_set_mag_output_data_rate()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_read_mag_xyzr(struct bma4xy_mag_xyzr_t *mag)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[BMA4XY_MAG_XYZR_DATA_SIZE+1] = {
				BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
				BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
				BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
				BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
				BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_DATA_MAG_X_LSB_REG, data_u8,
				BMA4XY_MAG_XYZR_DATA_LENGTH+READ_EXTRA_BYTE);


		if (BMA4XY_SUCCESS == com_rslt) {

			/* Data X */
			/*X-axis lsb value shifting*/
			data_u8[BMA4XY_MAG_X_LSB_BYTE+READ_EXTRA_BYTE] =
						BMA4XY_GET_BITSLICE(
						data_u8[BMA4XY_MAG_X_LSB_BYTE+
						READ_EXTRA_BYTE],
						BMA4XY_DATA_MAG_X_LSB);

			mag->x =
			(s16)((((s32)((s8)data_u8[BMA4XY_MAG_X_MSB_BYTE+
			READ_EXTRA_BYTE])) << 5) |
			(data_u8[BMA4XY_MAG_X_LSB_BYTE+READ_EXTRA_BYTE]));

			/* Data Y */
			/*Y-axis lsb value shifting*/
			data_u8[BMA4XY_MAG_Y_LSB_BYTE+READ_EXTRA_BYTE] =
						BMA4XY_GET_BITSLICE(
						data_u8[BMA4XY_MAG_Y_LSB_BYTE+
						READ_EXTRA_BYTE],
						BMA4XY_DATA_MAG_Y_LSB);

			mag->y =
				(s16)((((s32)((s8)data_u8[BMA4XY_MAG_Y_MSB_BYTE
				+READ_EXTRA_BYTE])) << 5) |
				(data_u8[BMA4XY_MAG_Y_LSB_BYTE+
				READ_EXTRA_BYTE]));

			/* Data Z */
			/*Z-axis lsb value shifting*/
			data_u8[BMA4XY_MAG_Z_LSB_BYTE+READ_EXTRA_BYTE] =
				BMA4XY_GET_BITSLICE(
				data_u8[BMA4XY_MAG_Z_LSB_BYTE+READ_EXTRA_BYTE],
				BMA4XY_DATA_MAG_Z_LSB);

			mag->z =
				(s16)((((s32)((s8)data_u8[BMA4XY_MAG_Z_MSB_BYTE
				+READ_EXTRA_BYTE])) << 7) |
				(data_u8[BMA4XY_MAG_Z_LSB_BYTE+
				READ_EXTRA_BYTE]));

			/* RHall */
			/*R-axis lsb value shifting*/
			data_u8[BMA4XY_MAG_R_LSB_BYTE+READ_EXTRA_BYTE] =
				BMA4XY_GET_BITSLICE(
				data_u8[BMA4XY_MAG_R_LSB_BYTE+
				READ_EXTRA_BYTE], BMA4XY_DATA_MAG_R_LSB);

			mag->r =
				(s16)((((s32)((s8)data_u8[BMA4XY_MAG_R_MSB_BYTE
				+READ_EXTRA_BYTE])) << 6) |
				(data_u8[BMA4XY_MAG_R_LSB_BYTE+
				READ_EXTRA_BYTE]));
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	burst data length (1,2,6,8 byte) from the register 0x4C bit 0 to 1
 *
 *	@param mag_burst_u8 : The data of mag burst read length
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_mag_burst(
u8 mag_burst_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write mag burst mode length*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_MAG_BURST_REG, data_u8,
					(BMA4XY_READ_LENGTH + READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_MAG_BURST, mag_burst_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_MAG_BURST_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_READ_LENGTH);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to get the burst data length
 *	from the register 0x4C bit 0 to 1
 *
 *	@param mag_burst_u8 : pointer used to store the burst length
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_mag_burst(
u8 *mag_burst_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write mag burst mode length*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_MAG_BURST_REG, data_u8,
					(BMA4XY_READ_LENGTH + READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			*mag_burst_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MAG_BURST);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API used to set the pre-set modes of bmm150
 *	The pre-set mode setting depends on the data rate and
 *	xy and z repetitions
 *
 *	@note
 *	Before setting the mag preset mode
 *	make sure the following two points are addressed
 *	@note
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bma4xy_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x10
 *		to the function bma4xy_get_if_mode(0x10)
 *	@note
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bma4xy_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bma4xy_set_command_register(0x19) function.
 *
 *
 *  @param  mode_u8: The value of pre-set mode selection value
 *  value    |  pre_set mode
 * ----------|------------
 *   1       | BMA4XY_MAG_PRESETMODE_LOWPOWER
 *   2       | BMA4XY_MAG_PRESETMODE_REGULAR
 *   3       | BMA4XY_MAG_PRESETMODE_HIGHACCURACY
 *   4       | BMA4XY_MAG_PRESETMODE_ENHANCED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_set_bmm150_mag_presetmode(u8 mode_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* set mag interface manual mode*/
	if (bma4xy_p->mag_manual_enable != BMA4XY_MANUAL_ENABLE)
		com_rslt = bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_ENABLE);

	switch (mode_u8) {
	case BMA4XY_MAG_PRESETMODE_LOWPOWER:

		/* write the XY and Z repetitions*/
		com_rslt = bma4xy_set_mag_write_data(BMA4XY_MAG_LOWPOWER_REPXY);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_XY_REP);
		bma4xy_p->delay_msec(10);

		/* write the Z repetitions*/
		com_rslt += bma4xy_set_mag_write_data(BMA4XY_MAG_LOWPOWER_REPZ);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_Z_REP);
		bma4xy_p->delay_msec(10);

		/* set the mag data_u8 rate as 10 in the register 0x4C*/
		com_rslt += bma4xy_set_mag_write_data(BMA4XY_MAG_LOWPOWER_DR);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(
			BMA4XY_BMM150_POWER_MODE_REG);
		bma4xy_p->delay_msec(10);
	break;
	case BMA4XY_MAG_PRESETMODE_REGULAR:

		/* write the XY and Z repetitions*/
		com_rslt = bma4xy_set_mag_write_data(BMA4XY_MAG_REGULAR_REPXY);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_XY_REP);
		bma4xy_p->delay_msec(10);

		/* write the Z repetitions*/
		com_rslt += bma4xy_set_mag_write_data(BMA4XY_MAG_REGULAR_REPZ);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_Z_REP);
		bma4xy_p->delay_msec(10);

		/* set the mag data_u8 rate as 10 in the register 0x4C*/
		com_rslt += bma4xy_set_mag_write_data(BMA4XY_MAG_REGULAR_DR);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(
						BMA4XY_BMM150_POWER_MODE_REG);
		bma4xy_p->delay_msec(10);
	break;
	case BMA4XY_MAG_PRESETMODE_HIGHACCURACY:

		/* write the XY and Z repetitions*/
		com_rslt = bma4xy_set_mag_write_data(
						BMA4XY_MAG_HIGHACCURACY_REPXY);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_XY_REP);
		bma4xy_p->delay_msec(10);

		/* write the Z repetitions*/
		com_rslt += bma4xy_set_mag_write_data(
						BMA4XY_MAG_HIGHACCURACY_REPZ);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_Z_REP);
		bma4xy_p->delay_msec(10);

		/* set the mag data_u8 rate as 20 in the register 0x4C*/
		com_rslt += bma4xy_set_mag_write_data(
						BMA4XY_MAG_HIGHACCURACY_DR);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(
						BMA4XY_BMM150_POWER_MODE_REG);
		bma4xy_p->delay_msec(10);
	break;
	case BMA4XY_MAG_PRESETMODE_ENHANCED:

		/* write the XY and Z repetitions*/
		com_rslt = bma4xy_set_mag_write_data(BMA4XY_MAG_ENHANCED_REPXY);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

		com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_XY_REP);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

		/* write the Z repetitions*/
		com_rslt += bma4xy_set_mag_write_data(BMA4XY_MAG_ENHANCED_REPZ);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

		com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_Z_REP);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

		/* set the mag data_u8 rate as 10 in the register 0x4C*/
		com_rslt += bma4xy_set_mag_write_data(BMA4XY_MAG_ENHANCED_DR);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

		com_rslt += bma4xy_set_mag_write_addr(
						BMA4XY_BMM150_POWER_MODE_REG);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
	break;
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
	if (bma4xy_V_bmm150_maual_auto_condition_u8 == BMA4XY_MANUAL_DISABLE) {
		com_rslt += bma4xy_set_mag_write_data(BMA4XY_BMM150_FORCE_MODE);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_write_addr(
						BMA4XY_BMM150_POWER_MODE_REG);
		bma4xy_p->delay_msec(10);

		com_rslt += bma4xy_set_mag_read_addr(BMA4XY_BMM150_DATA_REG);
		bma4xy_p->delay_msec(10);

		/* set mag interface auto mode*/
		if (bma4xy_p->mag_manual_enable == BMA4XY_MANUAL_ENABLE)
			com_rslt = bma4xy_set_mag_manual_enable(
			BMA4XY_MANUAL_DISABLE);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to read
 *	Enable register access on MAG_IF[2] or MAG_IF[3] writes.
 *	This implies that the DATA registers are not updated with
 *	magnetometer values. Accessing magnetometer requires
 *	the magnetometer in normal mode in PMU_STATUS.
 *	from the register 0x4C bit 7
 *
 *	@param mag_manual_u8 : The value of mag manual enable
 *	value	| mag manual
 *	--------|-------------------
 *	0x01	|	BMA4XY_ENABLE
 *	0x00	|	BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_mag_manual_enable(
u8 *mag_manual_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read mag manual */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_MAG_MANUAL_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*mag_manual_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MAG_MANUAL_ENABLE);
	}
	return com_rslt;
}

/*!
 *	@brief This function used to calculate the compensated value of mag
 *	Before start calculating the mag compensated data
 *	make sure the following two points are addressed
 *	@note
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bma4xy_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x10
 *		to the function bma4xy_get_if_mode(0x10)
 *	@note
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bma4xy_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bma4xy_set_command_register(0x19) function.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_compensate_xyz(
struct bma4xy_mag_xyz_s32_t *mag_comp_xyz)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	struct bma4xy_mag_xyzr_t mag_xyzr;

	com_rslt = bma4xy_read_mag_xyzr(&mag_xyzr);
	if (BMA4XY_SUCCESS == com_rslt) {
		/* Compensation for X axis */
		mag_comp_xyz->x = bma4xy_bmm150_mag_compensate_X(
							mag_xyzr.x, mag_xyzr.r);
		/* Compensation for Y axis */
		mag_comp_xyz->y = bma4xy_bmm150_mag_compensate_Y(
							mag_xyzr.y, mag_xyzr.r);
		/* Compensation for Z axis */
		mag_comp_xyz->z = bma4xy_bmm150_mag_compensate_Z(
							mag_xyzr.z, mag_xyzr.r);
	}

	return com_rslt;
}
BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_compensate_xyz_raw(
struct bma4xy_mag_xyz_s32_t *mag_comp_xyz, struct bma4xy_mag_xyzr_t mag_xyzr)
{
	/* variable used for return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* Compensation for X axis */
	mag_comp_xyz->x = bma4xy_bmm150_mag_compensate_X(
	mag_xyzr.x, mag_xyzr.r);

	/* Compensation for Y axis */
	mag_comp_xyz->y = bma4xy_bmm150_mag_compensate_Y(
	mag_xyzr.y, mag_xyzr.r);

	/* Compensation for Z axis */
	mag_comp_xyz->z = bma4xy_bmm150_mag_compensate_Z(
	mag_xyzr.z, mag_xyzr.r);

	return com_rslt;
}

/*!
 *	@brief This API used to get the compensated BMM150-X data
 *	the out put of X as s32
 *
 *  @param  mag_data_x_s16 : The value of mag raw X data
 *  @param  data_r_u16 : The value of mag R data
 *
 *	@return results of compensated X data value output as s32
 *
 */
s32 bma4xy_bmm150_mag_compensate_X(s16 mag_data_x_s16, u16 data_r_u16)
{
	s32 inter_retval = BMA4XY_INIT_VALUE;
	/* no overflow */
	if (mag_data_x_s16 != BMA4XY_MAG_FLIP_OVERFLOW_ADCVAL) {
		if ((data_r_u16 != 0) || (bma4xy_mag_trim.dig_xyz1 != 0)) {
				inter_retval = ((s32)(((u16)(
			(((s32)bma4xy_mag_trim.dig_xyz1)
			<< 14)/(data_r_u16 != 0 ? data_r_u16 :
			bma4xy_mag_trim.dig_xyz1))) - ((u16)0x4000)));
		} else {
			inter_retval = BMA4XY_MAG_OVERFLOW_OUTPUT;
			return inter_retval;
		}

		inter_retval = ((s32)((((s32)mag_data_x_s16) *
		((((((((s32)bma4xy_mag_trim.dig_xy2) *
		((((s32)inter_retval) * ((s32)inter_retval)) >> 7)) +
		(((s32)inter_retval) * ((s32)(((s16)bma4xy_mag_trim.dig_xy1)
		<< 7)))) >> 9) + ((s32)0x100000)) *
		((s32)(((s16)bma4xy_mag_trim.dig_x2) + ((s16)0xA0))))
			>> BMA4XY_SHIFT_BIT_POSITION_BY_12_BITS))
			>> BMA4XY_SHIFT_BIT_POSITION_BY_13_BITS)) +
		(((s16)bma4xy_mag_trim.dig_x1)
		<< 3);
	/* check the overflow output */
	if (inter_retval == (s32)BMA4XY_MAG_OVERFLOW_OUTPUT)
		inter_retval = BMA4XY_MAG_OVERFLOW_OUTPUT_S32;
	} else {
		/* overflow */
		inter_retval = BMA4XY_MAG_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

/*!
 *	@brief This API used to get the compensated BMM150-Y data
 *	the out put of Y as s32
 *
 *  @param  mag_data_y_s16 : The value of mag raw Y data
 *  @param  data_r_u16 : The value of mag R data
 *
 *	@return results of compensated Y data value output as s32
 */
s32 bma4xy_bmm150_mag_compensate_Y(s16 mag_data_y_s16, u16 data_r_u16)
{
s32 inter_retval = BMA4XY_INIT_VALUE;
/* no overflow */
if (mag_data_y_s16 != BMA4XY_MAG_FLIP_OVERFLOW_ADCVAL) {
	if ((data_r_u16 != 0)
	|| (bma4xy_mag_trim.dig_xyz1 != 0)) {
		inter_retval = ((s32)(((u16)(((
		(s32)bma4xy_mag_trim.dig_xyz1)
		<< BMA4XY_SHIFT_BIT_POSITION_BY_14_BITS) /
		(data_r_u16 != 0 ?
		 data_r_u16 : bma4xy_mag_trim.dig_xyz1))) -
		((u16)0x4000)));
		} else {
			inter_retval = BMA4XY_MAG_OVERFLOW_OUTPUT;
			return inter_retval;
		}
	inter_retval = ((s32)((((s32)mag_data_y_s16) * ((((((((s32)
		bma4xy_mag_trim.dig_xy2) * ((((s32) inter_retval) *
		((s32)inter_retval)) >> 7))
		+ (((s32)inter_retval) *
		((s32)(((s16)bma4xy_mag_trim.dig_xy1)
		<< 7))))
		>> 9) +
		((s32)0x100000))
		* ((s32)(((s16)bma4xy_mag_trim.dig_y2)
		+ ((s16)0xA0))))
		>> BMA4XY_SHIFT_BIT_POSITION_BY_12_BITS))
		>> BMA4XY_SHIFT_BIT_POSITION_BY_13_BITS)) +
		(((s16)bma4xy_mag_trim.dig_y1)
		<< 3);
	/* check the overflow output */
	if (inter_retval == (s32)BMA4XY_MAG_OVERFLOW_OUTPUT)
		inter_retval = BMA4XY_MAG_OVERFLOW_OUTPUT_S32;
} else {
	/* overflow */
	inter_retval = BMA4XY_MAG_OVERFLOW_OUTPUT;
}
return inter_retval;
}

/*!
 *	@brief This API used to get the compensated BMM150-Z data
 *	the out put of Z as s32
 *
 *  @param  mag_data_z_s16 : The value of mag raw Z data
 *  @param  data_r_u16 : The value of mag R data
 *
 *	@return results of compensated Z data value output as s32
 */
s32 bma4xy_bmm150_mag_compensate_Z(s16 mag_data_z_s16, u16 data_r_u16)
{
	s32 retval = BMA4XY_INIT_VALUE;

	if (mag_data_z_s16 != BMA4XY_MAG_HALL_OVERFLOW_ADCVAL) {
		if ((data_r_u16 != 0) && (bma4xy_mag_trim.dig_z2 != 0)
		&& (bma4xy_mag_trim.dig_z1 != 0)) {
			retval = (((
		((s32)(mag_data_z_s16 - bma4xy_mag_trim.dig_z4))
		<< 15) - ((((s32)bma4xy_mag_trim.dig_z3) *
				((s32)(((s16)data_r_u16) -
		((s16)bma4xy_mag_trim.dig_xyz1)))) >> 2))/
		(bma4xy_mag_trim.dig_z2 +
		((s16)(((((s32)bma4xy_mag_trim.dig_z1) *
				((((s16)data_r_u16) << 1))) + (1 << 15))
				>> 16))));
		}
	} else {
		retval = BMA4XY_MAG_OVERFLOW_OUTPUT;
	}
		return retval;
}


/*!
 *	@brief This function used to initialize the bmm150 sensor
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_interface_init(u8 *chip_id_u8)
{
	/* This variable used for provide the communication
	results*/
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8 = BMA4XY_INIT_VALUE;
	u8 data = BMA4XY_INIT_VALUE;

	/* Enable Mag by setting 0x01 in 7D register*/
	com_rslt += bma4xy_set_mag_enable(0x01);

	com_rslt += bma4xy_read_reg(0x7D, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_set_command_register(0x37);
	com_rslt += bma4xy_read_reg(0x7E, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(50);

	bma4xy_set_command_register(0x9A);
	com_rslt += bma4xy_read_reg(0x7E, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	bma4xy_set_command_register(0xC0);
	com_rslt += bma4xy_read_reg(0x7E, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	bma4xy_set_paging_enable(0x01);
	com_rslt += bma4xy_read_reg(0x7F, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_set_target_page(0x01);
	com_rslt += bma4xy_read_reg(0x7F, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(200);

	/* enable secondary interface and pull up conf in page1 */
	data = 0x03;
	bma4xy_write_reg(0x0B, &data, 1);
	com_rslt += bma4xy_read_reg(0x0B, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	bma4xy_p->delay_msec(10);
	bma4xy_set_target_page(0x00);

	com_rslt += bma4xy_read_reg(0x7F, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_set_advance_power_save(0x00);
	com_rslt += bma4xy_read_reg(0x7C, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_set_i2c_device_addr(BMA4XY_AUX_BMM150_I2C_ADDRESS);
	bma4xy_read_reg(0x4B, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_ENABLE);
	bma4xy_p->delay_msec(100);

	bma4xy_set_mag_burst(0x03);
	com_rslt += bma4xy_read_reg(0x4c, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_set_aux_if_mode(0x01);
	bma4xy_read_reg(0x6B, &data_u8, BMA4XY_READ_LENGTH);
	bma4xy_p->delay_msec(100);

	bma4xy_set_mag_write_data(0x01);
	bma4xy_p->delay_msec(100);

	bma4xy_set_mag_write_addr(BMA4XY_BMM150_POWER_CONTROL_REG);
	bma4xy_p->delay_msec(100);

	bma4xy_set_mag_read_addr(0x4B);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
						&data_u8, BMA4XY_READ_LENGTH);

	bma4xy_set_mag_write_data(0x02);
	bma4xy_p->delay_msec(100);

	bma4xy_set_mag_write_addr(BMA4XY_BMM150_POWER_MODE_REG);
	bma4xy_p->delay_msec(100);

	bma4xy_set_mag_read_addr(BMA4XY_BMM150_CHIP_ID);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
		&data_u8, BMA4XY_READ_LENGTH);

	if (BMA4XY_SUCCESS == com_rslt)
		*chip_id_u8 = data_u8;

	/* write the power mode register*/
	com_rslt += bma4xy_set_mag_write_data(BMA4XY_BMM_POWER_MODE_REG);

	/*write 0x4C register to write set power mode to normal*/
	com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_POWER_MODE_REG);
	bma4xy_p->delay_msec(10);

	/* read the mag trim values*/
	com_rslt += bma4xy_read_bmm150_mag_trim();

	/* To avoid the auto mode enable when manual mode operation running*/
	bma4xy_V_bmm150_maual_auto_condition_u8 = BMA4XY_MANUAL_ENABLE;

	/* write the XY and Z repetitions*/
	com_rslt += bma4xy_set_bmm150_mag_presetmode(
						BMA4XY_MAG_PRESETMODE_REGULAR);

	/* To avoid the auto mode enable when manual mode operation running*/
	bma4xy_V_bmm150_maual_auto_condition_u8 = BMA4XY_MANUAL_DISABLE;

	/* Set the power mode of mag to force mode*/
	com_rslt += bma4xy_set_mag_write_data(BMA4XY_BMM150_FORCE_MODE);
	bma4xy_p->delay_msec(10);

	/* write into power mode register*/
	com_rslt += bma4xy_set_mag_write_addr(BMA4XY_BMM150_POWER_MODE_REG);

	/* write the mag data_bw_u8 as 25Hz*/
	com_rslt += bma4xy_set_mag_output_data_rate(
					BMA4XY_MAG_OUTPUT_DATA_RATE_25HZ);
	bma4xy_p->delay_msec(10);

	/* When mag interface is auto mode - The mag read address
	starts the register 0x42*/
	com_rslt += bma4xy_set_mag_read_addr(BMA4XY_BMM150_DATA_REG);
	bma4xy_p->delay_msec(10);

	/* enable mag interface to auto mode*/
	com_rslt += bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_DISABLE);
	bma4xy_p->delay_msec(10);

	bma4xy_get_mag_manual_enable(&data_u8);

	return com_rslt;
}
/*!
 *	@brief Write FIFO Watermark interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x58 bit 1 and 5
 *	@brief interrupt1 bit 1 in the register 0x58
 *	@brief interrupt2 bit 5 in the register 0x58
 *
 *	@param channel_u8: The value of fifo Watermark interrupt selection
 *   channel_u8  |   interrupt
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_fifo_wm_u8 : The value of fifo Watermark interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_intr_fifo_wm(u8 channel_u8, u8 intr_fifo_wm_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		switch (channel_u8) {
		/* write the fifo water mark interrupt */
		case BMA4XY_INTR1_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_INTR_MAP_1_FIFO_WM_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_INTR_MAP_1_FIFO_WM,
					intr_fifo_wm_u8);
			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR_MAP_1_FIFO_WM_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_WRITE_LENGTH);
			}
		break;
		case BMA4XY_INTR2_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR_MAP_2_FIFO_WM_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_INTR_MAP_2_FIFO_WM,
					intr_fifo_wm_u8);
			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_INTR_MAP_2_FIFO_WM_REG,
				&data_u8[READ_EXTRA_BYTE], BMA4XY_WRITE_LENGTH);
			}
		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief Reads FIFO Full interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x58  bit 0 and 4
 *	@brief interrupt1 bit 0 in the register 0x58
 *	@brief interrupt2 bit 4 in the register 0x58
 *
 *	@param channel_u8: The value of fifo full interrupt selection
 *   channel_u8  |   interrupt
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_fifo_full_u8 : The value of fifo full interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_intr_fifo_full(u8 channel_u8,
							u8 *intr_fifo_full_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		switch (channel_u8) {
		/* read the fifo full interrupt */
		case BMA4XY_INTR1_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_INTR_MAP_1_FIFO_FULL_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
			if (BMA4XY_SUCCESS == com_rslt)
				*intr_fifo_full_u8 =
				BMA4XY_GET_BITSLICE(data_u8[READ_EXTRA_BYTE],
				BMA4XY_INTR_MAP_1_FIFO_FULL);
		break;
		case BMA4XY_INTR2_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_INTR_MAP_2_FIFO_FULL_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt)
				*intr_fifo_full_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR_MAP_2_FIFO_FULL);
		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}

/*!
 *	@brief Write FIFO Full interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x56 and 0x57 bit 4
 *	@brief interrupt1 bit 4 in the register 0x56
 *	@brief interrupt2 bit 4 in the register 0x57
 *
 *	@param channel_u8: The value of fifo full interrupt selection
 *   channel_u8  |   interrupt
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_fifo_full_u8 : The value of fifo full interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_intr_fifo_full(u8 channel_u8,
							u8 intr_fifo_full_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		switch (channel_u8) {
		/* write the fifo full interrupt */
		case BMA4XY_INTR1_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_INTR_MAP_1_FIFO_FULL_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] =
					BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_INTR_MAP_1_FIFO_FULL,
					intr_fifo_full_u8);
				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR_MAP_1_FIFO_FULL_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_WRITE_LENGTH);
			}
		break;
		case BMA4XY_INTR2_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_INTR_MAP_2_FIFO_FULL_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_INTR_MAP_2_FIFO_FULL,
					intr_fifo_full_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_INTR_MAP_1_INTR2_FIFO_FULL_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_WRITE_LENGTH);
			}
		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}

/*!
 *	@brief Reads FIFO Watermark interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x58  bit 1 and 5.
 *	@brief interrupt1 bit 1 in the register 0x58
 *	@brief interrupt2 bit 5 in the register 0x58
 *
 *	@param channel_u8: The value of fifo Watermark interrupt selection
 *   channel_u8  |   interrupt
 *  ---------------|---------------
 *       0         | BMA4XY_INTR1_MAP
 *       1         | BMA4XY_INTR2_MAP
 *
 *	@param intr_fifo_wm_u8 : The value of fifo Watermark interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_intr_fifo_wm(u8 channel_u8, u8 *intr_fifo_wm_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		switch (channel_u8) {

		/* read the fifo water mark interrupt */
		case BMA4XY_INTR1_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_INTR_MAP_1_FIFO_WM_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt)
				*intr_fifo_wm_u8 = BMA4XY_GET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_INTR_MAP_1_FIFO_WM);
		break;
		case BMA4XY_INTR2_MAP:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_INTR_MAP_2_FIFO_WM_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt)
				*intr_fifo_wm_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_INTR_MAP_2_FIFO_WM);
			break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
#ifdef FIFO_ENABLE
/*!
 *	@brief This function used to read the
 *	fifo data in header less mode
 *
 *	@note Configure the below functions for FIFO header less mode
 *	@note 3. bma4xy_set_fifo_down_accel
 *	@note 4. bma4xy_set_accel_fifo_filter_dat
 *	@note 5. bma4xy_set_fifo_mag_enable
 *	@note 6. bma4xy_set_fifo_accel_enable
 *	@note For interrupt configuration
 *	@note 1. bma4xy_set_intr_fifo_full
 *	@note 2. bma4xy_set_intr_fifo_wm
 *	@note 3. bma4xy_set_fifo_tag_intr2_enable
 *	@note 4. bma4xy_set_fifo_tag_intr1_enable
 *
 *	@note The fifo reads the whole 1024 bytes
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_read_fifo_headerless_mode(u8 mag_interface_u8)
{
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	struct bma4xy_fifo_data_header_less_t headerless_data;

	/* read the whole FIFO data*/
	com_rslt = bma4xy_read_fifo_headerless_mode_user_defined_length(
				FIFO_FRAME, &headerless_data, mag_interface_u8);

	return com_rslt;
}
/*!
 *	@brief This function used for reading the
 *	fifo data of  header less mode for using user defined length
 *
 *	@param fifo_user_length_u16: The value of length of fifo read data
 *  @param fifo_data: pointer to fifo data structure
 *  @param mag_if_mag_u8: represents mag interface
 *
 *	@note Configure the below functions for FIFO header less mode
 *	@note 3. bma4xy_set_fifo_down_accel
 *	@note 4. bma4xy_set_accel_fifo_filter_dat
 *	@note 5. bma4xy_set_fifo_mag_enable
 *	@note 6. bma4xy_set_fifo_accel_enable
 *	@note For interrupt configuration
 *	@note 1. bma4xy_set_intr_fifo_full
 *	@note 2. bma4xy_set_intr_fifo_wm
 *	@note 3. bma4xy_set_fifo_tag_intr2_enable
 *	@note 4. bma4xy_set_fifo_tag_intr1_enable
 *
 *	@note The fifo reads the whole 1024 bytes
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE
bma4xy_read_fifo_headerless_mode_user_defined_length(u16 fifo_user_length_u16,
			struct bma4xy_fifo_data_header_less_t *fifo_data,
			u8 mag_if_mag_u8)
{
	u8 data_u8 = BMA4XY_INIT_VALUE;
	u32 fifo_index_u16 = BMA4XY_INIT_VALUE;
	u32 fifo_length_u16 = BMA4XY_INIT_VALUE;
	u8 accel_index_u8 = BMA4XY_INIT_VALUE;
	u8 gyro_index_u8 = BMA4XY_INIT_VALUE;
	u8 mag_index_u8 = BMA4XY_INIT_VALUE;
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	struct bma4xy_mag_xyz_s32_t compensated_mag_data = {BMA4XY_INIT_VALUE};

	fifo_data->accel_frame_count = BMA4XY_INIT_VALUE;
	fifo_data->mag_frame_count = BMA4XY_INIT_VALUE;

	/* disable the header data */
	com_rslt = bma4xy_set_fifo_header_enable(BMA4XY_INIT_VALUE);

	/* read mag, accel and gyro enable status*/
	com_rslt += bma4xy_read_reg(BMA4XY_FIFO_CONFIG_1_ADDR, &data_u8,
							BMA4XY_READ_LENGTH);
	data_u8 = data_u8 & BMA4XY_FIFO_M_G_A_ENABLE;

	/* read the fifo data of 1024 bytes*/
	com_rslt += bma4xy_read_fifo_data(&fifo_data_u8[BMA4XY_INIT_VALUE],
		fifo_user_length_u16);

	fifo_length_u16 = fifo_user_length_u16;

	/* loop for executing the different conditions */
	for (fifo_index_u16 = 0; fifo_index_u16 < fifo_length_u16;) {

		/* condition for mag and accel enable*/
		if (data_u8 == BMA4XY_FIFO_M_G_A_ENABLE) {

			/* Raw mag x*/
			mag_data.mag_x_lsb =
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_X_LSB_DATA]);
		mag_data.mag_x_msb =
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_X_MSB_DATA]);
		/* Mag y data*/
		mag_data.mag_y_lsb =
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Y_LSB_DATA]);
		mag_data.mag_y_msb =
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Y_MSB_DATA]);
		/* Mag z data*/
		mag_data.mag_z_lsb =
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Z_LSB_DATA]);
		mag_data.mag_z_msb =
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Z_MSB_DATA]);
		/* Mag r data*/
		mag_data.mag_r_y2_lsb =
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_R_LSB_DATA]);
		mag_data.mag_r_y2_msb =
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_R_MSB_DATA]);
		com_rslt =
		bma4xy_second_if_mag_compensate_xyz(mag_data,
		mag_if_mag_u8, &compensated_mag_data);

			/* compensated mag x */
			fifo_data->mag_fifo[mag_index_u8].x =
							compensated_mag_data.x;

			/* compensated mag y */
			fifo_data->mag_fifo[mag_index_u8].y =
							compensated_mag_data.y;

			/* compensated mag z */
			fifo_data->mag_fifo[mag_index_u8].z =
							compensated_mag_data.z;

			/* check for mag frame count*/
			fifo_data->mag_frame_count = fifo_data->mag_frame_count
						+ BMA4XY_FRAME_COUNT;

		/* Accel raw x data_u8 */
		fifo_data->accel_fifo[accel_index_u8].x =
		(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MGA_FIFO_A_X_MSB]) << 8) |
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MGA_FIFO_A_X_LSB]));

		/* Accel raw y data_u8 */
		fifo_data->accel_fifo[accel_index_u8].y =
		(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MGA_FIFO_A_Y_MSB]) << 8) |
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MGA_FIFO_A_Y_LSB]));

		/* Accel raw z data_u8 */
		fifo_data->accel_fifo[accel_index_u8].z =
		(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MGA_FIFO_A_Z_MSB]) << 8) |
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MGA_FIFO_A_Z_LSB]));

		/* check for accel frame count*/
		fifo_data->accel_frame_count =
		fifo_data->accel_frame_count +
		BMA4XY_FRAME_COUNT;
		accel_index_u8++;
		mag_index_u8++;
		gyro_index_u8++;

		fifo_index_u16 = fifo_index_u16 +
		BMA4XY_FIFO_AMG_LENGTH;
	}
	/* condition for mag and gyro enable*/
	else if (data_u8 == BMA4XY_FIFO_M_G_ENABLE) {

		/* Raw mag x*/
		mag_data.mag_x_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_X_LSB_DATA]);
		mag_data.mag_x_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_X_MSB_DATA]);

		/* Mag y data*/
		mag_data.mag_y_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Y_LSB_DATA]);
		mag_data.mag_y_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Y_MSB_DATA]);

		/* Mag z data*/
		mag_data.mag_z_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Z_LSB_DATA]);
		mag_data.mag_z_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Z_MSB_DATA]);

		/* Mag r data*/
		mag_data.mag_r_y2_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_R_LSB_DATA]);
		mag_data.mag_r_y2_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_R_MSB_DATA]);

		com_rslt = bma4xy_second_if_mag_compensate_xyz(mag_data,
					mag_if_mag_u8, &compensated_mag_data);

		 /* compensated mag x */
		fifo_data->mag_fifo[mag_index_u8].x = compensated_mag_data.x;

		/* compensated mag y */
		fifo_data->mag_fifo[mag_index_u8].y = compensated_mag_data.y;

		/* compensated mag z */
		fifo_data->mag_fifo[mag_index_u8].z = compensated_mag_data.z;

		/* check for mag frame count*/
		fifo_data->mag_frame_count = fifo_data->mag_frame_count +
							BMA4XY_FRAME_COUNT;

		mag_index_u8++;

		fifo_index_u16 = fifo_index_u16 + BMA4XY_FIFO_MA_OR_MG_LENGTH;
	}
	/* condition for mag and accel enable*/
	else if (data_u8 == BMA4XY_FIFO_M_A_ENABLE) {
		/* Raw mag x*/
		mag_data.mag_x_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
				BMA4XY_FIFO_X_LSB_DATA]);
		mag_data.mag_x_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
				BMA4XY_FIFO_X_MSB_DATA]);

		/* Mag y data*/
		mag_data.mag_y_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
				BMA4XY_FIFO_Y_LSB_DATA]);
		mag_data.mag_y_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
				BMA4XY_FIFO_Y_MSB_DATA]);

		/* Mag z data*/
		mag_data.mag_z_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
				BMA4XY_FIFO_Z_LSB_DATA]);
		mag_data.mag_z_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
				BMA4XY_FIFO_Z_MSB_DATA]);
		/* Mag r data*/
		mag_data.mag_r_y2_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
				BMA4XY_FIFO_R_LSB_DATA]);
		mag_data.mag_r_y2_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
				BMA4XY_FIFO_R_MSB_DATA]);
		com_rslt = bma4xy_second_if_mag_compensate_xyz(mag_data,
			mag_if_mag_u8, &compensated_mag_data);
		/* compensated mag x */
		fifo_data->mag_fifo[mag_index_u8].x = compensated_mag_data.x;
		/* compensated mag y */
		fifo_data->mag_fifo[mag_index_u8].y = compensated_mag_data.y;
		/* compensated mag z */
		fifo_data->mag_fifo[mag_index_u8].z = compensated_mag_data.z;
		/* check for mag frame count*/
		fifo_data->mag_frame_count =
		fifo_data->mag_frame_count + BMA4XY_FRAME_COUNT;
		/* Accel raw x data_u8 */
		fifo_data->accel_fifo[accel_index_u8].x =
		(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MA_FIFO_A_X_MSB]) << 8) |
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MA_FIFO_A_X_LSB]));

		/* Accel raw y data_u8 */
		fifo_data->accel_fifo[accel_index_u8].y =
		(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MA_FIFO_A_Y_MSB]) << 8) |
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MA_FIFO_A_Y_LSB]));

		/* Accel raw z data_u8 */
		fifo_data->accel_fifo[accel_index_u8].z =
		(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MA_FIFO_A_Z_MSB]) << 8) |
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_MA_FIFO_A_Z_LSB]));

		/* check for accel frame count*/
		fifo_data->accel_frame_count = fifo_data->accel_frame_count +
					BMA4XY_FRAME_COUNT;
		accel_index_u8++;
		mag_index_u8++;

		fifo_index_u16 = fifo_index_u16 + BMA4XY_FIFO_MA_OR_MG_LENGTH;
	}
	/* condition  for accel enable*/
	else if (data_u8 == BMA4XY_FIFO_A_ENABLE) {
		/* Accel raw x data_u8 */
		fifo_data->accel_fifo[accel_index_u8].x =
		(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_X_MSB_DATA]) << 8) |
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_X_LSB_DATA]));

		/* Accel raw y data_u8 */
		fifo_data->accel_fifo[accel_index_u8].y =
		(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Y_MSB_DATA]) << 8) |
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Y_LSB_DATA]));

		/* Accel raw z data_u8 */
		fifo_data->accel_fifo[accel_index_u8].z =
		(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Z_MSB_DATA]) << 8) |
		(bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Z_LSB_DATA]));

		/* check for accel frame count*/
		fifo_data->accel_frame_count = fifo_data->accel_frame_count +
					BMA4XY_FRAME_COUNT;
		fifo_index_u16 = fifo_index_u16 + BMA4XY_FIFO_A_LENGTH;
		accel_index_u8++;
	}
	/* condition  for mag enable*/
	else if (data_u8 == BMA4XY_FIFO_M_ENABLE) {
		/* Raw mag x*/
		mag_data.mag_x_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_X_LSB_DATA]);
		mag_data.mag_x_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_X_MSB_DATA]);

		/* Mag y data*/
		mag_data.mag_y_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Y_LSB_DATA]);
		mag_data.mag_y_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Y_MSB_DATA]);
		/* Mag z data*/
		mag_data.mag_z_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Z_LSB_DATA]);
		mag_data.mag_z_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_Z_MSB_DATA]);
		/* Mag r data*/
		mag_data.mag_r_y2_lsb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_R_LSB_DATA]);
		mag_data.mag_r_y2_msb = (bma4xy_fifo_data_u8[fifo_index_u16 +
		BMA4XY_FIFO_R_MSB_DATA]);
		com_rslt = bma4xy_second_if_mag_compensate_xyz(mag_data,
			mag_if_mag_u8, &compensated_mag_data);

		/* compensated mag x */
		fifo_data->mag_fifo[mag_index_u8].x = compensated_mag_data.x;
		/* compensated mag y */
		fifo_data->mag_fifo[mag_index_u8].y = compensated_mag_data.y;
		/* compensated mag z */
		fifo_data->mag_fifo[mag_index_u8].z = compensated_mag_data.z;

		/* check for mag frame count*/
		fifo_data->mag_frame_count = fifo_data->mag_frame_count +
		BMA4XY_FRAME_COUNT;

		fifo_index_u16 = fifo_index_u16 + BMA4XY_FIFO_M_LENGTH;
		mag_index_u8++;
	}
	/* condition  for fifo over read enable*/
	if (bma4xy_fifo_data_u8[fifo_index_u16] == FIFO_CONFIG_CHECK1 &&
	bma4xy_fifo_data_u8[fifo_index_u16 + BMA4XY_FIFO_INDEX_LENGTH] ==
	FIFO_CONFIG_CHECK2) {
		break;
	}
	}
	return com_rslt;
}

/*!*
 *	@brief This function used for reading the
 *	fifo data of  header mode
 *	@param	mag_if_u8 for selecting the mag interface
 *	@param	header_data pointer to fifo structure
 *
 *	@note Configure the below functions for FIFO header mode
 *	@note 3. bma4xy_set_fifo_down_accel()
 *	@note 4. bma4xy_set_accel_fifo_filter_dat()
 *	@note 5. bma4xy_set_fifo_mag_enable()
 *	@note 6. bma4xy_set_fifo_accel_enable()
 *	@note 8. bma4xy_set_fifo_header_enable()
 *	@note For interrupt configuration
 *	@note 1. bma4xy_set_intr_fifo_full()
 *	@note 2. bma4xy_set_intr_fifo_wm()
 *	@note 3. bma4xy_set_fifo_tag_intr2_enable()
 *	@note 4. bma4xy_set_fifo_tag_intr1_enable()
 *
 *	@note The fifo reads the whole 1024 bytes
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMA4XY_RETURN_TYPE bma4xy_read_fifo_header_data(u8 mag_if_u8,
				struct bma4xy_fifo_data_header_t *header_data)
{
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* read the whole fifo data*/
	com_rslt = bma4xy_read_fifo_header_data_user_defined_length(
					FIFO_FRAME, mag_if_u8, header_data);

	return com_rslt;
}

/*!
 *	@brief This function used for reading the
 *	fifo data of  header mode for using user defined length
 *
 *	@note Configure the below functions for FIFO header mode
 *	@note 3. bma4xy_set_fifo_down_accel()
 *	@note 4. bma4xy_set_accel_fifo_filter_dat()
 *	@note 5. bma4xy_set_fifo_mag_enable()
 *	@note 6. bma4xy_set_fifo_accel_enable()
 *	@note 8. bma4xy_set_fifo_header_enable()
 *	@note For interrupt configuration
 *	@note 1. bma4xy_set_intr_fifo_full()
 *	@note 2. bma4xy_set_intr_fifo_wm()
 *	@note 3. bma4xy_set_fifo_tag_intr2_enable()
 *	@note 4. bma4xy_set_fifo_tag_intr1_enable()
 *
 *	@note The fifo reads the whole 1024 bytes
 *	and processing the data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMA4XY_RETURN_TYPE bma4xy_read_fifo_header_data_user_defined_length(
			u16 fifo_user_length_u16, u8 mag_if_mag_u8,
			struct bma4xy_fifo_data_header_t *fifo_header_data)
{
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 accel_index_u8 = BMA4XY_INIT_VALUE;
	u8 mag_index_u8 = BMA4XY_INIT_VALUE;
	s8 last_return_stat_s8 = BMA4XY_INIT_VALUE;
	u16 fifo_index_u16 = BMA4XY_INIT_VALUE;
	u8 frame_head_u8 = BMA4XY_INIT_VALUE;
	u8 frame_index_u8 = BMA4XY_INIT_VALUE;
	u16 fifo_length_u16 = BMA4XY_INIT_VALUE;
	struct bma4xy_mag_xyz_s32_t compensated_mag_data = {
	BMA4XY_INIT_VALUE};

	fifo_header_data->accel_frame_count = BMA4XY_INIT_VALUE;
	fifo_header_data->mag_frame_count = BMA4XY_INIT_VALUE;

	/* read fifo data_u8*/

	com_rslt = bma4xy_read_fifo_data(&fifo_data_u8[BMA4XY_INIT_VALUE],
							fifo_user_length_u16);

	if (BMA4XY_SUCCESS == com_rslt) {

		fifo_length_u16 = fifo_user_length_u16;
		for (fifo_index_u16 = 0; fifo_index_u16 < fifo_length_u16;) {
			fifo_header_data->fifo_header[frame_index_u8] =
						fifo_data_u8[fifo_index_u16];

			frame_head_u8 =
				fifo_header_data->fifo_header[frame_index_u8] &
						BMA4XY_FIFO_TAG_INTR_MASK;
			frame_index_u8++;

		switch (frame_head_u8) {
		/* Header frame of accel */
		case FIFO_HEAD_A:
		{	/*fifo data_u8 frame index + 1*/
			fifo_index_u16 = fifo_index_u16 +
						BMA4XY_FIFO_INDEX_LENGTH;

			if ((fifo_index_u16 + BMA4XY_FIFO_A_LENGTH) >
			fifo_length_u16) {
				last_return_stat_s8 = FIFO_A_OVER_LEN;
			break;
			}
			/* Accel raw x data_u8 */
			fifo_header_data->accel_fifo[accel_index_u8].x =
			(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_X_MSB_DATA]) << 8) |
			(bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_X_LSB_DATA]));

			/* Accel raw y data_u8 */
			fifo_header_data->accel_fifo[accel_index_u8].y =
			(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_Y_MSB_DATA]) << 8) |
			(bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_Y_LSB_DATA]));

			/* Accel raw z data_u8 */
			fifo_header_data->accel_fifo[accel_index_u8].z =
			(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_Z_MSB_DATA]) << 8) |
			(bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_Z_LSB_DATA]));

			/* check for accel frame count*/
			fifo_header_data->accel_frame_count =
					fifo_header_data->accel_frame_count +
					BMA4XY_FRAME_COUNT;

			/* index added to 6 for accel */
			fifo_index_u16 = fifo_index_u16 + BMA4XY_FIFO_A_LENGTH;
			accel_index_u8++;

		break;
		}
		/* Header frame of mag */
		case FIFO_HEAD_M:
		{	/*fifo data_u8 frame index + 1*/
			fifo_index_u16 = fifo_index_u16
			+ BMA4XY_FIFO_INDEX_LENGTH;

			if ((fifo_index_u16 + BMA4XY_FIFO_M_LENGTH) >
			(fifo_length_u16)) {
				last_return_stat_s8 = FIFO_M_OVER_LEN;
			break;
			}
			/* Mag x data*/
			mag_data.mag_x_lsb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_X_LSB_DATA]);
			mag_data.mag_x_msb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_X_MSB_DATA]);

			/* Mag y data*/
			mag_data.mag_y_lsb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_Y_LSB_DATA]);
			mag_data.mag_y_msb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_Y_MSB_DATA]);
			/* Mag z data*/
			mag_data.mag_z_lsb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_Z_LSB_DATA]);
			mag_data.mag_z_msb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_Z_MSB_DATA]);
			/* Mag r data*/
			mag_data.mag_r_y2_lsb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_R_LSB_DATA]);
			mag_data.mag_r_y2_msb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_R_MSB_DATA]);

			com_rslt = bma4xy_second_if_mag_compensate_xyz(mag_data,
					mag_if_mag_u8, &compensated_mag_data);

			 /* compensated mag x */
			fifo_header_data->mag_fifo[mag_index_u8].x =
						compensated_mag_data.x;
			/* compensated mag y */
			fifo_header_data->mag_fifo[mag_index_u8].y =
						compensated_mag_data.y;
			/* compensated mag z */
			fifo_header_data->mag_fifo[mag_index_u8].z =
						compensated_mag_data.z;

			/* check for mag frame count*/
			fifo_header_data->mag_frame_count =
					fifo_header_data->mag_frame_count +
							BMA4XY_FRAME_COUNT;

			mag_index_u8++;

			/*fifo M data_u8 frame index + 8*/
			fifo_index_u16 = fifo_index_u16 + BMA4XY_FIFO_M_LENGTH;
		break;
		}

		/* Header frame of mag and accel */
		case FIFO_HEAD_M_A:
			{	/*fifo data_u8 frame index + 1*/
			fifo_index_u16 = fifo_index_u16 + 1;

			if ((fifo_index_u16 + BMA4XY_FIFO_MA_OR_MG_LENGTH) >
			(fifo_length_u16)) {
				last_return_stat_s8 = FIFO_M_A_OVER_LEN;
				break;
			}
			/* Mag x data*/
			mag_data.mag_x_lsb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_X_LSB_DATA]);
			mag_data.mag_x_msb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_X_MSB_DATA]);

			/* Mag y data*/
			mag_data.mag_y_lsb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_Y_LSB_DATA]);
			mag_data.mag_y_msb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_Y_MSB_DATA]);
			/* Mag z data*/
			mag_data.mag_z_lsb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_Z_LSB_DATA]);
			mag_data.mag_z_msb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_Z_MSB_DATA]);
			/* Mag r data*/
			mag_data.mag_r_y2_lsb = (bma4xy_fifo_data_u8[
			fifo_index_u16 + BMA4XY_FIFO_R_LSB_DATA]);
			mag_data.mag_r_y2_msb = (
			bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_R_MSB_DATA]);

			com_rslt = bma4xy_second_if_mag_compensate_xyz(mag_data,
					mag_if_mag_u8, &compensated_mag_data);

			 /* compensated mag x */
			fifo_header_data->mag_fifo[mag_index_u8].x =
						compensated_mag_data.x;
			/* compensated mag y */
			fifo_header_data->mag_fifo[mag_index_u8].y =
						compensated_mag_data.y;
			/* compensated mag z */
			fifo_header_data->mag_fifo[mag_index_u8].z =
						compensated_mag_data.z;

			/* check for mag frame count*/
			fifo_header_data->mag_frame_count =
					fifo_header_data->mag_frame_count +
							BMA4XY_FRAME_COUNT;

			/* Accel raw x data_u8 */
			fifo_header_data->accel_fifo[accel_index_u8].x =
				(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
					BMA4XY_MA_FIFO_A_X_MSB]) << 8) |
				(bma4xy_fifo_data_u8[fifo_index_u16 +
					BMA4XY_MA_FIFO_A_X_LSB]));

			/* Accel raw y data_u8 */
			fifo_header_data->accel_fifo[accel_index_u8].y =
			(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
					BMA4XY_MA_FIFO_A_Y_MSB]) << 8) |
			(bma4xy_fifo_data_u8[fifo_index_u16 +
					BMA4XY_MA_FIFO_A_Y_LSB]));

			/* Accel raw z data_u8 */
			fifo_header_data->accel_fifo[accel_index_u8].z =
			(s16)(((bma4xy_fifo_data_u8[fifo_index_u16 +
					BMA4XY_MA_FIFO_A_Z_MSB]) << 8) |
			(bma4xy_fifo_data_u8[fifo_index_u16 +
					BMA4XY_MA_FIFO_A_Z_LSB]));

			/* check for accel frame count*/
			fifo_header_data->accel_frame_count =
					fifo_header_data->accel_frame_count +
							BMA4XY_FRAME_COUNT;

			/*fifo AM data_u8 frame index + 14(8+6)*/
			fifo_index_u16 = fifo_index_u16 +
						BMA4XY_FIFO_MA_OR_MG_LENGTH;
			accel_index_u8++;
			mag_index_u8++;
		break;
			}
		/* Header frame of sensor time */
		case FIFO_HEAD_SENSOR_TIME:
			{
			fifo_index_u16 = fifo_index_u16 + 1;

			if ((fifo_index_u16 + BMA4XY_FIFO_SENSOR_TIME_LENGTH) >
			(fifo_length_u16)) {
				last_return_stat_s8 = FIFO_SENSORTIME_RETURN;
			break;
			}
			/* Sensor time */
			fifo_header_data->fifo_time =
			(u32)((bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_SENSOR_TIME_MSB] << 16) |
			(bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_SENSOR_TIME_XLSB] << 8) |
			(bma4xy_fifo_data_u8[fifo_index_u16 +
			BMA4XY_FIFO_SENSOR_TIME_LSB]));

			fifo_index_u16 = fifo_index_u16 +
					BMA4XY_FIFO_SENSOR_TIME_LENGTH;
		break;
			}
		/* Header frame of skip frame */
		case FIFO_HEAD_SKIP_FRAME:
		{
			/*fifo data_u8 frame index + 1*/
			fifo_index_u16 = fifo_index_u16 + 1;
			if (fifo_index_u16 + BMA4XY_FIFO_INDEX_LENGTH
			> fifo_length_u16) {
				last_return_stat_s8 = FIFO_SKIP_OVER_LEN;
				break;
			}
			fifo_header_data->skip_frame =
					bma4xy_fifo_data_u8[fifo_index_u16];
			fifo_index_u16 = fifo_index_u16 +
					BMA4XY_FIFO_INDEX_LENGTH;
		break;
		}
		case FIFO_HEAD_INPUT_CONFIG:
		{
			/*fifo data_u8 frame index + 1*/
			fifo_index_u16 = fifo_index_u16 +
						BMA4XY_FIFO_INDEX_LENGTH;
			if ((fifo_index_u16 + BMA4XY_FIFO_INDEX_LENGTH)
							> fifo_length_u16) {
				last_return_stat_s8 =
						FIFO_INPUT_CONFIG_OVER_LEN;
				break;
			}
			fifo_header_data->fifo_input_config_info =
				bma4xy_fifo_data_u8[fifo_index_u16];

			fifo_index_u16 = fifo_index_u16 +
						BMA4XY_FIFO_INDEX_LENGTH;
		break;
			}
		/* Header frame of over read fifo data_u8 */
		case FIFO_HEAD_OVER_READ_LSB:
			{
		/*fifo data_u8 frame index + 1*/
			fifo_index_u16 = fifo_index_u16 + 1;

			if ((fifo_index_u16 + BMA4XY_FIFO_INDEX_LENGTH)
							> (fifo_length_u16)) {
				last_return_stat_s8 = FIFO_OVER_READ_RETURN;
				break;
			}
			if (bma4xy_fifo_data_u8[fifo_index_u16] ==
						FIFO_HEAD_OVER_READ_MSB) {
				/*fifo over read frame index + 1*/
				fifo_index_u16 = fifo_index_u16 +
						BMA4XY_FIFO_INDEX_LENGTH;
			break;
			} else {
				last_return_stat_s8 = FIFO_OVER_READ_RETURN;
			break;
			}
			}

		default:
			last_return_stat_s8 = BMA4XY_FIFO_INDEX_LENGTH;
		break;
		}
	if (last_return_stat_s8 != BMA4XY_SUCCESS)
		break;
	}
	}
return com_rslt;
}
#endif
/*!
 *	@brief This API reads data
 *	 ready FIFO watermark interrupt status
 *	from the register 0x1D bit 1
 *	flag is associated with a specific interrupt function.
 *	It is set when the FIFO watermark interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *  @param fifo_wm_intr_u8 : The status of fifo water mark interrupt
 *
 *	@note FIFO full interrupt can be configured by following functions
 *	@note bma4xy_set_intr_fifo_wm()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_stat1_fifo_wm_intr(u8 *fifo_wm_intr_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_INTR_STAT_1_FIFO_WM_INTR_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_wm_intr_u8 = BMA4XY_GET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_INTR_STAT_1_FIFO_WM_INTR);
	}
	return com_rslt;
}

/*!
 *	@brief This API reads data
 *	 ready FIFO full interrupt status
 *	from the register 0x1D bit 0
 *	flag is associated with a specific interrupt function.
 *	It is set when the FIFO full interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *  @param fifo_full_intr_u8 : The status of fifo water mark interrupt
 *
 *	@note FIFO full interrupt can be configured by following functions
 *	@note bma4xy_set_intr_fifo_full()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_stat1_fifo_full_intr(u8 *fifo_full_intr_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_INTR_STAT_1_FIFO_FULL_INTR_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_full_intr_u8 =
				BMA4XY_GET_BITSLICE(data_u8[READ_EXTRA_BYTE],
					BMA4XY_INTR_STAT_1_FIFO_WM_INTR);
	}
	return com_rslt;
}

/*!
 *	@brief This API reads the fifo data of the sensor
 *	from the register 0x26 bit 0 to 7
 *
 *  @param fifodata_u8 : Pointer holding the fifo data
 *  @param fifo_length_u16 : The value of fifo length maximum
 *	1024
 *
 *	@note For reading FIFO data use the following functions
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_read_fifo_data(u8 *fifodata_u8, u16 fifo_length_u16)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read fifo data*/
		com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_DATA_REG, fifodata_u8,
					fifo_length_u16);

	}
	return com_rslt;
}

/*!
 *	@brief This API is used to store the accel data(all 3 axes) in FIFO
 *  by setting bit 6 in the register 0x47
 *
 *  @param fifo_accel_u8 : The value of fifo accel enable
 *	value    | fifo accel
 * ----------|-------------------
 *  0x00     |  no accel data is stored
 *  0x01     |  accel data is stored
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_fifo_accel_enable(u8 fifo_accel_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (fifo_accel_u8 <= BMA4XY_MAX_VALUE_FIFO_ACCEL) {
			/* write the fifo mag enables*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_FIFO_ACCEL_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_ACCEL_ENABLE,
						fifo_accel_u8);
				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_FIFO_ACCEL_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to check whether the accel data is stored
 *	in FIFO (all 3 axes) or not  by reading the register 0x49 bit 6
 *
 *  @param fifo_accel_u8 : The pointer to store the value of fifo
 *	accel enable
 *
 *	value    | fifo accel
 * ----------|-------------------
 *  0x00     |  no accel data is stored
 *  0x01     |  accel data is stored
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_fifo_accel_enable(u8 *fifo_accel_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the accel fifo enable*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
			bma4xy_p->dev_addr, BMA4XY_FIFO_ACCEL_ENABLE_REG,
			data_u8, (BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_accel_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_ACCEL_ENABLE);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to  store
 *	magnetometer data in FIFO (all 3 axes) by setting the bit 5
 *	in  the register 0x49
 *
 *  @param fifo_mag_u8 : The value of fifo mag enable
 *	value    | fifo mag
 * ----------|-------------------
 *  0x00     |  no magnetometer data is stored
 *  0x01     |  magnetometer data is stored
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error_d
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_fifo_mag_enable(u8 fifo_mag_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (fifo_mag_u8 <= BMA4XY_MAX_VALUE_FIFO_MAG) {
			/* write the fifo mag enable*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr, BMA4XY_FIFO_MAG_ENABLE_REG,
				data_u8, (BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {

				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_FIFO_MAG_ENABLE, fifo_mag_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_FIFO_MAG_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_LENGTH);
			}
			} else {
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to check the whether
 *	magnetometer data is stored in FIFO (all 3 axes) or not
 *	by reading  the register 0x49 bit 5
 *
 *  @param fifo_mag_u8 : The pointer to store the value of
 *	fifo mag enable
 *	value    | fifo mag
 * ----------|-------------------
 *  0x00     |  no magnetometer data is stored
 *  0x01     |  magnetometer data is stored
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_fifo_mag_enable(u8 *fifo_mag_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the fifo mag enable*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_FIFO_MAG_ENABLE_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_mag_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_MAG_ENABLE);
	}
	return com_rslt;
}

/*!
 *	@brief This API set FIFO frame
 *	header enable from the register 0x49 bit 4
 *
 *  @param fifo_header_u8 :The value of fifo header
 *	value    | fifo header
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_fifo_header_enable(u8 fifo_header_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (fifo_header_u8 <= BMA4XY_MAX_VALUE_FIFO_HEADER) {
			/* write the fifo header */
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_FIFO_HEADER_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_HEADER_ENABLE,
						fifo_header_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_FIFO_HEADER_ENABLE_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_GEN_READ_WRITE_DATA_LENGTH);
			}
		} else {
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API reads whether FIFO
 *	header is enabled  or not from
 *	register 0x49 bit 4
 *
 *  @param fifo_header_u8 :The pointer to store
 *	the value of fifo header
 *	value    | fifo header
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_fifo_header_enable(u8 *fifo_header_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read fifo header */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_HEADER_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_header_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_HEADER_ENABLE);
	}
	return com_rslt;
}

/*!
 *	@brief This API get FIFO tag interrupt1 enable status
 *	from the resister 0x49 bit 3
 *
 *  @param fifo_tag_intr1_u8 : The pointer to store the
 *	value of fifo tag interrupt1
 *
 *	value    | fifo tag interrupt
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_fifo_tag_intr1_enable(u8 *fifo_tag_intr1_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read fifo tag interrupt*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_FIFO_TAG_INTR1_ENABLE_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_tag_intr1_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_TAG_INTR1_ENABLE);
	}
	return com_rslt;
}

/*!
 *	@brief This API set FIFO tag interrupt1 enable status
 *	from the resister 0x49 bit 3
 *
 *  @param fifo_tag_intr1_u8 :The value of fifo tag interrupt1
 *	value    | fifo tag interrupt
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_fifo_tag_intr1_enable(u8 fifo_tag_intr1_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (fifo_tag_intr1_u8 <= BMA4XY_MAX_VALUE_FIFO_INTR) {

			/* write the fifo tag interrupt*/
			com_rslt = bma4xy_set_input_enable(BMA4XY_INTR1_MAP,
							fifo_tag_intr1_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_FIFO_TAG_INTR1_ENABLE_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_TAG_INTR1_ENABLE,
						fifo_tag_intr1_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_FIFO_TAG_INTR1_ENABLE_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_WRITE_LENGTH);
		}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API reads FIFO tag interrupt2 enable status
 *	from the resister 0x49 bit 2
 *
 *  @param fifo_tag_intr2_u8 : The value of fifo tag interruptl
 *	value    | fifo tag interrupt
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_fifo_tag_intr2_enable(u8 *fifo_tag_intr2_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the fifo tag interrupt2*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_FIFO_TAG_INTR2_ENABLE_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_tag_intr2_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_TAG_INTR2_ENABLE);
	}
	return com_rslt;
}
/*!
 *	@brief This API set FIFO tag interrupt2 enable status
 *	from the resister 0x49 bit 2
 *
 *  @param fifo_tag_intr2_u8 : The value of fifo tag interrupt
 *	value    | fifo tag interrupt
 * ----------|-------------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_fifo_tag_intr2_enable(u8 fifo_tag_intr2_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (fifo_tag_intr2_u8 <= BMA4XY_MAX_VALUE_FIFO_INTR) {
			/* write the fifo tag interrupt2*/
			com_rslt = bma4xy_set_input_enable(BMA4XY_INTR2_MAP,
							fifo_tag_intr2_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_FIFO_TAG_INTR2_ENABLE_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_TAG_INTR2_ENABLE,
						fifo_tag_intr2_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_FIFO_TAG_INTR2_ENABLE_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_WRITE_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API gets the status whether samples
 *	should be written to fifo or not when fifo is full
 *	0x48 bit 1
 *
 *  @param fifo_stop_on_full_u8 : pointer used to store the
 *	status whether samples should be written to fifo or not
 *	when fifo is full
 *  value      |  fifo_stop_on_full_u8
 * ------------|-------------------------
 *    0x00     |  do not return sensor time frame
 *    0x01     |  return sensor time frame
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_get_fifo_stop_on_full(u8 *fifo_stop_on_full_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the fifo sensor time*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_STOP_ON_FULL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_stop_on_full_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_STOP_ON_FULL);
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to control the samples written to
 *	fifo when the fifo is full in address 0x48 bit 0
 *
 *  @param fifo_stop_on_full_u8 : The value of sensor time
 *  value      |  fifo_stop_on_full_u8
 * ------------|-------------------------
 *    0x00     |  do not stop writing to FIFO when full
 *    0x01     |  stop writing	into fifo when fifo is full
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_set_fifo_stop_on_full(u8 fifo_stop_on_full_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (fifo_stop_on_full_u8 <= BMA4XY_MAX_VALUE_FIFO_TIME) {

			/* write the fifo sensor time*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_FIFO_STOP_ON_FULL_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_STOP_ON_FULL,
						fifo_stop_on_full_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_FIFO_STOP_ON_FULL_REG,
				&data_u8[READ_EXTRA_BYTE], BMA4XY_WRITE_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API reads fifo sensor time
 *	frame after the last valid data frame from the register
 *	0x48 bit 1
 *
 *  @param fifo_time_enable_u8 : The value of sensor time
 *  value      |  fifo sensor time
 * ------------|-------------------------
 *    0x00     |  do not return sensor time frame
 *    0x01     |  return sensor time frame
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_get_fifo_time_enable(u8 *fifo_time_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the fifo sensor time*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_TIME_ENABLE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_time_enable_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_TIME_ENABLE);
		}
	return com_rslt;
}
/*!
 *	@brief This API set fifo sensor time
 *	frame after the last valid data frame form the register
 *	0x48 bit 1
 *
 *  @param fifo_time_enable_u8 : The value of sensor time
 *  value      |  fifo sensor time
 * ------------|-------------------------
 *    0x00     |  do not return sensor time frame
 *    0x01     |  return sensor time frame
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMA4XY_RETURN_TYPE bma4xy_set_fifo_time_enable(u8 fifo_time_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (fifo_time_enable_u8 <= BMA4XY_MAX_VALUE_FIFO_TIME) {

			/* write the fifo sensor time*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr, BMA4XY_FIFO_TIME_ENABLE_REG,
				data_u8, (BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_FIFO_TIME_ENABLE,
					fifo_time_enable_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_FIFO_TIME_ENABLE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_LENGTH);
		}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to read the  interrupt
 *	when FIFO contains water mark level from the register
 *	0x46 bit 0 to 7
 *
 *  @param  fifo_wm_u16 : The value of fifo water mark level
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_fifo_wm(u16 *fifo_wm_u16)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[3] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
	BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		/* read the fifo water mark level*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_FIFO_WM_REG, data_u8,
				(BMA4XY_FIFO_WM_LENGTH+READ_EXTRA_BYTE));


		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_wm_u16 = (data_u8[READ_EXTRA_BYTE+1] << 8) |
						(data_u8[READ_EXTRA_BYTE]);

	}
	return com_rslt;
}

/*!
 *	@brief This API is used to Trigger an interrupt
 *	when FIFO contains water mark level from the register
 *	0x46 bit 0 to 7 and 0x47 bit 8 to 12
 *
 *  @param  fifo_wm_u16 : The value of fifo water mark level
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_fifo_wm(u16 fifo_wm_u16)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		data_u8[0] = (u8)(fifo_wm_u16 & BMA4XY_SET_LOW_BYTE);
		data_u8[1] = (u8)((fifo_wm_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);
		/* write the fifo water mark level*/
		com_rslt = bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_WM_REG, data_u8,
					BMA4XY_FIFO_WM_LENGTH+READ_EXTRA_BYTE);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to read accel fifo filter data
 *	from the register 0x45 bit 7
 *
 *  @param accel_fifo_filter_u8 :The value of accel filter data
 *  value      |  accel_fifo_filter_u8
 * ------------|-------------------------
 *    0x00     |  Unfiltered data
 *    0x01     |  Filtered data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_fifo_filter_data(u8 *accel_fifo_filter_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		/* read the accel fifo filter data */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_FILTER_ACCEL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*accel_fifo_filter_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_FILTER_ACCEL);
		}
	return com_rslt;
}

/*!
 *	@brief This API is used to set accel fifo filter data
 *	from the register 0x45 bit 7
 *
 *  @param accel_fifo_filter_u8 :The value of accel filter data
 *  value      |  accel_fifo_filter_data
 * ------------|-------------------------
 *    0x00     |  Unfiltered data
 *    0x01     |  Filtered data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_fifo_filter_data(u8 accel_fifo_filter_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (accel_fifo_filter_u8 <= BMA4XY_MAX_VALUE_FIFO_FILTER) {

			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_FIFO_FILTER_ACCEL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			/* write accel fifo filter data */
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_FILTER_ACCEL,
						accel_fifo_filter_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_FIFO_FILTER_ACCEL_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to read Down sampling
 *	for accel (2*downs_accel) from the register 0x45
 *	bit 4 to 6
 *
 *  @param fifo_down_u8 :The value of accel fifo
 *	downsampling
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_fifo_down_accel(u8 *fifo_down_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the accel fifo down data */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_DOWN_ACCEL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*fifo_down_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_FIFO_DOWN_ACCEL);
		}
	return com_rslt;
}

 /*!
 *	@brief This API is used to set Down sampling
 *	for accel (2*downs_accel) from the register
 *	0x45 bit 4 to 6
 *
 *  @param fifo_down_u8 :The value of accel fifo
 *	downsampling.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_fifo_down_accel(u8 fifo_down_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* write the accel fifo down data */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_FIFO_DOWN_ACCEL_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_FIFO_DOWN_ACCEL, fifo_down_u8);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_FIFO_DOWN_ACCEL_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_WRITE_LENGTH);
			}
		}
	return com_rslt;
}

/*!
 *	@brief This API reads the  of the sensor
 *	form the register 0x24 and 0x25 bit 0 to 7 and 0 to 5
 *	respectively.
 *	@brief this byte counter is updated each time a complete frame
 *	was read or written
 *
 *  @param fifo_length_u16 : The value of fifo byte counter
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_fifo_length(u16 *fifo_length_u16)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 index = 0;
	/* Array contains the fifo length data
	data_u8[0] - fifo length
	data_u8[1] - fifo length*/
	u8 a_data_u8r[BMA4XY_FIFO_DATA_SIZE + 1] = {0, 0, 0};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read fifo length*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_FIFO_BYTE_COUNTER_LSB_REG, a_data_u8r,
				(BMA4XY_FIFO_DATA_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			index = BMA4XY_FIFO_LENGTH_MSB_BYTE + READ_EXTRA_BYTE;
			a_data_u8r[index] = BMA4XY_GET_BITSLICE(
						a_data_u8r[index],
						BMA4XY_FIFO_BYTE_COUNTER_MSB);

			*fifo_length_u16 =
				(u32)(((u32)((u8)(a_data_u8r[index]) << 8)) |
							a_data_u8r[index-1]);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This function used for reading the compensated data of
 *	mag secondary interface xyz data
 *
 *	@param mag_fifo_data: object of mag fifo data structure
 *	@param mag_second_if_u8: The value of mag selection
 *
 *  value   |   mag_second_if_u8
 * ---------|----------------------
 *    0     |    BMM150
 *    1     |    AKM09916
 *
 * @param compensated_mag_data: pointer to store the
 * compensated mag xyz data
 *
 *	@return results of bus communication function (applicable only
 *	for yamaha)
 *
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_second_if_mag_compensate_xyz(
			struct bma4xy_mag_fifo_data_t mag_fifo_data,
			u8 mag_second_if_u8,
			struct bma4xy_mag_xyz_s32_t *compensated_mag_data)
{
	s8 com_rslt = BMA4XY_INIT_VALUE;
	s16 mag_x_s16 = BMA4XY_INIT_VALUE;
	s16 mag_y_s16 = BMA4XY_INIT_VALUE;
	s16 mag_z_s16 = BMA4XY_INIT_VALUE;
	u16 mag_r_u16 = BMA4XY_INIT_VALUE;

	switch (mag_second_if_u8) {
	case BMA4XY_SEC_IF_BMM150:
		/* x data*/
		mag_x_s16 = (s16)((mag_fifo_data.mag_x_msb << 8)
						| (mag_fifo_data.mag_x_lsb));
		mag_x_s16 = (s16) (mag_x_s16 >> 3);

		/* y data*/
		mag_y_s16 = (s16)((mag_fifo_data.mag_y_msb << 8)
						| (mag_fifo_data.mag_y_lsb));
		mag_y_s16 = (s16) (mag_y_s16 >> 3);

		/* z data*/
		mag_z_s16 = (s16)((mag_fifo_data.mag_z_msb << 8)
						| (mag_fifo_data.mag_z_lsb));
		mag_z_s16 = (s16) (mag_z_s16 >> 1);

		/* r data*/
		mag_r_u16 = (u16)((mag_fifo_data.mag_r_y2_msb << 8)
						| (mag_fifo_data.mag_r_y2_lsb));
		mag_r_u16 = (u16) (mag_r_u16 >> 2);

		/* Compensated mag x data */
		compensated_mag_data->x = bma4xy_bmm150_mag_compensate_X(
							mag_x_s16, mag_r_u16);

		/* Compensated mag y data */
		compensated_mag_data->y = bma4xy_bmm150_mag_compensate_Y(
							mag_y_s16, mag_r_u16);

		/* Compensated mag z data */
		compensated_mag_data->z = bma4xy_bmm150_mag_compensate_Z(
							mag_z_s16, mag_r_u16);
	break;
	#ifdef AKM9916
	case BMA4XY_SEC_IF_AKM09916:
			/* Compensated for X data */
		compensated_mag_data->x = (s16)((mag_fifo_data.mag_x_msb << 8)
						| (mag_fifo_data.mag_x_lsb));
		/* y data*/
		compensated_mag_data->y = (s16)((mag_fifo_data.mag_y_msb << 8)
						| (mag_fifo_data.mag_y_lsb));
		/* z data*/
		compensated_mag_data->z = (s16)((mag_fifo_data.mag_z_msb << 8)
						| (mag_fifo_data.mag_z_lsb));
	break;
	#endif
	default:
		com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
	}
		return com_rslt;
}

/*!
 *	@brief This API is used to switch the page
 *	to page to zero or one
 *
 *  @param page : variable to set the page
 *  value      |	page
 * ------------|-------------------------
 *    0x00     |	page 0
 *    0x01     |	page 1
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMA4XY_RETURN_TYPE bma4xy_switch_page(u8 page)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	if (page == BMA4XY_WRITE_TARGET_PAGE1) {
		com_rslt += bma4xy_set_paging_enable(BMA4XY_WRITE_ENABLE_PAGE1);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
	}
	/*switch the page*/
	com_rslt += bma4xy_set_target_page(page);
	bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

	return com_rslt;
}


/*!
 *	@brief This API is used to enable the  secondary interface
 *	from register 0x0B bit 2 (page 1)
 *
 *  @param sec_interface_enable : variable to enable or disable sec.interface
 *  value      |  sec_interface_enable
 * ------------|-------------------------
 *    0x00     |  enable sec. interface
 *    0x01     |  disable sec. interface
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMA4XY_RETURN_TYPE bma4xy_enable_sec_interface(u8 sec_interface_enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = E_BMA4XY_COMM_RES;

	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_SEC_INTERFACE_ENABLE_REG,
				data_u8, (BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_SEC_INTERFACE_ENABLE,
					sec_interface_enable);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC
					(bma4xy_p->dev_addr,
					BMA4XY_SEC_INTERFACE_ENABLE_REG,
					&data_u8[READ_EXTRA_BYTE],
					BMA4XY_READ_LENGTH);
			}
	}
	return com_rslt;

}



/*!
 *	@brief This function used for initialize
 *	the AKM09916 sensor
 *
 *	@param	akm_chip_id: pointer used to store the chip id of akm9916
 *	@param akm_i2c_address_u8: The value of device address
 *	AKM sensor   |  Slave address
 * --------------|---------------------
 *  AKM09916     |  BMA4XY_AUX_AKM09916_I2C_ADDR
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_bst_akm_mag_interface_init(u8 akm_i2c_address_u8,
								u8 *akm_chip_id)
{
  /* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = E_BMA4XY_COMM_RES;
	u8 v_pull_value_u8 = BMA4XY_INIT_VALUE;
	u8 v_data_u8 = BMA4XY_INIT_VALUE;

	/* register 0x7E write the 0x37, 0x9A and 0xC0*/
	com_rslt += bma4xy_set_command_register(BMA4XY_COMMAND_REG_ONE);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_set_command_register(BMA4XY_COMMAND_REG_TWO);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_set_command_register(BMA4XY_COMMAND_REG_THREE);
	bma4xy_p->delay_msec(100);

	com_rslt +=  bma4xy_set_accel_enable(0x01);
	bma4xy_p->delay_msec(100);

	com_rslt += bma4xy_set_mag_enable(0x01);
	bma4xy_p->delay_msec(100);

	/*switch the page1*/
	bma4xy_switch_page(BMA4XY_WRITE_TARGET_PAGE1);

	/* enable the pullup configuration from
	the register 0x05 bit 4 and 5  to 10*/
	bma4xy_get_pullup_configuration(&v_pull_value_u8);
	bma4xy_p->delay_msec(100);

	v_pull_value_u8 = v_pull_value_u8 | BMA4XY_PULL_UP_DATA;

	com_rslt += bma4xy_set_pullup_configuration(v_pull_value_u8);
	bma4xy_p->delay_msec(100);

	/* enable sec. interface in 0x0B in page1
	0x00 => enable
	0x01 => disable */

	bma4xy_enable_sec_interface(0x00);
	bma4xy_p->delay_msec(100);

	/*switch the page0*/
	bma4xy_switch_page(BMA4XY_WRITE_TARGET_PAGE0);

	/* Write the AKM9916 i2c address*/
	com_rslt += bma4xy_set_i2c_device_addr(akm_i2c_address_u8);
	bma4xy_p->delay_msec(100);

	/* enable the mag interface to manual mode*/
	com_rslt += bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_ENABLE);
	bma4xy_p->delay_msec(100);

	bma4xy_get_mag_manual_enable(&v_data_u8);
	bma4xy_p->delay_msec(100);

	/*Enable the MAG interface */
	com_rslt += bma4xy_set_if_mode(BMA4XY_ENABLE_MAG_IF_MODE);
	bma4xy_p->delay_msec(100);

	bma4xy_get_if_mode(&v_data_u8);
	bma4xy_p->delay_msec(100);

	/* read the device id of the AKM sensor
	if device id is 0x09 - AKM09916*/
	com_rslt += bma4xy_set_mag_read_addr(AKM_CHIP_ID_REG);
	bma4xy_p->delay_msec(100);

	/* 0x04 is mag_x lsb register */
	com_rslt += bma4xy_read_reg(BMA4XY_MAG_DATA_READ_REG,
	akm_chip_id, BMA4XY_GEN_READ_WRITE_DATA_LENGTH);

	/* Set value power down mode mode*/
	com_rslt += bma4xy_set_mag_write_data(AKM_POWER_DOWN_MODE_DATA);
	bma4xy_p->delay_msec(100);

	/* AKM mode address is 0x31*/
	com_rslt += bma4xy_set_mag_write_addr(AKM_POWER_MODE_REG);
	bma4xy_p->delay_msec(100);

	/* Set AKM Force mode*/
	com_rslt += bma4xy_set_mag_write_data(
	AKM_CONTINUOUS_MEASUREMENT_MODE1);
	bma4xy_p->delay_msec(100);

	/* AKM mode address is 0x31*/
	com_rslt += bma4xy_set_mag_write_addr(AKM_POWER_MODE_REG);
	bma4xy_p->delay_msec(100);

	/* Set the AKM read xyz v_data_u8 address*/
	com_rslt += bma4xy_set_mag_read_addr(AKM_DATA_REGISTER);

	/* Enable mag interface to auto mode*/
	com_rslt += bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_DISABLE);
	bma4xy_p->delay_msec(100);

	return com_rslt;
}

/*!
 *	@brief This function used to set the AKM
 *	power mode.
 *	@note Before set the AKM power mode
 *	make sure the following two points are addressed
 *	@note	1.	Make sure the mag interface is enabled or not,
 *		by using the bma4xy_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x10
 *		to the function bma4xy_get_if_mode(0x10)
 *	@note	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bma4xy_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bma4xy_set_command_register(0x19) function.
 *
 *	@param akm_pow_mode_u8 : The value of akm power mode
 *  value   |    Description
 * ---------|--------------------
 *    0     |  AKM_POWER_DOWN_MODE
 *    1     |  AKM_SINGLE_MEAS_MODE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_bst_akm_set_powermode(u8 akm_pow_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* set mag interface manual mode*/
	if (bma4xy_p->mag_manual_enable != BMA4XY_MANUAL_ENABLE) {
		com_rslt = bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_ENABLE);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
	}
	if (BMA4XY_SUCCESS == com_rslt) {
		switch (akm_pow_mode_u8) {
		case AKM_POWER_DOWN_MODE:

			/* Set the power mode of AKM as power down mode*/
			com_rslt += bma4xy_set_mag_write_data(
						AKM_POWER_DOWN_MODE_DATA);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

			com_rslt += bma4xy_set_mag_write_addr(
							AKM_POWER_MODE_REG);

			bma4xy_p->delay_msec(BMA4XY_AUX_IF_DELAY);
		break;
		case AKM_SINGLE_MEAS_MODE:

			/* Set the power mode of AKM as
			single measurement mode*/
			com_rslt += bma4xy_set_mag_write_data(
						AKM_SINGLE_MEASUREMENT_MODE);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

			com_rslt += bma4xy_set_mag_write_addr(
							AKM_POWER_MODE_REG);
			bma4xy_p->delay_msec(BMA4XY_AUX_IF_DELAY);

			com_rslt += bma4xy_set_mag_read_addr(AKM_DATA_REGISTER);

		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
	}
	}
	/* set mag interface auto mode*/
	if (bma4xy_p->mag_manual_enable == BMA4XY_MANUAL_ENABLE) {
		com_rslt += bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_DISABLE);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
	}
	return com_rslt;
}
 /*!
 *	@brief This function used for set the magnetometer
 *	power mode of AKM09911 and AKM09912
 *	@note Before set the mag power mode
 *	make sure the following two point is addressed
 *		Make sure the mag interface is enabled or not,
 *		by using the bma4xy_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x10
 *		to the function bma4xy_set_if_mode(0x10)
 *
 *	@param mag_sec_if_pow_mode_u8 : The value of secondary if power mode
 *  value   |    Description
 * ---------|--------------------
 *    0     |  BMA4XY_MAG_FORCE_MODE
 *    1     |  BMA4XY_MAG_SUSPEND_MODE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_bst_akm_and_secondary_if_powermode(
						u8 mag_sec_if_pow_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* accel operation mode to normal*/
	com_rslt = bma4xy_set_command_register(ACCEL_MODE_NORMAL);
	bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

	if (BMA4XY_SUCCESS == com_rslt) {
		/* set mag interface manual mode*/
		if (bma4xy_p->mag_manual_enable != BMA4XY_MANUAL_ENABLE) {
			com_rslt += bma4xy_set_mag_manual_enable(
							BMA4XY_MANUAL_ENABLE);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
		}

		if (BMA4XY_SUCCESS == com_rslt) {
			switch (mag_sec_if_pow_mode_u8) {
			case BMA4XY_MAG_FORCE_MODE:

				/* set the secondary mag power mode as NORMAL*/
				com_rslt += bma4xy_set_mag_interface_normal();

				/* set the akm power mode as single
				measurement mode*/
				com_rslt += bma4xy_bst_akm_set_powermode(
							AKM_SINGLE_MEAS_MODE);
				bma4xy_p->delay_msec(BMA4XY_AUX_IF_DELAY);

				com_rslt += bma4xy_set_mag_read_addr(
							AKM_DATA_REGISTER);
				bma4xy_p->delay_msec(
						BMA4XY_GEN_READ_WRITE_DELAY);
			break;

			case BMA4XY_MAG_SUSPEND_MODE:

				/* set the akm power mode as power down mode*/
				com_rslt += bma4xy_bst_akm_set_powermode(
							AKM_POWER_DOWN_MODE);
				bma4xy_p->delay_msec(BMA4XY_AUX_IF_DELAY);

				/* set the secondary mag power mode as SUSPEND*/
				com_rslt += bma4xy_set_command_register(
							MAG_MODE_SUSPEND);
				bma4xy_p->delay_msec(BMA4XY_AUX_IF_DELAY);
			break;
			default:
				com_rslt = E_BMA4XY_OUT_OF_RANGE;
			break;
			}
		}
	}
	/* set mag interface auto mode*/
	if (bma4xy_p->mag_manual_enable == BMA4XY_MANUAL_ENABLE) {
		com_rslt += bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_DISABLE);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
	}
	return com_rslt;
}

/*!
 *	@brief This API reads magnetometer data X,Y,Z values
 *	from the register 0x0A to 0x0F
 *
 *	@brief The mag sensor data read form auxiliary mag
 *
 *  @param mag : The structure pointer to store the
 *	value of mag xyz data
 *
 *  @param sensor_select_u8 : To select the magnetometer
 *	value.
 *
 *  value    |   sensor
 *  ---------|----------------
 *   0       | BMM150
 *   1       | AKM09916
 *
 *	@note For mag data output rate configuration use the following function
 *	@note bma4xy_set_mag_output_data_rate()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_read_mag_xyz(struct bma4xy_mag_t *mag,
							u8 sensor_select_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 index;
	/* Array contains the mag XYZ lSB and MSB data
		data_u8[0] - X-LSB
		data_u8[1] - X-MSB
		data_u8[0] - Y-LSB
		data_u8[1] - Y-MSB
		data_u8[0] - Z-LSB
		data_u8[1] - Z-MSB
		*/
	u8 data_u8[BMA4XY_MAG_XYZ_DATA_SIZE+1] = {
	BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
	BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
	BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
	BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		switch (sensor_select_u8) {
		case BST_BMM:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr, BMA4XY_DATA_MAG_X_LSB_REG,
				data_u8, (BMA4XY_MAG_XYZ_DATA_LENGTH+
				READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				index = BMA4XY_MAG_X_LSB_BYTE+READ_EXTRA_BYTE;
				/*X-axis lsb value shifting*/
				data_u8[index] = BMA4XY_GET_BITSLICE(
					data_u8[index], BMA4XY_DATA_MAG_X_LSB);
				/* Data X */
				mag->x =
				(s16)((((s32)((s8)data_u8[index+1])) << 5) |
							(data_u8[index]));
				/* Data Y */
				/*Y-axis lsb value shifting*/
				data_u8[index+2] = BMA4XY_GET_BITSLICE(
						data_u8[index+2],
						BMA4XY_DATA_MAG_Y_LSB);
				mag->y =
				(s16)((((s32)((s8)data_u8[index+3])) << 5) |
							(data_u8[index+2]));
				/* Data Z */
				/*Z-axis lsb value shifting*/
				data_u8[index+4] = BMA4XY_GET_BITSLICE(
							data_u8[index+4],
							BMA4XY_DATA_MAG_Z_LSB);
				mag->z =
				(s16)((((s32)((s8)data_u8[index+5])) << 7) |
							(data_u8[index+4]));
			}
		break;
		case BST_AKM:
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr, BMA4XY_DATA_0_MAG_X_LSB_REG,
				data_u8, (BMA4XY_MAG_XYZ_DATA_LENGTH+
				READ_EXTRA_BYTE));
			if (BMA4XY_SUCCESS == com_rslt) {
				index = BMA4XY_MAG_X_LSB_BYTE+READ_EXTRA_BYTE;
				/* Data X */
				mag->x =
				(s16)((((s32)((s8)data_u8[index+1])) << 8) |
							(data_u8[index]));
				/* Data Y */
				mag->y  = ((((s32)((s8)data_u8[index+3])) << 8)
							| (data_u8[index+2]));
				/* Data Z */
				mag->z =
				(s16)((((s32)((s8)data_u8[index+5])) << 8) |
							(data_u8[index+4]));
			}
		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API read I2C interface configuration(if) mode
 * from the register 0x6B bit 4
 *
 *  @param  if_mode_u8 : The value of interface configuration mode
 *		Value |  Description
 *		----- |-----------------------------------------------------
 *		0x00  |  auxiliary interface:off
 *		0x01  |  auxiliary interface:on
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_if_mode(u8 *if_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt  = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read if mode*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_IF_CONFIG_IF_MODE_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*if_mode_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_IF_CONFIG_IF_MODE);
	}
	return com_rslt;
}

/*!
 *	@brief This API read
 *	pull up configuration from the register 0x0B bit 0 and 1
 *
 *  @param control_pullup_u8: The pointer to store the pullup
 *	configuration.
 *
 *  @note page1 should be set before calling this function
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_pullup_configuration(u8 *control_pullup_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt  = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read pull up value */
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_PULL_UP_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*control_pullup_u8 = BMA4XY_GET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_PULL_UP);
	}
	return com_rslt;

}
/*!
 * @brief This API write I2C interface configuration(if) mode
 * from the register 0x6B bit 4
 *
 *  @param  if_mode_u8 : The value of interface configuration mode
 *		Value |  Description
 *		----- |-----------------------------------------------------
 *		0x00  |  auxiliary interface:off
 *		0x01  |  auxiliary interface:on
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_if_mode(u8 if_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (if_mode_u8 <= BMA4XY_MAX_IF_MODE) {

			/* write if mode*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_IF_CONFIG_IF_MODE_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

			if (BMA4XY_SUCCESS == com_rslt) {
				data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_IF_CONFIG_IF_MODE, if_mode_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_IF_CONFIG_IF_MODE_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API switch mag interface to normal mode
 *	and confirm whether the mode switching done successfully or not
 *
 *	@return results of bus communication function and current MAG_PMU result
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_mag_interface_normal(void)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	/* aim to check the result of switching mag normal */
	u8 try_times_u8 = BMA4XY_MAG_NOAMRL_SWITCH_TIMES;
	u8 mag_pum_status_u8 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_set_command_register(MAG_MODE_NORMAL);
	bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

	if (BMA4XY_SUCCESS == com_rslt) {
		while (try_times_u8 != BMA4XY_INIT_VALUE) {
			bma4xy_switch_page(1);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
			com_rslt = bma4xy_get_mag_power_mode_stat(
							&mag_pum_status_u8);
			bma4xy_switch_page(0);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

			if (mag_pum_status_u8 == MAG_INTERFACE_PMU_ENABLE)
				break;

			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
			try_times_u8--;
		}
	if (mag_pum_status_u8 == MAG_INTERFACE_PMU_ENABLE)
		com_rslt += BMA4XY_SUCCESS;
	else
		com_rslt += E_BMA4XY_COMM_RES;
	}

	return com_rslt;
}

/*!
 *	@brief This API reads the magnetometer power mode from
 *	PMU status register 0x7D bit 0 and 1 in page1
 *
 *  @param mag_power_mode_stat_u8 : The value of mag power mode
 *	mag_powermode			|	value
 * -------------------|----------
 *	SUSPEND						|	0x00
 *	SUSPEND - ACTIVE	|	0x01
 *	ACTIVE - SUSPEND	|	0x02
 *	ACTIVE						|	0x03
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_mag_power_mode_stat(u8 *mag_power_mode_stat_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		if (BMA4XY_SUCCESS == com_rslt) {
			com_rslt += bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_MAG_POWER_MODE_STAT_REG,
				data_u8, (BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));
			if (BMA4XY_SUCCESS == com_rslt)
				*mag_power_mode_stat_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MAG_POWER_MODE_STAT);

		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the
 *	output data rate of magnetometer from the register 0x44 bit 0 to 3
 *
 *  @param  output_data_rate_u8 : Pointer to store the output data rate
 *	of Magnetometer sensor.
 *
 *  value   |    mag output data rate
 * ---------|---------------------------
 *  0x00    |BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED
 *  0x01    |BMA4XY_MAG_OUTPUT_DATA_RATE_0_78HZ
 *  0x02    |BMA4XY_MAG_OUTPUT_DATA_RATE_1_56HZ
 *  0x03    |BMA4XY_MAG_OUTPUT_DATA_RATE_3_12HZ
 *  0x04    |BMA4XY_MAG_OUTPUT_DATA_RATE_6_25HZ
 *  0x05    |BMA4XY_MAG_OUTPUT_DATA_RATE_12_5HZ
 *  0x06    |BMA4XY_MAG_OUTPUT_DATA_RATE_25HZ
 *  0x07    |BMA4XY_MAG_OUTPUT_DATA_RATE_50HZ
 *  0x08    |BMA4XY_MAG_OUTPUT_DATA_RATE_100HZ
 *  0x09    |BMA4XY_MAG_OUTPUT_DATA_RATE_200HZ
 *  0x0A    |BMA4XY_MAG_OUTPUT_DATA_RATE_400HZ
 *  0x0B    |BMA4XY_MAG_OUTPUT_DATA_RATE_800HZ
 *  0x0C    |BMA4XY_MAG_OUTPUT_DATA_RATE_1600HZ
 *  0x0D    |BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED0
 *  0x0E    |BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED1
 *  0x0F    |BMA4XY_MAG_OUTPUT_DATA_RATE_RESERVED2
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_mag_output_data_rate(u8 *output_data_rate_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the mag data output rate*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_MAG_CONFIG_OUTPUT_DATA_RATE_REG,
				data_u8, (BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
			*output_data_rate_u8 = BMA4XY_GET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_MAG_CONFIG_OUTPUT_DATA_RATE);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to get the
 *	output data rate of magnetometer from the register 0x44 bit 0 to 3
 *
 *	@param  offset_u8 : Pointer to store the offset
 *	of Magnetometer sensor.
 *
 *	@note trigger-readout offset in units of 2.5 ms. If set to zero,
 *	the offset is maximum, i.e. after readout a trigger is issued
 *	immediately
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_mag_offset(u8 *offset_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the mag data output rate*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_MAG_CONFIG_OFFSET_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt)
				*offset_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_MAG_CONFIG_OFFSET);
	}
	return com_rslt;
}

/*!
 *	@brief This API reads the status of accel data ready form the
 *	register 0x03 bit 7
 *	The status get reset when accel data register read out
 *
 *
 *	@param data_rdy_u8 :	Pointer to store the data ready interrupt
 *	status.
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_data_rdy(u8 *data_rdy_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/*reads the status of accel data ready*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_STAT_DATA_RDY_ACCEL_REG, data_u8,
				BMA4XY_READ_LENGTH+READ_EXTRA_BYTE);

		if (BMA4XY_SUCCESS == com_rslt)
			*data_rdy_u8 = BMA4XY_GET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_STAT_DATA_RDY_ACCEL);
	}
	return com_rslt;
}

/*!
 *	@brief This API reads the status of mag data ready form the
 *	register 0x03 bit 5
 *	The status get reset when accel data register read out
 *
 *	@param data_rdy_u8 :	Pointer to store the data ready interrupt
 *	status.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_mag_data_rdy(u8 *data_rdy_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt =	E_BMA4XY_NULL_PTR;
	} else {
		/*reads the status of accel data ready*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_STAT_DATA_RDY_MAG_REG, data_u8,
				BMA4XY_READ_LENGTH+READ_EXTRA_BYTE);

		if (BMA4XY_SUCCESS == com_rslt)
			*data_rdy_u8 = BMA4XY_GET_BITSLICE(
					data_u8[READ_EXTRA_BYTE],
					BMA4XY_STAT_DATA_RDY_MAG);
	}
	return com_rslt;
}

/*!
 *	@brief This function used for set the magnetometer
 *	power mode.
 *	@note
 *	Before set the mag power mode
 *	make sure the following two points are addressed
 *	@note
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bma4xy_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bma4xy_get_if_mode(0x02)
 *	@note
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bma4xy_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bma4xy_set_command_register(0x19) function.
 *
 *	@param mag_pow_mode_u8 : The value of mag power mode
 *  value    |  mode
 * ----------|------------
 *   0       | FORCE_MODE
 *   1       | SUSPEND_MODE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_set_power_mode(u8 mag_pow_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* set mag interface manual mode*/
	if (bma4xy_p->mag_manual_enable != BMA4XY_MANUAL_ENABLE) {
		com_rslt = bma4xy_set_mag_manual_enable(
							BMA4XY_MANUAL_ENABLE);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

	}

	if (BMA4XY_SUCCESS == com_rslt) {
		switch (mag_pow_mode_u8) {
		case FORCE_MODE:

			/* Set the power control bit enabled */
			com_rslt = bma4xy_bmm150_mag_wakeup();

			/* write the mag power mode as FORCE mode*/
			com_rslt += bma4xy_set_mag_write_data(
						BMA4XY_BMM150_FORCE_MODE);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

			com_rslt += bma4xy_set_mag_write_addr(
						BMA4XY_BMM150_POWER_MODE_REG);
			bma4xy_p->delay_msec(BMA4XY_AUX_IF_DELAY);

			/* To avoid the auto mode enable when manual
			mode operation running*/
			bma4xy_V_bmm150_maual_auto_condition_u8 =
			BMA4XY_MANUAL_ENABLE;

			/* set the preset mode */
			com_rslt += bma4xy_set_bmm150_mag_presetmode(
						BMA4XY_MAG_PRESETMODE_REGULAR);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

			/* To avoid the auto mode enable when manual
			mode operation running*/
			bma4xy_V_bmm150_maual_auto_condition_u8 =
							BMA4XY_MANUAL_DISABLE;

			/* set the mag read address to data registers*/
			com_rslt += bma4xy_set_mag_read_addr(
							BMA4XY_BMM150_DATA_REG);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
		break;
		case SUSPEND_MODE:

			/* Set the power mode of mag as suspend mode*/
			com_rslt = bma4xy_set_mag_write_data(
						BMA4XY_BMM150_POWER_OFF);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

			com_rslt += bma4xy_set_mag_write_addr(
					BMA4XY_BMM150_POWER_CONTROL_REG);

			bma4xy_p->delay_msec(BMA4XY_AUX_IF_DELAY);
		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}

	/* set mag interface auto mode*/
	if (bma4xy_p->mag_manual_enable == BMA4XY_MANUAL_ENABLE) {
		com_rslt += bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_DISABLE);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
	}

	return com_rslt;
}

/*!
 *	@brief This function used to set the magnetometer
 *	power mode.
 *	@note
 *	Before set the mag power mode
 *	make sure the following two point is addressed
 *		Make sure the mag interface is enabled or not,
 *		by using the bma4xy_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bma4xy_get_if_mode(0x02)
 *
 *	@param mag_sec_if_pow_mode_u8 : The value of mag power mode
 *  value    |  mode
 * ----------|------------
 *   0       | BMA4XY_MAG_FORCE_MODE
 *   1       | BMA4XY_MAG_SUSPEND_MODE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_bmm150_mag_and_secondary_if_power_mode(
						u8 mag_sec_if_pow_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	/* set the accel power mode to NORMAL*/
	com_rslt = bma4xy_set_command_register(ACCEL_MODE_NORMAL);
	bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

	/* set mag interface manual mode*/
	if (bma4xy_p->mag_manual_enable != BMA4XY_MANUAL_ENABLE) {
		com_rslt += bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_ENABLE);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
	}

	if (BMA4XY_SUCCESS == com_rslt) {
		switch (mag_sec_if_pow_mode_u8) {
		case BMA4XY_MAG_FORCE_MODE:

			/* set the secondary mag power mode as NORMAL*/
			com_rslt += bma4xy_set_mag_interface_normal();

			/* set the mag power mode as FORCE mode*/
			com_rslt += bma4xy_bmm150_mag_set_power_mode(
								FORCE_MODE);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
		break;
		case BMA4XY_MAG_SUSPEND_MODE:

			/* set the mag power mode as SUSPEND mode*/
			com_rslt += bma4xy_bmm150_mag_set_power_mode(
								SUSPEND_MODE);
			bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);

			/* set the secondary mag power mode as SUSPEND*/
			com_rslt += bma4xy_set_command_register(
							MAG_MODE_SUSPEND);
			bma4xy_p->delay_msec(BMA4XY_AUX_IF_DELAY);
		break;
		default:
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		break;
		}
	}

	if (bma4xy_p->mag_manual_enable == BMA4XY_MANUAL_ENABLE) {
		/* set mag interface auto mode*/
		com_rslt += bma4xy_set_mag_manual_enable(BMA4XY_MANUAL_DISABLE);
		bma4xy_p->delay_msec(BMA4XY_GEN_READ_WRITE_DELAY);
	}

	return com_rslt;
}
/*!
 *	@brief This function used to set the mag power control
 *	bit enable
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_bmm150_mag_wakeup(void)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 try_times_u8 = BMA4XY_BMM150_MAX_RETRY_WAKEUP;
	u8 power_control_bit_u8 = BMA4XY_INIT_VALUE;
	u8 i = BMA4XY_INIT_VALUE;

	for (i = BMA4XY_INIT_VALUE; i < try_times_u8; i++) {
		com_rslt = bma4xy_set_mag_write_data(BMA4XY_BMM150_POWER_ON);
		bma4xy_p->delay_msec(BMA4XY_BMM150_WAKEUP_DELAY1);

		/*write 0x4B register to enable power control bit*/
		com_rslt += bma4xy_set_mag_write_addr(
					BMA4XY_BMM150_POWER_CONTROL_REG);
		bma4xy_p->delay_msec(BMA4XY_BMM150_WAKEUP_DELAY2);

		com_rslt += bma4xy_set_mag_read_addr(
					BMA4XY_BMM150_POWER_CONTROL_REG);
		/* 0x04 is secondary read mag x lsb register */
		bma4xy_p->delay_msec(BMA4XY_BMM150_WAKEUP_DELAY3);

		com_rslt += bma4xy_read_reg(BMA4XY_DATA_0_ADDR,
				&power_control_bit_u8, BMA4XY_READ_LENGTH);

		power_control_bit_u8 =
			BMA4XY_BMM150_SET_POWER_CONTROL & power_control_bit_u8;

		if (power_control_bit_u8 == BMA4XY_BMM150_POWER_ON)
			break;
	}
	com_rslt = (i >= try_times_u8) ? BMA4XY_BMM150_POWER_ON_FAIL :
						BMA4XY_BMM150_POWER_ON_SUCCESS;
	return com_rslt;
}

/*!
 *	@brief This API is used to read
 *	I2C device address of auxiliary mag from the register 0x4B bit 1 to 7
 *
 *  @param i2c_device_addr_u8 : The value of mag I2C device address
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_i2c_device_addr(u8 *i2c_device_addr_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = E_BMA4XY_COMM_RES;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the mag I2C device address*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_I2C_DEVICE_ADDR_REG, data_u8,
				BMA4XY_READ_LENGTH+READ_EXTRA_BYTE);

		if (BMA4XY_SUCCESS == com_rslt)
			*i2c_device_addr_u8 = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_I2C_DEVICE_ADDR);
	}

	return com_rslt;
}

/*!
 *	@brief This API is used to read the micro controller
 *	status from the register 0x5F bit 0 to 7
 *
 *  @param uc_status : Structure pointer used to store the
 *	micro controller status read from the sensor.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_uc_status(
struct bma4xy_uc_status *uc_status)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = E_BMA4XY_COMM_RES;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read the mag I2C device address*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_STATUS, data_u8,
					BMA4XY_READ_LENGTH+READ_EXTRA_BYTE);

		if (BMA4XY_SUCCESS == com_rslt) {
			uc_status->sleep = (data_u8[READ_EXTRA_BYTE] & 0x01);
			uc_status->irq_ovm =
				((data_u8[READ_EXTRA_BYTE] & 0x02) >> 0x01);
			uc_status->wc_event =
				((data_u8[READ_EXTRA_BYTE] & 0x04) >> 0x02);
			uc_status->wc_event =
				((data_u8[READ_EXTRA_BYTE] & 0x08) >> 0x03);
		}
	}

	return com_rslt;
}

/*!
 *	@brief This API is used to enable or disable the offset compensation
 *	from the register 0x70 bit 3
 *
 *  @param offset_en : indicates whether accel offset can be enabled
 *	or disabled.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_offset_comp(u8 offset_en)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_NV_ACCEL_OFFSET_REG, data_u8,
					(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {

			/* write accel fifo filter data */
			data_u8[READ_EXTRA_BYTE] = BMA4XY_SET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_NV_ACCEL_OFFSET,
						offset_en);

			com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_NV_ACCEL_OFFSET_REG,
						&data_u8[READ_EXTRA_BYTE],
						BMA4XY_WRITE_LENGTH);
		}
	}

	return com_rslt;
}

/*!
 *	@brief This API is used to get the status of the offset compensation
 *	of accelerometer from the register 0x70 bit 3
 *
 *  @param offset_en : pointer used to indicate whether accel offset is
 *	enabled or disabled.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_offset_comp(u8 *offset_en)
{

	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};

	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {

		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_NV_ACCEL_OFFSET_REG, data_u8,
				(BMA4XY_READ_LENGTH+READ_EXTRA_BYTE));

		if (BMA4XY_SUCCESS == com_rslt) {

			/* write accel fifo filter data */
			*offset_en = BMA4XY_GET_BITSLICE(
						data_u8[READ_EXTRA_BYTE],
						BMA4XY_NV_ACCEL_OFFSET);
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to validate the user input for
 *	accelerometer offset compensation.
 *
 *  @param g_sign : indicates the g_sign of the sensor kept
 *	for accelerometer offset procedure
 *	@param axis : indicates the axis of the sensor
 *	which is in 1g or -1g position
 *
 *	@return results user input validation
 *	@retval 0 -> Success
 *	@retval -2 -> Error
 *
*/
BMA4XY_RETURN_TYPE validate_user_input(
int *gvalue) {

	u8 index = 0;
	int min_gval = -1;
	int max_gval = 1;

	while (index < 3) {
		if (gvalue[index] >= min_gval && gvalue[index] <= max_gval)
				index++;
		else
			return E_BMA4XY_OUT_OF_RANGE;
	}
	return BMA4XY_SUCCESS;

}

/*!
 *	@brief This function used to configure foc
 *	for accelerometer.
 *	@param	g_value : array which stores the accelerometer g units
 *	of x,y and z axis.
 *
 *	Variable		|	Description
 *	------------|-----------------------
 *	gvalue[0]		|	x axis g units
 *	gvalue[1]		|	y axis g units
 *	gvalue[2]		|	z axis g units
 *
 *	@note	The FOC procedure of the accelerometer is below
 *	Ensure sensor is not moving
 *	Ensure one axis to be aligned with gravitational field
 *	Set ACC_RANGE to the range in which BMy42 is used in the device
 *	Save ACC_CONF, PWR_CTRL.acc_en and PWR_CONF.adv_power_save
 *	Disable offset compensation that might be already in place.
 *	Set accel configuration to: 50Hz, CIC, no advanced power save mode.
 *	Switch accel to normal mode: PWR_CTRL.acc_en=0b1 and
 *	PWR_CONF.adv_power_save=0
 *	Measure data
 *	Read out x y z values
 *	Compensate for gravity for axis in line with gravity ( see 2. ) to zero
 *	Scale the values to the number format defined for the Register
 *	OFFSET_0.off_acc_x
 *	Invert all values (i.e. multiply with -1 ).
 *	Write values to offset registers
 *	Enable offset compensation (set NV_CONF.acc_off_en=1)
 *	Restore save settings of ACC_CONF, PWR_CTRL.acc_en, and
 *	PWR_CONF.adv_power_save
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval Any Negative Value -> Error
*/
BMA4XY_RETURN_TYPE bma4xy_configure_accel_foc(int g_value[3])
{

	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	struct bma4xy_accel_t accel_data = {BMA4XY_INIT_VALUE};
	struct bma4xy_accel_offset_t offset_axis = {BMA4XY_INIT_VALUE};
	struct accel_data_diff accel_value_diff;
	struct bma4xy_acc_conf acc_conf = {BMA4XY_INIT_VALUE};
	u8 accel_config = BMA4XY_ACCEL_CONFIG_FOC;
	u8 accel_en = BMA4XY_INIT_VALUE;
	u8 advance_power_save = BMA4XY_INIT_VALUE;
	u8 drdy = BMA4XY_INIT_VALUE;
	u8 shifter = BMA4XY_INIT_VALUE;
	u8 range = BMA4XY_INIT_VALUE;
	s8 try = BMA4XY_INIT_VALUE;
	s16 ideal_value_axis[3] = {BMA4XY_INIT_VALUE};
	u8 resolution = BMA4XY_INIT_VALUE;
	u8 index;

	/* scaling factor for 12, 14, 16 bit resolution */
	s8 scaling_factor[BMA4XY_RESOLUTION]
	[BMA4XY_RANGE] = {{2, 1, 0, -1}, {4, 3, 2, 1}, {6, 5, 4, 3} };

/* for 16 bit resolution */
#ifdef BMA480 /*imaginary variant - will be changed*/
	u16 counts_g[BMA4XY_RANGE] = {16384, 8192, 4096, 2048};

	resolution = 2;
/* for 14 bit resolution */
#elif defined(BMA454) || defined(BMA456) || defined(BMA455)
	u16 counts_g[BMA4XY_RANGE] = {4096, 2048, 1024, 512};

	resolution = 1;
/* for 12 bit resolution */
#elif defined(BMA420) || defined(BMA421) || defined(BMA422) \
|| defined(BMA423)
	u16 counts_g[BMA4XY_RANGE] = {1024, 512, 256, 128};

	resolution = 0;
#else
	u16 counts_g[BMA4XY_RANGE] = {16384, 8192, 4096, 2048};

	resolution = 2;
#endif

	/* used to validate user input*/
	com_rslt = validate_user_input(g_value);

	if (BMA4XY_SUCCESS == com_rslt) {
		/* register configurations for offset Calibration*/
		com_rslt += bma4xy_get_accel_range(&range);

		if (BMA4XY_SUCCESS ==  com_rslt) {

			/* for saving accel configuration,
			accelerometer enable status, advance power save*/
			com_rslt += bma4xy_get_accel_conf(&acc_conf);
			com_rslt += bma4xy_get_accel_enable(&accel_en);
			com_rslt += bma4xy_get_advance_power_save(
							&advance_power_save);

			/* disabling offset compensation tat is in place*/
			com_rslt += bma4xy_set_offset_comp(BMA4XY_DISABLE);

			if (BMA4XY_SUCCESS == com_rslt) {

				/* Set accel config to 50Hz, CIC,
				no under sampling */
				com_rslt += bma4xy_write_reg(
					BMA4XY_ACCEL_CONFIG_ADDR, &accel_config,
					BMA4XY_GEN_READ_WRITE_DATA_LENGTH);

				if (BMA4XY_SUCCESS == com_rslt) {
					/* Switch accel to normal mode and
					advance power save to zero*/
					com_rslt += bma4xy_set_accel_enable(
								BMA4XY_ENABLE);
					com_rslt +=
						bma4xy_set_advance_power_save(
								BMA4XY_DISABLE);

		if (BMA4XY_SUCCESS == com_rslt) {
			try = 0x20;

	/* Accel data Measurement */
	/* Timeout based exit */
	while ((drdy == 0) && (try--)) {
		com_rslt += bma4xy_get_accel_data_rdy(&drdy);

		if (drdy == BMA4XY_TRUE)
			com_rslt += bma4xy_read_accel_xyz(&accel_data);
	}
	if ((try > 0) && (BMA4XY_SUCCESS == com_rslt)) {

		for (index = 0; index < 3; index++)
			ideal_value_axis[index] =
					(counts_g[range] * g_value[index]);

	accel_value_diff.x.val = accel_data.x - ideal_value_axis[BMA4XY_X_AXIS];
	accel_value_diff.y.val = accel_data.y - ideal_value_axis[BMA4XY_Y_AXIS];
	accel_value_diff.z.val = accel_data.z - ideal_value_axis[BMA4XY_Z_AXIS];

	if (accel_value_diff.x.val < 0) {
			accel_value_diff.x.val = ABS(accel_value_diff.x.val);
			accel_value_diff.x.is_negative = 1;

	}
	if (accel_value_diff.y.val < 0) {
			accel_value_diff.y.val = ABS(accel_value_diff.y.val);
			accel_value_diff.y.is_negative = 1;
	}
	if (accel_value_diff.z.val < 0) {
			accel_value_diff.z.val = ABS(accel_value_diff.z.val);
			accel_value_diff.z.is_negative = 1;
	}

	/* Data register resolution less than or equal to  3.9 mg */
	if (scaling_factor[resolution][range] > 0) {

		/* scale according to offset register resolution*/
		offset_axis.x = ((accel_value_diff.x.val +
			(1<<((u8)(scaling_factor[resolution][range] - 1)))) >>
			((u8)scaling_factor[resolution][range]));

		offset_axis.y = ((accel_value_diff.y.val +
			(1<<((u8)(scaling_factor[resolution][range] - 1)))) >>
			((u8)scaling_factor[resolution][range]));

		offset_axis.z = ((accel_value_diff.z.val +
			(1<<((u8)(scaling_factor[resolution][range] - 1)))) >>
			((u8)scaling_factor[resolution][range]));

	} else if (scaling_factor[resolution][range] < 0) {

		shifter = scaling_factor[resolution][range] + 2;
		offset_axis.x = (accel_value_diff.x.val << (shifter));
		offset_axis.y = (accel_value_diff.y.val << (shifter));
		offset_axis.z = (accel_value_diff.z.val << (shifter));

	} else {
		/* scale according to offset register resolution*/
		offset_axis.x = (u8)(accel_value_diff.x.val);
		offset_axis.y = (u8)(accel_value_diff.y.val);
		offset_axis.z = (u8)(accel_value_diff.z.val);
	}
	/* for handling negative offset */
	/* Employing twos's Complement method*/
	if (accel_value_diff.x.is_negative == BMA4XY_TRUE) {
		offset_axis.x = ~offset_axis.x;
		offset_axis.x += 1;
	}
	if (accel_value_diff.y.is_negative == BMA4XY_TRUE) {
		offset_axis.y = ~offset_axis.y;
		offset_axis.y +=	1;
	}
	if (accel_value_diff.z.is_negative == BMA4XY_TRUE) {
		offset_axis.z = ~offset_axis.z;
		offset_axis.z += 1;
	}

	/* this step can be ignored with the above logic*/
	/* multiply with -1 according to step 12 */
	offset_axis.x = (offset_axis.x) * (-1);
	offset_axis.y = (offset_axis.y) * (-1);
	offset_axis.z = (offset_axis.z) * (-1);

	/* offset Values   written in offset register */
	com_rslt += bma4xy_write_reg(BMA4XY_OFFSET_0_ACCEL_OFF_X_REG,
		(u8 *)&offset_axis.x, BMA4XY_GEN_READ_WRITE_DATA_LENGTH);

	com_rslt += bma4xy_write_reg(BMA4XY_OFFSET_1_ACCEL_OFF_Y_REG,
		(u8 *)&offset_axis.y, BMA4XY_GEN_READ_WRITE_DATA_LENGTH);

	com_rslt += bma4xy_write_reg(BMA4XY_OFFSET_2_ACCEL_OFF_Z_REG,
		(u8 *)&offset_axis.z, BMA4XY_GEN_READ_WRITE_DATA_LENGTH);

	/* enable offset compensation */
	com_rslt += bma4xy_set_offset_comp(BMA4XY_ENABLE);

	/* restore settings of accel conf, power ctrl,advance power save */
	com_rslt += bma4xy_set_accel_output_data_rate(
		acc_conf.acc_odr, 0x04);

	com_rslt += bma4xy_set_accel_bw(acc_conf.acc_bwp);
	com_rslt += bma4xy_set_accel_enable(accel_en);
	com_rslt += bma4xy_set_advance_power_save(advance_power_save);
	} else {
		com_rslt = ERROR;
	}
	}
	}
	}
	}
	}
	return com_rslt;
}

/*!
 * @brief This API read accel select axis to be self-test
 *
 *  @param accel_selftest_axis_u8 :
 *	The value of accel self test axis selection
 *  Value  |  Description
 * --------|-------------
 *   0x00  | disabled
 *   0x01  | x-axis
 *   0x02  | y-axis
 *   0x03  | z-axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_selftest_enable(u8 *accel_selftest_axis_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt  = BMA4XY_INIT_VALUE;
	u8 data_u8 = BMA4XY_INIT_VALUE;
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read accel self test axis*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_ACCEL_SELFTEST_ENABLE_REG, &data_u8,
				BMA4XY_READ_LENGTH);

		*accel_selftest_axis_u8 = BMA4XY_GET_BITSLICE(data_u8,
						BMA4XY_ACCEL_SELFTEST_ENABLE);
		}
	return com_rslt;
}
/*!
 * @brief This API write accel select axis to be self-test
 *
 *  @param accel_selftest_axis_u8 :
 *	The value of accel self test axis selection
 *  Value  |  Description
 * --------|-------------
 *   0x00  | disabled
 *   0x01  | enable
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_selftest_enable(u8 accel_selftest_axis_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8 = BMA4XY_INIT_VALUE;
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (accel_selftest_axis_u8 <= BMA4XY_MAX_ACCEL_SELFTEST_AXIS) {
			/* write accel self test axis*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr,
				BMA4XY_ACCEL_SELFTEST_ENABLE_REG, &data_u8,
				BMA4XY_READ_LENGTH);

			if (com_rslt == BMA4XY_SUCCESS) {
				data_u8 = BMA4XY_SET_BITSLICE(data_u8,
						BMA4XY_ACCEL_SELFTEST_ENABLE,
						accel_selftest_axis_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_ACCEL_SELFTEST_ENABLE_REG,
					&data_u8, BMA4XY_WRITE_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API read accel self test axis sign
 *	from the register 0x6D bit 2
 *
 *  @param accel_selftest_sign_u8: The value of accel self test axis sign
 *  Value  |  Description
 * --------|-------------
 *   0x00  | negative
 *   0x01  | positive
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_selftest_sign(u8 *accel_selftest_sign_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt  = BMA4XY_INIT_VALUE;
	u8 data_u8 = BMA4XY_INIT_VALUE;
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read accel self test axis sign*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_ACCEL_SELFTEST_SIGN_REG, &data_u8,
				BMA4XY_READ_LENGTH);

		*accel_selftest_sign_u8 = BMA4XY_GET_BITSLICE(data_u8,
						BMA4XY_ACCEL_SELFTEST_SIGN);
	}
	return com_rslt;
}
/*!
 *	@brief This API write accel self test axis sign
 *	from the register 0x6D bit 2
 *
 *  @param accel_selftest_sign_u8: The value of accel self test axis sign
 *  Value  |  Description
 * --------|-------------
 *   0x00  | negative
 *   0x01  | positive
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_selftest_sign(u8 accel_selftest_sign_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8 = BMA4XY_INIT_VALUE;
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (accel_selftest_sign_u8 <= BMA4XY_MAX_VALUE_SELFTEST_SIGN) {

			/* write accel self test axis sign*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_ACCEL_SELFTEST_SIGN_REG,
						&data_u8, BMA4XY_READ_LENGTH);

			if (com_rslt == BMA4XY_SUCCESS) {
				data_u8 = BMA4XY_SET_BITSLICE(data_u8,
						BMA4XY_ACCEL_SELFTEST_SIGN,
						accel_selftest_sign_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
						bma4xy_p->dev_addr,
						BMA4XY_ACCEL_SELFTEST_SIGN_REG,
						&data_u8, BMA4XY_WRITE_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API read accel self test amplitude
 *	from the register 0x6D bit 3
 *        select amplitude of the selftest deflection:
 *
 *  @param accel_selftest_amp_u8 : The value of accel self test amplitude
 *  Value  |  Description
 * --------|-------------
 *   0x00  | LOW
 *   0x01  | HIGH
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_accel_selftest_amp(
u8 *accel_selftest_amp_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt  = BMA4XY_INIT_VALUE;
	u8 data_u8 = BMA4XY_INIT_VALUE;
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* read  self test amplitude*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr, BMA4XY_SELFTEST_AMP_REG,
				&data_u8, BMA4XY_READ_LENGTH);

		*accel_selftest_amp_u8 = BMA4XY_GET_BITSLICE(
						data_u8, BMA4XY_SELFTEST_AMP);
	}
	return com_rslt;
}
/*!
 *	@brief This API write accel self test amplitude
 *	from the register 0x6D bit 3
 *        select amplitude of the selftest deflection:
 *
 *  @param accel_selftest_amp_u8 : The value of accel self test amplitude
 *  Value  |  Description
 * --------|-------------
 *   0x00  | LOW
 *   0x01  | HIGH
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_selftest_amp(u8 accel_selftest_amp_u8)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8 = BMA4XY_INIT_VALUE;
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		if (accel_selftest_amp_u8 <= BMA4XY_MAX_VALUE_SELFTEST_AMP) {
			/* write  self test amplitude*/
			com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(
				bma4xy_p->dev_addr, BMA4XY_SELFTEST_AMP_REG,
				&data_u8, BMA4XY_READ_LENGTH);

			if (com_rslt == BMA4XY_SUCCESS) {
				data_u8 = BMA4XY_SET_BITSLICE(data_u8,
						BMA4XY_SELFTEST_AMP,
						accel_selftest_amp_u8);

				com_rslt += bma4xy_p->BMA4XY_BUS_WRITE_FUNC(
					bma4xy_p->dev_addr,
					BMA4XY_SELFTEST_AMP_REG, &data_u8,
					BMA4XY_GEN_READ_WRITE_DATA_LENGTH);
			}
		} else {
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to check whether the self test
 *	functionality of the sensor is working or not
 *	@param	result : used to store the result of self test
 *
 *	@return results of self test
 *	@retval 0 -> Success
 *	@retval 1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_perform_accel_selftest(u8 *result)
{

	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	struct bma4xy_accel_t accel_data_positive = {BMA4XY_INIT_VALUE};
	struct bma4xy_accel_t accel_data_negative = {BMA4XY_INIT_VALUE};
	struct bma4xy_selftest_accel_t accel_data_diff = {BMA4XY_INIT_VALUE};

	*result = BMA4XY_SELFTEST_FAIL;
	com_rslt += bma4xy_set_accel_selftest_config();

	bma4xy_p->delay_msec(20);
	com_rslt += bma4xy_selftest_config(BMA4XY_ENABLE);

	if (BMA4XY_SUCCESS == com_rslt) {
		bma4xy_p->delay_msec(100);
		com_rslt += bma4xy_read_accel_xyz(&accel_data_positive);

		com_rslt += bma4xy_selftest_config(BMA4XY_DISABLE);

		if (BMA4XY_SUCCESS == com_rslt) {
			bma4xy_p->delay_msec(100);
			com_rslt += bma4xy_read_accel_xyz(&accel_data_negative);

			accel_data_diff.x = ABS(accel_data_positive.x) +
						ABS(accel_data_negative.x);
			accel_data_diff.y = ABS(accel_data_positive.y) +
						ABS(accel_data_negative.y);
			accel_data_diff.z = ABS(accel_data_positive.z) +
						ABS(accel_data_negative.z);

			com_rslt += bma4xy_validate_selftest(accel_data_diff);
			if (BMA4XY_SUCCESS == com_rslt)
				*result = BMA4XY_SELFTEST_PASS;

			/* triggers a reset */
			com_rslt += bma4xy_set_command_register(0xB6);
			bma4xy_p->delay_msec(200);
		}

	}
	return com_rslt;

}
/*!
 *	@brief This function is used to enable the selftest, sign
 *	and amplitude of the sensor.
 *
 *	@return results of self test
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_selftest_config(u8 sign)
{

	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_set_accel_selftest_enable(BMA4XY_ENABLE);
	com_rslt += bma4xy_set_accel_selftest_sign(sign);
	com_rslt += bma4xy_set_accel_selftest_amp(BMA4XY_ENABLE);

	return com_rslt;


}
/*!
 *	@brief This function is used to enable the accel sensor
 *	and configure the accelerometer according to self test
 *	procedure.
 *
 *	@return results of self test
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_accel_selftest_config(void)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	com_rslt += bma4xy_set_accel_enable(BMA4XY_ENABLE);
	com_rslt += bma4xy_set_accel_range(BMA4XY_ACCEL_RANGE_8G);
	com_rslt += bma4xy_set_accel_under_sampling_parameter(BMA4XY_ENABLE);
	com_rslt += bma4xy_set_accel_bw(BMA4XY_ACCEL_NORMAL_AVG4);
	com_rslt += bma4xy_set_accel_output_data_rate(
				BMA4XY_ACCEL_OUTPUT_DATA_RATE_1600HZ,
				BMA4XY_ACCEL_NORMAL_AVG4);

	return com_rslt;

}

/*!
 *	@brief This function is used to validate the self test
 *	data of the accelerometer.
 *
 *	@return results of self test
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_validate_selftest(struct bma4xy_selftest_accel_t
								accel_data_diff)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

#ifdef BMA480
	if (accel_data_diff.x > 4000 && accel_data_diff.y
					> 4000 && accel_data_diff.z > 2000)
#elif defined(BMA454) || defined(BMA455) || defined(BMA456)
	if (accel_data_diff.x > 1020 && accel_data_diff.y
					> 1020 && accel_data_diff.z > 510)
#elif defined(BMA421) || defined(BMA420) || defined(BMA422)
	if (accel_data_diff.x > 205 && accel_data_diff.y
					> 205 && accel_data_diff.z > 103)
#else
	if (accel_data_diff.x > 4000 && accel_data_diff.y
					> 4000 && accel_data_diff.z > 2000)

#endif
		com_rslt = BMA4XY_SUCCESS;
	else
		com_rslt = ERROR;

	return com_rslt;

}

#if defined(BMA421) || defined(BMA422) || defined(BMA420) || defined(BMA455)

/*!
 *	@brief This function is used to remap_axes
 *	of the sensor
 *
 *	@param axis_remap_data : pointer used to store
 *  configuration data for axes remapping
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/

BMA4XY_RETURN_TYPE bma4xy_remap_axes(struct bma4xy_axes_remap *axis_remap_data)
{

	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (com_rslt == BMA4XY_SUCCESS) {

		index = (BMA4XY_AXES_REMAP_OFFSET * 2);

		feature_config[index] = ((axis_remap_data->map_x_axis &
					BMA4XY_MASK_X_AXIS) |
					((axis_remap_data->map_x_axis_sign << 2)
					& BMA4XY_MASK_X_AXIS_SIGN) |
					((axis_remap_data->map_y_axis << 3) &
					BMA4XY_MASK_Y_AXIS) |
					((axis_remap_data->map_y_axis_sign << 5)
					& BMA4XY_MASK_Y_AXIS_SIGN) |
					((axis_remap_data->map_z_axis << 6) &
					BMA4XY_MASK_Z_AXIS));

		feature_config[index + 1] = (axis_remap_data->map_z_axis_sign &
						BMA4XY_MASK_Z_AXIS_SIGN);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	}

	return com_rslt;
}

/*!
 *	@brief This function is used to enable
 *	the axis of any motion feature in the sensor
 *
 *
 *  @param axis :	enum variable used to set the
 *	axis of the any motion feature in
 *	the sensor.
 *
 *  value      |  axis
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE_ALL_AXIS
 *    0x01     |  BMA4XY_X_AXIS_ENABLE
 *    0x02     |  BMA4XY_Y_AXIS_ENABLE
 *    0x04     |  BMA4XY_Z_AXIS_ENABLE
 *    0x07     |  BMA4XY_ENABLE_ALL_AXIS
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_anymotion_enable_axis(enum bma4xy_axis_enable axis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	if (BMA4XY_SUCCESS == com_rslt) {

		index = (BMA4XY_ANY_MOTION_OFFSET * 2) +
					BMA4XY_ANY_MOTION_ENABLE_ALL_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_ANY_MOTION_ENABLE_ALL, axis);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the threshold
 *	value of any motion feature in the sensor
 *
 *
 *  @param anymotion_threshold :	variable used to set the
 *	threshold value of the any motion feature in
 *	the sensor.
 *
 *	@note valid values are from 0 to 2047
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_anymotion_set_threshold(u16 anymotion_threshold)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_ANY_MOTION_THRESHOLD_REG * 2) +
					BMA4XY_ANY_MOTION_THRESHOLD_BYTE;

		data_u16 = (u16)((feature_config[index] |
					(feature_config[index + 1] << 8)));

		data_u16 = BMA4XY_SET_BITSLICE(data_u16,
			BMA4XY_ANY_MOTION_THRESHOLD, anymotion_threshold);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
			BMA4XY_UC_DMA_DATA, feature_config,
			BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the threshold
 *	of any motion feature in the sensor
 *
 *  @param anymotion_threshold :	pointer used to store the
 *	threshold value of the any motion feature set in
 *	the sensor.
 *
 *	@note valid values are from 0 to 2047
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_anymotion_get_threshold(u16 *anymotion_threshold)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = (BMA4XY_ANY_MOTION_THRESHOLD_REG * 2) +
					BMA4XY_ANY_MOTION_THRESHOLD_BYTE;

		data_u16 = (u16)((feature_config[index] |
					(feature_config[index+1] << 8)));

		*anymotion_threshold = BMA4XY_GET_BITSLICE(data_u16,
						BMA4XY_ANY_MOTION_THRESHOLD);

	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the duration
 *	value of any motion feature in the sensor
 *
 *  @param anymotion_duration :	variable used to set the
 *	duration value of the any motion feature in
 *	the sensor.
 *
 *	@note valid values are from 0 to 255
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_anymotion_set_duration(u16 anymotion_duration)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ANY_MOTION_DURATION_REG) * 2) +
						BMA4XY_ANY_MOTION_DURATION_BYTE;

		data_u16 = (u16)((feature_config[index] |
					(feature_config[index + 1] << 8)));

		data_u16 = BMA4XY_SET_BITSLICE(data_u16,
				BMA4XY_ANY_MOTION_DURATION, anymotion_duration);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);

		feature_config[index+1] =
			(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the duration
 *	of any motion feature in the sensor
 *
 *
 *  @param anymotion_duration :	pointer used to store the
 *	duration value of the anymotion feature set in
 *	the sensor.
 *
 *	@note valid values are from 0 to 255
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_anymotion_get_duration(
u16 *anymotion_duration)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ANY_MOTION_DURATION_REG) * 2) +
						BMA4XY_ANY_MOTION_DURATION_BYTE;

		data_u16 = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

		*anymotion_duration = BMA4XY_GET_BITSLICE(data_u16,
						BMA4XY_ANY_MOTION_DURATION);
	}
	return com_rslt;
}

/*!
 *	@brief This function is used to set
 *	no motion or any motion feature in the sensor
 *
 *
 *  @param selection :	variable to select no motion
 *	or any motion feature in the sensor
 *
 *	value    |	selection
 * ----------|---------------
 *  0x01     |  BMA4XY_NO_MOTION
 *  0x00     |  BMA4XY_ANY_MOTION
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_anymotion_nomotion_selection(u8 selection)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	if (SUCCESS == com_rslt) {

		index = ((BMA4XY_NOMOTION_SELECTION_REG) * 2) +
						BMA4XY_NOMOTION_SELECTION_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_NOMOTION_SELECTION, selection);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}
#endif
#if defined(BMA421) || defined(BMA422) || defined(BMA455)
/*!
 *	@brief This function is used to enable or disable the step
 *	counter in the sensor
 *
 *
 *  @param enable : used to enable or disable step counter
 *
 *  value      |  enable
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE
 *    0x01     |  BMA4XY_ENABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_step_counter_enable(u8 enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_STEP_COUNTER_ENABLE_REG * 2) +
						BMA4XY_STEP_COUNTER_ENABLE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_STEP_COUNTER_ENABLE, enable);


		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to enable or disable the step
 *	detector in the sensor
 *
 *
 *  @param enable : used to enable or disable step detector
 *
 *  value      |  enable
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE
 *    0x01     |  BMA4XY_ENABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_step_detector_enable(u8 enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_STEP_DETECTOR_ENABLE_REG * 2) +
					BMA4XY_STEP_DETECTOR_ENABLE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_STEP_DETECTOR_ENABLE, enable);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the water mark level
 *	for step counter interrupt in the sensor
 *
 *
 *  @param step_counter_wm : used to specify the value of
 *	water mark level count.
 *
 *	@note valid values are from 1 to 1023
 *	@note value 0 is used for step detector interrupt
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_step_counter_set_watermark(u16 step_counter_wm)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_STEP_COUNTER_WATERMARK_REG * 2) +
					BMA4XY_STEP_COUNTER_WATERMARK_BYTE;

		data_u16 = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

		data_u16 = BMA4XY_SET_BITSLICE(data_u16,
				BMA4XY_STEP_COUNTER_WATERMARK, step_counter_wm);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index + 1] = (u8)((data_u16 &
						BMA4XY_SET_HIGH_BYTE) >> 8);


		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the water mark level
 *	set for step counter interrupt in the sensor
 *
 *
 *  @param step_counter_wm : pointer used to store the
 *	water mark level set in the sensor.
 *
 *	@note valid values are from 1 to 1023
 *	@note value 0 is used for step detector interrupt
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_step_counter_get_watermark(u16 *step_counter_wm)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_STEP_COUNTER_WATERMARK_REG * 2) +
					BMA4XY_STEP_COUNTER_WATERMARK_BYTE;

		data_u16 = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

		*step_counter_wm = BMA4XY_GET_BITSLICE(data_u16,
						BMA4XY_STEP_COUNTER_WATERMARK);

	}

	return com_rslt;
}
/*!
 *	@brief This function is used to reset  the counted steps
 *	of the step counter.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_reset_step_counter(void)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 data_u8 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_STEP_COUNTER_RESET_REG * 2) +
						BMA4XY_STEP_COUNTER_RESET_BYTE;

		data_u8 = BMA4XY_SET_BITSLICE(feature_config[index],
						BMA4XY_STEP_COUNTER_RESET, 1);

		feature_config[index] = data_u8;

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the no of
 *	counted steps from the step counter sensor.
 *
 *
 *  @param step_count : specifies the number of steps counted
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_step_counter_output(u32 *step_count)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[5] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
			BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE,
			BMA4XY_INIT_VALUE};

	com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
			BMA4XY_GPIO_0_REG, data_u8,
			BMA4XY_STEP_COUNTER_DATA_SIZE+READ_EXTRA_BYTE);

	if (BMA4XY_SUCCESS == com_rslt)
		*step_count = (u32)(data_u8[0 + READ_EXTRA_BYTE] |
					(data_u8[1 + READ_EXTRA_BYTE] << 8) |
					(data_u8[2 + READ_EXTRA_BYTE] << 16) |
					(data_u8[3 + READ_EXTRA_BYTE] << 24));


	return com_rslt;
}

/*!
 *	@brief This function is used to get the parameter
 *	of the step counter.
 *
 *	@settings  : enum variable used to specify the desired parameter
 *	to get.
 *
 *	value	|  setting
 *	--------|-------------------------------
 *	2	|  BMA4XY_STEPCOUNTER_SETTING_2
 *	3	|  BMA4XY_STEPCOUNTER_SETTING_3
 *	4	|  BMA4XY_STEPCOUNTER_SETTING_4
 *	5	|  BMA4XY_STEPCOUNTER_SETTING_5
 *	6	|  BMA4XY_STEPCOUNTER_SETTING_6
 *	7	|  BMA4XY_STEPCOUNTER_SETTING_7
 *	8	|  BMA4XY_STEPCOUNTER_SETTING_8
 *
 *	@parameter : variable used to get the value of parameter
 *	of the step counter requested by user.
 *
 *	@return results of communication result
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
*/
BMA4XY_RETURN_TYPE bma4xy_stepcounter_get_parameter(
		enum bma4xy_stepcounter_settings setting, u16 *parameter)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	if ((setting >= BMA4XY_STEPCOUNTER_SETTING_2) &&
	(setting <= BMA4XY_STEPCOUNTER_SETTING_8)) {

		com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

		switch (setting) {

		case BMA4XY_STEPCOUNTER_SETTING_2:
			index = (BMA4XY_STEP_COUNTER_PARAMETER2_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER2_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_3:
			index = (BMA4XY_STEP_COUNTER_PARAMETER3_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER3_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_4:
			index = (BMA4XY_STEP_COUNTER_PARAMETER4_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER4_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_5:
			index = (BMA4XY_STEP_COUNTER_PARAMETER5_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER5_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_6:
			index = (BMA4XY_STEP_COUNTER_PARAMETER6_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER6_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_7:
			index = (BMA4XY_STEP_COUNTER_PARAMETER7_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER7_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_8:
			index = (BMA4XY_STEP_COUNTER_PARAMETER8_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER8_BYTE;
		break;
		}

		*parameter = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

	} else {
			*parameter = BMA4XY_INVALID;
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the parameter
 *	of the step counter.
 *
 *	@settings  : enum variable used to specify the desired parameter
 *	to set.
 *
 *	value	|  setting
 *	--------|-------------------------------
 *	2	|  BMA4XY_STEPCOUNTER_SETTING_2
 *	3	|  BMA4XY_STEPCOUNTER_SETTING_3
 *	4	|  BMA4XY_STEPCOUNTER_SETTING_4
 *	5	|  BMA4XY_STEPCOUNTER_SETTING_5
 *	6	|  BMA4XY_STEPCOUNTER_SETTING_6
 *	7	|  BMA4XY_STEPCOUNTER_SETTING_7
 *	8	|  BMA4XY_STEPCOUNTER_SETTING_8
 *
 *	@parameter : variable used to set the value of parameter
 *	of the step counter.
 *
 *	@return results of communication result
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
*/
BMA4XY_RETURN_TYPE bma4xy_stepcounter_set_parameter(
			enum bma4xy_stepcounter_settings setting, u16 parameter)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	if ((setting >= BMA4XY_STEPCOUNTER_SETTING_2) &&
		(setting <= BMA4XY_STEPCOUNTER_SETTING_8)) {

		com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
			BMA4XY_UC_DMA_DATA, feature_config,
			BMA4XY_FEATURE_SIZE);

		switch (setting) {

		case BMA4XY_STEPCOUNTER_SETTING_2:
			index = (BMA4XY_STEP_COUNTER_PARAMETER2_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER2_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_3:
			index = (BMA4XY_STEP_COUNTER_PARAMETER3_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER3_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_4:
			index = (BMA4XY_STEP_COUNTER_PARAMETER4_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER4_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_5:
			index = (BMA4XY_STEP_COUNTER_PARAMETER5_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER5_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_6:
			index = (BMA4XY_STEP_COUNTER_PARAMETER6_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER6_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_7:
			index = (BMA4XY_STEP_COUNTER_PARAMETER7_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER7_BYTE;
		break;

		case BMA4XY_STEPCOUNTER_SETTING_8:
			index = (BMA4XY_STEP_COUNTER_PARAMETER8_REG * 2) +
					BMA4XY_STEP_COUNTER_PARAMETER8_BYTE;
		break;
		}

		feature_config[index] = (u8)(parameter & BMA4XY_SET_LOW_BYTE);
		feature_config[index + 1] =
				(u8)((parameter & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	} else {
			 com_rslt = E_BMA4XY_OUT_OF_RANGE;
	}

	return com_rslt;
}
#endif
#if defined(BMA422) || defined(BMA455)

/*!
 *	@brief This function is used to enable or disable the significant
 *	motion in the sensor
 *
 *
 *  @param enable : used to enable or disable significant motion
 *
 *  value      |  enable
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE
 *    0x01     |  BMA4XY_ENABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_significant_motion_enable(u8 enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = (BMA4XY_SIG_MOTION_ENABLE_REG * 2) +
						BMA4XY_SIG_MOTION_ENABLE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_SIG_MOTION_ENABLE, enable);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}
	return com_rslt;
}



/*!
 *	@brief This function is used to set the threshold value
 *	motion in the sensor
 *
 *  @param threshold_time : used to set threshold time for
 *	significant motion.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_significant_motion_set_threshold(u16 threshold_time)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_SIG_MOTION_THRESHOLD_REG * 2) +
					BMA4XY_SIG_MOTION_THRESHOLD_BYTE;

		feature_config[index] = (u8)(threshold_time &
							BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] = (u8)((threshold_time &
						BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}


	return com_rslt;
}

/*!
 *	@brief This function is used to get the threshold value
 *	motion in the sensor
 *
 *  @param threshold_time : used to get threshold time for
 *	significant motion.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_significant_motion_get_threshold(u16 *threshold_time)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_SIG_MOTION_THRESHOLD_REG * 2) +
					BMA4XY_SIG_MOTION_THRESHOLD_BYTE;

		*threshold_time = (s16) ((feature_config[index]) |
					((feature_config[index + 1]) << 8));
	}
	return com_rslt;
}
/*!
 *	@brief This function is used to set the proof time
 *	for significant motion in the sensor
 *
 *  @param proof_time : used to set proof time for
 *	significant motion.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_significant_motion_set_prooftime(u8 proof_time)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_SIG_MOTION_PROOFTIME_REG * 2) +
					BMA4XY_SIG_MOTION_PROOFTIME_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_SIG_MOTION_PROOFTIME,
					proof_time);


		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the proof time
 *	of significant motion in the sensor
 *
 *  @param proof_time: used to get proof time of
 *	significant motion.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_significant_motion_get_prooftime(u8 *proof_time)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_SIG_MOTION_PROOFTIME_REG * 2) +
					BMA4XY_SIG_MOTION_PROOFTIME_BYTE;

		*proof_time = BMA4XY_GET_BITSLICE(feature_config[index],
						BMA4XY_SIG_MOTION_PROOFTIME);
	}

	return com_rslt;
}


/*!
 *	@brief This function is used to set the skiptime
 *	for significant motion in the sensor
 *
 *  @param skip_time : used to set skip time for
 *	significant motion.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_significant_motion_set_skiptime(u16 skip_time)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_SIG_MOTION_SKIPTIME_REG * 2) +
						BMA4XY_SIG_MOTION_SKIPTIME_BYTE;

		data_u16 = BMA4XY_SET_BITSLICE(((u16)(feature_config[index] |
				((feature_config[index + 1] << 8)))),
				BMA4XY_SIG_MOTION_SKIPTIME, skip_time);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}


/*!
 *	@brief This function is used to get the skiptime
 *	of significant motion in the sensor
 *
 *  @param skip_time : used to get skip time for
 *	significant motion.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_significant_motion_get_skiptime(u16 *skip_time)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_SIG_MOTION_SKIPTIME_REG * 2) +
						BMA4XY_SIG_MOTION_SKIPTIME_BYTE;

		*skip_time = (u16)(feature_config[index] |
				((feature_config[index + 1] & 0x01) << 8));
	}
	return com_rslt;
}

/*!
 *	@brief This function is used to enable or disable the tilt
 *	detection  in the sensor
 *
 *
 *  @param enable : used to enable or disable tilt detection sensor
 *
 *  value      |  enable
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE
 *    0x01     |  BMA4XY_ENABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_tilt_enable(u8 enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_TILT_ENABLE_REG * 2) + BMA4XY_TILT_ENABLE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_TILT_ENABLE, enable);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the threshold
 *	of tilt sensor
 *
 *
 *  @param threshold :	specifies the threshold value
 *  tilt sensor
 *
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_tilt_set_threshold(u8 threshold)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_TILT_THRESHOLD_REG * 2) +
						BMA4XY_TILT_THRESHOLD_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_TILT_THRESHOLD, threshold);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	}
	return com_rslt;
}

/*!
 *	@brief This function is used to get the threshold time
 *	tilt sensor
 *
 *
 *  @param threshold_time : used to get threshold time for
 *	for tilt sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_tilt_get_threshold(u8 *threshold)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_TILT_THRESHOLD_REG * 2) +
						BMA4XY_TILT_THRESHOLD_BYTE;

		*threshold =  (feature_config[index] & BMA4XY_SET_LOW_NIBBLE);
	}
	return com_rslt;

}



/*!
 *	@brief This function is used to enable or disable the glance
 *	detector in the sensor
 *
 *  @param enable : used to enable or disable glance detector sensor
 *
 *  value      |  enable
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE
 *    0x01     |  BMA4XY_ENABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_glance_enable(u8 enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_GLANCE_ENABLE_REG * 2) +
						BMA4XY_GLANCE_ENABLE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_GLANCE_ENABLE, enable);


		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to enable or disable the pickup
 *	feature in the sensor
 *
 *
 *  @param enable : used to enable or disable pickup sensor
 *
 *  value      |  enable
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE
 *    0x01     |  BMA4XY_ENABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_pickup_enable(u8 enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_PICKUP_ENABLE_REG * 2) +
						BMA4XY_PICKUP_ENABLE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_PICKUP_ENABLE, enable);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to enable or disable the wakeup
 *	detector in the sensor
 *
 *
 *  @param enable : used to enable or disable wakeup detection
 *
 *  value      |  enable
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE
 *    0x01     |  BMA4XY_ENABLE
 *
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_wakeup_enable(u8 enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = (BMA4XY_WAKEUP_ENABLE_REG * 2) +
						BMA4XY_WAKEUP_ENABLE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
						feature_config[index],
						BMA4XY_WAKEUP_ENABLE, enable);

		com_rslt = bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}
#endif

#if defined(BMA420)
/*!
 *	@brief This function is used to enable/disable
 *	orientation feature in the sensor
 *
 *
 *  @param orientation_enable :	variable to enable/disable
 *	the orientation feature
 *
 *	value    |	enable
 * ----------|---------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_enable(u8 orientation_enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_ENABLE_REG) * 2) +
			BMA4XY_ORIENTATION_ENABLE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
			feature_config[index], BMA4XY_ORIENTATION_ENABLE,
			orientation_enable);


		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to enable/disable
 *	upside down detection  in the sensor
 *
 *
 *  @param ud_enable :	variable to enable/disable
 *	upside down detection in the sensor
 *
 *	value    |	enable
 * ----------|---------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_ud_enable(u8 ud_enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_UD_REG) * 2) +
						BMA4XY_ORIENTATION_UD_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
				feature_config[index], BMA4XY_ORIENTATION_UD,
				ud_enable);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the mode
 *	of orientation  in the sensor
 *
 *  @param orientation_mode :	variable to enable/disable
 *	upside down detection in the sensor
 *
 *	value     |	 orientation_mode
 * -----------|---------------
 *  0x00			|  BMA4XY_ORIENATATION_SYMMETRICAL
 *  0x01			|  BMA4XY_ORIENATATION_HIGH_ASYMMETRICAL
 *	0x02			|	 BMA4XY_ORIENATATION_LOW_ASYMMETRICAL
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_set_mode(u8 orientation_mode)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_MODE_REG) * 2) +
						BMA4XY_ORIENTATION_MODE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_ORIENTATION_MODE,
					orientation_mode);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the mode
 *	of orientation  set in the sensor
 *
 *
 *  @param orientation_mode :	pointer to store
 *	upside down detection set in the sensor
 *
 *	value     |	 orientation_mode
 * -----------|---------------
 *  0x00			|  BMA4XY_ORIENATATION_SYMMETRICAL
 *  0x01			|  BMA4XY_ORIENATATION_HIGH_ASYMMETRICAL
 *	0x02			|	 BMA4XY_ORIENATATION_LOW_ASYMMETRICAL
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_get_mode(u8 *orientation_mode)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_MODE_REG) * 2) +
						BMA4XY_ORIENTATION_MODE_BYTE;

		*orientation_mode = BMA4XY_GET_BITSLICE(
					feature_config[index],
					BMA4XY_ORIENTATION_MODE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the blocking
 *	mode of orientation feature in the sensor
 *
 *
 *  @param orientation_blocking :	variable to set
 *	the blocking mode in the sensor
 *
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_set_blocking(u8 orientation_blocking)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_BLOCKING_REG) * 2) +
					BMA4XY_ORIENTATION_BLOCKING_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
						feature_config[index],
						BMA4XY_ORIENTATION_BLOCKING,
						orientation_blocking);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the blocking
 *	mode of orientation feature set in the sensor
 *
 *
 *  @param orientation_blocking :	variable to get
 *	the blocking mode set  in the sensor
 *
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_get_blocking(u8 *orientation_blocking)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_BLOCKING_REG) * 2) +
					BMA4XY_ORIENTATION_BLOCKING_BYTE;

		*orientation_blocking = BMA4XY_GET_BITSLICE(
						feature_config[index],
						BMA4XY_ORIENTATION_BLOCKING);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the theta
 *	value of orientation feature in the sensor
 *
 *
 *  @param orientation_theta :	variable used to set the
 *	theta value of the orientation feature in
 *	the sensor.
 *
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_set_theta(u8 orientation_theta)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_THETA_REG) * 2) +
						BMA4XY_ORIENTATION_THETA_BYTE;

		data_u16 = (u16)((feature_config[index] |
					(feature_config[index + 1] << 8)));

		data_u16 = BMA4XY_SET_BITSLICE(data_u16,
				BMA4XY_ORIENTATION_THETA, orientation_theta);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] = (u8)((data_u16 &
						BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the theta
 *	value of orientation feature in the sensor
 *
 *
 *  @param orientation_theta :	pointer used to store the
 *	theta value of the orientation feature set in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_get_theta(u8 *orientation_theta)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_THETA_REG) * 2) +
						BMA4XY_ORIENTATION_THETA_BYTE;

		data_u16 = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

		*orientation_theta = BMA4XY_GET_BITSLICE(
					data_u16, BMA4XY_ORIENTATION_THETA);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the hysteresis
 *	value of orientation feature in the sensor
 *
 *  @param orientation_hysteresis :	variable used to set the
 *	hysteresis value of the orientation feature in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_set_hysteresis(u16 orientation_hysteresis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_HYSTERESIS_REG) * 2) +
					BMA4XY_ORIENTATION_HYSTERESIS_BYTE;

		data_u16 = (u16)((feature_config[index] |
					(feature_config[index + 1] << 8)));

		data_u16 = BMA4XY_SET_BITSLICE(
					data_u16, BMA4XY_ORIENTATION_HYSTERESIS,
					orientation_hysteresis);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);

		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the hysteresis
 *	value of orientation feature in the sensor
 *
 *
 *  @param orientation_hysteresis :	pointer used to store the
 *	hysteresis value of the orientation feature set in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_get_hysteresis(
						u16 *orientation_hysteresis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_ORIENTATION_HYSTERESIS_REG) * 2) +
					BMA4XY_ORIENTATION_HYSTERESIS_BYTE;

		data_u16 = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

		*orientation_hysteresis = BMA4XY_GET_BITSLICE(data_u16,
					BMA4XY_ORIENTATION_HYSTERESIS);

	}

	return com_rslt;
}

/*!
 *	@brief This function is used to enable/disable
 *	flat feature in the sensor
 *
 *
 *  @param flat_enable :	variable to enable/disable
 *	the flat feature
 *
 *	value    |	enable
 * ----------|---------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_flat_enable(u8 flat_enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

			index = ((BMA4XY_FLAT_ENABLE_REG) * 2) +
							BMA4XY_FLAT_ENABLE_BYTE;
		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_FLAT_ENABLE, flat_enable);
		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the theta
 *	value of flat feature in the sensor
 *
 *
 *  @param flat_theta :	variable used to set the
 *	theta value of the flat feature in
 *	the sensor.
 *
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_flat_set_theta(u8 flat_theta)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_FLAT_THETA_REG) * 2) +
							BMA4XY_FLAT_THETA_BYTE;

		data_u16 = (u16)(feature_config[index] |
						(feature_config[index+1] << 8));

		data_u16 = BMA4XY_SET_BITSLICE(data_u16, BMA4XY_FLAT_THETA,
						flat_theta);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);

		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the theta
 *	value of flat feature in the sensor
 *
 *
 *  @param flat_theta :	pointer used to store the
 *	theta value of the flat feature set in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_flat_get_theta(u8 *flat_theta)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = ((BMA4XY_FLAT_THETA_REG) * 2) + BMA4XY_FLAT_THETA_BYTE;

		data_u16 = (u16)(feature_config[index] |
						(feature_config[index+1] << 8));

		*flat_theta = BMA4XY_GET_BITSLICE(data_u16, BMA4XY_FLAT_THETA);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the hysteresis
 *	value of flat feature in the sensor
 *
 *
 *  @param flat_hysteresis :	variable used to set the
 *	hysteresis value of the flat feature in
 *	the sensor.
 *
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_flat_set_hysteresis(u8 flat_hysteresis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_FLAT_HYSTERESIS_REG) * 2) +
						BMA4XY_FLAT_HYSTERESIS_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
				feature_config[index], BMA4XY_FLAT_HYSTERESIS,
				flat_hysteresis);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the hysteresis
 *	value of flat feature in the sensor
 *
 *
 *  @param flat_hysteresis :	pointer used to store the
 *	hysteresis value of the flat feature set in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_flat_get_hysteresis(u8 *flat_hysteresis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
			BMA4XY_UC_DMA_DATA,
			feature_config, BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_FLAT_HYSTERESIS_REG) * 2) +
						BMA4XY_FLAT_HYSTERESIS_BYTE;

		*flat_hysteresis = BMA4XY_GET_BITSLICE(
				feature_config[index], BMA4XY_FLAT_HYSTERESIS);

	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the hold time
 *	of flat feature in the sensor
 *
 *
 *  @param flat_holdtime :	variable used to set the
 *	hold time of the flat feature in
 *	the sensor.
 *
 *	@note valid values are from 0 to 7
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_flat_set_holdtime(u8 flat_holdtime)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_FLAT_HOLD_TIME_REG) * 2) +
						BMA4XY_FLAT_HOLD_TIME_BYTE;

		data_u16 = (u16)(feature_config[index] |
						(feature_config[index+1] << 8));

		data_u16 = BMA4XY_SET_BITSLICE(data_u16, BMA4XY_FLAT_HOLD_TIME,
						flat_holdtime);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] = (u8)((data_u16 &
						BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the hold time
 *	of flat feature in the sensor
 *
 *
 *  @param flat_holdtime :	pointer used to store the
 *	hold time of the flat feature set in
 *	the sensor.
 *
 *	@note valid values are from 0 to 7
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_flat_get_holdtime(u8 *flat_holdtime)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_FLAT_HOLD_TIME_REG) * 2) +
						BMA4XY_FLAT_HOLD_TIME_BYTE;

		data_u16 = (u16)(feature_config[index] |
						(feature_config[index+1] << 8));

		*flat_holdtime = BMA4XY_GET_BITSLICE(data_u16,
							BMA4XY_FLAT_HOLD_TIME);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the blocking
 *	mode of flat feature in the sensor
 *
 *
 *  @param flat_blocking :	variable used to set the
 *	blocking mode value of the flat feature in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_flat_set_blocking(u8 flat_blocking)
{
		/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
						BMA4XY_UC_DMA_DATA,
						feature_config,
						BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_FLAT_BLOCKING_REG) * 2) +
						BMA4XY_FLAT_BLOCKING_BYTE;
		data_u16 = (u16)(feature_config[index] |
						(feature_config[index+1] << 8));

		data_u16 = BMA4XY_SET_BITSLICE(data_u16, BMA4XY_FLAT_BLOCKING,
						flat_blocking);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;

}

/*!
 *	@brief This function is used to get the blocking
 *	mode of flat feature in the sensor
 *
 *  @param flat_blocking :	pointer used to store the
 *	blocking mode value of the flat feature set in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_flat_get_blocking(u8 *flat_blocking)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_FLAT_BLOCKING_REG) * 2) +
						BMA4XY_FLAT_BLOCKING_BYTE;

		data_u16 = (u16)(feature_config[index] |
						(feature_config[index+1] << 8));

		*flat_blocking = BMA4XY_GET_BITSLICE(data_u16,
							BMA4XY_FLAT_BLOCKING);
	}

	return com_rslt;

}

/*!
 *	@brief This function is used to enable or disable the tap
 *	detector in the sensor
 *
 *
 *  @param tap_enable : used to enable or disable tap detection
 *	@param single_tap_en : used to control single tap or double tap
 *
 *  value      |  enable
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE
 *    0x01     |  BMA4XY_ENABLE
 *
 *
 *	value      |  single_tap_en
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DOUBLE_TAP
 *    0x01     |  BMA4XY_SINGLE_TAP
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_tap_enable(u8 tap_enable, u8 single_tap_en)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
			BMA4XY_UC_DMA_DATA, feature_config,
			BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_SINGLE_TAP_EN_REG) * 2) +
						BMA4XY_SINGLE_TAP_EN_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
				feature_config[index], BMA4XY_TAP_ENABLE,
				tap_enable);

		if ((BMA4XY_SINGLE_TAP == single_tap_en) &&
						(BMA4XY_ENABLE == tap_enable))
			feature_config[index] = BMA4XY_SET_BITSLICE(
						feature_config[index],
						BMA4XY_SINGLE_TAP_EN,
						single_tap_en);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the sensitivity of tap
 *	detector in the sensor
 *
 *
 *  @param sensitivity : variable used to set the sensitivity
 *	of the tap sensor
 *
 *	value      |  sensitivity
 * ------------|-------------------------
 *    0x00     |  MOST SENSITIVE
 *    0x07     |  LEAST SENSITIVE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_tap_set_sensitivity(u8 sensitivity)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_TAP_SENSITIVITY_REG) * 2) +
						BMA4XY_TAP_SENSITIVITY_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
				feature_config[index], BMA4XY_TAP_SENSITIVITY,
				sensitivity);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the sensitivity of tap
 *	detector in the sensor
 *
 *
 *  @param sensitivity : variable used to get the sensitivity
 *	of the tap sensor
 *
 *	value      |  sensitivity
 * ------------|-------------------------
 *    0x00     |  MOST SENSITIVE
 *    0x07     |  LEAST SENSITIVE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_tap_get_sensitivity(u8 *sensitivity)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
			BMA4XY_UC_DMA_DATA, feature_config,
			BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {
		index = ((BMA4XY_TAP_SENSITIVITY_REG) * 2) +
						BMA4XY_TAP_SENSITIVITY_BYTE;

		*sensitivity = BMA4XY_GET_BITSLICE(
				feature_config[index], BMA4XY_TAP_SENSITIVITY);
		}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the threshold
 *	value of high_g feature in the sensor
 *
 *
 *  @param high_g_threshold :	variable used to set the
 *	threshold value of the low_g feature in
 *	the sensor.
 *
 *	@note valid values are from 0 to 32767
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_high_g_set_threshold(u16 high_g_threshold)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
			BMA4XY_UC_DMA_DATA,
			feature_config, BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = (BMA4XY_HIGH_G_OFFSET * 2) +
						BMA4XY_HIGH_G_THRESHOLD_BYTE;

		data_u16 = BMA4XY_SET_BITSLICE(data_u16,
				BMA4XY_HIGH_G_THRESHOLD, high_g_threshold);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);

		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the threshold
 *	of high_g feature in the sensor
 *
 *
 *  @param high_g_threshold :	pointer used to store the
 *	threshold value of the low_g feature set in
 *	the sensor.
 *
 *	@note valid values are from 0 to 32767
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_high_g_get_threshold(u16 *high_g_threshold)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = (BMA4XY_HIGH_G_OFFSET * 2) +
						BMA4XY_HIGH_G_THRESHOLD_BYTE;

		data_u16 = (u16)((feature_config[index] |
					(feature_config[index+1] << 8)));

		*high_g_threshold = BMA4XY_GET_BITSLICE(data_u16,
						BMA4XY_HIGH_G_THRESHOLD);

	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the hysteresis
 *	value of high_g feature in the sensor
 *
 *
 *  @param high_g_hysteresis :	variable used to set the
 *	hysteresis value of the low_g feature in
 *	the sensor.
 *
 *	@note valid values are from 0 to 4095
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_high_g_set_hysteresis(u16 high_g_hysteresis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
			BMA4XY_UC_DMA_DATA,
			feature_config, BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_HIGH_G_HYSTERESIS_REG) * 2) +
						BMA4XY_HIGH_G_HYSTERESIS_BYTE;

		data_u16 = (feature_config[index] |
					(feature_config[index + 1] << 8));

		data_u16 = BMA4XY_SET_BITSLICE(data_u16,
				BMA4XY_HIGH_G_HYSTERESIS, high_g_hysteresis);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the hysteresis
 *	value of high_g feature in the sensor
 *
 *
 *  @param high_g_hysteresis :	pointer used to store the
 *	hysteresis value of the low_g feature set in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_high_g_get_hysteresis(u16 *high_g_hysteresis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
			BMA4XY_UC_DMA_DATA,
			feature_config, BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_HIGH_G_HYSTERESIS_REG) * 2) +
						BMA4XY_HIGH_G_HYSTERESIS_BYTE;

		data_u16 = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

		*high_g_hysteresis = BMA4XY_GET_BITSLICE(
					data_u16, BMA4XY_HIGH_G_HYSTERESIS);


	}

	return com_rslt;
}



/*!
 *	@brief This function is used to enable
 *	the axis of high_g feature in the sensor
 *
 *
 *  @param axis :	enum variable used to set the
 *	axis of the high_g feature	in
 *	the sensor.
 *
 *  value      |  axis
 * ------------|-------------------------
 *    0x00     |  BMA4XY_DISABLE_ALL_AXIS
 *    0x01     |  BMA4XY_X_AXIS_ENABLE
 *    0x02     |  BMA4XY_Y_AXIS_ENABLE
 *    0x04     |  BMA4XY_Z_AXIS_ENABLE
 *    0x07     |  BMA4XY_ENABLE_ALL_AXIS
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_high_g_enable_axis(enum bma4xy_axis_enable axis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = (BMA4XY_HIGH_G_ENABLE_ALL_REG * 2) +
						BMA4XY_HIGH_G_ENABLE_ALL_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_HIGH_G_ENABLE_ALL, axis);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the duration
 *	of high_g feature in the sensor
 *
 *
 *  @param high_g_duration :	variable used to set the
 *	duration value of the high_g feature	in
 *	the sensor.
 *
 *	@note valid values are from 0 to 4095
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_high_g_set_duration(u16 high_g_duration)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_HIGH_G_DURATION_REG) * 2) +
						BMA4XY_HIGH_G_DURATION_BYTE;

		data_u16 = BMA4XY_SET_BITSLICE(feature_config[index],
				BMA4XY_HIGH_G_DURATION, high_g_duration);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] = (u8)((data_u16 &
						BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the duration
 *	of high_g feature in the sensor
 *
 *
 *  @param high_g_duration :	pointer used to store the
 *	duration value of the high_g feature set in
 *	the sensor.
 *
 *	@note valid values are from 0 to 4095
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_high_g_get_duration(u16 *high_g_duration)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

			index = ((BMA4XY_HIGH_G_DURATION_REG) * 2) +
						BMA4XY_HIGH_G_DURATION_BYTE;

		data_u16 = (u16)(feature_config[index] |
						(feature_config[index+1] << 8));

		*high_g_duration = BMA4XY_GET_BITSLICE(data_u16,
							BMA4XY_HIGH_G_DURATION);

	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the threshold
 *	value of low_g feature in the sensor
 *
 *
 *  @param low_g_threshold :	variable used to set the
 *	threshold value of the low_g feature in
 *	the sensor.
 *
 *	@note valid values are from 0 to 32767
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_low_g_set_threshold(u16 low_g_threshold)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = (BMA4XY_LOW_G_THRESHOLD_REG * 2) +
						BMA4XY_LOW_G_THRESHOLD_BYTE;


		data_u16 = BMA4XY_SET_BITSLICE(data_u16, BMA4XY_LOW_G_THRESHOLD,
						low_g_threshold);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the threshold
 *	of low_g feature in the sensor
 *
 *
 *  @param low_g_threshold :	pointer used to store the
 *	threshold value of the low_g feature set in
 *	the sensor.
 *
 *	@note valid values are from 0 to 32767
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_low_g_get_threshold(u16 *low_g_threshold)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = (BMA4XY_LOW_G_THRESHOLD_REG * 2) +
						BMA4XY_LOW_G_THRESHOLD_BYTE;

		data_u16 = (u16)(feature_config[index] |
						(feature_config[index+1] << 8));

		*low_g_threshold = BMA4XY_GET_BITSLICE(data_u16,
							BMA4XY_LOW_G_THRESHOLD);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the duration
 *	value of low_g feature in the sensor
 *
 *
 *  @param duration :	variable used to set the
 *	duration value of the low_g feature in
 *	the sensor.
 *
 *	@note valid values are from 0 to 4095
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_low_g_set_duration(u16 duration)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_LOW_G_DURATION_REG) * 2) +
						BMA4XY_LOW_G_DURATION_BYTE;

		data_u16 = BMA4XY_SET_BITSLICE(feature_config[index],
					BMA4XY_LOW_G_DURATION, duration);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the duration
 *	of low_g feature in the sensor
 *
 *
 *  @param low_g_duration :	pointer used to store the
 *	duration value of the low_g feature set in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_low_g_get_duration(u16 *low_g_duration)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;
	u16 data_u16 = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_LOW_G_DURATION_REG) * 2) +
						BMA4XY_LOW_G_DURATION_BYTE;

		data_u16 = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

		*low_g_duration = BMA4XY_GET_BITSLICE(data_u16,
							BMA4XY_LOW_G_DURATION);

	}

	return com_rslt;
}

/*!
 *	@brief This function is used to set the hysteresis
 *	value of low_g feature in the sensor
 *
 *
 *  @param hysteresis :	variable used to set the
 *	hysteresis value of the low_g feature in
 *	the sensor.
 *
 *	@note valid values are from 0 to 4095
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_low_g_set_hysteresis(u16 hysteresis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_UC_DMA_DATA, feature_config,
				BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_LOW_G_HYSTERESIS_REG) * 2) +
						BMA4XY_LOW_G_HYSTERESIS_BYTE;

		data_u16 = (u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

		data_u16 = BMA4XY_SET_BITSLICE(data_u16,
					BMA4XY_LOW_G_HYSTERESIS, hysteresis);

		feature_config[index] = (u8)(data_u16 & BMA4XY_SET_LOW_BYTE);
		feature_config[index+1] =
				(u8)((data_u16 & BMA4XY_SET_HIGH_BYTE) >> 8);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the hysteresis
 *	value of low_g feature in the sensor
 *
 *
 *  @param hysteresis :	pointer used to store the
 *	hysteresis value of the low_g feature set in
 *	the sensor.
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_low_g_get_hysteresis(u16 *hysteresis)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u16 data_u16 = BMA4XY_INIT_VALUE;
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = ((BMA4XY_LOW_G_HYSTERESIS_REG) * 2) +
						BMA4XY_LOW_G_HYSTERESIS_BYTE;

		data_u16 =
			(u16)(feature_config[index] |
					(feature_config[index + 1] << 8));

		*hysteresis = BMA4XY_GET_BITSLICE(data_u16,
						BMA4XY_LOW_G_HYSTERESIS);
	}
	return com_rslt;
}

/*!
 *	@brief This function is used to enable/disable
 *	low_g feature in the sensor
 *
 *
 *  @param enable :	variable to enable/disable
 *	the low_g feature
 *
 *	value    |	enable
 * ----------|---------------
 *  0x01     |  BMA4XY_ENABLE
 *  0x00     |  BMA4XY_DISABLE
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_low_g_enable(u8 enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 feature_config[BMA4XY_FEATURE_SIZE] = {BMA4XY_INIT_VALUE,};
	u8 index = BMA4XY_INIT_VALUE;

	com_rslt = bma4xy_p->BMA4XY_BURST_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);

	if (BMA4XY_SUCCESS == com_rslt) {

		index = (BMA4XY_LOW_G_ENABLE_REG * 2) +
						BMA4XY_LOW_G_ENABLE_BYTE;

		feature_config[index] = BMA4XY_SET_BITSLICE(
					feature_config[index],
					BMA4XY_LOW_G_ENABLE, enable);

		com_rslt += bma4xy_p->burst_write(bma4xy_p->dev_addr,
					BMA4XY_UC_DMA_DATA, feature_config,
					BMA4XY_FEATURE_SIZE);
	}

	return com_rslt;
}

/*!
 *	@brief This function is used to get the output of
 *	of orientation sensor
 *
 *
 *  @param orientation_output :	pointer used to store
 *  the orientation output
 *
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_orientation_output(u8 *orientation_output)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* Read data from register*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_GPIO_0_REG, data,
				((BMA4XY_READ_LENGTH+READ_EXTRA_BYTE)));

		*orientation_output = (data[READ_EXTRA_BYTE] &
					BMA4XY_ORIENTATION_OUTPUT_MASK);

		}
	return com_rslt;
}


/*!
 *	@brief This function is used to get the output of
 *	of high_g detection
 *
 *
 *  @param high_g_output :	pointer used to store
 *  the high_g detection output
 *
 *
 *	@return results of DMA communication
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_high_g_detection_output(u8 *high_g_output)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};
	/* check the bma4xy_p structure as NULL*/
	if (bma4xy_p == BMA4XY_NULL) {
		com_rslt = E_BMA4XY_NULL_PTR;
	} else {
		/* Read data from register*/
		com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
				BMA4XY_GPIO_2_REG, data,
				((BMA4XY_READ_LENGTH+READ_EXTRA_BYTE)));

		*high_g_output = (data[READ_EXTRA_BYTE] &
				BMA4XY_SET_LOW_NIBBLE);

	}
	return com_rslt;

}
#endif
/*!
 *	@brief This method is used to configure fifo
 *	for accel, mag or both.
 *
 *	@param sensor_type : variable used to specify which sensor to
 *	be enabled for fifo (accel. mag or both)
 *	@param header_enable : variable used to enable header
 *
 *	 sensor_type	|	value
 * --------------------	|-------------------------
 *	BMA4XY_ACCEL	|	1
 *	BMA4XY_MAG	|	2
 *	BMA4XY_ALL	|	3
 *
 *	header_enable	|	value
 * -------------------	|--------------------------
 *	BMA4XY_ENABLE	|	1
 *	BMA4XY_DISABLE	|	0
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMA4XY_RETURN_TYPE bma4xy_setup_fifo(enum bma4xy_fifo_setup sensor_type,
					u8 header_enable)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 fifo_enable = BMA4XY_INIT_VALUE;
	u8 uc_config = BMA4XY_INIT_VALUE;

	switch (sensor_type) {

	case BMA4XY_ALL:
		{
		/* check if accel is enabled for fifo*/
		com_rslt += bma4xy_get_fifo_accel_enable(&fifo_enable);
		/* if not enabled, enable it*/
		if (fifo_enable == BMA4XY_DISABLE)
			com_rslt += bma4xy_set_fifo_accel_enable(
							FIFO_ACCEL_ENABLE);

			/* check if mag is enabled for fifo*/
			com_rslt += bma4xy_get_fifo_mag_enable(&fifo_enable);
			/* if not enabled, enable it*/
			if (fifo_enable == BMA4XY_DISABLE)
				com_rslt += bma4xy_set_fifo_mag_enable(
							FIFO_MAG_ENABLE);

	break;
		}
	case BMA4XY_MAG:
		{
		/* check if mag is enabled for fifo*/
		com_rslt += bma4xy_get_fifo_mag_enable(&fifo_enable);

		/* if not enabled, enable it*/
		if (fifo_enable == BMA4XY_DISABLE)
			com_rslt += bma4xy_set_fifo_mag_enable(FIFO_MAG_ENABLE);

	break;

		}
	case BMA4XY_ACCEL:
		{
		/* check if accel is enabled for fifo*/
		com_rslt += bma4xy_get_fifo_accel_enable(&fifo_enable);
		/* if not enabled, enable it*/
		if (fifo_enable == BMA4XY_DISABLE)
			com_rslt += bma4xy_set_fifo_accel_enable(
							FIFO_ACCEL_ENABLE);

	break;
		}
	default:
		{
			com_rslt = E_BMA4XY_OUT_OF_RANGE;
	break;
		}

		}
		if (BMA4XY_SUCCESS == com_rslt) {
			/* read uc configuration */
			bma4xy_read_reg(0x59, &uc_config, 1);

			/* Map ram1 instance to fifo*/
			uc_config |= (0x05 << 1);
			bma4xy_write_reg(0x59, &uc_config, 1);

			if (header_enable == BMA4XY_TRUE)
				bma4xy_set_fifo_header_enable(0x01);
			else
				bma4xy_set_fifo_header_enable(0x00);

		}
	return com_rslt;
}

/*!
 *	@brief This function is used to set  the interrupt mode
 *	of the bma4xy sensor.
 *
 *	@param mode: used to specify the interrupt mode to be set in
 *	in the sensor.
 *
 *		mode				|	value
 *	------------------------|-----------------------------
 *	BMA4XY_NON_LATCH_MODE	|	0
 *	BMA4XY_LATCH_MODE		|	1
 *
 *	@return results of communication result
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_set_interrupt_mode(u8 mode)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;

	if (mode == BMA4XY_NON_LATCH_MODE || mode == BMA4XY_LATCH_MODE)

		com_rslt = bma4xy_p->BMA4XY_BUS_WRITE_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR_LATCH_ADDR, &mode,
					BMA4XY_WRITE_LENGTH);
	else
		com_rslt = E_BMA4XY_OUT_OF_RANGE;


	return com_rslt;
}

/*!
 *	@brief This function is used to get  the interrupt mode
 *	of the bma4xy sensor.
 *
 *	@param mode: used to get the  interrupt mode set in
 *	in the sensor.
 *
 *		mode				|	value
 *	------------------------|-----------------------------
 *	BMA4XY_NON_LATCH_MODE	|	0
 *	BMA4XY_LATCH_MODE		|	1
 *
 *	@return results of communication result
 *	@retval 0 -> Success
 *	@retval -1 -> Fail
 *
 *
*/
BMA4XY_RETURN_TYPE bma4xy_get_interrupt_mode(u8 *mode)
{
	/* variable used to return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	u8 data_u8[2] = {BMA4XY_INIT_VALUE, BMA4XY_INIT_VALUE};


	com_rslt = bma4xy_p->BMA4XY_BUS_READ_FUNC(bma4xy_p->dev_addr,
					BMA4XY_INTR_LATCH_ADDR, data_u8,
					BMA4XY_READ_LENGTH+READ_EXTRA_BYTE);

	*mode = data_u8[READ_EXTRA_BYTE];

	return com_rslt;

}
BMA4XY_RETURN_TYPE bma4xy_bst_akm09916_compensate_xyz(
struct bma4xy_mag_xyz_s32_t *bst_akm_xyz)
{
	/* variable used for return the status of communication result*/
	BMA4XY_RETURN_TYPE com_rslt = BMA4XY_INIT_VALUE;
	struct bma4xy_mag_t mag_xyz;

	com_rslt = bma4xy_read_mag_xyz(&mag_xyz, BST_AKM);
	if (SUCCESS == com_rslt) {
		/* Compensation for X axis */
		bst_akm_xyz->x = mag_xyz.x;
		/* Compensation for Y axis */
		bst_akm_xyz->y = mag_xyz.y;
		/* Compensation for Z axis */
		bst_akm_xyz->z = mag_xyz.z;
	}
	return com_rslt;
}

