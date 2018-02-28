#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
//#include <linux/xlog.h>


/*===FEATURE SWITH===*/
 // #define FPTPDAFSUPPORT   //for pdaf switch
 // #define FANPENGTAO   //for debug log
//#define LOG_INF LOG_INF_LOD
/*===FEATURE SWITH===*/


/****************************Modify Following Strings for Debug****************************/
#define PFX "HI1333PDAF"
#define LOG_INF_NEW(format, args...)    pr_debug(PFX "[%s](%d) " format, __FUNCTION__,__LINE__, ##args)
#define LOG_INF(fmt, args...)           pr_err(PFX "[%s](%d) " fmt, __FUNCTION__,__LINE__, ##args)
#define LOG_1                           LOG_INF("HI1333PDAF,MIPI 4LANE\n")
#define SENSORDB                        LOG_INF
/****************************   Modify end    *******************************************/

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms)          mdelay(ms)

/**************  CONFIG BY SENSOR >>> ************/
#define EEPROM_WRITE_ID     0xa0
#define I2C_SPEED           100
#define MAX_OFFSET            0xFFFF

#define START_ADDR_PROC1        0X0765
#define DATA_SIZE_PROC1         496
//#define START_ADDR_PROC2        0X0957
#define DATA_SIZE                  1404

BYTE hi1333_eeprom_data[DATA_SIZE]= {0};

/**************  CONFIG BY SENSOR <<< ************/

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

    kdSetI2CSpeed(I2C_SPEED); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,EEPROM_WRITE_ID);
    return get_byte;
}


static bool _read_eeprom(kal_uint16 addr, kal_uint32 size ){
    //continue read reg by byte:
    static int i = 0;

    LOG_INF( "begin i=%d addr=%d size=%d byyzm \n",i,addr,size);
    for(; i<size; i++){
        hi1333_eeprom_data[i] = read_cmos_sensor_byte(addr+i);
        LOG_INF("add = 0x%x,\tvalue = 0x%x\n",i, hi1333_eeprom_data[i]);
    }

    if(i == DATA_SIZE )
    {
        i = 0;
    }
    LOG_INF( "return true i=%d addr=%d size=%d byyzm \n",i,addr,size);
    return true;
}

bool hi1333_read_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size)
{

    LOG_INF("begin\n");
    LOG_INF("Read EEPROM, addr = 0x%x, size = 0d%d\n", addr, size);

    addr = START_ADDR_PROC1;
    size = DATA_SIZE_PROC1;
    if(!_read_eeprom(addr, size)){
        LOG_INF("error:read_eeprom fail!\n");
        LOG_INF( "return false \n");
        return false;
    }

    addr = START_ADDR_PROC1+2;//START_ADDR_PROC2;
    size = DATA_SIZE;
    if(!_read_eeprom(addr, size)){
        LOG_INF("error:read_eeprom fail!\n");
        LOG_INF( "return false \n");
        return false;
    }

    memcpy(data, hi1333_eeprom_data, size);

    LOG_INF( "return true \n");
    return true;
}


