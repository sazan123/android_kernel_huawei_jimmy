/*****************************************************************************
 *
 * Filename:
 * ---------
 *     HI1333mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi1333_mipi_raw_Sensor.h"

#define PFX "hi1333_mipi_raw_Sensor.c"

#define LOG_INF(format, args...)    pr_err(PFX "[%s](%d) " format, __FUNCTION__,__LINE__, ##args)

//#define ENABLE_IVHDR 1

//test KYM
unsigned char bTh = 0;
unsigned char bGap = 0;


extern void kdSetI2CSpeed(u16 i2cSpeed);


#ifdef ENABLE_IVHDR
static void SetIHDR(BOOL EnableIHDR);    //Add iHDR enable function
#endif

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = HI1333_SENSOR_ID,

    .checksum_value = 0xabaa55c3,       //0x6d01485c // Auto Test Mode ÃßÈÄ..
    .pre = {
        .pclk = 600000000,                //record different mode's pclk
        .linelength = 5952,             //record different mode's linelength
        .framelength = 3300,             //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
//        .grabwindow_width = 2112, //2104,        //record different mode's width of grabwindow
        .grabwindow_width = 2104, //2104,        //record different mode's width of grabwindow

        .grabwindow_height = 1560,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 85,
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 600000000,
        .linelength = 5952,
        .framelength = 3300,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4208,
        .grabwindow_height = 3120,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    // need to setting
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 600000000,
        .linelength = 5952,
        .framelength = 3300,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4208,
        .grabwindow_height = 3120,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 150,
    },
    .normal_video = {
        .pclk = 600000000,
        .linelength = 5952,
        .framelength = 3300,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4208,
        .grabwindow_height = 3120,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 600000000,
        .linelength = 5952,
        .framelength = 840, //832,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640 ,
        .grabwindow_height = 480 ,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 600000000,
        .linelength = 5952,
        .framelength = 1120,//832,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 1200,
    },
        //.margin = 6,
        .margin = 10,
        .min_shutter = 1,
        .max_frame_length = 0xffff,
        .ae_shut_delay_frame = 0,
        .ae_sensor_gain_delay_frame = 0,
        .ae_ispGain_delay_frame = 2,
        .ihdr_support = 0,        //1, support; 0,not support
           .ihdr_le_firstline = 0,   //1,le first ; 0, se first
        .sensor_mode_num = 5,      //support sensor mode num

        .cap_delay_frame = 3,
        .pre_delay_frame = 3,
        .video_delay_frame = 3,
        .hs_video_delay_frame = 3,    //enter high speed video  delay frame num
        .slim_video_delay_frame = 3,  //enter slim video delay frame num

        .isp_driving_current = ISP_DRIVING_2MA,
        .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
        //.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
        //.mipi_sensor_type = MIPI_OPHY_CSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
        .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
        .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
//    SENSOR_OUTPUT_FORMAT_RAW_B = 0,
//    SENSOR_OUTPUT_FORMAT_RAW_Gb,
//    SENSOR_OUTPUT_FORMAT_RAW_Gr,
//    SENSOR_OUTPUT_FORMAT_RAW_R,
        .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
        .mclk = 24,
        .mipi_lane_num = SENSOR_MIPI_4_LANE,
        .i2c_addr_table = {0x40, 0xff},
        .i2c_speed = 400,
};


static imgsensor_struct imgsensor = {
        .mirror = IMAGE_NORMAL,                //mirrorflip information
        .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
        .shutter = 0x0100,                    //current shutter
        .gain = 0xe0,                        //current gain
        .dummy_pixel = 0,                    //current dummypixel
        .dummy_line = 0,                    //current dummyline
        .current_fps = 300,                 //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
        .autoflicker_en = KAL_FALSE,        //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
         .test_pattern = KAL_FALSE,
        .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
         .ihdr_en = 0,
        .i2c_write_id = 0x40,
};
#if 0
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
 { 4240, 1564,  0,  0, 4240, 1564,     2120, 1564,     8, 2, 2104, 1560,     0, 0, 2104, 1560},      // preview (2104 x 1560)
 { 4240, 3124,  0,  0, 4240, 3124,     4240, 3124,    16, 2, 4208, 3120,     0, 0, 4208, 3120},      // capture (4208 x 3120)
 { 4240, 3124,  0,  0, 4240, 3124,     4240, 3124,    16, 2, 4208, 3120,     0, 0, 4208, 3120},      // video   (4208 x 3120)
 { 4240,  724,  0,  0, 4240,  724,     4240,  724,    34, 2,  640,  480,     0, 0,  640,  480},      // hight speed video (640 x 480)
 { 4240,  484,  0,  0, 4240,  484,     4240,  484,    66, 2, 1280,  720,     0, 0, 1280,  720},      // slim video (1280 x 720)
};
#else
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
    { 4240, 1564,  0,  0, 4240, 1564,      2120, 1564,      8, 2, 2104, 1560,     0, 0, 2104, 1560},        // preview (2104 x 1560)
    { 4240, 3124,  0,  0, 4240, 3124,      4240, 3124,     16, 2, 4208, 3120,     0, 0, 4208, 3120},        // capture (4208 x 3120)
    { 4240, 3124,  0,  0, 4240, 3124,      4240, 3124,     16, 2, 4208, 3120,     0, 0, 4208, 3120},        // video   (4208 x 3120)
    { 4240,  724,  0,  0, 4240,  724,      4240,  724,     34, 2,  640,  480,     0, 0,  640,  480},        // hight speed video (640 x 480)
    { 4240,  484,  0,  0, 4240,  484,      4240,  484,     66, 2, 1280,  720,     0, 0, 1280,  720},        // slim video (1280 x 720)

};
#endif


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    //kdSetI2CSpeed(imgsensor_info.i2c_speed);

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

    iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor1D(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor(0x0006, imgsensor.frame_length);
    write_cmos_sensor(0x0008, imgsensor.line_length);

}    /*    set_dummy  */

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    //kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();

}    /*    set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{
    kal_uint16 realtime_fps = 0;
    //kal_uint32 frame_length = 0;

    LOG_INF("E\n");
    /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
    /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk * 10 / (imgsensor.line_length * imgsensor.frame_length);
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else{

            LOG_INF("Enable autoflicker\n");
            write_cmos_sensor(0x0006, imgsensor.frame_length);
            }
    } else {
            LOG_INF("Disable autoflicker\n");
        // Extend frame length
            write_cmos_sensor(0x0006, imgsensor.frame_length);
    }

    write_cmos_sensor1D(0x0073, (shutter & 0x0F0000) >> 16 );
    write_cmos_sensor1D(0x0074, (shutter & 0x00FF00) >> 8 );
    write_cmos_sensor1D(0x0075, (shutter & 0x0000FF) );

    LOG_INF("frame_length = %d , shutter = %d \n", imgsensor.frame_length, shutter);


}    /*    write_shutter  */



/*************************************************************************
* FUNCTION
*    enable set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
}    /*    set_shutter */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0000;
    reg_gain = gain / 4 - 16;

    return (kal_uint16)reg_gain;

}
/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/

static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    LOG_INF("set_gain is %d\n",gain);

    if( gain < BASEGAIN || gain > 16 * BASEGAIN ){
        LOG_INF("Error gain setting");
        if(gain < BASEGAIN )
            gain = BASEGAIN;
        else if( gain > 16 * BASEGAIN )
            gain = 16 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain; // set current gain to global
    spin_unlock(&imgsensor_drv_lock);

//    reg_gain = reg_gain & 0x00FF;
    write_cmos_sensor(0x0077,reg_gain);

    return gain;
}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
/*
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    LOG_INF("mook ihdr_write_shutter_gain\n");

    if (imgsensor.ihdr_en) {

        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;

        write_cmos_sensor1D(0x0046,0x01);

        // Extend frame length first
        LOG_INF("mook le:%d, se:%d, gain:%d, length:%d\n",le,se,gain,imgsensor.frame_length);

        write_cmos_sensor(0x0006, imgsensor.frame_length);
        // LONG
         write_cmos_sensor(0x0004, le); /// 0x0004 : long exp
         write_cmos_sensor(0x0054, se); /// 0x0054 : short exp
         set_gain(gain);

        write_cmos_sensor1D(0x0046,0x00);

        LOG_INF("--------------------- ihdr Shuuter Gain Apply -----------------\n");
    }
*/
}

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d", image_mirror);

    /********************************************************
       *
       *   0x3820[2] ISP Vertical flip
       *   0x3820[1] Sensor Vertical flip
       *
       *   0x3821[2] ISP Horizontal mirror
       *   0x3821[1] Sensor Horizontal mirror
       *
       *   ISP and Sensor flip or mirror register bit should be the same!!
       *
       ********************************************************/
    spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror;
    spin_unlock(&imgsensor_drv_lock);
    switch (image_mirror) {
        case IMAGE_NORMAL:
            write_cmos_sensor1D(0x000E,0x00);
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor1D(0x000E,0x01);
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor1D(0x000E,0x02);
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor1D(0x000E,0x03);
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
    }

}
#endif
/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}    /*    night_mode    */


/*************************************************************************
* FUNCTION
*    Initial, preview, capture(normal_video), hs_video, slim_video
*
* DESCRIPTION
*    Normal write setting
*
* RETURNS
*    None
*
*************************************************************************/



void hi1333_init_setting_normal(void)
{
  LOG_INF("Setting Ver1.27\n");
    write_cmos_sensor(0x0a00, 0x0000); //stream off
    ////////////////////////////////
    //---------TG firmware--------//
    ////////////////////////////////

    // Firmware tg_pt
    write_cmos_sensor(0x2000, 0x4031);
    write_cmos_sensor(0x2002, 0x83c8);
    write_cmos_sensor(0x2004, 0x4345);
    write_cmos_sensor(0x2006, 0x43b2);
    write_cmos_sensor(0x2008, 0x80ea);
    write_cmos_sensor(0x200a, 0x43b2);
    write_cmos_sensor(0x200c, 0x80ec);
    write_cmos_sensor(0x200e, 0x43b2);
    write_cmos_sensor(0x2010, 0x80ee);
    write_cmos_sensor(0x2012, 0x1292);
    write_cmos_sensor(0x2014, 0xd80a);
    write_cmos_sensor(0x2016, 0x0900);
    write_cmos_sensor(0x2018, 0x7314);
    write_cmos_sensor(0x201a, 0x12b0);
    write_cmos_sensor(0x201c, 0xfbb8);
    write_cmos_sensor(0x201e, 0x1292);
    write_cmos_sensor(0x2020, 0xd80e);
    write_cmos_sensor(0x2022, 0x1292);
    write_cmos_sensor(0x2024, 0xd810);
    write_cmos_sensor(0x2026, 0x1292);
    write_cmos_sensor(0x2028, 0xd812);
    write_cmos_sensor(0x202a, 0xd0f2);
    write_cmos_sensor(0x202c, 0x0003);
    write_cmos_sensor(0x202e, 0x00d8);
    write_cmos_sensor(0x2030, 0x40b2);
    write_cmos_sensor(0x2032, 0x0c30);
    write_cmos_sensor(0x2034, 0x7800);
    write_cmos_sensor(0x2036, 0x40b2);
    write_cmos_sensor(0x2038, 0x0f3c);
    write_cmos_sensor(0x203a, 0x7802);
    write_cmos_sensor(0x203c, 0x40b2);
    write_cmos_sensor(0x203e, 0x05d7);
    write_cmos_sensor(0x2040, 0x7804);
    write_cmos_sensor(0x2042, 0x40b2);
    write_cmos_sensor(0x2044, 0x0aeb);
    write_cmos_sensor(0x2046, 0x7806);
    write_cmos_sensor(0x2048, 0x12b0);
    write_cmos_sensor(0x204a, 0xf490);
    write_cmos_sensor(0x204c, 0x4392);
    write_cmos_sensor(0x204e, 0x732a);
    write_cmos_sensor(0x2050, 0x0c0f);
    write_cmos_sensor(0x2052, 0x1292);
    write_cmos_sensor(0x2054, 0xd828);
    write_cmos_sensor(0x2056, 0x93c2);
    write_cmos_sensor(0x2058, 0x809f);
    write_cmos_sensor(0x205a, 0x25c8);
    write_cmos_sensor(0x205c, 0x40b2);
    write_cmos_sensor(0x205e, 0x002f);
    write_cmos_sensor(0x2060, 0x7534);
    write_cmos_sensor(0x2062, 0x12b0);
    write_cmos_sensor(0x2064, 0xf9a4);
    write_cmos_sensor(0x2066, 0x1292);
    write_cmos_sensor(0x2068, 0xd81e);
    write_cmos_sensor(0x206a, 0x1292);
    write_cmos_sensor(0x206c, 0xd820);
    write_cmos_sensor(0x206e, 0x43c2);
    write_cmos_sensor(0x2070, 0x009c);
    write_cmos_sensor(0x2072, 0x40b2);
    write_cmos_sensor(0x2074, 0x081b);
    write_cmos_sensor(0x2076, 0x7500);
    write_cmos_sensor(0x2078, 0x40b2);
    write_cmos_sensor(0x207a, 0x043f);
    write_cmos_sensor(0x207c, 0x7502);
    write_cmos_sensor(0x207e, 0x40b2);
    write_cmos_sensor(0x2080, 0x181b);
    write_cmos_sensor(0x2082, 0x7504);
    write_cmos_sensor(0x2084, 0x40b2);
    write_cmos_sensor(0x2086, 0x0809);
    write_cmos_sensor(0x2088, 0x7506);
    write_cmos_sensor(0x208a, 0x40b2);
    write_cmos_sensor(0x208c, 0x082d);
    write_cmos_sensor(0x208e, 0x7508);
    write_cmos_sensor(0x2090, 0x40b2);
    write_cmos_sensor(0x2092, 0x3c24);
    write_cmos_sensor(0x2094, 0x750a);
    write_cmos_sensor(0x2096, 0x40b2);
    write_cmos_sensor(0x2098, 0x0800);
    write_cmos_sensor(0x209a, 0x750c);
    write_cmos_sensor(0x209c, 0x40b2);
    write_cmos_sensor(0x209e, 0x0409);
    write_cmos_sensor(0x20a0, 0x750e);
    write_cmos_sensor(0x20a2, 0x40b2);
    write_cmos_sensor(0x20a4, 0x041b);
    write_cmos_sensor(0x20a6, 0x7510);
    write_cmos_sensor(0x20a8, 0x40b2);
    write_cmos_sensor(0x20aa, 0x0812);
    write_cmos_sensor(0x20ac, 0x7512);
    write_cmos_sensor(0x20ae, 0x4392);
    write_cmos_sensor(0x20b0, 0x7f06);
    write_cmos_sensor(0x20b2, 0x43a2);
    write_cmos_sensor(0x20b4, 0x7f0a);
    write_cmos_sensor(0x20b6, 0xb2f2);
    write_cmos_sensor(0x20b8, 0x00d0);
    write_cmos_sensor(0x20ba, 0x257b);
    write_cmos_sensor(0x20bc, 0x421e);
    write_cmos_sensor(0x20be, 0x00de);
    write_cmos_sensor(0x20c0, 0x0800);
    write_cmos_sensor(0x20c2, 0x7f08);
    write_cmos_sensor(0x20c4, 0x425f);
    write_cmos_sensor(0x20c6, 0x00d6);
    write_cmos_sensor(0x20c8, 0xf37f);
    write_cmos_sensor(0x20ca, 0x4f82);
    write_cmos_sensor(0x20cc, 0x7f00);
    write_cmos_sensor(0x20ce, 0x4e82);
    write_cmos_sensor(0x20d0, 0x7f02);
    write_cmos_sensor(0x20d2, 0x0800);
    write_cmos_sensor(0x20d4, 0x7f08);
    write_cmos_sensor(0x20d6, 0x425f);
    write_cmos_sensor(0x20d8, 0x00d6);
    write_cmos_sensor(0x20da, 0xf37f);
    write_cmos_sensor(0x20dc, 0x521f);
    write_cmos_sensor(0x20de, 0x80b2);
    write_cmos_sensor(0x20e0, 0x4f82);
    write_cmos_sensor(0x20e2, 0x7f00);
    write_cmos_sensor(0x20e4, 0x4e82);
    write_cmos_sensor(0x20e6, 0x7f02);
    write_cmos_sensor(0x20e8, 0xb3e2);
    write_cmos_sensor(0x20ea, 0x00c3);
    write_cmos_sensor(0x20ec, 0x255f);
    write_cmos_sensor(0x20ee, 0x40b2);
    write_cmos_sensor(0x20f0, 0x001f);
    write_cmos_sensor(0x20f2, 0x7322);
    write_cmos_sensor(0x20f4, 0x93c2);
    write_cmos_sensor(0x20f6, 0x80a0);
    write_cmos_sensor(0x20f8, 0x2445);
    write_cmos_sensor(0x20fa, 0x9382);
    write_cmos_sensor(0x20fc, 0x0790);
    write_cmos_sensor(0x20fe, 0x354c);
    write_cmos_sensor(0x2100, 0x421f);
    write_cmos_sensor(0x2102, 0x0790);
    write_cmos_sensor(0x2104, 0xf03f);
    write_cmos_sensor(0x2106, 0x7fff);
    write_cmos_sensor(0x2108, 0x4f82);
    write_cmos_sensor(0x210a, 0x80ea);
    write_cmos_sensor(0x210c, 0x4292);
    write_cmos_sensor(0x210e, 0x0792);
    write_cmos_sensor(0x2110, 0x80ec);
    write_cmos_sensor(0x2112, 0x4292);
    write_cmos_sensor(0x2114, 0x0794);
    write_cmos_sensor(0x2116, 0x80ee);
    write_cmos_sensor(0x2118, 0xc2e2);
    write_cmos_sensor(0x211a, 0x0f84);
    write_cmos_sensor(0x211c, 0x4349);
    write_cmos_sensor(0x211e, 0x9349);
    write_cmos_sensor(0x2120, 0x2138);
    write_cmos_sensor(0x2122, 0x4036);
    write_cmos_sensor(0x2124, 0x02ac);
    write_cmos_sensor(0x2126, 0x434a);
    write_cmos_sensor(0x2128, 0x494c);
    write_cmos_sensor(0x212a, 0x4a4b);
    write_cmos_sensor(0x212c, 0x4c0f);
    write_cmos_sensor(0x212e, 0x5f0f);
    write_cmos_sensor(0x2130, 0x5f0f);
    write_cmos_sensor(0x2132, 0x5f0f);
    write_cmos_sensor(0x2134, 0x4c0d);
    write_cmos_sensor(0x2136, 0x5d0d);
    write_cmos_sensor(0x2138, 0x5d0f);
    write_cmos_sensor(0x213a, 0x5d0f);
    write_cmos_sensor(0x213c, 0x5d0f);
    write_cmos_sensor(0x213e, 0x5b0f);
    write_cmos_sensor(0x2140, 0x5f0f);
    write_cmos_sensor(0x2142, 0x510f);
    write_cmos_sensor(0x2144, 0x4b0e);
    write_cmos_sensor(0x2146, 0x5e0e);
    write_cmos_sensor(0x2148, 0x560e);
    write_cmos_sensor(0x214a, 0x4e9f);
    write_cmos_sensor(0x214c, 0x6000);
    write_cmos_sensor(0x214e, 0x0000);
    write_cmos_sensor(0x2150, 0x4e9f);
    write_cmos_sensor(0x2152, 0x5000);
    write_cmos_sensor(0x2154, 0x000e);
    write_cmos_sensor(0x2156, 0x4d0f);
    write_cmos_sensor(0x2158, 0x5c0f);
    write_cmos_sensor(0x215a, 0x5f0f);
    write_cmos_sensor(0x215c, 0x5c0f);
    write_cmos_sensor(0x215e, 0x5b0f);
    write_cmos_sensor(0x2160, 0x5f0f);
    write_cmos_sensor(0x2162, 0x503f);
    write_cmos_sensor(0x2164, 0x80aa);
    write_cmos_sensor(0x2166, 0x4e9f);
    write_cmos_sensor(0x2168, 0x5fd6);
    write_cmos_sensor(0x216a, 0x0062);
    write_cmos_sensor(0x216c, 0x4e9f);
    write_cmos_sensor(0x216e, 0x4fd6);
    write_cmos_sensor(0x2170, 0x0046);
    write_cmos_sensor(0x2172, 0x535a);
    write_cmos_sensor(0x2174, 0x907a);
    write_cmos_sensor(0x2176, 0x0007);
    write_cmos_sensor(0x2178, 0x2bd7);
    write_cmos_sensor(0x217a, 0x5359);
    write_cmos_sensor(0x217c, 0x9369);
    write_cmos_sensor(0x217e, 0x2bcf);
    write_cmos_sensor(0x2180, 0xd2e2);
    write_cmos_sensor(0x2182, 0x0f84);
    write_cmos_sensor(0x2184, 0x4392);
    write_cmos_sensor(0x2186, 0x7320);
    write_cmos_sensor(0x2188, 0x93c2);
    write_cmos_sensor(0x218a, 0x80a0);
    write_cmos_sensor(0x218c, 0x2016);
    write_cmos_sensor(0x218e, 0x93c2);
    write_cmos_sensor(0x2190, 0x00d0);
    write_cmos_sensor(0x2192, 0x3403);
    write_cmos_sensor(0x2194, 0x4382);
    write_cmos_sensor(0x2196, 0x0b8a);
    write_cmos_sensor(0x2198, 0x0c0a);
    write_cmos_sensor(0x219a, 0x0900);
    write_cmos_sensor(0x219c, 0x730a);
    write_cmos_sensor(0x219e, 0x1292);
    write_cmos_sensor(0x21a0, 0xd828);
    write_cmos_sensor(0x21a2, 0x93c2);
    write_cmos_sensor(0x21a4, 0x809f);
    write_cmos_sensor(0x21a6, 0x24f1);
    write_cmos_sensor(0x21a8, 0x40b2);
    write_cmos_sensor(0x21aa, 0x002f);
    write_cmos_sensor(0x21ac, 0x7534);
    write_cmos_sensor(0x21ae, 0x12b0);
    write_cmos_sensor(0x21b0, 0xf9a4);
    write_cmos_sensor(0x21b2, 0x1292);
    write_cmos_sensor(0x21b4, 0xd81e);
    write_cmos_sensor(0x21b6, 0x1292);
    write_cmos_sensor(0x21b8, 0xd820);
    write_cmos_sensor(0x21ba, 0x425b);
    write_cmos_sensor(0x21bc, 0x07a3);
    write_cmos_sensor(0x21be, 0x4349);
    write_cmos_sensor(0x21c0, 0xb3d2);
    write_cmos_sensor(0x21c2, 0x0480);
    write_cmos_sensor(0x21c4, 0x240e);
    write_cmos_sensor(0x21c6, 0x93c2);
    write_cmos_sensor(0x21c8, 0x809e);
    write_cmos_sensor(0x21ca, 0x240b);
    write_cmos_sensor(0x21cc, 0x425f);
    write_cmos_sensor(0x21ce, 0x0a8e);
    write_cmos_sensor(0x21d0, 0xc312);
    write_cmos_sensor(0x21d2, 0x104f);
    write_cmos_sensor(0x21d4, 0x114f);
    write_cmos_sensor(0x21d6, 0x114f);
    write_cmos_sensor(0x21d8, 0x114f);
    write_cmos_sensor(0x21da, 0x907f);
    write_cmos_sensor(0x21dc, 0x0003);
    write_cmos_sensor(0x21de, 0x2c01);
    write_cmos_sensor(0x21e0, 0x4359);
    write_cmos_sensor(0x21e2, 0x434a);
    write_cmos_sensor(0x21e4, 0x9349);
    write_cmos_sensor(0x21e6, 0x2404);
    write_cmos_sensor(0x21e8, 0x9345);
    write_cmos_sensor(0x21ea, 0x2402);
    write_cmos_sensor(0x21ec, 0x12b0);
    write_cmos_sensor(0x21ee, 0xfc4e);
    write_cmos_sensor(0x21f0, 0x1292);
    write_cmos_sensor(0x21f2, 0xd824);
    write_cmos_sensor(0x21f4, 0x1292);
    write_cmos_sensor(0x21f6, 0xd822);
    write_cmos_sensor(0x21f8, 0xb3d2);
    write_cmos_sensor(0x21fa, 0x00bf);
    write_cmos_sensor(0x21fc, 0x2002);
    write_cmos_sensor(0x21fe, 0x43c2);
    write_cmos_sensor(0x2200, 0x009c);
    write_cmos_sensor(0x2202, 0x12b0);
    write_cmos_sensor(0x2204, 0xfcea);
    write_cmos_sensor(0x2206, 0x4382);
    write_cmos_sensor(0x2208, 0x7114);
    write_cmos_sensor(0x220a, 0x43c2);
    write_cmos_sensor(0x220c, 0x80a9);
    write_cmos_sensor(0x220e, 0xb3e2);
    write_cmos_sensor(0x2210, 0x00c3);
    write_cmos_sensor(0x2212, 0x434e);
    write_cmos_sensor(0x2214, 0x634e);
    write_cmos_sensor(0x2216, 0xb3d2);
    write_cmos_sensor(0x2218, 0x00bf);
    write_cmos_sensor(0x221a, 0x2004);
    write_cmos_sensor(0x221c, 0xb35e);
    write_cmos_sensor(0x221e, 0x2402);
    write_cmos_sensor(0x2220, 0x43d2);
    write_cmos_sensor(0x2222, 0x009c);
    write_cmos_sensor(0x2224, 0x4392);
    write_cmos_sensor(0x2226, 0x7f0c);
    write_cmos_sensor(0x2228, 0x4392);
    write_cmos_sensor(0x222a, 0x7f10);
    write_cmos_sensor(0x222c, 0x4392);
    write_cmos_sensor(0x222e, 0x770a);
    write_cmos_sensor(0x2230, 0x4392);
    write_cmos_sensor(0x2232, 0x770e);
    write_cmos_sensor(0x2234, 0x9392);
    write_cmos_sensor(0x2236, 0x7114);
    write_cmos_sensor(0x2238, 0x2009);
    write_cmos_sensor(0x223a, 0x12b0);
    write_cmos_sensor(0x223c, 0xf6bc);
    write_cmos_sensor(0x223e, 0x0900);
    write_cmos_sensor(0x2240, 0x7112);
    write_cmos_sensor(0x2242, 0x1292);
    write_cmos_sensor(0x2244, 0xd81a);
    write_cmos_sensor(0x2246, 0x1292);
    write_cmos_sensor(0x2248, 0xd83e);
    write_cmos_sensor(0x224a, 0x3ff4);
    write_cmos_sensor(0x224c, 0x0b00);
    write_cmos_sensor(0x224e, 0x7304);
    write_cmos_sensor(0x2250, 0x0000);
    write_cmos_sensor(0x2252, 0xb35a);
    write_cmos_sensor(0x2254, 0x247f);
    write_cmos_sensor(0x2256, 0x403e);
    write_cmos_sensor(0x2258, 0x0f84);
    write_cmos_sensor(0x225a, 0xc2ee);
    write_cmos_sensor(0x225c, 0x0000);
    write_cmos_sensor(0x225e, 0x474f);
    write_cmos_sensor(0x2260, 0x5f0f);
    write_cmos_sensor(0x2262, 0x510f);
    write_cmos_sensor(0x2264, 0x4fa8);
    write_cmos_sensor(0x2266, 0x0000);
    write_cmos_sensor(0x2268, 0x4f98);
    write_cmos_sensor(0x226a, 0x0002);
    write_cmos_sensor(0x226c, 0x0002);
    write_cmos_sensor(0x226e, 0x4f98);
    write_cmos_sensor(0x2270, 0x0004);
    write_cmos_sensor(0x2272, 0x0004);
    write_cmos_sensor(0x2274, 0x4f98);
    write_cmos_sensor(0x2276, 0x0006);
    write_cmos_sensor(0x2278, 0x0006);
    write_cmos_sensor(0x227a, 0x4f98);
    write_cmos_sensor(0x227c, 0x0008);
    write_cmos_sensor(0x227e, 0x0008);
    write_cmos_sensor(0x2280, 0x4f98);
    write_cmos_sensor(0x2282, 0x000a);
    write_cmos_sensor(0x2284, 0x000a);
    write_cmos_sensor(0x2286, 0x4f98);
    write_cmos_sensor(0x2288, 0x000c);
    write_cmos_sensor(0x228a, 0x000c);
    write_cmos_sensor(0x228c, 0xd2ee);
    write_cmos_sensor(0x228e, 0x0000);
    write_cmos_sensor(0x2290, 0x403f);
    write_cmos_sensor(0x2292, 0x80c0);
    write_cmos_sensor(0x2294, 0x429f);
    write_cmos_sensor(0x2296, 0x7100);
    write_cmos_sensor(0x2298, 0x0000);
    write_cmos_sensor(0x229a, 0x4f2f);
    write_cmos_sensor(0x229c, 0x903f);
    write_cmos_sensor(0x229e, 0x0028);
    write_cmos_sensor(0x22a0, 0x2814);
    write_cmos_sensor(0x22a2, 0x4f0e);
    write_cmos_sensor(0x22a4, 0x503e);
    write_cmos_sensor(0x22a6, 0xffd8);
    write_cmos_sensor(0x22a8, 0x4e82);
    write_cmos_sensor(0x22aa, 0x80c6);
    write_cmos_sensor(0x22ac, 0x4e82);
    write_cmos_sensor(0x22ae, 0x7a04);
    write_cmos_sensor(0x22b0, 0xb36b);
    write_cmos_sensor(0x22b2, 0x2439);
    write_cmos_sensor(0x22b4, 0x4e0f);
    write_cmos_sensor(0x22b6, 0xe33f);
    write_cmos_sensor(0x22b8, 0xf32f);
    write_cmos_sensor(0x22ba, 0xc32e);
    write_cmos_sensor(0x22bc, 0xdf0e);
    write_cmos_sensor(0x22be, 0x4e82);
    write_cmos_sensor(0x22c0, 0x7a06);
    write_cmos_sensor(0x22c2, 0x4392);
    write_cmos_sensor(0x22c4, 0x7a0a);
    write_cmos_sensor(0x22c6, 0x0800);
    write_cmos_sensor(0x22c8, 0x7a0a);
    write_cmos_sensor(0x22ca, 0xb35a);
    write_cmos_sensor(0x22cc, 0x240c);
    write_cmos_sensor(0x22ce, 0x5355);
    write_cmos_sensor(0x22d0, 0x5038);
    write_cmos_sensor(0x22d2, 0xf000);
    write_cmos_sensor(0x22d4, 0x454a);
    write_cmos_sensor(0x22d6, 0xf35a);
    write_cmos_sensor(0x22d8, 0x1292);
    write_cmos_sensor(0x22da, 0xd83e);
    write_cmos_sensor(0x22dc, 0x930f);
    write_cmos_sensor(0x22de, 0x27aa);
    write_cmos_sensor(0x22e0, 0x43c2);
    write_cmos_sensor(0x22e2, 0x80a0);
    write_cmos_sensor(0x22e4, 0x3f51);
    write_cmos_sensor(0x22e6, 0x9349);
    write_cmos_sensor(0x22e8, 0x27f7);
    write_cmos_sensor(0x22ea, 0x421f);
    write_cmos_sensor(0x22ec, 0x80c0);
    write_cmos_sensor(0x22ee, 0x921f);
    write_cmos_sensor(0x22f0, 0x80ec);
    write_cmos_sensor(0x22f2, 0x2414);
    write_cmos_sensor(0x22f4, 0x921f);
    write_cmos_sensor(0x22f6, 0x80ee);
    write_cmos_sensor(0x22f8, 0x240a);
    write_cmos_sensor(0x22fa, 0x921f);
    write_cmos_sensor(0x22fc, 0x80ea);
    write_cmos_sensor(0x22fe, 0x23ec);
    write_cmos_sensor(0x2300, 0x436a);
    write_cmos_sensor(0x2302, 0x4347);
    write_cmos_sensor(0x2304, 0x4074);
    write_cmos_sensor(0x2306, 0x001c);
    write_cmos_sensor(0x2308, 0x4038);
    write_cmos_sensor(0x230a, 0x022e);
    write_cmos_sensor(0x230c, 0x3fe5);
    write_cmos_sensor(0x230e, 0x435a);
    write_cmos_sensor(0x2310, 0x4077);
    write_cmos_sensor(0x2312, 0x000e);
    write_cmos_sensor(0x2314, 0x4038);
    write_cmos_sensor(0x2316, 0x624a);
    write_cmos_sensor(0x2318, 0x4345);
    write_cmos_sensor(0x231a, 0x3fde);
    write_cmos_sensor(0x231c, 0x435a);
    write_cmos_sensor(0x231e, 0x4347);
    write_cmos_sensor(0x2320, 0x4038);
    write_cmos_sensor(0x2322, 0x629e);
    write_cmos_sensor(0x2324, 0x3ff9);
    write_cmos_sensor(0x2326, 0xb35b);
    write_cmos_sensor(0x2328, 0x2408);
    write_cmos_sensor(0x232a, 0x4e0f);
    write_cmos_sensor(0x232c, 0xe33f);
    write_cmos_sensor(0x232e, 0xf22f);
    write_cmos_sensor(0x2330, 0xc22e);
    write_cmos_sensor(0x2332, 0xdf0e);
    write_cmos_sensor(0x2334, 0x4e82);
    write_cmos_sensor(0x2336, 0x7a0e);
    write_cmos_sensor(0x2338, 0x3fc4);
    write_cmos_sensor(0x233a, 0x907b);
    write_cmos_sensor(0x233c, 0x0003);
    write_cmos_sensor(0x233e, 0x23c1);
    write_cmos_sensor(0x2340, 0x4e0f);
    write_cmos_sensor(0x2342, 0xe33f);
    write_cmos_sensor(0x2344, 0xf03f);
    write_cmos_sensor(0x2346, 0x0006);
    write_cmos_sensor(0x2348, 0xf03e);
    write_cmos_sensor(0x234a, 0xfff9);
    write_cmos_sensor(0x234c, 0xdf0e);
    write_cmos_sensor(0x234e, 0x4e82);
    write_cmos_sensor(0x2350, 0x7a10);
    write_cmos_sensor(0x2352, 0x3fb7);
    write_cmos_sensor(0x2354, 0xb36a);
    write_cmos_sensor(0x2356, 0x279c);
    write_cmos_sensor(0x2358, 0x9447);
    write_cmos_sensor(0x235a, 0x2c15);
    write_cmos_sensor(0x235c, 0x9077);
    write_cmos_sensor(0x235e, 0x000e);
    write_cmos_sensor(0x2360, 0x240f);
    write_cmos_sensor(0x2362, 0x403f);
    write_cmos_sensor(0x2364, 0x0f84);
    write_cmos_sensor(0x2366, 0xc2ef);
    write_cmos_sensor(0x2368, 0x0000);
    write_cmos_sensor(0x236a, 0x4898);
    write_cmos_sensor(0x236c, 0x600e);
    write_cmos_sensor(0x236e, 0x6000);
    write_cmos_sensor(0x2370, 0x4898);
    write_cmos_sensor(0x2372, 0x500e);
    write_cmos_sensor(0x2374, 0x5000);
    write_cmos_sensor(0x2376, 0xd2ef);
    write_cmos_sensor(0x2378, 0x0000);
    write_cmos_sensor(0x237a, 0x5328);
    write_cmos_sensor(0x237c, 0x5357);
    write_cmos_sensor(0x237e, 0x3f88);
    write_cmos_sensor(0x2380, 0x4038);
    write_cmos_sensor(0x2382, 0x0282);
    write_cmos_sensor(0x2384, 0x3fee);
    write_cmos_sensor(0x2386, 0x434a);
    write_cmos_sensor(0x2388, 0x3f83);
    write_cmos_sensor(0x238a, 0x40b2);
    write_cmos_sensor(0x238c, 0x0007);
    write_cmos_sensor(0x238e, 0x7534);
    write_cmos_sensor(0x2390, 0x3f0e);
    write_cmos_sensor(0x2392, 0x4036);
    write_cmos_sensor(0x2394, 0x0258);
    write_cmos_sensor(0x2396, 0x3ec7);
    write_cmos_sensor(0x2398, 0x40b2);
    write_cmos_sensor(0x239a, 0x02f0);
    write_cmos_sensor(0x239c, 0x80ea);
    write_cmos_sensor(0x239e, 0x40b2);
    write_cmos_sensor(0x23a0, 0x02ad);
    write_cmos_sensor(0x23a2, 0x80ec);
    write_cmos_sensor(0x23a4, 0x40b2);
    write_cmos_sensor(0x23a6, 0x02a0);
    write_cmos_sensor(0x23a8, 0x80ee);
    write_cmos_sensor(0x23aa, 0x3eb6);
    write_cmos_sensor(0x23ac, 0x42b2);
    write_cmos_sensor(0x23ae, 0x7322);
    write_cmos_sensor(0x23b0, 0x3ea1);
    write_cmos_sensor(0x23b2, 0x42e2);
    write_cmos_sensor(0x23b4, 0x00d6);
    write_cmos_sensor(0x23b6, 0x40b2);
    write_cmos_sensor(0x23b8, 0x1f24);
    write_cmos_sensor(0x23ba, 0x3000);
    write_cmos_sensor(0x23bc, 0x40b2);
    write_cmos_sensor(0x23be, 0x0807);
    write_cmos_sensor(0x23c0, 0x4000);
    write_cmos_sensor(0x23c2, 0x40b2);
    write_cmos_sensor(0x23c4, 0x1f3c);
    write_cmos_sensor(0x23c6, 0x3002);
    write_cmos_sensor(0x23c8, 0x40b2);
    write_cmos_sensor(0x23ca, 0x2807);
    write_cmos_sensor(0x23cc, 0x4002);
    write_cmos_sensor(0x23ce, 0x40b2);
    write_cmos_sensor(0x23d0, 0xf944);
    write_cmos_sensor(0x23d2, 0x3004);
    write_cmos_sensor(0x23d4, 0x40b2);
    write_cmos_sensor(0x23d6, 0x2006);
    write_cmos_sensor(0x23d8, 0x4004);
    write_cmos_sensor(0x23da, 0x40b2);
    write_cmos_sensor(0x23dc, 0x1c04);
    write_cmos_sensor(0x23de, 0x3006);
    write_cmos_sensor(0x23e0, 0x40b2);
    write_cmos_sensor(0x23e2, 0x0032);
    write_cmos_sensor(0x23e4, 0x4006);
    write_cmos_sensor(0x23e6, 0x403e);
    write_cmos_sensor(0x23e8, 0xf3fc);
    write_cmos_sensor(0x23ea, 0x3e6a);
    write_cmos_sensor(0x23ec, 0x40b2);
    write_cmos_sensor(0x23ee, 0x0007);
    write_cmos_sensor(0x23f0, 0x7534);
    write_cmos_sensor(0x23f2, 0x3e37);
    write_cmos_sensor(0x23f4, 0x5031);
    write_cmos_sensor(0x23f6, 0x0038);
    write_cmos_sensor(0x23f8, 0x4030);
    write_cmos_sensor(0x23fa, 0xfd7c);
    write_cmos_sensor(0x23fc, 0x7400);
    write_cmos_sensor(0x23fe, 0x202c);
    write_cmos_sensor(0x2400, 0x2320);
    write_cmos_sensor(0x2402, 0x334b);
    write_cmos_sensor(0x2404, 0x0044);
    write_cmos_sensor(0x2406, 0x0046);
    write_cmos_sensor(0x2408, 0x004b);
    write_cmos_sensor(0x240a, 0x00c9);
    write_cmos_sensor(0x240c, 0x0044);
    write_cmos_sensor(0x240e, 0x7000);
    write_cmos_sensor(0x2410, 0x4e07);
    write_cmos_sensor(0x2412, 0x0058);
    write_cmos_sensor(0x2414, 0x0049);
    write_cmos_sensor(0x2416, 0x5003);
    write_cmos_sensor(0x2418, 0x5021);
    write_cmos_sensor(0x241a, 0x004b);
    write_cmos_sensor(0x241c, 0x0ad3);
    write_cmos_sensor(0x241e, 0x00d4);
    write_cmos_sensor(0x2420, 0x0055);
    write_cmos_sensor(0x2422, 0x0024);
    write_cmos_sensor(0x2424, 0x0199);
    write_cmos_sensor(0x2426, 0x700b);
    write_cmos_sensor(0x2428, 0x2fcb);
    write_cmos_sensor(0x242a, 0x0056);
    write_cmos_sensor(0x242c, 0x00dd);
    write_cmos_sensor(0x242e, 0x005a);
    write_cmos_sensor(0x2430, 0x0025);
    write_cmos_sensor(0x2432, 0x20b0);
    write_cmos_sensor(0x2434, 0x0765);
    write_cmos_sensor(0x2436, 0x20bc);
    write_cmos_sensor(0x2438, 0x20e5);
    write_cmos_sensor(0x243a, 0x1683);
    write_cmos_sensor(0x243c, 0x0002);
    write_cmos_sensor(0x243e, 0x0005);
    write_cmos_sensor(0x2440, 0x2e08);
    write_cmos_sensor(0x2442, 0x0045);
    write_cmos_sensor(0x2444, 0x200b);
    write_cmos_sensor(0x2446, 0x2007);
    write_cmos_sensor(0x2448, 0x0b49);
    write_cmos_sensor(0x244a, 0x00ca);
    write_cmos_sensor(0x244c, 0x00c9);
    write_cmos_sensor(0x244e, 0x00ca);
    write_cmos_sensor(0x2450, 0x055a);
    write_cmos_sensor(0x2452, 0x20e9);
    write_cmos_sensor(0x2454, 0x20b0);
    write_cmos_sensor(0x2456, 0x704d);
    write_cmos_sensor(0x2458, 0x2fe5);
    write_cmos_sensor(0x245a, 0x5040);
    write_cmos_sensor(0x245c, 0x0025);
    write_cmos_sensor(0x245e, 0x5060);
    write_cmos_sensor(0x2460, 0x2118);
    write_cmos_sensor(0x2462, 0x3108);
    write_cmos_sensor(0x2464, 0x7800);
    write_cmos_sensor(0x2466, 0x3108);
    write_cmos_sensor(0x2468, 0x01c1);
    write_cmos_sensor(0x246a, 0x01c4);
    write_cmos_sensor(0x246c, 0x01c5);
    write_cmos_sensor(0x246e, 0x01c1);
    write_cmos_sensor(0x2470, 0x3708);
    write_cmos_sensor(0x2472, 0x3308);
    write_cmos_sensor(0x2474, 0x7800);
    write_cmos_sensor(0x2476, 0x8087);
    write_cmos_sensor(0x2478, 0x9564);
    write_cmos_sensor(0x247a, 0xc91c);
    write_cmos_sensor(0x247c, 0x5085);
    write_cmos_sensor(0x247e, 0xa765);
    write_cmos_sensor(0x2480, 0xfa95);
    write_cmos_sensor(0x2482, 0xfe74);
    write_cmos_sensor(0x2484, 0x6829);
    write_cmos_sensor(0x2486, 0x5000);
    write_cmos_sensor(0x2488, 0x0000);
    write_cmos_sensor(0x248a, 0x0311);
    write_cmos_sensor(0x248c, 0x07ea);
    write_cmos_sensor(0x248e, 0x0000);
    write_cmos_sensor(0x2490, 0x120b);
    write_cmos_sensor(0x2492, 0x120a);
    write_cmos_sensor(0x2494, 0x1209);
    write_cmos_sensor(0x2496, 0x1208);
    write_cmos_sensor(0x2498, 0x1207);
    write_cmos_sensor(0x249a, 0x1206);
    write_cmos_sensor(0x249c, 0x1205);
    write_cmos_sensor(0x249e, 0x1204);
    write_cmos_sensor(0x24a0, 0x8321);
    write_cmos_sensor(0x24a2, 0x43d2);
    write_cmos_sensor(0x24a4, 0x0780);
    write_cmos_sensor(0x24a6, 0x4382);
    write_cmos_sensor(0x24a8, 0x80c4);
    write_cmos_sensor(0x24aa, 0x1292);
    write_cmos_sensor(0x24ac, 0xd830);
    write_cmos_sensor(0x24ae, 0x4309);
    write_cmos_sensor(0x24b0, 0xb3d2);
    write_cmos_sensor(0x24b2, 0x07dc);
    write_cmos_sensor(0x24b4, 0x2419);
    write_cmos_sensor(0x24b6, 0x430a);
    write_cmos_sensor(0x24b8, 0x403f);
    write_cmos_sensor(0x24ba, 0x022e);
    write_cmos_sensor(0x24bc, 0x8a0f);
    write_cmos_sensor(0x24be, 0x434d);
    write_cmos_sensor(0x24c0, 0x436e);
    write_cmos_sensor(0x24c2, 0x1292);
    write_cmos_sensor(0x24c4, 0xd834);
    write_cmos_sensor(0x24c6, 0x490b);
    write_cmos_sensor(0x24c8, 0x5b0b);
    write_cmos_sensor(0x24ca, 0x503b);
    write_cmos_sensor(0x24cc, 0x814c);
    write_cmos_sensor(0x24ce, 0x5319);
    write_cmos_sensor(0x24d0, 0x425e);
    write_cmos_sensor(0x24d2, 0x80a3);
    write_cmos_sensor(0x24d4, 0x425f);
    write_cmos_sensor(0x24d6, 0x80a4);
    write_cmos_sensor(0x24d8, 0x1292);
    write_cmos_sensor(0x24da, 0xd838);
    write_cmos_sensor(0x24dc, 0x4f8b);
    write_cmos_sensor(0x24de, 0x004a);
    write_cmos_sensor(0x24e0, 0x532a);
    write_cmos_sensor(0x24e2, 0x903a);
    write_cmos_sensor(0x24e4, 0x0200);
    write_cmos_sensor(0x24e6, 0x2be8);
    write_cmos_sensor(0x24e8, 0x4306);
    write_cmos_sensor(0x24ea, 0x9306);
    write_cmos_sensor(0x24ec, 0x20c8);
    write_cmos_sensor(0x24ee, 0x4292);
    write_cmos_sensor(0x24f0, 0x07de);
    write_cmos_sensor(0x24f2, 0x80c4);
    write_cmos_sensor(0x24f4, 0x4038);
    write_cmos_sensor(0x24f6, 0x0536);
    write_cmos_sensor(0x24f8, 0x403b);
    write_cmos_sensor(0x24fa, 0x0363);
    write_cmos_sensor(0x24fc, 0x4325);
    write_cmos_sensor(0x24fe, 0x40b1);
    write_cmos_sensor(0x2500, 0x0006);
    write_cmos_sensor(0x2502, 0x0000);
    write_cmos_sensor(0x2504, 0x4037);
    write_cmos_sensor(0x2506, 0x035a);
    write_cmos_sensor(0x2508, 0x4034);
    write_cmos_sensor(0x250a, 0x0362);
    write_cmos_sensor(0x250c, 0x1292);
    write_cmos_sensor(0x250e, 0xd836);
    write_cmos_sensor(0x2510, 0x903f);
    write_cmos_sensor(0x2512, 0x0013);
    write_cmos_sensor(0x2514, 0x24b0);
    write_cmos_sensor(0x2516, 0x903f);
    write_cmos_sensor(0x2518, 0x0014);
    write_cmos_sensor(0x251a, 0x34a9);
    write_cmos_sensor(0x251c, 0x931f);
    write_cmos_sensor(0x251e, 0x24a3);
    write_cmos_sensor(0x2520, 0x454f);
    write_cmos_sensor(0x2522, 0x5f4f);
    write_cmos_sensor(0x2524, 0x5f4f);
    write_cmos_sensor(0x2526, 0x5f4f);
    write_cmos_sensor(0x2528, 0x5f4f);
    write_cmos_sensor(0x252a, 0xdfc2);
    write_cmos_sensor(0x252c, 0x07dc);
    write_cmos_sensor(0x252e, 0x4382);
    write_cmos_sensor(0x2530, 0x8148);
    write_cmos_sensor(0x2532, 0x4382);
    write_cmos_sensor(0x2534, 0x814a);
    write_cmos_sensor(0x2536, 0x425f);
    write_cmos_sensor(0x2538, 0x07dc);
    write_cmos_sensor(0x253a, 0xf37f);
    write_cmos_sensor(0x253c, 0xf50f);
    write_cmos_sensor(0x253e, 0x2407);
    write_cmos_sensor(0x2540, 0x9306);
    write_cmos_sensor(0x2542, 0x248f);
    write_cmos_sensor(0x2544, 0x4039);
    write_cmos_sensor(0x2546, 0x023c);
    write_cmos_sensor(0x2548, 0x430a);
    write_cmos_sensor(0x254a, 0x970a);
    write_cmos_sensor(0x254c, 0x2864);
    write_cmos_sensor(0x254e, 0x5408);
    write_cmos_sensor(0x2550, 0x4882);
    write_cmos_sensor(0x2552, 0x80c4);
    write_cmos_sensor(0x2554, 0x4307);
    write_cmos_sensor(0x2556, 0x421c);
    write_cmos_sensor(0x2558, 0x8148);
    write_cmos_sensor(0x255a, 0x421d);
    write_cmos_sensor(0x255c, 0x814a);
    write_cmos_sensor(0x255e, 0x403a);
    write_cmos_sensor(0x2560, 0x00ff);
    write_cmos_sensor(0x2562, 0x430b);
    write_cmos_sensor(0x2564, 0x12b0);
    write_cmos_sensor(0x2566, 0xfdd6);
    write_cmos_sensor(0x2568, 0x4e0a);
    write_cmos_sensor(0x256a, 0x4f0b);
    write_cmos_sensor(0x256c, 0x531a);
    write_cmos_sensor(0x256e, 0x630b);
    write_cmos_sensor(0x2570, 0x1292);
    write_cmos_sensor(0x2572, 0xd836);
    write_cmos_sensor(0x2574, 0x4f0e);
    write_cmos_sensor(0x2576, 0x4e0f);
    write_cmos_sensor(0x2578, 0x5f0f);
    write_cmos_sensor(0x257a, 0x7f0f);
    write_cmos_sensor(0x257c, 0xe33f);
    write_cmos_sensor(0x257e, 0x9e0a);
    write_cmos_sensor(0x2580, 0x2447);
    write_cmos_sensor(0x2582, 0x4317);
    write_cmos_sensor(0x2584, 0x430e);
    write_cmos_sensor(0x2586, 0x425f);
    write_cmos_sensor(0x2588, 0x07dc);
    write_cmos_sensor(0x258a, 0xf37f);
    write_cmos_sensor(0x258c, 0xf12f);
    write_cmos_sensor(0x258e, 0x2401);
    write_cmos_sensor(0x2590, 0x431e);
    write_cmos_sensor(0x2592, 0x470f);
    write_cmos_sensor(0x2594, 0xfe0f);
    write_cmos_sensor(0x2596, 0x2406);
    write_cmos_sensor(0x2598, 0x454f);
    write_cmos_sensor(0x259a, 0x5f4f);
    write_cmos_sensor(0x259c, 0x5f4f);
    write_cmos_sensor(0x259e, 0x5f4f);
    write_cmos_sensor(0x25a0, 0xdfc2);
    write_cmos_sensor(0x25a2, 0x07dc);
    write_cmos_sensor(0x25a4, 0x5316);
    write_cmos_sensor(0x25a6, 0x9326);
    write_cmos_sensor(0x25a8, 0x2ba0);
    write_cmos_sensor(0x25aa, 0xf0f2);
    write_cmos_sensor(0x25ac, 0xfff0);
    write_cmos_sensor(0x25ae, 0x07dc);
    write_cmos_sensor(0x25b0, 0x434d);
    write_cmos_sensor(0x25b2, 0x436e);
    write_cmos_sensor(0x25b4, 0x403f);
    write_cmos_sensor(0x25b6, 0x002e);
    write_cmos_sensor(0x25b8, 0x1292);
    write_cmos_sensor(0x25ba, 0xd834);
    write_cmos_sensor(0x25bc, 0x425e);
    write_cmos_sensor(0x25be, 0x80a3);
    write_cmos_sensor(0x25c0, 0x425f);
    write_cmos_sensor(0x25c2, 0x80a4);
    write_cmos_sensor(0x25c4, 0x1292);
    write_cmos_sensor(0x25c6, 0xd838);
    write_cmos_sensor(0x25c8, 0x4f08);
    write_cmos_sensor(0x25ca, 0x930f);
    write_cmos_sensor(0x25cc, 0x2467);
    write_cmos_sensor(0x25ce, 0x434d);
    write_cmos_sensor(0x25d0, 0x426e);
    write_cmos_sensor(0x25d2, 0x480f);
    write_cmos_sensor(0x25d4, 0x1292);
    write_cmos_sensor(0x25d6, 0xd834);
    write_cmos_sensor(0x25d8, 0x5228);
    write_cmos_sensor(0x25da, 0x421b);
    write_cmos_sensor(0x25dc, 0xd838);
    write_cmos_sensor(0x25de, 0x425e);
    write_cmos_sensor(0x25e0, 0x80a5);
    write_cmos_sensor(0x25e2, 0x425f);
    write_cmos_sensor(0x25e4, 0x80a6);
    write_cmos_sensor(0x25e6, 0x128b);
    write_cmos_sensor(0x25e8, 0x4f0a);
    write_cmos_sensor(0x25ea, 0x425e);
    write_cmos_sensor(0x25ec, 0x80a3);
    write_cmos_sensor(0x25ee, 0x425f);
    write_cmos_sensor(0x25f0, 0x80a4);
    write_cmos_sensor(0x25f2, 0x128b);
    write_cmos_sensor(0x25f4, 0x930a);
    write_cmos_sensor(0x25f6, 0x2002);
    write_cmos_sensor(0x25f8, 0x930f);
    write_cmos_sensor(0x25fa, 0x2450);
    write_cmos_sensor(0x25fc, 0x93c2);
    write_cmos_sensor(0x25fe, 0x80a4);
    write_cmos_sensor(0x2600, 0x2403);
    write_cmos_sensor(0x2602, 0x4f8a);
    write_cmos_sensor(0x2604, 0x0000);
    write_cmos_sensor(0x2606, 0x3fe3);
    write_cmos_sensor(0x2608, 0x42da);
    write_cmos_sensor(0x260a, 0x80a3);
    write_cmos_sensor(0x260c, 0x0000);
    write_cmos_sensor(0x260e, 0x3fdf);
    write_cmos_sensor(0x2610, 0x9f0b);
    write_cmos_sensor(0x2612, 0x23b7);
    write_cmos_sensor(0x2614, 0x3fb7);
    write_cmos_sensor(0x2616, 0x480f);
    write_cmos_sensor(0x2618, 0x5a0f);
    write_cmos_sensor(0x261a, 0x435d);
    write_cmos_sensor(0x261c, 0x407e);
    write_cmos_sensor(0x261e, 0x0003);
    write_cmos_sensor(0x2620, 0x1292);
    write_cmos_sensor(0x2622, 0xd834);
    write_cmos_sensor(0x2624, 0x403d);
    write_cmos_sensor(0x2626, 0x80a4);
    write_cmos_sensor(0x2628, 0x4d6f);
    write_cmos_sensor(0x262a, 0x108f);
    write_cmos_sensor(0x262c, 0x425e);
    write_cmos_sensor(0x262e, 0x80a3);
    write_cmos_sensor(0x2630, 0x5e0f);
    write_cmos_sensor(0x2632, 0x4f89);
    write_cmos_sensor(0x2634, 0x5000);
    write_cmos_sensor(0x2636, 0x425f);
    write_cmos_sensor(0x2638, 0x80a5);
    write_cmos_sensor(0x263a, 0x5f0f);
    write_cmos_sensor(0x263c, 0x5f0f);
    write_cmos_sensor(0x263e, 0x5f0f);
    write_cmos_sensor(0x2640, 0x5f0f);
    write_cmos_sensor(0x2642, 0x4d6e);
    write_cmos_sensor(0x2644, 0xc312);
    write_cmos_sensor(0x2646, 0x104e);
    write_cmos_sensor(0x2648, 0x114e);
    write_cmos_sensor(0x264a, 0x114e);
    write_cmos_sensor(0x264c, 0x114e);
    write_cmos_sensor(0x264e, 0xf37e);
    write_cmos_sensor(0x2650, 0x5e0f);
    write_cmos_sensor(0x2652, 0x4f89);
    write_cmos_sensor(0x2654, 0x6000);
    write_cmos_sensor(0x2656, 0x5329);
    write_cmos_sensor(0x2658, 0x503a);
    write_cmos_sensor(0x265a, 0x0003);
    write_cmos_sensor(0x265c, 0x970a);
    write_cmos_sensor(0x265e, 0x2bdb);
    write_cmos_sensor(0x2660, 0x3f76);
    write_cmos_sensor(0x2662, 0x4309);
    write_cmos_sensor(0x2664, 0x3f71);
    write_cmos_sensor(0x2666, 0x4218);
    write_cmos_sensor(0x2668, 0x80c4);
    write_cmos_sensor(0x266a, 0x5318);
    write_cmos_sensor(0x266c, 0x3f60);
    write_cmos_sensor(0x266e, 0x903f);
    write_cmos_sensor(0x2670, 0x0037);
    write_cmos_sensor(0x2672, 0x2356);
    write_cmos_sensor(0x2674, 0x5b0b);
    write_cmos_sensor(0x2676, 0x4218);
    write_cmos_sensor(0x2678, 0x80c4);
    write_cmos_sensor(0x267a, 0x5b08);
    write_cmos_sensor(0x267c, 0x3ff6);
    write_cmos_sensor(0x267e, 0x4292);
    write_cmos_sensor(0x2680, 0x0092);
    write_cmos_sensor(0x2682, 0x80c4);
    write_cmos_sensor(0x2684, 0x4038);
    write_cmos_sensor(0x2686, 0x0fce);
    write_cmos_sensor(0x2688, 0x403b);
    write_cmos_sensor(0x268a, 0x00fd);
    write_cmos_sensor(0x268c, 0x4235);
    write_cmos_sensor(0x268e, 0x42b1);
    write_cmos_sensor(0x2690, 0x0000);
    write_cmos_sensor(0x2692, 0x4037);
    write_cmos_sensor(0x2694, 0x00fc);
    write_cmos_sensor(0x2696, 0x4034);
    write_cmos_sensor(0x2698, 0x00fc);
    write_cmos_sensor(0x269a, 0x3f38);
    write_cmos_sensor(0x269c, 0x0261);
    write_cmos_sensor(0x269e, 0x0000);
    write_cmos_sensor(0x26a0, 0x43c2);
    write_cmos_sensor(0x26a2, 0x0780);
    write_cmos_sensor(0x26a4, 0xd2e2);
    write_cmos_sensor(0x26a6, 0x0f84);
    write_cmos_sensor(0x26a8, 0x5321);
    write_cmos_sensor(0x26aa, 0x4134);
    write_cmos_sensor(0x26ac, 0x4135);
    write_cmos_sensor(0x26ae, 0x4136);
    write_cmos_sensor(0x26b0, 0x4137);
    write_cmos_sensor(0x26b2, 0x4138);
    write_cmos_sensor(0x26b4, 0x4139);
    write_cmos_sensor(0x26b6, 0x413a);
    write_cmos_sensor(0x26b8, 0x413b);
    write_cmos_sensor(0x26ba, 0x4130);
    write_cmos_sensor(0x26bc, 0x120b);
    write_cmos_sensor(0x26be, 0x120a);
    write_cmos_sensor(0x26c0, 0x1209);
    write_cmos_sensor(0x26c2, 0x1208);
    write_cmos_sensor(0x26c4, 0x1207);
    write_cmos_sensor(0x26c6, 0x1206);
    write_cmos_sensor(0x26c8, 0x1205);
    write_cmos_sensor(0x26ca, 0x1204);
    write_cmos_sensor(0x26cc, 0x8031);
    write_cmos_sensor(0x26ce, 0x000c);
    write_cmos_sensor(0x26d0, 0x4291);
    write_cmos_sensor(0x26d2, 0x0cb2);
    write_cmos_sensor(0x26d4, 0x0002);
    write_cmos_sensor(0x26d6, 0x4381);
    write_cmos_sensor(0x26d8, 0x0004);
    write_cmos_sensor(0x26da, 0xe1d1);
    write_cmos_sensor(0x26dc, 0x0002);
    write_cmos_sensor(0x26de, 0x0004);
    write_cmos_sensor(0x26e0, 0x43c1);
    write_cmos_sensor(0x26e2, 0x0005);
    write_cmos_sensor(0x26e4, 0xe191);
    write_cmos_sensor(0x26e6, 0x0002);
    write_cmos_sensor(0x26e8, 0x0004);
    write_cmos_sensor(0x26ea, 0x1091);
    write_cmos_sensor(0x26ec, 0x0004);
    write_cmos_sensor(0x26ee, 0x43c1);
    write_cmos_sensor(0x26f0, 0x0003);
    write_cmos_sensor(0x26f2, 0x1091);
    write_cmos_sensor(0x26f4, 0x0002);
    write_cmos_sensor(0x26f6, 0x5191);
    write_cmos_sensor(0x26f8, 0x0002);
    write_cmos_sensor(0x26fa, 0x0002);
    write_cmos_sensor(0x26fc, 0x6191);
    write_cmos_sensor(0x26fe, 0x0004);
    write_cmos_sensor(0x2700, 0x0004);
    write_cmos_sensor(0x2702, 0x5191);
    write_cmos_sensor(0x2704, 0x0002);
    write_cmos_sensor(0x2706, 0x0002);
    write_cmos_sensor(0x2708, 0x6191);
    write_cmos_sensor(0x270a, 0x0004);
    write_cmos_sensor(0x270c, 0x0004);
    write_cmos_sensor(0x270e, 0x4381);
    write_cmos_sensor(0x2710, 0x0000);
    write_cmos_sensor(0x2712, 0x412f);
    write_cmos_sensor(0x2714, 0x5f0f);
    write_cmos_sensor(0x2716, 0x5f0f);
    write_cmos_sensor(0x2718, 0x4f1e);
    write_cmos_sensor(0x271a, 0x0c34);
    write_cmos_sensor(0x271c, 0x4f1f);
    write_cmos_sensor(0x271e, 0x0c32);
    write_cmos_sensor(0x2720, 0x1292);
    write_cmos_sensor(0x2722, 0xd840);
    write_cmos_sensor(0x2724, 0x4e04);
    write_cmos_sensor(0x2726, 0x4f05);
    write_cmos_sensor(0x2728, 0x412c);
    write_cmos_sensor(0x272a, 0x5c0c);
    write_cmos_sensor(0x272c, 0x4c81);
    write_cmos_sensor(0x272e, 0x0006);
    write_cmos_sensor(0x2730, 0x4c16);
    write_cmos_sensor(0x2732, 0x0c2a);
    write_cmos_sensor(0x2734, 0x4307);
    write_cmos_sensor(0x2736, 0x4e0c);
    write_cmos_sensor(0x2738, 0x4f0d);
    write_cmos_sensor(0x273a, 0x460a);
    write_cmos_sensor(0x273c, 0x470b);
    write_cmos_sensor(0x273e, 0x12b0);
    write_cmos_sensor(0x2740, 0xfdd6);
    write_cmos_sensor(0x2742, 0x4c0a);
    write_cmos_sensor(0x2744, 0x4d0b);
    write_cmos_sensor(0x2746, 0x4c81);
    write_cmos_sensor(0x2748, 0x0008);
    write_cmos_sensor(0x274a, 0x4d81);
    write_cmos_sensor(0x274c, 0x000a);
    write_cmos_sensor(0x274e, 0x460c);
    write_cmos_sensor(0x2750, 0x470d);
    write_cmos_sensor(0x2752, 0x12b0);
    write_cmos_sensor(0x2754, 0xfd96);
    write_cmos_sensor(0x2756, 0x8e04);
    write_cmos_sensor(0x2758, 0x7f05);
    write_cmos_sensor(0x275a, 0x440e);
    write_cmos_sensor(0x275c, 0x450f);
    write_cmos_sensor(0x275e, 0xee4f);
    write_cmos_sensor(0x2760, 0xee0f);
    write_cmos_sensor(0x2762, 0x108f);
    write_cmos_sensor(0x2764, 0xf37e);
    write_cmos_sensor(0x2766, 0x108e);
    write_cmos_sensor(0x2768, 0x5e0e);
    write_cmos_sensor(0x276a, 0x6f0f);
    write_cmos_sensor(0x276c, 0x5e0e);
    write_cmos_sensor(0x276e, 0x6f0f);
    write_cmos_sensor(0x2770, 0x4e0c);
    write_cmos_sensor(0x2772, 0x4f0d);
    write_cmos_sensor(0x2774, 0x460a);
    write_cmos_sensor(0x2776, 0x470b);
    write_cmos_sensor(0x2778, 0x12b0);
    write_cmos_sensor(0x277a, 0xfdd6);
    write_cmos_sensor(0x277c, 0x4c04);
    write_cmos_sensor(0x277e, 0x4d05);
    write_cmos_sensor(0x2780, 0x4306);
    write_cmos_sensor(0x2782, 0x4307);
    write_cmos_sensor(0x2784, 0xb2f2);
    write_cmos_sensor(0x2786, 0x0c01);
    write_cmos_sensor(0x2788, 0x244f);
    write_cmos_sensor(0x278a, 0x411e);
    write_cmos_sensor(0x278c, 0x0006);
    write_cmos_sensor(0x278e, 0x4e1f);
    write_cmos_sensor(0x2790, 0x0c62);
    write_cmos_sensor(0x2792, 0x108f);
    write_cmos_sensor(0x2794, 0xf37f);
    write_cmos_sensor(0x2796, 0x4f0d);
    write_cmos_sensor(0x2798, 0x430e);
    write_cmos_sensor(0x279a, 0x411c);
    write_cmos_sensor(0x279c, 0x0006);
    write_cmos_sensor(0x279e, 0x4c1f);
    write_cmos_sensor(0x27a0, 0x0c62);
    write_cmos_sensor(0x27a2, 0xf37f);
    write_cmos_sensor(0x27a4, 0x4f0a);
    write_cmos_sensor(0x27a6, 0x430b);
    write_cmos_sensor(0x27a8, 0x934d);
    write_cmos_sensor(0x27aa, 0x34e4);
    write_cmos_sensor(0x27ac, 0x4d08);
    write_cmos_sensor(0x27ae, 0x4e09);
    write_cmos_sensor(0x27b0, 0xf038);
    write_cmos_sensor(0x27b2, 0x007f);
    write_cmos_sensor(0x27b4, 0xf309);
    write_cmos_sensor(0x27b6, 0xe338);
    write_cmos_sensor(0x27b8, 0xe339);
    write_cmos_sensor(0x27ba, 0x5318);
    write_cmos_sensor(0x27bc, 0x6309);
    write_cmos_sensor(0x27be, 0x934a);
    write_cmos_sensor(0x27c0, 0x34d5);
    write_cmos_sensor(0x27c2, 0xf03a);
    write_cmos_sensor(0x27c4, 0x007f);
    write_cmos_sensor(0x27c6, 0xf30b);
    write_cmos_sensor(0x27c8, 0xe33a);
    write_cmos_sensor(0x27ca, 0xe33b);
    write_cmos_sensor(0x27cc, 0x531a);
    write_cmos_sensor(0x27ce, 0x630b);
    write_cmos_sensor(0x27d0, 0x880a);
    write_cmos_sensor(0x27d2, 0x790b);
    write_cmos_sensor(0x27d4, 0x421e);
    write_cmos_sensor(0x27d6, 0x80d2);
    write_cmos_sensor(0x27d8, 0x430f);
    write_cmos_sensor(0x27da, 0x4e0c);
    write_cmos_sensor(0x27dc, 0x4f0d);
    write_cmos_sensor(0x27de, 0x12b0);
    write_cmos_sensor(0x27e0, 0xfd96);
    write_cmos_sensor(0x27e2, 0x4e06);
    write_cmos_sensor(0x27e4, 0x4f07);
    write_cmos_sensor(0x27e6, 0xe849);
    write_cmos_sensor(0x27e8, 0xe809);
    write_cmos_sensor(0x27ea, 0x1089);
    write_cmos_sensor(0x27ec, 0xf378);
    write_cmos_sensor(0x27ee, 0x1088);
    write_cmos_sensor(0x27f0, 0x5806);
    write_cmos_sensor(0x27f2, 0x6907);
    write_cmos_sensor(0x27f4, 0x5606);
    write_cmos_sensor(0x27f6, 0x6707);
    write_cmos_sensor(0x27f8, 0x5606);
    write_cmos_sensor(0x27fa, 0x6707);
    write_cmos_sensor(0x27fc, 0x5606);
    write_cmos_sensor(0x27fe, 0x6707);
    write_cmos_sensor(0x2800, 0x470e);
    write_cmos_sensor(0x2802, 0x430f);
    write_cmos_sensor(0x2804, 0xf03e);
    write_cmos_sensor(0x2806, 0x8000);
    write_cmos_sensor(0x2808, 0xde0f);
    write_cmos_sensor(0x280a, 0x930f);
    write_cmos_sensor(0x280c, 0x24a9);
    write_cmos_sensor(0x280e, 0xe336);
    write_cmos_sensor(0x2810, 0xe337);
    write_cmos_sensor(0x2812, 0x5316);
    write_cmos_sensor(0x2814, 0x6307);
    write_cmos_sensor(0x2816, 0x1086);
    write_cmos_sensor(0x2818, 0x1087);
    write_cmos_sensor(0x281a, 0xe746);
    write_cmos_sensor(0x281c, 0xe706);
    write_cmos_sensor(0x281e, 0xf377);
    write_cmos_sensor(0x2820, 0xe336);
    write_cmos_sensor(0x2822, 0xe337);
    write_cmos_sensor(0x2824, 0x5316);
    write_cmos_sensor(0x2826, 0x6307);
    write_cmos_sensor(0x2828, 0x411a);
    write_cmos_sensor(0x282a, 0x0008);
    write_cmos_sensor(0x282c, 0x411b);
    write_cmos_sensor(0x282e, 0x000a);
    write_cmos_sensor(0x2830, 0xea4b);
    write_cmos_sensor(0x2832, 0xea0b);
    write_cmos_sensor(0x2834, 0x108b);
    write_cmos_sensor(0x2836, 0xf37a);
    write_cmos_sensor(0x2838, 0x108a);
    write_cmos_sensor(0x283a, 0x5a0a);
    write_cmos_sensor(0x283c, 0x6b0b);
    write_cmos_sensor(0x283e, 0x5a0a);
    write_cmos_sensor(0x2840, 0x6b0b);
    write_cmos_sensor(0x2842, 0xf034);
    write_cmos_sensor(0x2844, 0x03ff);
    write_cmos_sensor(0x2846, 0xf305);
    write_cmos_sensor(0x2848, 0xd40a);
    write_cmos_sensor(0x284a, 0xd50b);
    write_cmos_sensor(0x284c, 0xb0f2);
    write_cmos_sensor(0x284e, 0x0040);
    write_cmos_sensor(0x2850, 0x0c01);
    write_cmos_sensor(0x2852, 0x241f);
    write_cmos_sensor(0x2854, 0x411e);
    write_cmos_sensor(0x2856, 0x0002);
    write_cmos_sensor(0x2858, 0x411f);
    write_cmos_sensor(0x285a, 0x0004);
    write_cmos_sensor(0x285c, 0x8a0e);
    write_cmos_sensor(0x285e, 0x7b0f);
    write_cmos_sensor(0x2860, 0x2c18);
    write_cmos_sensor(0x2862, 0x811a);
    write_cmos_sensor(0x2864, 0x0002);
    write_cmos_sensor(0x2866, 0x711b);
    write_cmos_sensor(0x2868, 0x0004);
    write_cmos_sensor(0x286a, 0x412e);
    write_cmos_sensor(0x286c, 0x4e5f);
    write_cmos_sensor(0x286e, 0x0794);
    write_cmos_sensor(0x2870, 0x4f4e);
    write_cmos_sensor(0x2872, 0x430f);
    write_cmos_sensor(0x2874, 0x4e0c);
    write_cmos_sensor(0x2876, 0x4f0d);
    write_cmos_sensor(0x2878, 0x12b0);
    write_cmos_sensor(0x287a, 0xfd96);
    write_cmos_sensor(0x287c, 0x4e0a);
    write_cmos_sensor(0x287e, 0x4f0b);
    write_cmos_sensor(0x2880, 0x108a);
    write_cmos_sensor(0x2882, 0x108b);
    write_cmos_sensor(0x2884, 0xeb4a);
    write_cmos_sensor(0x2886, 0xeb0a);
    write_cmos_sensor(0x2888, 0xf37b);
    write_cmos_sensor(0x288a, 0x511a);
    write_cmos_sensor(0x288c, 0x0002);
    write_cmos_sensor(0x288e, 0x611b);
    write_cmos_sensor(0x2890, 0x0004);
    write_cmos_sensor(0x2892, 0x560a);
    write_cmos_sensor(0x2894, 0x670b);
    write_cmos_sensor(0x2896, 0x4a07);
    write_cmos_sensor(0x2898, 0x4b08);
    write_cmos_sensor(0x289a, 0xf038);
    write_cmos_sensor(0x289c, 0x000f);
    write_cmos_sensor(0x289e, 0x93c2);
    write_cmos_sensor(0x28a0, 0x80a1);
    write_cmos_sensor(0x28a2, 0x202d);
    write_cmos_sensor(0x28a4, 0x93c2);
    write_cmos_sensor(0x28a6, 0x80a2);
    write_cmos_sensor(0x28a8, 0x202a);
    write_cmos_sensor(0x28aa, 0x93c2);
    write_cmos_sensor(0x28ac, 0x80a0);
    write_cmos_sensor(0x28ae, 0x2027);
    write_cmos_sensor(0x28b0, 0x4039);
    write_cmos_sensor(0x28b2, 0x0c18);
    write_cmos_sensor(0x28b4, 0x496f);
    write_cmos_sensor(0x28b6, 0x4f4e);
    write_cmos_sensor(0x28b8, 0x430f);
    write_cmos_sensor(0x28ba, 0x470a);
    write_cmos_sensor(0x28bc, 0x480b);
    write_cmos_sensor(0x28be, 0x4e0c);
    write_cmos_sensor(0x28c0, 0x4f0d);
    write_cmos_sensor(0x28c2, 0x12b0);
    write_cmos_sensor(0x28c4, 0xfd96);
    write_cmos_sensor(0x28c6, 0x4e07);
    write_cmos_sensor(0x28c8, 0x4f08);
    write_cmos_sensor(0x28ca, 0x411b);
    write_cmos_sensor(0x28cc, 0x0006);
    write_cmos_sensor(0x28ce, 0x5b0b);
    write_cmos_sensor(0x28d0, 0x496f);
    write_cmos_sensor(0x28d2, 0x4f4e);
    write_cmos_sensor(0x28d4, 0x403f);
    write_cmos_sensor(0x28d6, 0x0100);
    write_cmos_sensor(0x28d8, 0x8e0f);
    write_cmos_sensor(0x28da, 0x4f0c);
    write_cmos_sensor(0x28dc, 0x4c0d);
    write_cmos_sensor(0x28de, 0x5d0d);
    write_cmos_sensor(0x28e0, 0x7d0d);
    write_cmos_sensor(0x28e2, 0xe33d);
    write_cmos_sensor(0x28e4, 0x4b1a);
    write_cmos_sensor(0x28e6, 0x8138);
    write_cmos_sensor(0x28e8, 0x4b1b);
    write_cmos_sensor(0x28ea, 0x813a);
    write_cmos_sensor(0x28ec, 0x12b0);
    write_cmos_sensor(0x28ee, 0xfd96);
    write_cmos_sensor(0x28f0, 0x5e07);
    write_cmos_sensor(0x28f2, 0x6f08);
    write_cmos_sensor(0x28f4, 0x1087);
    write_cmos_sensor(0x28f6, 0x1088);
    write_cmos_sensor(0x28f8, 0xe847);
    write_cmos_sensor(0x28fa, 0xe807);
    write_cmos_sensor(0x28fc, 0xf378);
    write_cmos_sensor(0x28fe, 0x412d);
    write_cmos_sensor(0x2900, 0x5d0d);
    write_cmos_sensor(0x2902, 0x5d0d);
    write_cmos_sensor(0x2904, 0x478d);
    write_cmos_sensor(0x2906, 0x8138);
    write_cmos_sensor(0x2908, 0x488d);
    write_cmos_sensor(0x290a, 0x813a);
    write_cmos_sensor(0x290c, 0x470e);
    write_cmos_sensor(0x290e, 0x480f);
    write_cmos_sensor(0x2910, 0xc312);
    write_cmos_sensor(0x2912, 0x100f);
    write_cmos_sensor(0x2914, 0x100e);
    write_cmos_sensor(0x2916, 0x110f);
    write_cmos_sensor(0x2918, 0x100e);
    write_cmos_sensor(0x291a, 0x110f);
    write_cmos_sensor(0x291c, 0x100e);
    write_cmos_sensor(0x291e, 0x110f);
    write_cmos_sensor(0x2920, 0x100e);
    write_cmos_sensor(0x2922, 0x412c);
    write_cmos_sensor(0x2924, 0x5c0c);
    write_cmos_sensor(0x2926, 0xb3d2);
    write_cmos_sensor(0x2928, 0x0c01);
    write_cmos_sensor(0x292a, 0x240d);
    write_cmos_sensor(0x292c, 0xb3e2);
    write_cmos_sensor(0x292e, 0x0c04);
    write_cmos_sensor(0x2930, 0x240a);
    write_cmos_sensor(0x2932, 0x421f);
    write_cmos_sensor(0x2934, 0x0c08);
    write_cmos_sensor(0x2936, 0x5d0f);
    write_cmos_sensor(0x2938, 0x5f0f);
    write_cmos_sensor(0x293a, 0x5f0f);
    write_cmos_sensor(0x293c, 0x5f0f);
    write_cmos_sensor(0x293e, 0x5f0f);
    write_cmos_sensor(0x2940, 0x5f0f);
    write_cmos_sensor(0x2942, 0x5f0f);
    write_cmos_sensor(0x2944, 0x8f0e);
    write_cmos_sensor(0x2946, 0x4e8c);
    write_cmos_sensor(0x2948, 0x0c1a);
    write_cmos_sensor(0x294a, 0xb3d2);
    write_cmos_sensor(0x294c, 0x0c00);
    write_cmos_sensor(0x294e, 0x2406);
    write_cmos_sensor(0x2950, 0x470a);
    write_cmos_sensor(0x2952, 0xf03a);
    write_cmos_sensor(0x2954, 0x03ff);
    write_cmos_sensor(0x2956, 0x5a0a);
    write_cmos_sensor(0x2958, 0x5a0a);
    write_cmos_sensor(0x295a, 0x3c12);
    write_cmos_sensor(0x295c, 0x430a);
    write_cmos_sensor(0x295e, 0x3c10);
    write_cmos_sensor(0x2960, 0x1086);
    write_cmos_sensor(0x2962, 0x1087);
    write_cmos_sensor(0x2964, 0xe746);
    write_cmos_sensor(0x2966, 0xe706);
    write_cmos_sensor(0x2968, 0xf377);
    write_cmos_sensor(0x296a, 0x3f5e);
    write_cmos_sensor(0x296c, 0xf03a);
    write_cmos_sensor(0x296e, 0x007f);
    write_cmos_sensor(0x2970, 0xf30b);
    write_cmos_sensor(0x2972, 0x3f2e);
    write_cmos_sensor(0x2974, 0x4d08);
    write_cmos_sensor(0x2976, 0x4e09);
    write_cmos_sensor(0x2978, 0xf038);
    write_cmos_sensor(0x297a, 0x007f);
    write_cmos_sensor(0x297c, 0xf309);
    write_cmos_sensor(0x297e, 0x3f1f);
    write_cmos_sensor(0x2980, 0x4a8c);
    write_cmos_sensor(0x2982, 0x0c56);
    write_cmos_sensor(0x2984, 0x5391);
    write_cmos_sensor(0x2986, 0x0000);
    write_cmos_sensor(0x2988, 0x92a1);
    write_cmos_sensor(0x298a, 0x0000);
    write_cmos_sensor(0x298c, 0x2ac2);
    write_cmos_sensor(0x298e, 0x5031);
    write_cmos_sensor(0x2990, 0x000c);
    write_cmos_sensor(0x2992, 0x4134);
    write_cmos_sensor(0x2994, 0x4135);
    write_cmos_sensor(0x2996, 0x4136);
    write_cmos_sensor(0x2998, 0x4137);
    write_cmos_sensor(0x299a, 0x4138);
    write_cmos_sensor(0x299c, 0x4139);
    write_cmos_sensor(0x299e, 0x413a);
    write_cmos_sensor(0x29a0, 0x413b);
    write_cmos_sensor(0x29a2, 0x4130);
    write_cmos_sensor(0x29a4, 0x120b);
    write_cmos_sensor(0x29a6, 0x4292);
    write_cmos_sensor(0x29a8, 0x8128);
    write_cmos_sensor(0x29aa, 0x812c);
    write_cmos_sensor(0x29ac, 0x4292);
    write_cmos_sensor(0x29ae, 0x812a);
    write_cmos_sensor(0x29b0, 0x812e);
    write_cmos_sensor(0x29b2, 0x425f);
    write_cmos_sensor(0x29b4, 0x0084);
    write_cmos_sensor(0x29b6, 0xf37f);
    write_cmos_sensor(0x29b8, 0x421e);
    write_cmos_sensor(0x29ba, 0x0086);
    write_cmos_sensor(0x29bc, 0x1292);
    write_cmos_sensor(0x29be, 0xd840);
    write_cmos_sensor(0x29c0, 0x4e82);
    write_cmos_sensor(0x29c2, 0x8128);
    write_cmos_sensor(0x29c4, 0x4f82);
    write_cmos_sensor(0x29c6, 0x812a);
    write_cmos_sensor(0x29c8, 0x93c2);
    write_cmos_sensor(0x29ca, 0x80a0);
    write_cmos_sensor(0x29cc, 0x2404);
    write_cmos_sensor(0x29ce, 0x4e82);
    write_cmos_sensor(0x29d0, 0x812c);
    write_cmos_sensor(0x29d2, 0x4f82);
    write_cmos_sensor(0x29d4, 0x812e);
    write_cmos_sensor(0x29d6, 0x4292);
    write_cmos_sensor(0x29d8, 0x0088);
    write_cmos_sensor(0x29da, 0x731a);
    write_cmos_sensor(0x29dc, 0x4382);
    write_cmos_sensor(0x29de, 0x7544);
    write_cmos_sensor(0x29e0, 0x4292);
    write_cmos_sensor(0x29e2, 0x80b2);
    write_cmos_sensor(0x29e4, 0x7546);
    write_cmos_sensor(0x29e6, 0x93c2);
    write_cmos_sensor(0x29e8, 0x809f);
    write_cmos_sensor(0x29ea, 0x2443);
    write_cmos_sensor(0x29ec, 0x403f);
    write_cmos_sensor(0x29ee, 0x80ae);
    write_cmos_sensor(0x29f0, 0x4fa2);
    write_cmos_sensor(0x29f2, 0x7550);
    write_cmos_sensor(0x29f4, 0x421e);
    write_cmos_sensor(0x29f6, 0x80b2);
    write_cmos_sensor(0x29f8, 0x5f2e);
    write_cmos_sensor(0x29fa, 0x4e82);
    write_cmos_sensor(0x29fc, 0x7552);
    write_cmos_sensor(0x29fe, 0x425f);
    write_cmos_sensor(0x2a00, 0x00f3);
    write_cmos_sensor(0x2a02, 0xf37f);
    write_cmos_sensor(0x2a04, 0x421b);
    write_cmos_sensor(0x2a06, 0xd82a);
    write_cmos_sensor(0x2a08, 0x403d);
    write_cmos_sensor(0x2a0a, 0x8130);
    write_cmos_sensor(0x2a0c, 0x421e);
    write_cmos_sensor(0x2a0e, 0x00f4);
    write_cmos_sensor(0x2a10, 0x128b);
    write_cmos_sensor(0x2a12, 0x93c2);
    write_cmos_sensor(0x2a14, 0x809f);
    write_cmos_sensor(0x2a16, 0x2024);
    write_cmos_sensor(0x2a18, 0x434d);
    write_cmos_sensor(0x2a1a, 0x403e);
    write_cmos_sensor(0x2a1c, 0x809c);
    write_cmos_sensor(0x2a1e, 0x403f);
    write_cmos_sensor(0x2a20, 0x8130);
    write_cmos_sensor(0x2a22, 0x12b0);
    write_cmos_sensor(0x2a24, 0xfa82);
    write_cmos_sensor(0x2a26, 0x93c2);
    write_cmos_sensor(0x2a28, 0x809f);
    write_cmos_sensor(0x2a2a, 0x2012);
    write_cmos_sensor(0x2a2c, 0x93c2);
    write_cmos_sensor(0x2a2e, 0x809f);
    write_cmos_sensor(0x2a30, 0x240c);
    write_cmos_sensor(0x2a32, 0x421e);
    write_cmos_sensor(0x2a34, 0x8130);
    write_cmos_sensor(0x2a36, 0x421f);
    write_cmos_sensor(0x2a38, 0x8132);
    write_cmos_sensor(0x2a3a, 0x821e);
    write_cmos_sensor(0x2a3c, 0x8134);
    write_cmos_sensor(0x2a3e, 0x721f);
    write_cmos_sensor(0x2a40, 0x8136);
    write_cmos_sensor(0x2a42, 0x2c1d);
    write_cmos_sensor(0x2a44, 0x4392);
    write_cmos_sensor(0x2a46, 0x733e);
    write_cmos_sensor(0x2a48, 0x3c1a);
    write_cmos_sensor(0x2a4a, 0x4382);
    write_cmos_sensor(0x2a4c, 0x733e);
    write_cmos_sensor(0x2a4e, 0x3c17);
    write_cmos_sensor(0x2a50, 0x435d);
    write_cmos_sensor(0x2a52, 0x403e);
    write_cmos_sensor(0x2a54, 0x809d);
    write_cmos_sensor(0x2a56, 0x403f);
    write_cmos_sensor(0x2a58, 0x8134);
    write_cmos_sensor(0x2a5a, 0x12b0);
    write_cmos_sensor(0x2a5c, 0xfa82);
    write_cmos_sensor(0x2a5e, 0x3fe6);
    write_cmos_sensor(0x2a60, 0x425f);
    write_cmos_sensor(0x2a62, 0x00f2);
    write_cmos_sensor(0x2a64, 0xf37f);
    write_cmos_sensor(0x2a66, 0x403d);
    write_cmos_sensor(0x2a68, 0x8134);
    write_cmos_sensor(0x2a6a, 0x421e);
    write_cmos_sensor(0x2a6c, 0x00f0);
    write_cmos_sensor(0x2a6e, 0x128b);
    write_cmos_sensor(0x2a70, 0x3fd3);
    write_cmos_sensor(0x2a72, 0x4382);
    write_cmos_sensor(0x2a74, 0x7550);
    write_cmos_sensor(0x2a76, 0x4292);
    write_cmos_sensor(0x2a78, 0x80b2);
    write_cmos_sensor(0x2a7a, 0x7552);
    write_cmos_sensor(0x2a7c, 0x3fc0);
    write_cmos_sensor(0x2a7e, 0x413b);
    write_cmos_sensor(0x2a80, 0x4130);
    write_cmos_sensor(0x2a82, 0x120b);
    write_cmos_sensor(0x2a84, 0x120a);
    write_cmos_sensor(0x2a86, 0x1209);
    write_cmos_sensor(0x2a88, 0x1208);
    write_cmos_sensor(0x2a8a, 0x1207);
    write_cmos_sensor(0x2a8c, 0x1206);
    write_cmos_sensor(0x2a8e, 0x1205);
    write_cmos_sensor(0x2a90, 0x4e06);
    write_cmos_sensor(0x2a92, 0x4d45);
    write_cmos_sensor(0x2a94, 0x421d);
    write_cmos_sensor(0x2a96, 0x80aa);
    write_cmos_sensor(0x2a98, 0x430e);
    write_cmos_sensor(0x2a9a, 0x4217);
    write_cmos_sensor(0x2a9c, 0x8128);
    write_cmos_sensor(0x2a9e, 0x4218);
    write_cmos_sensor(0x2aa0, 0x812a);
    write_cmos_sensor(0x2aa2, 0x470b);
    write_cmos_sensor(0x2aa4, 0x480c);
    write_cmos_sensor(0x2aa6, 0x8d0b);
    write_cmos_sensor(0x2aa8, 0x7e0c);
    write_cmos_sensor(0x2aaa, 0x4b0d);
    write_cmos_sensor(0x2aac, 0x4c0e);
    write_cmos_sensor(0x2aae, 0x4f39);
    write_cmos_sensor(0x2ab0, 0x4f3a);
    write_cmos_sensor(0x2ab2, 0x490b);
    write_cmos_sensor(0x2ab4, 0x4a0c);
    write_cmos_sensor(0x2ab6, 0x8d0b);
    write_cmos_sensor(0x2ab8, 0x7e0c);
    write_cmos_sensor(0x2aba, 0x2869);
    write_cmos_sensor(0x2abc, 0x425f);
    write_cmos_sensor(0x2abe, 0x00e9);
    write_cmos_sensor(0x2ac0, 0xf37f);
    write_cmos_sensor(0x2ac2, 0x521f);
    write_cmos_sensor(0x2ac4, 0x80aa);
    write_cmos_sensor(0x2ac6, 0x531f);
    write_cmos_sensor(0x2ac8, 0x4f0a);
    write_cmos_sensor(0x2aca, 0x430b);
    write_cmos_sensor(0x2acc, 0x4349);
    write_cmos_sensor(0x2ace, 0x421e);
    write_cmos_sensor(0x2ad0, 0x7560);
    write_cmos_sensor(0x2ad2, 0x421f);
    write_cmos_sensor(0x2ad4, 0x7562);
    write_cmos_sensor(0x2ad6, 0x1292);
    write_cmos_sensor(0x2ad8, 0xd840);
    write_cmos_sensor(0x2ada, 0x4a0c);
    write_cmos_sensor(0x2adc, 0x4b0d);
    write_cmos_sensor(0x2ade, 0x8e0c);
    write_cmos_sensor(0x2ae0, 0x7f0d);
    write_cmos_sensor(0x2ae2, 0x2c01);
    write_cmos_sensor(0x2ae4, 0x4359);
    write_cmos_sensor(0x2ae6, 0x434f);
    write_cmos_sensor(0x2ae8, 0x93c2);
    write_cmos_sensor(0x2aea, 0x80a0);
    write_cmos_sensor(0x2aec, 0x2001);
    write_cmos_sensor(0x2aee, 0x435f);
    write_cmos_sensor(0x2af0, 0x494e);
    write_cmos_sensor(0x2af2, 0xff4e);
    write_cmos_sensor(0x2af4, 0x434f);
    write_cmos_sensor(0x2af6, 0x93c6);
    write_cmos_sensor(0x2af8, 0x0000);
    write_cmos_sensor(0x2afa, 0x2001);
    write_cmos_sensor(0x2afc, 0x435f);
    write_cmos_sensor(0x2afe, 0xff4e);
    write_cmos_sensor(0x2b00, 0x4ec6);
    write_cmos_sensor(0x2b02, 0x0000);
    write_cmos_sensor(0x2b04, 0x9345);
    write_cmos_sensor(0x2b06, 0x2036);
    write_cmos_sensor(0x2b08, 0x93c2);
    write_cmos_sensor(0x2b0a, 0x809f);
    write_cmos_sensor(0x2b0c, 0x240d);
    write_cmos_sensor(0x2b0e, 0x4a82);
    write_cmos_sensor(0x2b10, 0x7540);
    write_cmos_sensor(0x2b12, 0x4b82);
    write_cmos_sensor(0x2b14, 0x7542);
    write_cmos_sensor(0x2b16, 0x934e);
    write_cmos_sensor(0x2b18, 0x2447);
    write_cmos_sensor(0x2b1a, 0x421f);
    write_cmos_sensor(0x2b1c, 0x80ac);
    write_cmos_sensor(0x2b1e, 0x5f82);
    write_cmos_sensor(0x2b20, 0x7544);
    write_cmos_sensor(0x2b22, 0x5f82);
    write_cmos_sensor(0x2b24, 0x7546);
    write_cmos_sensor(0x2b26, 0x3c40);
    write_cmos_sensor(0x2b28, 0xb0f2);
    write_cmos_sensor(0x2b2a, 0x0020);
    write_cmos_sensor(0x2b2c, 0x00d0);
    write_cmos_sensor(0x2b2e, 0x434f);
    write_cmos_sensor(0x2b30, 0x634f);
    write_cmos_sensor(0x2b32, 0x934f);
    write_cmos_sensor(0x2b34, 0x2003);
    write_cmos_sensor(0x2b36, 0x4e4f);
    write_cmos_sensor(0x2b38, 0xf31f);
    write_cmos_sensor(0x2b3a, 0x200a);
    write_cmos_sensor(0x2b3c, 0xb0f2);
    write_cmos_sensor(0x2b3e, 0x0020);
    write_cmos_sensor(0x2b40, 0x00d0);
    write_cmos_sensor(0x2b42, 0x434f);
    write_cmos_sensor(0x2b44, 0x634f);
    write_cmos_sensor(0x2b46, 0xf37f);
    write_cmos_sensor(0x2b48, 0xf37e);
    write_cmos_sensor(0x2b4a, 0xce0f);
    write_cmos_sensor(0x2b4c, 0x930f);
    write_cmos_sensor(0x2b4e, 0x2409);
    write_cmos_sensor(0x2b50, 0x43b2);
    write_cmos_sensor(0x2b52, 0x7540);
    write_cmos_sensor(0x2b54, 0x43b2);
    write_cmos_sensor(0x2b56, 0x7542);
    write_cmos_sensor(0x2b58, 0x4a82);
    write_cmos_sensor(0x2b5a, 0x754c);
    write_cmos_sensor(0x2b5c, 0x4b82);
    write_cmos_sensor(0x2b5e, 0x754e);
    write_cmos_sensor(0x2b60, 0x3c23);
    write_cmos_sensor(0x2b62, 0x4a82);
    write_cmos_sensor(0x2b64, 0x7540);
    write_cmos_sensor(0x2b66, 0x4b82);
    write_cmos_sensor(0x2b68, 0x7542);
    write_cmos_sensor(0x2b6a, 0x43b2);
    write_cmos_sensor(0x2b6c, 0x754c);
    write_cmos_sensor(0x2b6e, 0x43b2);
    write_cmos_sensor(0x2b70, 0x754e);
    write_cmos_sensor(0x2b72, 0x3c1a);
    write_cmos_sensor(0x2b74, 0x4a82);
    write_cmos_sensor(0x2b76, 0x754c);
    write_cmos_sensor(0x2b78, 0x4b82);
    write_cmos_sensor(0x2b7a, 0x754e);
    write_cmos_sensor(0x2b7c, 0x934e);
    write_cmos_sensor(0x2b7e, 0x2414);
    write_cmos_sensor(0x2b80, 0x421f);
    write_cmos_sensor(0x2b82, 0x80ac);
    write_cmos_sensor(0x2b84, 0x5f82);
    write_cmos_sensor(0x2b86, 0x7550);
    write_cmos_sensor(0x2b88, 0x5f82);
    write_cmos_sensor(0x2b8a, 0x7552);
    write_cmos_sensor(0x2b8c, 0x3c0d);
    write_cmos_sensor(0x2b8e, 0x8907);
    write_cmos_sensor(0x2b90, 0x7a08);
    write_cmos_sensor(0x2b92, 0x470a);
    write_cmos_sensor(0x2b94, 0x480b);
    write_cmos_sensor(0x2b96, 0x425f);
    write_cmos_sensor(0x2b98, 0x00e9);
    write_cmos_sensor(0x2b9a, 0x4f4e);
    write_cmos_sensor(0x2b9c, 0x430f);
    write_cmos_sensor(0x2b9e, 0x5e0a);
    write_cmos_sensor(0x2ba0, 0x6f0b);
    write_cmos_sensor(0x2ba2, 0x531a);
    write_cmos_sensor(0x2ba4, 0x630b);
    write_cmos_sensor(0x2ba6, 0x3f92);
    write_cmos_sensor(0x2ba8, 0x4135);
    write_cmos_sensor(0x2baa, 0x4136);
    write_cmos_sensor(0x2bac, 0x4137);
    write_cmos_sensor(0x2bae, 0x4138);
    write_cmos_sensor(0x2bb0, 0x4139);
    write_cmos_sensor(0x2bb2, 0x413a);
    write_cmos_sensor(0x2bb4, 0x413b);
    write_cmos_sensor(0x2bb6, 0x4130);
    write_cmos_sensor(0x2bb8, 0xb0f2);
    write_cmos_sensor(0x2bba, 0x0010);
    write_cmos_sensor(0x2bbc, 0x00d0);
    write_cmos_sensor(0x2bbe, 0x201e);
    write_cmos_sensor(0x2bc0, 0x40b2);
    write_cmos_sensor(0x2bc2, 0x003e);
    write_cmos_sensor(0x2bc4, 0x00b6);
    write_cmos_sensor(0x2bc6, 0x40b2);
    write_cmos_sensor(0x2bc8, 0x3e00);
    write_cmos_sensor(0x2bca, 0x00b8);
    write_cmos_sensor(0x2bcc, 0x40b2);
    write_cmos_sensor(0x2bce, 0x5050);
    write_cmos_sensor(0x2bd0, 0x00ce);
    write_cmos_sensor(0x2bd2, 0x40f2);
    write_cmos_sensor(0x2bd4, 0x0015);
    write_cmos_sensor(0x2bd6, 0x07da);
    write_cmos_sensor(0x2bd8, 0x430e);
    write_cmos_sensor(0x2bda, 0x923e);
    write_cmos_sensor(0x2bdc, 0x2425);
    write_cmos_sensor(0x2bde, 0x4e0f);
    write_cmos_sensor(0x2be0, 0x5f0f);
    write_cmos_sensor(0x2be2, 0x4f9f);
    write_cmos_sensor(0x2be4, 0xf476);
    write_cmos_sensor(0x2be6, 0x8080);
    write_cmos_sensor(0x2be8, 0x531e);
    write_cmos_sensor(0x2bea, 0x903e);
    write_cmos_sensor(0x2bec, 0x000d);
    write_cmos_sensor(0x2bee, 0x2bf5);
    write_cmos_sensor(0x2bf0, 0x40b2);
    write_cmos_sensor(0x2bf2, 0x0101);
    write_cmos_sensor(0x2bf4, 0x07a0);
    write_cmos_sensor(0x2bf6, 0x40f2);
    write_cmos_sensor(0x2bf8, 0x0025);
    write_cmos_sensor(0x2bfa, 0x00ea);
    write_cmos_sensor(0x2bfc, 0x90f2);
    write_cmos_sensor(0x2bfe, 0x0003);
    write_cmos_sensor(0x2c00, 0x07a3);
    write_cmos_sensor(0x2c02, 0x240f);
    write_cmos_sensor(0x2c04, 0x93e2);
    write_cmos_sensor(0x2c06, 0x07a3);
    write_cmos_sensor(0x2c08, 0x2408);
    write_cmos_sensor(0x2c0a, 0xb3d2);
    write_cmos_sensor(0x2c0c, 0x00d0);
    write_cmos_sensor(0x2c0e, 0x2415);
    write_cmos_sensor(0x2c10, 0x403f);
    write_cmos_sensor(0x2c12, 0x8080);
    write_cmos_sensor(0x2c14, 0x1292);
    write_cmos_sensor(0x2c16, 0xd832);
    write_cmos_sensor(0x2c18, 0x3c10);
    write_cmos_sensor(0x2c1a, 0x40b2);
    write_cmos_sensor(0x2c1c, 0x000c);
    write_cmos_sensor(0x2c1e, 0x8092);
    write_cmos_sensor(0x2c20, 0x3ff4);
    write_cmos_sensor(0x2c22, 0x42b2);
    write_cmos_sensor(0x2c24, 0x8092);
    write_cmos_sensor(0x2c26, 0x3ff1);
    write_cmos_sensor(0x2c28, 0x421f);
    write_cmos_sensor(0x2c2a, 0x8090);
    write_cmos_sensor(0x2c2c, 0xf03f);
    write_cmos_sensor(0x2c2e, 0x0770);
    write_cmos_sensor(0x2c30, 0xd21f);
    write_cmos_sensor(0x2c32, 0xf486);
    write_cmos_sensor(0x2c34, 0x4f82);
    write_cmos_sensor(0x2c36, 0x8090);
    write_cmos_sensor(0x2c38, 0x3fd7);
    write_cmos_sensor(0x2c3a, 0x43d2);
    write_cmos_sensor(0x2c3c, 0x0f82);
    write_cmos_sensor(0x2c3e, 0x4392);
    write_cmos_sensor(0x2c40, 0x731c);
    write_cmos_sensor(0x2c42, 0x0b00);
    write_cmos_sensor(0x2c44, 0x7304);
    write_cmos_sensor(0x2c46, 0x1200);
    write_cmos_sensor(0x2c48, 0x43e2);
    write_cmos_sensor(0x2c4a, 0x0180);
    write_cmos_sensor(0x2c4c, 0x4130);
    write_cmos_sensor(0x2c4e, 0x120b);
    write_cmos_sensor(0x2c50, 0x120a);
    write_cmos_sensor(0x2c52, 0x403f);
    write_cmos_sensor(0x2c54, 0x0290);
    write_cmos_sensor(0x2c56, 0xc2e2);
    write_cmos_sensor(0x2c58, 0x0f84);
    write_cmos_sensor(0x2c5a, 0x430d);
    write_cmos_sensor(0x2c5c, 0x903d);
    write_cmos_sensor(0x2c5e, 0x0007);
    write_cmos_sensor(0x2c60, 0x243c);
    write_cmos_sensor(0x2c62, 0x903d);
    write_cmos_sensor(0x2c64, 0x000e);
    write_cmos_sensor(0x2c66, 0x2436);
    write_cmos_sensor(0x2c68, 0x903d);
    write_cmos_sensor(0x2c6a, 0x0015);
    write_cmos_sensor(0x2c6c, 0x2430);
    write_cmos_sensor(0x2c6e, 0x4f9f);
    write_cmos_sensor(0x2c70, 0x6000);
    write_cmos_sensor(0x2c72, 0x600e);
    write_cmos_sensor(0x2c74, 0x4f9f);
    write_cmos_sensor(0x2c76, 0x5000);
    write_cmos_sensor(0x2c78, 0x500e);
    write_cmos_sensor(0x2c7a, 0x532f);
    write_cmos_sensor(0x2c7c, 0x531d);
    write_cmos_sensor(0x2c7e, 0x903d);
    write_cmos_sensor(0x2c80, 0x001c);
    write_cmos_sensor(0x2c82, 0x2bec);
    write_cmos_sensor(0x2c84, 0x430c);
    write_cmos_sensor(0x2c86, 0x430b);
    write_cmos_sensor(0x2c88, 0x930c);
    write_cmos_sensor(0x2c8a, 0x201e);
    write_cmos_sensor(0x2c8c, 0x403a);
    write_cmos_sensor(0x2c8e, 0x0282);
    write_cmos_sensor(0x2c90, 0x430d);
    write_cmos_sensor(0x2c92, 0x4d0f);
    write_cmos_sensor(0x2c94, 0x5f0f);
    write_cmos_sensor(0x2c96, 0x4a0e);
    write_cmos_sensor(0x2c98, 0x5f0e);
    write_cmos_sensor(0x2c9a, 0x4b0f);
    write_cmos_sensor(0x2c9c, 0x5f0f);
    write_cmos_sensor(0x2c9e, 0x5c0f);
    write_cmos_sensor(0x2ca0, 0x5d0f);
    write_cmos_sensor(0x2ca2, 0x5f0f);
    write_cmos_sensor(0x2ca4, 0x503f);
    write_cmos_sensor(0x2ca6, 0x80aa);
    write_cmos_sensor(0x2ca8, 0x4f9e);
    write_cmos_sensor(0x2caa, 0x0062);
    write_cmos_sensor(0x2cac, 0x6000);
    write_cmos_sensor(0x2cae, 0x4f9e);
    write_cmos_sensor(0x2cb0, 0x0046);
    write_cmos_sensor(0x2cb2, 0x5000);
    write_cmos_sensor(0x2cb4, 0x531d);
    write_cmos_sensor(0x2cb6, 0x903d);
    write_cmos_sensor(0x2cb8, 0x0007);
    write_cmos_sensor(0x2cba, 0x2beb);
    write_cmos_sensor(0x2cbc, 0x503b);
    write_cmos_sensor(0x2cbe, 0x0003);
    write_cmos_sensor(0x2cc0, 0x531c);
    write_cmos_sensor(0x2cc2, 0x932c);
    write_cmos_sensor(0x2cc4, 0x2be1);
    write_cmos_sensor(0x2cc6, 0x3c0c);
    write_cmos_sensor(0x2cc8, 0x403a);
    write_cmos_sensor(0x2cca, 0x022e);
    write_cmos_sensor(0x2ccc, 0x3fe1);
    write_cmos_sensor(0x2cce, 0x403f);
    write_cmos_sensor(0x2cd0, 0x022e);
    write_cmos_sensor(0x2cd2, 0x3fcd);
    write_cmos_sensor(0x2cd4, 0x403f);
    write_cmos_sensor(0x2cd6, 0x023c);
    write_cmos_sensor(0x2cd8, 0x3fca);
    write_cmos_sensor(0x2cda, 0x403f);
    write_cmos_sensor(0x2cdc, 0x0282);
    write_cmos_sensor(0x2cde, 0x3fc7);
    write_cmos_sensor(0x2ce0, 0xd2e2);
    write_cmos_sensor(0x2ce2, 0x0f84);
    write_cmos_sensor(0x2ce4, 0x413a);
    write_cmos_sensor(0x2ce6, 0x413b);
    write_cmos_sensor(0x2ce8, 0x4130);
    write_cmos_sensor(0x2cea, 0x120b);
    write_cmos_sensor(0x2cec, 0x120a);
    write_cmos_sensor(0x2cee, 0x93c2);
    write_cmos_sensor(0x2cf0, 0x00e9);
    write_cmos_sensor(0x2cf2, 0x243e);
    write_cmos_sensor(0x2cf4, 0x0b00);
    write_cmos_sensor(0x2cf6, 0x7306);
    write_cmos_sensor(0x2cf8, 0x0001);
    write_cmos_sensor(0x2cfa, 0xb3e2);
    write_cmos_sensor(0x2cfc, 0x00c3);
    write_cmos_sensor(0x2cfe, 0x2430);
    write_cmos_sensor(0x2d00, 0x421b);
    write_cmos_sensor(0x2d02, 0x733a);
    write_cmos_sensor(0x2d04, 0x430a);
    write_cmos_sensor(0x2d06, 0x421c);
    write_cmos_sensor(0x2d08, 0x733c);
    write_cmos_sensor(0x2d0a, 0x430d);
    write_cmos_sensor(0x2d0c, 0x5a0c);
    write_cmos_sensor(0x2d0e, 0x6b0d);
    write_cmos_sensor(0x2d10, 0x531c);
    write_cmos_sensor(0x2d12, 0x630d);
    write_cmos_sensor(0x2d14, 0x425f);
    write_cmos_sensor(0x2d16, 0x00e9);
    write_cmos_sensor(0x2d18, 0x4f4e);
    write_cmos_sensor(0x2d1a, 0x430f);
    write_cmos_sensor(0x2d1c, 0x521e);
    write_cmos_sensor(0x2d1e, 0x812c);
    write_cmos_sensor(0x2d20, 0x621f);
    write_cmos_sensor(0x2d22, 0x812e);
    write_cmos_sensor(0x2d24, 0x4e0b);
    write_cmos_sensor(0x2d26, 0x8c0b);
    write_cmos_sensor(0x2d28, 0x4b82);
    write_cmos_sensor(0x2d2a, 0x80e6);
    write_cmos_sensor(0x2d2c, 0x8c0e);
    write_cmos_sensor(0x2d2e, 0x7d0f);
    write_cmos_sensor(0x2d30, 0x2c0b);
    write_cmos_sensor(0x2d32, 0x4382);
    write_cmos_sensor(0x2d34, 0x80e6);
    write_cmos_sensor(0x2d36, 0xd0f2);
    write_cmos_sensor(0x2d38, 0x0010);
    write_cmos_sensor(0x2d3a, 0x00bf);
    write_cmos_sensor(0x2d3c, 0x4292);
    write_cmos_sensor(0x2d3e, 0x80e6);
    write_cmos_sensor(0x2d40, 0x7334);
    write_cmos_sensor(0x2d42, 0x0f00);
    write_cmos_sensor(0x2d44, 0x7300);
    write_cmos_sensor(0x2d46, 0x3c16);
    write_cmos_sensor(0x2d48, 0x421f);
    write_cmos_sensor(0x2d4a, 0x80aa);
    write_cmos_sensor(0x2d4c, 0x533f);
    write_cmos_sensor(0x2d4e, 0x921f);
    write_cmos_sensor(0x2d50, 0x80e6);
    write_cmos_sensor(0x2d52, 0x2ff4);
    write_cmos_sensor(0x2d54, 0x4f82);
    write_cmos_sensor(0x2d56, 0x80e6);
    write_cmos_sensor(0x2d58, 0xd0f2);
    write_cmos_sensor(0x2d5a, 0x0020);
    write_cmos_sensor(0x2d5c, 0x00bf);
    write_cmos_sensor(0x2d5e, 0x3fee);
    write_cmos_sensor(0x2d60, 0x425f);
    write_cmos_sensor(0x2d62, 0x00e9);
    write_cmos_sensor(0x2d64, 0xf37f);
    write_cmos_sensor(0x2d66, 0x4f82);
    write_cmos_sensor(0x2d68, 0x7334);
    write_cmos_sensor(0x2d6a, 0x0f00);
    write_cmos_sensor(0x2d6c, 0x7300);
    write_cmos_sensor(0x2d6e, 0x3c02);
    write_cmos_sensor(0x2d70, 0x0900);
    write_cmos_sensor(0x2d72, 0x730e);
    write_cmos_sensor(0x2d74, 0x413a);
    write_cmos_sensor(0x2d76, 0x413b);
    write_cmos_sensor(0x2d78, 0x4130);
    write_cmos_sensor(0x2d7a, 0xffff);
    write_cmos_sensor(0x2d7c, 0xdf02);
    write_cmos_sensor(0x2d7e, 0x3ffe);
    write_cmos_sensor(0x2d80, 0x430e);
    write_cmos_sensor(0x2d82, 0x930a);
    write_cmos_sensor(0x2d84, 0x2407);
    write_cmos_sensor(0x2d86, 0xc312);
    write_cmos_sensor(0x2d88, 0x100c);
    write_cmos_sensor(0x2d8a, 0x2801);
    write_cmos_sensor(0x2d8c, 0x5a0e);
    write_cmos_sensor(0x2d8e, 0x5a0a);
    write_cmos_sensor(0x2d90, 0x930c);
    write_cmos_sensor(0x2d92, 0x23f7);
    write_cmos_sensor(0x2d94, 0x4130);
    write_cmos_sensor(0x2d96, 0x430e);
    write_cmos_sensor(0x2d98, 0x430f);
    write_cmos_sensor(0x2d9a, 0x3c08);
    write_cmos_sensor(0x2d9c, 0xc312);
    write_cmos_sensor(0x2d9e, 0x100d);
    write_cmos_sensor(0x2da0, 0x100c);
    write_cmos_sensor(0x2da2, 0x2802);
    write_cmos_sensor(0x2da4, 0x5a0e);
    write_cmos_sensor(0x2da6, 0x6b0f);
    write_cmos_sensor(0x2da8, 0x5a0a);
    write_cmos_sensor(0x2daa, 0x6b0b);
    write_cmos_sensor(0x2dac, 0x930c);
    write_cmos_sensor(0x2dae, 0x23f6);
    write_cmos_sensor(0x2db0, 0x930d);
    write_cmos_sensor(0x2db2, 0x23f4);
    write_cmos_sensor(0x2db4, 0x4130);
    write_cmos_sensor(0x2db6, 0x4030);
    write_cmos_sensor(0x2db8, 0xfd96);
    write_cmos_sensor(0x2dba, 0xee0e);
    write_cmos_sensor(0x2dbc, 0x403b);
    write_cmos_sensor(0x2dbe, 0x0011);
    write_cmos_sensor(0x2dc0, 0x3c05);
    write_cmos_sensor(0x2dc2, 0x100d);
    write_cmos_sensor(0x2dc4, 0x6e0e);
    write_cmos_sensor(0x2dc6, 0x9a0e);
    write_cmos_sensor(0x2dc8, 0x2801);
    write_cmos_sensor(0x2dca, 0x8a0e);
    write_cmos_sensor(0x2dcc, 0x6c0c);
    write_cmos_sensor(0x2dce, 0x6d0d);
    write_cmos_sensor(0x2dd0, 0x831b);
    write_cmos_sensor(0x2dd2, 0x23f7);
    write_cmos_sensor(0x2dd4, 0x4130);
    write_cmos_sensor(0x2dd6, 0xef0f);
    write_cmos_sensor(0x2dd8, 0xee0e);
    write_cmos_sensor(0x2dda, 0x4039);
    write_cmos_sensor(0x2ddc, 0x0021);
    write_cmos_sensor(0x2dde, 0x3c0a);
    write_cmos_sensor(0x2de0, 0x1008);
    write_cmos_sensor(0x2de2, 0x6e0e);
    write_cmos_sensor(0x2de4, 0x6f0f);
    write_cmos_sensor(0x2de6, 0x9b0f);
    write_cmos_sensor(0x2de8, 0x2805);
    write_cmos_sensor(0x2dea, 0x2002);
    write_cmos_sensor(0x2dec, 0x9a0e);
    write_cmos_sensor(0x2dee, 0x2802);
    write_cmos_sensor(0x2df0, 0x8a0e);
    write_cmos_sensor(0x2df2, 0x7b0f);
    write_cmos_sensor(0x2df4, 0x6c0c);
    write_cmos_sensor(0x2df6, 0x6d0d);
    write_cmos_sensor(0x2df8, 0x6808);
    write_cmos_sensor(0x2dfa, 0x8319);
    write_cmos_sensor(0x2dfc, 0x23f1);
    write_cmos_sensor(0x2dfe, 0x4130);
    write_cmos_sensor(0x2ffe, 0xd800);
    //    EOFIRM
    //--------------------------------------------------------------------
    // end of software code
    //--------------------------------------------------------------------
    //----- Initial -----//
    write_cmos_sensor(0x3048, 0x5020); //sreg09
    write_cmos_sensor(0x0f30, 0x001f); //pll_cfg_ramp0
    write_cmos_sensor(0x0f36, 0x001f); //pll_cfg_mipi0
    write_cmos_sensor(0x0c00, 0x11d8); //blc_ctrl0 - b4:dpc_en b0:blc_en, blc_ctrl1 - b7:bwi b4:act_ofs b0:tobp_ofs
    write_cmos_sensor(0x0c02, 0x0011); //blc_ctrl2 - b0:hdr_en, blc_ctrl3 - b4:dither_en b0:channel_blc
    write_cmos_sensor(0x0c04, 0x5000); //blc_ctrl4 - b7:obp bypass, blc_ctrl5
    write_cmos_sensor(0x0c06, 0x01eb); //blc_dig_offset
    write_cmos_sensor(0x0c10, 0x0040); //act_rr_offset
    write_cmos_sensor(0x0c12, 0x0040); //act_gr_offset
    write_cmos_sensor(0x0c14, 0x0040); //act_gb_offset
    write_cmos_sensor(0x0c16, 0x0040); //act_bb_offset
    write_cmos_sensor(0x0c18, 0x8000); //fobp_iir_ctrl0
    write_cmos_sensor(0x0c62, 0x0194); //dark_pedestal_r
    write_cmos_sensor(0x0c64, 0x0286); //dark_pedestal_gr
    write_cmos_sensor(0x0c66, 0x0294); //dark_pedestal_gb
    write_cmos_sensor(0x0c68, 0x0100); //dark_pedestal_b
    write_cmos_sensor(0x0cb2, 0x0200); //black_level_reference
    write_cmos_sensor(0x0714, 0xe8e8); //black_level_ratio_r_gr
    write_cmos_sensor(0x0716, 0xede8); //black_level_ratio_gb_b
    write_cmos_sensor(0x0900, 0x0300); //mipi_tx_op_en, mipi_tx_test_mode
    write_cmos_sensor(0x0902, 0xc319); //mipi_tx_op_mode1, mipi_tx_op_mode2
    write_cmos_sensor(0x095a, 0x0099); //mipi_static0
    write_cmos_sensor(0x095c, 0x1111); //mipi_static1
    write_cmos_sensor(0x095e, 0xaac0); //mipi_static2
    write_cmos_sensor(0x0960, 0x5d2e); //mipi_static3
    write_cmos_sensor(0x0a38, 0x080c); //lpd00 x/y
    write_cmos_sensor(0x0a3a, 0x140c); //lpd01 x/y
    write_cmos_sensor(0x0a3c, 0x280c); //lpd02 x/y
    write_cmos_sensor(0x0a3e, 0x340c); //lpd03 x/y
    write_cmos_sensor(0x0a40, 0x0418); //lpd04 x/y
    write_cmos_sensor(0x0a42, 0x1818); //lpd05 x/y
    write_cmos_sensor(0x0a44, 0x2418); //lpd06 x/y
    write_cmos_sensor(0x0a46, 0x3818); //lpd07 x/y
    write_cmos_sensor(0x0a48, 0x082c); //lpd08 x/y
    write_cmos_sensor(0x0a4a, 0x142c); //lpd09 x/y
    write_cmos_sensor(0x0a4c, 0x282c); //lpd10 x/y
    write_cmos_sensor(0x0a4e, 0x342c); //lpd11 x/y
    write_cmos_sensor(0x0a50, 0x0438); //lpd12 x/y
    write_cmos_sensor(0x0a52, 0x1838); //lpd13 x/y
    write_cmos_sensor(0x0a54, 0x2438); //lpd14 x/y
    write_cmos_sensor(0x0a56, 0x3838); //lpd15 x/y
    write_cmos_sensor(0x0a58, 0x0808); //rgt00 x/y
    write_cmos_sensor(0x0a5a, 0x1408); //rgt01 x/y
    write_cmos_sensor(0x0a5c, 0x2808); //rgt02 x/y
    write_cmos_sensor(0x0a5e, 0x3408); //rgt03 x/y
    write_cmos_sensor(0x0a60, 0x041c); //rgt04 x/y
    write_cmos_sensor(0x0a62, 0x181c); //rgt05 x/y
    write_cmos_sensor(0x0a64, 0x241c); //rgt06 x/y
    write_cmos_sensor(0x0a66, 0x381c); //rgt07 x/y
    write_cmos_sensor(0x0a68, 0x0828); //rgt08 x/y
    write_cmos_sensor(0x0a6a, 0x1428); //rgt09 x/y
    write_cmos_sensor(0x0a6c, 0x2828); //rgt10 x/y
    write_cmos_sensor(0x0a6e, 0x3428); //rgt11 x/y
    write_cmos_sensor(0x0a70, 0x043c); //rgt12 x/y
    write_cmos_sensor(0x0a72, 0x183c); //rgt13 x/y
    write_cmos_sensor(0x0a74, 0x243c); //rgt14 x/y
    write_cmos_sensor(0x0a76, 0x383c); //rgt15 x/y
    write_cmos_sensor(0x0404, 0x015c); //blk_width
    write_cmos_sensor(0x0406, 0x0138); //blk_height
    write_cmos_sensor(0x040a, 0x0115); //inv_col02
    write_cmos_sensor(0x040c, 0x022a); //inv_col01
    write_cmos_sensor(0x040e, 0xbc52); //inv_col35
    write_cmos_sensor(0x0410, 0x0056); //inv_row02
    write_cmos_sensor(0x0412, 0x00ac); //inv_row1
    write_cmos_sensor(0x0414, 0x6907); //inv_row35
    write_cmos_sensor(0x0422, 0x0011); //inv_pd_col02
    write_cmos_sensor(0x0424, 0x0023); //inv_pd_col01
    write_cmos_sensor(0x0426, 0x2f15); //inv_pd_col35
    write_cmos_sensor(0x0428, 0x0016); //inv_pd_row02
    write_cmos_sensor(0x042a, 0x002b); //inv_pd_row1
    write_cmos_sensor(0x042c, 0x3483); //inv_pd_row35
    write_cmos_sensor(0x0324, 0x0100); //ADPC col_boarder on
    write_cmos_sensor(0x0600, 0x003e); //pdpc_ctl0 - b0:dyn dpc on, b1-2 : pd flag selection, b3 : pd_pxl_bypass_en - Dyn-OFF & PD-ON
    write_cmos_sensor(0x0a78, 0x0400); //Not ROUND_ACC_initialization
//    write_cmos_sensor(0x000e, 0x0100); //image orient
    write_cmos_sensor(0x000e, 0x0200); //image orient
    write_cmos_sensor(0x0a10, 0x400c); //data pedestal on
    write_cmos_sensor(0x003e, 0x0000); //test mode sel - off
    write_cmos_sensor(0x0074, 0x0bb8); //coarse integ time
//write_cmos_sensor(0x0a04, 0x03ee); //isp enable
    write_cmos_sensor(0x0a04, 0x03e2); //isp enable
    write_cmos_sensor(0x0076, 0x0000); //analog gain
    write_cmos_sensor(0x0724, 0x0f1f); //extra static
    write_cmos_sensor(0x0068, 0x0703); //ci margin
    write_cmos_sensor(0x0060, 0x0000); //customer code
    write_cmos_sensor(0x0062, 0x0127); //RAM F/W version
    write_cmos_sensor(0x075c, 0x0100); //OTP ctrl b5:lsc_flag b4:lsc_checksum b3:pdlsc_en b2:dga_en b1:lsc_en b0:adpc_en
    write_cmos_sensor(0x075e, 0x0535); //LSC para otp address
    write_cmos_sensor(0x0012, 0x0fcd); //PDLSC para otp address
    write_cmos_sensor(0x0806, 0x0002); //fmt y start
}

void hi1333_preview_setting_normal(void)
{
     write_cmos_sensor(0x0a00, 0x0000);
    // frame rate = 61.09fps
    // vblank length = 892.80us
    //----- Mode -----//
    write_cmos_sensor(0x0f38, 0x037d); //pll_cfg_mipi1
    write_cmos_sensor(0x0f3a, 0x4107); //pll_cfg_mipi2 b10-8:001 mipi 1/2
    write_cmos_sensor(0x093e, 0x0100); //mipi_tx_col_read_ctrl
    write_cmos_sensor(0x0920, 0xc103); //mipi_exit_seq, tlpx
    write_cmos_sensor(0x0922, 0x030d); //tclk_prepare, tclk_zero
    write_cmos_sensor(0x0924, 0x0204); //tclk_pre, ths_prepare
    write_cmos_sensor(0x0926, 0x060a); //ths_zero, ths_trail
    write_cmos_sensor(0x0928, 0x0704); //tclk_post, tclk_trail
    write_cmos_sensor(0x092a, 0x0505); //texit, tsync
    write_cmos_sensor(0x092c, 0x0a00); //tpd_sync
    write_cmos_sensor(0x0910, 0x0358); //mipi_vblank_delay
    write_cmos_sensor(0x0912, 0x0062); //mipi_hblank_delay
    write_cmos_sensor(0x0a2a, 0x8060); //PDAF patch x/y num bin2
    write_cmos_sensor(0x0a2c, 0x2020); //PDAF patch x/y size bin2
    write_cmos_sensor(0x0a32, 0x0301); //PDAF cnt012 bin2
    write_cmos_sensor(0x0a26, 0x0048); //PDAF x patch offset bin2
    write_cmos_sensor(0x0a28, 0x001c); //PDAF y patch offset bin2
    write_cmos_sensor(0x0a36, 0x0000); //PDAF win loc sx/sy bin2
    write_cmos_sensor(0x0408, 0x0000); //lsc spare
    write_cmos_sensor(0x0418, 0x0000); //lsc win_h
    write_cmos_sensor(0x0800, 0x0400); //fmt ctrl
    write_cmos_sensor(0x0008, 0x02e8); //line length pck
    write_cmos_sensor(0x000c, 0x000c); //colgen start
    write_cmos_sensor(0x0804, 0x0008); //fmt x cropping
    write_cmos_sensor(0x0026, 0x003c); //y addr start active
    write_cmos_sensor(0x002c, 0x0c71); //y addr end active
    write_cmos_sensor(0x005c, 0x0404); //y dummy size
    write_cmos_sensor(0x002e, 0x3311); //y even/odd inc tobp
    write_cmos_sensor(0x0032, 0x3311); //y even/odd inc active
    write_cmos_sensor(0x0006, 0x0ce4); //frame length lines
    write_cmos_sensor(0x0a0e, 0x0002); //image mode/digial binning mode
    write_cmos_sensor(0x0a12, 0x0838); //x output size
    write_cmos_sensor(0x0a14, 0x0618); //y output size
    write_cmos_sensor(0x0050, 0x4300); //analog control b7:vblank_analog_off b6:ag_cal b5:pd_rst2 b4:rdo_set_off b3:pat_addr_en b2:sub3 b1:sreg re-load b0:sreg write
    write_cmos_sensor(0x0722, 0x0702); //d2a_pxl_drv_pwr/d2a_row_binning_en - on
    write_cmos_sensor(0x004c, 0x0100); //tg enable,hdr off
     write_cmos_sensor(0x0a00, 0x0100);
};


void hi1333_capture_setting_normal(void)
{
    write_cmos_sensor(0x0a00, 0x0000);
    // frame rate = 30.55fps
    // vblank length = 1785.60us
    //----- Mode -----//
    write_cmos_sensor(0x0f38, 0x037d); //pll_cfg_mipi1
    write_cmos_sensor(0x0f3a, 0x4007); //pll_cfg_mipi2
    write_cmos_sensor(0x093e, 0x0000); //mipi_tx_col_read_ctrl
    write_cmos_sensor(0x0920, 0xc106); //mipi_exit_seq, tlpx
    write_cmos_sensor(0x0922, 0x061a); //tclk_prepare, tclk_zero
    write_cmos_sensor(0x0924, 0x0206); //tclk_pre, ths_prepare
    write_cmos_sensor(0x0926, 0x0b09); //ths_zero, ths_trail
    write_cmos_sensor(0x0928, 0x0b08); //tclk_post, tclk_trail
    write_cmos_sensor(0x092a, 0x0a06); //texit, tsync
    write_cmos_sensor(0x092c, 0x1600); //tpd_sync
    write_cmos_sensor(0x0910, 0x06d1); //mipi_vblank_delay
    write_cmos_sensor(0x0912, 0x00dd); //mipi_hblank_delay
    write_cmos_sensor(0x0914, 0x0040); //mipi_hblank_short_delay1
    write_cmos_sensor(0x0916, 0x0040); //mipi_hblank_short_delay2
    write_cmos_sensor(0x091a, 0x003b); //mipi_pd_hblank_delay
    write_cmos_sensor(0x0a2a, 0x8060); //PDAF patch x/y num normal
    write_cmos_sensor(0x0a2c, 0x2020); //PDAF patch x/y size normal
    write_cmos_sensor(0x0a32, 0x0301); //PDAF cnt012 normal
    write_cmos_sensor(0x0a26, 0x0048); //PDAF x patch offset normal
    write_cmos_sensor(0x0a28, 0x001a); //PDAF y patch offset normal
    write_cmos_sensor(0x0a36, 0x0000); //PDAF win loc sx/sy normal
    write_cmos_sensor(0x0408, 0x0202); //lsc spare
    write_cmos_sensor(0x0418, 0x0000); //lsc win_h
    write_cmos_sensor(0x0800, 0x0000); //fmt ctrl
    write_cmos_sensor(0x0008, 0x02e8); //line length pck
    write_cmos_sensor(0x000c, 0x000c); //colgen start
    write_cmos_sensor(0x0804, 0x0010); //fmt x cropping
    write_cmos_sensor(0x0026, 0x003e); //y addr start active
    write_cmos_sensor(0x002c, 0x0c71); //y addr end active
    write_cmos_sensor(0x005c, 0x0202); //y dummy size
    write_cmos_sensor(0x002e, 0x1111); //y even/odd inc tobp
    write_cmos_sensor(0x0032, 0x1111); //y even/odd inc active
    write_cmos_sensor(0x0006, 0x0ce4); //frame length lines
    write_cmos_sensor(0x0a0e, 0x0001); //image mode/digial binning mode
    write_cmos_sensor(0x0a12, 0x1070); //x output size
    write_cmos_sensor(0x0a14, 0x0c30); //y output size
    write_cmos_sensor(0x0050, 0x4300); //analog control b7:vblank_analog_off b6:ag_cal b5:pd_rst2 b4:rdo_set_off b3:pat_addr_en b2:sub3 b1:sreg re-load b0:sreg write
    write_cmos_sensor(0x0722, 0x0700); //d2a_pxl_drv_pwr/d2a_row_binning_en
    write_cmos_sensor(0x004c, 0x0100); //tg enable,hdr off
    write_cmos_sensor(0x0a00, 0x0100);
}

void hi1333_capture_setting_pip(void)
{
    write_cmos_sensor(0x0a00, 0x0000);

    write_cmos_sensor(0x0f38, 0x077d); //pll_cfg_mipi1 b7-0: 4C mipi_mdiv
    write_cmos_sensor(0x0f3a, 0x4007); //pll_cfg_mipi2
    write_cmos_sensor(0x093e, 0x0000); //mipi_tx_col_read_ctrl
    write_cmos_sensor(0x0920, 0xc103); //mipi_exit_seq, tlpx
    write_cmos_sensor(0x0922, 0x030d); //tclk_prepare, tclk_zero
    write_cmos_sensor(0x0924, 0x0204); //tclk_pre, ths_prepare
    write_cmos_sensor(0x0926, 0x060a); //ths_zero, ths_trail
    write_cmos_sensor(0x0928, 0x0704); //tclk_post, tclk_trail
    write_cmos_sensor(0x092a, 0x0505); //texit, tsync
    write_cmos_sensor(0x092c, 0x0a00); //tpd_sync
    write_cmos_sensor(0x0910, 0x06fa); //mipi_vblank_delay
    write_cmos_sensor(0x0912, 0x00eb); //mipi_hblank_delay
    write_cmos_sensor(0x0914, 0x004e); //mipi_hblank_short_delay1
    write_cmos_sensor(0x0916, 0x004e); //mipi_hblank_short_delay2
    write_cmos_sensor(0x091a, 0x0049); //mipi_pd_hblank_delay
    write_cmos_sensor(0x0a2a, 0x8060); //PDAF patch x/y num normal
    write_cmos_sensor(0x0a2c, 0x2020); //PDAF patch x/y size normal
    write_cmos_sensor(0x0a32, 0x0301); //PDAF cnt012 normal
    write_cmos_sensor(0x0a26, 0x0048); //PDAF x patch offset normal
    write_cmos_sensor(0x0a28, 0x001a); //PDAF y patch offset normal
    write_cmos_sensor(0x0a36, 0x0000); //PDAF win loc sx/sy normal
    write_cmos_sensor(0x0408, 0x0202); //lsc spare
    write_cmos_sensor(0x0418, 0x0000); //lsc win_h
    write_cmos_sensor(0x0800, 0x0000); //fmt ctrl
    write_cmos_sensor(0x0008, 0x05d0); //line length pck
    write_cmos_sensor(0x000c, 0x0006); //colgen start
    write_cmos_sensor(0x0804, 0x0010); //fmt x cropping
    write_cmos_sensor(0x0026, 0x003e); //y addr start active
    write_cmos_sensor(0x002c, 0x0c71); //y addr end active
    write_cmos_sensor(0x005c, 0x0202); //y dummy size
    write_cmos_sensor(0x002e, 0x1111); //y even/odd inc tobp
    write_cmos_sensor(0x0032, 0x1111); //y even/odd inc active
    write_cmos_sensor(0x0006, 0x0ce4); //frame length lines
    write_cmos_sensor(0x0a0e, 0x0001); //image mode/digial binning mode
    write_cmos_sensor(0x0a12, 0x1070); //x output size
    write_cmos_sensor(0x0a14, 0x0c30); //y output size
    write_cmos_sensor(0x0050, 0x4300); //analog control b4:rdo_set_off b3:pat_addr_en b2:sub3 b1:sreg re-load b0:sreg write
    write_cmos_sensor(0x0722, 0x0700); //d2a_pxl_drv_pwr/d2a_row_binning_en
    write_cmos_sensor(0x004c, 0x0100); //tg enable,hdr off


    write_cmos_sensor(0x0a00, 0x0100);
}

void hi1333_hs_video_setting_normal(void)
{
    write_cmos_sensor(0x0a00, 0x0000);

    write_cmos_sensor(0x0f38, 0x037d); //pll_cfg_mipi1
    write_cmos_sensor(0x0f3a, 0x4407); //pll_cfg_mipi2 b10-8:100 mipi 1/6
    write_cmos_sensor(0x093e, 0x0500); //mipi_tx_col_read_ctrl
    write_cmos_sensor(0x0920, 0xc101); //mipi_exit_seq, tlpx
    write_cmos_sensor(0x0922, 0x0105); //tclk_prepare, tclk_zero
    write_cmos_sensor(0x0924, 0x0201); //tclk_pre, ths_prepare
    write_cmos_sensor(0x0926, 0x020a); //ths_zero, ths_trail
    write_cmos_sensor(0x0928, 0x0202); //tclk_post, tclk_trail
    write_cmos_sensor(0x092a, 0x0203); //texit, tsync
    write_cmos_sensor(0x092c, 0x0100); //tpd_sync
    write_cmos_sensor(0x0910, 0x010f); //mipi_vblank_delay
    write_cmos_sensor(0x0912, 0x001f); //mipi_hblank_delay
    write_cmos_sensor(0x0a2a, 0x4030); //PDAF patch x/y num vga
    write_cmos_sensor(0x0a2c, 0x4040); //PDAF patch x/y size vga
    write_cmos_sensor(0x0a32, 0x030c); //PDAF cnt012 vga
    write_cmos_sensor(0x0a26, 0x0048); //PDAF x patch offset vga
    write_cmos_sensor(0x0a28, 0x0000); //PDAF y patch offset vga
    write_cmos_sensor(0x0a36, 0x0002); //PDAF win loc sx/sy vga
    write_cmos_sensor(0x0408, 0x0200); //lsc spare
    write_cmos_sensor(0x0418, 0x006c); //lsc win_h
    write_cmos_sensor(0x0800, 0x1400); //fmt ctrl
    write_cmos_sensor(0x0008, 0x02e8); //line length pck
    write_cmos_sensor(0x000c, 0x000c); //colgen start
    write_cmos_sensor(0x0804, 0x0022); //fmt x cropping
    write_cmos_sensor(0x0026, 0x00ac); //y addr start active
    write_cmos_sensor(0x002c, 0x0bff); //y addr end active
    write_cmos_sensor(0x005c, 0x040c); //y dummy size
    write_cmos_sensor(0x002e, 0x3311); //y even/odd inc tobp
    write_cmos_sensor(0x0032, 0x5577); //y even/odd inc active
//    write_cmos_sensor(0x0006, 0x0340); //frame length lines
    write_cmos_sensor(0x0006, 0x0348); //frame length lines
    write_cmos_sensor(0x0a0e, 0x0006); //image mode/digial binning mode
    write_cmos_sensor(0x0a12, 0x0280); //x output size
    write_cmos_sensor(0x0a14, 0x01e0); //y output size
    write_cmos_sensor(0x0050, 0x4720); //analog control b4:rdo_set_off b3:pat_addr_en b2:sub3 b1:sreg re-load b0:sreg write
    write_cmos_sensor(0x0722, 0x0702); //d2a_pxl_drv_pwr/d2a_row_binning_en - on
    write_cmos_sensor(0x004c, 0x0100); //tg enable,hdr off



    write_cmos_sensor(0x0a00, 0x0100);

}

void hi1333_slim_video_setting_normal(void)
{
    write_cmos_sensor(0x0a00, 0x0000);
    // 90fps
    write_cmos_sensor(0x0f38, 0x037d); //pll_cfg_mipi1
    write_cmos_sensor(0x0f3a, 0x4207); //pll_cfg_mipi2 b10-8:010 mipi 1/3
    write_cmos_sensor(0x093e, 0x0200); //mipi_tx_col_read_ctrl
    write_cmos_sensor(0x0920, 0xc102); //mipi_exit_seq, tlpx
    write_cmos_sensor(0x0922, 0x0209); //tclk_prepare, tclk_zero
    write_cmos_sensor(0x0924, 0x0202); //tclk_pre, ths_prepare
    write_cmos_sensor(0x0926, 0x0404); //ths_zero, ths_trail
    write_cmos_sensor(0x0928, 0x0603); //tclk_post, tclk_trail
    write_cmos_sensor(0x092a, 0x0304); //texit, tsync
    write_cmos_sensor(0x092c, 0x0400); //tpd_sync
    write_cmos_sensor(0x0910, 0x023c); //mipi_vblank_delay
    write_cmos_sensor(0x0912, 0x0057); //mipi_hblank_delay
    write_cmos_sensor(0x0a2a, 0x4030); //PDAF patch x/y num hd
    write_cmos_sensor(0x0a2c, 0x4040); //PDAF patch x/y size hd
    write_cmos_sensor(0x0a32, 0x1300); //PDAF cnt012 hd
    write_cmos_sensor(0x0a26, 0x0048); //PDAF x patch offset hd
    write_cmos_sensor(0x0a28, 0x0000); //PDAF y patch offset hd
    write_cmos_sensor(0x0a36, 0x0000); //PDAF win loc sx/sy hd
    write_cmos_sensor(0x0408, 0x0200); //lsc spare
    write_cmos_sensor(0x0418, 0x01a2); //lsc win_h
    write_cmos_sensor(0x0800, 0x0800); //fmt ctrl
    write_cmos_sensor(0x0008, 0x02e8); //line length pck
    write_cmos_sensor(0x000c, 0x000c); //colgen start
    write_cmos_sensor(0x0804, 0x0042); //fmt x cropping
    write_cmos_sensor(0x0026, 0x021a); //y addr start active
    write_cmos_sensor(0x002c, 0x0a93); //y addr end active
    write_cmos_sensor(0x005c, 0x0206); //y dummy size
    write_cmos_sensor(0x002e, 0x1111); //y even/odd inc tobp
    write_cmos_sensor(0x0032, 0x3333); //y even/odd inc active
//    write_cmos_sensor(0x0006, 0x0340); //frame length lines
    write_cmos_sensor(0x0006, 0x0460); //frame length lines
    write_cmos_sensor(0x0a0e, 0x0003); //image mode/digial binning mode
    write_cmos_sensor(0x0a12, 0x0500); //x output size
    write_cmos_sensor(0x0a14, 0x02d0); //y output size
    write_cmos_sensor(0x0050, 0x4720); //analog control b4:rdo_set_off b3:pat_addr_en b2:sub3 b1:sreg re-load b0:sreg write
    write_cmos_sensor(0x0722, 0x0700); //d2a_pxl_drv_pwr/d2a_row_binning_en
    write_cmos_sensor(0x004c, 0x0100); //tg enable,hdr off



    write_cmos_sensor(0x0a00, 0x0100);

}

extern int back_camera_find_success;
extern bool camera_back_probe_ok;
/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;

    LOG_INF("E\n");

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
            spin_lock(&imgsensor_drv_lock);
            imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
            spin_unlock(&imgsensor_drv_lock);
            do {
                *sensor_id = ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));

                if (*sensor_id == imgsensor_info.sensor_id)
                {
                    LOG_INF("i2c write id  : 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                    back_camera_find_success = 3;
                    camera_back_probe_ok = 1;

                    return ERROR_NONE;
            }
                LOG_INF("get_imgsensor_id Read sensor id fail, id: 0x%x 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                retry--;
            } while(retry > 0);
            i++;
            retry = 2;
}

    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint16 sensor_id = 0;

    LOG_INF("begin\n");
    LOG_INF("E\n");

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {

        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        LOG_INF("SP\n");
        do {
            sensor_id = ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);

                break;
            }
            LOG_INF("Read sensor id fail, write id:0x%x id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }

    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;



    /* initail sequence write in  */
    LOG_INF("hi1333_find sensor pass!! Start initial setting\n");
    hi1333_init_setting_normal();
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
       imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    LOG_INF("return ERROR_NONE\n");
    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("begin\n");
    LOG_INF("E\n");

    /*No Need to implement this function*/
    LOG_INF("return ERROR_NONE\n");
    return ERROR_NONE;
}    /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("begin\n");
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    hi1333_preview_setting_normal();


    LOG_INF("return ERROR_NONE\n");
    return ERROR_NONE;
}    /*    preview   */


static void capture_setting(kal_uint16 currentfps)
{
    LOG_INF(" capture, currefps = %d\n",currentfps);

    LOG_INF("burst disable\n");
    if( currentfps < 300 )    hi1333_capture_setting_pip();
    else                    hi1333_capture_setting_normal();

}

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("begin\n");
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
    {
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    else //PIP capture: 15fps for 13M
    {
        if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps, imgsensor_info.cap1.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }

    spin_unlock(&imgsensor_drv_lock);

    capture_setting(imgsensor.current_fps);
    LOG_INF("return ERROR_NONE\n");
    return ERROR_NONE;
}    /* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E");
    LOG_INF("Normal_Video\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    //normal_video_setting(imgsensor.current_fps);

    hi1333_capture_setting_normal();

#ifdef ENABLE_IVHDR
    SetIHDR(imgsensor.ihdr_en);
#endif
    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

      hi1333_hs_video_setting_normal();

    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    hi1333_slim_video_setting_normal();

    return ERROR_NONE;
}    /*    slim_video     */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");

    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */


static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    //sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    //sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    sensor_info->PDAF_Support = 0;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("[contrlo]scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            LOG_INF("case MSDK_SCENARIO_ID_CAMERA_PREVIEW\n");
            preview(image_window, sensor_config_data);
            break;
       // case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
       //     capture(image_window, sensor_config_data);
       //     break;
        // 2016.09.09 Coby
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            LOG_INF("case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG  MSDK_SCENARIO_ID_CAMERA_ZSD \n");
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            LOG_INF("case MSDK_SCENARIO_ID_VIDEO_PREVIEW\n");
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            LOG_INF("case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO\n");
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            LOG_INF("case MSDK_SCENARIO_ID_SLIM_VIDEO\n");
               slim_video(image_window, sensor_config_data);
               break;
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d \n", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
        // Dynamic frame rate
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);

    if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = 10 * framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d\n ", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable)
        imgsensor.autoflicker_en = KAL_TRUE;
    else //Cancel Auto flick
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
              if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
                    imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
                    imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
                    imgsensor.min_frame_length = imgsensor.frame_length;
                    spin_unlock(&imgsensor_drv_lock);
            } else {
                    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
                    imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
                    imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
                    imgsensor.min_frame_length = imgsensor.frame_length;
                    spin_unlock(&imgsensor_drv_lock);
            }
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    UINT16 enable_TP = 0;
    enable_TP = ((read_cmos_sensor(0x0A04) << 8) | read_cmos_sensor(0x0A05));

    LOG_INF("enable: %d", enable);
    // 0x0A05[0]: 1 enable,  0 disable
    // 0x020A[1:0]; 0x01:BLACK, 0x02:COLOR BAR, 0x03:GREY, 0x04:RANDOM

    if (enable) {
//        enable_TP |= 0x0001;
        LOG_INF("mook 0x0A04: 0x%x\n", enable_TP);
        write_cmos_sensor(0x0A04,0x0141);
        write_cmos_sensor(0x020A,0x0200);

    } else {
//        enable_TP &= 0xFFFE;
//        LOG_INF("mook 0x0A04: 0x%x\n", enable_TP);
        write_cmos_sensor(0x0A04,0x03C1);
        write_cmos_sensor(0x020A,0x0000);
    }
    return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;


    unsigned long long *feature_data=(unsigned long long *) feature_para;

    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            LOG_INF("case SENSOR_FEATURE_GET_PERIOD\n");
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ\n");
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            LOG_INF("case SENSOR_FEATURE_SET_ESHUTTER\n");
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            LOG_INF("case SENSOR_FEATURE_SET_NIGHTMODE\n");
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            LOG_INF("case SENSOR_FEATURE_SET_GAIN\n");
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            LOG_INF("case SENSOR_FEATURE_SET_FLASHLIGHT\n");
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            LOG_INF("case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ\n");
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            LOG_INF("case SENSOR_FEATURE_SET_REGISTER\n");
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            LOG_INF("case SENSOR_FEATURE_GET_REGISTER\n");
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            LOG_INF("case SENSOR_FEATURE_GET_LENS_DRIVER_ID\n");
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            LOG_INF("case SENSOR_FEATURE_SET_VIDEO_MODE\n");
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            LOG_INF("case SENSOR_FEATURE_CHECK_SENSOR_ID\n");
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            LOG_INF("case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE\n");
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            LOG_INF("case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO\n");
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            LOG_INF("case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO\n");
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;

/*
        case SENSOR_FEATURE_SET_WRITE_0x0A1A:
//            write_cmos_sensor( 0x041A, (*feature_data_32)&0x0000FFFF );
            bTh = ((*feature_data_32)&0x000000FF);
            break;

        case SENSOR_FEATURE_SET_WRITE_0x0A1B:
//            write_cmos_sensor( 0x041C, (*feature_data_32)&0x0000FFFF );
            bGap = ((*feature_data_32)&0x000000FF);
            write_cmos_sensor( 0x0A1A, (bTh << 8) | bGap);
            break;

        case SENSOR_FEATURE_GET_READ_PARA:
#if 0
            LOG_INF("[HYNIX][SW] read  0x041A:0x%x, 0x041C:0x%x, 0x0536:0x%x, 0x0538:0x%x\n",
                    (read_cmos_sensor(0x041A) << 8 | read_cmos_sensor(0x041B)),
                    (read_cmos_sensor(0x041C) << 8 | read_cmos_sensor(0x041D)),
                    (read_cmos_sensor(0x0536) << 8 | read_cmos_sensor(0x0537)),
                    (read_cmos_sensor(0x0538) << 8 | read_cmos_sensor(0x0539))
                    );
#endif
          LOG_INF("[HYNIX][SW] Th=0x%x, Gap:0x%x, G_sum:0x%x, Level:0x%x\n",
               read_cmos_sensor(0x0A1A),
               read_cmos_sensor(0x0A1B),
               read_cmos_sensor(0x0234),
               read_cmos_sensor(0x0235)
            );
            break;
*/

        case SENSOR_FEATURE_SET_TEST_PATTERN:
            LOG_INF("case SENSOR_FEATURE_SET_TEST_PATTERN\n");
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            LOG_INF("case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE\n");
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("case SENSOR_FEATURE_SET_FRAMERATE\n");
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("case SENSOR_FEATURE_SET_HDR\n");
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
#ifdef ENABLE_IVHDR
            SetIHDR(imgsensor.ihdr_en);
#endif
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("case SENSOR_FEATURE_GET_CROP_INFO\n");
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
        break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN\n");
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        case SENSOR_FEATURE_SET_HDR_SHUTTER:
            LOG_INF("case SENSOR_FEATURE_SET_HDR_SHUTTER\n");
            LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        default:
            LOG_INF("no this  case\n");
            break;
    }

    return ERROR_NONE;
}   /*  feature_control()  */

#ifdef ENABLE_IVHDR
static void SetIHDR(BOOL EnableIHDR)
{
    // Hynix add iHDR Enable Setting Start
    if(EnableIHDR)
    {
        LOG_INF("Hi1333 Video iHDR enable Setting Write\n");
        write_cmos_sensor(0x004c, 0x0101);
    }
    else
    {
        LOG_INF("Hi1333 Video iHDR disable Setting Write\n");
        write_cmos_sensor(0x004c, 0x0100);
    }
}
#endif

SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 HI1333_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    LOG_INF("begin\n");
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    LOG_INF("return ERROR_NONE\n");
    return ERROR_NONE;
}
