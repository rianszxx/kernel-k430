/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 OV8858mipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6795
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *	PengtaoFan
 *	20150716:更新补充打印log中丢失的参数
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
//#include <asm/system.h>
//#include <linux/xlog.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov8858mipiraw_Sensor.h"

#define PFX "OV8858"

//#define OV8858R2AOTP

//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
typedef enum {
	OV8858R2A,
	OV8858R1A
}OV8858_VERSION;

OV8858_VERSION ov8858version = OV8858R2A;
unsigned short syncShutter;

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = OV8858_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0xd14be45,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 74400000,				//record different mode's pclk
		.linelength  = 1928,				//record different mode's linelength
		.framelength = 1284,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 23,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 148800000,
		.linelength  = 1940,
		.framelength = 2556,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.cap1 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 148800000,
		.linelength  = 2425,
		.framelength = 4090,//MODIFY FOR 15FPS FOR MT6735 //2556
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 150,
	},
	.normal_video = {
		.pclk = 148800000,
		.linelength  = 1940,
		.framelength = 2556,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 148800000,				//record different mode's pclk
		.linelength  = 2270,				//record different mode's linelength
		.framelength = 546,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 640,		//record different mode's width of grabwindow
		.grabwindow_height = 480,		//record different mode's height of grabwindow
		/*	 following for  MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 30,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 74400000,				//record different mode's pclk
		.linelength  = 1928,				//record different mode's linelength
		.framelength = 1284,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 30,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	

	},
	.margin = 4,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter
	.max_frame_length = 0x90f7,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num ,don't support Slow motion
	
	.cap_delay_frame = 3,		//enter capture delay frame num
	.pre_delay_frame = 3, 		//enter preview delay frame num
	.video_delay_frame = 3,		//enter video delay frame num
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
  .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
  .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_addr_table = {0x42, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,/*IMAGE_NORMAL,*/				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x4C00,					//current shutter
	.gain = 0x200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x42,//record current sensor's i2c write id
};


/* Sensor output window information*/
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 3296, 2480,	  12,	12, 3272, 2456, 1636, 1228,   0,	0, 1632, 1224,	 0, 0, 1632, 1224}, // Preview 
 { 3296, 2480,	  12,	12, 3272, 2456, 3264, 2452,   0,	0, 3264, 2448,	 0, 0, 3264, 2448}, // capture 
 { 3296, 2480,	  12,	12, 3272, 2456, 3264, 2452,   0,	0, 3264, 2448,	 0, 0, 3264, 2448}, // video 
 { 3296, 2480,	  12,	12, 3272, 2456,  640,  480,   0,	0,  640,  480,	 0, 0,  640,  480}, //hight speed video 
 { 3296, 2480,	  12,	12, 3272, 2456, 1636, 1228,   0,	0, 1632, 1224,	 0, 0, 1632, 1224}};// slim video 

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
  
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should be = %d? \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

#if 0
static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	   
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
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
	} else {
		// Extend frame length
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
	write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
	write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */
#endif


/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
//	kal_uint32 frame_length = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	//write_shutter(shutter);
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

    write_cmos_sensor(0x3208,0x00); // Group hold start recommanded by OVT korea
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	write_cmos_sensor(0x3502, (shutter << 4) & 0xFF);
	write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);	  
	write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
    if( (imgsensor_info.ae_shut_delay_frame !=0) || (imgsensor_info.ae_sensor_gain_delay_frame !=0) )
	{
        write_cmos_sensor(0x3208,0x10); // Group hold end recommanded by OVT korea
        write_cmos_sensor(0x3208, 0xA0); // Group launch  recommanded by OVT korea
	}
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}
#if 0
static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	
	reg_gain = gain*2;
	//reg_gain = reg_gain & 0xFFFF;
	return (kal_uint16)reg_gain;
}
#endif
/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;
	LOG_INF("set_gain %d \n", gain);
	if (gain < BASEGAIN || gain > 10 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 10 * BASEGAIN)
			gain = 10 * BASEGAIN;		 
	}

	//reg_gain = gain2reg(gain);
	reg_gain = gain*2;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

    if( (imgsensor_info.ae_shut_delay_frame !=0) || (imgsensor_info.ae_sensor_gain_delay_frame !=0) )
	{
        write_cmos_sensor(0x3208,0x00); // Group hold start recommanded by OVT korea
	}  

	write_cmos_sensor(0x3508, (reg_gain>>8));
	write_cmos_sensor(0x3509, (reg_gain&0xFF));   

    write_cmos_sensor(0x3208,0x10); // Group hold end recommanded by OVT korea
    write_cmos_sensor(0x3208, 0xA0); // Group launch  recommanded by OVT korea
    
	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
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
			
			
		// Extend frame length first
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3512, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3511, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3510, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}


#if 1
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

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
	
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));		
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}
#endif
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/
static void r1asensor_init(void)
{
	LOG_INF("E\n");
//   ;-----------------------------------------
//   @@ OV8858R1A Key 1632x1224_30FPS_MIPI_2LANE(Binning)
//   100 99 1632 1224 ; Resolution
//   102 80 1
//   102 3601 bb8;30fps
//   102 40 0 ; HDR Mode off
//   ;FPGA set-up
//   c8 01 f2 ;MIPI FPGA
//   ;
//   CL 100 100;delay
//   ;;
//   ;; MIPI=720Mbps, SysClk=144Mhz,Dac Clock=360Mhz.
//   ;;
//   ;;
//   ;; v00_EM_AM14 (02/26/2014) : initial setting
//   ;;
//   ;; AM09 : Add 1080P,720P, HDR and 2 Lane Formats
//   ;;      : Update the setting for R1A final Version(EK)
//   ;; AM14 : fullwell 7K

	write_cmos_sensor(0x3605, 0x30);
	write_cmos_sensor(0x360b, 0x06);
	write_cmos_sensor(0x360c, 0xdc);
	write_cmos_sensor(0x3617, 0x90);
	write_cmos_sensor(0x3618, 0x56);
	write_cmos_sensor(0x361b, 0x00);
	write_cmos_sensor(0x3633, 0x0c);
	write_cmos_sensor(0x3634, 0x0c);
	write_cmos_sensor(0x3635, 0x0c);
	write_cmos_sensor(0x3636, 0x0c);
	write_cmos_sensor(0x3501, 0x4b);
	write_cmos_sensor(0x3502, 0xc0);
	write_cmos_sensor(0x3503, 0x00);
	write_cmos_sensor(0x3508, 0x01);
	write_cmos_sensor(0x3706, 0x6a);
	write_cmos_sensor(0x370b, 0x6a);
	write_cmos_sensor(0x3750, 0x00);
	write_cmos_sensor(0x3751, 0x00);
	write_cmos_sensor(0x37b5, 0x33);
	write_cmos_sensor(0x3d85, 0x14);
	write_cmos_sensor(0x3d8c, 0x73);
	write_cmos_sensor(0x3d8d, 0xde);
	write_cmos_sensor(0x5780, 0xfc);
	write_cmos_sensor(0x5781, 0xdf);
	write_cmos_sensor(0x5782, 0x3f);
	write_cmos_sensor(0x5783, 0x08);
	write_cmos_sensor(0x5784, 0x0c);
	write_cmos_sensor(0x5785, 0x10);
	write_cmos_sensor(0x5786, 0x20);
	write_cmos_sensor(0x5787, 0x40);
	write_cmos_sensor(0x5788, 0x08);
	write_cmos_sensor(0x5789, 0x08);
	write_cmos_sensor(0x578a, 0x02);
	write_cmos_sensor(0x578b, 0x01);
	write_cmos_sensor(0x578c, 0x01);
	write_cmos_sensor(0x578d, 0x0c);
	write_cmos_sensor(0x578e, 0x02);
	write_cmos_sensor(0x578f, 0x01);
	write_cmos_sensor(0x5790, 0x01);
	write_cmos_sensor(0x5791, 0xff);
	write_cmos_sensor(0x5792, 0x07);
	write_cmos_sensor(0x5793, 0x03);
	write_cmos_sensor(0x5794, 0x03);
	write_cmos_sensor(0x5871, 0x0d);
	write_cmos_sensor(0x5870, 0x18);
	write_cmos_sensor(0x586e, 0x40);
	write_cmos_sensor(0x586f, 0x20);
	write_cmos_sensor(0x58f8, 0x00);
	write_cmos_sensor(0x3763, 0x00);
	write_cmos_sensor(0x3768, 0x00);
	write_cmos_sensor(0x470b, 0x00);
	write_cmos_sensor(0x4202, 0x00);
	write_cmos_sensor(0x400d, 0x20);
	write_cmos_sensor(0x4040, 0x00);
	write_cmos_sensor(0x403e, 0x00);
	write_cmos_sensor(0x4041, 0x07);
	write_cmos_sensor(0x3007, 0x00);
	write_cmos_sensor(0x400a, 0x00);
	write_cmos_sensor(0x4009, 0x81);
	write_cmos_sensor(0x4011, 0x00);
	write_cmos_sensor(0x4020, 0x00);
	write_cmos_sensor(0x4021, 0x04);
	write_cmos_sensor(0x4022, 0x04);
	write_cmos_sensor(0x4023, 0xb9);
	write_cmos_sensor(0x4024, 0x05);
	write_cmos_sensor(0x4025, 0x2a);
	write_cmos_sensor(0x4026, 0x05);
	write_cmos_sensor(0x4027, 0x2b);
	write_cmos_sensor(0x4028, 0x00);
	write_cmos_sensor(0x4029, 0x02);
	write_cmos_sensor(0x402a, 0x04);
	write_cmos_sensor(0x402b, 0x04);
	write_cmos_sensor(0x402c, 0x02);
	write_cmos_sensor(0x402d, 0x02);
	write_cmos_sensor(0x402e, 0x08);
	write_cmos_sensor(0x402f, 0x02);
}
static void sensor_init(void)
{
	LOG_INF("E\n");
//  @@ OV8858R2A Initial 1632x1224_30FPS_MIPI_2LANE(Binning)
//  100 99 1632 1224 ; Resolution
//  102 80 1
//  102 3601 bb8;30fps
//  102 40 0 ; HDR Mode off
//  ;FPGA set-up
//  c8 01 f2 ;MIPI FPGA
//  ;
//  CL 100 100;delay
//  ;;
//  ;; MIPI=720Mbps, SysClk=148.8Mhz,Dac Clock=360Mhz.
//  ;;
//  ;;
//  ;; v00_01_00 (05/29/2014) : initial setting
//  ;;
//  ;; AM19 : 3617 <- 0xC0
//  ;;
//  ;; AM20 : change 0x3618=0x5a
//  ;; AM22 :
//  ;;   6c 3617 40 
//  ::   6c 4011 20 
//  ::   6c 403e 08
//  ;;   6c 4040 07
//  ;;   6c 58f8 3d
//  ;;


	write_cmos_sensor(0x103 , 0x01);
	write_cmos_sensor(0x303f, 0x01);
	write_cmos_sensor(0x3012, 0x6c);
	write_cmos_sensor(0x100 , 0x00);
	write_cmos_sensor(0x302 , 0x1e);
	write_cmos_sensor(0x303 , 0x00);
	write_cmos_sensor(0x304 , 0x03);
	write_cmos_sensor(0x30d , 0x1f);
	write_cmos_sensor(0x30e , 0x02);
	write_cmos_sensor(0x30f , 0x04);
	write_cmos_sensor(0x312 , 0x03);
	write_cmos_sensor(0x31e , 0x0c);
	write_cmos_sensor(0x3007, 0x80);
	write_cmos_sensor(0x3600, 0x00);
	write_cmos_sensor(0x3601, 0x00);
	write_cmos_sensor(0x3602, 0x00);
	write_cmos_sensor(0x3603, 0x00);
	write_cmos_sensor(0x3604, 0x22);
	write_cmos_sensor(0x3605, 0x20);
	write_cmos_sensor(0x3606, 0x00);
	write_cmos_sensor(0x3607, 0x20);
	write_cmos_sensor(0x3608, 0x11);
	write_cmos_sensor(0x3609, 0x28);
	write_cmos_sensor(0x360a, 0x00);
	write_cmos_sensor(0x360b, 0x05);
	write_cmos_sensor(0x360c, 0xd4);
	write_cmos_sensor(0x360d, 0x40);
	write_cmos_sensor(0x360e, 0x0c);
	write_cmos_sensor(0x360f, 0x20);
	write_cmos_sensor(0x3610, 0x07);
	write_cmos_sensor(0x3611, 0x20);
	write_cmos_sensor(0x3612, 0x88);
	write_cmos_sensor(0x3613, 0x80);
	write_cmos_sensor(0x3614, 0x58);
	write_cmos_sensor(0x3615, 0x00);
	write_cmos_sensor(0x3616, 0x4a);
	write_cmos_sensor(0x3617, 0x40);
	write_cmos_sensor(0x3618, 0x5a);
	write_cmos_sensor(0x3619, 0x70);
	write_cmos_sensor(0x361a, 0x99);
	write_cmos_sensor(0x361b, 0x0a);
	write_cmos_sensor(0x361c, 0x07);
	write_cmos_sensor(0x361d, 0x00);
	write_cmos_sensor(0x361e, 0x00);
	write_cmos_sensor(0x361f, 0x00);
	write_cmos_sensor(0x3638, 0xff);
	write_cmos_sensor(0x3633, 0x0f);
	write_cmos_sensor(0x3634, 0x0f);
	write_cmos_sensor(0x3635, 0x0f);
	write_cmos_sensor(0x3636, 0x12);
	write_cmos_sensor(0x3645, 0x13);
	write_cmos_sensor(0x3646, 0x83);
	write_cmos_sensor(0x364a, 0x07);
	write_cmos_sensor(0x3015, 0x00);
	write_cmos_sensor(0x3018, 0x72);
	write_cmos_sensor(0x3020, 0x93);
	write_cmos_sensor(0x3022, 0x01);
	write_cmos_sensor(0x3031, 0x0a);
	write_cmos_sensor(0x3034, 0x00);
	write_cmos_sensor(0x3106, 0x01);
	write_cmos_sensor(0x3305, 0xf1);
	write_cmos_sensor(0x3308, 0x00);
	write_cmos_sensor(0x3309, 0x28);
	write_cmos_sensor(0x330a, 0x00);
	write_cmos_sensor(0x330b, 0x20);
	write_cmos_sensor(0x330c, 0x00);
	write_cmos_sensor(0x330d, 0x00);
	write_cmos_sensor(0x330e, 0x00);
	write_cmos_sensor(0x330f, 0x40);
	write_cmos_sensor(0x3307, 0x04);
	write_cmos_sensor(0x3500, 0x00);
	write_cmos_sensor(0x3501, 0x4d);
	write_cmos_sensor(0x3502, 0x40);
	write_cmos_sensor(0x3503, 0x80);
	write_cmos_sensor(0x3505, 0x80);
	write_cmos_sensor(0x3508, 0x02);
	write_cmos_sensor(0x3509, 0x00);
	write_cmos_sensor(0x350c, 0x00);
	write_cmos_sensor(0x350d, 0x80);
	write_cmos_sensor(0x3510, 0x00);
	write_cmos_sensor(0x3511, 0x02);
	write_cmos_sensor(0x3512, 0x00);
	write_cmos_sensor(0x3700, 0x18);
	write_cmos_sensor(0x3701, 0x0c);
	write_cmos_sensor(0x3702, 0x28);
	write_cmos_sensor(0x3703, 0x19);
	write_cmos_sensor(0x3704, 0x14);
	write_cmos_sensor(0x3705, 0x00);
	write_cmos_sensor(0x3706, 0x82);
	write_cmos_sensor(0x3707, 0x04);
	write_cmos_sensor(0x3708, 0x24);
	write_cmos_sensor(0x3709, 0x33);
	write_cmos_sensor(0x370a, 0x01);
	write_cmos_sensor(0x370b, 0x82);
	write_cmos_sensor(0x370c, 0x04);
	write_cmos_sensor(0x3718, 0x12);
	write_cmos_sensor(0x3719, 0x31);
	write_cmos_sensor(0x3712, 0x42);
	write_cmos_sensor(0x3714, 0x24);
	write_cmos_sensor(0x371e, 0x19);
	write_cmos_sensor(0x371f, 0x40);
	write_cmos_sensor(0x3720, 0x05);
	write_cmos_sensor(0x3721, 0x05);
	write_cmos_sensor(0x3724, 0x06);
	write_cmos_sensor(0x3725, 0x01);
	write_cmos_sensor(0x3726, 0x06);
	write_cmos_sensor(0x3728, 0x05);
	write_cmos_sensor(0x3729, 0x02);
	write_cmos_sensor(0x372a, 0x03);
	write_cmos_sensor(0x372b, 0x53);
	write_cmos_sensor(0x372c, 0xa3);
	write_cmos_sensor(0x372d, 0x53);
	write_cmos_sensor(0x372e, 0x06);
	write_cmos_sensor(0x372f, 0x10);
	write_cmos_sensor(0x3730, 0x01);
	write_cmos_sensor(0x3731, 0x06);
	write_cmos_sensor(0x3732, 0x14);
	write_cmos_sensor(0x3733, 0x10);
	write_cmos_sensor(0x3734, 0x40);
	write_cmos_sensor(0x3736, 0x20);
	write_cmos_sensor(0x373a, 0x05);
	write_cmos_sensor(0x373b, 0x06);
	write_cmos_sensor(0x373c, 0x0a);
	write_cmos_sensor(0x373e, 0x03);
	write_cmos_sensor(0x3750, 0x0a);
	write_cmos_sensor(0x3751, 0x0e);
	write_cmos_sensor(0x3755, 0x10);
	write_cmos_sensor(0x3758, 0x00);
	write_cmos_sensor(0x3759, 0x4c);
	write_cmos_sensor(0x375a, 0x06);
	write_cmos_sensor(0x375b, 0x13);
	write_cmos_sensor(0x375c, 0x20);
	write_cmos_sensor(0x375d, 0x02);
	write_cmos_sensor(0x375e, 0x00);
	write_cmos_sensor(0x375f, 0x14);
	write_cmos_sensor(0x3768, 0xcc);
	write_cmos_sensor(0x3769, 0x44);
	write_cmos_sensor(0x376a, 0x44);
	write_cmos_sensor(0x3761, 0x00);
	write_cmos_sensor(0x3762, 0x00);
	write_cmos_sensor(0x3763, 0x18);
	write_cmos_sensor(0x3766, 0xff);
	write_cmos_sensor(0x376b, 0x00);
	write_cmos_sensor(0x3772, 0x23);
	write_cmos_sensor(0x3773, 0x02);
	write_cmos_sensor(0x3774, 0x16);
	write_cmos_sensor(0x3775, 0x12);
	write_cmos_sensor(0x3776, 0x04);
	write_cmos_sensor(0x3777, 0x00);
	write_cmos_sensor(0x3778, 0x17);
	write_cmos_sensor(0x37a0, 0x44);
	write_cmos_sensor(0x37a1, 0x3d);
	write_cmos_sensor(0x37a2, 0x3d);
	write_cmos_sensor(0x37a3, 0x00);
	write_cmos_sensor(0x37a4, 0x00);
	write_cmos_sensor(0x37a5, 0x00);
	write_cmos_sensor(0x37a6, 0x00);
	write_cmos_sensor(0x37a7, 0x44);
	write_cmos_sensor(0x37a8, 0x4c);
	write_cmos_sensor(0x37a9, 0x4c);
	write_cmos_sensor(0x3760, 0x00);
	write_cmos_sensor(0x376f, 0x01);
	write_cmos_sensor(0x37aa, 0x44);
	write_cmos_sensor(0x37ab, 0x2e);
	write_cmos_sensor(0x37ac, 0x2e);
	write_cmos_sensor(0x37ad, 0x33);
	write_cmos_sensor(0x37ae, 0x0d);
	write_cmos_sensor(0x37af, 0x0d);
	write_cmos_sensor(0x37b0, 0x00);
	write_cmos_sensor(0x37b1, 0x00);
	write_cmos_sensor(0x37b2, 0x00);
	write_cmos_sensor(0x37b3, 0x42);
	write_cmos_sensor(0x37b4, 0x42);
	write_cmos_sensor(0x37b5, 0x31);
	write_cmos_sensor(0x37b6, 0x00);
	write_cmos_sensor(0x37b7, 0x00);
	write_cmos_sensor(0x37b8, 0x00);
	write_cmos_sensor(0x37b9, 0xff);
	write_cmos_sensor(0x3800, 0x00);
	write_cmos_sensor(0x3801, 0x0c);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x0c);
	write_cmos_sensor(0x3804, 0x0c);
	write_cmos_sensor(0x3805, 0xd3);
	write_cmos_sensor(0x3806, 0x09);
	write_cmos_sensor(0x3807, 0xa3);
	write_cmos_sensor(0x3808, 0x06);
	write_cmos_sensor(0x3809, 0x60);
	write_cmos_sensor(0x380a, 0x04);
	write_cmos_sensor(0x380b, 0xc8);
	write_cmos_sensor(0x380c, 0x07);
	write_cmos_sensor(0x380d, 0x88);
	write_cmos_sensor(0x380e, 0x04);
	write_cmos_sensor(0x380f, 0xdc);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x04);
	write_cmos_sensor(0x3813, 0x02);
	write_cmos_sensor(0x3814, 0x03);
	write_cmos_sensor(0x3815, 0x01);
	write_cmos_sensor(0x3820, 0x00);
	write_cmos_sensor(0x3821, 0x67);
	write_cmos_sensor(0x382a, 0x03);
	write_cmos_sensor(0x382b, 0x01);
	write_cmos_sensor(0x3830, 0x08);
	write_cmos_sensor(0x3836, 0x02);
	write_cmos_sensor(0x3837, 0x18);
	
	// modify for pre cap view diff 20150105
	//write_cmos_sensor(0x3841, 0xff);
	write_cmos_sensor(0x3841, 0xef);
	
	write_cmos_sensor(0x3846, 0x48);
	write_cmos_sensor(0x3d85, 0x16);
	write_cmos_sensor(0x3d8c, 0x73);
	write_cmos_sensor(0x3d8d, 0xde);
	write_cmos_sensor(0x3f08, 0x08);
	write_cmos_sensor(0x3f0a, 0x00);
	write_cmos_sensor(0x4000, 0xf1);
	write_cmos_sensor(0x4001, 0x96);// 0x10
	write_cmos_sensor(0x4005, 0x10);
	write_cmos_sensor(0x4002, 0x27);
	write_cmos_sensor(0x4009, 0x83);
	write_cmos_sensor(0x400a, 0x01);
	write_cmos_sensor(0x400b, 0x0c);
	write_cmos_sensor(0x400d, 0x10);
	write_cmos_sensor(0x4011, 0x20);
	write_cmos_sensor(0x401b, 0x00);
	write_cmos_sensor(0x401d, 0x00);
	write_cmos_sensor(0x4020, 0x00);
	write_cmos_sensor(0x4021, 0x04);
	write_cmos_sensor(0x4022, 0x04);
	write_cmos_sensor(0x4023, 0xb9);
	write_cmos_sensor(0x4024, 0x05);
	write_cmos_sensor(0x4025, 0x2a);
	write_cmos_sensor(0x4026, 0x05);
	write_cmos_sensor(0x4027, 0x2b);
	write_cmos_sensor(0x4028, 0x00);
	write_cmos_sensor(0x4029, 0x02);
	write_cmos_sensor(0x402a, 0x04);
	write_cmos_sensor(0x402b, 0x04);
	write_cmos_sensor(0x402c, 0x02);
	write_cmos_sensor(0x402d, 0x02);
	write_cmos_sensor(0x402e, 0x08);
	write_cmos_sensor(0x402f, 0x02);
	write_cmos_sensor(0x401f, 0x00);
	write_cmos_sensor(0x4034, 0x3f);
	write_cmos_sensor(0x403d, 0x04);
	write_cmos_sensor(0x403e, 0x08);
	write_cmos_sensor(0x4040, 0x07);
	write_cmos_sensor(0x4041, 0xc6);
	write_cmos_sensor(0x4300, 0xff);
	write_cmos_sensor(0x4202, 0x00);
	write_cmos_sensor(0x4301, 0x00);
	write_cmos_sensor(0x4302, 0x0f);
	write_cmos_sensor(0x4316, 0x00);
	write_cmos_sensor(0x4500, 0x58);
	write_cmos_sensor(0x4503, 0x18);
	write_cmos_sensor(0x4600, 0x00);
	write_cmos_sensor(0x4601, 0xcb);
	write_cmos_sensor(0x470b, 0x28);
	
	//NO CONTIUS MODE ADD BY PENGTAO FAN 20140723
	//write_cmos_sensor(0x4800, 0x6C);
	//CONTIUS MODE ADD BY PENGTAO FAN 20140723
	//write_cmos_sensor(0x4800, 0x4C);
	write_cmos_sensor(0x4800, 0x4C);
	
	write_cmos_sensor(0x481f, 0x32);
	write_cmos_sensor(0x4837, 0x16);
	write_cmos_sensor(0x4850, 0x10);
	write_cmos_sensor(0x4851, 0x32);
	write_cmos_sensor(0x4b00, 0x2a);
	write_cmos_sensor(0x4b0d, 0x00);
	write_cmos_sensor(0x4d00, 0x04);
	write_cmos_sensor(0x4d01, 0x18);
	write_cmos_sensor(0x4d02, 0xc3);
	write_cmos_sensor(0x4d03, 0xff);
	write_cmos_sensor(0x4d04, 0xff);
	write_cmos_sensor(0x4d05, 0xff);
	//DPC off white test(0x5000, 0x78);
	//DPC on  white test(0x5000, 0x7e);
	write_cmos_sensor(0x5000, 0x7e);
	write_cmos_sensor(0x5001, 0x01);
	write_cmos_sensor(0x5002, 0x08);
	write_cmos_sensor(0x5003, 0x20);
	write_cmos_sensor(0x5046, 0x12);
	write_cmos_sensor(0x5780, 0x3e);
	write_cmos_sensor(0x5781, 0x0f);
	write_cmos_sensor(0x5782, 0x44);
	write_cmos_sensor(0x5783, 0x02);
	write_cmos_sensor(0x5784, 0x01);
	write_cmos_sensor(0x5785, 0x00);
	write_cmos_sensor(0x5786, 0x00);
	write_cmos_sensor(0x5787, 0x04);
	write_cmos_sensor(0x5788, 0x02);
	write_cmos_sensor(0x5789, 0x0f);
	write_cmos_sensor(0x578a, 0xfd);
	write_cmos_sensor(0x578b, 0xf5);
	write_cmos_sensor(0x578c, 0xf5);
	write_cmos_sensor(0x578d, 0x03);
	write_cmos_sensor(0x578e, 0x08);
	write_cmos_sensor(0x578f, 0x0c);
	write_cmos_sensor(0x5790, 0x08);
	write_cmos_sensor(0x5791, 0x04);
	write_cmos_sensor(0x5792, 0x00);
	write_cmos_sensor(0x5793, 0x52);
	write_cmos_sensor(0x5794, 0xa3);
	write_cmos_sensor(0x5871, 0x0d);
	write_cmos_sensor(0x5870, 0x18);
	write_cmos_sensor(0x586e, 0x10);
	write_cmos_sensor(0x586f, 0x08);
	write_cmos_sensor(0x58f8, 0x3d); //; LEN VSCALE FACTOR
	write_cmos_sensor(0x5901, 0x00);
	write_cmos_sensor(0x5b00, 0x02);
	write_cmos_sensor(0x5b01, 0x10);
	write_cmos_sensor(0x5b02, 0x03);
	write_cmos_sensor(0x5b03, 0xcf);
	write_cmos_sensor(0x5b05, 0x6c);
	write_cmos_sensor(0x5e00, 0x00);
	write_cmos_sensor(0x5e01, 0x41);
	write_cmos_sensor(0x382d, 0x7f);
	write_cmos_sensor(0x4825, 0x3a);
	write_cmos_sensor(0x4826, 0x40);
	write_cmos_sensor(0x4808, 0x25);
	if (ov8858version == OV8858R1A){
		LOG_INF("inter r1asensor_init() ov8858version = %d(0=r2a,1=r1a)\n",ov8858version); 
		r1asensor_init();
	}
	write_cmos_sensor(0x0100, 0x01);
}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E\n");
//   @@ Binning Size 1632x1224 30fps 4Lane_PCLK=74.4M
//   100 99 1632 1224
	write_cmos_sensor(0x100 , 0x00);      //	
	write_cmos_sensor(0x30e , 0x02);     //
	write_cmos_sensor(0x312 , 0x03);     //
	write_cmos_sensor(0x3015, 0x00);     //
	write_cmos_sensor(0x3700, 0x18);     //
	write_cmos_sensor(0x3701, 0x0c);     //
	write_cmos_sensor(0x3702, 0x28);     //
	write_cmos_sensor(0x3703, 0x19);     //
	write_cmos_sensor(0x3704, 0x14);     //
	write_cmos_sensor(0x3707, 0x04);     //
	write_cmos_sensor(0x3708, 0x24);     //
	write_cmos_sensor(0x3709, 0x33);     //
	write_cmos_sensor(0x370c, 0x04);     //
	write_cmos_sensor(0x3718, 0x12);     //
	write_cmos_sensor(0x3712, 0x42);     //
	write_cmos_sensor(0x371e, 0x19);     //
	write_cmos_sensor(0x371f, 0x40);     //
	write_cmos_sensor(0x3720, 0x05);     //
	write_cmos_sensor(0x3721, 0x05);     //
	write_cmos_sensor(0x3724, 0x06);     //
	write_cmos_sensor(0x3725, 0x01);     //
	write_cmos_sensor(0x3726, 0x06);     //
	write_cmos_sensor(0x3728, 0x05);     //
	write_cmos_sensor(0x3729, 0x02);     //
	write_cmos_sensor(0x372a, 0x03);     //
	write_cmos_sensor(0x372b, 0x53);     //
	write_cmos_sensor(0x372c, 0xa3);     //
	write_cmos_sensor(0x372d, 0x53);     //
	write_cmos_sensor(0x372e, 0x06);     //
	write_cmos_sensor(0x372f, 0x10);     //
	write_cmos_sensor(0x3730, 0x01);     //
	write_cmos_sensor(0x3731, 0x06);     //
	write_cmos_sensor(0x3732, 0x14);     //
	write_cmos_sensor(0x3736, 0x20);     //
	write_cmos_sensor(0x373a, 0x05);     //
	write_cmos_sensor(0x373b, 0x06);     //
	write_cmos_sensor(0x373c, 0x0a);     //
	write_cmos_sensor(0x373e, 0x03);     //
	write_cmos_sensor(0x375a, 0x06);     //
	write_cmos_sensor(0x375b, 0x13);     //
	write_cmos_sensor(0x375d, 0x02);     //
	write_cmos_sensor(0x375f, 0x14);     //
	write_cmos_sensor(0x3772, 0x23);     //
	write_cmos_sensor(0x3773, 0x02);     //
	write_cmos_sensor(0x3774, 0x16);     //
	write_cmos_sensor(0x3775, 0x12);     //
	write_cmos_sensor(0x3776, 0x04);     //
	write_cmos_sensor(0x3778, 0x17);     //
	write_cmos_sensor(0x37a0, 0x44);     //
	write_cmos_sensor(0x37a1, 0x3d);     //
	write_cmos_sensor(0x37a2, 0x3d);     //
	write_cmos_sensor(0x37a7, 0x44);     //
	write_cmos_sensor(0x37a8, 0x4c);     //
	write_cmos_sensor(0x37a9, 0x4c);     //
	write_cmos_sensor(0x37aa, 0x44);     //
	write_cmos_sensor(0x37ab, 0x2e);     //
	write_cmos_sensor(0x37ac, 0x2e);     //
	write_cmos_sensor(0x37ad, 0x33);     //
	write_cmos_sensor(0x37ae, 0x0d);     //
	write_cmos_sensor(0x37af, 0x0d);     //
	write_cmos_sensor(0x37b3, 0x42);     //
	write_cmos_sensor(0x37b4, 0x42);     //
	write_cmos_sensor(0x37b5, 0x31);     // ;;; 
	write_cmos_sensor(0x3768, 0x22);     //
	write_cmos_sensor(0x3769, 0x44);     //
	write_cmos_sensor(0x376a, 0x44);     //
	write_cmos_sensor(0x3808, 0x06);     //
	write_cmos_sensor(0x3809, 0x60);     //
	write_cmos_sensor(0x380a, 0x04);     //
	write_cmos_sensor(0x380b, 0xc8);     //
	write_cmos_sensor(0x380c, 0x07);     //
	write_cmos_sensor(0x380d, 0x88);     //
	write_cmos_sensor(0x380e, 0x05);     //;04
	write_cmos_sensor(0x380f, 0x04);     //;dc
	write_cmos_sensor(0x3814, 0x03);     //
	write_cmos_sensor(0x3821, 0x67);     //
	write_cmos_sensor(0x382a, 0x03);     //
	write_cmos_sensor(0x382b, 0x01);     //
	write_cmos_sensor(0x3830, 0x08);     //
	write_cmos_sensor(0x3836, 0x02);     //

	write_cmos_sensor(0x3843, 0x00);     //	
	// modify for pre cap view diff 20150105
	write_cmos_sensor(0x3811, 0x03);     //	
	write_cmos_sensor(0x3845, 0x02);     //	

	write_cmos_sensor(0x3f08, 0x08);     //
	write_cmos_sensor(0x4001, 0x96);     // 0x10
	write_cmos_sensor(0x4020, 0x00);     //
	write_cmos_sensor(0x4021, 0x04);     //
	write_cmos_sensor(0x4022, 0x04);     //
	write_cmos_sensor(0x4023, 0xb9);     //
	write_cmos_sensor(0x4024, 0x05);     //
	write_cmos_sensor(0x4025, 0x2a);     //
	write_cmos_sensor(0x4026, 0x05);     //
	write_cmos_sensor(0x4027, 0x2b);     //
	write_cmos_sensor(0x402a, 0x04);     //
	write_cmos_sensor(0x402b, 0x04);     //
	write_cmos_sensor(0x402e, 0x08);     //
	write_cmos_sensor(0x402f, 0x02);     //
	write_cmos_sensor(0x4600, 0x00);     //
	write_cmos_sensor(0x4601, 0xcb);     //
	write_cmos_sensor(0x5901, 0x00);     //
	write_cmos_sensor(0x382d, 0x7f);     // ;;;
	write_cmos_sensor(0x100 , 0x1);     //
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	if (currefps == 150) {
//  @@ Full size 3264x2448 30fps 4lane_PCLK=148.8M
//  100 99 3264 2448);    //
	write_cmos_sensor(0x100 , 0x0 );  //  
	write_cmos_sensor(0x30e , 0x00);  //  
	write_cmos_sensor(0x312 , 0x01);  //  
	write_cmos_sensor(0x3015, 0x01);  //  
	write_cmos_sensor(0x3700, 0x30);  //  
	write_cmos_sensor(0x3701, 0x18);  //  
	write_cmos_sensor(0x3702, 0x50);  //  
	write_cmos_sensor(0x3703, 0x32);  //  
	write_cmos_sensor(0x3704, 0x28);  //  
	write_cmos_sensor(0x3707, 0x08);  //  
	write_cmos_sensor(0x3708, 0x48);  //  
	write_cmos_sensor(0x3709, 0x66);  //  
	write_cmos_sensor(0x370c, 0x07);  //  
	write_cmos_sensor(0x3718, 0x14);  //  
	write_cmos_sensor(0x3712, 0x44);  //  
	write_cmos_sensor(0x371e, 0x31);  //  
	write_cmos_sensor(0x371f, 0x7f);  //  
	write_cmos_sensor(0x3720, 0x0a);  //  
	write_cmos_sensor(0x3721, 0x0a);  //  
	write_cmos_sensor(0x3724, 0x0c);  //  
	write_cmos_sensor(0x3725, 0x02);  //  
	write_cmos_sensor(0x3726, 0x0c);  //  
	write_cmos_sensor(0x3728, 0x0a);  //  
	write_cmos_sensor(0x3729, 0x03);  //  
	write_cmos_sensor(0x372a, 0x06);  //  
	write_cmos_sensor(0x372b, 0xa6);  //  
	write_cmos_sensor(0x372c, 0xa6);  //  
	write_cmos_sensor(0x372d, 0xa6);  //  
	write_cmos_sensor(0x372e, 0x0c);  //  
	write_cmos_sensor(0x372f, 0x20);  //  
	write_cmos_sensor(0x3730, 0x02);  //  
	write_cmos_sensor(0x3731, 0x0c);  //  
	write_cmos_sensor(0x3732, 0x28);  //  
	write_cmos_sensor(0x3736, 0x30);  //  
	write_cmos_sensor(0x373a, 0x0a);  //  
	write_cmos_sensor(0x373b, 0x0b);  //  
	write_cmos_sensor(0x373c, 0x14);  //  
	write_cmos_sensor(0x373e, 0x06);  //  
	write_cmos_sensor(0x375a, 0x0c);  //  
	write_cmos_sensor(0x375b, 0x26);  //  
	write_cmos_sensor(0x375d, 0x04);  //  
	write_cmos_sensor(0x375f, 0x28);  //  
	write_cmos_sensor(0x3772, 0x46);  //  
	write_cmos_sensor(0x3773, 0x04);  //  
	write_cmos_sensor(0x3774, 0x2c);  //  
	write_cmos_sensor(0x3775, 0x13);  //  
	write_cmos_sensor(0x3776, 0x08);  //  
	write_cmos_sensor(0x3778, 0x17);  //  
	write_cmos_sensor(0x37a0, 0x88);  //  
	write_cmos_sensor(0x37a1, 0x7a);  //  
	write_cmos_sensor(0x37a2, 0x7a);  //  
	write_cmos_sensor(0x37a7, 0x88);  //  
	write_cmos_sensor(0x37a8, 0x98);  //  
	write_cmos_sensor(0x37a9, 0x98);  //  
	write_cmos_sensor(0x37aa, 0x88);  //  
	write_cmos_sensor(0x37ab, 0x5c);  //  
	write_cmos_sensor(0x37ac, 0x5c);  //  
	write_cmos_sensor(0x37ad, 0x55);  //  
	write_cmos_sensor(0x37ae, 0x19);  //  
	write_cmos_sensor(0x37af, 0x19);  //  
	write_cmos_sensor(0x37b3, 0x84);  //  
	write_cmos_sensor(0x37b4, 0x84);  //  
	write_cmos_sensor(0x37b5, 0x60);  //   ; 66
	write_cmos_sensor(0x3768, 0x22);  //  
	write_cmos_sensor(0x3769, 0x44);  //  
	write_cmos_sensor(0x376a, 0x44);  //  
	write_cmos_sensor(0x3808, 0x0c);  //  
	write_cmos_sensor(0x3809, 0xc0);  //  
	write_cmos_sensor(0x380a, 0x09);  //  
	write_cmos_sensor(0x380b, 0x90);  //  
	write_cmos_sensor(0x380c, 0x09);  //  
	write_cmos_sensor(0x380d, 0x79);  //  
	write_cmos_sensor(0x380e, 0x0f);  //  
	write_cmos_sensor(0x380f, 0xfa);  //   ;aa
	write_cmos_sensor(0x3814, 0x01);  //  
	write_cmos_sensor(0x3821, 0x46);  //  
	write_cmos_sensor(0x382a, 0x01);  //  
	write_cmos_sensor(0x382b, 0x01);  //  
	write_cmos_sensor(0x3830, 0x06);  //  
	write_cmos_sensor(0x3836, 0x01);  //  
	write_cmos_sensor(0x3f08, 0x10);  //  

	write_cmos_sensor(0x3843, 0x00);     //	
	// modify for pre cap view diff 20150105
	write_cmos_sensor(0x3811, 0x05);     //	
	write_cmos_sensor(0x3845, 0x00);     //	

	
	write_cmos_sensor(0x4001, 0x86);  // 0x00
	write_cmos_sensor(0x4020, 0x00);  //  
	write_cmos_sensor(0x4021, 0x04);  //  
	write_cmos_sensor(0x4022, 0x0b);  //  
	write_cmos_sensor(0x4023, 0xc3);  //  
	write_cmos_sensor(0x4024, 0x0c);  //  
	write_cmos_sensor(0x4025, 0x36);  //  
	write_cmos_sensor(0x4026, 0x0c);  //  
	write_cmos_sensor(0x4027, 0x37);  //  
	write_cmos_sensor(0x402a, 0x04);  //  
	write_cmos_sensor(0x402b, 0x08);  //  
	write_cmos_sensor(0x402e, 0x0c);  //  
	write_cmos_sensor(0x402f, 0x02);  //  
	write_cmos_sensor(0x4600, 0x01);  //  
	write_cmos_sensor(0x4601, 0x97);  //  
	write_cmos_sensor(0x5901, 0x00);  //  
	write_cmos_sensor(0x382d, 0xff);  //   ;;;
	write_cmos_sensor(0x100 , 0x1);   // 
	}
	else 
	{   
//  @@ Full size 3264x2448 30fps 4lane_PCLK=148.8M
//  100 99 3264 2448);    //
	write_cmos_sensor(0x100 , 0x0 );  //  
	write_cmos_sensor(0x30e , 0x00);  //  
	write_cmos_sensor(0x312 , 0x01);  //  
	write_cmos_sensor(0x3015, 0x01);  //  
	write_cmos_sensor(0x3700, 0x30);  //  
	write_cmos_sensor(0x3701, 0x18);  //  
	write_cmos_sensor(0x3702, 0x50);  //  
	write_cmos_sensor(0x3703, 0x32);  //  
	write_cmos_sensor(0x3704, 0x28);  //  
	write_cmos_sensor(0x3707, 0x08);  //  
	write_cmos_sensor(0x3708, 0x48);  //  
	write_cmos_sensor(0x3709, 0x66);  //  
	write_cmos_sensor(0x370c, 0x07);  //  
	write_cmos_sensor(0x3718, 0x14);  //  
	write_cmos_sensor(0x3712, 0x44);  //  
	write_cmos_sensor(0x371e, 0x31);  //  
	write_cmos_sensor(0x371f, 0x7f);  //  
	write_cmos_sensor(0x3720, 0x0a);  //  
	write_cmos_sensor(0x3721, 0x0a);  //  
	write_cmos_sensor(0x3724, 0x0c);  //  
	write_cmos_sensor(0x3725, 0x02);  //  
	write_cmos_sensor(0x3726, 0x0c);  //  
	write_cmos_sensor(0x3728, 0x0a);  //  
	write_cmos_sensor(0x3729, 0x03);  //  
	write_cmos_sensor(0x372a, 0x06);  //  
	write_cmos_sensor(0x372b, 0xa6);  //  
	write_cmos_sensor(0x372c, 0xa6);  //  
	write_cmos_sensor(0x372d, 0xa6);  //  
	write_cmos_sensor(0x372e, 0x0c);  //  
	write_cmos_sensor(0x372f, 0x20);  //  
	write_cmos_sensor(0x3730, 0x02);  //  
	write_cmos_sensor(0x3731, 0x0c);  //  
	write_cmos_sensor(0x3732, 0x28);  //  
	write_cmos_sensor(0x3736, 0x30);  //  
	write_cmos_sensor(0x373a, 0x0a);  //  
	write_cmos_sensor(0x373b, 0x0b);  //  
	write_cmos_sensor(0x373c, 0x14);  //  
	write_cmos_sensor(0x373e, 0x06);  //  
	write_cmos_sensor(0x375a, 0x0c);  //  
	write_cmos_sensor(0x375b, 0x26);  //  
	write_cmos_sensor(0x375d, 0x04);  //  
	write_cmos_sensor(0x375f, 0x28);  //  
	write_cmos_sensor(0x3772, 0x46);  //  
	write_cmos_sensor(0x3773, 0x04);  //  
	write_cmos_sensor(0x3774, 0x2c);  //  
	write_cmos_sensor(0x3775, 0x13);  //  
	write_cmos_sensor(0x3776, 0x08);  //  
	write_cmos_sensor(0x3778, 0x17);  //  
	write_cmos_sensor(0x37a0, 0x88);  //  
	write_cmos_sensor(0x37a1, 0x7a);  //  
	write_cmos_sensor(0x37a2, 0x7a);  //  
	write_cmos_sensor(0x37a7, 0x88);  //  
	write_cmos_sensor(0x37a8, 0x98);  //  
	write_cmos_sensor(0x37a9, 0x98);  //  
	write_cmos_sensor(0x37aa, 0x88);  //  
	write_cmos_sensor(0x37ab, 0x5c);  //  
	write_cmos_sensor(0x37ac, 0x5c);  //  
	write_cmos_sensor(0x37ad, 0x55);  //  
	write_cmos_sensor(0x37ae, 0x19);  //  
	write_cmos_sensor(0x37af, 0x19);  //  
	write_cmos_sensor(0x37b3, 0x84);  //  
	write_cmos_sensor(0x37b4, 0x84);  //  
	write_cmos_sensor(0x37b5, 0x60);  //   ; 66
	write_cmos_sensor(0x3768, 0x22);  //  
	write_cmos_sensor(0x3769, 0x44);  //  
	write_cmos_sensor(0x376a, 0x44);  //  
	write_cmos_sensor(0x3808, 0x0c);  //  
	write_cmos_sensor(0x3809, 0xc0);  //  
	write_cmos_sensor(0x380a, 0x09);  //  
	write_cmos_sensor(0x380b, 0x90);  //  
	write_cmos_sensor(0x380c, 0x07);  //  
	write_cmos_sensor(0x380d, 0x94);  //  
	write_cmos_sensor(0x380e, 0x09);  //  
	write_cmos_sensor(0x380f, 0xfc);  //   ;aa
	write_cmos_sensor(0x3814, 0x01);  //  
	write_cmos_sensor(0x3821, 0x46);  //  
	write_cmos_sensor(0x382a, 0x01);  //  
	write_cmos_sensor(0x382b, 0x01);  //  
	write_cmos_sensor(0x3830, 0x06);  //  
	write_cmos_sensor(0x3836, 0x01);  //  
	// modify for pre cap view diff 20150105
	write_cmos_sensor(0x3811, 0x05);     //	
	write_cmos_sensor(0x3845, 0x00);     //	
	
	write_cmos_sensor(0x3f08, 0x10);  //  
	write_cmos_sensor(0x4001, 0x86);  // 0x00
	write_cmos_sensor(0x4020, 0x00);  //  
	write_cmos_sensor(0x4021, 0x04);  //  
	write_cmos_sensor(0x4022, 0x0b);  //  
	write_cmos_sensor(0x4023, 0xc3);  //  
	write_cmos_sensor(0x4024, 0x0c);  //  
	write_cmos_sensor(0x4025, 0x36);  //  
	write_cmos_sensor(0x4026, 0x0c);  //  
	write_cmos_sensor(0x4027, 0x37);  //  
	write_cmos_sensor(0x402a, 0x04);  //  
	write_cmos_sensor(0x402b, 0x08);  //  
	write_cmos_sensor(0x402e, 0x0c);  //  
	write_cmos_sensor(0x402f, 0x02);  //  
	write_cmos_sensor(0x4600, 0x01);  //  
	write_cmos_sensor(0x4601, 0x97);  //  
	write_cmos_sensor(0x5901, 0x00);  //  
	write_cmos_sensor(0x382d, 0xff);  //   ;;;
	write_cmos_sensor(0x100 , 0x1);   // 
	}
		
}
#if 0
static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
    capture_setting(currefps);
}
#endif
static void hs_video_setting(void)
{
	LOG_INF("E\n");
//  @@ OV8858R2A VGA 120fps_PCLK=148.8M
//  100 99 640 480
	write_cmos_sensor(0x0100, 0x00);  //
	write_cmos_sensor(0x30e , 0x00);  //
	write_cmos_sensor(0x312 , 0x01);  //
	write_cmos_sensor(0x3015, 0x01);  //
	write_cmos_sensor(0x3700, 0x30);  //
	write_cmos_sensor(0x3701, 0x18);  //
	write_cmos_sensor(0x3702, 0x50);  //
	write_cmos_sensor(0x3703, 0x32);  //
	write_cmos_sensor(0x3704, 0x28);  //
	write_cmos_sensor(0x3707, 0x08);  //
	write_cmos_sensor(0x3708, 0x48);  //
	write_cmos_sensor(0x3709, 0x66);  //
	write_cmos_sensor(0x370c, 0x07);  //
	write_cmos_sensor(0x3718, 0x14);  //
	write_cmos_sensor(0x3712, 0x44);  //
	write_cmos_sensor(0x371e, 0x31);  //
	write_cmos_sensor(0x371f, 0x7f);  //
	write_cmos_sensor(0x3720, 0x0a);  //
	write_cmos_sensor(0x3721, 0x0a);  //
	write_cmos_sensor(0x3724, 0x0c);  //
	write_cmos_sensor(0x3725, 0x02);  //
	write_cmos_sensor(0x3726, 0x0c);  //
	write_cmos_sensor(0x3728, 0x0a);  //
	write_cmos_sensor(0x3729, 0x03);  //
	write_cmos_sensor(0x372a, 0x06);  //
	write_cmos_sensor(0x372b, 0xa6);  //
	write_cmos_sensor(0x372c, 0xa6);  //
	write_cmos_sensor(0x372d, 0xa6);  //
	write_cmos_sensor(0x372e, 0x0c);  //
	write_cmos_sensor(0x372f, 0x20);  //
	write_cmos_sensor(0x3730, 0x02);  //
	write_cmos_sensor(0x3731, 0x0c);  //
	write_cmos_sensor(0x3732, 0x28);  //
	write_cmos_sensor(0x3736, 0x30);  //
	write_cmos_sensor(0x373a, 0x0a);  //
	write_cmos_sensor(0x373b, 0x0b);  //
	write_cmos_sensor(0x373c, 0x14);  //
	write_cmos_sensor(0x373e, 0x06);  //
	write_cmos_sensor(0x375a, 0x0c);  //
	write_cmos_sensor(0x375b, 0x26);  //
	write_cmos_sensor(0x375d, 0x04);  //
	write_cmos_sensor(0x375f, 0x28);  //
	write_cmos_sensor(0x3772, 0x46);  //
	write_cmos_sensor(0x3773, 0x04);  //
	write_cmos_sensor(0x3774, 0x2c);  //
	write_cmos_sensor(0x3775, 0x13);  //
	write_cmos_sensor(0x3776, 0x08);  //
	write_cmos_sensor(0x3778, 0x17);  //
	write_cmos_sensor(0x37a0, 0x88);  //
	write_cmos_sensor(0x37a1, 0x7a);  //
	write_cmos_sensor(0x37a2, 0x7a);  //
	write_cmos_sensor(0x37a7, 0x88);  //
	write_cmos_sensor(0x37a8, 0x98);  //
	write_cmos_sensor(0x37a9, 0x98);  //
	write_cmos_sensor(0x37aa, 0x88);  //
	write_cmos_sensor(0x37ab, 0x5c);  //
	write_cmos_sensor(0x37ac, 0x5c);  //
	write_cmos_sensor(0x37ad, 0x55);  //
	write_cmos_sensor(0x37ae, 0x19);  //
	write_cmos_sensor(0x37af, 0x19);  //
	write_cmos_sensor(0x37b3, 0x84);  //
	write_cmos_sensor(0x37b4, 0x84);  //
	write_cmos_sensor(0x37b5, 0x60);  //
	write_cmos_sensor(0x3768, 0x00);  //
	write_cmos_sensor(0x3769, 0xc0);  //
	write_cmos_sensor(0x376a, 0x42);  //
	write_cmos_sensor(0x3808, 0x02);  //
	write_cmos_sensor(0x3809, 0x80);  //
	write_cmos_sensor(0x380a, 0x01);  //
	write_cmos_sensor(0x380b, 0xe0);  //
	write_cmos_sensor(0x380c, 0x08);  //;09
	write_cmos_sensor(0x380d, 0xDe);  //;02
	write_cmos_sensor(0x380e, 0x02);  //
	write_cmos_sensor(0x380f, 0x22);  //;08
	write_cmos_sensor(0x3814, 0x03);  //
	write_cmos_sensor(0x3821, 0x6f);  //
	write_cmos_sensor(0x382a, 0x05);  //
	write_cmos_sensor(0x382b, 0x03);  //
	write_cmos_sensor(0x3830, 0x0c);  //
	write_cmos_sensor(0x3836, 0x02);  //
	write_cmos_sensor(0x3f08, 0x10);  //
	write_cmos_sensor(0x4001, 0x96);  // 0x10
	write_cmos_sensor(0x4020, 0x01);  //
	write_cmos_sensor(0x4021, 0x20);  //
	write_cmos_sensor(0x4022, 0x01);  //
	write_cmos_sensor(0x4023, 0x9f);  //
	write_cmos_sensor(0x4024, 0x03);  //
	write_cmos_sensor(0x4025, 0xe0);  //
	write_cmos_sensor(0x4026, 0x04);  //
	write_cmos_sensor(0x4027, 0x5f);  //
	write_cmos_sensor(0x402a, 0x02);  //
	write_cmos_sensor(0x402b, 0x04);  //
	write_cmos_sensor(0x402e, 0x02);  //
	write_cmos_sensor(0x402f, 0x04);  //
	write_cmos_sensor(0x4600, 0x00);  //
	write_cmos_sensor(0x4601, 0x4f);  //
	write_cmos_sensor(0x5901, 0x04);  //
	
	// modify for pre cap view diff 20150105
	write_cmos_sensor(0x3811, 0x03);     //	
	write_cmos_sensor(0x3845, 0x02);     //	

	write_cmos_sensor(0x382d, 0x7f);  //
	write_cmos_sensor(0x0100, 0x01);  //
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}


/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0x300B) << 8) | read_cmos_sensor(0x300C));
			if (*sensor_id == imgsensor_info.sensor_id) {
				if((read_cmos_sensor(0x302A)) == 0XB2){
 				ov8858version = OV8858R2A;
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x, ov8858version = %d(0=r2a,1=r1a)\n", imgsensor.i2c_write_id,*sensor_id,ov8858version);	
				return ERROR_NONE;
				}
				else if((read_cmos_sensor(0x302A)) == 0XB1){
 				ov8858version = OV8858R1A;
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x, ov8858version = %d(0=r2a,1=r1a)\n", imgsensor.i2c_write_id,*sensor_id,ov8858version);	
				return ERROR_NONE;
				}
				else{
				LOG_INF("read ov8858 R1A R2A bate fail\n");	  
				return ERROR_SENSOR_CONNECT_FAIL;
				}
			}	
			LOG_INF("Read sensor id fail,imgsensor.i2c_write_id: 0x%x, *sensor_id=0x%x\n", imgsensor.i2c_write_id,*sensor_id);
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

#ifdef OV8858R2AOTP

struct otp_struct {
int flag; // bit[7]: info, bit[6]:wb, bit[5]:vcm, bit[4]:lenc
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
int lenc[240];
int checksum;
int VCM_start;
int VCM_end;
int VCM_dir;
};

//static struct otp_struct *otp_ptr = (struct otp_struct *)kzalloc(sizeof(struct otp_struct), GFP_KERNEL);

#if 0
struct otp_struct otp_struct_obj = {
0,
0,
0,
0,
0,
0,
0,
0,
{0},
0,
0,
0,
0,
};


otp_ptr = &otp_struct_obj;
#endif

// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc/invalid otp lenc, 1 valid otp lenc
int read_otp(struct otp_struct *otp_ptr)
{
	int otp_flag, addr, temp, i;
	//set 0x5002[3] to “0”
	int temp1;
	temp1 = read_cmos_sensor(0x5002);
	write_cmos_sensor(0x5002, (0x00 & 0x08) | (temp1 & (~0x08)));
	// read OTP into buffer
	write_cmos_sensor(0x3d84, 0xC0);
	write_cmos_sensor(0x3d88, 0x70); // OTP start address
	write_cmos_sensor(0x3d89, 0x10);
	write_cmos_sensor(0x3d8A, 0x72); // OTP end address
	write_cmos_sensor(0x3d8B, 0x0a);
	write_cmos_sensor(0x3d81, 0x01); // load otp into buffer
	mdelay(10);
	// OTP base information and WB calibration data
	otp_flag = read_cmos_sensor(0x7010);
	addr = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x7011; // base address of info group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x7019; // base address of info group 2
	}
	if(addr != 0) {
		(*otp_ptr).flag = 0xC0; // valid info and AWB in OTP
		(*otp_ptr).module_integrator_id = read_cmos_sensor(addr);
		(*otp_ptr).lens_id = read_cmos_sensor( addr + 1);
		(*otp_ptr).production_year = read_cmos_sensor( addr + 2);
		(*otp_ptr).production_month = read_cmos_sensor( addr + 3);
		(*otp_ptr).production_day = read_cmos_sensor(addr + 4);
		temp = read_cmos_sensor(addr + 7);
		(*otp_ptr).rg_ratio = (read_cmos_sensor(addr + 5)<<2) + ((temp>>6) & 0x03);
		(*otp_ptr).bg_ratio = (read_cmos_sensor(addr + 6)<<2) + ((temp>>4) & 0x03);
	}
	else {
		(*otp_ptr).flag = 0x00; // not info and AWB in OTP
		(*otp_ptr).module_integrator_id = 0;
		(*otp_ptr).lens_id = 0;
		(*otp_ptr).production_year = 0;
		(*otp_ptr).production_month = 0;
		(*otp_ptr).production_day = 0;
		(*otp_ptr).rg_ratio = 0;
		(*otp_ptr).bg_ratio = 0;
	}
	// OTP VCM Calibration
	otp_flag = read_cmos_sensor(0x7021);
	addr = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x7022; // base address of VCM Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x7025; // base address of VCM Calibration group 2
	}
	if(addr != 0) {
		(*otp_ptr).flag |= 0x20;
		temp = read_cmos_sensor(addr + 2);
		(* otp_ptr).VCM_start = (read_cmos_sensor(addr)<<2) | ((temp>>6) & 0x03);
		(* otp_ptr).VCM_end = (read_cmos_sensor(addr + 1) << 2) | ((temp>>4) & 0x03);
		(* otp_ptr).VCM_dir = (temp>>2) & 0x03;
	}
	else {
		(* otp_ptr).VCM_start = 0;
		(* otp_ptr).VCM_end = 0;
		(* otp_ptr).VCM_dir = 0;
	}
	// OTP Lenc Calibration
	otp_flag = read_cmos_sensor(0x7028);
	addr = 0;
	int checksum2=0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x7029; // base address of Lenc Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x711a; // base address of Lenc Calibration group 2
	}
	if(addr != 0) {
		for(i=0;i<240;i++) {
			(* otp_ptr).lenc[i]=read_cmos_sensor(addr + i);
			checksum2 += (* otp_ptr).lenc[i];
		}
		checksum2 = (checksum2)%255 +1;
		(*otp_ptr).checksum = read_cmos_sensor(addr + 240);
		if((*otp_ptr).checksum == checksum2){
			(*otp_ptr).flag |= 0x10;	
		}
	}
	else {
		for(i=0;i<240;i++) {
			(* otp_ptr).lenc[i]=0;
		}
	}
	for(i=0x7010;i<=0x720a;i++) {
		write_cmos_sensor(i,0); // clear OTP buffer, recommended use continuous write to accelarate
	}
	//set 0x5002[3] to “1”
	temp1 = read_cmos_sensor(0x5002);
	write_cmos_sensor(0x5002, (0x08 & 0x08) | (temp1 & (~0x08)));
	return (*otp_ptr).flag;
}
// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc

int apply_otp(struct otp_struct *otp_ptr)
{
	int RG_Ratio_Typical = 0x148, BG_Ratio_Typical = 0x12A;
	int rg, bg, R_gain, G_gain, B_gain, Base_gain, temp, i;
	// apply OTP WB Calibration
	if ((*otp_ptr).flag & 0x40) {
		rg = (*otp_ptr).rg_ratio;
		bg = (*otp_ptr).bg_ratio;
		//calculate G gain
		R_gain = (RG_Ratio_Typical*1000) / rg;
		B_gain = (BG_Ratio_Typical*1000) / bg;
		G_gain = 1000;
		if (R_gain < 1000 || B_gain < 1000)
		{
			if (R_gain < B_gain)
				Base_gain = R_gain;
			else
				Base_gain = B_gain;
		}
		else
		{
			Base_gain = G_gain;
		}
		R_gain = 0x400 * R_gain / (Base_gain);
		B_gain = 0x400 * B_gain / (Base_gain);
		G_gain = 0x400 * G_gain / (Base_gain);
		// update sensor WB gain
		if (R_gain>0x400) {
			write_cmos_sensor(0x5032, R_gain>>8);
			write_cmos_sensor(0x5033, R_gain & 0x00ff);
		}
		if (G_gain>0x400) {
			write_cmos_sensor(0x5034, G_gain>>8);
			write_cmos_sensor(0x5035, G_gain & 0x00ff);
		}
		if (B_gain>0x400) {
			write_cmos_sensor(0x5036, B_gain>>8);
			write_cmos_sensor(0x5037, B_gain & 0x00ff);
		}
	}
	// apply OTP Lenc Calibration
	if ((*otp_ptr).flag & 0x10) {
		temp = read_cmos_sensor(0x5000);
		temp = 0x80 | temp;
		write_cmos_sensor(0x5000, temp);
		for(i=0;i<240;i++) {
			write_cmos_sensor(0x5800 + i, (*otp_ptr).lenc[i]);
		}
	}
	return (*otp_ptr).flag;
}

#endif
/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 
	LOG_INF("PLATFORM:MT6595,MIPI 4LANE\n");
	//LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor(0x300B) << 8) | read_cmos_sensor(0x300C));
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, i2c write id: 0x%x, sensor id: 0x%x\n\n", imgsensor.i2c_write_id,sensor_id);
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
	sensor_init();
	
	mdelay(10);
	#ifdef OV8858R2AOTP
		LOG_INF("Apply the sensor OTP\n");
		struct otp_struct *otp_ptr = (struct otp_struct *)kzalloc(sizeof(struct otp_struct), GFP_KERNEL);
		read_otp(otp_ptr);
		apply_otp(otp_ptr);
		kfree(otp_ptr);
	#endif
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
	imgsensor.gain = 0x100;
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

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
    set_mirror_flip(imgsensor.mirror); 
    //set_mirror_flip(IMAGE_HV_MIRROR);
	mdelay(10);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
    set_mirror_flip(imgsensor.mirror); 
    //set_mirror_flip(IMAGE_HV_MIRROR);
	mdelay(10);

	if(imgsensor.test_pattern == KAL_TRUE)
	{
		write_cmos_sensor(0x5002,0x00);
  }

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror); 
    //set_mirror_flip(IMAGE_HV_MIRROR);
	mdelay(10);
	
	
	return ERROR_NONE;
}	/*	normal_video   */

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
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	mdelay(10);
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	mdelay(10);
	
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

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
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
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
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
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
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
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
			set_dummy();			
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
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();	
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
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x5E00, 0x80);
		write_cmos_sensor(0x5002, 0x00);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x5E00, 0x00);
		write_cmos_sensor(0x5002, 0x08);
	}	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
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
    /* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            if( (imgsensor_info.ae_shut_delay_frame ==0) && (imgsensor_info.ae_sensor_gain_delay_frame==0) )
            {
                syncShutter = (unsigned short)*feature_data;
                LOG_INF("SENSOR_FEATURE_SET_ESHUTTER syncShutter = %d\n",syncShutter);
            }
            else
            {
                set_shutter(*feature_data);
            }
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            if( (imgsensor_info.ae_shut_delay_frame ==0) && (imgsensor_info.ae_sensor_gain_delay_frame==0) )
            {
            	set_shutter(syncShutter);
            }
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
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
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        //LGE_CHANGE_S: [2015-12-22] yonghwan.lym@lge.com, Get FOV from sensor for CTS
        case SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:
            {
                uintptr_t *pHorFOV = (uintptr_t)(*(feature_data));
                uintptr_t *pVerFOV = (uintptr_t)(*(feature_data+1));

                LOG_INF("SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:\n");
                *pHorFOV = (0x3B)<<16 | (0x3B); // 16:9 = 59, 4:3 = 59 // (59)<<16 | (59)
                *pVerFOV = (0x31)<<16 | (0x31); // 16:9 = 49, 4:3 = 49 // (49)<<16 | (49)
                *feature_para_len = sizeof(uintptr_t) * 2;
                break;
            }
		 break;
        //LGE_CHANGE_E: [2015-12-22] yonghwan.lym@lge.com, Get FOV from sensor for CTS
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */


static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 OV8858_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/

UINT32 OV8858R2A_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/


