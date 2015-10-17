//20141222
#ifndef _ADNS3080_H__
#define _ADNS3080_H__	 	

#include "Project_cfg.h"
#include "stm32f10x.h"


#if OPFLOW_SENSOR_ONBOARD_SPI_EN

	//#define ADNS3080_POWER_DOWN_CTL				1
	#define ADNS3080_USE_SPI1						0						
	#define ADNS3080_USE_SPI2						1

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//ADNS3080¸´Î»¶Ë
	#define ADNS3080_RST_PORT              GPIOB
	#define ADNS3080_RST_CLK               RCC_APB2Periph_GPIOB  
	#define ADNS3080_RST_PIN               GPIO_Pin_3
	#define Set_ADNS3080_RST  {GPIO_SetBits(ADNS3080_RST_PORT,ADNS3080_RST_PIN);}
	#define Clr_ADNS3080_RST  {GPIO_ResetBits(ADNS3080_RST_PORT,ADNS3080_RST_PIN);}
	//ADNS3080Æ¬Ñ¡ÐÅºÅ	 
	#define ADNS3080_NCS_PORT              GPIOB
	#define ADNS3080_NCS_CLK               RCC_APB2Periph_GPIOB 
	#define ADNS3080_NCS_PIN               GPIO_Pin_12
	#define Set_ADNS3080_NCS  {GPIO_SetBits(ADNS3080_NCS_PORT,ADNS3080_NCS_PIN);}
	#define Clr_ADNS3080_NCS  {GPIO_ResetBits(ADNS3080_NCS_PORT,ADNS3080_NCS_PIN);}  

	//ADNS3080 powerdown mode
	#define ADNS3080_NPD_PORT              GPIOA
	#define ADNS3080_NPD_CLK               RCC_APB2Periph_GPIOA  
	#define ADNS3080_NPD_PIN               GPIO_Pin_2
	#define Set_ADNS3080_NPD  {GPIO_SetBits(ADNS3080_NPD_PORT,ADNS3080_NPD_PIN);}
	#define Clr_ADNS3080_NPD  {GPIO_ResetBits(ADNS3080_NPD_PORT,ADNS3080_NPD_PIN);}	
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
	
	// orientations for ADNS3080 sensor
	//#define AP_OPTICALFLOW_ADNS3080_PINS_FORWARD ROTATION_YAW_180
	//#define AP_OPTICALFLOW_ADNS3080_PINS_FORWARD_RIGHT ROTATION_YAW_135
	//#define AP_OPTICALFLOW_ADNS3080_PINS_RIGHT ROTATION_YAW_90
	//#define AP_OPTICALFLOW_ADNS3080_PINS_BACK_RIGHT ROTATION_YAW_45
	//#define AP_OPTICALFLOW_ADNS3080_PINS_BACK ROTATION_NONE
	//#define AP_OPTICALFLOW_ADNS3080_PINS_BACK_LEFT ROTATION_YAW_315
	//#define AP_OPTICALFLOW_ADNS3080_PINS_LEFT ROTATION_YAW_270
	//#define AP_OPTICALFLOW_ADNS3080_PINS_FORWARD_LEFT ROTATION_YAW_225

	// field of view of ADNS3080 sensor lenses
	#define AP_OPTICALFLOW_ADNS3080_08_FOV 		0.202458  //radians// 11.6 degrees

	// scaler - value returned when sensor is moved equivalent of 1 pixel
	#define AP_OPTICALFLOW_ADNS3080_SCALER    2.1//0.53


	// ADNS3080 hardware config
	#define ADNS3080_PIXELS_X                 30
	#define ADNS3080_PIXELS_Y                 30
	#define ADNS3080_CLOCK_SPEED              24000000

#if OPFLOW_SENSOR_ONBOARD_SPI_EN

	// Register Map for the ADNS3080 Optical OpticalFlow Sensor
	#define ADNS3080_PRODUCT_ID            0x00
	#define ADNS3080_REVISION_ID           0x01
	#define ADNS3080_MOTION                0x02
	#define ADNS3080_DELTA_X               0x03
	#define ADNS3080_DELTA_Y               0x04
	#define ADNS3080_SQUAL                 0x05
	#define ADNS3080_PIXEL_SUM             0x06
	#define ADNS3080_MAXIMUM_PIXEL         0x07
	#define ADNS3080_CONFIGURATION_BITS    0x0a
	#define ADNS3080_EXTENDED_CONFIG       0x0b
	#define ADNS3080_DATA_OUT_LOWER        0x0c
	#define ADNS3080_DATA_OUT_UPPER        0x0d
	#define ADNS3080_SHUTTER_LOWER         0x0e
	#define ADNS3080_SHUTTER_UPPER         0x0f
	#define ADNS3080_FRAME_PERIOD_LOWER    0x10
	#define ADNS3080_FRAME_PERIOD_UPPER    0x11
	#define ADNS3080_MOTION_CLEAR          0x12
	#define ADNS3080_FRAME_CAPTURE         0x13
	#define ADNS3080_SROM_ENABLE           0x14
	#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
	#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
	#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
	#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
	#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1d
	#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
	#define ADNS3080_SROM_ID               	0x1f
	#define ADNS3080_OBSERVATION           	0x3d
	#define ADNS3080_INVERSE_PRODUCT_ID    	0x3f
	#define ADNS3080_PIXEL_BURST           	0x40
	#define ADNS3080_MOTION_BURST          	0x50
	#define ADNS3080_SROM_LOAD             	0x60

	// Configuration Bits
	#define ADNS3080_LED_MODE_ALWAYS_ON        0x00
	#define ADNS3080_LED_MODE_WHEN_REQUIRED    0x01

	#define ADNS3080_RESOLUTION_400        	400
	#define ADNS3080_RESOLUTION_1600       	600

	// Extended Configuration bits
	#define ADNS3080_SERIALNPU_OFF  		0x02

	#define ADNS3080_FRAME_RATE_MAX         6469
	#define ADNS3080_FRAME_RATE_MIN         2000


	union NumericIntType
	{
		int16_t intValue;
		uint16_t uintValue;
		uint8_t byteValue[2];
	};

	bool ADNS3080_DeviceInit(void);
	void ADNS3080_SPIInit(void);
	void ADNS3080_Print_Pixel_Data(void);
	void ADNS3080_Update(void);

#endif //#if OPFLOW_SENSOR_ONBOARD_SPI_EN
	extern int8_t motion_reg,raw_dx,raw_dy,g_A3080_dx,g_A3080_dy,_overflow;
	extern uint8_t surface_quality;

#endif


