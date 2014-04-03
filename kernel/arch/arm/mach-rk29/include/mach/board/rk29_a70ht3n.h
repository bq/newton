/*
 * rk29_a80htn.h
 *
 * Overview:  
 *
 * Copyright (c) 2011, YiFang Digital
 *
 * Version:  1.0
 * Created:  12/28/2011 11:20:11 AM
 * Author:  zqqu <zqqu@yifangdigital.com>
 * Company:  YiFang Digital
 * History:
 *
 * 
 */


#ifndef __RK29_A70H3N_H
#define __RK29_A70H3N_H
#define NO_IOMUX_PINNAME  NULL
#define NO_IO_MUX_MODE		NULL

/***************************************************
 *
 *                      CPU FREQ
 *
 **************************************************/
#define CPU_FREQ_TABLE {\
	{ .index = 1125000, .frequency =  408000 },\
	{ .index = 1125000, .frequency =  816000 },\
	{ .index = 1225000, .frequency = 1008000 },\
	{ .frequency = CPUFREQ_TABLE_END },\
}

/***************************************************
 *
 *                      KEY
 *
 **************************************************/
#define GPIO_POWER_KEY				RK29_PIN6_PA7

#define PRESS_LEV_LOW			1
#define PRESS_LEV_HIGH			0

#define KEYS_MAP	{\
	{	\
		.desc	= "play",	\
		.code	= KEY_POWER,	\
		.gpio	= GPIO_POWER_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
		.wakeup	= 1,	\
	},	\
}

/**********************************************************************************************
 *
 *										LCD
 *
 *********************************************************************************************/
#define LCD_TXD_PIN          INVALID_GPIO
#define LCD_CLK_PIN          INVALID_GPIO
#define LCD_CS_PIN           INVALID_GPIO

#define FB_ID                       0
#define FB_DISPLAY_ON_PIN           RK29_PIN6_PD1
#define FB_LCD_STANDBY_PIN          RK29_PIN6_PD0
#define FB_LCD_CABC_EN_PIN          INVALID_GPIO
#define FB_MCU_FMK_PIN              INVALID_GPIO

#define FB_DISPLAY_ON_VALUE         GPIO_HIGH
#define FB_LCD_STANDBY_VALUE        GPIO_HIGH

#ifdef CONFIG_MACH_A7_COMMON
#define OUT_CLK			 32000000
#define LCDC_ACLK       400000000   

/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888

/* Timing */
#define H_PW			48
#define H_BP			40
#define H_VD			800
#define H_FP			62

#define V_PW			3
#define V_BP			29
#define V_VD			480
#define V_FP			48

/* Other */
#define DCLK_POL		0
#define SWAP_RB			0

#define LCD_WIDTH       154
#define LCD_HEIGHT      87
#elif defined CONFIG_MACH_YF70CK29
#define OUT_CLK			 50000000
#define LCDC_ACLK       400000000   

/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888

/* Timing */
#define H_PW			1
#define H_BP			159
#define H_VD			1024
#define H_FP			160

#define V_PW			3
#define V_BP			20
#define V_VD			600
#define V_FP			12

/* Other */
#define DCLK_POL		0
#define SWAP_RB			0

#define LCD_WIDTH       154
#define LCD_HEIGHT      87
#endif

/**********************************************************************************************
 *
 *										BACKLIGHT
 *
 *********************************************************************************************/
#define PWM_ID				0
#define PWM_MUX_NAME		GPIO1B5_PWM0_NAME
#define PWM_MUX_MODE		GPIO1L_PWM0
#define PWM_MUX_MODE_GPIO	GPIO1L_GPIO1B5
#define PWM_GPIO			RK29_PIN1_PB5
#define PWM_EFFECT_VALUE	0

/**the value of MIN_BACKLIGHT_SCALE must be between 0~10*/
#define MIN_BACKLIGHT_SCALE	11
#define BACKLIGHT_REDUCE_VALUE 24

/**********************************************************************************************
 *
 *							PWM VOLTAGE REGULATOR
 *
 *********************************************************************************************/
#define REGULATOR_PWM_ID					2
#define REGULATOR_PWM_MUX_NAME      		GPIO2A3_SDMMC0WRITEPRT_PWM2_NAME
#define REGULATOR_PWM_MUX_MODE				GPIO2L_PWM2
#define REGULATOR_PWM_MUX_MODE_GPIO			GPIO2L_GPIO2A3
#define REGULATOR_PWM_GPIO					RK29_PIN2_PA3



/**********************************************************************************************
 *
 *										TOUCH PANEL
 *
 *********************************************************************************************/
#define TOUCH_POWER_PIN			RK29_PIN6_PB0
#define TOUCH_RESET_PIN			RK29_PIN6_PC3
#define TOUCH_INT_PIN			RK29_PIN0_PA2
#define TOUCH_USE_I2C2			1
//#define TOUCH_KEY_LED			RK29_PIN6_PA6
//#define TOUCHKEY_ON_SCREEN
#define TP_USE_WAKEUP_PIN 1
#define TOUCH_EN_LEVEL			GPIO_HIGH

/**********************************************************************************************
 *
 *										RTC
 *
 *********************************************************************************************/
#define GPIO_RTC_INT			 RK29_PIN0_PA1

/***************************************************
 *
 *                      AUDIO
 *
 **************************************************/
#define GPIO_SPK_CON			 RK29_PIN6_PB6
#define RT5631_DEF_VOL					0xc0
#define RT5631_DEF_VOL_SPK				0xca
#define DEF_HP_EQ				NORMAL
#define RT5631_ENABLE_ALC_DAC 0
/**********************************************************************************************
 *
 *										GSENSOR
 *
 *********************************************************************************************/
#define MMA8452_INT_PIN   RK29_PIN0_PA3


/**********************************************************************************************
 *
 *										USB
 *
 *********************************************************************************************/
#define GPIO_USB_INT				RK29_PIN0_PA0
#define MASS_STORAGE_NAME			"Arnova"
#define MASS_STORAGE_PRODUCT		"7bG2 DT"
#define USB_PRODUCT_ID				0x1440
#define ADB_PRODUCT_ID				0x1441
#define VENDOR_ID					0x0E79
#define ADB_PRODUCT_NAME			"Arnova"
#define ADB_MANUFACTURE_NAME		"7bG2 DT"

/**********************************************************************************************
 *
 *										HDMI
 *
 *********************************************************************************************/
#define ANX7150_ATTACHED_BUS1
#define GPIO_HDMI_DET			 RK29_PIN1_PD7
#define	GPIO_ANX7150_RST		INVALID_GPIO
#define ANX7150_RST_MUX_NAME	GPIO2C7_SPI1RXD_NAME
#define ANX7150_RST_MUX_MODE	GPIO2H_GPIO2C7


/**********************************************************************************************
 *
 *										SDMMC
 *
 *********************************************************************************************/
#define SDMMC_POWER_PIN		RK29_PIN5_PD5
#define SDMMC_DET_PIN		RK29_PIN2_PA2


/**********************************************************************************************
 *
 *									WIFI/BT	
 *
 *********************************************************************************************/
#define GPIO_WIFI_POWER       RK29_PIN6_PB3


/**********************************************************************************************
 *
 *									PMU	
 *
 *********************************************************************************************/
#define TPS65910_HOST_IRQ		RK29_PIN4_PD0

/**********************************************************************************************
 *
 *									BATTERY
 *
 *********************************************************************************************/
#define DC_DET_EFFECTIVE		0
#define CHG_OK_EFFECTIVE		1
#define GPIO_DC_DET				RK29_PIN0_PA0
#define GPIO_CHG_OK				RK29_PIN4_PA3
#define ADC_CLI_VALUE			25
#define ADC_ADD_VALUE			1
#define CHARGE_FULL_GATE 		4150
//#define DC_SW_PIN				RK29_PIN6_PB4
//#define DC_ENABLE_LEVEL			GPIO_HIGH


//This parameter is for new battery driver//
#define	DC_CURRENT_IN_TWO_MODE          //This macro to distinguish some model such as A7/A8  will work in two charging current mode
#define GPIO_CURRENT_CONTROL		RK29_PIN6_PB4
#define GPIO_CURRENT_CONTROL_LEVEL	GPIO_HIGH /* huge current charge control pin level */

#define DC_DET_WITH_USB_INT
#define	TIMER_MS_COUNTS		            50	//timers length(ms)

#define	SLOPE_SECOND_COUNTS	            15	//time interval(s) for computing voltage slope
#define	DISCHARGE_MIN_SECOND	        60	//minimum time interval for discharging 1% battery
#define	CHARGE_MIN_SECOND	            100	//minimum time interval for charging 1% battery
#define	CHARGE_MID_SECOND	            130	//time interval for charging 1% battery when battery capacity over 80%
#define	CHARGE_MAX_SECOND	            240 //max time interval for charging 1% battery
#define CHARGE_FULL_DELAY_TIMES         10  //delay time when charging FULL
#define USBCHARGE_IDENTIFY_TIMES        5   //time for identifying USB and Charge
#define STABLE_SECOND					8  
#define SHUTDOWN_SECOND					20
#define SPEEDLOSE_SECOND                120 //play game rapid down

#define	NUM_VOLTAGE_SAMPLE	            ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//samling numbers
#define	NUM_DISCHARGE_MIN_SAMPLE	    ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	
#define	NUM_CHARGE_MIN_SAMPLE	        ((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MID_SAMPLE	        (((CHARGE_MID_SECOND + 90) * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MAX_SAMPLE	        (((CHARGE_MAX_SECOND + 90)* 1000) / TIMER_MS_COUNTS)	    
#define NUM_CHARGE_FULL_DELAY_TIMES     ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_USBCHARGE_IDENTIFY_TIMES    ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)
#define NUM_STABLE_SAMPLE				((STABLE_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SHUTD0WN_SAMPLE             ((SHUTDOWN_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SPEEDLOSE_SAMPLE  			((SPEEDLOSE_SECOND * 1000) / TIMER_MS_COUNTS)

#define BAT_2V5_VALUE	        2500
#define BATT_MAX_VOL_VALUE	    4190	//voltage of FULL battery
#define	BATT_ZERO_VOL_VALUE     3500	//voltage when poweroff
#define BATT_NOMAL_VOL_VALUE    3800
#define SHUTDOWNVOLTAGE			3400

//define  divider resistors for ADC sampling, units as K
#define BAT_PULL_UP_R           200
#define BAT_PULL_DOWN_R         200

#define RK29_USB_CHARGE_SUPPORT
#define USB_PULLUP_PIN		RK29_PIN5_PA2

//Battery table list definition
#define BAT_RAW_TABLE_BAT { \
	3500, 3623, 3679, 3706, 3734, 3769, 3816, 3874, 3938, 3993, 4060 \
}
#define BAT_RAW_TABLE_AC { \
	3670, 3785, 3835, 3857, 3884, 3918, 3965, 4022, 4083, 4150, 4200 \
}
#define BAT_RAW_TABLE_USB { \
	3560, 3678, 3732, 3757, 3784, 3819, 3866, 3923, 3985, 4040, 4100 \
}


/***************************************************
 *
 *                  CAMERA SENSOR
 *
 **************************************************/
#define CONFIG_SENSOR_0			RK29_CAM_SENSOR_BCAM		/* back camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_0	0xef

#define CONFIG_SENSOR_IIC_ADAPTER_ID_0    1
#define CONFIG_SENSOR_ORIENTATION_0       90
#define CONFIG_SENSOR_POWER_PIN_0         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_0         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_0       RK29_PIN6_PB7
#define CONFIG_SENSOR_FALSH_PIN_0         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_0 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_0 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_0 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_0 RK29_CAM_FLASHACTIVE_L
#define OV5642_BP_REGULATOR	0x0b	










//---------------------------------------------------------------------------
#define CONFIG_SENSOR_1			RK29_CAM_SENSOR_FCAM/* front camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_1				0xff

#define CONFIG_SENSOR_IIC_ADAPTER_ID_1    1
#define CONFIG_SENSOR_ORIENTATION_1       270
#define CONFIG_SENSOR_POWER_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_1       RK29_PIN5_PD7
#define CONFIG_SENSOR_FALSH_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_1 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_1 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_1 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_1 RK29_CAM_FLASHACTIVE_L

#define NT99250_MIRROR	0
#define NT99250_FLIP	0
#endif

