/*
 * rk29_m900hc.h
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


#ifndef __RK29_A8HC_H
#define __RK29_A8HC_H
#define NO_IOMUX_PINNAME  NULL
#define NO_IO_MUX_MODE		NULL

/***************************************************
 *
 *                      CPU FREQ
 *
 **************************************************/
#define CPU_FREQ_TABLE {\
	{ .index = 1150000, .frequency =  408000 },\
	{ .index = 1200000, .frequency =  816000 },\
	{ .index = 1250000, .frequency = 1008000 },\
	{ .frequency = CPUFREQ_TABLE_END },\
}

/***************************************************
 *
 *                      KEY
 *
 **************************************************/
#define GPIO_MENU_KEY
#define GPIO_VOLUMEUP_KEY			RK29_PIN6_PA1
#define GPIO_VOLUMEDOWN_KEY			RK29_PIN6_PA2
#define GPIO_HOME_KEY
#define GPIO_SEARCH_KEY
#define GPIO_BACK_KEY
#define GPIO_CAMERA_KEY
#define GPIO_POWER_KEY				RK29_PIN6_PA7

#define PRESS_LEV_LOW			1
#define PRESS_LEV_HIGH			0

#define KEYS_MAP	{\
	{ \
		.desc		= "menu", \
		.code		= KEY_F1, \
		.adc_value	= 400, \
		.gpio		= INVALID_GPIO \
	}, \
	{ \
		.desc		= "vol+", \
		.code		= KEY_VOLUMEUP, \
		.adc_value	= 735, \
		.gpio		= INVALID_GPIO \
	}, \
	{ \
		.desc		= "vol-", \
		.code		= KEY_VOLUMEDOWN, \
		.adc_value	= 615, \
		.gpio		= INVALID_GPIO \
	}, \
	{ \
		.desc		= "home", \
		.code		= KEY_HOME, \
		.adc_value	= 935, \
		.gpio		= INVALID_GPIO \
	}, \
	{ \
		.desc		= "esc", \
		.code		= KEY_BACK, \
		.adc_value	= 111, \
		.gpio		= INVALID_GPIO \
	}, \
	{ \
		.desc	= "search", \
		.code	= KEY_SEARCH, \
		.adc_value	= 816, \
		.gpio	= INVALID_GPIO \
	}, \
	{ \
		.desc	= "play", \
		.code	= KEY_POWER, \
		.gpio	= RK29_PIN6_PA7, \
		.active_low = PRESS_LEV_LOW, \
		.wakeup	= 1, \
	}, \
}

#define ADC_KEY_CHN		1
/**********************************************************************************************
 *
 *										LCD
 *
 *********************************************************************************************/
#define	LCD_SELECT_PIN			RK29_PIN4_PD0
#define LCD_SELECT_MUX_NAME		NO_IOMUX_PINNAME
#define LCD_SELECT_MUX_MODE		NO_IO_MUX_MODE
#define LCD_TXD_PIN          INVALID_GPIO
#define LCD_CLK_PIN          INVALID_GPIO
#define LCD_CS_PIN           INVALID_GPIO

#define FB_ID                       0
#define FB_DISPLAY_ON_PIN           RK29_PIN6_PD1
#define FB_LCD_STANDBY_PIN          INVALID_GPIO
#define FB_LCD_CABC_EN_PIN          INVALID_GPIO
#define FB_MCU_FMK_PIN              INVALID_GPIO
#define BACKLIGHT_CONTROL_PIN       RK29_PIN6_PD0

#define FB_DISPLAY_ON_VALUE         GPIO_HIGH
#define FB_LCD_STANDBY_VALUE        GPIO_HIGH
#define BACKLIGHT_CONTROL_VALUE     GPIO_HIGH



#define OUT_CLK			 42000000
#define LCDC_ACLK       400000000   

/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888

/* Timing */
#define H_PW			48
#define H_BP			40
#define H_VD			800
#define H_FP			200//112

#define V_PW			2
#define V_BP			10//39
#define V_VD			600
#define V_FP			18

#define DCLK_POL		1
#define SWAP_RB			1

#define LCD_WIDTH       202
#define LCD_HEIGHT      152

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
/*
#define BL_EN_MUX_NAME		GPIOF34_UART3_SEL_NAME
#define BL_EN_MUX_MODE		IOMUXB_GPIO1_B34
#define BL_EN_PIN			GPIO0L_GPIO0A5
#define BL_EN_VALUE			GPIO_HIGH
*/
/**the value of MIN_BACKLIGHT_SCALE must be between 0~10*/
#define MIN_BACKLIGHT_SCALE	12

#define BACKLIGHT_REDUCE_VALUE 24

/*************************************************** *
 *
 *							PWM VOLTAGE REGULATOR
 *
 *********************************************************************************************/
#define REGULATOR_PWM_ID					2
#define REGULATOR_PWM_MUX_NAME      		GPIO2A3_SDMMC0WRITEPRT_PWM2_NAME
#define REGULATOR_PWM_MUX_MODE				GPIO2L_PWM2
#define REGULATOR_PWM_MUX_MODE_GPIO			GPIO2L_GPIO2A3
#define REGULATOR_PWM_GPIO					RK29_PIN2_PA3


/********************************************************************************************
 *
 *                      VIBRATION
 *
 ********************************************************************************************/
#define GPIO_VIB_CON		RK29_PIN6_PD3	
#define VIB_CON_ENABLE		GPIO_HIGH



/**********************************************************************************************
 *
 *										TOUCH PANEL
 *
 *********************************************************************************************/
#define TOUCH_POWER_PIN	RK29_PIN4_PB3
#define TOUCH_RESET_PIN RK29_PIN6_PC3
#define TOUCH_INT_PIN   RK29_PIN0_PA2

#define TOUCH_POWER_MUX_NAME      GPIO4B3_FLASHDATA11_NAME
//#define TOUCH_EN_MUX_MODE      GPIO4L_FLASH_DATA11
#define TOUCH_POWER_MUX_MODE_GPIO GPIO4L_GPIO4B3
#define TOUCH_POWER_LEVEL GPIO_HIGH

#define TOUCH_USE_I2C2	1
#define	TP_USE_WAKEUP_PIN
#define TOUCH_KEY_LED			RK29_PIN6_PA6

#define TOUCHKEY_ON_SCREEN

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
#if defined (CONFIG_SND_RK29_SOC_RT5631)
#define RTL5631_HP_HIGH
#define CONFIG_ADJUST_VOL_BY_CODEC
#define GPIO_SPK_CON			 RK29_PIN6_PB6
#define DEF_VOL					0xc1
#define DEF_VOL_SPK					0xc8
#define DEF_HP_EQ	NORMAL
#elif defined (CONFIG_SND_RK29_SOC_WM8900)
#define GPIO_SPK_CON			 RK29_PIN6_PB6
#define DEF_VOL						0x39
#endif
#define RT5631_DEF_VOL					0xc1
#define WM8900_DEF_VOL						0x39

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
#define GPIO_USB_INT			 RK29_PIN0_PA0
#define MASS_STORAGE_NAME "Arnova"
#define MASS_STORAGE_PRODUCT ""
#define USB_PRODUCT_ID			0x144A
#define ADB_PRODUCT_ID			0x0c02
#define VENDOR_ID			0x0E79
#define ADB_PRODUCT_NAME		"rk2918"
#define ADB_MANUFACTURE_NAME	"RockChip"

/**********************************************************************************************
 *
 *										HDMI
 *
 *********************************************************************************************/
#define ANX7150_ATTACHED_BUS3
#define GPIO_HDMI_DET			 RK29_PIN1_PD7
#define	GPIO_ANX7150_RST		RK29_PIN2_PC7
#define ANX7150_RST_MUX_NAME	GPIO2C7_SPI1RXD_NAME
#define ANX7150_RST_MUX_MODE	GPIO2H_GPIO2C7

/********************************************************************************************
 *
 *										3G
 *
 *******************************************************************************************/
#define G3_POWER_ON					RK29_PIN6_PB1
#define G3_POWER_ENABLE				GPIO_HIGH
#define G3_POWER_DISABLE			GPIO_LOW

#define G3_RADIO_ON_OFF             RK29_PIN6_PB3
#define G3_RADIO_ENABLE				GPIO_HIGH
#define G3_RADIO_DISABLE			GPIO_LOW

#define G3_RESET					RK29_PIN6_PB4
#define G3_RESET_ENABLE				GPIO_HIGH
#define G3_RESET_DISABLE			GPIO_LOW


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
#define GPIO_WIFI_POWER       RK29_PIN5_PD6
#define GPIO_WIFI_RESET          RK29_PIN6_PC0
#define GPIO_BT_RESET            RK29_PIN6_PC4

/**********************************************************************************************
 *
 *									BATTERY
 *
 *********************************************************************************************/
#define	DC_CURRENT_IN_TWO_MODE          //This macro to distinguish some model such as A7/A8  will work in two charging current mode
#define GPIO_CURRENT_CONTROL		RK29_PIN2_PC6
#define GPIO_CURRENT_CONTROL_LEVEL	GPIO_LOW    /* huge current charge control pin level */
#define DC_DET_EFFECTIVE		0
#define CHG_OK_EFFECTIVE		1
#define GPIO_DC_DET			RK29_PIN0_PA0
#define GPIO_CHG_OK			RK29_PIN4_PA3
#define ADC_CLI_VALUE		15
#define CHARGE_FULL_GATE 		4150

//This parameter is for new battery driver//
#define DC_DET_WITH_USB_INT
#define	TIMER_MS_COUNTS		            50	//timers length(ms)

#define	SLOPE_SECOND_COUNTS	            15	//统计电压斜率的时间间隔s
#define	DISCHARGE_MIN_SECOND	        45	//最短放电电1%时间
#define	CHARGE_MIN_SECOND	            100	//最短充电电1%时间
#define	CHARGE_MID_SECOND	            160	//最短充电电1%时间
#define	CHARGE_MAX_SECOND	            180	//最长充电电1%时间
#define CHARGE_FULL_DELAY_TIMES         10  //充电满检测防抖时间
#define USBCHARGE_IDENTIFY_TIMES        5   //插入USB混流，pc识别检测时间
#define STABLE_SECOND					8  //check ok µçÆ½»á»Î¶¯¡£¡£
#define SHUTDOWN_SECOND					20
#define SPEEDLOSE_SECOND                120 //play game rapid down

#define	NUM_VOLTAGE_SAMPLE	            ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//samling numbers
#define	NUM_DISCHARGE_MIN_SAMPLE	    ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	
#define	NUM_CHARGE_MIN_SAMPLE	        (((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)*3)	    
#define	NUM_CHARGE_MID_SAMPLE	        (((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)*3)	    
#define	NUM_CHARGE_MAX_SAMPLE	        (((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)*3)    
#define NUM_CHARGE_FULL_DELAY_TIMES     ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_USBCHARGE_IDENTIFY_TIMES    ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_STABLE_SAMPLE				((STABLE_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SHUTD0WN_SAMPLE             ((SHUTDOWN_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SPEEDLOSE_SAMPLE  			((SPEEDLOSE_SECOND * 1000) / TIMER_MS_COUNTS)

#define BAT_2V5_VALUE	        2500
#define BATT_MAX_VOL_VALUE	    4180	//满电时的电池电压	 FOR A7
#define	BATT_ZERO_VOL_VALUE     3500	//关机时的电池电压
#define BATT_NOMAL_VOL_VALUE    3800
#define SHUTDOWNVOLTAGE			3400

//define  divider resistors for ADC sampling, units as K
#define BAT_PULL_UP_R           200
#define BAT_PULL_DOWN_R         200

#define RK29_USB_CHARGE_SUPPORT

//Battery table list definition
#define BAT_RAW_TABLE_BAT { \
	3500, 3623, 3679, 3706, 3734, 3769, 3816, 3874, 3938, 3993, 4065 \
}
#define BAT_RAW_TABLE_AC { \
	3630, 3740, 3793, 3820, 3845, 3880, 3927, 3985, 4050, 4103, 4200 \
}
#define BAT_RAW_TABLE_USB { \
	3560, 3678, 3732, 3757, 3784, 3819, 3866, 3923, 3985, 4040, 4100 \
}


/***************************************************
 *
 *                  CAMERA SENSOR
 *
 **************************************************/
#if defined CONFIG_SOC_CAMERA_NT99250_BACK
#define CONFIG_SENSOR_0 RK29_CAM_SENSOR_NT99250_BACK
#define CONFIG_SENSOR_IIC_ADDR_0 	    0x6c
#elif defined CONFIG_SOC_CAMERA_GT2005_BACK
#define CONFIG_SENSOR_0 RK29_CAM_SENSOR_GT2005_BACK
#define CONFIG_SENSOR_IIC_ADDR_0 	    0x78
#elif defined CONFIG_SOC_CAMERA_OV2655_BACK
#define CONFIG_SENSOR_0 RK29_CAM_SENSOR_OV2655_BACK
#define CONFIG_SENSOR_IIC_ADDR_0 	    0x60
#else
#define CONFIG_SENSOR_0 RK29_CAM_SENSOR_OV5642			/* back camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_0 	    0x78
#endif

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








//----------------------------------------------------------------------------------
#if defined CONFIG_SOC_CAMERA_OV2655
#define CONFIG_SENSOR_1						RK29_CAM_SENSOR_OV2655
#define CONFIG_SENSOR_IIC_ADDR_1			0x60

#elif defined CONFIG_SOC_CAMERA_FCAM
#define CONFIG_SENSOR_1						RK29_CAM_SENSOR_FCAM	/* front camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_1			0xff

#else
#define CONFIG_SENSOR_1						RK29_CAM_SENSOR_GC0307	/* front camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_1			0x42
#endif

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









#define GC0308_MIRROR	0
#define GC0308_FLIP		0

#endif

