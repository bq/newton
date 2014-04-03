/* drivers/power/rk2918_battery.c
 *
 * battery detect driver for the rk2918 
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
 */


#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <linux/adc.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/kernel.h>

#include <linux/reboot.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>

#if 0
#define DBG(x...)   printk(x)
#else
#define DBG(x...)
#endif
#define CONFIG_MACH_RK29_ACH10
#if defined(CONFIG_MACH_RK29_MSI735)
#define RK29_USB_CHARGE_SUPPORT
#endif

/*******************以下参数可以修改******************************/
#if defined CONFIG_MACH_RK29_ACH10
#define DISCHARGE_MIN_SECOND                45                //低电量时，充电1%最短时间
#define CHARGE_MIN_SECOND                45                //低电量时，充电1%最短时间
#define	TIMER_MS_COUNTS		    50	//定时器的长度ms
#define	SLOPE_SECOND_COUNTS	    60	//统计电压斜率的时间间隔s
#define	CAPACITY_SECOND_COUNTS	60	//统计电量连续稳定时间s
#define BAT_CHARGE_FULL_TIMES   1
#define BAT_USBCHARGE_IDENTIFY_TIMES 5
#define	HIGH_CAPACITY_CHARGE_SECOND_COUNTS	135	//高电压时充电增长1%需要的时间s(目前参数是根据80-90之间的充电时间取平均值进行估算的)
#define BAT_STABLE_UP_DOWN_TIMES 50 //modify by hjc @02-15:60->50
#define RESET_CHARGE_IC_CHECK_INTERVAL 10
#else
#define	TIMER_MS_COUNTS		    50	//定时器的长度ms
#define	SLOPE_SECOND_COUNTS	    10	//统计电压斜率的时间间隔s
#define	CAPACITY_SECOND_COUNTS	15	//统计电量连续稳定时间s
#define BAT_CHARGE_FULL_TIMES   10
#define BAT_USBCHARGE_IDENTIFY_TIMES 5
#define	HIGH_CAPACITY_CHARGE_SECOND_COUNTS	135	//高电压时充电增长1%需要的时间s(目前参数是根据80-90之间的充电时间取平均值进行估算的)
#endif

#define	NUM_VOLTAGE_SAMPLE	        ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//存储的采样点个数
#define	NUM_CAPACITY_STABILE_SAMPLE	((CAPACITY_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//存储的采样点个数
#define	NUM_HIGH_CAPACITY_CHARGE_SAMPLE	((HIGH_CAPACITY_CHARGE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//高电压充电时增长1%时的采样个数
#define NUM_BAT_CHARGE_FULL_TIMES   ((BAT_CHARGE_FULL_TIMES * 1000) / TIMER_MS_COUNTS)	//充电满状态持续时间长度
#define NUM_BAT_USBCHARGE_IDENTIFY_TIMES   ((BAT_USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	//充电满状态持续时间长度
#define NUM_BAT_STABLE_UP_DOWN_SAMPLE ((BAT_STABLE_UP_DOWN_TIMES * 1000) / TIMER_MS_COUNTS)

#define NUM_RESET_CHARGE_IC_CHECK_INTERVAL ((RESET_CHARGE_IC_CHECK_INTERVAL * 1000) / TIMER_MS_COUNTS)

#define BAT_2V5_VALUE	2500
#define BATT_MAX_VOL_VALUE	    4180	//满电时的电池电压	 FOR A7
#define	BATT_ZERO_VOL_VALUE     3500	//关机时的电池电压
#define BATT_NOMAL_VOL_VALUE    3800
#define BATT_FILENAME "/data/bat_last_capacity.dat"

#define BAT_ADC_TABLE_LEN       11
static int adc_raw_table_bat[BAT_ADC_TABLE_LEN] = 
{
	
#if defined(CONFIG_MACH_RK29_ACH7)
	3500, 3579, 3649, 3676, 3694, 3731, 3789, 3856, 3927, 4007, 4150
#elif defined(CONFIG_MACH_RK29_ACH10)
    //3500, 3579, 3649, 3676, 3694, 3731, 3789, 3856, 3927, 4007, 4150
      3500, 3579, 3630, 3649, 3674, 3700, 3734, 3794, 3866, 3962, 4070 	
    //3500, 3566, 3610, 3629, 3656, 3689, 3734, 3804, 3866, 3962, 4070   //default modify by hjc @02-15
#else
    BATT_ZERO_VOL_VALUE + 0 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 1 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 2 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 3 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 4 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 5 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 6 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 7 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 8 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 9 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_MAX_VOL_VALUE
#endif
};

static int adc_raw_table_ac[BAT_ADC_TABLE_LEN] = 
{
#if defined(CONFIG_MACH_RK29_ACH7)
	//3760, 3886, 3964, 3989, 4020, 4062, 4123, 4180, 4189, 4190, 4190      //实际测量值
	3651, 3740, 3800, 3827, 3845, 3885, 3950, 4007, 4078, 4158, 4300//4185//4301
#elif defined(CONFIG_MACH_RK29_ACH10)
    //3651, 3740, 3800, 3827, 3845, 3885, 3950, 4007, 4078, 4158, 4300//4185//4301
    //3650, 3714, 3743, 3777, 3802, 3831, 3881, 3939, 4017, 4105, 4200
  //  3610, 3774, 3808, 3828, 3852, 3886, 3935, 3994, 4047, 4101, 4160//cst 
     // 3610, 3774, 3808, 3828, 3852, 3886, 3935, 3974, 4007, 4031, 4160//0314 @ sc
	    3610, 3764, 3828, 3858, 3890, 3936, 3975, 4004, 4027, 4051, 4160
#else
    BATT_ZERO_VOL_VALUE + 0 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 1 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 2 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 3 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 4 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 5 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 6 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 7 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 8 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_ZERO_VOL_VALUE + 9 * (BATT_MAX_VOL_VALUE - BATT_ZERO_VOL_VALUE) / 10,
    BATT_MAX_VOL_VALUE
#endif
};

static int gBatFullFlag =  0;

static int gBatLastStatus = 0;
static int gBatStatus =  POWER_SUPPLY_STATUS_UNKNOWN;
static int gBatHealth = POWER_SUPPLY_HEALTH_GOOD;
static int gBatLastPresent = 0;
static int gBatPresent = 1;
static int gBatLastVoltage =  0;
static int gBatVoltage =  BATT_NOMAL_VOL_VALUE;
static int gBatLastCapacity = 0;
static int gBatLastChargeCapacity = 100;          //记录充电时的电池容量
static int gBatCapacity = ((BATT_NOMAL_VOL_VALUE-BATT_ZERO_VOL_VALUE)*100/(BATT_MAX_VOL_VALUE-BATT_ZERO_VOL_VALUE));
static int gLastBatCapacity = ((BATT_NOMAL_VOL_VALUE-BATT_ZERO_VOL_VALUE)*100/(BATT_MAX_VOL_VALUE-BATT_ZERO_VOL_VALUE));
static int gBatStatusChangeCnt = 0;
static int gBatCapacityUpCnt = 0;
static int gBatCapacityDownCnt = 0;
static int gBatHighCapacityChargeCnt = 0;
static int gBatHighCapacityChargeFlag = 0;
static int gBatUsbChargeFlag = 0;
static int gBatUsbChargeCnt = 0;

static int gBatVoltageSamples[NUM_VOLTAGE_SAMPLE+2]; //add 2 to handle one bug
static int gBatSlopeValue = 0;
static int gBatVoltageValue[2]={0,0};
static int *pSamples = &gBatVoltageSamples[0];		//采样点指针
static int gFlagLoop = 0;		//采样足够标志
//static int gNumSamples = 0;
static int gNumCharge = 0;
static int gMaxCharge = 0;
static int gNumLoader = 0;
static int gMaxLoader = 0;
static int gSuspendTime = 0;
static int gResumeTime = 0;
static int gAdcSampleCount = 0;
static int gPowerSupplyChanged = 0;
static int gResetChargeIcChkCount = 0;
static int gCapacityZero = 0;

static int gBatVoltageTrue = 0;
s32 BatterySpendCnt = 0;              //记录启动时间
static struct regulator *pChargeregulator;
static int gBatChargeStatus = 0;

static int reset_flag = 0;
static int gFirstFlag = 1;            //第一次启动标志
static struct wake_lock wakelock;
int debug_message = 0;
int rk29_battery_dbg_level = 1;

extern int dwc_vbus_status(void);
extern int get_msc_connect_flag(void);

struct rk2918_battery_data {
	int irq;
	spinlock_t lock;
	
	struct delayed_work	work;
	struct workqueue_struct *wq;
	
	struct work_struct 	timer_work;
	struct timer_list timer;
	struct power_supply battery;
	//struct power_supply usb;
	struct power_supply ac;

	int (*reset_charge_ic)(void);    
    int dc_det_pin;
    int batt_low_pin;
    int charge_set_pin;
	int charge_ok_pin;

    int dc_det_level;
    int batt_low_level;
    int charge_set_level;
	int charge_ok_level;
	
    int dc_det_irq;

	int adc_bat_divider;
	int bat_max;
	int bat_min;
	int adc_val;
	
	int full_times;
	
	struct adc_client *client; 
};


/* temporary variable used between rk2918_battery_probe() and rk2918_battery_open() */
static struct rk2918_battery_data *gBatteryData;

enum {
	BATTERY_STATUS          = 0,
	BATTERY_HEALTH          = 1,
	BATTERY_PRESENT         = 2,
	BATTERY_CAPACITY        = 3,
	BATTERY_AC_ONLINE       = 4,
	BATTERY_STATUS_CHANGED	= 5,
	AC_STATUS_CHANGED   	= 6,
	BATTERY_INT_STATUS	    = 7,
	BATTERY_INT_ENABLE	    = 8,
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

static  char is_charge = 0;
#include <linux/fs.h>

static void rk2918_batscan_timer(unsigned long data);

static ssize_t rk2918_battery_startget_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	printk("===1====%s=====\n",__FUNCTION__);
	static int only_one = 0;

	if (only_one != 0)
	{
		return len;
	}

	only_one = 1;
	

	if ((*buf=='A'))
	{
		printk("%s success  write start to report battery\n",__FUNCTION__);
		rk2918_batscan_timer(NULL);

	}
	else
	{

		printk("%s error %s %d !!!!!!!!!!!!!!!!\n\n",__FUNCTION__,buf,len);
		rk2918_batscan_timer(NULL);
	}
	printk("====2===%s=====\n",__FUNCTION__);
	return len;

}

static DEVICE_ATTR(startget,0666,NULL,rk2918_battery_startget_store);
static void rk2918_charge_enable(void)
{
		printk("------------- rk2918_charge_enable.\n");
		wake_lock(&wakelock);//delete by hjc @0226
    if (gBatteryData->charge_set_pin != INVALID_GPIO)
    {
        gpio_direction_output(gBatteryData->charge_set_pin, gBatteryData->charge_set_level);
    }
}

static void rk2918_charge_disable(void)
{
		printk("------------- rk2918_charge_disable.\n");
		wake_unlock(&wakelock);//delete by hjc @0226
    if (gBatteryData->charge_set_pin != INVALID_GPIO)
    {
        gpio_direction_output(gBatteryData->charge_set_pin, 1 - gBatteryData->charge_set_level);
    }
}

static void rk2918_get_charge_status(void)
{
    int charge_on = 0;
    
    if (gBatteryData->dc_det_pin != INVALID_GPIO)
    {
        if (gpio_get_value (gBatteryData->dc_det_pin) == gBatteryData->dc_det_level)
        {
            charge_on = 1;
        }
    }
    
#ifdef RK29_USB_CHARGE_SUPPORT
    if (charge_on == 0)
    {
        if (1 == dwc_vbus_status())         //检测到USB插入，但是无法识别是否是充电器
        {                                   //通过延时检测PC识别标志，如果超时检测不到，说明是充电
            if (0 == get_msc_connect_flag())
            {                               //插入充电器时间大于一定时间之后，开始进入充电状态
                if (++gBatUsbChargeCnt >= NUM_BAT_USBCHARGE_IDENTIFY_TIMES)
                {
                    gBatUsbChargeCnt = NUM_BAT_USBCHARGE_IDENTIFY_TIMES + 1;
                    charge_on = 1;
                }
            }                               //否则，不进入充电模式
        }                   
        else
        {
            gBatUsbChargeCnt = 0;
            if (2 == dwc_vbus_status()) 
            {
                charge_on = 1;
            }
        }
    }
#endif
        
    if (charge_on)
    {
        if(gBatChargeStatus !=1) 
        {            
            gBatChargeStatus = 1;
            gBatStatusChangeCnt = 0;        //状态变化开始计数
            rk2918_charge_enable();
        }
    } 
    else 
    {
        if(gBatChargeStatus != 0) 
        {
            gBatChargeStatus = 0;
            gBatStatusChangeCnt = 0;        //状态变化开始计数
            rk2918_charge_disable();
        }
    }
}

static void rk2918_get_bat_status(struct rk2918_battery_data *bat)
{
    rk2918_get_charge_status();
    
	if(gBatChargeStatus == 1)
	{
        if (gBatteryData->charge_ok_pin == NULL)
        {
            printk("dc_det_pin invalid!\n");
            return;
        }
#if 0  
	if(gResetChargeIcChkCount++ > NUM_RESET_CHARGE_IC_CHECK_INTERVAL) 
	{
		gResetChargeIcChkCount = 0;
               if ((gpio_get_value(gBatteryData->charge_ok_pin) == gBatteryData->charge_ok_level) && (gBatCapacity < 85)) //hjc 90->85
               {
                       if(gBatteryData->reset_charge_ic)
                               gBatteryData->reset_charge_ic();
               }
	}
#endif
if(gResetChargeIcChkCount++ > NUM_RESET_CHARGE_IC_CHECK_INTERVAL) 
{
		gResetChargeIcChkCount = 0;
 	 if ((gpio_get_value(gBatteryData->charge_ok_pin) == gBatteryData->charge_ok_level) && (gBatCapacity < 99)) //hjc 90->85
         {
         	   printk("(gpio_get_value(gBatteryData->charge_ok_pin) == gBatteryData->charge_ok_level) && (gBatCapacity < 90)");
	           reset_flag = 1;
                   if(gBatteryData->reset_charge_ic)
                            gBatteryData->reset_charge_ic();
         }

	  	 if ((gpio_get_value(gBatteryData->charge_ok_pin) == gBatteryData->charge_ok_level) && (gBatVoltage < 4031)) //hjc 90->85
         {
         	   printk("(gpio_get_value(gBatteryData->charge_ok_pin) == gBatteryData->charge_ok_level) && (gBatVoltage < 4031)");
                   if((gBatteryData->reset_charge_ic)&&(reset_flag == 0))
                            gBatteryData->reset_charge_ic();
				   reset_flag = 0;
         }
}

 
	    if ((gpio_get_value(gBatteryData->charge_ok_pin) == gBatteryData->charge_ok_level) && (gBatCapacity >= 99)) //90->99
        {
            gBatteryData->full_times++;
            if (gBatteryData->full_times >= NUM_BAT_CHARGE_FULL_TIMES)
		    {
		        gBatStatus = POWER_SUPPLY_STATUS_FULL;
		        gBatteryData->full_times = NUM_BAT_CHARGE_FULL_TIMES + 1;
		    }
		    else
		    {
		        gBatStatus = POWER_SUPPLY_STATUS_CHARGING;
		    }
	    }
	    else
	    {
	        gBatStatus = POWER_SUPPLY_STATUS_CHARGING;
	        gBatteryData->full_times = 0;
	    }
	}
	else 
    {
	    gBatteryData->full_times = 0;
        gBatStatus = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
}

static void rk2918_get_bat_health(struct rk2918_battery_data *bat)
{
	gBatHealth = POWER_SUPPLY_HEALTH_GOOD;
}

static void rk2918_get_bat_present(struct rk2918_battery_data *bat)
{
	if(gBatVoltage < bat->bat_min)
	gBatPresent = 0;
	else
	gBatPresent = 1;
}

int capacitytmp = 0;
static int rk2918_get_bat_capacity_raw(int BatVoltage)
{
    int i = 0;
	int capacity = 0;
	int *p = adc_raw_table_bat;
    
    if (gBatChargeStatus == 1)
    {
        p = adc_raw_table_ac;
    }
	
	if(BatVoltage >= p[BAT_ADC_TABLE_LEN - 1])
	{
	    //当电压超过最大值
	    capacity = 100;
	}	
	else if(BatVoltage <= p[0])
	{
	    //当电压低于最小值
	    capacity = 0;
	}
	else
	{
    	//计算容量
    	for(i = 0; i < BAT_ADC_TABLE_LEN - 1; i++)
        {
    		
    		if((p[i] <= BatVoltage) && (BatVoltage < p[i+1]))
    		{
    			capacity = i * 10 + ((BatVoltage - p[i]) * 10) / (p[i+1] - p[i]);
    			break;
    		}
    	}
    }  
    return capacity;
}

static int rk2918_get_bat_capacity_ext(int BatVoltage)
{
    int i = 0;
	int capacity = 0;
//	int *p = adc_raw_table_bat;
//    
//    if (gBatChargeStatus == 1)
//    {
//        p = adc_raw_table_ac;
//    }
//	
//	if(BatVoltage >= p[BAT_ADC_TABLE_LEN - 1])
//	{
//	    //当电压超过最大值
//	    capacity = 100;
//	}	
//	else if(BatVoltage <= p[0])
//	{
//	    //当电压低于最小值
//	    capacity = 0;
//	}
//	else
//	{
//    	//计算容量
//    	for(i = 0; i < BAT_ADC_TABLE_LEN - 1; i++)
//        {
//    		
//    		if((p[i] <= BatVoltage) && (BatVoltage < p[i+1]))
//    		{
//    			capacity = i * 10 + ((BatVoltage - p[i]) * 10) / (p[i+1] - p[i]);
//    			break;
//    		}
//    	}
//    }
    capacity = rk2918_get_bat_capacity_raw(BatVoltage);
	capacitytmp = capacity;
    
    if (gBatChargeStatus == 1)
    {
        //对于插入充电器时,电压直接提升至满电时,通过定时器显示容量
        if ((capacity >= 90) && (gBatStatus != POWER_SUPPLY_STATUS_FULL))
        {
            if (gBatHighCapacityChargeFlag == 0)
            {
                gBatHighCapacityChargeFlag = 1;
                gBatHighCapacityChargeCnt = 0;
            }
            else
            {
                gBatHighCapacityChargeCnt++;
                capacity += gBatHighCapacityChargeCnt / NUM_HIGH_CAPACITY_CHARGE_SAMPLE;
                if (capacity >= 100) 
                {
                    capacity = 99;
                }
            }
        }

		if (capacity == 100)
			capacity = 99;  // 充电时, 只有POWER_SUPPLY_STATUS_FULL时才能显示100
        
        if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
		{
		    capacity = 100;
		}
		
		if (capacity == 0)
		    capacity = 1;
        
        //充电时,只允许电压上升
        if (capacity < gBatCapacity)
        {
            capacity = gBatCapacity;
        }
        
        gBatLastChargeCapacity = capacity;      //记录充电时电池容量
    }    
    else
    {   
        //放电时,如果只允许电压下降,打开下面的代码
        #if 0
        if (capacity > gBatCapacity)
        {
            capacity = gBatCapacity;
        }
        #else
        
        gBatHighCapacityChargeFlag = 0;         //放电时清除标志
        
        //如果在连续的时间内(NUM_CAPACITY_STABILE_SAMPLE)电池容量稳定(上升或下降)，更新电池容量
        //否则，不更新
        if ((capacity - gLastBatCapacity) < 0)
        {
            //电压下降
            gBatCapacityDownCnt++;
            gBatCapacityUpCnt = 0;
        }
        else if ((capacity - gLastBatCapacity) > 0)
        {
            //电压上升
            gBatCapacityDownCnt = 0;
            gBatCapacityUpCnt ++;
        }
        else
        {
            //稳定,更新
            if (++gBatCapacityDownCnt >= NUM_CAPACITY_STABILE_SAMPLE)
            {
                gBatCapacityDownCnt = NUM_CAPACITY_STABILE_SAMPLE + 1;
            }
            if (++gBatCapacityUpCnt >= NUM_CAPACITY_STABILE_SAMPLE)
            {
                gBatCapacityUpCnt = NUM_CAPACITY_STABILE_SAMPLE + 1;
            }
        }
        gLastBatCapacity = capacity;

        if ((gBatCapacityUpCnt > NUM_CAPACITY_STABILE_SAMPLE) || (gBatCapacityDownCnt > NUM_CAPACITY_STABILE_SAMPLE))
        {
            //稳定(上升或下降),更新;
        }
        else
        {
            //否则，不更新
            capacity = gBatCapacity;
        }
                
        //
        if (gBatStatusChangeCnt < NUM_VOLTAGE_SAMPLE)
        {
            //充放电状态变化后，Buffer填满之前，不更新
            capacity = gBatCapacity;
        }
        
        if (capacity > gBatLastChargeCapacity)
        {
            //检测到电池容量 > 拔掉充电器之前的容量，不更新
            capacity = gBatCapacity;
        }

#if defined(CONFIG_MACH_RK29_ACH10)        
        //电池电压上升，不更新
        if (capacity > gBatCapacity)
        {
            capacity = gBatCapacity;
        }
#endif

		if (capacity == 100)
			capacity = 99;  // 放电时，电量最多只能显示99%
        
        #endif
    }

#if 1
	// 电量平滑处理
//	if(gBatCapacity > 10 && gPowerSupplyChanged == 1) {
	if(gBatCapacity > 0 && gPowerSupplyChanged == 1) {
		if(++gAdcSampleCount > NUM_BAT_STABLE_UP_DOWN_SAMPLE) {
			gAdcSampleCount = 0;
			if(gBatChargeStatus == 1) { // 充电时, 每过一定的时间60s，电量最多只能上升1%
				if(capacity > gBatLastCapacity + 1)
					capacity = gBatLastCapacity + 1;
			} else {                    // 放电时, 每过一定的时间60s，电量最多只能下降1%
				if(capacity < gBatLastCapacity - 1)
					capacity = gBatLastCapacity - 1;
				if((capacity <= 0)&&(gBatVoltageTrue < 3460)&&(gCapacityZero++ >= 3*NUM_BAT_STABLE_UP_DOWN_SAMPLE)){
					//capacity = 1;delete by hjc
					capacity = 0;
					gCapacityZero = 0;
				
				}else if(capacity <= 0){
					capacity = 1;
				}	
			}
		} else { // 在一定时间内60s，电量保持稳定
			capacity = gBatLastCapacity;
		}
	} else {
		gAdcSampleCount = 0;
	}
#endif	
	
	return capacity;
}

unsigned long AdcTestvalue = 0;
unsigned long AdcTestCnt = 0;
static void rk2918_get_bat_voltage(struct rk2918_battery_data *bat)
{
	int value;
	int i,*pSamp,*pStart = &gBatVoltageSamples[0],num = 0;
	int temp[2] = {0,0};
	
	value = gBatteryData->adc_val;
	AdcTestvalue = value;
    adc_async_read(gBatteryData->client);
    
	*pSamples++ = (value * BAT_2V5_VALUE * 2) / 1024;
	num = pSamples - pStart;
	if (num >= NUM_VOLTAGE_SAMPLE)
	{
	    pSamples = pStart;
	    gFlagLoop = 1;
	}
	if (gFlagLoop == 1)
	{
	    num = NUM_VOLTAGE_SAMPLE;
	}
	value = 0;
	for (i = 0; i < num; i++)
	{
	    value += gBatVoltageSamples[i];
	}
	gBatVoltage = value / num;
	gBatVoltageTrue = gBatVoltage;
	//gBatVoltage = (value * BAT_2V5_VALUE * 2) / 1024;
	
	/*消除毛刺电压*/
	if(gBatVoltage >= BATT_MAX_VOL_VALUE + 10)
		gBatVoltage = BATT_MAX_VOL_VALUE + 10;
	else if(gBatVoltage <= BATT_ZERO_VOL_VALUE - 10)
		gBatVoltage = BATT_ZERO_VOL_VALUE - 10;

    //充放电状态变化时,开始计数	
	if (++gBatStatusChangeCnt > NUM_VOLTAGE_SAMPLE)  
	    gBatStatusChangeCnt = NUM_VOLTAGE_SAMPLE + 1;
}


static int rk29_adc_read(int num)
{
	//同步读adc 
	int i,temp=0;
	for(i=0;i<num;i++){
		temp += adc_sync_read(gBatteryData->client);
	}
	temp = temp/num;
	return (temp * BAT_2V5_VALUE *2)/1024;
}

static int rk2918_battery_resume_get_capacity(int deltatime){	//待机唤醒后执行
	int tmp = 0;	
	int capacity = 0;		
	tmp = rk29_adc_read(20);	
	rk2918_get_charge_status;	
	capacity = rk2918_get_bat_capacity_raw(tmp);	
	printk("%s,deltatime=%d,capacity=%d,gBatCapacity=%d,gBatChargeStatus=%d\n",__FUNCTION__,deltatime,capacity,gBatCapacity,gBatChargeStatus);	
	if(1 == gBatChargeStatus){		
		if(deltatime > (100-gBatCapacity)*CHARGE_MIN_SECOND)			
			deltatime = (100 - gBatCapacity)*CHARGE_MIN_SECOND;		
		if(capacity > gBatCapacity+(deltatime/CHARGE_MIN_SECOND))      //如果充电量比最大值还大		
		capacity = gBatCapacity+(deltatime / CHARGE_MIN_SECOND);//采样电池容量偏差较大，将容量拉回		
		else if(capacity < gBatCapacity)			
			capacity = gBatCapacity;		
		}	
	else{		
		if(deltatime > gBatCapacity*DISCHARGE_MIN_SECOND)			
			deltatime = gBatCapacity*DISCHARGE_MIN_SECOND;		
		if(capacity < gBatCapacity-deltatime*DISCHARGE_MIN_SECOND)      //如果放电量比最小值还小			
			capacity = gBatCapacity - deltatime*DISCHARGE_MIN_SECOND;//采样电池容量偏差较大，将容量拉回		
		else if(capacity > gBatCapacity)			
			capacity = gBatCapacity;	
		}	
		if(capacity == 0)
			capacity = 1;	
		if((capacity == 100)&&(gBatStatus == POWER_SUPPLY_STATUS_FULL)) 
			capacity=100;	
		else if((capacity == 100)&&(gBatStatus != POWER_SUPPLY_STATUS_FULL))
		capacity = 99;		
		return capacity;
	}


static int rk2918_battery_load_get_capacity(void)
{
	//第一次启动时执行，从文件中读出的备份值和实际值比较确定capacity
	int i;
	int tmp = 0;
	int loadcapacity = 0;
	int truecapacity = 0;
 	char value[11];
 	char *p = value;
 	//struct file *fp = filp_open(BATT_FILENAME,O_RDONLY,0);
	struct file *fp = filp_open("/data/bat_last_capacity.dat",O_RDONLY,0);

	tmp = rk29_adc_read(50);
	
	rk2918_get_bat_status(gBatteryData);// ADD BY HJC
	
	truecapacity = rk2918_get_bat_capacity_raw(tmp);

	printk("hjc----%s-----\n",__FUNCTION__);
	if(IS_ERR(fp)){
		printk("open file /data/bat_last_capacity.dat failed!\n");
		printk("truecapacity= %d\n",truecapacity);
		return truecapacity;
	}

	kernel_read(fp,0,value,10);
	value[10]=0;

	while(*p){
		if(*p == 0x0d){
			*p = 0;
			break;
		}
		p++;
	}
	sscanf(value,"%d",&loadcapacity);

	printk("hj-------------------loadcapacity=%d,truecapacity=%d\n",loadcapacity,truecapacity);

#if 1
	if((loadcapacity < 5)&&(gBatStatus== POWER_SUPPLY_STATUS_CHARGING)&&(tmp > 3740)&&(tmp < 3900)){
		tmp = tmp-70;
		truecapacity = rk2918_get_bat_capacity_raw(tmp);
		printk("hjc  1111----------loadcapacity=%d,truecapacity=%d\n",loadcapacity,truecapacity);

	}else if((loadcapacity < 5)&&(gBatStatus == POWER_SUPPLY_STATUS_DISCHARGING)&&(tmp > 3579)&&(tmp < 3630)){
		//tmp = tmp-61;
		truecapacity = rk2918_get_bat_capacity_raw(tmp);
		printk("hjc  2222----------loadcapacity=%d,truecapacity=%d\n",loadcapacity,truecapacity);
	}	
#endif
//如果从文件中读取的电压比实际的高很多的话，说明是长时间放置导致放电
	if (loadcapacity > truecapacity){
	    if ((loadcapacity - truecapacity) > 25)//15->25
	        loadcapacity = truecapacity;
	}
	else{
		if (((truecapacity-loadcapacity) >= 30)){
			if (truecapacity < 30){
				if (loadcapacity < 10)
					loadcapacity = truecapacity/2;
			}
			else{
				if (truecapacity < 80)
					loadcapacity = truecapacity - 2*(10 - truecapacity/10);	
				else
					loadcapacity = truecapacity;
				}
			}
		}
	if (loadcapacity == 0)
		loadcapacity = 1;
	else if ((loadcapacity >= 100)||(gBatStatus == POWER_SUPPLY_STATUS_FULL))
		loadcapacity = 100;
	 if((loadcapacity >= 100)&&(gBatStatus != POWER_SUPPLY_STATUS_FULL))  //add 0909
		loadcapacity = 99;
	return loadcapacity;
		
}

static void rk2918_get_bat_capacity(struct rk2918_battery_data *bat)
{
	int deltatime = 0;
	struct timespec now;
	getnstimeofday(&now);
	deltatime = now.tv_sec-BatterySpendCnt;

	if(gFirstFlag){
		gFirstFlag--;
		gBatCapacity = rk2918_battery_load_get_capacity();
	}
	else if((deltatime > 30)&&(gFirstFlag == 0))
	{
		//printk("in the else if,deltatime=%d,gFirstFlag=%d\n",deltatime,gFirstFlag);
		gBatCapacity = rk2918_battery_resume_get_capacity(deltatime);
	}
	else{
		gBatCapacity = rk2918_get_bat_capacity_ext(gBatVoltage);
	}
	BatterySpendCnt=now.tv_sec;
}

static void rk2918_battery_timer_work(struct work_struct *work)
{		
	rk2918_get_bat_status(gBatteryData);
	rk2918_get_bat_health(gBatteryData);
	rk2918_get_bat_present(gBatteryData);
	rk2918_get_bat_voltage(gBatteryData);
	rk2918_get_bat_capacity(gBatteryData);
if (rk29_battery_dbg_level){
	if (++AdcTestCnt >= 40)
	{
	    AdcTestCnt = 0;
	    printk("hjc---chgok_pin = %d, gBatStatus = %d, adc_value = %d, TrueBatVoltage = %d,gBatVoltage = %d, gBatCapacity = %d, capacitytmp = %d,gPowerSupplyChanged=%d\n", gpio_get_value(gBatteryData->charge_ok_pin),
	            gBatStatus, AdcTestvalue, ((AdcTestvalue * BAT_2V5_VALUE * 2) / 1024), gBatVoltage, gBatCapacity, capacitytmp,gPowerSupplyChanged);
	}
}
	/*update battery parameter after adc and capacity has been changed*/
	if((gBatStatus != gBatLastStatus) || (gBatPresent != gBatLastPresent) || (gBatCapacity != gBatLastCapacity))
	{
		//gNumSamples = 0;
		gPowerSupplyChanged = 1;
		gBatLastStatus = gBatStatus;
		gBatLastPresent = gBatPresent;
		gBatLastCapacity = gBatCapacity;
		power_supply_changed(&gBatteryData->battery);
	}
}


static void rk2918_batscan_timer(unsigned long data)
{
    gBatteryData->timer.expires  = jiffies + msecs_to_jiffies(TIMER_MS_COUNTS);
	add_timer(&gBatteryData->timer);
	schedule_work(&gBatteryData->timer_work);	
}

#if 0
static int rk2918_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	charger =  CHARGER_USB;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = dwc_vbus_status();
		DBG("%s:%d\n",__FUNCTION__,val->intval);
		break;

	default:
		return -EINVAL;
	}
	
	return 0;

}
#endif

static int rk2918_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	charger_type_t charger;
	charger =  CHARGER_USB;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		{
			//if (gBatChargeStatus == 1)
			if ((gBatChargeStatus == 1)&&(gBatCapacity != 100))
				val->intval = 1;
			else
				val->intval = 0;	
		}
		DBG("%s:%d\n",__FUNCTION__,val->intval);
		break;
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int rk2918_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct rk2918_battery_data *data = container_of(psy,
		struct rk2918_battery_data, battery);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = gBatStatus;
		DBG("gBatStatus=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = gBatHealth;
		DBG("gBatHealth=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = gBatPresent;
		DBG("gBatPresent=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if(gBatVoltageValue[1] == 0)
		val ->intval = gBatVoltage;
		else
		val ->intval = gBatVoltageValue[1];
		DBG("gBatVoltage=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = gBatCapacity;
		DBG("gBatCapacity=%d%%\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = data->bat_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = data->bat_min;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property rk2918_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

#if 0
static enum power_supply_property rk2918_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
#endif

static enum power_supply_property rk2918_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


#ifdef CONFIG_PM
static int rk2918_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	//struct timespec ts;
	
	/* flush all pending status updates */
	flush_scheduled_work();

	//getnstimeofday(&ts);
	//gSuspendTime = ts.tv_sec;
    gBatLastChargeCapacity = gBatCapacity;                  //睡眠唤醒后的电池容量不能大于睡眠之前的电池容量
    gPowerSupplyChanged = 0;
	gAdcSampleCount = 0;
	return 0;
}

static int rk2918_battery_resume(struct platform_device *dev)
{
	int i;
    int tmp = 0;
	//struct timespec ts;

	//getnstimeofday(&ts);
	//gResumeTime = ts.tv_sec;
	//printk("rk2918_battery_resume: gResumeTime = %d, gSuspendTime = %d\n", gResumeTime, gSuspendTime);
	if(1) {
	    for (i = 0; i < 20; i++)
	    {
	        tmp += adc_sync_read(gBatteryData->client);
	        mdelay(1);
	    }
	    tmp = tmp / 20;
	    gBatteryData->adc_val = tmp;
	    gBatVoltage = (tmp * BAT_2V5_VALUE * 2) / 1024;
	    rk2918_get_charge_status();
	    gBatCapacity = rk2918_get_bat_capacity_raw(gBatVoltage);
	    if (gBatCapacity == 0) gBatCapacity = 1;
	    printk("rk2918_battery_resume: gBatVoltage = %d, gBatCapacity = %d\n", gBatVoltage, gBatCapacity);
	    
#if 0//defined(CONFIG_MACH_RK29_ACH10) 
		if(gBatChargeStatus == 0) { // 未充电时
			if(gBatCapacity > gBatLastChargeCapacity)
				gBatCapacity = gBatLastChargeCapacity; // 唤醒后电量不能马上上升
		} else if(gBatChargeStatus == 1) { // 充电时
			if(gBatCapacity < gBatLastChargeCapacity)
				gBatCapacity = gBatLastChargeCapacity; // 唤醒后电量不能马上下降
		}
#endif	
	}

	/* things may have changed while we were away */
	schedule_work(&gBatteryData->timer_work);
	return 0;
}
#else
#define rk2918_battery_suspend NULL
#define rk2918_battery_resume NULL
#endif

static irqreturn_t rk2918_battery_interrupt(int irq, void *dev_id)
{
//    if ((rk2918_get_charge_status()) && (gBatFullFlag != 1))
//    {
//        gBatFullFlag = 1;
//    }

    return 0;
}

static irqreturn_t rk2918_dc_wakeup(int irq, void *dev_id)
{   
    gBatStatusChangeCnt = 0;        //状态变化开始计数
    
    disable_irq_wake(gBatteryData->dc_det_irq);
    queue_delayed_work(gBatteryData->wq, &gBatteryData->work, 0);
	
    return IRQ_HANDLED;
}

static void rk2918_battery_work(struct work_struct *work)
{
    int ret;
    int irq_flag;
    
    rk28_send_wakeup_key();
    
    free_irq(gBatteryData->dc_det_irq, gBatteryData);
    irq_flag = (!gpio_get_value (gBatteryData->dc_det_pin)) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
	ret = request_irq(gBatteryData->dc_det_irq, rk2918_dc_wakeup, irq_flag, "rk2918_battery", gBatteryData);
	if (ret) {
		free_irq(gBatteryData->dc_det_irq, gBatteryData);
	}
	enable_irq_wake(gBatteryData->dc_det_irq);
}

static void rk2918_battery_callback(struct adc_client *client, void *param, int result)
{
    gBatteryData->adc_val = result;
	return;
}

#define POWER_ON_PIN    RK29_PIN4_PA4
static void rk2918_low_battery_check(void)
{
    int i;
    int tmp = 0;
    
    for (i = 0; i < 100; i++)
    {
        tmp += adc_sync_read(gBatteryData->client);
        mdelay(1);
    }
    tmp = tmp / 100;
    gBatteryData->adc_val = tmp;
    gBatVoltage = (tmp * BAT_2V5_VALUE * 2) / 1024;
    rk2918_get_charge_status();
    gBatCapacity = rk2918_get_bat_capacity_raw(gBatVoltage);
    if (gBatCapacity == 0) gBatCapacity = 1;
    gLastBatCapacity = gBatCapacity;
	gAdcSampleCount = 0;
    printk("rk2918_low_battery_check: gBatVoltage = %d, gBatCapacity = %d\n", gBatVoltage, gBatCapacity);
    
   // if (gBatVoltage <= BATT_ZERO_VOL_VALUE + 50)
	if (gBatVoltage <= BATT_ZERO_VOL_VALUE - 30)
    {
        printk("low battery: powerdown\n");
        gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
        tmp = 0;
        while(1)
        {
            if(gpio_get_value(POWER_ON_PIN) == GPIO_HIGH)
		    {
			    gpio_set_value(POWER_ON_PIN,GPIO_LOW);
		    }
		    mdelay(5);
		    if (++tmp > 50) break;
		}
    }
    gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
}

static ssize_t battery_mode_store(struct class *cls,struct class_attribute *attr, char *_buf,size_t _count)
{

        rk29_battery_dbg_level = simple_strtoul(_buf, NULL, 16);    
	printk("rk29_battery_dbg_level = %d\n",rk29_battery_dbg_level);        return _count;
} 

static ssize_t battery_mode_show(struct class *cls,struct class_attribute *attr, char *_buf,size_t _count)
{
       return sprintf(_buf, "%d\n", rk29_battery_dbg_level);
	
}


static struct class *bat_class = NULL;
static CLASS_ATTR(batterydebug, 0666, battery_mode_show, battery_mode_store);

static int rk2918_battery_probe(struct platform_device *pdev)
{
	int ret,error;
	struct adc_client *client;
	struct rk2918_battery_data *data;
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;
	int irq_flag;
	
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}

    //clear io
    data->dc_det_pin     = INVALID_GPIO;
    data->batt_low_pin   = INVALID_GPIO;
    data->charge_set_pin = INVALID_GPIO;
	data->charge_ok_pin  = INVALID_GPIO;
	data->reset_charge_ic = NULL;
	
	if (pdata && pdata->io_init) {
		ret = pdata->io_init();
		if (ret) 
			goto err_free_gpio1;		
	}
	
	//dc det
	if (pdata->dc_det_pin != INVALID_GPIO)
	{
    	ret = gpio_request(pdata->dc_det_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto err_free_gpio1;
    	}
	
    	gpio_pull_updown(pdata->dc_det_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->dc_det_pin);
    	if (ret) {
    		printk("failed to set gpio dc_det input\n");
    		goto err_free_gpio1;
    	}
    	data->dc_det_pin   = pdata->dc_det_pin;
    	data->dc_det_level = pdata->dc_det_level;
    }
	
	//charge set for usb charge
	if (pdata->charge_set_pin != INVALID_GPIO)
	{
    	ret = gpio_request(pdata->charge_set_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto err_free_gpio1;
    	}
    	data->charge_set_pin = pdata->charge_set_pin;
    	data->charge_set_level = pdata->charge_set_level;
    	gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
    }
	
	//charge_ok
	if (pdata->charge_ok_pin != INVALID_GPIO)
	{
        ret = gpio_request(pdata->charge_ok_pin, NULL);
    	if (ret) {
    		printk("failed to request charge_ok gpio\n");
    		goto err_free_gpio2;
    	}
	
    	gpio_pull_updown(pdata->charge_ok_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->charge_ok_pin);
    	if (ret) {
    		printk("failed to set gpio charge_ok input\n");
    		goto err_free_gpio2;
    	}
    	data->charge_ok_pin   = pdata->charge_ok_pin;
    	data->charge_ok_level = pdata->charge_ok_level;
    }

       if(pdata->reset_charge_ic != NULL) 
       {
               data->reset_charge_ic = pdata->reset_charge_ic;
               data->reset_charge_ic();
       }
       
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "rk29_battery");       
    
	client = adc_register(0, rk2918_battery_callback, NULL);
    if(!client)
		goto err_adc_register_failed;
    
	memset(gBatVoltageSamples, 0, sizeof(gBatVoltageSamples));
	spin_lock_init(&data->lock);
    data->adc_val = adc_sync_read(client);
	data->client = client;
    data->battery.properties = rk2918_battery_props;
	data->battery.num_properties = ARRAY_SIZE(rk2918_battery_props);
	data->battery.get_property = rk2918_battery_get_property;
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	data->adc_bat_divider = 414;
	data->bat_max = BATT_MAX_VOL_VALUE;
	data->bat_min = BATT_ZERO_VOL_VALUE;
	DBG("bat_min = %d\n",data->bat_min);
	
#if 0
	data->usb.properties = rk2918_usb_props;
	data->usb.num_properties = ARRAY_SIZE(rk2918_ac_props);
	data->usb.get_property = rk2918_usb_get_property;
	data->usb.name = "usb";
	data->usb.type = POWER_SUPPLY_TYPE_USB;
#endif

	data->ac.properties = rk2918_ac_props;
	data->ac.num_properties = ARRAY_SIZE(rk2918_ac_props);
	data->ac.get_property = rk2918_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;

	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret)
	{
		printk(KERN_INFO "fail to ac power_supply_register\n");
		goto err_ac_failed;
	}
#if 0
	ret = power_supply_register(&pdev->dev, &data->usb);
	if (ret)
	{
		printk(KERN_INFO "fail to usb power_supply_register\n");
		goto err_usb_failed;
	}
#endif
	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
	{
		printk(KERN_INFO "fail to battery power_supply_register\n");
		goto err_battery_failed;
	}
	platform_set_drvdata(pdev, data);
	
	INIT_WORK(&data->timer_work, rk2918_battery_timer_work);
	gBatteryData = data;
	
    irq_flag = (pdata->charge_ok_level) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
	ret = request_irq(gpio_to_irq(pdata->charge_ok_pin), rk2918_battery_interrupt, irq_flag, "rk2918_battery", data);
	if (ret) {
		printk("failed to request irq\n");
		goto err_irq_failed;
	}

	irq_flag = (!gpio_get_value (pdata->dc_det_pin)) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
	ret = request_irq(gpio_to_irq(pdata->dc_det_pin), rk2918_dc_wakeup, irq_flag, "rk2918_battery", data);
	if (ret) {
		printk("failed to request dc det irq\n");
		goto err_dcirq_failed;
	}
	data->dc_det_irq = gpio_to_irq(pdata->dc_det_pin);
	data->wq = create_workqueue("rk2918_battery");
	INIT_DELAYED_WORK(&data->work, rk2918_battery_work);
	
	enable_irq_wake(gpio_to_irq(pdata->dc_det_pin));

	setup_timer(&data->timer, rk2918_batscan_timer, (unsigned long)data);
	//data->timer.expires  = jiffies+100;
	//add_timer(&data->timer);
	
    gBatCapacity = 1;
    gLastBatCapacity = gBatCapacity;
	
        bat_class = class_create(THIS_MODULE, "battery");
    if (IS_ERR(bat_class))
        {
            printk("Create class battery failed.\n");
            error = -12;
            goto err_free_mem;
        }
        error = class_create_file(bat_class,&class_attr_batterydebug);
     ret = device_create_file(&pdev->dev, &dev_attr_startget);
     if (ret)
     {
	
		printk("make a mistake in creating devices  attr file\n\n ");
     }

        printk("Create class battery is sucess\n");  

	
    
    rk2918_low_battery_check();

	printk(KERN_INFO "rk2918_battery: driver initialized\n");
	
	return 0;
err_free_mem:

	printk("%s,probe battery ok!\n",__FUNCTION__);
	return 0;

	
err_dcirq_failed:
    free_irq(gpio_to_irq(pdata->dc_det_pin), data);
    
err_irq_failed:
	free_irq(gpio_to_irq(pdata->charge_ok_pin), data);
    
err_battery_failed:
//	power_supply_unregister(&data->usb);
//err_usb_failed:
err_ac_failed:
	power_supply_unregister(&data->ac);
	
err_adc_register_failed:
err_free_gpio2:
	gpio_free(pdata->charge_ok_pin);
err_free_gpio1:
    gpio_free(pdata->dc_det_pin);
    
err_data_alloc_failed:
	kfree(data);

    printk("rk2918_battery: error!\n");
    
	return ret;
}

static int rk2918_battery_remove(struct platform_device *pdev)
{
	struct rk2918_battery_data *data = platform_get_drvdata(pdev);
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;

	power_supply_unregister(&data->battery);
//	power_supply_unregister(&data->usb);
	power_supply_unregister(&data->ac);
	free_irq(data->irq, data);
	gpio_free(pdata->charge_ok_pin);
	gpio_free(pdata->dc_det_pin);
	kfree(data);
	gBatteryData = NULL;
	return 0;
}

static struct platform_driver rk2918_battery_driver = {
	.probe		= rk2918_battery_probe,
	.remove		= rk2918_battery_remove,
	.suspend	= rk2918_battery_suspend,
	.resume		= rk2918_battery_resume,
	.driver = {
		.name = "rk2918-battery",
		.owner	= THIS_MODULE,
	}
};

static int __init rk2918_battery_init(void)
{
	return platform_driver_register(&rk2918_battery_driver);
}

static void __exit rk2918_battery_exit(void)
{
	platform_driver_unregister(&rk2918_battery_driver);
}

module_init(rk2918_battery_init);
module_exit(rk2918_battery_exit);

MODULE_DESCRIPTION("Battery detect driver for the rk2918");
MODULE_AUTHOR("luowei lw@rock-chips.com");
MODULE_LICENSE("GPL");

