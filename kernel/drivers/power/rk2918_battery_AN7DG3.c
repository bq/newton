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
#include <linux/slab.h>
#include <linux/earlysuspend.h>

#if 0
#define DBG(x...)   printk(x)
#else
#define DBG(x...)
#endif

#define RK29_USB_CHARGE_SUPPORT

extern char usb_flag_detect;
int rk29_battery_dbg_level = 0;

/*******************以下参数可以修改******************************/
#define	TIMER_MS_COUNTS		            50	//定时器的长度ms

#define	SLOPE_SECOND_COUNTS	            15	//统计电压斜率的时间间隔s
#define	DISCHARGE_MIN_SECOND	        85	//最短放电电1%时间
#define	CHARGE_MIN_SECOND	            70	//最短充电电1%时间
#define	CHARGE_MID_SECOND	            120	//最短充电电1%时间
#define	CHARGE_MAX_SECOND	            150	//最长充电电1%时间
#define CHARGE_FULL_DELAY_TIMES         5  //充电满检测防抖时间
#define USBCHARGE_IDENTIFY_TIMES        3   //插入USB混流，pc识别检测时间
#define STABLE_SECOND					8  //check ok 电平会晃动。。
#define SHUTDOWN_SECOND					20
#define SPEEDLOSE_SECOND                120 //play game rapid down

#define	NUM_VOLTAGE_SAMPLE	            ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//存储的采样点个数
#define	NUM_DISCHARGE_MIN_SAMPLE	    ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	//存储的采样点个数
#define	NUM_CHARGE_MIN_SAMPLE	        ((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    //存储的采样点个数
#define	NUM_CHARGE_MID_SAMPLE	        ((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	    //存储的采样点个数
#define	NUM_CHARGE_MAX_SAMPLE	        ((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	    //存储的采样点个数
#define NUM_CHARGE_FULL_DELAY_TIMES     ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	//充电满状态持续时间长度
#define NUM_USBCHARGE_IDENTIFY_TIMES    ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	//充电满状态持续时间长度
#define NUM_STABLE_SAMPLE				((STABLE_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SHUTD0WN_SAMPLE             ((SHUTDOWN_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SPEEDLOSE_SAMPLE  			((SPEEDLOSE_SECOND * 1000) / TIMER_MS_COUNTS)

#define BAT_2V5_VALUE	        2500
#define BATT_MAX_VOL_VALUE	    4180	//满电时的电池电压	 FOR A7
#define	BATT_ZERO_VOL_VALUE     3400	//关机时的电池电压
#define BATT_NOMAL_VOL_VALUE    3800

//定义ADC采样分压电阻，以实际值为准，单位K
#define BAT_PULL_UP_R           200
#define BAT_PULL_DOWN_R         200

#define BAT_ADC_TABLE_LEN       11

//台电项目插拔充电器140mv的波动,插USB 75mv / 100mv的波动。
static int adc_raw_table_bat[BAT_ADC_TABLE_LEN] = //放电记录来填写
{
#if defined(CONFIG_MACH_RK29_ACH7)
	3500, 3579, 3649, 3676, 3694, 3731, 3789, 3856, 3927, 4007, 4150
#elif defined(CONFIG_MACH_RK29_ACH8)
    3530, 3597, 3628, 3641, 3660, 3697, 3747, 3809, 3879, 3945, 4050
#elif defined(CONFIG_MACH_RK29_MSI735)
    3500, 3545, 3576, 3615, 3660, 3715, 3770, 3830, 3870, 3900, 4035
    //电容和电压基本是线性的关系可以均分等分
#elif defined(CONFIG_MACH_RK29SDK_DDR3)
	3500, 3588, 3640, 3680, 3693, 3738, 3761, 3827, 3897, 3970, 4010
#endif
};

static int adc_raw_table_ac[BAT_ADC_TABLE_LEN] = 
{
#if defined(CONFIG_MACH_RK29_ACH7)
	//3760, 3886, 3964, 3989, 4020, 4062, 4123, 4180, 4189, 4190, 4190      //实际测量值
	3691, 3760, 3800, 3827, 3845, 3885, 3950, 4007, 4078, 4158, 4300//4185//4301
#elif defined(CONFIG_MACH_RK29_ACH8)
    3680, 3747, 3778, 3791, 3810, 3847, 3897, 3959, 4050, 4095, 4210//4185//4301
#elif defined(CONFIG_MACH_RK29_MSI735)
    3690, 3714, 3740, 3771, 3805, 3855, 3915, 3940, 3995, 4027, 4080 
#elif defined(CONFIG_MACH_RK29SDK_DDR3)
   // 3740, 3790, 3832, 3863, 3873, 3895/*f*/, 3920/*f*/, 3982, 4054, 4110, 4195 
   3740, 3810, 3842, 3880, 3920, 3940/*f*/, 3966/*f*/, 4022, 4077, 4130, 4165
#endif
};

extern char usb_insert;

static int adc_raw_table_usb[BAT_ADC_TABLE_LEN] = 
{
    //3650, 3690, 3720, 3635, 3750, 3787, 3800,3850, 3899, 3970, 4030//4185//4301
    3607, 3680, 3705, 3745, 3780, 3818, 3850, 3895, 3953, 4000, 4066
};


static int shutdownvoltage = 3500;
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
static int gBatUsbChargeFlag = 0;
static int gBatUsbChargeCnt = 0;
static int shutdownflag = 0;
static int entercount = 0;

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

static struct regulator *pChargeregulator;
static int gBatChargeStatus =0;
static int gUsbCharge = 0;
static int gBatStatusBack =  POWER_SUPPLY_STATUS_UNKNOWN;

static int openfailflag = 0;
static uint8_t openfailcount = 20;  //  文件打开失败的话，重复次数

static int resumeflag = 0;

static struct wake_lock battery_wake_lock;



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
	
#ifdef RK29_USB_CHARGE_SUPPORT
	struct power_supply usb;
#endif
	struct power_supply ac;
    
    int dc_det_pin;
    int batt_low_pin;
    int charge_set_pin;
	int charge_big_charge;
	int charge_big_level;
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

#define BATT_FILENAME "/data/bat_last_capacity.dat"

#include <linux/fs.h>

static void rk2918_batscan_timer(unsigned long data);

static ssize_t rk2918_battery_startget_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
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

	return len;

}

static DEVICE_ATTR(startget,0666,NULL,rk2918_battery_startget_store);


static int rk2918_get_bat_capacity_raw(int BatVoltage);
int lastlost = 0;
static int rk2918_battery_load_capacity(void)
{
    int i;
    int tmp = 0;
	int  loadcapacity = 0;
	int  truecapacity = 0;
    char value[11];
    static char lastthree[6]= {0};
	char* p = value;
	int flag = dwc_vbus_status();
    struct file* fp = filp_open(BATT_FILENAME,O_RDONLY,0);
  
    
    //get true capacity
    for (i = 0; i < 30; i++)
    {
        tmp += adc_sync_read(gBatteryData->client);
        mdelay(1);
    }
    tmp = tmp / 30;
    tmp = (tmp * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
    truecapacity = rk2918_get_bat_capacity_raw(tmp);
 	
	
	if(IS_ERR(fp))
    {
		printk("hj---->open file /data/bat_last_capacity.dat failed\n");
		printk("truecapacity = %d\n", truecapacity);
		
		if(truecapacity>=100)
			truecapacity = 100;
	 	if(truecapacity==0)
			truecapacity=1;
#if 0
	if (flag == 1)
	{
		if (truecapacity > 60)
			truecapacity -= 35;
	}
#endif
		openfailflag = 1;

		if (openfailcount <= 5)
		{
			lastthree[openfailcount-1] = truecapacity;

			if (openfailcount == 1)
			{
				tmp = 0;
				for (i=0;i<5;i++)
				{
					tmp += lastthree[4-i];	
					printk("%s...............%d\n",__func__,tmp);
				}

				truecapacity = tmp/5;
				printk("%s...............%d\n",__func__,tmp);

			}
		}
		return truecapacity;
	}
	else
	{
		openfailflag = 0;
		openfailcount = 0;
	}


	kernel_read(fp,0,value,10);
    filp_close(fp,NULL);

	value[10]=0;
	while(*p)
	{
	    if(*p==0x0d)
	    {
			*p=0;
			break;
		}
		p++;
	}	
	sscanf(value,"%d",&loadcapacity);
	printk("hj---->loadcapacity = %d, truecapacity = %d\n",loadcapacity, truecapacity);
	
	//如果从文件中读取的电压比实际的高很多的话，说明是长时间放置导致放电
#if 0
	if (loadcapacity > truecapacity)
	{
	    if ((loadcapacity - truecapacity) > 15)
	    {
	        loadcapacity = truecapacity;
	    }
	}
	else
#endif

	{

		if (((truecapacity-loadcapacity) >= 25))
		{

			if (truecapacity < 30)
			{
				if (loadcapacity < 10)
				{
					loadcapacity = truecapacity/4;
				}
			}
			else
			{
				if (truecapacity < 80)
					loadcapacity = truecapacity - 8;
				else
					loadcapacity = truecapacity;
				
			}

		}
	}
	    
	if (loadcapacity == 0)
	{
		loadcapacity = 1;
	}
	else
	{
		if (loadcapacity >= 100)
		{
			loadcapacity = 99;
		}
	}
/*
	if (gpio_get_value(gBatteryData->charge_ok_pin) ==  gBatteryData->charge_ok_level)
	{
		loadcapacity = 99;
	}

	if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
	{	
		loadcapacity = 100;
	}

*/
	lastlost = loadcapacity;
	return loadcapacity;
}


void rk2918_charge_enable(void)
{
	printk("charging  with charger ,start to large charege\n");
   if (gBatteryData->charge_set_pin != INVALID_GPIO)
    {
       gpio_direction_output(gBatteryData->charge_set_pin, gBatteryData->charge_set_level);
    }
}
#define POWER_ON_PIN    RK29_PIN4_PA4

void rk2918_charge_enable_bl(char flag)
{
	int ret = 0;
#if 0	
	ret = gpio_request(RK29_PIN6_PB4, NULL);
	if (ret) {
		printk("failed to request charge_set  gpio...........error...\n");		
	}
#endif

  	switch (flag)
  	{
  	case 1:
  		gpio_direction_output(RK29_PIN6_PB4, 1);
  		is_charge = 2;
		printk("%s..start to large charge\n",__FUNCTION__);	
		break;
	case 0:
		if(gpio_get_value(RK29_PIN4_PA2) == GPIO_LOW)					
		{	
			printk("%s..low battery: powerdown\n",__FUNCTION__);
			while(1)
			{
				gpio_set_value(POWER_ON_PIN,GPIO_LOW);	
				mdelay(100);
			}
		}
		else
		{
	  		gpio_direction_output(RK29_PIN6_PB4, 0);
	  		is_charge = 1;
	  	}
		printk("%s..with usb not charge  %d\n",__FUNCTION__,gpio_get_value(RK29_PIN4_PA2));
		break;
	default:
		if(gpio_get_value(RK29_PIN4_PA2) == GPIO_LOW)					
		{	
			printk("%s..low battery: powerdown\n",__FUNCTION__);
			gpio_set_value(POWER_ON_PIN,GPIO_LOW);					
		}
		else
		{
			printk("%s ...nothing\n",__FUNCTION__);
			is_charge = 0;
		}
  	}

   
}

EXPORT_SYMBOL(rk2918_charge_enable_bl);


static void rk2918_charge_disable(void)
{
    if (gBatteryData->charge_set_pin != INVALID_GPIO)
    {
       gpio_direction_output(gBatteryData->charge_set_pin, 1 - gBatteryData->charge_set_level);
    }
}


void rk2918_get_charge_status(void)
{
    int charge_on = 0;
    int tmp = get_msc_connect_flag();
	int flag = dwc_vbus_status();
	static  char  last_flag = 0;


	//printk("%s...........get_msc_connect_flag=%d,dwc_vbus_status=%d\n",__FUNCTION__,tmp,flag);
#ifdef RK29_USB_CHARGE_SUPPORT
    if ((gpio_get_value(RK29_PIN0_PA0) == 0) &&(charge_on == 0) )
    {
        //if (suspend_flag) return;
 #if 1           
        if (1 == flag)         //USB PC
        {   
			charge_on = 1;
			//printk("PC\n");
        } 
 
        else
        {
             if (2 == flag)  //充电器
            {
                charge_on = 1;
				//printk("charger\n");
            }
        }
#endif
#if 0
	if (usb_insert == 2)
	{
		charge_on = 1;
		//printk("%s.......charging  with charger\n",__FUNCTION__);
	}   
#endif 
    }
	
#endif
//printk("%s...dwc_vbus_status=%d,get_msc_connect_flag=%d\n",__FUNCTION__,dwc_vbus_status(),tmp);
   if (charge_on)
    {
        if((gBatChargeStatus !=1) || (last_flag != flag)) 
        {            
            gBatChargeStatus = 1;
			last_flag = flag;
            gBatStatusChangeCnt = 0;        //状态变化开始计数
            wake_lock(&battery_wake_lock);
			if (flag == 2)
			{
				rk2918_charge_enable();
			}		
			
        }

    } 
    else 
    {
        if(gBatChargeStatus != 0) 
        {
            gBatChargeStatus = 0;
            gBatStatusChangeCnt = 0;        //状态变化开始计数
            wake_unlock(&battery_wake_lock);
		    rk2918_charge_disable();
			
        }
    }

}

EXPORT_SYMBOL(rk2918_get_charge_status);

static void  rk2918_get_bat_status(struct rk2918_battery_data *bat)
{
	static int stable = 0;
	
  	rk2918_get_charge_status(); 
	if((gBatChargeStatus == 1))
	{
	        if (gBatteryData->charge_ok_pin == INVALID_GPIO)
	        {
	            printk("dc_det_pin invalid!\n");
	            return;
	        }	

			if ((gBatStatus == POWER_SUPPLY_STATUS_FULL) && (gpio_get_value(gBatteryData->charge_ok_pin) ==  gBatteryData->charge_ok_level))
			{
				return;
			}
		if ((gpio_get_value(gBatteryData->charge_ok_pin) ==  gBatteryData->charge_ok_level) && (gBatCapacity >= 90))	
		{
			gBatteryData->full_times++;
			stable = 0;
			
			if (gBatteryData->full_times >= NUM_CHARGE_FULL_DELAY_TIMES)
			{
				gBatStatus = POWER_SUPPLY_STATUS_FULL;
				gBatStatusBack=POWER_SUPPLY_STATUS_FULL;
	    	    gBatteryData->full_times = NUM_CHARGE_FULL_DELAY_TIMES + 1;			
			}
			else	
				gBatStatus = POWER_SUPPLY_STATUS_CHARGING;			
		}
		else 
		{
	    	    gBatStatus = POWER_SUPPLY_STATUS_CHARGING;			
	        
		}

		/*	
    		if(gBatStatus!=POWER_SUPPLY_STATUS_FULL)
    		{
		    	if ((gpio_get_value(gBatteryData->charge_ok_pin) == gBatteryData->charge_ok_level) && (gBatCapacity >= 90))
	        	{
	        		
	            		gBatteryData->full_times++;
	            		if (gBatteryData->full_times >= NUM_CHARGE_FULL_DELAY_TIMES)
			    	{	    		
			        	gBatStatus = POWER_SUPPLY_STATUS_FULL;
			        	gBatteryData->full_times = NUM_CHARGE_FULL_DELAY_TIMES + 1;
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
    		*/
	}
	else 
    	{
    	stable = 0;	
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
extern int suspend_flag;
static int rk2918_get_bat_capacity_raw(int BatVoltage)
{
	int i = 0;
	int capacity = 0;
	int *p = adc_raw_table_bat;
	int flag = dwc_vbus_status();

	if(gpio_get_value(RK29_PIN0_PA0)==0)//有USB插入。。
	{
	//	if(flag == 2)
		{
			p = adc_raw_table_ac;
			//printk("%s...ac\n",__FUNCTION__);
		}
	//	else
		{
	//		p = adc_raw_table_usb;
			//printk("%s...USB\n",__FUNCTION__);
		}
			
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

unsigned long batteryspendcnt = 0;
unsigned long batteryspendcnt1 = 0;
static int rk2918_battery_resume_get_Capacity(int deltatime)
{
	int i;
	int tmp = 0;
	int capacity = 0;
	int num = 0;
	int voltagetmp = 0;
	pSamples = &gBatVoltageSamples[0];
    
    for (i = 0; i < 50; i++)
    {
         voltagetmp = adc_sync_read(gBatteryData->client);
         *pSamples = (voltagetmp * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
	 	 tmp = tmp + *pSamples++;	
          mdelay(1);
    }

    tmp = tmp / 50;
    //tmp = (tmp * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
    capacity = rk2918_get_bat_capacity_raw(tmp);
    printk("rk2918_battery_resume:%d,%d,%d,%d\n",capacity,gBatCapacity,tmp,gBatChargeStatus);
    if (gBatChargeStatus == 1)
    {
        //if (deltatime > (100 - gBatCapacity) * CHARGE_MIN_SECOND)
        //    deltatime = (100 - gBatCapacity) * CHARGE_MIN_SECOND;
       // if (capacity > gBatCapacity + (deltatime / CHARGE_MIN_SECOND))       //采样电池容量偏差较大，将容量拉回
        //{
       //     capacity = gBatCapacity + (deltatime / CHARGE_MIN_SECOND);
      //  }
        if(capacity > gBatCapacity)
        {
        	if((capacity-gBatCapacity)>5)
				gBatCapacity = capacity;
			else
			   capacity=gBatCapacity ;
        }
        else 
        {            	
			capacity = gBatCapacity;		
        }

		if ((gpio_get_value(gBatteryData->charge_ok_pin) ==  gBatteryData->charge_ok_level))
			{
				capacity = 100;
				gBatStatus = POWER_SUPPLY_STATUS_FULL;
				 gBatteryData->full_times = NUM_CHARGE_FULL_DELAY_TIMES + 1;
			}
    }
    else
    {
        //if (deltatime > gBatCapacity * DISCHARGE_MIN_SECOND)
           // deltatime = gBatCapacity * DISCHARGE_MIN_SECOND;            
        //if (capacity < gBatCapacity - (deltatime / DISCHARGE_MIN_SECOND))    //采样电池容量偏差较大，将容量拉回
        //{
          //  capacity = gBatCapacity - (deltatime / DISCHARGE_MIN_SECOND);
        //}
        if (capacity < gBatCapacity)
        {
			if((capacity-gBatCapacity)>5)
				gBatCapacity = capacity;
			else
			    capacity=gBatCapacity ;
        }
        else 
        {        	
			 capacity=gBatCapacity ;
        }
    }
    if (capacity == 0) capacity = 1;
    if (capacity >= 100) capacity = 100;
    
    if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
   {
	    capacity = 100;
    }
		
    printk("rk2918_battery_resume: gBatVoltage = %d, gBatCapacity = %d, deltatime = %d, ktmietmp.tv.sec = %ul, capacity = %d,gFlagLoop=%d\n", 
           gBatVoltage, gBatCapacity, deltatime, batteryspendcnt, capacity,gFlagLoop);
    
    return capacity;
}
 

static int rk2918_get_bat_capacity_ext(int BatVoltage)
{
	int capacity = 0;
	int num = 0;
	int i = 0;
	static int lostcount = 0;
	uint8_t speedlose = 1;

	
    	capacity = rk2918_get_bat_capacity_raw(BatVoltage);
	capacitytmp = capacity;
	
	//充放电状态变化后，Buffer填满之前，不更新
    if (gBatStatusChangeCnt < NUM_VOLTAGE_SAMPLE)
    {
        capacity = gBatCapacity;
    }
        
    if (gBatChargeStatus == 1)
    {
 			
        if ((capacity > gBatCapacity) && (gBatCapacity < 99) && ((gBatCapacityUpCnt++) >= NUM_CHARGE_MIN_SAMPLE))
        {

            	capacity = gBatCapacity + 1; 
		     
        	 gBatCapacityUpCnt = 0;
             gBatHighCapacityChargeCnt = 0;
	      }
    	  else if ((capacity < gBatCapacity) && (gBatCapacity < 90))
        {    	
			  if (++gBatHighCapacityChargeCnt > NUM_CHARGE_MAX_SAMPLE*3)
			  {
				  //长时间内充电电压无变化，开始计时充电
				  capacity = gBatCapacity + 1;
				  gBatHighCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);
			  }
			  else
			  {
				  capacity = gBatCapacity;
			  }

        }
        else
        {
                 capacity = gBatCapacity;
        }

		if (capacity == 0)
		{
			capacity = 1;
		}


	if (gBatStatusBack == POWER_SUPPLY_STATUS_FULL)
	{	
		capacity = 100;
	}	
	else if((gBatStatusBack==POWER_SUPPLY_STATUS_UNKNOWN)&&(gBatCapacity>=100))
	{
		capacity = 99;
	}

	//shutdownvoltage = 3500;
	
    }    
    else
    {   
        //放电时,只允许电压下降
        if (capacity > gBatCapacity)
        {     
            	capacity = gBatCapacity;				
        }
   
        if ((capacity < gBatCapacity) && ((gBatCapacityDownCnt++) >= NUM_DISCHARGE_MIN_SAMPLE))
        {  
 /*
 [ 6377.140657] gBatStatus = 3, adc_val = 775, TrueBatVol = 3784,gBatVol = 3764, gBatCap = 53, captmp = 56, sec = 1293972374,full_times=0,chargeok=1,flag =1,dcdet=0
 [ 6378.263192] gBatStatus = 3, adc_val = 775, TrueBatVol = 3784,gBatVol = 3269, gBatCap = 27, captmp = 0, sec = 1293973200,full_times=0,chargeok=1,flag =1,dcdet=0
 [ 6397.263182] gBatStatus = 3, adc_val = 780, TrueBatVol = 3808,gBatVol = 3806, gBatCap = 27, captmp = 62, sec = 1293973219,full_times=0,chargeok=1,flag =1,dcdet=0
		if(capacity==0)
		{
			if((gBatCapacity-capacity)>3)
				capacity = gBatCapacity -(gBatCapacity - capacity)/2;
			else
				capacity =0;
		}		
		else		
*/
			capacity = gBatCapacity - 1;       					
			gBatCapacityDownCnt = 0;	  
        }
        else
        {
            capacity = gBatCapacity;	
        }

	if(capacity>=100)
		capacity = 99;
	
	gBatStatusBack = POWER_SUPPLY_STATUS_UNKNOWN;

#if 0
 		if (lostcount++ > NUM_SPEEDLOSE_SAMPLE)
 		{
			lostcount = 0;
			if (((lastlost-gBatVoltage)>20)&&(gBatVoltage >= 3400))// start play game
			{
				shutdownvoltage = 3400;	
				entercount++;
				printk("%s...lastlost=%d. entercount=%d enter game\n",__func__,gBatVoltage,entercount);
			}
			else
			{
				if (((gBatVoltage-lastlost)>20)&& (entercount>0))//exit game
				{
					shutdownvoltage = 3500;
					entercount--;
					printk("%s.. lastlost=%d..entercount=%d exit game\n",__func__,gBatVoltage,entercount);
				}
			}
			lastlost = gBatVoltage;	
 		}
#endif


// <8%capacity enter game will make mistake
// 		if (((gBatVoltage<=shutdownvoltage) && (lastlost-gBatVoltage)<=3) || (gBatVoltage <= 3300))
		if ((gBatVoltage<shutdownvoltage) && (gBatCapacity < 8)) 
 		{	
 			shutdownflag++;
 			if(shutdownflag >= NUM_SHUTD0WN_SAMPLE)
			{
				capacity = 0;
				printk("%s.........%d...voltage is too small....%d\n",__func__,__LINE__,shutdownflag);
				shutdownflag = NUM_SHUTD0WN_SAMPLE+1;
			}
 		}	

 
    }

	return capacity;
}

unsigned long AdcTestvalue = 0;
unsigned long AdcTestCnt = 0;
static void rk2918_get_bat_voltage(struct rk2918_battery_data *bat)
{
	int value;
	int i,*pStart = &gBatVoltageSamples[0],num = 0;

	value = gBatteryData->adc_val;
	AdcTestvalue = value;
    adc_async_read(gBatteryData->client);
    
	*pSamples++ = (value * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
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
	//gBatVoltage = (value * BAT_2V5_VALUE * 2) / 1024;
#if 0
	/*消除毛刺电压*/
	if(gBatVoltage >= BATT_MAX_VOL_VALUE + 10)
		gBatVoltage = BATT_MAX_VOL_VALUE + 10;
	else if(gBatVoltage <= BATT_ZERO_VOL_VALUE - 10)
		gBatVoltage = BATT_ZERO_VOL_VALUE - 10;
#endif
    //充放电状态变化时,开始计数	
	if (++gBatStatusChangeCnt > NUM_VOLTAGE_SAMPLE)  
	    gBatStatusChangeCnt = NUM_VOLTAGE_SAMPLE + 1;
}

int first_flag = 1;
static void rk2918_get_bat_capacity(struct rk2918_battery_data *bat)
{

    int deltatime = 0;
	struct timespec now;
	getnstimeofday(&now);
	deltatime = now.tv_sec-batteryspendcnt1;
	if (resumeflag == 1)
		printk("%s..batteryspendcnt1=%d  now.tv_sec=%d,deltatime=%d\n",__FUNCTION__,batteryspendcnt1,(unsigned long)now.tv_sec,deltatime);


	//if (first_flag || (openfailcount > 1)) 
	if (first_flag || (openfailflag && (openfailcount > 1))) 
	//多次打开，刚开始采样时不准，多次采样。。
	{
	     if(first_flag == 1)
	     		first_flag= 0;
	     		
	    openfailcount--;
	  	//printk("%s,first_flag=%d,openfailflag=%d,openfailcount=%d\n",__func__,first_flag,openfailflag,openfailcount);   		
	  	
	    gBatCapacity = rk2918_battery_load_capacity();
	    if (gBatCapacity == 0) gBatCapacity = 1;
	}

	else if ((resumeflag == 1) && (deltatime > 700) && (first_flag == 0))//处理休眠之后的电量回复,如果超过十分钟
	{		
			resumeflag = 0;
        	gBatCapacity = rk2918_battery_resume_get_Capacity(deltatime);
        
	}
	else
	{
			resumeflag = 0;
			gBatCapacity = rk2918_get_bat_capacity_ext(gBatVoltage);
        /*	
        	if(deltatime>1)
        	{
			if(gBatCapacity--<2)
				gBatCapacity  = 50;

			if(gBatCapacity==15)
				gBatCapacity = 4;
			batteryspendcnt = ktmietmp.tv.sec;
        	}
        */        
      }
	batteryspendcnt1 = now.tv_sec;
	
}

static void rk2918_battery_timer_work(struct work_struct *work)
{	
  int value = 0;
  int flag = 0;
  int dcdet = 0;
 
    rk2918_get_bat_status(gBatteryData);
	rk2918_get_bat_health(gBatteryData);
	rk2918_get_bat_present(gBatteryData);
	rk2918_get_bat_voltage(gBatteryData);
	rk2918_get_bat_capacity(gBatteryData);

	value = gpio_get_value(gBatteryData->charge_ok_pin);
	flag = gpio_get_value(RK29_PIN0_PA0);
	dcdet = gpio_get_value(gBatteryData->dc_det_pin);
	
	if (rk29_battery_dbg_level)
	{
    	if (++AdcTestCnt >= 20)
    	{
    	    AdcTestCnt = 0;
    	    printk("taidian-----gBatStatus = %d, adc_val = %d, TrueBatVol = %d,gBatVol = %d, gBatCap = %d, captmp = %d, sec = %lu,full_times=%d,chargeok=%d, \
					flag =%d,dcdet=%d,shutdownflag=%d  batteryspendcnt=%d\n", 
    	            gBatStatus, AdcTestvalue, ((AdcTestvalue * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R)), 
    	            gBatVoltage, gBatCapacity, capacitytmp, batteryspendcnt1,gBatteryData->full_times/20,value,flag,dcdet,shutdownflag,batteryspendcnt1);
    	}
    }

	

	/*update battery parameter after adc and capacity has been changed*/
	if((gBatStatus != gBatLastStatus) || (gBatPresent != gBatLastPresent) || (gBatCapacity != gBatLastCapacity)||(gBatCapacity==0))
	{
		//gNumSamples = 0;
		gBatLastStatus = gBatStatus;
		gBatLastPresent = gBatPresent;
		gBatLastCapacity = gBatCapacity;
		power_supply_changed(&gBatteryData->battery);
	}
}


static void rk2918_batscan_timer(unsigned long data)
{
	//printk("%s..........\n",__FUNCTION__);
	gBatteryData->timer.expires  = jiffies + msecs_to_jiffies(TIMER_MS_COUNTS);
	add_timer(&gBatteryData->timer);
	schedule_work(&gBatteryData->timer_work);	
}


#ifdef RK29_USB_CHARGE_SUPPORT
static int rk2918_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	charger =  CHARGER_USB;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = get_msc_connect_flag();
		printk("%s:%d\n",__FUNCTION__,val->intval);
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
			if ((gBatChargeStatus == 1) && (gBatCapacity != 100))
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
		val ->intval = gBatVoltage*1000;
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

#ifdef RK29_USB_CHARGE_SUPPORT
static enum power_supply_property rk2918_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
#endif

static enum power_supply_property rk2918_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

#ifdef CONFIG_PM
static int rk2918_battery_suspend_late(struct platform_device *dev, pm_message_t state)
{
	struct timespec now;
	unsigned long  deltatime = 0;
	//ktime_t	 last;
	//last = ktime_get();
	getnstimeofday(&now);
	deltatime = now.tv_sec-batteryspendcnt1;
	
	printk("%s..batteryspendcnt1=%d  now.tv_sec=%d,deltatime=%d\n",__FUNCTION__,batteryspendcnt1,(unsigned long)now.tv_sec,deltatime);

	//deltatime = last.tv64-batteryspendcnt1;
	/* things may have changed while we were away */
	//printk("%s..batteryspendcnt1=%d  last.tv64=%d,deltatime=%d\n",__FUNCTION__,batteryspendcnt1,(unsigned long)last.tv64,deltatime);


	//ktime_get_ts(&now);
	//printk("%s now.tv_sec=%d  batteryspendcnt=%d\n",__FUNCTION__,batteryspendcnt,now.tv_sec);
		
		flush_scheduled_work();
		del_timer(&gBatteryData->timer);
		//resumeflag = 1;
		return 0;
}

static int rk2918_battery_resume_late(struct platform_device *dev, pm_message_t state)
{

		//flush_scheduled_work();
	struct timespec now;
	unsigned long  deltatime = 0;
	//ktime_t	 last;
	//last = ktime_get();
	//getnstimeofday(&now);
	//deltatime = now.tv_sec-batteryspendcnt1;
	
	//printk("%s..batteryspendcnt1=%d  now.tv_sec=%d,deltatime=%d\n",__FUNCTION__,batteryspendcnt1,(unsigned long)now.tv_sec,deltatime);

	//deltatime = last.tv64-batteryspendcnt1;
	//* things may have changed while we were away */
	//printk("%s..batteryspendcnt1=%d  last.tv64=%d,deltatime=%d\n",__FUNCTION__,batteryspendcnt1,(unsigned long)last.tv64,deltatime);

	//ktime_get_ts(&now);
	//printk("%s now.tv_sec=%d  batteryspendcnt=%d\n",__FUNCTION__,batteryspendcnt,now.tv_sec);
	setup_timer(&gBatteryData->timer, rk2918_batscan_timer, (unsigned long)gBatteryData);
	gBatteryData->timer.expires  = jiffies + 150;
	add_timer(&gBatteryData->timer);
		resumeflag = 1;
		return 0;
}

static int rk2918_battery_suspend()//(struct platform_device *dev, pm_message_t state)
{
	/* flush all pending status updates */
	struct timespec now;
	ktime_get_ts(&now);
	printk("%s now.tv_sec=%d  batteryspendcnt=%d\n",__FUNCTION__,batteryspendcnt,now.tv_sec);

	unsigned long  deltatime = 0;
	//ktime_t	 last;
	//last = ktime_get();
	getnstimeofday(&now);
	deltatime = now.tv_sec-batteryspendcnt;
	
	printk("%s..batteryspendcnt=%d  now.tv_sec=%d,deltatime=%d\n",__FUNCTION__,batteryspendcnt,(unsigned long)now.tv_sec,deltatime);

	//deltatime = last.tv64-batteryspendcnt;
	/* things may have changed while we were away */
	//printk("%s..batteryspendcnt=%d  last.tv64=%d,deltatime=%d\n",__FUNCTION__,batteryspendcnt,(unsigned long)last.tv64,deltatime);

	ktime_get_ts(&now);
	printk("%s now.tv_sec=%d\n",__FUNCTION__,now.tv_sec);
	
#if 0
[   61.404403] rk2918_battery_suspend..batteryspendcnt=1293964072  now.tv_sec=1293964072,deltatime=0
[   61.428840] rk2918_battery_suspend..batteryspendcnt=1293964072  last.tv64=1276406522,deltatime=-17557550
[   61.468607] rk2918_battery_suspend now.tv_sec=61
#endif

	return 0;
}

static int rk2918_battery_resume()//(struct platform_device *dev)
{

	struct timespec now;	
	ktime_get_ts(&now);
	printk("%s now.tv_sec=%d  batteryspendcnt=%d\n",__FUNCTION__,batteryspendcnt,now.tv_sec);

	unsigned long  deltatime = 0;
	//ktime_t	 last;
	//last = ktime_get();
	getnstimeofday(&now);
	
	deltatime = now.tv_sec-batteryspendcnt1;
	/* things may have changed while we were away */
	printk("%s..batteryspendcnt1=%d  now.tv_sec=%d,deltatime=%d\n",__FUNCTION__,batteryspendcnt1,(unsigned long)now.tv_sec,deltatime);

	//deltatime = last.tv64-batteryspendcnt1;
	/* things may have changed while we were away */
	//printk("%s..batteryspendcnt1=%d  last.tv64=%d,deltatime=%d\n",__FUNCTION__,batteryspendcnt1,(unsigned long)last.tv64,deltatime);

	ktime_get_ts(&now);
	printk("%s now.tv_sec=%d\n",__FUNCTION__,now.tv_sec);
#if 0
[   73.047133] rk2918_battery_resume..batteryspendcnt=1293964100  now.tv_sec=1293964100,deltatime=0
[   73.047163] rk2918_battery_resume..batteryspendcnt=1293964100  last.tv64=16181967,deltatime=-1277782133
[   73.047187] rk2918_battery_resume now.tv_sec=73
#endif

	//if (resumeflag == 1)
	//	schedule_work(&gBatteryData->timer_work);
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
static int  lockcount = 0;
static irqreturn_t rk2918_dc_wakeup(int irq, void *dev_id)
{   
    //gBatStatusChangeCnt = 0;        //状态变化开始计数
    
    disable_irq_nosync(gBatteryData->dc_det_irq);
    gBatStatusChangeCnt = 0; 
    queue_delayed_work(gBatteryData->wq, &gBatteryData->work, 0);
	
    return IRQ_HANDLED;
}


static irqreturn_t rk2918_batt_low_wakeup(int irq, void *dev_id)
{   
   
    disable_irq_nosync(gpio_to_irq(RK29_PIN4_PA2));

	 
	  if (gpio_get_value(RK29_PIN4_PA2) == GPIO_LOW)				  
	  {   
		  printk("%s..low battery: powerdown\n",__FUNCTION__);
		  gpio_set_value(POWER_ON_PIN,GPIO_LOW);				  
	  }
	  else
	  {
		 printk("%s..but not poweroff...maybe error\n",__FUNCTION__);
		 enable_irq(gpio_to_irq(RK29_PIN4_PA2));
	  }
    return IRQ_HANDLED;
}

static void rk2918_battery_work(struct work_struct *work)
{
    int ret,i;
    int irq_flag;

    printk("%s............>>>>>>>>>>>>>>>",__func__);
    rk28_send_wakeup_key();
    
    free_irq(gBatteryData->dc_det_irq, gBatteryData);
    irq_flag = (!gpio_get_value (gBatteryData->dc_det_pin)) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
	ret = request_irq(gBatteryData->dc_det_irq, rk2918_dc_wakeup, irq_flag, "rk2918_battery", gBatteryData);
	if (ret) {
		free_irq(gBatteryData->dc_det_irq, gBatteryData);
		printk("%s..........%d",__func__,__LINE__);
	}
	enable_irq(gBatteryData->dc_det_irq);
}

static void rk2918_battery_callback(struct adc_client *client, void *param, int result)
{
    gBatteryData->adc_val = result;
	return;
}


//POWER_ON_PIN   开机后，硬件自动拉高。
//给它赋值为0，自动关机
void rk2918_low_battery_check(void)
{
	int tmp = 0;   
	int  tmpcount = 0;    
	int  LowBatteryValue=adc_raw_table_bat[0];
	
	while(1)        
	{       
		if (usb_flag_detect == 2)
			break;
		tmp += adc_sync_read(gBatteryData->client);		    
		tmpcount++;		    
		mdelay(1);		    		    
		if(tmpcount>100)		   
		{			
			tmp = ((tmp /100)* BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);	
			if(tmp <LowBatteryValue )
//USB_iNT  RK29_PIN0_PA0   插入USB为低电平。。。 
//实现，插USB不关机
			{
				if (gpio_get_value(RK29_PIN0_PA0))
				{			 	
					printk("low battery without usb and charge: powerdown\n");        			
					{			    		
						gpio_set_value(POWER_ON_PIN,GPIO_LOW);		    		
					}	
					mdelay(1000);
					mdelay(1000);					
				}
				if (usb_flag_detect == 1)
				{
					printk("low battery with usb not  charge: waiting for\n");
					LowBatteryValue = 3700;	
					mdelay(1000);
					mdelay(1000);
					mdelay(1000);
					mdelay(1000);
					mdelay(1000);
					if (gpio_get_value(RK29_PIN0_PA0) == 1)
						usb_flag_detect = 0;
					mdelay(1000);
				}
				
			
			}			
			else			 
			{					
				break;						 
			}			 
			tmp =0;			 
			tmpcount =0;		    
		}	
	}
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
}
static ssize_t rk29_battery_dbg_show(struct class *cls,struct class_attribute *attr, char *_buf)
{
    return sprintf(_buf, "%d\n", rk29_battery_dbg_level);
}
static ssize_t rk29_battery_dbg_store(struct class *cls, struct class_attribute *attr,const char *_buf, size_t _count)
{
    rk29_battery_dbg_level = simple_strtoul(_buf, NULL, 16);
	int flag = dwc_vbus_status();
    printk("rk29_battery_dbg_level = %d  dwc_vbus_status=%d\n",rk29_battery_dbg_level,flag);
    
    return _count;
} 

static struct class *rk29_battery_dbg_class = NULL;
static CLASS_ATTR(rk29_battery_dbg, 0666, rk29_battery_dbg_show, rk29_battery_dbg_store);

#if 1
static void rk2918_battery_early_suspend(struct early_suspend *h)
{
	rk2918_battery_suspend();
}

static void rk2918_battery_early_resume(struct early_suspend *h)
{
	rk2918_battery_resume();
}

static struct early_suspend suspend_info = {
	.suspend = rk2918_battery_early_suspend,
	.resume = rk2918_battery_early_resume,
	.level = 200,
};
#endif

void  rk2918_charge_big_charge(char flag)
{

	printk("%s",__FUNCTION__);
	int charge_flag = dwc_vbus_status();
	
	if (charge_flag == 2)
	{
		if (flag == 1)
		{
			printk("with charging start to big charge when system enter early suspend\n");
			gpio_set_value(gBatteryData->charge_big_charge,gBatteryData->charge_big_level);
		}
		else
		{
			printk("with charging start to small charge when system exit early suspend\n");
			gpio_set_value(gBatteryData->charge_big_charge,1 - gBatteryData->charge_big_level);
		}
	}

}

EXPORT_SYMBOL(rk2918_charge_big_charge);

static int rk2918_battery_probe(struct platform_device *pdev)
{
	int ret;
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

	//charge_big_charge  //early suspend : big charge   high level. early resume:small charge  low lever
	if (pdata->charge_big_charge != INVALID_GPIO)
	{
    	ret = gpio_request(pdata->charge_big_charge, NULL);
    	if (ret) {
    		printk("failed to request charge_big_charge gpio\n");
    		goto err_free_gpio1;
    	}
	
    	gpio_pull_updown(pdata->charge_big_charge, GPIOPullUp);//important
    	ret = gpio_direction_output(pdata->charge_big_charge,1-pdata->charge_big_level);
    	if (ret) {
    		printk("failed to set gpio dc_det input\n");
    		goto err_free_gpio1;
    	}
    	data->charge_big_charge   = pdata->charge_big_charge;
    	data->charge_big_level = pdata->charge_big_level;
    }
	else
	{
			printk("%s..........not exist\n",__FUNCTION__);

	}
	
	//charge set for usb charge
	if (pdata->charge_set_pin != INVALID_GPIO)
	{
#if 1
	    	ret = gpio_request(pdata->charge_set_pin, NULL);
    		if (ret) {
    			printk("failed to request dc_det gpio\n");
	   			goto err_free_gpio1;
    		}
	    	gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
#endif

    	data->charge_set_pin = pdata->charge_set_pin;
    	data->charge_set_level = pdata->charge_set_level;
    	 switch (usb_flag_detect)
	{
		case  2:
			gpio_direction_output(pdata->charge_set_pin, pdata->charge_set_level);
	 		printk("open with charge :begint to large charge\n");	
			break;
		case  1:
			printk("open with computer usb\n");
			break;
		case  0:
			printk("open with nothing\n");
			break;
		default :
			printk("this error error \n\n\n\n");
	}	
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
	
#ifdef RK29_USB_CHARGE_SUPPORT
	data->usb.properties = rk2918_usb_props;
	data->usb.num_properties = ARRAY_SIZE(rk2918_usb_props);
	data->usb.get_property = rk2918_usb_get_property;
	data->usb.name = "usb";
	data->usb.type = POWER_SUPPLY_TYPE_USB;
#endif

	data->ac.properties = rk2918_ac_props;
	data->ac.num_properties = ARRAY_SIZE(rk2918_ac_props);
	data->ac.get_property = rk2918_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;

//735 不一样。。。	
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
	gBatteryData->full_times = 0;
	
//    irq_flag = (pdata->charge_ok_level) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
//	ret = request_irq(gpio_to_irq(pdata->charge_ok_pin), rk2918_battery_interrupt, irq_flag, "rk2918_battery", data);
//	if (ret) {
//		printk("failed to request irq\n");
//		goto err_irq_failed;
//	}

#if 0
    if (pdata->dc_det_pin != INVALID_GPIO)
    {
        irq_flag = (!gpio_get_value (pdata->dc_det_pin)) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
    	ret = request_irq(gpio_to_irq(pdata->dc_det_pin), rk2918_dc_wakeup, irq_flag, "rk2918_battery", data);
    	if (ret) {
    		printk("failed to request dc det irq\n");
    		goto err_dcirq_failed;
    	}
    	data->dc_det_irq = gpio_to_irq(pdata->dc_det_pin);
    	data->wq = create_rt_workqueue("rk2918_battery");
    	INIT_DELAYED_WORK(&data->work, rk2918_battery_work);
    	
    	enable_irq_wake(gpio_to_irq(pdata->dc_det_pin));
    }
 #endif
     	ret = request_irq(gpio_to_irq(pdata->batt_low_pin), rk2918_batt_low_wakeup, IRQF_TRIGGER_FALLING, "rk2918_battery_batt_low", data);
    	if (ret) {
    		printk("failed to request batt low  det irq\n");
    		goto err_dcirq_failed;
    	}
    	
		 enable_irq_wake(gpio_to_irq(pdata->batt_low_pin));

 	rk2918_low_battery_check();
	setup_timer(&data->timer, rk2918_batscan_timer, (unsigned long)data);
	//data->timer.expires  = jiffies + 2500;
	//add_timer(&data->timer);
	//printk("%s-%8x   %8x  %8x  %8x\n\n\n\n\n\n\n ",__FUNCTION__,data,gBatteryData,&data->timer,&gBatteryData->timer);
    
    
      rk29_battery_dbg_class = class_create(THIS_MODULE, "rk29_battery");
      class_create_file(rk29_battery_dbg_class,&class_attr_rk29_battery_dbg);

     ret = device_create_file(&pdev->dev, &dev_attr_startget);
     if (ret)
     {
	
		printk("make a mistake in creating devices  attr file\n\n ");
     }

	//register_early_suspend(&suspend_info);

	wake_lock_init(&battery_wake_lock,WAKE_LOCK_SUSPEND,"battery");
	
	
	printk(KERN_INFO "rk2918_battery: probe  ok...yj.TODAY now .\n");
	
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
	.suspend	= rk2918_battery_suspend_late,
	.resume		= rk2918_battery_resume_late,
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

subsys_initcall_sync(rk2918_battery_init);
module_exit(rk2918_battery_exit);

MODULE_DESCRIPTION("Battery detect driver for the rk2918");
MODULE_AUTHOR("luowei lw@rock-chips.com");
MODULE_LICENSE("GPL");

