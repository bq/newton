/* drivers/input/touchscreen/touchplus_ts.c
 *
 * Copyright (C) 2011 TouchPlus, Inc.
 * http://www.touchplus.com.tw
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
//#include <linux/smp_lock.h>
#include <linux/delay.h>
#include <linux/slab.h>  
#include <mach/gpio.h>
#include <linux/kallsyms.h>
#include <linux/miscdevice.h>
#include "touchplus_ts.h"
#include <linux/device.h>
#include <mach/hardware.h>
#include <mach/iomux.h>
#include <mach/irqs.h>
#include <mach/rk29_iomap.h>
#include <mach/board.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/async.h>
#include <linux/workqueue.h>
#include <linux/input/mt.h>

#include <linux/types.h>       //dev_t 
#include <linux/cdev.h>        //struct cdev 
#include <linux/fs.h>          //alloc_chrdev_region()
#include <linux/device.h>      //class_create()



#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "Kevin Ma <kevin.ma@touchplus.com.tw>"
#define DRIVER_DESC "TouchPlus I2C Touchscreen Driver with tune fuction"
#define DRIVER_LICENSE "GPL"

#define TOUCHPLUS_DEBUG 1
#define TOUCH_UP 1
#define TOUCH_DOWN 0
#define MAX_SUPPORT_POINT 5
#define RESET_PIN_SUPPORT
#define I2C_SPEED 200*1000

//#define TOUCHSCREEN_MAXX 1024
//#define TOUCHSCREEN_MAXY 600
//#define MAX_FINGER 5
#if 0
#define gpio_shutdown ((GPIOD_bank_bit2_24(23)<<16) |GPIOD_bit_bit2_24(23))
#define gpio_irq ((GPIOD_bank_bit2_24(24)<<16) |GPIOD_bit_bit2_24(24))
#endif


//====================================
//==========================================================
// touch screen runing mode
#define CMD_NORMAL                  0
#define CMD_SELF_RAWDATA            1
#define CMD_MUTUAL_RAWDATA          2
#define CMD_CONFIG_READ             3
#define CMD_CONFIG_WRITE            4
#define CMD_CRC_CHECK               5
#define CMD_PROGRAM_WRITE           6
#define CMD_SELF_DIST_RAWDATA       7
#define CMD_MUTUAL_DIST_RAWDATA     8

//#define DEBUG
#ifdef DEBUG
#define DEBUG_MSG(msg...) printk(msg) 
#else
#define DEBUG_MSG(msg...)  
#endif

//#define DEBUG_REPORT
#ifdef DEBUG_REPORT
#define DEBUG_COORDINATE(msg...) printk(msg) 
#else
#define DEBUG_COORDINATE(msg...)
#endif

//#define BOOTLOADER_DEBUG
#ifdef BOOTLOADER_DEBUG
#define DEBUG_BL(msg...) printk(msg) 
#else
#define DEBUG_BL(msg...)
#endif

// touch screen  register list
#define MSI_REPORT                      0x00
#define MSI_VERSION                     52
#define MSI_SPECOP                      (MSI_VERSION	+ 3)	// 55
#define MSI_CMD_MODE                    (MSI_SPECOP	+ 1)	// 56
#define MSI_X_NBR                       (MSI_CMD_MODE	+ 1)	// 57
#define MSI_Y_NBR                       (MSI_X_NBR      + 1)	// 58
#define MSI_KEY_NBR                     (MSI_Y_NBR     	+ 1)	// 59
#define MSI_UNLOCK                      (MSI_KEY_NBR    + 1)	// 60
#define MSI_POWER_MODE		            (MSI_UNLOCK     + 1)	// 61
#define MSI_INT_MODE		            (MSI_POWER_MODE	+ 1)	// 62
#define MSI_INT_WIDTH                   (MSI_INT_MODE   + 1)	// 63
#define MSI_X_RES                       (MSI_INT_WIDTH  + 1)	// 64
#define MSI_Y_RES                       (MSI_X_RES      + 2)	// 66
#define MSI_UPDATE_POS	                (MSI_Y_RES      + 2)	// 68

// definitions
#define ISP_UNLOCK_CODE        0x5d
#define AUTOTUNE_SUCCESS       0x55
#define CMD_AUTOTUNE           0x03
#define PACK_SIZE			   144    // tpf encryption length
#define FACTORY_DATA_BUF_SIZE  144    // raw data buffer maximum  size
#define FACTORY_DATA_BUF_ADDR  96     // raw data address

// touch screen register structure

typedef struct {
	unsigned char addr;
	unsigned char val;
}msi_reg;

typedef struct {
	unsigned char addr;
	unsigned char len;
	unsigned char buf[250];
}msi_regs;
//
// global data
static unsigned char ts_buf[256];
static unsigned char runmode = CMD_NORMAL; 
static unsigned char autotune_status = 0;
static unsigned char interrupt_flag = 0;
static unsigned int  temp_val;
//====================================

// device data declaration
dev_t                  devid;
static struct cdev     *touchplus_cdev; 
static int             touchplus_Major = 0;
static int             touchplus_Minor = 0;
static struct class    *touchplus_class = NULL;


#if 1
/*add by hjc*/
#define TOUCH_RESET_PIN    RK29_PIN6_PC3
#define TOUCH_INT_PIN         RK29_PIN0_PA2
static u8 calibration_flag = 0;
static int calibration_time =15;
//static int calibration_test = 7;
//static u8 FLAG=0;
static int irq_flag;
static int GPIO_IRQ;

//static int irq_suspend = 0;

// report data define
typedef struct {
    unsigned short wx;
    unsigned short wy;
    unsigned char  id;
}__attribute__ (( packed ))touchplus_finger_status;


typedef struct {
    unsigned char touch_status;
    unsigned char buttons;
    touchplus_finger_status finger_status[MAX_FINGER];
}__attribute__ (( packed ))touchplus_touch_data;

static void touchplus_hw_init(void)
{
        int ret;

        printk("%s\n", __FUNCTION__);

        if(TOUCH_RESET_PIN != INVALID_GPIO){
                gpio_request(TOUCH_RESET_PIN, "touchplus_reset");
                gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
        }

        if(TOUCH_INT_PIN != INVALID_GPIO){
                ret = gpio_request(TOUCH_INT_PIN, "touchplus_irq");
                if(ret != 0){
                        gpio_free(TOUCH_INT_PIN);
                        printk("%s: touchplus irq request err\n", __func__);
                }
                else{
                        gpio_direction_input(TOUCH_INT_PIN);
                        gpio_pull_updown(TOUCH_INT_PIN, GPIO_HIGH);
                }
        }
}

static void touchplus_hw_remove(void)
{
        int ret;

        printk("%s\n", __FUNCTION__);

        if(TOUCH_RESET_PIN != INVALID_GPIO){
                gpio_free(TOUCH_RESET_PIN);
                //gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
        }

        if(TOUCH_INT_PIN != INVALID_GPIO){
                gpio_free(TOUCH_INT_PIN);
        }
}
static void touchplus_hw_reset(int reset)
{
        printk("%s: %d\n", __FUNCTION__, reset);

        if(TOUCH_RESET_PIN != INVALID_GPIO){
                if(reset){
                        gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
                }
                else{
                        gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
                }
        }
}

/*read the touchplus register ,used i2c bus*/
static int touchplus_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	ret =i2c_master_reg8_recv(client, reg, buf, len, I2C_SPEED);
	if(ret < 0)
		printk("touchplus_ts_work_func:i2c_transfer fail =%d\n",ret);
	return ret;
}
/* set the touchplus registe,used i2c bus*/
static int touchplus_write_regs(struct i2c_client *client, u8 reg, u8 const buf[], unsigned short len)
{
	int ret;
	ret = i2c_master_reg8_send(client,reg, buf, len, I2C_SPEED);
 	if (ret < 0) {
	  printk("touchplus_ts_work_func:i2c_transfer fail =%d\n",ret);
    }
	return ret;
}
#endif
static struct i2c_driver touchplus_i2c_ts_driver;
static struct workqueue_struct *touchplus_wq;
struct touchplus_i2c_ts_data
{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work work;
	int irq;
	u8 iobuf[200];
	//struct miscdevice miscdev;
	struct early_suspend early_suspend;//add by hjc
};

#ifdef CONFIG_HAS_EARLYSUSPEND  // add by hjc
static void touchplus_ts_early_suspend(struct early_suspend *h);
static void touchplus_ts_late_resume(struct early_suspend *h);
#endif

/* for handy access */
static struct touchplus_i2c_ts_data *g_tsd =NULL;
#if 0
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];
	msgs[0].scl_rate = I2C_SPEED;

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len;
	msgs[1].buf=&buf[0];
	msgs[1].scl_rate = I2C_SPEED;
	
	ret=i2c_transfer(client->adapter,msgs,2);
	return ret;
}

static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;
	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;
	msg.scl_rate = I2C_SPEED;
	ret=i2c_transfer(client->adapter,&msg,1);
	return ret;
}
#endif
#if 0
static void touchplus_touchpos_dump( struct i2c_client *client)
{
	int i, npoints;
	struct touchplus_i2c_ts_data *tsd = dev_get_drvdata(&client->dev);
	
	tsd->iobuf[0] = 0;
	i2c_read_bytes( client, tsd->iobuf, 52);
	npoints = tsd->iobuf[0];
	for ( i=0; i<npoints;i++) {
		const u8* posp = tsd->iobuf + 2 + i*5;
		int x = posp[1]<<8 | posp[0];
		int y = posp[3]<<8 | posp[2];
		int fingerid= posp[4];
		printk("%02X: (%d,%d)\n", fingerid, x, y);
	}
}
#endif
static  char last_num = 0;
static char  last_ID[MAX_SUPPORT_POINT] = {0};
static void report_position(struct work_struct *work)
{
	//printk("%s\n",__FUNCTION__);
	struct touchplus_i2c_ts_data *tsd = container_of(work,
			struct touchplus_i2c_ts_data, work.work);
	int i,j=0,z=0;
	char track_id[MAX_SUPPORT_POINT] = {0};
	
	unsigned char fingerbuf[1];
	tsd->iobuf[0] = 2;

        interrupt_flag = 1;  //touchplus
	
	memset(tsd->iobuf, 0, sizeof(tsd->iobuf));
	touchplus_read_regs(tsd->client,0x00,tsd->iobuf,5*sizeof(touchplus_touch_data)+2);
        memcpy(ts_buf,tsd->iobuf,sizeof(touchplus_touch_data));
	fingerbuf[0] = tsd->iobuf[0];
	//printk("====fingerbug[0]=%d====\n",fingerbuf[0]);
	if(fingerbuf[0] != 0){
		for(i=0;i<fingerbuf[0];i++)				
		{
			const u8* posp = tsd->iobuf + i*5+2;
			int x = posp[1]<<8 | posp[0];
			int y = posp[3]<<8 | posp[2];
			int fingerid= posp[4];
			  if(x > TOUCHSCREEN_MAXX || y > TOUCHSCREEN_MAXY
			  	 || fingerid > MAX_FINGER) continue;
			fingerid = fingerid-1;
			track_id[j++] = fingerid;
						
			//printk("fingerid=%d: (%d,%d,%d)\n", fingerid, x, y,posp[4]);
				input_mt_slot(tsd->input_dev, fingerid);
				input_mt_report_slot_state(tsd->input_dev, MT_TOOL_FINGER, true);
				input_report_abs(tsd->input_dev, ABS_MT_TOUCH_MAJOR, 10);
				input_report_abs(tsd->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(tsd->input_dev, ABS_MT_POSITION_Y, y);
				//printk("i=%d,TOUCH_DOWN\n",i);
		}
		
		}
	
	//printk("%d,%d,%d,%d,%d\n",finger_status[0],finger_status[1],finger_status[2],finger_status[3],finger_status[4]);

		if (last_num >= fingerbuf[0])
		{
			for (i=0; i<last_num; i++)
			{
				if (last_ID[i] != track_id[z])
				{
					input_mt_slot(tsd->input_dev, last_ID[i]);
					input_mt_report_slot_state(tsd->input_dev, MT_TOOL_FINGER, false);
					//printk("touch up ,finger id =%d\n",last_ID[i]);
				}
				else
				{
					z++;
				}
			}	

		}		

	input_sync( tsd->input_dev);
	//enable_irq(tsd->irq);
	last_num = fingerbuf[0];
	memcpy(last_ID,track_id,MAX_SUPPORT_POINT);
	
}

static void touchplus_ts_poscheck(struct work_struct *work)
{
	int tmp,i;
	struct touchplus_i2c_ts_data *tsd = container_of(work,
			struct touchplus_i2c_ts_data, work.work);
	//report_position(work);	
	tmp=gpio_get_value(GPIO_IRQ);
	//printk("%s,gpio=%d\n",__FUNCTION__,tmp);
	if(!tmp){
		report_position(work);
		queue_delayed_work(touchplus_wq, &tsd->work, msecs_to_jiffies(10));//5->10
		//queue_delayed_work(touchplus_wq, &tsd->work, 1);
	}else{
		for(i=0;i<5;i++){
			input_mt_slot(tsd->input_dev, i);
			input_mt_report_slot_state(tsd->input_dev, MT_TOOL_FINGER, false);
		}

		//printk("TOUCH_UP\n");
		last_num = 0;
		input_sync(tsd->input_dev);

		enable_irq(tsd->irq);
	}
}

static irqreturn_t touchplus_ts_isr(int irq, void *dev_id)
{
	//printk("%s\n",__FUNCTION__);
	struct touchplus_i2c_ts_data *tsdata = dev_id;
	
   if(runmode == CMD_NORMAL && !autotune_status) {   //Touchplus
	disable_irq_nosync(irq);
	//queue_work( touchplus_wq, &tsdata->work.work);
	queue_delayed_work(touchplus_wq, &tsdata->work, 0);
   	}
	return IRQ_HANDLED;
}

static int touchplus_ts_open(struct input_dev *dev)
{
	//struct touchplus_i2c_ts_data *tsdata = 
	//	(struct touchplus_i2c_ts_data *) input_get_drvdata( dev);
	printk("in %s\n", __FUNCTION__);
	return 0;
}


static void touchplus_ts_close(struct input_dev *dev)
{
	//struct touchplus_i2c_ts_data *tsdata = 
	//	(struct touchplus_i2c_ts_data *) input_get_drvdata( dev);
	printk("in %s\n", __FUNCTION__);
}

/* touchplus miscdev implementation.  This is for low-level raw commands. */
static int touchplus_miscdev_open(struct inode *inode, struct file *filp)
{
    printk("%s():\n", __FUNCTION__);
    return 0;
}

static int
touchplus_miscdev_release(struct inode *inode, struct file *filp)
{
    printk("%s():\n", __FUNCTION__);
    return 0;
}
// ***************************************************************************
// read device. 
// 
// Params:
// none
//
// Return:
// 0: success.  -1:fail
// ---------------------------------------------------------------------------
static ssize_t
touchplus_miscdev_read(struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
	int ret = 1;
	//int i;
	

   // printk("%s(%d) %s\n", __FILE__,__LINE__,__FUNCTION__);

	
	switch(runmode)
	{
		case CMD_NORMAL:
		if(interrupt_flag)
        {
              interrupt_flag = 0;
			  ret = copy_to_user(buf,ts_buf,sizeof(touchplus_touch_data));
			//ret = copy_to_user(buf,ts_buf,MAX_FINGER*5+2);
			  // ret =0: success.   ret > 0(fail copy bytes) : fail.
			  if(!ret) {
			 //   for(i=0; i<sizeof(touchplus_touch_data); i++)
			   // printk("report data: %d ",ts_buf[i]);
			  }
			  else ret = -1;
        }
        break;
		
		case CMD_SELF_RAWDATA:
		{
		 //unsigned char buff[2];
     temp_val =	count;
         if((!temp_val) || (temp_val > FACTORY_DATA_BUF_SIZE)) {ret = -1; break;}
		 while(gpio_get_value(GPIO_IRQ));		
		 mdelay(10);
		
		 ret = touchplus_read_regs(g_tsd->client,FACTORY_DATA_BUF_ADDR, ts_buf, temp_val);
		 
         if(ret < 0) {
	        printk("i2c self read error\n");
			return -1;
	     }
		 ret = copy_to_user(buf,ts_buf,temp_val);
	     // ret =0: success.   ret > 0(fail copy bytes) : fail.
		 if(!ret) {
		 	    #if 0
		 	    printk("self raw read done:%d!", temp_val);
				
			    for(i=0; i<temp_val; i++)
			    printk("self data: %d ",ts_buf[i]);
				printk("\n");
				#endif
	     }
		 else ret = -1;
	    }
		break;
		
		case CMD_MUTUAL_RAWDATA:
		{
		 temp_val =	count;
         if((!temp_val) || (temp_val > FACTORY_DATA_BUF_SIZE)) {ret = -1; break;}
		 while(gpio_get_value(GPIO_IRQ));			
		 ret = touchplus_read_regs(g_tsd->client,FACTORY_DATA_BUF_ADDR, ts_buf, temp_val);
		 		 //if(ret != temp_val) {
		 		 if(ret < 0) {
	        DEBUG_MSG("i2c mutual read error\n");
			return -1;
	     }
		 ret = copy_to_user(buf,ts_buf,temp_val);
	     // ret =0: success.   ret > 0(fail copy bytes) : fail.
		 if(!ret) {
		 	    #if 0
		 	    DEBUG_MSG("mutual raw read done:%d!", temp_val);
				
			    for(i=0; i<temp_val; i++)
			    DEBUG_MSG("mutual data: %d ",ts_buf[i]);
				DEBUG_MSG("\n");
				#endif
	     }
		 else ret = -1;
	    }
		break;
		default: 
		break;
	}
	
	return ret;
}

static int touchplus_calibrate( struct i2c_client *client);

// ***************************************************************************
// touch panel autotune status. 
// 
// Params:
// none
//
// Return:
// 0: autotune done 1: autotune busy -1: i2c error. 
// ---------------------------------------------------------------------------
static int touchplus_ts_autotune_status(void){
	char buf[2];
	int ret = 0;
	
    ret = touchplus_read_regs(g_tsd->client, MSI_SPECOP, buf, 1);
	if(ret < 0) { DEBUG_MSG("I2C read error!\n"); return -1; }
	if(buf[0] == AUTOTUNE_SUCCESS) { DEBUG_MSG("autotune done\n"); return 0;}
	else ret = 1;
	
	return ret;
}
// ***************************************************************************
// touch panel autotune start. 
// 
// Params:
// none
//
// Return:
// 0: success -1: iic error 
// ---------------------------------------------------------------------------
static int touchplus_ts_autotune(void){
	char buf[2];
	int ret = 0;    
    int cnt = 0;

    autotune_status = 1;
    disable_irq_nosync(g_tsd->irq);  // disable irq, don't need wait irq routine process done
	cancel_delayed_work_sync(&g_tsd->work);

    buf[0] = CMD_AUTOTUNE;
    ret = touchplus_write_regs(g_tsd->client, MSI_SPECOP, buf,1);

	if(ret < 0 ) { DEBUG_MSG("I2C write error!\n"); ret = -1; }
	else ret = 0;
	DEBUG_MSG("autotune start ...\n");
    cnt = 0;
    while(1)
    {
     ret = touchplus_ts_autotune_status();
	 if(ret <0) return -1;
	 if(ret == 0 ) break; //success
	 if(cnt++ > 100) { DEBUG_MSG("autotune timer out!\n"); return -1;}
	 mdelay(1000);
    }

	autotune_status = 0;

	enable_irq(g_tsd->irq);

	return ret;
}


// ***************************************************************************
// IOCTL. 
// 
// Params:
// none
//
// Return:
// 0: sucess  -1: i2c error. 
// ---------------------------------------------------------------------------
static long touchplus_miscdev_ioctl(struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
    int i,retval = 0;
    msi_regs regs;
    msi_reg reg;
    unsigned char iob[64];
	  unsigned char buf[2];

		
   printk("IOCTL command parser\n"); 
   if (_IOC_TYPE(cmd) != IOCTL_MAGIC_NUMBER) return -EFAULT;
   if (_IOC_NR(cmd) >= IOCTL_ID_INVALID) return -EFAULT;

   if(g_tsd->client == NULL) return -EFAULT;
   printk("IOCTL command: %d\n", cmd);


   switch(cmd)
   {
	 case IOCTL_CMD_AUTOTUNE_SET:		  
	 	DEBUG_MSG("touch screen autotune start!\n"); 		  
		retval = touchplus_ts_autotune();          
		break; 		         
	case IOCTL_CMD_AUTOTUNE_GET:	   	  
		DEBUG_MSG("touch screnn autotune status!\n"); 		 
		retval = touchplus_ts_autotune_status();          
		break;
   
	   case IOCTL_CMD_ENABLE_IRQ:
	   	  printk("enable IRQ!\n"); 
	   	  enable_irq(g_tsd->irq);			
          break;
		  
	   case IOCTL_CMD_DISABLE_IRQ:
	   	  printk("disable IRQ!\n"); 
	   	  disable_irq_nosync(g_tsd->irq);
          break;
		  
		  case IOCTL_CMD_RESET_TSP_SET:
          disable_irq_nosync(g_tsd->irq);

          printk("Reset touch screen\n");
          #ifdef RESET_PIN_SUPPORT
          gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		  gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);  // reset pulse to reset touch screen
		  mdelay(10);
		  gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
     

          #else         //normal mode
          g_tsd->client->addr = TOUCHPLUS_NORMAL_I2C_ADDRESS;
		  //ISP unlock
		  buf[0] = ISP_UNLOCK_CODE;
		  touchplus_write_regs(g_tsd->client, MSI_UNLOCK, buf,1);
          // software reset

		  buf[0] = CMD_PROGRAM_WRITE;
		  touchplus_write_regs(g_tsd->client, MSI_CMD_MODE, buf,1);
		  printk("APROM reset to LDROM\n");
          #endif

          mdelay(500);
          enable_irq(g_tsd->irq);
          break;

	   case IOCTL_CMD_CMDMODE_SET:
	 
          if(copy_from_user(&iob, (void *)arg, 1))
				return -EFAULT;
		  printk("CMD MODE: %d\n", iob[0]);
          runmode = iob[0];
		  if(runmode == CMD_PROGRAM_WRITE) /* enter bootloader mode */
		  {  

          printk("i2cc slave address: %x\n", g_tsd->client->addr);
		  disable_irq_nosync(g_tsd->irq);
		  cancel_delayed_work_sync(&g_tsd->work);
		  temp_val = 0; //used for package counter
		   
		  #ifdef RESET_PIN_SUPPORT
		  gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		  gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);  // reset pulse to reset touch screen
		  mdelay(10);
		  gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		  mdelay(30);
		  
		  DEBUG_BL("1--i2c slave address: %x\n", g_tsd->client->addr);
          g_tsd->client->addr = TOUCHPLUS_BOOTLOADER_I2C_ADDRESS;
          DEBUG_BL("2--i2c slave address: %x\n", g_tsd->client->addr);
	      
          buf[0]=0;
	      i=0;
	      while(i++ < 20 ) // 200ms bootloader check
	      {
            retval = i2c_master_send(g_tsd->client,buf,1);
            DEBUG_BL("%d\n",i);
            //ret =touchplus_write_regs(g_tsd->client,0x00,buff[0],1);
			if(retval == 1) break;
			mdelay(10);
	   	  }
		  if(i >= 20) {

		    g_tsd->client->addr = TOUCHPLUS_NORMAL_I2C_ADDRESS;
		    //ISP unlock
		    buf[0] = ISP_UNLOCK_CODE;
		    touchplus_write_regs(g_tsd->client, MSI_UNLOCK, buf,1);
            // software reset

		    buf[0] = CMD_PROGRAM_WRITE;
		    touchplus_write_regs(g_tsd->client, MSI_CMD_MODE, buf,1);
		    printk("APROM reset to LDROM\n");
		    mdelay(20);
		    DEBUG_BL("1--i2c slave address: %x\n", g_tsd->client->addr);
            g_tsd->client->addr = TOUCHPLUS_BOOTLOADER_I2C_ADDRESS;
            DEBUG_BL("2--i2c slave address: %x\n", g_tsd->client->addr);
		    i=0;
		    buf[0]=0;
	        while(i++ < 20 ) // 200ms bootloader check
	        {
             retval = i2c_master_send(g_tsd->client,buf,1);
             DEBUG_BL("%d\n",i);
             //ret =touchplus_write_regs(g_tsd->client,0x00,buff[0],1);
			 if(retval == 1) break;
			 mdelay(10);
		     }
		     if(i >= 20) {
		
		       enable_irq(g_tsd->irq);	
		       printk("Bootloader doesn't exist or error!!\n"); 
		       return -1;
		     }
		  }
          #else		//normal mode
		   g_tsd->client->addr = TOUCHPLUS_NORMAL_I2C_ADDRESS;
		  //ISP unlock
		  buf[0] = ISP_UNLOCK_CODE;
		  touchplus_write_regs(g_tsd->client, MSI_UNLOCK, buf,1);
          // software reset

		  buf[0] = CMD_PROGRAM_WRITE;
		  touchplus_write_regs(g_tsd->client, MSI_CMD_MODE, buf,1);
		  printk("APROM reset to LDROM\n");

		  DEBUG_BL("1--i2c slave address: %x\n", g_tsd->client->addr);
          g_tsd->client->addr = TOUCHPLUS_BOOTLOADER_I2C_ADDRESS;
          DEBUG_BL("2--i2c slave address: %x\n", g_tsd->client->addr);
		  i=0;
		  buf[0]=0;
	      while(i++ < 20 ) // 200ms bootloader check
	      {
            retval = i2c_master_send(g_tsd->client,buf,1);
             DEBUG_BL("%d\n",i);
            //ret =touchplus_write_regs(g_tsd->client,0x00,buff[0],1);
			if(retval == 1) break;
			mdelay(10);
		  }
		  if(i >= 20) {
		
		    enable_irq(g_tsd->irq);	
		    printk("Bootloader doesn't exist or error!!\n"); 
		    return -1;
		  }
          #endif


		  }
		  else if(runmode == CMD_SELF_RAWDATA){
		          //retval = touchplus_read_regs(g_tsd->client, MSI_X_NBR, buf, 2);
              //    if(retval != 2) { printk("I2C read touch screen  channel error\n"); return -1; }
  		        //  temp_val = (buf[0] + buf[1])* 2;
		          //printk("XCH: %d, YCH: %d, self raw data count: %d\n",buf[0],buf[1],temp_val);
		          //mdelay(1000);
		  	  	  disable_irq_nosync(g_tsd->irq);
		          cancel_delayed_work_sync(&g_tsd->work);
                  touchplus_write_regs(g_tsd->client, MSI_CMD_MODE,iob,1);
		          mdelay(30);
		  }
		  else if(runmode == CMD_MUTUAL_RAWDATA){
               // retval = touchplus_read_regs(g_tsd->client, MSI_X_NBR, buf, 2);
                //  if(retval != 2) { printk("I2C read touch screen  channel error\n"); return -1; }
  		        //  temp_val = buf[1] * 2; //y*2
		         // DEBUG_MSG("XCH: %d, YCH: %d, mutual raw data count: %d\n",buf[0],buf[1],temp_val);
		          //mdelay(300);
		  	  	  disable_irq_nosync(g_tsd->irq);
		          cancel_delayed_work_sync(&g_tsd->work);
				  retval = touchplus_write_regs(g_tsd->client, MSI_CMD_MODE,iob,1);
		          mdelay(30);
		  } else { 

		  touchplus_write_regs(g_tsd->client, MSI_CMD_MODE,iob,1);
		 }
		
		  break;
		  
	   case IOCTL_CMD_MSIREG_GET: /* read  MSI registers */
	   	  printk("read registers\n");
		  if (copy_from_user( &regs, (void *)arg, sizeof(msi_regs)))
				return -EFAULT;
		  printk("register addr: %d,len: %d\n", regs.addr, regs.len);
		  retval = touchplus_read_regs(g_tsd->client, regs.addr, iob, regs.len);
		  if(retval < 0)
		  {
             printk("read register error\n");
             return -1;
          }
		  mdelay(regs.len);
                  
          memcpy( regs.buf, iob, regs.len);
		  if(copy_to_user( (void *) arg, &regs, sizeof(msi_regs)))
		  	    return -EFAULT;
		  break;

	   case IOCTL_CMD_MSIREG_SET: /* write a single MSI registers */
		  if(copy_from_user( &reg, (void *)arg, sizeof(msi_reg)))
				return -EFAULT;
          buf[0]=reg.val;
          retval = touchplus_write_regs(g_tsd->client, reg.addr,buf,1);
          if(retval < 0)
		  {
             printk("write register error\n");
             return -1;
          }
		  break;

    
       default: retval = -EFAULT; 
	      break;
   }

   return retval;   
   	}

// ***************************************************************************
// write device
// 
// Params:
// none
//
// Return:
// 0: 
// ---------------------------------------------------------------------------
static ssize_t
touchplus_miscdev_write(struct file *filp, const char __user *buf,
		        size_t count, loff_t *pos)
{
	int ret;
	int i;
	int opcode;
	unsigned char buff[2];

	printk("%s\n", __FUNCTION__);
	
	switch(runmode)
	{
		case CMD_PROGRAM_WRITE:
		
		memset(ts_buf, 0, PACK_SIZE);

		if (copy_from_user(ts_buf,buf,count))
		{
			printk("COPY FAIL ");
			return -EFAULT;
		}
		
        #ifdef BOOTLOADER_DEBUG
		
		for(i=0;i<PACK_SIZE;i++)
		{
			if(ts_buf[i] < 0x10)
			printk("0%x",ts_buf[i]);
			else
			printk("%x",ts_buf[i]);
		}
        #endif

        opcode = ts_buf[0];

	    DEBUG_BL("-------2-------\n");
		ret = i2c_master_send(g_tsd->client,ts_buf,count);
		if(ret != count)
		{
			printk("143 bytes write error, ret = %d\n",ret);
			return -1;
		}
		temp_val++;
		printk("packet no = %3d\n",temp_val);
		if(opcode != 0x01)  // not jump line
		{
			mdelay(1);
			while(gpio_get_value(GPIO_IRQ)); // wait touch controller processing data to flash. INT is low means done.
			mdelay(1);


			ret = i2c_master_recv(g_tsd->client,buff,1);
			if(ret != 1)
			{
				printk("read IIC slave status error,ret = %d\n",ret);
				return -1;

			}
	
		}
		else  // file download finished. wait touch screen controller jump to APROM
		{
              
            #ifdef RESET_PIN_SUPPORT
            gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		    gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);  // reset pulse to reset touch screen
		    mdelay(10);
		    gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
            #endif
            mdelay(500);
		    g_tsd->client->addr = TOUCHPLUS_NORMAL_I2C_ADDRESS;
			printk("i2c slave address: %x\n", g_tsd->client->addr);
			i=0;
			buff[0] = 0;
			while(1)  // probe i2c address 0x55
			{
			  ret = i2c_master_send(g_tsd->client,buff,1);
			  if(ret == 1) {ret = 0; break; }//probe sucess
			  if(i++ > 10) { printk("Time out to APROM!!\n");
			  return -1; // timer out, may be flash update fail
			  }
			  mdelay(500);
			}
			runmode = CMD_NORMAL;
			enable_irq(g_tsd->irq);
			printk("Jump to APROM!!\n");
			return 0; //success
		}

		if((buff[0]&0x80)&&(opcode != 0x01))
		{
			printk("download failed\n");
			return -1;
		}
	
		break;
		
		default: 
		break;
	}
	
	
	return 0;
}

static struct file_operations touchplus_miscdev_fops = {
	.owner = THIS_MODULE,
	.open = touchplus_miscdev_open,
	.release = touchplus_miscdev_release,
	.read = touchplus_miscdev_read,
	.write = touchplus_miscdev_write,
	.unlocked_ioctl = touchplus_miscdev_ioctl,
};




static int touchplus_calibrate( struct i2c_client *client)
{
	//struct touchplus_i2c_ts_data *tsd = dev_get_drvdata(&client->dev);
	//int i;
	//int value;
	u8 Mbuf[1];
	u8 Ybuf[1];

	 //u8 PWDbuf[1];

	Ybuf[0]=0;
	Mbuf[0]=0x03;
	printk("write Mbuf=0x03 to the regitser 0x37\n");
	
	touchplus_write_regs(client, MSI_SPECOP, Mbuf, 1);

	printk("write ok!,if gpio=high to read 0x37==0x55?\n");
	return 0;

}

//static ssize_t touch_mode_show(struct class *cls, char *_buf)
static ssize_t touch_mode_show(struct class *cls,struct class_attribute *attr, char *_buf)
//static ssize_t touch_mode_show(struct class *cls, char *_buf)
{
	//printk("%s\n",__FUNCTION__); 
//	int i,value;
	u8 Ybuf[1];
	Ybuf[0]=0;

	calibration_flag=gpio_get_value(g_tsd->client->irq);
	printk("hjc-%s,calibration=%d\n",__FUNCTION__,calibration_flag);
/*
	if(calibration_flag){
		touchplus_read_regs(g_tsd->client, 0x37,Ybuf, 1);
		printk("i=%d,calibration_flag=%d,Ybuf=0x%x\n",i,calibration_flag,Ybuf[0]);
		enable_irq(irq_flag);
	}
		
*/	
	if(calibration_flag == 1)
	{
	        calibration_flag = 0;
		enable_irq(irq_flag);
		enable_irq(irq_flag);//add by hjc @ 0229
		printk("hjc----return successful\n");
		//return sprintf(_buf,"failed");//for test
		return sprintf(_buf,"successful");	
	}
	else
	{
	    	calibration_flag = 0;
		if(--calibration_time <= 0){
			printk("return failed\n");
			//touchplus_calibrate( g_tsd->client);//calibration again
			calibration_time=15;
			return sprintf(_buf,"failed");
		}else{
			if(calibration_time>16)
				calibration_time=15;
			printk("return calibration\n");
			return sprintf(_buf,"calibration");
		}
	}
}

//static ssize_t touch_mode_store(struct class *cls, const char *_buf, size_t _count)
static ssize_t touch_mode_store(struct class *cls,struct class_attribute *attr,const char *_buf,size_t _count)
//static ssize_t touch_mode_store(struct class *cls, const char *_buf,size_t _count)
{
  //  u8  ucData;
    calibration_flag = 0;
    if(!strncmp(_buf,"tp_cal" , strlen("tp_cal")))//hjc for test 
    {
            printk("TP Calibration is start!!! \n");
	    //calibration_flag = 1;	
	   disable_irq(irq_flag);
	   cancel_delayed_work_sync(&g_tsd->work);//add by hjc @ 0229
	   touchplus_calibrate( g_tsd->client);	//hjc 02-10
    }
    return _count;
} 


static ssize_t touch_version_show(struct class *cls,struct class_attribute *attr, char *_buf)
{
	//printk("%s\n",__FUNCTION__); 
	//int i,value;
	int minor,major;
	unsigned char buf[3];
	//fingerbuf[1]=0;
	//npoints = i2c_read_bytes(tsd->client,fingerbuf,1);
	printk("enable_irq,gpio_get_value(g_tsd->client->irq)=%d",gpio_get_value(g_tsd->client->irq));
	touchplus_read_regs(g_tsd->client, 0x34, buf, 3);
	minor = buf[0]&0x0f;
	major = buf[0]>>4;

	printk("version:%1x.",major);
	printk("%1x.",minor);
	printk("%x",buf[2]);  
	printk("%2x\n",buf[1]);
	//printk("fingerbug[0]=%x,buf[1]=%x,buf[2]=%x\n",fingerbuf[0],fingerbuf[1],fingerbuf[2]);
	return sprintf(_buf,"touchversion");	

}

static ssize_t touch_version_store(struct class *cls,struct class_attribute *attr, const char *_buf,size_t _count)
{
#if 0   
	u8  ucData;
    //printk("%s\n",__FUNCTION__); 
    calibration_flag = 0;
    if(!strncmp(_buf,"tp_cal" , strlen("tp_cal")))
    {
            printk("TP Calibration is start!!! \n");
	    //calibration_flag = 1;	
	   disable_irq(irq_flag);
	   
	  if(touchplus_calibrate( g_tsd->client)==0)
	  	calibration_flag = 1;
	  else
	  	calibration_flag = 0;	
	    mdelay(500);
    }
 
#endif	
   return _count;
} 

// ***************************************************************************
// get touch panel device information
// 
// Params:
// none
//
// Return:
// 0: success 
// --------------------------------------------------------------------------
static int touchplus_ts_deviceinfo(void)
{
   int ret = 0;
   unsigned char buf[4];
   unsigned char major,minor;
   
   ret = touchplus_read_regs(g_tsd->client, MSI_VERSION, buf, 3);
   if(ret != 3) { printk("I2C read touch panel version error\n"); return -1; }
   minor = buf[0]&0x0f;
   major = buf[0]>>4;

   printk("version:%1d.",major); printk("%1d.",minor); 
   printk("%x",buf[2]);  printk("%2x\n",buf[1]);
   
   
   ret = touchplus_read_regs(g_tsd->client, MSI_X_NBR, buf, 1);
   if(ret != 1) { printk("I2C read touch panel X channel error\n"); return -1; }
   printk("X-CH:%d ",buf[0]); 

   ret = touchplus_read_regs(g_tsd->client, MSI_Y_NBR, buf, 1);
   if(ret != 1) { printk("I2C read touch panel Y channel error\n"); return -1; }
   printk("Y-CH:%d ",buf[0]); 

   
   ret = touchplus_read_regs(g_tsd->client, MSI_X_RES, buf, 2);
   if(ret != 2) { printk("I2C read touch panel horizontal resolution error\n"); return -1; }
   printk("X-SIZE:%d ",((buf[1]<<8) | buf[0])); 

   
   ret = touchplus_read_regs(g_tsd->client, MSI_Y_RES, buf, 2);
   if(ret != 2) { printk("I2C read touch panel vertical resolution error\n"); return -1; }
   printk("Y-SIZE:%d\n",((buf[1]<<8) | buf[0]));


   return ret;
}

static CLASS_ATTR(touchcalibration, 0666, touch_mode_show, touch_mode_store);
static CLASS_ATTR(touchversion, 0666, touch_version_show, touch_version_store);

//////
unsigned char tp_flag=0;
static int touchplus_i2c_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct touchplus_i2c_ts_data *tsdata;
	struct input_dev *input;
	int error;
	int ret = 0;
	char buf[5] = {0};
//	struct touchplus_platform_data *pdata = client->dev.platform_data;//add by hjc

	printk("touchplus_i2c_ts_probe\n");
/*add by hjc if...*/	
#if 0
	if (!pdata) {
		dev_err(&client->dev, "empty platform_data\n");
		goto err_check_functionality_failed;
    }
	    /*hardware init*/	
	if(pdata->hw_init)
		pdata->hw_init();

	if(pdata->reset){
		pdata->reset(1);
		mdelay(100);
		pdata->reset(0);
		mdelay(200);
	}
#endif

	touchplus_hw_init();
#if 1
	touchplus_hw_reset(1);
	mdelay(100);
	touchplus_hw_reset(0);
	mdelay(100);
	touchplus_hw_reset(1);
	mdelay(200);	
#endif
	printk("%s,init hardware end hear!\n",__FUNCTION__);

	ret = touchplus_read_regs(client,57,buf,1);
	if((ret <  0) || ((buf[0] != 38) && (buf[0] != 27)))
	{
		printk("touchplus_i2c_ts_probe:i2c_transfer error ret=%d reg=%d x_number=%d\n",ret,57,buf[0]);
		touchplus_hw_remove();
		return ret;
	}	

	tp_flag = 1;
/*add by hjc end hear*/

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata)
	{
		dev_err(&client->dev, "failed to allocate driver data!\n");
		error = -ENOMEM;
		dev_set_drvdata(&client->dev, NULL);
		return error;
	}
	g_tsd = tsdata;

	dev_set_drvdata(&client->dev, tsdata);
	//touchplus_status_dump( client);

	printk("===================%s,mdelay 1000*30,client->irq=%d\n",__FUNCTION__,client->irq);
/*
	touchplus_calibrate( client);//delete for test
	for(i=0;i<30;i++){
		if(gpio_get_value(client->irq)){
			printk("i=%d,calibration ok\n",i);
			continue;
		}else{
			printk("i=%d,irq=%d\n",gpio_get_value(client->irq));
			mdelay(1000);
		}
	}
*/
	input = input_allocate_device();
	if (!input)
	{
		dev_err(&client->dev, "failed to allocate input device!\n");
		error = -ENOMEM;
		input_free_device(input);
		kfree(tsdata);
	}
	
#if 0
    set_bit(EV_ABS, input->evbit);
    set_bit(EV_KEY, input->evbit);
    //set_bit(EV_SYN, input->evbit);
	//set_bit( BTN_TOUCH, input->keybit);
    set_bit( ABS_MT_TOUCH_MAJOR, input->absbit);
    set_bit( ABS_MT_WIDTH_MAJOR, input->absbit);
    set_bit( ABS_MT_POSITION_X, input->absbit);
    set_bit( ABS_MT_POSITION_Y, input->absbit);
#endif

    //set_bit( ABS_MT_TRACKING_ID, input->absbit);

	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	__set_bit(EV_ABS, input->evbit);
	
	input_mt_init_slots(input, MAX_SUPPORT_POINT);//for 4.0 hjc
	input_set_abs_params( input, ABS_X, 0, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params( input, ABS_Y, 0, TOUCHSCREEN_MAXY, 0, 0);

	input_set_abs_params( input, ABS_MT_POSITION_X, 0, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params( input, ABS_MT_POSITION_Y, 0, TOUCHSCREEN_MAXY, 0, 0);
        input_set_abs_params( input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		
	//input_set_abs_params( input, ABS_MT_WIDTH_MAJOR, 0, 25, 0, 0);//for 4.0 hjc
	
     // input_set_abs_params( input, ABS_MT_TRACKING_ID, 1, 10, 0, 0);

	input->name = client->name;
        input->phys = "I2C";
	input->id.bustype = BUS_I2C;

	input->dev.parent = &client->dev;
	input->open = touchplus_ts_open;
	input->close = touchplus_ts_close;
	input_set_drvdata(input, tsdata);

	tsdata->client = client;
	tsdata->input_dev = input;

	//INIT_WORK(&tsdata->work.work, touchplus_ts_poscheck);//for long time delay
	INIT_DELAYED_WORK(&tsdata->work, touchplus_ts_poscheck);

	tsdata->irq = client->irq;
	GPIO_IRQ = tsdata->irq; 
	if (input_register_device(input))
	{
		input_free_device(input);
		kfree(tsdata);
	}
	tsdata->irq = gpio_to_irq(client->irq);
	irq_flag = tsdata->irq;
	
	printk("%s,tsdata->irq=%d\n",__FUNCTION__,tsdata->irq);
	if (request_irq(tsdata->irq, touchplus_ts_isr, IRQF_TRIGGER_LOW, client->name, tsdata))   //IRQF_TRIGGER_LOW | IRQF_TRIGGER_RISING  -->
	{
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input_unregister_device(input);
		input = NULL;
	}
	//tsdata->miscdev.minor = 70,
	//tsdata->miscdev.name = "touchplus",
	//tsdata->miscdev.fops = &touchplus_miscdev_fops;
	//misc_register( &(tsdata->miscdev));
	//device_init_wakeup(&client->dev, 1); //delete by hjc
	//msleep(200);
//add by hjc
#ifdef CONFIG_HAS_EARLYSUSPEND
   //tsdata->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB+1;//changed by hjc @0223
  tsdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    tsdata->early_suspend.suspend = touchplus_ts_early_suspend;
    tsdata->early_suspend.resume = touchplus_ts_late_resume;
    register_early_suspend(&tsdata->early_suspend);
#endif

       touchplus_ts_deviceinfo();

        printk("%s,probe touchplus ok!\n",__FUNCTION__);
        return 0;


	
	printk("%s,probe touchplus ok!\n",__FUNCTION__);
	return 0;
//err_check_functionality_failed:
	return error;
}

static int touchplus_i2c_ts_remove(struct i2c_client *client)
{
	struct touchplus_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	printk("touchplus_i2c_ts_remove\n");
	free_irq(tsdata->irq, tsdata);
	input_unregister_device(tsdata->input_dev);
	//misc_deregister( &(tsdata->miscdev));
	kfree(tsdata);
	dev_set_drvdata(&client->dev, NULL);
	g_tsd = NULL;
	return 0;
}

static int touchplus_i2c_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct touchplus_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
/*	if (device_may_wakeup(&client->dev))
		enable_irq_wake(tsdata->irq);
*/
	unsigned char pmdreg=0x3D;
	u8 buf[1];
	//buf[0]=0xA6;//modify for pcb 6
	
	//u8 buf1[1];
	u8 buf2[1];
	
	buf[0] = 0xA2;
	buf2[0]=0x3D;
	
	//printk("hjc******************%s*****************1\n",__FUNCTION__);
	disable_irq_nosync(tsdata->irq);
	
	cancel_delayed_work_sync(&tsdata->work);
#if 0
	//printk("hjc******************%s*****************2\n",__FUNCTION__);
	touchplus_read_regs(client, 0x34, buf1, 1);
	printk("1---read 0x3D buf1 = 0x%x\n",buf1[0]);
	touchplus_read_regs(client, 0x34, buf2, 1);
	printk("2---read 0x3D buf2 = 0x%x\n",buf2[0]);
#endif	
	touchplus_write_regs(client, pmdreg, buf, 1);

	return 0;
}

static int touchplus_i2c_ts_resume(struct i2c_client *client)
{
	struct touchplus_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	//unsigned char pmdreg=0x3D;
	u8 buf[1];
	//buf[0]=0xA4;//modify for pcb 6
	buf[0]=0xA0;
/*
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(tsdata->irq);
*/	
	//printk("-------%s-------\n",__FUNCTION__);
	
#if 1
	 gpio_direction_output(TOUCH_INT_PIN, 0);
         gpio_set_value(TOUCH_INT_PIN, 0);
         mdelay(10);
	 gpio_set_value(TOUCH_INT_PIN, 1);
	mdelay(10);
	
	 gpio_direction_input(TOUCH_INT_PIN);
         gpio_pull_updown(TOUCH_INT_PIN, GPIO_HIGH);
#endif
#if 0
        touchplus_hw_reset(0);
	mdelay(10);
	touchplus_hw_reset(1);
	mdelay(50);
#endif
	
	//touchplus_calibrate( client);	

	enable_irq(tsdata->irq);
	enable_irq(tsdata->irq);
	//printk("enable_irq,gpio_get_value(g_tsd->client->irq)=%d",gpio_get_value(g_tsd->client->irq));
	//touchplus_write_regs(client, pmdreg, buf, 1);
	//printk("%s\n",__FUNCTION__);
	return 0;
}
//add by hjc
#ifdef CONFIG_HAS_EARLYSUSPEND
static void touchplus_ts_early_suspend(struct early_suspend *h)
{
    struct touchplus_i2c_ts_data *ts;
    printk("%s\n",__FUNCTION__);
    ts = container_of(h, struct touchplus_i2c_ts_data, early_suspend);
    touchplus_i2c_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void touchplus_ts_late_resume(struct early_suspend *h)
{
    struct touchplus_i2c_ts_data *ts;
    printk("%s\n",__FUNCTION__);
    ts = container_of(h, struct touchplus_i2c_ts_data, early_suspend);
    touchplus_i2c_ts_resume(ts->client);
    //printk("****%s*******/n,mdelay 200 ms******\n",__FUNCTION__);
   // mdelay(200);
}
#endif

#define TOUCHPLUS_I2C_NAME "touchplus_ts"
static const struct i2c_device_id touchplus_ts_id[] = {
	{ TOUCHPLUS_I2C_NAME, 0 },
	{ }
};


static struct i2c_driver touchplus_i2c_ts_driver = {
	.probe = touchplus_i2c_ts_probe, 
	.remove = touchplus_i2c_ts_remove,
	.suspend = touchplus_i2c_ts_suspend, 
	.resume = touchplus_i2c_ts_resume,
	.id_table = touchplus_ts_id,
 	.driver = {
		.owner = THIS_MODULE,
		.name = TOUCHPLUS_I2C_NAME,
	}
};

// ***************************************************************************
// register device. 
// 
// Params:
// none
//
// Return:
// 
// ---------------------------------------------------------------------------
static int touchplus_ts_registerdev(void)
{
    int err;

    touchplus_cdev        = cdev_alloc(); 
    cdev_init(touchplus_cdev, &touchplus_miscdev_fops); 
    touchplus_cdev->owner = THIS_MODULE;

    alloc_chrdev_region(&devid, 0, 1, "touchplus"); 
    touchplus_Major = MAJOR(devid);
    touchplus_Minor = MINOR(devid);
    DEBUG_MSG(KERN_INFO "touchplus device major number %d.\n", touchplus_Major); 
    DEBUG_MSG(KERN_INFO "touchplus device minor number %d.\n", touchplus_Minor);

    err = cdev_add(touchplus_cdev, devid, 1); 
    if (err) { 
        DEBUG_MSG(KERN_NOTICE "Error %d adding device\n", err); 
        return -1; 
    }
#if 0
    touchplus_class = class_create(THIS_MODULE, "touchplus_class"); 
    if (IS_ERR(touchplus_class)) { 
        DEBUG_MSG(KERN_INFO "create class error\n"); 
        return -1; 
    } 

	err = class_create_file(touchplus_class,&class_attr_touchcalibration);
	if (err) { 
        DEBUG_MSG(KERN_INFO "create class file error\n"); 
        return -1; 
    } 
	#endif
	#if 1
        touchplus_class = class_create(THIS_MODULE, "touchpanel");
    if (IS_ERR(touchplus_class))
        {
            printk("Create class touchpanel failed.\n");
            return 0;
        }
    err  = class_create_file(touchplus_class,&class_attr_touchcalibration);
    if (err) { 
        printk(KERN_INFO "create class file error\n"); 
        return -1; 
    } 
    err = class_create_file(touchplus_class, &class_attr_touchversion);
    if (err) { 
        printk(KERN_INFO "create class file error\n"); 
        return -1; 
    } 

	printk("Create class touchpanel is sucess\n");  
#endif        
    device_create(touchplus_class, NULL, devid, NULL, "touchplus_i2c_ts" "%d", MINOR(devid));    
   

    return 0;
}
// ***************************************************************************
// unregister device. 
// 
// Params:
// none
//
// Return:
// 
// --------------------------------------------------------------------------
static void  touchplus_ts_unregisterdev(void)
{
     unregister_chrdev_region(devid, 1); 
     cdev_del(touchplus_cdev);

     device_destroy(touchplus_class, devid); 
     class_destroy(touchplus_class);
}

static int __init touchplus_i2c_ts_init(void)
{
	//bind_gpiofuncs();  //changed by hjc
	printk("touchplus_i2c_init\n");
  touchplus_ts_registerdev();
	//touchplus_wq = create_workqueue("touchplus_wq");
	touchplus_wq = create_workqueue("touchplus_wq");
	
	if(!touchplus_wq){
		printk("%s,create_wrokqueue faild!\n",__FUNCTION__);
		return -ENOMEM;
		
	}
	return i2c_add_driver(&touchplus_i2c_ts_driver);
}

static void __exit touchplus_i2c_ts_exit(void)
{
	printk("touchplus_i2c_exit\n");
	touchplus_ts_unregisterdev();
	i2c_del_driver(&touchplus_i2c_ts_driver);
	if(touchplus_wq)
		destroy_workqueue(touchplus_wq);
}

MODULE_AUTHOR("hjc@rock-chips.com");
MODULE_DESCRIPTION("touchplus tp driver");
MODULE_LICENSE("GPL");

module_init( touchplus_i2c_ts_init);
module_exit( touchplus_i2c_ts_exit);


