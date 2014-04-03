/* drivers/input/touchscreen/touchplus_i2c_ts.c
 *
 * Copyright (C) 2010 TouchPlus, Inc.
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



#if 0
static int (*mygpio_get_value)(unsigned gpio) = NULL;
static void (*mygpio_enable_edge_int)(int pin , int flag, int group) = NULL;
static int (*mygpio_to_idx)(unsigned gpio) = NULL;
static int (*mygpio_direction_input)(unsigned gpio) = NULL;
static int (*mygpio_direction_output)(unsigned gpio, int value) = NULL;

static void bind_gpiofuncs(void)
{
	mygpio_get_value = kallsyms_lookup_name( "gpio_get_value");
	mygpio_enable_edge_int = kallsyms_lookup_name( "gpio_enable_edge_int");
	mygpio_to_idx = kallsyms_lookup_name( "gpio_to_idx");
	mygpio_direction_input = kallsyms_lookup_name( "gpio_direction_input");
	mygpio_direction_output = kallsyms_lookup_name( "gpio_direction_output");
}
#endif
#define TS_MAXX 1024//800  //changed by hjc
#define TS_MAXY 600

#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "Kevin Ma <kevin.ma@touchplus.com.tw>"
#define DRIVER_DESC "TouchPlus I2C Touchscreen Driver with tune fuction"
#define DRIVER_LICENSE "GPL"

#define TOUCHPLUS_DEBUG 1
#define TOUCH_UP 1
#define TOUCH_DOWN 0
#define MAX_SUPPORT_POINT 5
#if 0
#define gpio_shutdown ((GPIOD_bank_bit2_24(23)<<16) |GPIOD_bit_bit2_24(23))
#define gpio_irq ((GPIOD_bank_bit2_24(24)<<16) |GPIOD_bit_bit2_24(24))
#endif
#if 1
/*add by hjc*/
#define TOUCH_RESET_PIN    RK29_PIN6_PC3
#define TOUCH_INT_PIN         RK29_PIN0_PA2
static u8 calibration_flag = 0;
static int calibration_time =15;
static int calibration_test = 7;
static u8 FLAG=0;
static int irq_flag;
static int GPIO_IRQ;
static unsigned int finger_status[5]={TOUCH_UP,TOUCH_UP,TOUCH_UP,TOUCH_UP,TOUCH_UP};

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
	ret =i2c_master_reg8_recv(client, reg, buf, len, 400*1000);
	if(ret < 0)
		printk("touchplus_ts_work_func:i2c_transfer fail =%d\n",ret);
	return ret;
}
/* set the touchplus registe,used i2c bus*/
static int touchplus_write_regs(struct i2c_client *client, u8 reg, u8 const buf[], unsigned short len)
{
	int ret;
	ret = i2c_master_reg8_send(client,reg, buf, len, 400*1000);
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
	struct miscdevice miscdev;
	struct early_suspend early_suspend;//add by hjc
};

#ifdef CONFIG_HAS_EARLYSUSPEND  // add by hjc
static void touchplus_ts_early_suspend(struct early_suspend *h);
static void touchplus_ts_late_resume(struct early_suspend *h);
#endif

/* for handy access */
static struct touchplus_i2c_ts_data *g_tsd =NULL;
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];
	msgs[0].scl_rate = 400*1000;

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len;
	msgs[1].buf=&buf[0];
	msgs[1].scl_rate = 400*1000;
	
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
	msg.scl_rate = 400*1000;
	ret=i2c_transfer(client->adapter,&msg,1);
	return ret;
}

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

static void report_position(struct work_struct *work)
{
	//printk("%s\n",__FUNCTION__);
	struct touchplus_i2c_ts_data *tsd = container_of(work,
			struct touchplus_i2c_ts_data, work.work);
	int i, npoints;
	tsd->iobuf[0] = 2;
	unsigned char fingerbuf[1];
	fingerbuf[1]=0;
	//npoints = i2c_read_bytes(tsd->client,fingerbuf,1);
	touchplus_read_regs(tsd->client, 0x00, fingerbuf, 1);
	if(fingerbuf[0] != 0){
		memset(tsd->iobuf, 0, sizeof(tsd->iobuf));
		touchplus_read_regs(tsd->client,0x02,tsd->iobuf,5*fingerbuf[0]);
		for(i=0;i<5;i++){
		if((i+1 >fingerbuf[0])||(fingerbuf[0]==0)){
			//printk("i=%d,fingerbuf[0]=%d,finger_status[i]=%d\n",i,fingerbuf[0],finger_status[i]);
			if(finger_status[i] == 0){
				finger_status[i] = 1;
				//input_report_abs(tsd->input_dev, ABS_MT_TOUCH_MAJOR, 0); //Finger Size
				//input_report_abs(tsd->input_dev, ABS_MT_WIDTH_MAJOR, 0); //Touch Size
				//input_mt_sync(tsd->input_dev);

				input_mt_slot(tsd->input_dev, i);
				input_mt_report_slot_state(tsd->input_dev, MT_TOOL_FINGER, false);
				
				//printk("TOUCH_UP\n");
			}
		}else{
			const u8* posp = tsd->iobuf + i*5;
			int x = posp[1]<<8 | posp[0];
			int y = posp[3]<<8 | posp[2];
			int fingerid= posp[4];
			//printk("%d: (%d,%d)\n", fingerid, x, y);
			//if((x!=0)&&(y!=0)){
				finger_status[i] = 0;
				//printk("finger_status[%d]=%d\n",i,finger_status[i]);
				/*input_report_abs( tsd->input_dev, ABS_MT_TRACKING_ID, fingerid);
				input_report_abs( tsd->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs( tsd->input_dev, ABS_MT_POSITION_Y, y);
				input_report_abs( tsd->input_dev, ABS_MT_TOUCH_MAJOR, 10);
				input_report_abs( tsd->input_dev, ABS_MT_WIDTH_MAJOR, 10);
				input_mt_sync( tsd->input_dev);*/
				input_mt_slot(tsd->input_dev, i);
				input_mt_report_slot_state(tsd->input_dev, MT_TOOL_FINGER, true);
				input_report_abs(tsd->input_dev, ABS_MT_TOUCH_MAJOR, 10);
				input_report_abs(tsd->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(tsd->input_dev, ABS_MT_POSITION_Y, y);
				//printk("i=%d,TOUCH_DOWN\n",i);
			//}
		}
		
		}
	}else{
		//input_mt_sync(tsd->input_dev);
	}
	//printk("%d,%d,%d,%d,%d\n",finger_status[0],finger_status[1],finger_status[2],finger_status[3],finger_status[4]);
	input_sync( tsd->input_dev);
	//enable_irq(tsd->irq);
	
}

static void touchplus_ts_poscheck(struct work_struct *work)
{
	int tmp,i;
	struct touchplus_i2c_ts_data *tsd = container_of(work,
			struct touchplus_i2c_ts_data, work.work);
	report_position(work);
	tmp=gpio_get_value(GPIO_IRQ);
	//printk("%s,gpio=%d\n",__FUNCTION__,tmp);
	if(!tmp){
		queue_delayed_work(touchplus_wq, &tsd->work, msecs_to_jiffies(5));
	}else{

		//input_report_abs(tsd->input_dev, ABS_MT_TOUCH_MAJOR, 0); //Finger Size
		//input_report_abs(tsd->input_dev, ABS_MT_WIDTH_MAJOR, 0); //Touch Size
		
		//input_mt_sync(tsd->input_dev);
		//input_sync( tsd->input_dev);

		//input_mt_slot(input_dev, i);
		//input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
		for(i=0;i<5;i++){
			input_mt_slot(tsd->input_dev, i);
			input_mt_report_slot_state(tsd->input_dev, MT_TOOL_FINGER, false);
		}
				
		//printk("TOUCH_UP\n");
		input_sync(tsd->input_dev);

		enable_irq(tsd->irq);
	}
}

static irqreturn_t touchplus_ts_isr(int irq, void *dev_id)
{
	printk("%s\n",__FUNCTION__);
	struct touchplus_i2c_ts_data *tsdata = dev_id;

	disable_irq_nosync(irq);
	//queue_work( touchplus_wq, &tsdata->work.work);
	queue_delayed_work(touchplus_wq, &tsdata->work, 0);
	return IRQ_HANDLED;
}

static void touchplus_status_dump( struct i2c_client *client);
static int touchplus_ts_open(struct input_dev *dev)
{
	struct touchplus_i2c_ts_data *tsdata = 
		(struct touchplus_i2c_ts_data *) input_get_drvdata( dev);
	printk("in %s\n", __FUNCTION__);
	return 0;
}


static void touchplus_ts_close(struct input_dev *dev)
{
	struct touchplus_i2c_ts_data *tsdata = 
		(struct touchplus_i2c_ts_data *) input_get_drvdata( dev);
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

static ssize_t
touchplus_miscdev_read(struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
    *pos = 0;
    return 0;
}

static u8* sc_read( struct i2c_client *client);
static int touchplus_calibrate( struct i2c_client *client);
static int touchplus_miscdev_ioctl(struct inode *inode, struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
	u8 iob[20];
	struct tpreg tpreg;
	struct i2c_client *client;
	if ( g_tsd==NULL) 
		return 0;
	client = g_tsd->client;
	switch (cmd) {
		case TOUCHPLUS_IOC_CALI: /* do calibration */
			return touchplus_calibrate( client);
			break;
		case TOUCHPLUS_IOC_READ_REG: /* read a single reg */
			if (copy_from_user( &tpreg, (void *)arg, sizeof( struct tpreg)))
				return -EFAULT;
			iob[0] = tpreg.idx;
			i2c_read_bytes( client, iob, 1);
			tpreg.val = iob[0];
			copy_to_user( (void *) arg, &tpreg, sizeof( struct tpreg));
			break;
		case TOUCHPLUS_IOC_WRITE_REG: /* write a single reg */
			if (copy_from_user( &tpreg, (void *)arg, sizeof( struct tpreg)))
				return -EFAULT;
			iob[0] = tpreg.idx;
			iob[1] = tpreg.val;
			i2c_master_send( client, iob, 2);
			return 0;
		case TOUCHPLUS_IOC_READ_REGS: /* read regs */ {
			struct tpregs tpregs;
			if (copy_from_user( &tpregs, (void *)arg, sizeof( struct tpregs)))
				return -EFAULT;
			iob[0] = tpregs.base;
			i2c_read_bytes( client, iob, tpregs.len);
			memcpy( tpregs.buf, iob, tpregs.len);
			copy_to_user( (void *) arg, &tpregs, sizeof( struct tpregs));
		}
			break;
		case TOUCHPLUS_IOC_READ_RAW: /* read raw data */ {
			u8* buf =  sc_read( client);
			copy_to_user( (void *) arg, buf, 128);
		}
			break;
		case TOUCHPLUS_IOC_FW_DL: /* download firmware */
			break;
		default:
			return -1;
	}
	return 0;
}

static ssize_t
touchplus_miscdev_write(struct file *filp, const char __user *buf,
		        size_t count, loff_t *pos)
{
    printk("%s():\n", __FUNCTION__);
    return count;
}

static struct file_operations touchplus_miscdev_fops = {
	.owner = THIS_MODULE,
	.open = touchplus_miscdev_open,
	.release = touchplus_miscdev_release,
	.read = touchplus_miscdev_read,
	.write = touchplus_miscdev_write,
	.unlocked_ioctl = touchplus_miscdev_ioctl,
};


static void dumpbuf( const char *cap, const u8 *buf,  int len);

static int touchplus_calibrate( struct i2c_client *client)
{
	struct touchplus_i2c_ts_data *tsd = dev_get_drvdata(&client->dev);
	int i;
	int value;
	u8 Mbuf[1];
	u8 Ybuf[1];

	 u8 PWDbuf[1];

	Ybuf[0]=0;
	Mbuf[0]=0x03;
	printk("write Mbuf=0x03 to the regitser 0x37\n");
#if 1
//disable idle
	touchplus_read_regs(client, 0x3D, PWDbuf, 1);
	PWDbuf[0] &= ~(1<<2); 
	touchplus_write_regs(client, 0x3D, PWDbuf, 1);
	touchplus_read_regs(client, 0x3D, PWDbuf, 1);
	printk("disable idle 0x%x\n",PWDbuf[0]);
	
//
#endif	
	touchplus_write_regs(client, 0x37, Mbuf, 1);

	printk("write ok!,if gpio=high to read 0x37==0x55?\n");

}
#if 0
static int touchplus_calibrate( struct i2c_client *client)
{
	struct touchplus_i2c_ts_data *tsd = dev_get_drvdata(&client->dev);
	int i;
	int value;
	u8 Mbuf[1];
	u8 Ybuf[1];

	 u8 PWDbuf[1];

	Ybuf[0]=0;
	Mbuf[0]=0x03;
	printk("write Mbuf=0x03 to the regitser 0x37\n");
#if 1
//disable idle
	touchplus_read_regs(client, 0x3D, PWDbuf, 1);
	PWDbuf[0] &= ~(1<<2); 
	touchplus_write_regs(client, 0x3D, PWDbuf, 1);
	touchplus_read_regs(client, 0x3D, PWDbuf, 1);
	printk("disable idle 0x%x\n",PWDbuf[0]);
	
//
#endif	
	touchplus_write_regs(client, 0x37, Mbuf, 1);

	printk("write ok!,if gpio=high to read 0x37==0x55?\n");

	mdelay(1000);

	for(i=0;i<100;i++){
		value=gpio_get_value(client->irq);

		if(value){
			//FLAG=1;
			touchplus_read_regs(client, 0x37,Ybuf, 1);
			printk("i=%d,value=%d,Ybuf=0x%x\n",i,value,Ybuf[0]);
			//enable_irq(irq_flag);
			break;
		}
		mdelay(1200);
	}
	printk("out of for,i=%d\n",i);
	enable_irq(irq_flag);

	if(i<=100)
		return 0;
	else 
		return -1;

}
#endif




static void touchplus_set_rawmode( struct i2c_client *client, int mode)
{
	struct touchplus_i2c_ts_data *tsd = dev_get_drvdata(&client->dev);

	tsd->iobuf[0] = REG_CMDID;
	tsd->iobuf[1] = mode;
	i2c_master_send( client, tsd->iobuf, 2);
}

static u8* sc_read( struct i2c_client *client)
{
	struct touchplus_i2c_ts_data *tsd = dev_get_drvdata(&client->dev);

	touchplus_set_rawmode( client, TPMODE_SC);
	tsd->iobuf[0] = REG_CMDID;
	i2c_read_bytes( client, tsd->iobuf, 145);
	touchplus_set_rawmode( client, TPMODE_NORMAL);
	return tsd->iobuf;
}

static void touchplus_status_dump( struct i2c_client *client)
{
	static char msgbuf[200];
	u8 regbuf[220];
	u8 iobuf[20], verstr[20];
	char *p;
	int len=0;

	iobuf[0] = REGBASE_CFG;
	//i2c_read_bytes( client, iobuf, 14);
	touchplus_read_regs(client, iobuf[0], iobuf, 14);//changed by hjc
	memcpy( regbuf + REGBASE_CFG, iobuf, 14);

	p = msgbuf;
	len = sprintf( p, "INT: mode=0x%02X, pulseWidth=0x%02X\n", 
			regbuf[ REG_INTMODE], regbuf[REG_INTWIDTH]); p+=len;
	len = sprintf( p, "Power: mode=0x%02X\n", regbuf[ REG_POWERMODE]); p+=len;
	sprintf( verstr, "%d.%d-%d(svn %d)", 
			regbuf[ REG_VERSION] & 0x03, regbuf[ REG_VERSION+1] & 0x03,
			regbuf[ REG_VERSION+1] >>3, 
			(regbuf[REG_VERSION+3]<< 8) | regbuf[REG_VERSION+2]);

	len = sprintf( p, "Version: %s\n",  verstr); p+=len;
	len = sprintf( p, "MaxTouch: %d\n", regbuf[ REG_MAXTOUCH]); p+=len;
	len = sprintf( p, "Button: 0x%02X\n", regbuf[ REG_BUTTON]); p+=len;
    len = sprintf( p, "Tuning: 0x%02X\n", 	regbuf[ REG_TUNING]); p+=len;
    len = sprintf( p, "Scanrate: 0x%02X\n", regbuf[ REG_SCANRATE]); p+=len;
    len = sprintf( p, "IdlePeriod: 0x%02X\n",	regbuf[ REG_IDLEPERIOD]); p+=len;
    len = sprintf( p, "CMDID: 0x%02X\n", regbuf[ REG_CMDID]); p+=len;
	printk ( msgbuf);
}

static void dumpbuf( const char *cap, const u8 *buf,  int len)
{
	int i;
	printk(cap);
	for ( i=0; i< len; i++) 
		printk("%02X%c", buf[i], ( (i% 8)==0 && i >0) ? '\n': ' ');
	printk("\n");
}
///////


static ssize_t touch_mode_show(struct class *cls, char *_buf)
{
	//printk("%s\n",__FUNCTION__); 
	int i,value;
	u8 Ybuf[1];
	Ybuf[0]=0;

	calibration_flag=gpio_get_value(g_tsd->client->irq);
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
		printk("return successful\n");
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

static ssize_t touch_mode_store(struct class *cls, const char *_buf, size_t _count)
{
    u8  ucData;
    //printk("%s\n",__FUNCTION__); 
    calibration_flag = 0;
    if(!strncmp(_buf,"tp_cal" , strlen("tp_cal")))
    {
            printk("TP Calibration is start!!! \n");
	    //calibration_flag = 1;	
	   disable_irq(irq_flag);
	   touchplus_calibrate( g_tsd->client);	
    }
    return _count;
} 

#if 0
static ssize_t touch_mode_show(struct class *cls, char *_buf)
{
	//printk("%s\n",__FUNCTION__); 
	int i,value;

	
	if(calibration_flag == 1)
	{
	    calibration_flag = 0;
		printk("return successful\n");
		return sprintf(_buf,"successful");	
	}
	else
	{
	    calibration_flag = 0;
		printk("return faild\n");
		return sprintf(_buf,"fail");
	}
}

static ssize_t touch_mode_store(struct class *cls, const char *_buf, size_t _count)
{
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
    return _count;
} 

#endif
static struct class *tp_class = NULL;
static CLASS_ATTR(touchcalibration, 0666, touch_mode_show, touch_mode_store);

//////
static int touchplus_i2c_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct touchplus_i2c_ts_data *tsdata;
	struct input_dev *input;
	int error,i;
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
/*
	printk("%s,mdelay 1000*30,client->irq=%d\n",__FUNCTION__,client->irq);
	touchplus_calibrate( client);//delete for test
	for(i=0;i<30;i++){
		if(gpio_get_value(client->irq)){
			printk("calibration ok\n");
			continue;
		}else{
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
	input_set_abs_params( input, ABS_X, 0, TS_MAXX, 0, 0);
	input_set_abs_params( input, ABS_Y, 0, TS_MAXY, 0, 0);

	input_set_abs_params( input, ABS_MT_POSITION_X, 0, TS_MAXX, 0, 0);
	input_set_abs_params( input, ABS_MT_POSITION_Y, 0, TS_MAXY, 0, 0);
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
	tsdata->miscdev.minor = 70,
	tsdata->miscdev.name = "touchplus",
	tsdata->miscdev.fops = &touchplus_miscdev_fops;
	//misc_register( &(tsdata->miscdev));
	//device_init_wakeup(&client->dev, 1); //delete by hjc
	//msleep(200);
//add by hjc
#ifdef CONFIG_HAS_EARLYSUSPEND
    tsdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    tsdata->early_suspend.suspend = touchplus_ts_early_suspend;
    tsdata->early_suspend.resume = touchplus_ts_late_resume;
    register_early_suspend(&tsdata->early_suspend);
#endif
        tp_class = class_create(THIS_MODULE, "touchpanel");
    if (IS_ERR(tp_class))
        {
            printk("Create class touchpanel failed.\n");
            error = -ENOMEM;
            goto err_free_mem;
        }
        error = class_create_file(tp_class,&class_attr_touchcalibration);
        printk("Create class touchpanel is sucess\n");  
        

        printk("%s,probe touchplus ok!\n",__FUNCTION__);
        return 0;
err_free_mem:
        input_free_device(input);
        kfree(tsdata);

	
	printk("%s,probe touchplus ok!\n",__FUNCTION__);
	return 0;
err_check_functionality_failed:
	return error;
}

static int touchplus_i2c_ts_remove(struct i2c_client *client)
{
	struct touchplus_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	printk("touchplus_i2c_ts_remove\n");
	free_irq(tsdata->irq, tsdata);
	input_unregister_device(tsdata->input_dev);
	misc_deregister( &(tsdata->miscdev));
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
	buf[0] = 0xA2;
	u8 buf1[1];
	u8 buf2[1];
	buf2[0]=0x3D;
	
	printk("hjc******************%s*****************1\n",__FUNCTION__);
	disable_irq_nosync(tsdata->irq);
	printk("hjc******************%s*****************2\n",__FUNCTION__);
	touchplus_read_regs(client, pmdreg, buf1, 1);
	printk("1---read 0x3D buf1 = 0x%x\n",buf1[0]);
	
	i2c_read_bytes(client, buf2, 1);
	printk("2---read 0x3D buf2 = 0x%x\n",buf2[0]);
	
	touchplus_write_regs(client, pmdreg, buf, 1);
	printk("%s\n",__FUNCTION__);
	return 0;
}

static int touchplus_i2c_ts_resume(struct i2c_client *client)
{
	struct touchplus_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	unsigned char pmdreg=0x3D;
	u8 buf[1];
	//buf[0]=0xA4;//modify for pcb 6
	buf[0]=0xA0;
/*
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(tsdata->irq);
*/	
	

	 gpio_direction_output(TOUCH_INT_PIN, 0);
         gpio_set_value(TOUCH_INT_PIN, 0);
         mdelay(10);
	 gpio_set_value(TOUCH_INT_PIN, 1);
	 mdelay(10);
	 gpio_direction_input(TOUCH_INT_PIN);
         gpio_pull_updown(TOUCH_INT_PIN, GPIO_HIGH);

	//touchplus_calibrate( client);	
		 
	
	enable_irq(tsdata->irq);
	//touchplus_write_regs(client, pmdreg, buf, 1);
	printk("%s\n",__FUNCTION__);
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

static int __init touchplus_i2c_ts_init(void)
{
	//bind_gpiofuncs();  //changed by hjc
	printk("touchplus_i2c_init\n");

	//touchplus_wq = create_workqueue("touchplus_wq");
	touchplus_wq = create_workqueue("touchplus_wq");
	
	if(!touchplus_wq){
		printk("%s,create_wrokqueue faild!\n");
		return -ENOMEM;
		
	}
	return i2c_add_driver(&touchplus_i2c_ts_driver);
}

static void __exit touchplus_i2c_ts_exit(void)
{
	printk("touchplus_i2c_exit\n");
	i2c_del_driver(&touchplus_i2c_ts_driver);
	if(touchplus_wq)
		destroy_workqueue(touchplus_wq);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

module_init( touchplus_i2c_ts_init);
module_exit( touchplus_i2c_ts_exit);


