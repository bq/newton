/* drivers/input/touchscreen/nt11003_touch.c
 *
 * Copyright (C) 2010 - 2011 Goodix, Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/input/mt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/proc_fs.h>
#include "nt11003.h"
#include "UniTouch7inch.h"
#include <mach/board.h>

#define BABBAGE_NT11003_TS_RST1   RK29_PIN6_PC3
#define BABBAGE_NT11003_TS_INT1   RK29_PIN0_PA2


#define INT_PORT BABBAGE_NT11003_TS_INT1
#define TS_INT gpio_to_irq(INT_PORT)

// Chip Reset Type
#define  HW_RST      0
#define  SW_RST      1
#define  SECTORSIZE  2048
//#define  nvctp_GobalRset(x,t)   (  )
//#define  WR_CMD      0
//#define  RD_CMD		 1
enum
{
  RS_OK         = 0,
  RS_INIT_ER    = 8,
  RS_ERAS_ER    = 9,
  RS_FLCS_ER    = 10,
  RS_WD_ER      = 11
} ;

static struct i2c_client *this_client;

//extern void rk29_judge_charging(void);
//extern char yj_usb_count;



/*******************************************************	
Description:
	Read data from the i2c slave device;
	This operation consisted of 2 i2c_msgs,the first msg used
	to write the operate address,the second msg used to read data.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer
*********************************************************/
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	int retries = 0;

	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];
 	msgs[0].scl_rate = 100*1000;
 	
	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len-1;
	msgs[1].buf=&buf[1];
	msgs[1].scl_rate = 100*1000;

	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,msgs, 2);
		if(ret == 2)break;
		retries++;
	}
	return ret;
}

/*******************************************************	
Description:
	write data to the i2c slave device.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;
	int retries = 0;

	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;		
	msg.scl_rate = 100*1000;
	
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,&msg, 1);
		if(ret == 1)break;
		retries++;
	}
	return ret;
}
/************************************************************
Description:
	update TPnt11003
***********************************************************/

static int i2c_read_bytes1(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	int retries = 0;

	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=0x7f;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];
 	msgs[0].scl_rate = 100*1000;
 	
	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=0x7f;
	msgs[1].len=len-1;
	msgs[1].buf=&buf[1];
	msgs[1].scl_rate = 100*1000;

	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,msgs, 2);
		if(ret == 2)break;
		retries++;
	}
	return ret;
}

/*******************************************************	
Description:
	write data to the i2c slave device.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int i2c_write_bytes1(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;
	int retries = 0;

	msg.flags=!I2C_M_RD;
	msg.addr=0x7f;
	msg.len=len;
	msg.buf=data;		
	msg.scl_rate = 100*1000;
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,&msg, 1);
		if(ret == 1)break;
		retries++;
	}
	return ret;
}

static int novatek_ts_ChipID(struct i2c_client *client)
{

    int ret;
   // struct nt11003_ts_data *ts = i2c_get_clientdata(client);
    uint8_t Write_data[] ={0xff, 0xf0, 0x00};
    uint8_t Read_data[]= {0}; 

    ret = i2c_write_bytes(client, Write_data, (sizeof(Write_data)/sizeof(Write_data[0])));

    if (ret <= 0)
    {
      dev_err(&(client->dev),"I2C transfer error. Number:%d\n ", ret);
    }

   ret = i2c_read_bytes(client, Read_data, 1);

    if (ret <= 0)
	{
		dev_err(&(client->dev),"I2C transfer error. Number:%d\n ", ret);

    }
    return (Read_data[0]);

}

/******************************************************************************************************
Novatek Capacitor Touch Drive Controller standard firmware source code : Boot Loader function 
Boot Loader :  
       1, initial boot loader function ;
           This step can do two methods: 
            one is hardware reset :
                     _________                                                                                                               _____________
           GRST                 |____________________________________________________________________|
                                   |<----- 5ms -----> |i2c_write_bytes(0x7F, 0x00, 0x00, 1) |<-- 2ms -->|
                                   
           Other is Software reset:
           i2c_write_bytes(0x7F, 0x00, 0xA5, 1);
           nvctp_DelayMs(10);
           i2c_write_bytes(0x7F, 0x00, 0x00, 1);
           nvctp_DelayMs(2);
           
         Then check this operation is finished,
           i2c_read_bytes(0x7F, 0x00, &iic_data_buffer,1);
           if iic_data_buffer == AAH then the operation is finished ,the system can go next step, else try this operation again; 
           
       2, erase flash memery
          this chip internal flash made by 8 sectors,we can erase one sector but one time. so must erase 8 Circle( i = 0 ..7)
          iic_data_buffer[0] = 0x33;
          iic_data_buffer[1] = 0x00 + (i*8);
          i2c_write_bytes(0x7F, 0x00, iic_data_buffer, 2);
          nvctp_DelayMs(75);
          i2c_read_bytes(0x7F, 0x00, &iic_data_buffer,1);
          if iic_data_buffer == AAH then the operation is finished ,the system can go next step, else try this operation again; 
      3, Write data to flash
          we can write 8 byte data for once circle,
          iic_data_buffer[0] = 0x55;
          iic_data_buffer[1] = Flash Adress High Byte;
          iic_data_buffer[2] = Flash Adress Low Byte;
          iic_data_buffer[3] = 8;

******************************************************************************************************/
void nvctp_DelayMs(unsigned long wtime)
{
	mdelay(wtime);
}
/*******************************************************************************************




********************************************************************************************/
unsigned char nvctp_InitBootloader(unsigned char bType,struct i2c_client *client)
{
	unsigned char ret = RS_OK;
    unsigned char status;
	unsigned char iic_buffer[13];
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	iic_buffer[0] = 0x00;
	if(bType == HW_RST)
	 {
		//nvctp_GobalRset(1,2);
		//nvctp_GobalRset(0,10);
		iic_buffer[1] = 0x00;
		i2c_write_bytes1(ts->client, iic_buffer, 2);
		nvctp_DelayMs(2);
		//nvctp_GobalRset(1,2);				
	 }
	else if(bType == SW_RST)
	{
	    iic_buffer[1] = 0xA5;
		i2c_write_bytes1(ts->client, iic_buffer, 2);
        nvctp_DelayMs(10);
		iic_buffer[1] = 0x00;
        i2c_write_bytes1(ts->client, iic_buffer, 2);
        nvctp_DelayMs(2);
	}

  /*Read status*/
   i2c_read_bytes1(ts->client,iic_buffer,2);
   	printk("iic_buffer[1] %d\n",iic_buffer[1]);
   if(iic_buffer[1] != 0xAA)
   	{
	   ret = RS_INIT_ER;
   	}
   	printk("ret %d\n",ret);
   return ret;
}
/*******************************************************************************************




********************************************************************************************/
unsigned char nvctp_EraseSector(unsigned char cSector,struct i2c_client *client)
{
	unsigned char ret = RS_ERAS_ER;
	unsigned char cBuffer[3];
	unsigned char i,status; 
	
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	cBuffer[0] = 0x00;
	
	cBuffer[1] = 0x33;

	for(i = 5; i >= 0; i --)
	{
		
		cBuffer[2] = 0x00;
			
		i2c_write_bytes1(ts->client,  cBuffer, 3);
		nvctp_DelayMs(10);
		
		/*Read status*/
   		i2c_read_bytes1(ts->client, cBuffer,2);
   		if(cBuffer[1] == 0xAA)
   		{
	   		ret = cSector;
			break;
   		}
		
		nvctp_DelayMs(10);
	}
	return ret;
}
/*******************************************************************************************




********************************************************************************************/
unsigned char nvctp_ReadFinalChecksum(unsigned long flash_addr, unsigned long final_checksum,struct i2c_client *client)
{
	unsigned char iic_data_buffer[14];
	unsigned long wValue;
  unsigned char ret = RS_OK;
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	
	/*inital bootloader*/
	ret = nvctp_InitBootloader(SW_RST,ts->client);
	if(ret != RS_OK)
	 {
		return ret;
	 }
	iic_data_buffer[0]=0x00;	
	
	iic_data_buffer[1] = 0x99;
	iic_data_buffer[2] = (unsigned char)(flash_addr >> 8);
	iic_data_buffer[3] = flash_addr&0xFF;
	iic_data_buffer[4] = 8;
	i2c_write_bytes1(ts->client,iic_data_buffer,5);
	nvctp_DelayMs(2);
	i2c_read_bytes1(ts->client,iic_data_buffer,14);
	wValue = (unsigned long)iic_data_buffer[12]<<8 |iic_data_buffer[13];

	if(wValue != final_checksum)
	{
		ret = RS_FLCS_ER;
	}

	return ret;
	
}
/*******************************************************************************************




********************************************************************************************/
unsigned char nvctp_ReadWriteDataStatus(unsigned long *flash_addr,unsigned char iic_read_length,struct i2c_client *client)
{
	unsigned long i,j;
	unsigned char ret = RS_OK;
	unsigned char iic_data_buffer[14];
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	iic_data_buffer[0]=0x00;	
	
	if(iic_read_length > 14)
	  iic_read_length = 0;
	
	for(i = 0; i < 5; i ++)
		{
			i2c_read_bytes1(ts->client,iic_data_buffer,iic_read_length);

			if(iic_data_buffer[1] == 0xAA)
			{
				break;
			}
			else
			{
				if (i == 4)
				{
					j = (*flash_addr + 1) / 2048;
					if( (*flash_addr + 1) % 2048 )
						j++;
					if (nvctp_EraseSector(j,ts->client)== (unsigned char)j)
					{
						*flash_addr = (j - 1)*SECTORSIZE  - 8;
					}
					else
					{
						ret = RS_WD_ER;// report error..
						break;
					}
				}
			}
			nvctp_DelayMs(1);
		}
	return ret;
}
/*******************************************************************************************




********************************************************************************************/

unsigned char nvctp_WriteDatatoFlash( unsigned char *fw_BinaryData, unsigned long BinaryDataLen,struct i2c_client *client)
{
   	unsigned char ret = RS_OK;
  	unsigned char iic_data_buffer[14];
	  unsigned char i,j,count;
	  unsigned char iic_buffer[16] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	  unsigned char Checksum[16] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    unsigned long flash_addr;
		struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	  iic_data_buffer[0]=0x00;	
	  count = 0;
	
WriteDatatoFlash:
		
    flash_addr = 0;
    count ++;
    for( i = 0; i < 128; i++ )
     {
     	 flash_addr = 128*i;  
     	 for (j = 0; j < 16; j++)
     	 {
        /* Write Data to flash*/
        iic_data_buffer[1] = 0x55;
	      iic_data_buffer[2] = (unsigned char)(flash_addr >> 8);
        iic_data_buffer[3] = (unsigned char)flash_addr;
		    iic_data_buffer[4] = 8;
		    
		    iic_data_buffer[6] = fw_BinaryData[flash_addr + 0];
		    iic_data_buffer[7] = fw_BinaryData[flash_addr + 1];
		    iic_data_buffer[8] = fw_BinaryData[flash_addr + 2];
		    iic_data_buffer[9] = fw_BinaryData[flash_addr + 3];
		    iic_data_buffer[10] = fw_BinaryData[flash_addr + 4];
		    iic_data_buffer[11]= fw_BinaryData[flash_addr + 5];
		    iic_data_buffer[12]= fw_BinaryData[flash_addr + 6];
		    iic_data_buffer[13]= fw_BinaryData[flash_addr + 7];
        
		    Checksum[j] = ~(iic_data_buffer[2]+iic_data_buffer[3]+iic_data_buffer[4]+iic_data_buffer[6]+\
		    	         iic_data_buffer[7]+iic_data_buffer[8]+iic_data_buffer[9]+\
		    	  		 iic_data_buffer[10]+iic_data_buffer[11]+iic_data_buffer[12]+iic_data_buffer[13]) + 1;
		    iic_data_buffer[5] = Checksum[j];
		    
		    i2c_write_bytes1(ts->client, iic_data_buffer, 14);
		    nvctp_DelayMs(1);
		    
		    /*Read Status*/
		   // i2c_read_bytes1(ts->client, iic_data_buffer,2);
		   // //if(nvctp_ReadWriteDataStatus(&flash_addr, 1,ts->client) == RS_WD_ER)
		   // if(iic_data_buffer[1] != 0xAA)
		   // {
		   //       	ret = RS_WD_ER;
		   //       	return ret;
	     // }
		  flash_addr += 8;		
	  }
	  nvctp_DelayMs(10);
	  
    /*Setup force genrate Check sum */
    flash_addr = 128*i;  
    for (j = 0; j < 16; j++)
    {
				iic_data_buffer[1] = 0x99;
				iic_data_buffer[2] = (unsigned char)(flash_addr >> 8);
      	iic_data_buffer[3] = (unsigned char)flash_addr;
		    iic_data_buffer[4] = 8;
				i2c_write_bytes1(ts->client, iic_data_buffer,5);
				nvctp_DelayMs(1);
				i2c_read_bytes1(ts->client, iic_data_buffer, 14);
		
				if(iic_data_buffer[5] != Checksum[j] )
					{
			  			ret = RS_WD_ER;
					}
				flash_addr += 8;	
	  }
   }
   
   if((ret == RS_WD_ER)||(count <5))
   goto WriteDatatoFlash;
	/* Check final check sum for machine code*/
//	ret = nvctp_ReadFinalChecksum((BinaryDataLen - 8), ((unsigned long)fw_BinaryData[BinaryDataLen - 2] << 8 |fw_BinaryData[BinaryDataLen - 1]));
	return ret;
	
}
/*******************************************************************************************




********************************************************************************************/
unsigned char nvctp_Bootloader(unsigned char *nvctp_binaryfile , unsigned long nvctp_binaryfilelength,struct i2c_client *client)
{	
	unsigned char i;
	unsigned char ret = RS_OK;
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	
//    /*inital bootloader*/
//	ret = nvctp_InitBootloader(SW_RST,ts->client);
//	if(ret != RS_OK)
//	 {
//		return ret;
//	 }
    /*Erase Sector*/
	//for(i = 0; i < 8; i ++)
	{
		ret = nvctp_EraseSector(0,ts->client);
	//	if(ret != i)
	//	  return ret;
	}
	/*Write binary data to flash*/
	ret = nvctp_WriteDatatoFlash(nvctp_binaryfile,nvctp_binaryfilelength,ts->client);
	if(ret != RS_OK)
	{
		return ret;
	}
	
	return ret;
}

#define  FW_DATASIZE      (1024*16)
#define  FW_CHECKSUM_ADDR     (FW_DATASIZE - 8)
static int  nvctp_CheckIsBootloader(struct i2c_client *client)
{
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
    unsigned int FW_CHECKSUM ;
    unsigned char iic_buffer[3];
 printk("nvctp_CheckIsBootloader");
	FW_CHECKSUM = (unsigned int)(nvctp_BinaryFile[FW_DATASIZE-2]<< 8 |nvctp_BinaryFile[FW_DATASIZE-1]);
    if(nvctp_ReadFinalChecksum(FW_CHECKSUM_ADDR,FW_CHECKSUM,ts->client))
    	{
    	  printk("nvctp_CheckIsBootloader1");
	       nvctp_Bootloader(nvctp_BinaryFile,sizeof(nvctp_BinaryFile),ts->client);
		   
    	}
    	else
    	{
    		iic_buffer[0] = 0x00;
    		iic_buffer[1] = 0xA5;
		    i2c_write_bytes1(ts->client, iic_buffer, 2);			    	 		  		    
    	}
	     gpio_set_value(RK29_PIN6_PC3,GPIO_HIGH);  
       nvctp_DelayMs(50);
       gpio_set_value(RK29_PIN6_PC3,GPIO_LOW);
       nvctp_DelayMs(50);
       gpio_set_value(RK29_PIN6_PC3,GPIO_HIGH); 
}

/*******************************************************
Description:
	Goodix touchscreen work function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static char last_num = 0;
static  uint8_t last_track_id[MAX_FINGER_NUM] = {0};
extern void rk2918_charge_enable_bl(char flag);

static void nt11003_ts_work_func(struct work_struct *work)
{	
	int ret=-1;
	int tmp = 0;
	uint8_t  point_data[MAX_FINGER_NUM*6+1]={0};
	uint8_t  check_sum = 0;
	uint16_t  finger_current = 0;
	uint16_t  finger_bit = 0;
	unsigned int  count = 0, point_count = 0;
	unsigned int position = 0;	
	uint8_t track_id[MAX_FINGER_NUM] = {0};
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned char index = 0;
	unsigned char touch_num = 0;
	char i = 0, j = 0;
	
	struct nt11003_ts_data *ts = container_of(work, struct nt11003_ts_data, work_ed.work);

	point_data[0] = READ_COOR_ADDR;		//read coor address
	ret=i2c_read_bytes(ts->client, point_data,  sizeof(point_data)/sizeof(point_data[0]));
	if(ret <= 0)	
	{
		dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);	
	}

 	touch_num = MAX_FINGER_NUM;
  	for(index = 0; index < MAX_FINGER_NUM; index++)
  	{
  		position = 1 + 6*index;
  		if(point_data[position]&0x3== 0x03)
  		  touch_num--;	
  	}
	
  	//printk("touch_num=%d last_num=%d\n",touch_num,last_num);
	if(touch_num)
	{
		for(index=0; index<touch_num; index++)
		{
			position = 1 + 6*index;
		  track_id[index] = (point_data[position]>>3)-1;
			input_x = (unsigned int) (point_data[position+1]<<4) + (unsigned int)( point_data[position+3]>>4);
			input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
			input_w =(unsigned int) (point_data[position+4])+127;		
			//input_x = input_x *SCREEN_MAX_HEIGHT/(TOUCH_MAX_HEIGHT);	
			//input_y = input_y *SCREEN_MAX_WIDTH/(TOUCH_MAX_WIDTH);

			if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))continue;
//			printk("track_id[index]=%d,input_x = %d,input_y = %d, input_w = %d\n",track_id[index], input_x, input_y, input_w);

			input_mt_slot(ts->input_dev, track_id[index]);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);		
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);			
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);

	//		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
	//		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, track_id[index]);
	//		input_mt_sync(ts->input_dev);
		}
#if 0
		printk("track_id[]=");
		for (i=0;i<touch_num;i++)
		{
				printk("%4d",track_id[i]);
		}
		printk("\nlast_track_id[]=");
		for (i=0;i<last_num;i++)
		{
				printk("%4d",last_track_id[i]);
		}
		printk("\n");
#endif
		if (last_num >= touch_num)
		{
			for (i=0; i<last_num; i++)
			{
				if (last_track_id[i] != track_id[j])
				{
					input_mt_slot(ts->input_dev,last_track_id[i]);
		//			printk("there go away  %d\n\n",last_track_id[i]);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
				}
				else
				{
					j++;					
				}
			}
		}
	//	printk("i=%d,j=%d\n",i,j);
		memcpy(last_track_id,track_id,MAX_FINGER_NUM);
		
	}
	else
	{
//		printk("this an error\n");
//		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
//		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
//		input_mt_sync(ts->input_dev);
		for (i=0;i<10;i++)
		{
			input_mt_slot(ts->input_dev,i);
//			printk("different there go away  %d\n\n",last_track_id[i]);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}

	}


	input_sync(ts->input_dev);
	last_num = touch_num;

	if(ts->use_irq)
	enable_irq(ts->client->irq);

}

/*******************************************************
Description:
	Timer interrupt service routine.

Parameter:
	timer:	timer struct pointer.
	
return:
	Timer work mode. HRTIMER_NORESTART---not restart mode
*******************************************************/
static enum hrtimer_restart nt11003_ts_timer_func(struct hrtimer *timer)
{
	struct nt11003_ts_data *ts = container_of(timer, struct nt11003_ts_data, timer);
	queue_work(nt11003_wq, &ts->work_ed);
	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Description:
	External interrupt service routine.

Parameter:
	irq:	interrupt number.
	dev_id: private data pointer.
	
return:
	irq execute status.
*******************************************************/
static irqreturn_t nt11003_ts_irq_handler(int irq, void *dev_id)
{
	struct nt11003_ts_data *ts = dev_id;
	disable_irq_nosync(ts->client->irq);
	queue_delayed_work(nt11003_wq, &ts->work_ed,1);
	//queue_work(nt11003_wq, &ts->work);
	
	return IRQ_HANDLED;
}


/*******************************************************
Description:
	Goodix touchscreen probe function.

Parameter:
	client:	i2c device struct.
	id:device id.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
#define TOUCH_RESET_PIN RK29_PIN6_PC3
#define TOUCH_INT_PIN   RK29_PIN0_PA2

extern char tp_flag;
static int nt11003_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int retry=0;
	struct nt11003_ts_data *ts;
	char *version_info = NULL;
    struct input_dev *input_dev; 
	struct nt11003_platform_data *pdata = pdata =  client->dev.platform_data;

	if (tp_flag == 0)
		printk("Install touch driver..the tp is nt11003,not touchplus tp_flag=%d sc\n",tp_flag);
	else
	{
		printk("the tp is touchplus ,not nt11003 tp_flag=%d\n",tp_flag);
		return 0;	
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk( "Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
		//printk("i2cok================================.\n");
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}


	client->irq=TS_INT;
	
	i2c_connect_client_nt11003 = client;
	
	INIT_DELAYED_WORK(&ts->work_ed, nt11003_ts_work_func);
	ts->client = this_client=client;
	i2c_set_clientdata(client, ts);
	   	if (pdata->init_platform_hw)                              
		pdata->init_platform_hw();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk("Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	

	ts->abs_x_max = 1024;
	ts->abs_y_max = 600;

//	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
//	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
//	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 						// absolute coor (x,y)

	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);

	input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM);


//	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
//	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
//	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
//	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
	

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NT11003_I2C_NAME;//nt11003_ts_name;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;	//screen firmware version
	
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk("Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	ts->bad_data = 0;
		
///////////////////////////////////////////////////////////////////////////



  //nvctp_CheckIsBootloader(ts->client);

/////////////////////////////////////////////////////////////////////////////		
		

		//enable_irq(client->irq);  
    
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nt11003_ts_early_suspend;
	ts->early_suspend.resume = nt11003_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	if (client->irq)
	{
		ret  = request_irq(client->irq, nt11003_ts_irq_handler , IRQF_TRIGGER_FALLING/*IRQF_DISABLED*/, "nt1103-ts", ts); //irq_table[ts->int_trigger_type],client->name, ts);
		if (ret != 0) {
			printk("Cannot allocate ts INT!ERRNO:%d\n", ret);
			gpio_direction_input(INT_PORT);
			gpio_free(INT_PORT);
		}
		else 
		{	
		//	disable_irq(client->irq);
			ts->use_irq = 1;
//			printk("Reques EIRQ %d succesd on GPIO:%d\n",TS_INT,INT_PORT);
		}

	}
	

	printk("Start %s in %s mode\n", 
		ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	printk( "Driver Modify Date:2011-06-27\n");
	#if 0
		rk29_judge_charging();
		//yj_usb_count = 1;
		//rk2918_charge_enable_bl(5);	
	#endif
	return 0;

err_init_godix_ts:
	if(ts->use_irq)
	{
		ts->use_irq = 0;
		free_irq(client->irq,ts);
		gpio_direction_input(INT_PORT);
		gpio_free(INT_PORT);
	}
	else 
		hrtimer_cancel(&ts->timer);

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);
err_i2c_failed:	
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
err_create_proc_entry:
	return ret;
}


/*******************************************************
Description:
	Goodix touchscreen driver release function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int nt11003_ts_remove(struct i2c_client *client)
{
	
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
#ifdef CONFIG_TOUCHSCREEN_NT11003_IAP
	remove_proc_entry("nt11003-update", NULL);
#endif
	//nt11003_debug_sysfs_deinit();
	if (ts && ts->use_irq) 
	{
		gpio_direction_input(INT_PORT);
		gpio_free(INT_PORT);
		free_irq(client->irq, ts);
	}	
	else if(ts)
		hrtimer_cancel(&ts->timer);
	
	dev_notice(&client->dev,"The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	
	return 0;
}

static int nt11003_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	char i = 0;
#if 0
	  u8 write_cmd1[3] = {0xff,0x8f,0xff};
  	u8 write_cmd2[2] = {0x00,0xae};
  	
    i2c_write_bytes(this_client, write_cmd1, 3); 
	  i2c_write_bytes(this_client, write_cmd2, 2);
#endif
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
    //gpio_set_value(RK29_PIN6_PD3, 1);
      cancel_work_sync(&ts->work_ed);
	  disable_irq(client->irq);
	  for (i = 0; i<MAX_FINGER_NUM; i++)
	  {
		input_mt_slot(ts->input_dev,last_track_id[i]);		
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		last_track_id[i] = 0;
	  }
	  return 0;
}

static int nt11003_ts_resume(struct i2c_client *client)
{
#if 0
    u8 write_cmd1[3] = {0xff,0x8f,0xff};
    u8 write_cmd2[2] = {0x00,0x00};

    i2c_write_bytes(this_client, write_cmd1,  3);
	  i2c_write_bytes(this_client, write_cmd2,  2);


    gpio_set_value(RK29_PIN6_PC3,GPIO_HIGH);  
    mdelay(50);
    gpio_set_value(RK29_PIN6_PC3,GPIO_LOW);
    mdelay(50);
    gpio_set_value(RK29_PIN6_PC3,GPIO_HIGH);
#endif
	//gpio_set_value(RK29_PIN6_PD3, 0);
	  enable_irq(client->irq);
	  return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nt11003_ts_early_suspend(struct early_suspend *h)
{
	struct nt11003_ts_data *ts;
	ts = container_of(h, struct nt11003_ts_data, early_suspend);
	nt11003_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void nt11003_ts_late_resume(struct early_suspend *h)
{
	struct nt11003_ts_data *ts;
	ts = container_of(h, struct nt11003_ts_data, early_suspend);
	nt11003_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id nt11003_ts_id[] = {
	{ NT11003_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver nt11003_ts_driver = {
	.probe		= nt11003_ts_probe,
	.remove		= nt11003_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= nt11003_ts_suspend,
	.resume		= nt11003_ts_resume,
#endif
	.id_table	= nt11003_ts_id,
	.driver = {
		.name	= NT11003_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

/*******************************************************	
Description:
	Driver Install function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit nt11003_ts_init(void)
{
	int ret;
	
	nt11003_wq = create_workqueue("nt11003_wq");		//create a work queue and worker thread
	if (!nt11003_wq) {
		printk(KERN_ALERT "creat workqueue faiked\n");
		return -ENOMEM;
		
	}
	ret=i2c_add_driver(&nt11003_ts_driver);
	return ret; 
}

/*******************************************************	
Description:
	Driver uninstall function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit nt11003_ts_exit(void)
{
	printk(KERN_ALERT "Touchscreen driver of guitar exited.\n");
	i2c_del_driver(&nt11003_ts_driver);
	if (nt11003_wq)
		destroy_workqueue(nt11003_wq);		//release our work queue
}

late_initcall(nt11003_ts_init);
module_exit(nt11003_ts_exit);

MODULE_DESCRIPTION("Goodix Touchscreen Driver");
MODULE_LICENSE("GPL");
