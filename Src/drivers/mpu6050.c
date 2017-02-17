#include <stm32f103xb.h>
#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"   
#include "i2c.h"
#include "stdbool.h"
#include "HAL.h"
#include "stdint.h"
#include "stmflash.h"

u8						mpu6050_buffer[14];					//iic读取后存放数据

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//MPU6050 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码

bool MPU6050DetectGyro(gyro_t *gyro)
{
	u8 res;

	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res!=MPU_ADDR) return false;

	gyro->init = MPU_Init_Gyro;
	gyro->read = mpuGyroRead;
	//gyro->isDataReady = mpuIsDataReady;

	// 16.4 dps/lsb scalefactor
	gyro->scale = 2000.f / 32768.f;

	return true;
}

bool MPU6050DetectAcc(acc_t *acc)
{
    acc->init = MPU_Init_Acc;
    acc->read = mpuAccRead;
    //acc->revisionCode = (mpuDetectionResult.resolution == MPU_HALF_RESOLUTION ? 'o' : 'n'); // es/non-es variance between MPU6050 sensors, half of the naze boards are mpu6000ES.

	return true;
}

bool mpuAccRead(int16_t *data)
{
	u8 res;
	res = IIC_Read_Reg_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,mpu6050_buffer);
	data[0] = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]);
	data[1] = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]);
	data[2] = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);

	return !res;
}

bool mpuGyroRead(int16_t *data)
{
	u8 res;
	res = IIC_Read_Reg_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG+8,6,mpu6050_buffer+8);
	data[0] = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	data[1] = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]);
	data[2] = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]);

	return !res;
}

void MPU6050_Read(void)
{
	IIC_Read_Reg_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,mpu6050_buffer);
}


void MPU_Init_Gyro(void)
{ 
	u8 res;
	IIC_Init();//初始化IIC总线
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    delay_ms(300);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps

	MPU_Set_LPF(42);
	//MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
//	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,
			0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);	//INT引脚低电平有效

	MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X03);	//设置CLKSEL,PLL Z轴为参考
	//MPU_Set_Rate(50);						//设置采样率为50Hz

}

void MPU_Init_Acc(void)
{
	MPU_Set_Accel_Fsr(2);					//加速度传感器,±8g
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res=1;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
	addr =addr;
	while(len--)
	{
		*buf=MPU_Read_Byte(reg);
		buf++;
		reg++;
	}
	return 0;//+++++

// 	IIC_Start(); 
//	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
//	if(IIC_Wait_Ack())	//等待应答
//	{
//		IIC_Stop();		 
//		return 1;		
//	}
//    IIC_Send_Byte(reg);	//写寄存器地址
//    IIC_Wait_Ack();		//等待应答
//    IIC_Start();
//	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
//    IIC_Wait_Ack();		//等待应答 
//	delay_us(4);//+++++++
//	while(len)
//	{
//		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
//		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
//		delay_us(4);
//		len--;
//		buf++; 
//	}    
//    IIC_Stop();	//产生一个停止条件 
//	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}


