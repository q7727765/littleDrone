/*
 * i2.c
 *
 *  Created on: 2016年12月27日
 *      Author: 50430
 */
#include "i2c.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"


static void I2C_delay(void)
{
   for(u8 i=9;i--;){
	   __ASM("NOP");
   }
}


void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Configure GPIO pins : SDA_Pin SCL_Pin */

	GPIO_InitStruct.Pin = SDA_Pin|SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	IIC_SCL=1;
	IIC_SDA=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT() ;     //sda线输出
	IIC_SDA=1;
	IIC_SCL=1;
	I2C_delay();
	IIC_SDA=0;//START:when CLK is high,DATA change form high to low
	I2C_delay();
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
	//I2C_delay();
}
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	//I2C_delay();
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	I2C_delay();
	IIC_SCL=1;
	//I2C_delay();
	IIC_SDA=1;//发送I2C总线结束信号
	I2C_delay();
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;

	//IIC_SCL=1;I2C_delay();
	SDA_IN();      //SDA设置为输入
	IIC_SDA=1;I2C_delay();
	IIC_SCL=1;I2C_delay();
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			//SendChar("i2c hahaha \r\n");
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0
	//I2C_delay();
	return 0;
}
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	//I2C_delay();
	SDA_OUT();
	IIC_SDA=0;
	I2C_delay();
	IIC_SCL=1;
	I2C_delay();
	IIC_SCL=0;
//	I2C_delay();
}
//不产生ACK应答
void IIC_NAck(void)
{
	IIC_SCL=0;
	//I2C_delay();
	SDA_OUT();
	IIC_SDA=1;
	I2C_delay();
	IIC_SCL=1;
	I2C_delay();
	IIC_SCL=0;
	//I2C_delay();
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
	SDA_OUT();
    IIC_SCL=0;//拉低时钟开始数据传输
    //I2C_delay();
    for(t=0;t<8;t++)
    {
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1;
        I2C_delay();   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		I2C_delay();
		IIC_SCL=0;
		I2C_delay();
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA = 1;//这里要释放总线啊兄弟，你读数据的时候ack拉低了SDA，不释放，后面还连读个毛啊。
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0;
        I2C_delay();
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;
        I2C_delay();
    }
    if (!ack){
        IIC_NAck();//发送nACK
    }
    else{
        IIC_Ack(); //发送ACK
    }
    return receive;
}


u8 IIC_Read_Reg(u8 addr,u8 reg)
{
	u8 res;
    IIC_Start();
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
	IIC_Wait_Ack();		//等待应答
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令
    IIC_Wait_Ack();		//等待应答
	res=IIC_Read_Byte(0);//读取数据,发送nACK
    IIC_Stop();			//产生一个停止条件
	return res;
}

u8 IIC_Write_Reg(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
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

u8 IIC_Write_Reg_Len(u8 addr,u8 reg,u8 len,u8 *buf)
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

u8 IIC_Read_Reg_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{

 	IIC_Start();
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令

	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();
		return 1;
	}

    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答

    IIC_Start();
    IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令
	IIC_Wait_Ack();		//等待应答

	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK
		else *buf = IIC_Read_Byte(1);		//读数据,发送ACK

		len--;
		buf++;
	}
    IIC_Stop();	//产生一个停止条件

	return 0;

}
