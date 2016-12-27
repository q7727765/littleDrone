/*
 * i2c.h
 *
 *  Created on: 2016年12月27日
 *      Author: 50430
 */

#ifndef SRC_DRIVERS_I2C_H_
#define SRC_DRIVERS_I2C_H_

#include "stm32f103xb.h"


#define SDA_IN()  {GPIOA->CRH &= ~(0xF<<12);GPIOA->CRH |= (0x8<<12);}	//PA11输入模式
#define SDA_OUT() {GPIOA->CRH &= ~(0xF<<12);GPIOA->CRH |= (0x1<<12);} //PA11输出模式
//IO操作函数
#define IIC_SCL    PAout(12) //SCL
#define IIC_SDA    PAout(11) //SDA
#define READ_SDA   PAin(11)  //输入SDA

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

#endif /* SRC_DRIVERS_I2C_H_ */
