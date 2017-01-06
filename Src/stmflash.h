#pragma once

#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//STM32 FLASH 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2013/7/27
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////	
//********************************************************************************
//V1.1修改说明
//修正了STMFLASH_Write函数地址偏移的一个bug.
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
//用户根据自己的需要设置
#define STM32_FLASH_SIZE 	64 	 		//所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN 	1              	//使能FLASH写入(0，不是能;1，使能)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
//FLASH解锁键值
#define MY_FLASH_KEY1               0X45670123
#define MY_FLASH_KEY2               0XCDEF89AB

#define NumbOfVar 15

#define EE_6050_ACC_X_OFFSET_ADDR	0
#define EE_6050_ACC_Y_OFFSET_ADDR	1
#define EE_6050_ACC_Z_OFFSET_ADDR	2
#define EE_6050_GYRO_X_OFFSET_ADDR	3
#define EE_6050_GYRO_Y_OFFSET_ADDR	4
#define EE_6050_GYRO_Z_OFFSET_ADDR	5
#define EE_PID_ROL_P	6
#define EE_PID_ROL_I	7
#define EE_PID_ROL_D	8
#define EE_PID_PIT_P	9
#define EE_PID_PIT_I	10
#define EE_PID_PIT_D	11
#define EE_PID_YAW_P	12
#define EE_PID_YAW_I	13
#define EE_PID_YAW_D	14


typedef volatile uint8_t vu8;
typedef volatile uint16_t vu16;
typedef volatile uint32_t vu32;

void Para_ResetToFactorySetup();
void Param_SavePID();
u8 EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
u8 EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

void EE_SAVE_ACC_OFFSET(void);
void EE_READ_ACC_OFFSET(void);
void EE_SAVE_GYRO_OFFSET(void);
void EE_READ_GYRO_OFFSET(void);
void EE_SAVE_PID(void);
void EE_READ_PID(void);


void STMFLASH_Unlock(void);					  //FLASH解锁
void STMFLASH_Lock(void);					  //FLASH上锁
u8 STMFLASH_GetStatus(void);				  //获得状态
u8 STMFLASH_WaitDone(u16 time);				  //等待操作结束
u8 STMFLASH_ErasePage(u32 paddr);			  //擦除页
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);//写入半字
u16 STMFLASH_ReadHalfWord(u32 faddr);		  //读出半字  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//指定地址开始写入指定长度的数据
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//指定地址开始读取指定长度数据
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//从指定地址开始读出指定长度的数据

//测试写入
void Test_Write(u32 WriteAddr,u16 WriteData);								   


















