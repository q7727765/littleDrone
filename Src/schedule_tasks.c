/*
 * schedule_tasks.c
 *
 *  Created on: 2016年12月28日
 *      Author: 50430
 */


#include "delay.h"
#include "usart.h"
#include "hmc5883l.h"
#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include "mpu6050.h"

int16_t mag_data[3];
extern mag_t mag;
extern u8 mpu6050_buffer[14];					//iic读取后存放数据

extern S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//最新一次读取值
extern S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//零漂
extern u8							GYRO_OFFSET_OK;
extern u8							ACC_OFFSET_OK;

void taskUpdateMPU6050(void){

	uint8_t xl,xm;
	int16_t x;

	MPU6050_Read();
	MPU6050_Dataanl();

	SendChar("X:");
	SendInt(MPU6050_ACC_LAST.X);
	_n();
	SendChar("Y:");
	SendInt(MPU6050_ACC_LAST.Y);
	_n();
	SendChar("Z:");
	SendInt(MPU6050_ACC_LAST.Z);
	_n();
	SendChar("GX:");
	SendInt(MPU6050_GYRO_LAST.X);
	_n();
	SendChar("GY:");
	SendInt(MPU6050_GYRO_LAST.Y);
	_n();
	SendChar("GZ:");
	SendInt(MPU6050_GYRO_LAST.Z);
	_n();
////	double a,b,c;
//	xm = IIC_Read_Reg(MAG_ADDRESS,MAG_DATA_REGISTER+2);
//	xl = IIC_Read_Reg(MAG_ADDRESS,MAG_DATA_REGISTER+3);
////	sta = hmc5883lRead(mag_data);
//	x = (int16_t)(xm<<8 | xl);
//	SendChar("x:");
//	SendInt(x);
//	_n();

//	a = (double)(mag_data[0]<<8 + mag_data[1]);
//	SendChar("X:");
//	SendInt(mag_data[0]);
//	_n();

}
