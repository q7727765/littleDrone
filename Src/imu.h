/*
 * imu.h
 *
 *  Created on: 2016年12月29日
 *      Author: 50430
 */

#pragma once


#include "stm32f103xb.h"

#define YAW Q_ANGLE.Z
#define PITCH -Q_ANGLE.Y
#define ROLL Q_ANGLE.X


typedef struct{
				float X;
				float Y;
				float Z;}S_FLOAT_XYZ;
extern S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度
extern S_FLOAT_XYZ GYRO_I;

extern int16_t gyro_data[3];
extern int16_t acc_data[3];
extern int16_t mag_data[3];
void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);


