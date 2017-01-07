/*
 * imu.h
 *
 *  Created on: 2016年12月29日
 *      Author: 50430
 */

#pragma once


#include "stm32f103xb.h"
#include "HAL.h"

#define YAW 	now_attitude.yaw
#define PITCH  -now_attitude.pitch
#define ROLL 	now_attitude.roll


typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;
}S_INT16_XYZ;
typedef struct{
				float X;
				float Y;
				float Z;
}S_FLOAT_XYZ;


extern attitude_t now_attitude;			//四元数计算出的角度
extern S_FLOAT_XYZ GYRO_I;

extern int16_t gyro_data[3];
extern int16_t acc_data[3];
extern int16_t mag_data[3];
void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);


