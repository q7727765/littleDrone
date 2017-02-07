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
#define PITCH  	now_attitude.pitch
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
extern S_INT16_XYZ ACC_AVG;
extern int16_t gyro_data[3];
extern int16_t acc_data[3];
extern int16_t mag_data[3];

extern float gyro_rdata[3];
extern float acc_rdata[3];

extern float gyro_ldata[3];
extern float acc_ldata[3];



void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);

#define M_PI_F 3.1415926
#define M_PI_F 3.1415926
#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* 癈			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/

#define SENSOR_MAX_G 8.0f		//constant g		// tobe fixed to 8g. but IMU need to  correct at the same time
#define SENSOR_MAX_W 2000.0f	//deg/s
#define ACC_SCALE  (SENSOR_MAX_G/32768.0f)
#define GYRO_SCALE  (SENSOR_MAX_W/32768.0f)


extern u8 str0[];
extern u8 str1[];
extern u8 str2[];
extern u8 str3[];
extern u8 str4[];
extern u8 str5[];
extern u8 str6[];
extern u8 str7[];
extern u8 str8[];
extern u8 str9[];
