#include "control.h"
#include "time.h"
#include "mpu6050.h"
#include "imu.h"
#include "config.h"
#include "stdint.h"

typedef volatile int8_t vs8;
typedef volatile int16_t vs16;
typedef volatile int32_t vs32;

//++++--
uint16_t rc_yaw;
uint16_t rc_thr;



PID PID_ROL,PID_PIT,PID_YAW;
u8 ARMED = 0;

float rol_i=0,pit_i=0,yaw_p=0;
float bbuf;
float Get_MxMi(float num,float max,float min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}

void CONTROL(float rol_now, float pit_now, float yaw_now, float rol_tar, float pit_tar, float yaw_tar)
{
	u16 moto1=0,moto2=0,moto3=0,moto4=0;
	
	vs16 yaw_d;
	float err_rol = rol_tar - rol_now;
	float err_pit = pit_tar + pit_now;
	//float err_yaw = yaw_tar - yaw_now;
	
	
	//*******************ROLL的计算*********************//
	rol_i += err_rol;
	if(rol_i>2000)
	rol_i=2000;
	if(rol_i<-2000)
	rol_i=-2000;
	
	PID_ROL.pout = PID_ROL.P * err_rol;
	PID_ROL.dout = -PID_ROL.D*MPU6050_GYRO_LAST.X;
	PID_ROL.iout = PID_ROL.I*rol_i;
	////////////////////////////////////////////////////
	
	//*******************PIT的计算*********************//
	pit_i += err_pit;
	if(pit_i>2000)
	pit_i=2000;
	if(pit_i<-2000)
	pit_i=-2000;

	PID_PIT.pout = PID_PIT.P * err_pit;
	PID_PIT.dout = -PID_PIT.D * MPU6050_GYRO_LAST.Y;
	PID_PIT.iout = PID_PIT.I *pit_i;	
	////////////////////////////////////////////////////
	
	//*******************YAW的计算*********************//
	if(rc_yaw<1400||rc_yaw>1600)
	{
		bbuf = MPU6050_GYRO_LAST.Z + (rc_yaw-1500)*2;
	}
	yaw_p+=MPU6050_GYRO_LAST.Z*0.0609756f*0.002f;
	if(yaw_p>20)
		yaw_p = 20;
	if(yaw_p<-20)
		yaw_p=-20;
	
	PID_YAW.pout = PID_YAW.P*yaw_p;
	PID_YAW.dout = PID_YAW.dout*MPU6050_GYRO_LAST.Z;
	////////////////////////////////////////////////////
	if(rc_thr<START_THR)
	{
		pit_i = 0;
		rol_i = 0;
		yaw_p = 0;
	}
	
	PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;
	PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
	PID_YAW.OUT = PID_YAW.pout /*+ PID_YAW.iout*/ + PID_YAW.dout;
	
	if(rc_thr>START_THR&&ARMED)
	{
		//moto为0~1000的值对应占空比0~100
		moto1 = rc_thr -1000 + PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT;
		moto2 = rc_thr -1000 - PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT;
		moto3 = rc_thr -1000 - PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT;
		moto4 = rc_thr -1000 + PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT;
		
	}
	else
	{
		moto1 = 0;
		moto2 = 0;
		moto3 = 0;
		moto4 = 0;
	}
	Motor_Out(moto1,moto2,moto3,moto4);
	
}
