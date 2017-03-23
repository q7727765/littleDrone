#pragma once

#include <math.h>
#include <stdint.h>


#define IMU_SAMPLE_RATE 			500.0f	//imu采样率（HZ）//1000.0f/(float)DMP_CALC_PRD
#define IMU_FILTER_CUTOFF_FREQ	30.0f

//校准时间
#define ACC_CALC_TIME  2000//ms
#define GYRO_CALC_TIME   2000000l	//us

typedef float  quad[4];
typedef float  vector3f[3];	//不可作为返回值，指针
typedef float  matrix3f[3][3];

typedef struct float_xyz
{
    float X;
    float Y;
    float Z;

}S_FLOAT_XYZ;

typedef struct mat3_tt
{
float m[3][3];
}mat3;

typedef struct vec3_tt
{
float v[3];
}vec3;


enum{ROLL,PITCH,YAW,THROTTLE};
enum{X,Y,Z};

typedef struct IMU_tt
{
uint8_t caliPass;
uint8_t magCaliPass;
uint8_t ready;
int16_t accADC[3];
int16_t gyroADC[3];
int16_t magADC[3];
float 	accRaw[3];		//m/s^2
float 	gyroRaw[3];		//rad/s
int16_t	magRaw[3];		//
float   accOffset[3];		//m/s^2
float   gyroOffset[3];
int16_t	magOffset[3];
float   accb[3];		//filted, in body frame
float   accg[3];
float   gyro[3];
float   DCMgb[3][3];
float   q[4];
float   roll;				//deg
float   pitch;
float 	yaw;
float   rollRad;				//rad
float   pitchRad;
float 	yawRad;
}imu_t;

//enum{ROLL,PITCH,YAW};
#define M_PI_F 3.1415926
#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* 癈			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/

//extern volatile float accFilted[3],gyroFilted[3];
//extern float DCMbg[3][3],DCMgb[3][3];
//extern float accZoffsetTemp;
//extern float IMU_Pitch,IMU_Roll,IMU_Yaw;
extern imu_t imu;
extern uint8_t imuCaliFlag;


/* Function prototypes */
static float invSqrt(float number);
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);

void IMUSO3Thread(void);
uint8_t IMU_Calibrate(void);
