/*
 * schedule_tasks.c
 *
 *  Created on: 2016年12月28日
 *      Author: 50430
 */

#include "stm32f1xx_hal.h"
#include "delay.h"
#include "usart.h"
#include "hmc5883l.h"
#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include "mpu6050.h"
#include "ms5611.h"
#include "HAL.h"
#include "imu.h"
#include "ANO_DT.h"
#include "control.h"
#include "nrf24l01.h"

uint32_t baroPressureSumtt = 0;
extern int32_t baroPressure;
extern int32_t baroTemperature;
extern baro_t baro;
int16_t mag_data[3] = {0};
uint8_t ms5611_buf[3];
extern mag_t mag;
extern u8 mpu6050_buffer[14];					//iic读取后存放数据

extern S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//最新一次读取值
extern S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//零漂
extern u8							GYRO_OFFSET_OK;
extern u8							ACC_OFFSET_OK;

extern ADC_HandleTypeDef hadc1;
battery_t battery;

void taskUsartDebug(void)
{

	ANO_DT_Data_Exchange();

}
void taskUpdateMPU6050(void)
{



}

void taskUpdateMAG(void)
{
	mag.read(mag_data);

}
attitude_t tar_attitude;

void taskUpdateAttitude(void)
{
	Prepare_Data();//imu数据获取
	Get_Attitude();//姿态解算

	tar_attitude.pitch = (rc.value[rc_pit_num] - 1500) / 12;
	tar_attitude.roll  = (rc.value[rc_rol_num] - 1500) / 12;
	tar_attitude.yaw   = (rc.value[rc_yaw_num] - 1500) / 12;

	CONTROL(now_attitude.roll  ,
			now_attitude.pitch ,
			now_attitude.yaw   ,
			-tar_attitude.roll  ,
			-tar_attitude.pitch,
			-tar_attitude.yaw);
}

void taskPIDLoop(void)
{

}

void taskUpdateRC(void)
{
	NRF24L01_RxPacket((u8*)rc.value);

	if(rc.value[rc_push_num] == 1000){
		ACC_OFFSET_OK  = 0;
		GYRO_OFFSET_OK = 0;
	}

	if(rc.value[rc_aux2_num] == 2000){
		ARMED = 1;
	}else{
		ARMED = 0;
	}

}

void taskBatteryMoniter(void)
{
	battery.scale = 2.298;
	battery.raw_data = (uint16_t)HAL_ADC_GetValue(&hadc1);
	battery.voltage = ((float)battery.raw_data/4096) * battery.scale * 333;
}

void taskRUNLED(void)
{
	static char sta = 0;

	sta = !sta;//(sta+1)%2;

	if(sta)
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_RESET);

}
