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
#include "led.h"

uint32_t baroPressureSumtt = 0;
extern int32_t baroPressure;
extern int32_t baroTemperature;
extern baro_t baro;
int16_t mag_data[3] = {0};
uint8_t ms5611_buf[3];
extern mag_t mag;
extern u8 mpu6050_buffer[14];					//iic读取后存放数据


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



union _dat{
	uint16_t full;
	uint8_t  byte[2];
}d_temp;

void taskUpdateAttitude(void)
{
#define _debug_taskUpdateAttitude 1

	static uint32_t t0,t1,t2,t3,t4,t5,t6;
	static uint32_t old;
	old = t1;

#if _debug_taskUpdateAttitude
	t1 = micros();
#endif

	IMUSO3Thread();

	if(imu.caliPass == 0){
		led.model = ON;
		if(IMU_Calibrate()){
			//gParamsSaveEEPROMRequset=1;	//请求记录到EEPROM
			imu.caliPass=1;
			led.model = DOUBLE_FLASH;
		}
	}
#if _debug_taskUpdateAttitude
	t2 = micros();
#endif



#if _debug_taskUpdateAttitude
	t3 = micros();
#endif

	tar_attitude.pitch = (rc.value[rc_pit_num] - 1500) / 25.0;
	tar_attitude.roll  = (rc.value[rc_rol_num] - 1500) / 25.0 ;
	tar_attitude.yaw   = (rc.value[rc_yaw_num] - 1500) / 25.0;

#if _debug_taskUpdateAttitude
	t4 = micros();
#endif

	CONTROL(imu.roll  ,
			imu.pitch ,
			imu.yaw   ,
			-tar_attitude.roll  ,
			-tar_attitude.pitch,
			-tar_attitude.yaw);

#if _debug_taskUpdateAttitude

	t5 = micros();

	//imu数据采集时间
	d_temp.full = (uint16_t)(t2 - t1);
	str0[0] = d_temp.byte[1];
	str0[1] = d_temp.byte[0];

	//四元数姿态融合
	d_temp.full = (uint16_t)(t5 - t1);
	str0[2] = d_temp.byte[1];
	str0[3] = d_temp.byte[0];

	//两次任务执行间隔
	d_temp.full = (uint16_t)(t1 - old);
	str0[4] = d_temp.byte[1];
	str0[5] = d_temp.byte[0];


#endif
}

void taskPIDLoop(void)
{

}

void taskUpdateRC(void)
{
	NRF24L01_RxPacket((u8*)rc.value);

	if(rc.value[rc_push_num] == 1000){
		imu.caliPass = 0;
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

void taskLED(void)
{

	switch(led.model){
	case ON:
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_RESET);
		break;
	case OFF:
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_SET);
		break;
	case SINGLE_FLASH:
		break;
	case DOUBLE_FLASH:
		double_flash();
		break;
	case SINGLE_FLASH_500MS:
		single_flash_500ms();
		break;
	default:;
	}

}
