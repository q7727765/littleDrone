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


void taskUsartDebug(void)
{

	ANO_DT_Data_Exchange();
//		uint8_t xl,xm;
//		int16_t x;
//
//		uint8_t sta;
//
//		static uint8_t mo = 1;
//
//		//acc & gyro
//
//		SendChar("X:");
//		SendInt(accdata[0]);
//		_n();
//		SendChar("Y:");
//		SendInt(accdata[1]);
//		_n();
//		SendChar("Z:");
//		SendInt(accdata[2]);
//		_n();
//		SendChar("GX:");
//		SendInt(gyrodata[0]);
//		_n();
//		SendChar("GY:");
//		SendInt(gyrodata[1]);
//		_n();
//		SendChar("GZ:");
//		SendInt(gyrodata[2]);
//		_n();
//
//		SendChar("ANGLE_X:");
//		SendDouble(Q_ANGLE.X);
//		_n();
//		SendChar("ANGLE_Y:");
//		SendDouble(Q_ANGLE.Y);
//		_n();
//		SendChar("ANGLE_Z:");
//		SendDouble(Q_ANGLE.Z);
//		_n();
//
//
//	//mag
//
//
//		sta = mag.read(mag_data);
//
//		SendChar("mag_x:");
//		SendInt(mag_data[0]);
//		_n();
//		SendChar("mag_y:");
//		SendInt(mag_data[2]);
//		_n();
//		SendChar("mag_z:");
//		SendInt(mag_data[1]);
//		_n();
//
//	//baro
//
//		if(mo){
//
//		    baro.get_ut();
//		    baro.start_up();
//
//		}else{
//
//		    baro.get_up();
//		    baro.start_ut();
//		    baro.calculate(&baroPressure, &baroTemperature);
//		    baroPressureSumtt =recalculateBarometerTotal(
//		    		48,
//		    		baroPressureSumtt,
//					baroPressure);
//
//		}
//
//		mo =! mo;
//
//	    SendChar("baroPressure:");
//	    SendInt(baroPressure);
//	    _n();
//	    SendChar("baroTemperature:");
//	    SendInt(baroTemperature);
//	    _n();
////	    SendChar("baroPressureSumtt:");
////	    SendInt(baroPressureSumtt);
////	    _n();
//
//	//	IIC_Write_Reg(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + CMD_ADC_4096, 1); // D1 (pressure) conversion start!
//	//	sta = IIC_Read_Reg_Len(MS5611_ADDR,CMD_ADC_READ,3,ms5611_buf);
//	//	baro = ((uint32_t)ms5611_buf[0] << 16) | (uint32_t)(ms5611_buf[1] << 8) | (uint32_t)ms5611_buf[2];
//	//	SendChar("baro:");
//	//	SendInt(baro);
//	//	_n();
}
void taskUpdateMPU6050(void)
{

	Prepare_Data();

}

void taskUpdateAttitude(void)
{
	Get_Attitude();
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
