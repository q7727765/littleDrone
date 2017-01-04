/*
 * HAL_gyro.c
 *
 *  Created on: 2017Äê1ÔÂ4ÈÕ
 *      Author: 50430
 */

#include "HAL.h"
#include "usart.h"
#include "mpu6050.h"
#include "delay.h"

gyro_t gyro;

void detectGyro()
{
	while(!	MPU6050DetectGyro(&gyro)) {
		delay_ms(500);
		SendChar("Initing gyro\r\n");
		_n();
	}
	SendChar("gyro OK\r\n");
	_n();
}
