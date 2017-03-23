/*
 * HAL_accgyro.c
 *
 *  Created on: 2017Äê1ÔÂ3ÈÕ
 *      Author: 50430
 */


#include "HAL.h"
#include "mpu6050.h"
#include "delay.h"
#include "usart.h"

acc_t acc;


void detectAcc()
{
	while(!	MPU6050DetectAcc(&acc)) {
		delay_ms(500);
		SendChar("Initing acc\r\n");
		_n();
	}
	SendChar("acc OK\r\n");
	_n();
}

