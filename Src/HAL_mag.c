/*
 * HAL_mag.c
 *
 *  Created on: 2017Äê1ÔÂ3ÈÕ
 *      Author: 50430
 */


#include "HAL.h"
#include "stdbool.h"
#include "usart.h"
#include "hmc5883l.h"
#include "delay.h"
#include "stdint.h"
#include "imu.h"
#include "led.h"
#include "stmflash.h"

mag_t mag;

void detectMag()
{

	while(!hmc5883lDetect(&mag)) {
		delay_ms(500);
		SendChar("Initing HMC\r\n");
		_n();
	}
	SendChar("HMC OK\r\n");
	_n();

}

#define MAG_CAL_DAT_NUM 300

void mag_calibration(void)
{
	static int16_t magX_min,magX_max,magY_min,magY_max,magZ_min,magZ_max;
	static uint16_t count = 0;

	if(count == 0){
		magX_max = magX_min = imu.magADC[X];
		magY_max = magY_min = imu.magADC[Y];
		magZ_max = magZ_min = imu.magADC[Z];
		count++;
		return;
	}

	if(imu.magADC[X] < magX_min){
		magX_min = imu.magADC[X];
	}else if(imu.magADC[X] > magX_max){
		magX_max = imu.magADC[X];
	}

	if(imu.magADC[Y] < magY_min){
		magY_min = imu.magADC[Y];
	}else if(imu.magADC[Y] > magY_max){
		magY_max = imu.magADC[Y];
	}

	if(imu.magADC[Z] < magZ_min){
		magZ_min = imu.magADC[Z];
	}else if(imu.magADC[Z] > magZ_max){
		magZ_max = imu.magADC[Z];
	}

	count++;

//	if(count == MAG_CAL_DAT_NUM){

	if(rc.value[rc_aux1_num] == 1000){
		imu.magOffset[X] = (magX_max + magX_min) / 2;
		imu.magOffset[Y] = (magY_max + magY_min) / 2;
		imu.magOffset[Z] = (magZ_max + magZ_min) / 2;

		count = 0;

		led.model = SINGLE_FLASH_500MS;
		imu.magCaliPass = 1;
		EE_SAVE_MAG_OFFSET();
	}

}
