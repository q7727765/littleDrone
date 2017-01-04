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
