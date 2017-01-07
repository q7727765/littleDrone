/*
 * HAL_motor.c
 *
 *  Created on: 2017Äê1ÔÂ7ÈÕ
 *      Author: 50430
 */

#include "HAL.h"
#include "stm32f103xb.h"

motor_t motor;

void motor_init()
{
	motor.value[0] = &(TIM2->CCR1);
	motor.value[1] = &(TIM2->CCR2);
	motor.value[2] = &(TIM2->CCR3);
	motor.value[3] = &(TIM2->CCR4);
}

void motor_out(u16 m1,u16 m2,u16 m3,u16 m4)
{
	if(m1<0|m1>1000) return;
	if(m2<0|m2>1000) return;
	if(m3<0|m3>1000) return;
	if(m4<0|m4>1000) return;


	*(motor.value[0]) = m1;
	*(motor.value[1]) = m2;
	*(motor.value[2]) = m3;
	*(motor.value[3]) = m4;
}
