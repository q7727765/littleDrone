/*
 * signalLamp.c
 *
 *  Created on: 2017Äê2ÔÂ8ÈÕ
 *      Author: 50430
 */

#include "led.h"
#include "HAL.h"

#include "stm32f1xx_hal.h"

led_t led = {
		.state = 1,
		.model = SINGLE_FLASH_500MS
};

void single_flash_500ms(void)
{
	static uint8_t count = 0;

	count++;
	if(count>10)count = 0;

	if(count<5)
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_RESET);

}

void double_flash(void)
{
	static uint8_t count = 0;

	count++;

	if(count < 2)
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_SET);
	else if(count < 4)
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_RESET);
	else if(count < 6)
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_SET);
	else if(count < 8)
		HAL_GPIO_WritePin(GPIOB, LED_SIGN_Pin , GPIO_PIN_RESET);
	else if(count > 8){
		count = 0;
		led.model = SINGLE_FLASH_500MS;
	}

}
