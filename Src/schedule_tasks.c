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
#include "maths.h"
#include "stmflash.h"

extern ADC_HandleTypeDef hadc1;

battery_t battery;

union _dat16{
	uint16_t full;
	uint8_t  byte[2];
}d_temp;


void taskUsartDebug(void)
{

	ANO_DT_Data_Exchange();

}

void taskUpdateAttiAngle(void)
{
	static uint32_t t0,t1;
	static uint32_t old;

	old = t0;
	t0 = micros();

	CtrlAttiAng();

	t1 = micros();

	//外环计算时间 [first]/2+1 = 6
	d_temp.full = (uint16_t)(t1 - t0) * 100;
	str0[10] = d_temp.byte[1];
	str0[11] = d_temp.byte[0];

	//外环周期 [first]/2+1 = 7
	d_temp.full = (uint16_t)(t0 - old);
	str0[12] = d_temp.byte[1];
	str0[13] = d_temp.byte[0];
}

void taskUpdateMAG(void)
{
	static uint32_t t0,old;

	old = t0;
	t0 = micros();

	//mag更新周期  [first]/2+1 = 12
	d_temp.full = (uint16_t)((t0 - old) / 10);
	str0[22] = d_temp.byte[1];
	str0[23] = d_temp.byte[0];

	mag.read(imu.magADC);

	if(imu.magCaliPass == 0){
		led.model = ON;
		mag_calibration();
	}

	imu.magRaw[X] = (imu.magADC[X] - imu.magOffset[X]);
	imu.magRaw[Y] = (imu.magADC[Y] - imu.magOffset[Y]);
	imu.magRaw[Z] = (imu.magADC[Z] - imu.magOffset[Z]);
}

attitude_t tar_attitude;

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

	CtrlAttiRate();

#if _debug_taskUpdateAttitude
	t3 = micros();
#endif

	CtrlMotor();

#if _debug_taskUpdateAttitude
	t4 = micros();
#endif

#if _debug_taskUpdateAttitude

	t5 = micros();

	//imu数据采集和姿态计算 [first]/2+1 = 1
	d_temp.full = (uint16_t)(t2 - t1);
	str0[0] = d_temp.byte[1];
	str0[1] = d_temp.byte[0];

	//内环pid控制 [first]/2+1 = 2
	d_temp.full = (uint16_t)(t3 - t2)*100;
	str0[2] = d_temp.byte[1];
	str0[3] = d_temp.byte[0];

	//输出到电机 [first]/2+1 = 3
	d_temp.full = (uint16_t)(t4 - t3)*100;
	str0[4] = d_temp.byte[1];
	str0[5] = d_temp.byte[0];

	//整个任务计算时间 [first]/2+1 = 4
	d_temp.full = (uint16_t)(t5 - t1);
	str0[6] = d_temp.byte[1];
	str0[7] = d_temp.byte[0];

	//两次任务执行间隔 [first]/2+1 = 5
	d_temp.full = (uint16_t)(t1 - old);
	str0[8] = d_temp.byte[1];
	str0[9] = d_temp.byte[0];
#endif
}

void taskUpdateBaro(void)
{
	static uint32_t t0,old;

	old = t0;
	t0 = micros();

	//Baro更新周期 [first]/2+1 = 13
	d_temp.full = (uint16_t)((t0 - old) / 10);
	str0[24] = d_temp.byte[1];
	str0[25] = d_temp.byte[0];

	MS5611_ThreadNew();


}
void taskUpdateRC(void)
{
	static uint32_t t0,t1;
	static uint32_t old;
	static rc_t oldrc;
	static uint16_t lost_rc_time = 0;
	static uint8_t push_key_1000 = 0,push_key_2000 = 1;
	static uint16_t push_key_down_time = 0;

	old = t0;
	t0 = micros();

	if(rc_matched == 0){
		EE_SAVE_RC_ADDR_AND_MATCHED();
		rc_match();
		EE_SAVE_RC_ADDR_AND_MATCHED();
		//led.model = DOUBLE_FLASH;
		return ;
	}

	if(NRF24L01_RxPacket((u8*)rc.value) !=0){
		lost_rc_time ++;
		if(lost_rc_time > 100){
			rc.value[rc_aux2_num] = 1000;
			motorLock = 1;
		}
		return ;
	}else {
		lost_rc_time = 0;
	}


	if(rc.value[rc_check_pin1] == '@' &&
			rc.value[rc_check_pin2] == '#'){

		rc.value[rc_thr_num] = constrain(rc.value[rc_thr_num] + 0,1000,2000);
		rc.value[rc_yaw_num] = constrain(rc.value[rc_yaw_num] + 0,1000,2000);
		rc.value[rc_pit_num] = constrain(rc.value[rc_pit_num] + 0,1000,2000);
		rc.value[rc_rol_num] = constrain(rc.value[rc_rol_num] + 0,1000,2000);

		rc.thr = rc.value[rc_thr_num]-1000;
		rc.yaw = YAW_RATE_MAX * dbScaleLinear((rc.value[rc_yaw_num] - 1500),500,APP_YAW_DB);
		rc.pit = -Angle_Max * dbScaleLinear((rc.value[rc_pit_num] - 1500),500,APP_PR_DB);
		rc.rol = -Angle_Max * dbScaleLinear((rc.value[rc_rol_num] - 1500),500,APP_PR_DB);

		if(push_key_2000 && (rc.value[rc_push_num] == 1000)){
			push_key_1000 = 1;
			push_key_2000 = 0;
		}else if(push_key_1000 && (rc.value[rc_push_num] == 1000)){
			push_key_down_time++;
			if(push_key_down_time > 50){//RC_TASK任务周期是10ms
				rc_matched = 0;
			}
		}else if(push_key_1000 && (rc.value[rc_push_num] == 2000)){
			push_key_1000 = 0;
			push_key_2000 = 1;
			if(push_key_down_time > 50){//RC_TASK任务周期是10ms

			}else{
				imu.caliPass = 0;
			}
			push_key_down_time = 0;
		}



		if(rc.value[rc_aux2_num] == 2000){
			motorLock = 0;
		}else{
			motorLock = 1;
		}

		if(rc.value[rc_aux1_num] == 2000){
			imu.magCaliPass = 0;
		}
	}
	t1 = micros();
	//遥控计算时间 [first]/2+1 = 8
	d_temp.full = (uint16_t)(t1 - t0)*100;
	str0[14] = d_temp.byte[1];
	str0[15] = d_temp.byte[0];

	//遥控更新周期 [first]/2+1 = 9
	d_temp.full = (uint16_t)(t0 - old)/10;
	str0[16] = d_temp.byte[1];
	str0[17] = d_temp.byte[0];

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
	case FAST_FLASH:
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
