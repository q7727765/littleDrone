/*
 * hmc5883l.h
 *
 *  Created on: 2016Äê12ÔÂ29ÈÕ
 *      Author: 50430
 */


/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "stdbool.h"
#include "HAL.h"

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03


#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)    // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)    // High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2


typedef void (*sensorInitFuncPtr)(void);                    // sensor init prototype
typedef bool (*sensorReadFuncPtr)(int16_t *data);           // sensor read prototype


typedef struct hmc5883Config_s {

    uint32_t gpioAPB2Peripherals;

    uint16_t gpioPin;
    GPIO_TypeDef *gpioPort;

    uint8_t exti_port_source;
    uint32_t exti_line;
    uint8_t exti_pin_source;
    IRQn_Type exti_irqn;
} hmc5883Config_t;


bool hmc5883lDetect(mag_t* mag);
void hmc5883lInit(void);
bool hmc5883lRead(int16_t *magData);


