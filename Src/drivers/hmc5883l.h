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

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03


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

typedef struct mag_s {
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
} mag_t;

bool hmc5883lDetect(mag_t* mag, const hmc5883Config_t *hmc5883ConfigToUse);
void hmc5883lInit(void);
bool hmc5883lRead(int16_t *magData);


