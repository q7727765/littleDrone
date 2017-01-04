/*
 * sensor.c
 *
 *  Created on: 2017Äê1ÔÂ3ÈÕ
 *      Author: 50430
 */



#pragma once

#include "stdint.h"
#include "stdbool.h"

typedef void (*baroOpFuncPtr)(void);                       // baro start operation
typedef void (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature); // baro calculation (filled params are pressure and temperature)

typedef void (*sensorInitFuncPtr)(void);                    // sensor init prototype
typedef bool (*sensorReadFuncPtr)(int16_t *data);           // sensor read prototype

struct acc_s;
typedef void (*sensorAccInitFuncPtr)(void);                    // sensor init prototype
typedef void (*sensorGyroInitFuncPtr)(void);         // gyro sensor init prototype
typedef bool (*sensorIsDataReadyFuncPtr)(void);             // sensor data ready prototype

