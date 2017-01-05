/*
 * HAL.c
 *
 *  Created on: 2017Äê1ÔÂ3ÈÕ
 *      Author: 50430
 */

#pragma once

#include "stdint.h"
#include "sensor.h"

typedef struct gyro_s {
    sensorGyroInitFuncPtr init;                             // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    sensorIsDataReadyFuncPtr isDataReady;                   // check if sensor has new readings
    float scale;                                            // scalefactor
} gyro_t;

typedef struct acc_s {
	sensorAccInitFuncPtr init;                             // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    uint16_t acc_1G;
    char revisionCode;                                      // a revision code for the sensor, if known
} acc_t;

typedef struct mag_s {
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
} mag_t;

typedef struct baro_s {
    uint16_t ut_delay;
    uint16_t up_delay;
    baroOpFuncPtr start_ut;
    baroOpFuncPtr get_ut;
    baroOpFuncPtr start_up;
    baroOpFuncPtr get_up;
    baroCalculateFuncPtr calculate;


} baro_t;

extern acc_t acc;
extern gyro_t gyro;
extern mag_t mag;
extern baro_t baro;


extern void detectAcc();
extern void detectGyro();
extern void detectMag();
extern void detectBaro();
extern uint32_t recalculateBarometerTotal(uint8_t baroSampleCount, uint32_t pressureTotal, int32_t newPressureReading);
