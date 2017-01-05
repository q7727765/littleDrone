/*
 * imu.h
 *
 *  Created on: 2016Äê12ÔÂ29ÈÕ
 *      Author: 50430
 */

#pragma once


#include "stdint.h"
#include "stdbool.h"

#define STATIC_UNIT_TESTED static

#define XYZ_AXIS_COUNT 3


typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;

extern attitudeEulerAngles_t attitude;
extern float gyroScale;

void imuInit(void);
void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz,
                                bool useYaw, float yawError);

void imuUpdateEulerAngles(void);

