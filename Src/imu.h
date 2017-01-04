/*
 * imu.h
 *
 *  Created on: 2016Äê12ÔÂ29ÈÕ
 *      Author: 50430
 */

#pragma once

#define STATIC_UNIT_TESTED static
#include "stdint.h"
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

