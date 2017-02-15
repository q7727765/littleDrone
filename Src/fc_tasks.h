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

typedef enum {
    /* Actual tasks */
    TASK_SYSTEM = 0,

    TASK_UPDATE_ATT_ANGLE,

	TASK_UPDATE_ATTITUDE,

	TASK_UPDATE_MAG,

	TASK_PID_LOOP,

	TASK_UPDATE_RC,

	TASK_BATTERY_MONITOR,

	TASK_USART_DEBUG,

	TASK_LED,

    TASK_BATTERY,

    TASK_RX,

    TASK_GPS,

    TASK_COMPASS,

    TASK_BARO,

    /* Count of real tasks */
    TASK_COUNT
} cfTaskId_e;


void taskSystem(void);
void taskUpdateAttiAngle(void);
void taskLED(void);
void configureScheduler(void);
void taskUpdateAttitude(void);
void taskUsartDebug(void);
void taskUpdateMAG(void);
void taskPIDLoop(void);
void taskUpdateRC(void);
void taskBatteryMoniter(void);

