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

#include <stdbool.h>
#include <stdint.h>

#include "fc_tasks.h"

#include "scheduler/scheduler.h"

#include "imu.h"

// No need for a linked list for the queue, since items are only inserted at startup
#ifdef UNIT_TEST
#define TASK_QUEUE_ARRAY_SIZE (TASK_COUNT + 2) // 1 extra space so test code can check for buffer overruns
#else
#define TASK_QUEUE_ARRAY_SIZE (TASK_COUNT + 1) // extra item for NULL pointer at end of queue
#endif

const uint32_t taskQueueArraySize = TASK_QUEUE_ARRAY_SIZE;
const uint32_t taskCount = TASK_COUNT;
cfTask_t* taskQueueArray[TASK_QUEUE_ARRAY_SIZE];

//周期单位：us
cfTask_t cfTasks[] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = 1000000 / 10,          // 10 Hz, every 100 ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    [TASK_UPDATE_MPU6050] = {
        .taskName = "UPDATE_MPU6050",
        .taskFunc = taskUpdateMPU6050,
        .desiredPeriod = 1000,
        .staticPriority = TASK_PRIORITY_REALTIME,
    },

    [TASK_UPDATE_ATTITUDE] = {
        .taskName = "UPDATE_ATTITUDE",
        .taskFunc = Get_Attitude,
        .desiredPeriod = 1000,
        .staticPriority = TASK_PRIORITY_REALTIME_PRO,
    },

    [TASK_RUNNLED] = {
        .taskName = "RUNLED",
        .taskFunc = taskRUNLED,
        .desiredPeriod = 500000,
        .staticPriority = TASK_PRIORITY_LOW,
    },

    [TASK_USART_DEBUG] = {
        .taskName = "USART_DEBUG",
        .taskFunc = taskUsartDebug,
        .desiredPeriod = 1000,
        .staticPriority = TASK_PRIORITY_HIGH,
    },


};

void configureScheduler(void)
{
    schedulerInit();
    setTaskEnabled(TASK_SYSTEM, 1);
    setTaskEnabled(TASK_UPDATE_MPU6050, 1);
    setTaskEnabled(TASK_RUNNLED, 1);
    setTaskEnabled(TASK_UPDATE_ATTITUDE, 1);
    setTaskEnabled(TASK_USART_DEBUG, 1);
}
