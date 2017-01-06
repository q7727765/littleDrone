#pragma once

#include "stm32f103xb.h"

#define ARMED_LED PBout(3)
typedef struct PID{float P,pout,I,iout,D,dout,IMAX,OUT;}PID;

extern u8 ARMED;
extern PID PID_ROL,PID_PIT,PID_YAW;

void CONTROL(float rol_now, float pit_now, float yaw_now, float rol_tar, float pit_tar, float yaw_tar);




