#ifndef _COMMON_H
#define _COMMON_H

#include "mpu.h"
#include "pwm.h"

#define SPEED_0 2500
#define SPEED_MIN 4000
#define SPEED_MAX 10000

typedef struct{
	PWM_map*pwm1,*pwm2;
	GPIO_map*gpio;
	int i2c;
}Quad_ctx;

unsigned long gettime();
int pidToInt(float pid);

/////////// GLOBAL ////////////
Quad_ctx ctx;

float usr_pitch,usr_roll;
unsigned global_speed;

#endif
