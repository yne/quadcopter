#ifndef _COMMON_H
#define _COMMON_H

#include "mpu.h"
#include "pwm.h"

typedef struct{
	PWM_map*pwm1,*pwm2;
	GPIO_map*gpio;
	int i2c;
}Quad_ctx;

unsigned long gettime();
int pidToInt(float pid);

/////////// GLOBAL ////////////
Quad_ctx ctx;


#endif
