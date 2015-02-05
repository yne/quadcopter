#ifndef _TYPES_H
#define _TYPES_H


#include "mpu.h"
#include "pwm.h"

typedef struct{
	PWM_map*pwm1,*pwm2;
	GPIO_map*gpio;
	int i2c;
	//MPU_Data_RAW avg;
}Quad_ctx;




/////////// GLOBAL ////////////
Quad_ctx ctx;


#endif
