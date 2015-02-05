#ifndef _PID_H
#define _PID_H

#define DT 10000

#define PITCH 0
#define ROLL 1
#define YAW 2

float pidCompute(int axe, float value, float mesure);
void pidInit();

#endif
