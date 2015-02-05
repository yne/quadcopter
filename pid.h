#ifndef _PID_H
#define _PID_H

#define DT 10000

#define PITCH 1
#define ROLL 2
#define YAW 3

float pid(int axe, float value, float mesure);
void pidInit();

#endif
