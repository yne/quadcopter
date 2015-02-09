#ifndef _PID_H
#define _PID_H

#define DT 10000

#define PITCH 0
#define ROLL 1
#define YAW 2

typedef struct {
	unsigned long lastTime[3];
	float lastErr[3];
	float integrale[3];
}Pid_data;

float pidCompute(int axe, float value, float mesure);
void pidInit();
float Kp,Ki,Kd;

#endif
