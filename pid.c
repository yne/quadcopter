#include <unistd.h>
//#include <stdio.h>

#include "common.h"
#include "pid.h"



typedef struct {
	unsigned long lastTime[3];
	float lastErr[3];
	float integrale[3];
}Pid_data;
Pid_data pid_data = {0,0,0,0,0,0,0,0,0};

void pidInit(){
	pid_data.lastTime[0]= gettime();
	pid_data.lastTime[1]= gettime();
	pid_data.lastTime[2]= gettime();
}

float pidCompute(int axe, float value, float mesure){

	float Kp=1.9, Ki=0.0, Kd=0.0;

	float pid;
	unsigned long now = gettime();
	float dt = (((float)(pid_data.lastTime[axe] - now) / 1000000.0)/100000.0);
	float erreur = mesure - value;
	float derivee = (erreur - pid_data.lastErr[axe])/(float)dt;
	pid_data.integrale[axe] += ((erreur + pid_data.lastErr[axe] ) / 2) * dt;
	//printf("%f\n", dt);
	//printf("%f %f %f\n", erreur, pid_data.integrale[axe], derivee);
	pid = (Kp * erreur) + (Ki * pid_data.integrale[axe]) + (Kd * derivee);

	pid_data.lastErr[axe] = erreur;
	pid_data.lastTime[axe] = now;

	return pid;
}
