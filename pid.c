#include <unistd.h>

#include "common.h"
#include "pid.h"

Pid_data pid_data = {0,0,0,0,0,0,0,0,0};
float Kp=0.63, Ki=0.0, Kd=0.0;

void pidInit(){
	pid_data.lastTime[0]= gettime();
	pid_data.lastTime[1]= gettime();
	pid_data.lastTime[2]= gettime();
}

float pidCompute(int axe, float value, float mesure){
	//compute and update dt
	unsigned long now = gettime();
	float dt = (((float)(pid_data.lastTime[axe] - now) / 1000000.0)/100000.0);
	pid_data.lastTime[axe] = now;
	//compute equations params
	float erreur = mesure - value;
	float derivee = (erreur - pid_data.lastErr[axe])/dt;
	float integral = (pid_data.integrale[axe] += ((erreur + pid_data.lastErr[axe] ) / 2) * dt);
	
	pid_data.lastErr[axe] = erreur;
	//printf("%f\n", dt);
	//printf("%f %f %f\n", erreur, pid_data.integrale[axe], derivee);
	return (Kp * erreur) + (Ki * pid_data.integrale[axe]) + (Kd * derivee);
}
