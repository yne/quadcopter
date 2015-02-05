#include <stdio.h>
#include <signal.h>

#include "common.h"
#include "mpu.h"
#include "pwm.h"
#include "pid.h"
#include "keyboard.h"

void killme(int signo){
	printf("\nBreak (%s)\n",signo?"Ctrl":"Esc");
	pwmStop();
	mpuStop();
	kbdStop();
}

int main(){
	printf("Quadcopter, en avant toute !\n");
	int i;
	float r_computed, p_computed; //PID ROLL & PID PITCH
	
	signal(SIGINT ,killme);//CTRL+C
	signal(SIGSTOP,killme);//CTRL+Z

	
	
	pidInit();
	mpuInit();
	pwmInit();
	kbdInit();
	
	while(1){
		mpuUpdate();
		p_computed = pidCompute (PITCH, 0.0, getAX());
		r_computed = pidCompute (ROLL, 0.0, getAY());
		printf("p : %f\tr : %f\n", p_computed, r_computed);
		setSpeed(MOTOR_FL|MOTOR_FR|MOTOR_BL|MOTOR_BR, 4000, pidToInt(r_computed) , pidToInt(p_computed));
		usleep(10000);
		//printf("Hello\n");
	}

	
	/***********************************************************
	FIXER PWM
	MODIFIER LES PWM EN FONCTION DES PID
	CALIBRER LE GYRO AU DEMARRAGE
	TESTER AVEC LES HELICES POUR MODIFIER NOS Kp Ki Kd
	***********************************************************/
	
	pwmStop();
	mpuStop();
	kbdStop();
	return 0;
}
