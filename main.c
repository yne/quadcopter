#include <stdio.h>

#include "types.h"
#include "mpu.h"
#include "pwm.h"
#include "pid.h"

void print(){
	printPID(0);
	printPID(1);
	printPID(2);
}
int main(){
	printf("Quadcopter, en avant toute !\n");
	int i;
	pidInit();
	mpuInit();
	
	//MPU_Data g;
	for(i = 0; i < 150; ){
		//g=mpuGet();
		mpuUpdate();
		getAX();
		getAY();
		//printf("X \t PID : %f\n", pid(PITCH, 0.0, getAX()));
		//printf("Y \t PID : %f\n", i, pid(ROLL , 0.0, g.AccY));
		
		//usleep(DT);
	}
	
	
	
	
	
	
	
	
	/*
	
	
	
	for(i = 0; i < 150; i ++){
		printf("%d \t PID : %f\n", i, pid(0, 0.0, 1.0));
	}*/
	return 0;
}
