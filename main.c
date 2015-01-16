/*
gcc main.c -o main -lpthread -lm -Wall && ./main
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <pthread.h>

#define I2C_PATH "/dev/i2c-3"
#define PWM_PATH "/sys/class/pwm/ehrpwm.%i:%i/duty_ns"
#define KBD_PATH "/dev/input/event0"
#define PWM0 1<<0
#define PWM1 1<<1
#define PWM2 1<<2
#define PWM3 1<<3

int i2c,keyboard,pwm_fd[4];//handle
int duties[4];
int alive=1;
pthread_t kbd_th,i2c_th,pwm_th;

#include "i2c.c"
#include "pwm.c"
#include "kbd.c"

/* MAIN */
void kill_it(int signo){
	printf("stop by signal (%i)\n",signo);
	alive=0;
}
void all_stop(){
	pwm_stop(PWM0|PWM1|PWM2|PWM3);
	i2c_stop();
	kbd_stop();
}
void all_start(){
	kbd_start();
	i2c_start();
	pwm_start(PWM0|PWM1|PWM2|PWM3);
	signal(SIGINT ,kill_it);//CTRL+C
	signal(SIGSTOP,kill_it);//CTRL+Z
}
int main(){
	all_start();
	while(alive){
		//equilibrium equation
		//penche au fond  0 = ax = +
		//penche devant   3 = ax = -
		//penche a droite 1 = ay = +
		//penche a gauche 2 = ay = -
		pwm_set(PWM0,duties[0]);
		pwm_set(PWM1,duties[1]);
		pwm_set(PWM2,duties[2]);
		pwm_set(PWM3,duties[3]);
		//don't rush it
		usleep(10*1000);//sleep 10 ms
	}
	all_stop();
	return 0;
}
