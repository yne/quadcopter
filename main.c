#include <stdio.h>
#include <signal.h>

#include "common.h"
#include "mpu.h"
#include "pwm.h"
#include "pid.h"
#include "keyboard.h"

int alive=1;

#ifdef BENCH
#include <sys/time.h>
#include <linux/input.h>
#include "MPU6050.h"
#include "mpu.h"

extern int KBD;

void gettime_(){
	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);
}
void readKBD_(){
	struct input_event ie;
	read(KBD, &ie, sizeof(ie));
}
void setPWM_(){
	ctx.pwm1->cmpa=42;
}
void getMPU_(){
	union i2c_smbus_data d={block:{14}};
	struct i2c_smbus_ioctl_data  blk={I2C_SMBUS_READ,MPU6050_RA_ACCEL_XOUT_H,I2C_SMBUS_I2C_BLOCK_DATA,(union i2c_smbus_data*)&d};
	ioctl(ctx.i2c,I2C_SMBUS,&blk);
}
int bench(void (*cc)()){
	unsigned start = gettime();
	
	int i;for(i=0;i<1000;i++){
		cc();
	}
	
	return gettime()-start;
}
#endif

void killme(int signo){
#ifndef WCET
	printf("\n//Break (%s)\n",signo?"Ctrl":"Esc");
#endif
	alive=0;
}

int main(int argc,char**argv){
#ifndef WCET
	signal(SIGINT ,killme);//CTRL+C
	signal(SIGSTOP,killme);//CTRL+Z

	int i;
	for(i=1;i<argc;i++){
		switch(argv[i][0]){
			case 'p':Kp=0.01*atoi(argv[i]+2);break;
			case 'i':Ki=0.01*atoi(argv[i]+2);break;
			case 'd':Kd=0.01*atoi(argv[i]+2);break;
			default:;
		}
	}
	//Initialisation
	mpuInit();
	pidInit();
	pwmInit();
	kbdInit();
#ifndef BENCH
	printf("<html><body><div id=\"curve_chart\" style=\"width: 1920px; height: 1080px\"></div>"
"<script src=\"https://www.google.com/jsapi?autoload={'modules':[{'name':'visualization','version':'1','packages':['corechart']}]}\"></script>"
"<script>google.setOnLoadCallback(drawChart);function drawChart() {"
"(new google.visualization.LineChart(document.getElementById('curve_chart')))"
".draw(google.visualization.arrayToDataTable(["
"	['n', 'Roll', 'Pitch'],");
#endif
	for(i=0;alive;i++){
#endif
#ifdef BENCH
		printf("gettime:%u us\n",bench(gettime_));
		printf("readKBD:%u us\n",bench(readKBD_));
		printf("setPWM :%u us\n",bench(setPWM_));
		printf("getMPU :%u us\n",bench(getMPU_));
		alive=0;
#else
		kbdGet();
		mpuUpdate();
		float p_computed = pidCompute (PITCH, usr_pitch, getAX());
		float r_computed = pidCompute (ROLL , usr_roll , getAY());
		setSpeed(MOTOR_FL|MOTOR_FR|MOTOR_BL|MOTOR_BR, global_speed, pidToInt(r_computed) , pidToInt(p_computed));
#endif
#ifndef WCET
		//printf("['%i',%f,%f],\n",i, p_computed, r_computed);
	}
#ifndef BENCH
	printf("]), {curveType: 'function',legend: { position: 'bottom' }});}</script></body></html>");
#endif

	pwmStop();
	mpuStop();
	kbdStop();
#endif
	return 0;
}
