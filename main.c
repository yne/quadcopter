#include <stdio.h>
#include <signal.h>

#include "common.h"
#include "mpu.h"
#include "pwm.h"
#include "pid.h"
#include "keyboard.h"

int alive=1;

void killme(int signo){
	printf("\n//Break (%s)\n",signo?"Ctrl":"Esc");
	alive=0;
}

int main(int argc,char**argv){
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
	printf("<html><body><div id=\"curve_chart\" style=\"width: 1920px; height: 1080px\"></div>"
"<script src=\"https://www.google.com/jsapi?autoload={'modules':[{'name':'visualization','version':'1','packages':['corechart']}]}\"></script>"
"<script>google.setOnLoadCallback(drawChart);function drawChart() {"
"(new google.visualization.LineChart(document.getElementById('curve_chart')))"
".draw(google.visualization.arrayToDataTable(["
"	['n', 'Roll', 'Pitch'],");
	
	for(i=0;alive;i++){
		kbdGet();
		mpuUpdate();
		float p_computed = pidCompute (PITCH, usr_pitch, getAX());
		float r_computed = pidCompute (ROLL , usr_roll , getAY());
		printf("['%i',%f,%f],\n",i, p_computed, r_computed);
		setSpeed(MOTOR_FL|MOTOR_FR|MOTOR_BL|MOTOR_BR, global_speed, pidToInt(r_computed) , pidToInt(p_computed));
	}
	printf("]), {curveType: 'function',legend: { position: 'bottom' }});}</script></body></html>");

	pwmStop();
	mpuStop();
	kbdStop();
	return 0;
}
