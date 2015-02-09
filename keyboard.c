#include <linux/input.h>
#include <stdio.h>
#include <fcntl.h>
#include "common.h"
#include "pid.h"

int KBD;
void kbdInit(){//char*path){
	char path[] = "/dev/input/event0";
	if((KBD = open(path, O_RDONLY|O_NONBLOCK))<0)
		perror(__FUNCTION__);
}

void kbdGet(){
	if(KBD<0)return;//not connected
	struct input_event ie;
	if(read(KBD, &ie, sizeof(ie))<=0)return;//nothing to read
	if((ie.type==EV_MSC) || !ie.code || (ie.value==2))return;
	if(ie.value==1){//keypress
		switch(ie.code){
		case KEY_ESC     :killme(0);break;
		case KEY_PAGEUP  :global_speed+=250;break;
		case KEY_PAGEDOWN:global_speed-=250;break;
		//lower a PWM using directionals arrows
		case KEY_UP      :usr_pitch= .1;break;
		case KEY_RIGHT   :usr_roll = .1;break;
		case KEY_DOWN    :usr_pitch=-.1;break;
		case KEY_LEFT    :usr_roll =-.1;break;
		//coef
		case KEY_A      :Kp+=.1;break;
		case KEY_Z      :Ki+=.1;break;
		case KEY_E      :Kd+=.1;break;
		case KEY_Q      :Kp-=.1;break;
		case KEY_S      :Ki-=.1;break;
		case KEY_D      :Kd-=.1;break;
		default:fprintf(stderr,"unhandled code:%i\n",ie.code);
		}
		printf("speed:%i Kp:%f Ki:%f Kd:%F\n",global_speed,Kp,Ki,Kd);
	}
	if(ie.value==0){//keyrelease = set back to original value
		switch(ie.code){
		case KEY_UP      :usr_pitch=.0;break;
		case KEY_RIGHT   :usr_roll =.0;break;
		case KEY_DOWN    :usr_pitch=.0;break;
		case KEY_LEFT    :usr_roll =.0;break;
		default:;
		}
	}
}
void kbdStop(){
	close(KBD);
}