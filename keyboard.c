#include <linux/input.h>
#include <stdio.h>
#include <fcntl.h>

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
		case KEY_PAGEUP  :break;
		case KEY_PAGEDOWN:break;
		//lower a PWM using directionals arrows
		case KEY_UP      :break;
		case KEY_RIGHT   :break;
		case KEY_DOWN    :break;
		case KEY_LEFT    :break;
		default:fprintf(stderr,"unhandled code:%i\n",ie.code);
		}
	}
	if(ie.value==0){//keyrelease = set back to original value
		switch(ie.code){
		case KEY_UP      :break;
		case KEY_RIGHT   :break;
		case KEY_DOWN    :break;
		case KEY_LEFT    :break;
		default:;
		}
	}
}
void kbdStop(){
	close(KBD);
}