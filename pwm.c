#include <assert.h>
//#include <stdio.h>
#include <stdlib.h>
//#include <stdint.h>
//#include <stdbool.h>
//#include <unistd.h>
#include <fcntl.h>
//#include <errno.h>
//#include <string.h>
//#include <linux/input.h>
//#include <sys/ioctl.h>
#include <sys/mman.h>
#include "pwm.h" //camsi.ups-tlse.fr/lib/exe/fetch.php?media=embedded:rm-mpu-6000a-00v4.2.pdf

#include "types.h"
/////////// GLOBAL ////////////


#define PWM1  ctx.pwm1
#define PWM2  ctx.pwm2
#define GPIO  ctx.gpio

void*mapGet(int start){
	int devmem_fd = open("/dev/mem", O_RDWR|O_SYNC);
	assert(devmem_fd>=0);
	void*map_addr=mmap(NULL,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,devmem_fd,start);
	close(devmem_fd);
	assert(map_addr!=MAP_FAILED);
	return map_addr;
}

///////////// P W M //////////
void pwmInit(){
	ctx.pwm1=mapGet(PWM_1_START);
	ctx.pwm2=mapGet(PWM_2_START);
	ctx.gpio=mapGet(GPIO_1_START);
}
void pwmStop(){
	munmap(&ctx.pwm1,getpagesize());
	munmap(&ctx.pwm2,getpagesize());
	munmap(&ctx.gpio,getpagesize());
}
/*
int main(){
	setvbuf(stdout, NULL, _IONBF, 0);//debufferise stdout (pour les \r via putty)
	mpuInit();
	pwmInit();
	#define H 4800
	#define M 3000
	#define L 3000
	#define S .1
	PWM1->cmpa=PWM1->cmpb=PWM2->cmpa=PWM2->cmpb=L;
	while(1){
		MPU_Data g=mpuGet();
		printf("% 4.2f    % 4.2f    % 4.2f    % 4.2f    % 4.2f    % 4.2f    \r",g.AccX,g.AccY,g.AccZ,g.GyrX,g.GyrY,g.GyrZ);
		PWM1->cmpa=((g.AccX>+S) || (g.AccY>+S))?H:M;//NE++
		PWM1->cmpb=((g.AccX<-S) || (g.AccY>+S))?H:M;//SE-+
		PWM2->cmpa=((g.AccX>+S) || (g.AccY<-S))?H:M;//NW+-
		PWM2->cmpb=((g.AccX<-S) || (g.AccY<-S))?H:M;//SW--
	}
	pwmStop();
	return 0;
}*/
