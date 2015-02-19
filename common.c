#include <sys/time.h>
#include "common.h"

unsigned long gettime(){
	struct timeval tv;
	struct timezone tz;
#ifndef WCET
	gettimeofday(&tv, &tz);
#endif
	return tv.tv_sec * 1000000L + tv.tv_usec;   
}

int pidToInt(float pid){
	return (int) (pid * 5000.0);
}

float usr_pitch=0.0,usr_roll=0.0;
unsigned global_speed=SPEED_MIN;
