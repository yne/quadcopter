#include <sys/time.h>
unsigned long gettime(){
	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);
	return tv.tv_sec * 1000000L + tv.tv_usec;   
}

int pidToInt(float pid){
	return (int) (pid * 5000.0);
	
	
}