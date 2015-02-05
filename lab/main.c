#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <math.h>
#include "pwm.h" //camsi.ups-tlse.fr/lib/exe/fetch.php?media=embedded:rm-mpu-6000a-00v4.2.pdf
#include "MPU6050.h"

#define assert(e)  ((e) ? (void)0 : _assert(#e,__FILE__,__LINE__,__FUNCTION__))
typedef struct{float AccX,AccY,AccZ,GyrX,GyrY,GyrZ,PX,PY;}MPU_Data;
typedef struct{
	PWM_map*pwm1,*pwm2;
	GPIO_map*gpio;
	int i2c,kbd;//file descriptors
	MPU_Data avg,cur,old;
	float PIDint[2],PIDerr[2];
	unsigned long t;
	int alive,speed[3];
}Quad_ctx;
#define PWM1  ctx.pwm1
#define PWM2  ctx.pwm2
#define NE PWM1->cmpa
#define SE PWM1->cmpb
#define NW PWM2->cmpa
#define SW PWM2->cmpb
#define GPIO  ctx.gpio
#define I2C   ctx.i2c
#define KBD   ctx.kbd

///////////GLOBAL//////////
Quad_ctx ctx={alive:1,speed:{3000,3000,3800}};
void _assert(char*code,char*file,int line,const char*func){exit(fprintf(stderr,"\n%s:%i(%s) > %s\n",file,line,func,code));}
void*mapGet(int start){
	int devmem_fd = open("/dev/mem", O_RDWR|O_SYNC);
	assert(devmem_fd>=0);
	void*map_addr=mmap(NULL,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,devmem_fd,start);
	close(devmem_fd);
	assert(map_addr!=MAP_FAILED);
	return map_addr;
}
void killme(int signo){
	ctx.alive=0;
	printf("\nBreak (%s)\n",signo?"Ctrl":"Esc");
}
////////// M P U //////////////
#define AF (16.0 / 32768.0) //AcclFactor = 16G
#define GF (M_PI * 500.0 / (32768.0*180.0)) //GyroFactor = 500 degree / sec
#define BS(H) ((short)((d.block[H*2+1]<<8)|d.block[H*2+2]))
unsigned long gettime(){
	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);
	return tv.tv_sec * 1000000L + tv.tv_usec;
}
#define K 0.90
#define Ki 0.00
#define Kd 0.00
#define Kp 1.90
float pidCompute(float*integral,float*lastErr, float mesure,float dt){
	float erreur = mesure;
	*integral += ((erreur + *lastErr ) / 2) * dt;
	float delta_err = (erreur - *lastErr)/dt;
	*lastErr = erreur;
	return
		(Ki * *integral) +
		(Kd * delta_err) +
		(Kp * erreur);
}
int mpuSet(unsigned short cmd,unsigned char val){
	union i2c_smbus_data data={byte:val};
	struct i2c_smbus_ioctl_data blk={I2C_SMBUS_WRITE,cmd,I2C_SMBUS_BYTE_DATA,&data};
	return ioctl(I2C,I2C_SMBUS,&blk);
}
MPU_Data mpuGet(){
	union i2c_smbus_data d={block:{14}};
	struct i2c_smbus_ioctl_data  blk={I2C_SMBUS_READ,MPU6050_RA_ACCEL_XOUT_H,I2C_SMBUS_I2C_BLOCK_DATA,&d};
	assert(ioctl(I2C,I2C_SMBUS,&blk)>=0);

	ctx.old=ctx.cur;
	float old_t=ctx.t;
	ctx.t = gettime();
	float dt = (ctx.t - old_t) / 1000000.0;

	float AX=K*(ctx.cur.AccX + (ctx.cur.GyrX - ctx.avg.GyrX) * dt) + (1.0 - K) * atan2f(ctx.cur.AccX,sqrt(powf(ctx.cur.AccY,2.0) + powf(ctx.cur.AccZ,2.0)));
	float AY=K*(ctx.cur.AccY + (ctx.cur.GyrY - ctx.avg.GyrY) * dt) + (1.0 - K) * atan2f(ctx.cur.AccY,sqrt(powf(ctx.cur.AccX,2.0) + powf(ctx.cur.AccZ,2.0)));

	float PX=pidCompute(&ctx.PIDint[0],&ctx.PIDerr[0],AX,dt/ 10000.0);
	float PY=pidCompute(&ctx.PIDint[1],&ctx.PIDerr[1],AY,dt/ 10000.0);

	ctx.cur=(MPU_Data){BS(0)*AF,BS(1)*AF,BS(2)*AF,BS(4)*GF-ctx.avg.GyrX,BS(5)*GF-ctx.avg.GyrY,BS(6)*GF,PX,PY};

	return ctx.cur;
}
void mpuInit(char*path,int calib){
	assert((I2C = open(path, O_RDWR))>=0);
	assert(ioctl(I2C,I2C_SLAVE,MPU6050_DEFAULT_ADDRESS)>=0);
	mpuSet(MPU6050_RA_GYRO_CONFIG , MPU6050_GYRO_FS_250);
	mpuSet(MPU6050_RA_PWR_MGMT_1  , MPU6050_CLOCK_PLL_XGYRO);
	//mpuSet(MPU6050_RA_PWR_MGMT_2  , MPU6050_WAKE_FREQ_1P25);
	mpuSet(MPU6050_RA_SMPLRT_DIV  , MPU6050_CLOCK_DIV_267);
	mpuSet(MPU6050_RA_INT_ENABLE  , MPU6050_INTERRUPT_DATA_RDY);
	mpuSet(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16);
	MPU_Data tmp,avg;
	int i;for(i=0;i<calib;i++){
		tmp=mpuGet();
		avg.GyrX+=tmp.GyrX/calib;
		avg.GyrY+=tmp.GyrY/calib;
	}
	ctx.avg=avg;
	printf("AVG=X:%f Y:%f\n",avg.GyrX,avg.GyrY);
}
void mpuStop(){
	close(I2C);
}
///////////// P W M //////////
void pwmInit(){
	PWM1=mapGet(PWM_1_START);
	PWM2=mapGet(PWM_2_START);
	GPIO=mapGet(GPIO_1_START);
	NE=SE=NW=NE=ctx.speed[0];
}
void pwmStop(){
	NE=SE=NW=NE=ctx.speed[0];
	munmap(&PWM1,getpagesize());
	munmap(&PWM2,getpagesize());
	munmap(&GPIO,getpagesize());
}
///////////// K B D //////////
void kbdInit(char*path){
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
///////////// M A I N ////////
void init(){
	setvbuf(stdout, NULL, _IONBF, 0);//debufferise stdout (pour les \r via putty)
	signal(SIGINT ,killme);//CTRL+C
	signal(SIGSTOP,killme);//CTRL+Z
	kbdInit("/dev/input/event0");
	mpuInit("/dev/i2c-3",32);
	pwmInit();
}
void stop(){
	pwmStop();
	mpuStop();
	kbdStop();
}
int main(){
	init();
	#define S .15
	while(ctx.alive){//can be stop using CTRL+C/Z / ESC
		MPU_Data g=mpuGet();
		kbdGet();
		printf("% 2.2f  % 2.2f  % 2.2f  % 2.2f  % 2.2f  % 2.2f   [%f/%f]    \r",g.AccX,g.AccY,g.AccZ, g.GyrX,g.GyrY,g.GyrZ,g.PX,g.PY);
		NE=ctx.speed[((g.AccX>+S) || (g.AccY>+S))?2:1];
		SE=ctx.speed[((g.AccX<-S) || (g.AccY>+S))?2:1];
		NW=ctx.speed[((g.AccX>+S) || (g.AccY<-S))?2:1];
		SW=ctx.speed[((g.AccX<-S) || (g.AccY<-S))?2:1];
	}
	stop();
	return 0;
}
