#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "pwm.h" //camsi.ups-tlse.fr/lib/exe/fetch.php?media=embedded:rm-mpu-6000a-00v4.2.pdf
#include "mpu.h"

/////////// GLOBAL ////////////
typedef struct{float AccX,AccY,AccZ,Term,GyrX,GyrY,GyrZ;}MPU_Data;
typedef struct{
	PWM_map*pwm1,*pwm2;
	GPIO_map*gpio;
	int i2c;
	MPU_Data avg;
}Quad_ctx;
Quad_ctx ctx;
#define PWM1  ctx.pwm1
#define PWM2  ctx.pwm2
#define GPIO  ctx.gpio
#define I2C   ctx.i2c

void die(char*msg){
	perror(msg);
	exit(0);
}
void*mapGet(int start){
	int devmem_fd = open("/dev/mem", O_RDWR|O_SYNC);
	assert(devmem_fd>=0);
	void*map_addr=mmap(NULL,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,devmem_fd,start);
	close(devmem_fd);
	assert(map_addr!=MAP_FAILED);
	return map_addr;
}

////////// M P U //////////////

#define I2C_PATH "/dev/i2c-3"

int mpuSet(unsigned short cmd,unsigned char val){
	union i2c_smbus_data data={byte:val};
	struct i2c_smbus_ioctl_data blk={I2C_SMBUS_WRITE,cmd,I2C_SMBUS_BYTE_DATA,&data};
	return ioctl(I2C,I2C_SMBUS,&blk);
}
MPU_Data mpuGet(){
	float AF=  16.0 / 32768.0; //AcclFactor = 16G
	float GF= 500.0 / 32768.0; //GyroFactor = 500 degree / sec
	union i2c_smbus_data d={block:{14}};
	MPU_Data avg={};
	struct i2c_smbus_ioctl_data  blk={I2C_SMBUS_READ,MPU6050_RA_ACCEL_XOUT_H,I2C_SMBUS_I2C_BLOCK_DATA,&d};
	int i;for(i=0;i<10;i++){
		assert(ioctl(I2C,I2C_SMBUS,&blk)>=0);
		//normalisation
		#define SW(H) ((short)((d.block[H*2+1]<<8)|d.block[H*2+2]))
		avg.AccX+=SW(0)*AF/10;
		avg.AccY+=SW(1)*AF/10;
		avg.AccZ+=SW(2)*AF/10;
		avg.GyrX+=SW(4)*GF/10;
		avg.GyrY+=SW(5)*GF/10;
		avg.GyrZ+=SW(6)*GF/10;
	}
	return avg;
}
void mpuInit(){
	assert((I2C = open(I2C_PATH, O_RDWR))>=0);
	assert(ioctl(I2C,I2C_SLAVE,MPU6050_DEFAULT_ADDRESS)>=0);
	mpuSet(MPU6050_RA_GYRO_CONFIG , MPU6050_GYRO_FS_500);
	mpuSet(MPU6050_RA_PWR_MGMT_1  , MPU6050_CLOCK_PLL_XGYRO);
	mpuSet(MPU6050_RA_PWR_MGMT_2  , MPU6050_WAKE_FREQ_1P25);
	mpuSet(MPU6050_RA_SMPLRT_DIV  , MPU6050_CLOCK_DIV_267);
	mpuSet(MPU6050_RA_INT_ENABLE  , MPU6050_INTERRUPT_DATA_RDY);
	mpuSet(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16);
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

int main(){
	setvbuf(stdout, NULL, _IONBF, 0);//debufferise stdout (pour les \r via putty)
	mpuInit();
	pwmInit();
	#define H 4000
	#define M 3500
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
}
