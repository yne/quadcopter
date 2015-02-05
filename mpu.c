#include <assert.h>
#include <fcntl.h>
#include <math.h>

#include <stdio.h>

#include "MPU6050.h"
#include "mpu.h"

#include "types.h"


#include <sys/time.h>
unsigned long gettime();


/////////// GLOBAL ////////////

#define I2C   ctx.i2c
#define K 0.90

////////// M P U //////////////

#define I2C_PATH "/dev/i2c-3"
MPU_Data mpu_data; //Data du acceleration
MPU_Data_RAW mpu_data_raw; //data brutes

unsigned long lastTime;
float dt;
//float gXdt = 0.0;
//float gYdt = 0.0;

float avggX = 0;
float avggY = 0;

int mpuSet(unsigned short cmd,unsigned char val){
	union i2c_smbus_data data={byte:val};
	struct i2c_smbus_ioctl_data blk={I2C_SMBUS_WRITE,cmd,I2C_SMBUS_BYTE_DATA,&data};
	return ioctl(I2C,I2C_SMBUS,&blk);
}
/*
MPU_Data mpuGet(){
	union i2c_smbus_data d={block:{14}};
	struct i2c_smbus_ioctl_data  blk={I2C_SMBUS_READ,MPU6050_RA_ACCEL_XOUT_H,I2C_SMBUS_I2C_BLOCK_DATA,(union i2c_smbus_data*)&d};
	assert(ioctl(I2C,I2C_SMBUS,&blk)>=0);
	//normalisation
	float AF=  16.0 / 32768.0; //AcclFactor = 16G
	float GF= 500.0 / 32768.0; //GyroFactor = 500 degree / sec
#define SW(H) ((short)((d.block[H*2+1]<<8)|d.block[H*2+2]))
	return ((MPU_Data) {
		SW(0)*AF-ctx.avg.AccX,
				SW(1)*AF-ctx.avg.AccY,
				SW(2)*AF-ctx.avg.AccZ,
				SW(4)*GF-ctx.avg.GyrX,
				SW(5)*GF-ctx.avg.GyrY,
				SW(6)*GF-ctx.avg.GyrZ
	});
}
*/

#define SW(H) ((short)((d.block[H*2+1]<<8)|d.block[H*2+2]))
void mpuUpdate(){
	union i2c_smbus_data d={block:{14}};
	struct i2c_smbus_ioctl_data  blk={I2C_SMBUS_READ,MPU6050_RA_ACCEL_XOUT_H,I2C_SMBUS_I2C_BLOCK_DATA,(union i2c_smbus_data*)&d};
	assert(ioctl(I2C,I2C_SMBUS,&blk)>=0);
	//normalisation
	float AF=  16.0 / 32768.0; //AcclFactor = 16G
	float GF= M_PI * 500.0 / (32768.0 * 180.0); //GyroFactor = 500 degree / sec

	mpu_data_raw = (MPU_Data_RAW){
				SW(0)*AF,//-ctx.avg.AccX,
				SW(1)*AF,//-ctx.avg.AccY,
				SW(2)*AF,//-ctx.avg.AccZ,
				SW(4)*GF,//-ctx.avg.GyrX,
				SW(5)*GF,//-ctx.avg.GyrY,
				SW(6)*GF//-ctx.avg.GyrZ
	};
	//gXdt = fmod((gXdt + mpu_data_raw.GyrX), M_PI);
	//gYdt = fmod((gYdt + mpu_data_raw.GyrY), M_PI);
	dt = (gettime() - lastTime) / 1000000.0;
	lastTime = gettime();
}


void mpuInit(){
	assert((I2C = open(I2C_PATH, O_RDWR))>=0);
	assert(ioctl(I2C,I2C_SLAVE,MPU6050_DEFAULT_ADDRESS)>=0);
	mpuSet(MPU6050_RA_GYRO_CONFIG , MPU6050_GYRO_FS_500);
	mpuSet(MPU6050_RA_PWR_MGMT_1  , MPU6050_CLOCK_PLL_XGYRO);
	//mpuSet(MPU6050_RA_PWR_MGMT_2  , MPU6050_WAKE_FREQ_1P25);
	mpuSet(MPU6050_RA_SMPLRT_DIV  , MPU6050_CLOCK_DIV_267);
	mpuSet(MPU6050_RA_INT_ENABLE  , MPU6050_INTERRUPT_DATA_RDY);
	mpuSet(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16);
	
	int i;
	for(i = 0; i < 1000; i ++){
		mpuUpdate();
		avggX =+ mpu_data_raw.GyrX;
		avggY =+ mpu_data_raw.GyrY;
		usleep(500);
		
	}
	avggX /= 1000;
	avggY /= 1000;
	lastTime = gettime();
	printf("%f %f\n", avggX, avggY );
}

//typedef struct{float AccX,AccY,AccZ,Term,GyrX,GyrY,GyrZ;}MPU_Data;
//typedef struct{float AX,AY;}MPU_Data;

float getAX(){
	mpu_data.AX = K*(mpu_data.AX + (mpu_data_raw.GyrX- avggX) * dt) + (1.0 - K) * atan2f(mpu_data_raw.AccX,
															  sqrt(powf(mpu_data_raw.AccY,2.0) + powf(mpu_data_raw.AccZ,2.0))
															  );
	printf("AX en ° : %f\n",mpu_data.AX * 180.0 / M_PI);
return mpu_data.AX;
}

float getAY(){
	mpu_data.AY = K*(mpu_data.AY + (mpu_data_raw.GyrY- avggY) * dt) + (1.0 - K) * atan2f(mpu_data_raw.AccY,
															  sqrt(powf(mpu_data_raw.AccX,2.0) + powf(mpu_data_raw.AccZ,2.0))
															  );
	printf("AY en ° : %f\n",mpu_data.AY * 180.0 / M_PI);
	return mpu_data.AY;
}


/*
int main(){
	setvbuf(stdout, NULL, _IONBF, 0);//debufferise stdout (pour les \r via putty)
	mpuInit();
	pwmInit();
	#define H 4000
	#define L 3000
	#define S .7
	PWM1->cmpa=PWM1->cmpb=PWM2->cmpa=PWM2->cmpb=L;
	while(0){
		MPU_Data g=mpuGet();
		printf("% 4.2f    % 4.2f    % 4.2f    % 4.2f    % 4.2f    % 4.2f    \r",g.AccX,g.AccY,g.AccZ,g.GyrX,g.GyrY,g.GyrZ);
		PWM1->cmpa=((g.AccX>S) && (g.AccY>S))?H:L;//NE++
		PWM1->cmpb=((g.AccX<S) && (g.AccY>S))?H:L;//SE-+
		PWM2->cmpa=((g.AccX>S) && (g.AccY<S))?H:L;//NW+-
		PWM2->cmpb=((g.AccX<S) && (g.AccY<S))?H:L;//SW--
	}
	pwmStop();
	return 0;
}
 */
