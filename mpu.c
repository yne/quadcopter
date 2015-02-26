#include <assert.h>
#include <fcntl.h>
#include <math.h>

#include <stdio.h>

#include "MPU6050.h"
#include "mpu.h"

#include "common.h"


/////////// GLOBAL ////////////

#define I2C   ctx.i2c
#define K 0.95

////////// M P U //////////////

#define I2C_PATH "/dev/i2c-3"
MPU_Data mpu_data; //Data du acceleration
MPU_Data_RAW mpu_data_raw; //data brutes

unsigned long lastTime;
float dt;

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
	//normalisation
	float AF=  16.0 / 32768.0; //AcclFactor = 16G
	float GF= M_PI * 500.0 / (32768.0 * 180.0); //GyroFactor = 500 degree / sec
	int i;
	mpu_data_raw = (MPU_Data_RAW){};
	for(i=0;i<4;i++){
		union i2c_smbus_data d={block:{14}};
		struct i2c_smbus_ioctl_data  blk={I2C_SMBUS_READ,MPU6050_RA_ACCEL_XOUT_H,I2C_SMBUS_I2C_BLOCK_DATA,(union i2c_smbus_data*)&d};
		if(ioctl(I2C,I2C_SMBUS,&blk)<0)printf("Warning : Unable to get MPU data\n");
		mpu_data_raw.AccX+=SW(0)*AF/4.0;
		mpu_data_raw.AccY+=SW(1)*AF/4.0;
		mpu_data_raw.AccZ+=SW(2)*AF/4.0;
		mpu_data_raw.GyrX+=SW(4)*GF/4.0;
		mpu_data_raw.GyrY+=SW(5)*GF/4.0;
		mpu_data_raw.GyrZ+=SW(6)*GF/4.0;
	}
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
	for(i = 0; i < 50; i ++){
		mpuUpdate();
		avggX =+ mpu_data_raw.GyrX;
		avggY =+ mpu_data_raw.GyrY;
		usleep(500);
		
	}
	avggX /= 50;
	avggY /= 50;
	lastTime = gettime();
	//printf("%f %f\n", avggX, avggY );
}
void mpuStop(){
	close(I2C);
}


float atan2_fast( float y, float x ){
	static const uint32_t sign_mask = 0x80000000;
	static const float b = 0.596227f;
	// Extract the sign bits
	uint32_t ux_s = sign_mask & *((uint32_t *)&x);
	uint32_t uy_s = sign_mask & *((uint32_t *)&y);
	// Determine the quadrant offset
	float q = (float)( ( ~ux_s & uy_s ) >> 29 | ux_s >> 30 );
	// Calculate the arctangent in the first quadrant
	float bxy_a = fabs( b * x * y );
	float num = bxy_a + y * y;
	float atan_1q = num / ( x * x + bxy_a + num );
	// Translate it to the proper quadrant
	uint32_t uatan_2q = (ux_s ^ uy_s) | *((uint32_t *)&atan_1q);
	return q + *((float *)&uatan_2q);
}
float sqrt_fast(const float x){
	const float xhalf = 0.5f*x;
	union{float x;int i;} u;
	u.x = x;
	u.i = 0x5f3759df - (u.i >> 1); // gives initial guess y0
	return x*u.x*(1.5f - xhalf*u.x*u.x);// Newton step, repeating increases accuracy
}

float getAX(){
	return mpu_data.AX = K*(mpu_data.AX + (mpu_data_raw.GyrX- avggX) * dt) + (1.0 - K) * atan2_fast(mpu_data_raw.AccX,sqrt_fast((mpu_data_raw.AccY*mpu_data_raw.AccY) + (mpu_data_raw.AccZ*mpu_data_raw.AccZ)));
}
float getAY(){
	return mpu_data.AY = K*(mpu_data.AY + (mpu_data_raw.GyrY- avggY) * dt) + (1.0 - K) * atan2_fast(mpu_data_raw.AccY,sqrt_fast((mpu_data_raw.AccX*mpu_data_raw.AccX) + (mpu_data_raw.AccZ*mpu_data_raw.AccZ)));
}
