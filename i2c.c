/*
PWR_MGR – configuration horloge (il faut absolument sélectionner une horloge, par exemple PLL_XGYRO),
GYRO_CONFIG – précision du gyromètre,
ACCEL_CONFIG – précision de l'accéléromètre
*/
#define ACCEL_XOUT 0x3B
#define ACCEL_YOUT 0x3D
#define ACCEL_ZOUT 0x3F
#define GYRO_XOUT  0x43
#define GYRO_YOUT  0x45
#define GYRO_ZOUT  0x47


/* I2C */
#define GDIVIDER 8192
#define NMULTIPL 9.80665

int xyzOffsetAverageDivider=0;
long xOffsetAverageSum=0, yOffsetAverageSum=0, zOffsetAverageSum=0;
short   ax  , ay  , az  , gx  , gy  , gz  ;//current
short  Dax=0,Day=0,Daz=0,Dgx=0,Dgy=0,Dgz=0;//delta from base

// Velocity and Traveled distance
double dt= 0.01; // time between samples: 10 ms
double xVelocity=.0, yVelocity=.0, zVelocity=.0; // in m/s
double xTravel=.0, yTravel=.0, zTravel=.0; // in m

void addMeasurementsToOffset( short xAcceleration, short yAcceleration, short zAcceleration) {
	xOffsetAverageSum += xAcceleration;
	yOffsetAverageSum += yAcceleration;
	zOffsetAverageSum += zAcceleration;
	xyzOffsetAverageDivider++;
}
void addMeasurementsToTravel( short xAcceleration, short yAcceleration, short zAcceleration) {
	// (convert to double and) remove offset if there is any
	double ax = xAcceleration;
	double ay = yAcceleration;
	double az = zAcceleration;
	if( xyzOffsetAverageDivider > 0) {
		ax -= (double)xOffsetAverageSum / xyzOffsetAverageDivider;
		ay -= (double)yOffsetAverageSum / xyzOffsetAverageDivider;
		az -= (double)zOffsetAverageSum / xyzOffsetAverageDivider;
	}
	
	// convert to g force
	ax = (ax*NMULTIPL)/GDIVIDER;
	ay = (ay*NMULTIPL)/GDIVIDER;
	az = (az*NMULTIPL)/GDIVIDER;
	
	// change in velocity, v = v0 + at
	xVelocity += ax * dt;
	yVelocity += ay * dt;
	zVelocity += az * dt;
	// distance moved in dt, s = 1/2 a t^2 + vt
	xTravel += (0.5 * ax * dt * dt + xVelocity * dt);
	yTravel += (0.5 * ay * dt * dt + yVelocity * dt);
	zTravel += (0.5 * az * dt * dt + zVelocity * dt);
	
	//printf("Travel : %7f %7f %7f\n",xTravel,yTravel,yTravel);
	//printf("Veloce : %7f %7f %7f\n",xVelocity,yVelocity,zVelocity);
}
double getVelocity(){return sqrt((xVelocity*xVelocity + yVelocity*yVelocity + zVelocity*zVelocity));}
double getTravel(){return sqrt((xTravel * xTravel + yTravel * yTravel + zTravel * zTravel));}

unsigned char i2c_read(unsigned char cmd){
	union i2c_smbus_data i2cdata;
	struct i2c_smbus_ioctl_data  blk={1,cmd,I2C_SMBUS_BYTE_DATA,&i2cdata};
	if(ioctl(i2c,I2C_SMBUS,&blk)<0)
		printf("read error\n");
	return i2cdata.byte;
}
void i2c_write(unsigned char cmd, unsigned char value){
	union i2c_smbus_data i2cdata={.byte=value};
	struct i2c_smbus_ioctl_data  blk={0,cmd,I2C_SMBUS_BYTE_DATA,&i2cdata};
	if(ioctl(i2c,I2C_SMBUS,&blk)<0)
		printf("write error");
}

#define RD16(addr) ((i2c_read(addr)<<8) | i2c_read(addr+1))
#define DIVIDER 500

void i2c_getGyro(short*ax,short*ay,short*az,short*gx,short*gy,short*gz){
	*ax=RD16(ACCEL_XOUT)-Dax,*ay=RD16(ACCEL_YOUT)-Day,*az=RD16(ACCEL_ZOUT)-Daz;//acceleration
	*gx=RD16(GYRO_XOUT )-Dgz,*gy=RD16(GYRO_YOUT )-Dgy,*gz=RD16(GYRO_ZOUT )-Dgz;//gyroscope
}
void*i2c_thread(void* arg){
	//calibrate
	int nb,_ax=0,_ay=0,_az=0,_gx=0,_gy=0,_gz=0;//accumulated values
	for(nb=0;(nb<=50) && alive;nb++){//calibrate = accumulate value
		i2c_getGyro(&ax,&ay,&az,&gx,&gy,&gz);
		_ax+=ax,_ay+=ay,_az+=az;
		_gx+=gx,_gy+=gy,_gz+=gz;
		usleep(10*1000);//10ms
	}
	//setup delta using average accumulated values
	Dax=_ax/nb;Day=_ay/nb;Daz=_az/nb;
	Dgx=_gx/nb;Dgy=_gy/nb;Dgz=_gz/nb;
	
	//main loop
	while(alive){
		i2c_getGyro(&ax,&ay,&az,&gx,&gy,&gz);
		printf("G : %5i %5i %5i\tA : %5i %5i %5i \n",
			gx/DIVIDER, gy/DIVIDER, gz/DIVIDER,
			ax/DIVIDER, ay/DIVIDER, az/DIVIDER);
		//printf("G_: %7i %7i %7i A_: %7i %7i %7i \n\n",_gx,_gy,_gz, _ax,_ay,_az);
		//addMeasurementsToOffset(ax,ay,az);
		//addMeasurementsToTravel(ax,ay,az);
		usleep(10*1000);//10ms
	}
	return NULL;
}
void i2c_start(){
	printf("%s\n",__func__);
	
	if((i2c = open(I2C_PATH, O_RDWR))<0)
		printf("can't open i2c bus %s\n",I2C_PATH);
	
	//system("i2cset -y 3 0x68 0x6B 0x01");//0x68=vendor_id , 0x6B=PWR_MGMT_1
	
	if(ioctl(i2c,I2C_SLAVE,0x68) < 0)
		printf("Failed to acquire peripherical access\n");
	
	if(pthread_create(&i2c_th, NULL, i2c_thread, NULL)<0)
		printf("i2c_thread start error\n");
}
void i2c_stop(){
	printf("%s\n",__func__);
	close(i2c);
}