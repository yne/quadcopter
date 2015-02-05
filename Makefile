main: main.c pid.o mpu.o pwm.o common.o keyboard.o
	gcc main.c pid.o mpu.o pwm.o common.o keyboard.o -o main -lm

pid.o: pid.c common.h pid.h
	gcc -c pid.c

mpu.o: mpu.c mpu.h MPU6050.h common.h
	gcc -c mpu.c

pwm.o: pwm.c common.h pwm.h
	gcc -c pwm.c
	
common.o: common.c common.h 
	gcc -c common.c
	
keyboard.o: keyboard.c keyboard.h common.h 
	gcc -c keyboard.c

clean:
	rm *.o main
