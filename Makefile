main: main.c pid.o mpu.o pwm.o
	gcc main.c pid.o mpu.o pwm.o -o main -lm

pid.o: pid.c types.h pid.h
	gcc -c pid.c

mpu.o: mpu.c mpu.h MPU6050.h types.h
	gcc -c mpu.c

pwm.o: pwm.c types.h pwm.h
	gcc -c pwm.c

clean:
	rm *.o main
