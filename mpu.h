#ifndef _MPU_H
#define _MPU_H

typedef struct{float AccX,AccY,AccZ,Term,GyrX,GyrY,GyrZ;}MPU_Data_RAW;
typedef struct{float AX,AY;}MPU_Data;

MPU_Data mpuGet();
void mpuInit();
void mpuStop();

float getAX();
float getAY();

#endif
