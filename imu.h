#include <avr/io.h>
#include <stdint.h>
#include "mpu6050.h"

#ifndef _IMU_
#define _IMU_

#define ROLL  _X
#define PITCH _Y
#define YAW   _Z

#if MPU6050_GYRO_FS==MPU6050_GYRO_FS_250
#define resolutionDevider 131.0
#elif MPU6050_GYRO_FS==MPU6050_GYRO_FS_500
#define resolutionDevider 65.5
#elif MPU6050_GYRO_FS==MPU6050_GYRO_FS_1000
#define resolutionDevider 32.8
#elif MPU6050_GYRO_FS==MPU6050_GYRO_FS_2000
#define resolutionDevider 16.4
#endif

#ifndef DT_FLOAT
#define DT_FLOAT 0.001
#endif

#define accCmplFilterConst 0.003
/*enum{
	_X,
	_Y,
	_Z
}axis;*/

#define ACC_1G 16384

#ifdef USE_FLOAT
extern float attitudeVector[3];
#else
extern int32_t attitudeVector[3];
#endif
extern int16_t gyroADC[3];
extern int16_t accADC[3];
extern int16_t accCalibrationData[3];

void initIMU();
void calibrateGyro();
void calibrateAcc();
void updateAcc();
void updateGyroVector();
void updateAccVector();
void estimateAttitude(int32_t* angle);

#endif