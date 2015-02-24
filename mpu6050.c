#include "mpu6050.h"

#define ACC_ORIENTATION(X, Y, Z)  {accADC[0]  = X; accADC[1]  = Y; accADC[2]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[0] = X; gyroADC[1] = Y; gyroADC[2] = Z;}

uint8_t rawADC[6];
uint16_t accADC[3];
uint16_t gyroADC[3];

void mpu6050_init(){
	i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);
	volatile uint16_t wait=16000;
	while(--wait);
	
	i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03); // set clock source
	i2c_writeReg(MPU6050_ADDRESS, 0x1B, MPU6050_GYRO_FS <<3); // set gyro range
	//i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);
	i2c_writeReg(MPU6050_ADDRESS, 0x1C, MPU6050_ACCEL_FS <<3); // set acc range
	//i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);
	i2c_writeReg(MPU6050_ADDRESS, 0x1A, 0x00);
	i2c_errors_count=0;
}

/*void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}*/

uint8_t mpu6050_test(){
	uint8_t deviceId;
	deviceId = i2c_readReg(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I);
	return deviceId==0x68;
}

void mpu6050_readAccel(int16_t* buffer){
	uint8_t raw[6];
	i2c_start(MPU6050_ADDRESS<<1);
	i2c_write(0x3B);
	//i2c_read_to_buf(MPU6050_ADDRESS, &raw, 6);
	i2c_start((MPU6050_ADDRESS<<1)|1);
	uint8_t* b=(uint8_t*) raw;
	uint8_t i=6;
	while(i--){
		*b++=i2c_read(i);
	}
	buffer[IMU_X]=DIR_X*(int16_t)((raw[0]<<8)|raw[1]);//>>4;
	buffer[IMU_Y]=DIR_Y*(int16_t)((raw[2]<<8)|raw[3]);//>>4;
	buffer[IMU_Z]=DIR_Z*(int16_t)((raw[4]<<8)|raw[5]);//>>4;
}
void mpu6050_readGyro(int16_t* buffer){
	uint8_t raw[6];
	i2c_start(MPU6050_ADDRESS<<1);
	i2c_write(0x43);
	//i2c_read_to_buf(MPU6050_ADDRESS, &raw, 6);
	i2c_start((MPU6050_ADDRESS<<1)|1);
	uint8_t* b=(uint8_t*) raw;
	uint8_t i=6;
	while(i--){
		*b++=i2c_read(i);
	}
	buffer[IMU_X]=DIR_X*(int16_t)((raw[0]<<8)|raw[1]);
	buffer[IMU_Y]=DIR_Y*(int16_t)((raw[2]<<8)|raw[3]);
	buffer[IMU_Z]=DIR_Z*(int16_t)((raw[4]<<8)|raw[5]);
}