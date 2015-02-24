#include <util/delay.h>
#include "imu.h"
#include "fastMath.h"
#include <math.h>

#include "config.h"

#ifdef USE_FLOAT
float attitudeVector[3];
#else
int32_t attitudeVector[3];
#endif
int16_t accADC[3];
int16_t gyroADC[3];

//int32_t angle[2];

#define gyroGain (1.0 / resolutionDevider / 180.0 * PI * DT_FLOAT)
//#define gyroGain 0.0001333

int16_t gyroCalibrationData[3];
int16_t accCalibrationData[3];

void calibrateGyro(){
	uint8_t counter=200;
	int16_t tmpGyro[3];
	int32_t gyroSum[3]={0,0,0};
	while(counter--){
		mpu6050_readGyro(tmpGyro);
		for(uint8_t i=0;i<3;i++){
			gyroSum[i]+=tmpGyro[i];
		}
		_delay_ms(1);
	}
	for(uint8_t i=0;i<3;i++){
		gyroCalibrationData[i]=(int16_t)(gyroSum[i]/200);
	}
}
void calibrateAcc(){
	uint8_t counter=200;
	int16_t tmpAcc[3];
	int32_t accSum[3]={0,0,0};
	const int16_t imuOffset[]={0,0,ACC_1G};
	while(counter--){
		mpu6050_readAccel(tmpAcc);
		for(uint8_t i=0;i<3;i++){
			accSum[i]+=tmpAcc[i];
		}
		_delay_ms(1);
	}
	for(uint8_t i=0;i<3;i++){
		accCalibrationData[i]=(int16_t)(accSum[i]/200)-imuOffset[i];
	}
}

void initIMU(){
	int16_t rawAcc[3];
	mpu6050_readAccel(rawAcc);
	attitudeVector[_X]=rawAcc[_X]-accCalibrationData[_X];
	attitudeVector[_Y]=rawAcc[_Y]-accCalibrationData[_Y];
	attitudeVector[_Z]=rawAcc[_Z]-accCalibrationData[_Z];
}
void updateAcc(){
	//int16_t rawAcc[3];
	mpu6050_readAccel(accADC);
}
void updateAccVector(){
	for(uint8_t i=0;i<3;i++){
		//attitudeVector[i]=accCmplFilterConst*accADC[i]+(1-accCmplFilterConst)*attitudeVector[i];
		#ifdef USE_FLOAT
		attitudeVector[i]+=accCmplFilterConst*(accADC[i]-attitudeVector[i]);
		#else
		attitudeVector[i]+=( ( (accADC[i]-accCalibrationData[i]) - attitudeVector[i] ) >> 8 );
		#endif
	}
}
#ifdef USE_FLOAT
inline void rotateVector(float* vect, float *delta)
#else
inline void rotateVector(int32_t* vect, int32_t *delta)
#endif
{
	//fp_vector v_tmp = *v;
	#ifdef USE_FLOAT
	float tmp_vect[3];
	#else
	int32_t tmp_vect[3];
	#endif
	for(uint8_t i=0;i<3;i++){
		tmp_vect[i]=vect[i];
	}
	#ifdef USE_FLOAT
	vect[_Z] += (delta[_Y] * tmp_vect[_X] - delta[_X] * tmp_vect[_Y]);
	vect[_X] -= (delta[_Y] * tmp_vect[_Z] - delta[_Z] * tmp_vect[_Y]);
	vect[_Y] += (delta[_X] * tmp_vect[_Z] - delta[_Z] * tmp_vect[_X]);
	#else
	vect[_Z] += (int32_t)(delta[_Y] * tmp_vect[_X] - delta[_X] * tmp_vect[_Y])>>16;
	vect[_X] -= (int32_t)(delta[_Y] * tmp_vect[_Z] - delta[_Z] * tmp_vect[_Y])>>16;
	vect[_Y] += (int32_t)(delta[_X] * tmp_vect[_Z] - delta[_Z] * tmp_vect[_X])>>16;
	#endif

	/*
	vect[_Z] -= delta[_Y] * tmp_vect[_X] + delta[_X] * tmp_vect[_Y];
	vect[_X] += delta[_Y] * tmp_vect[_Z] - delta[_Z] * tmp_vect[_Y];
	vect[_Y] += delta[_X] * tmp_vect[_Z] + delta[_Z] * tmp_vect[_X];
	*/
}
void updateGyroVector(){
	#ifdef USE_FLOAT
	float deltaAngle[3];
	#else
	int32_t deltaAngle[3];
	#endif

	//float tmpAttitude[3];
	mpu6050_readGyro(gyroADC);
	for(uint8_t i=0;i<3;i++){
		#ifdef USE_FLOAT
		int32_t gyroData=(int32_t)(gyroADC[i]-gyroCalibrationData[i]);
		#else
		int32_t gyroData=(int32_t)(gyroADC[i]-gyroCalibrationData[i])<<16;
		#endif
		deltaAngle[i]=gyroGain*gyroData;
		//tmpAttitude[i]=(float)attitudeVector[i];
	}
	rotateVector(attitudeVector,deltaAngle);
	/*for(uint8_t i=0;i<3;i++){
		attitudeVector[i]=(int32_t)tmpAttitude[i];
	}*/
}
void estimateAttitude(int32_t* angle){
	// 200us
	angle[ROLL]  = Rajan_FastArcTan2_deg1000(attitudeVector[_Y] , sqrt(attitudeVector[_Z]*attitudeVector[_Z]+attitudeVector[_X]*attitudeVector[_X]));
	// 142 us
	angle[PITCH] = Rajan_FastArcTan2_deg1000(-attitudeVector[_X] , attitudeVector[_Z]);
}
