#include "brushlessmotor.h"
#include "config.h"
#include <math.h>

uint8_t rollMotorSinArray[256];
uint8_t pitchMotorSinArray[256];
int8_t pwmSinArray[256];
int8_t motorPWM[2][3];
void initTimer(){
	TCCR0A=0xA1;
	TCCR0B=0x01;

	TCCR1A=0xA1;
	TCCR1B=0x01;

	TCCR2A=0xA1;
	TCCR2B=0x01;

	TIMSK0=0x01;
	TIMSK1=0x00;
	TIMSK2=0x00;

	OCR2A=0;
	OCR2B=0;
	OCR1A=0;
	OCR1B=0;
	OCR0A=0;
	OCR0B=0;

	DDRB|=(1<<PB1)|(1<<PB2)|(1<<PB3);
	DDRD|=(1<<PD3)|(1<<PD5)|(1<<PD6);
}
void initSinArray(){
	for(uint16_t i=0;i<256;i++){
		//pwmSinArray[i] = sin(2.0 * i / 256 * 3.14159265) * 127.0;
		int16_t tmp;
		tmp = sin(2.0 * i / 256 * 3.14159265) * 127.0;
		tmp += 127;
		rollMotorSinArray[i] = (uint8_t)((tmp*rollMotorPWM)>>8);
		pitchMotorSinArray[i]= (uint8_t)((tmp*pitchMotorPWM)>>8);
	}
}
inline void setPWM(uint8_t motorID,uint8_t pos,uint8_t maxPwm){
	uint16_t pwm;

	pwm=pwmSinArray[pos];
	pwm=pwm*maxPwm;
	motorPWM[motorID][0]=(pwm>>8)+128;

	pwm=pwmSinArray[(uint8_t)(pos+85)];
	pwm=pwm*maxPwm;
	motorPWM[motorID][1]=(pwm>>8)+128;

	pwm=pwmSinArray[(uint8_t)(pos+170)];
	pwm=pwm*maxPwm;
	motorPWM[motorID][2]=(pwm>>8)+128;
}
void setMotorPos(uint8_t roll, uint8_t pitch){
	motorPWM[rollMotor][0]=rollMotorSinArray[roll];
	roll+=85;
	motorPWM[rollMotor][1]=rollMotorSinArray[roll];
	roll+=85;
	motorPWM[rollMotor][2]=rollMotorSinArray[roll];

	motorPWM[pitchMotor][0]=pitchMotorSinArray[pitch];
	pitch+=85;
	motorPWM[pitchMotor][1]=pitchMotorSinArray[pitch];
	pitch+=85;
	motorPWM[pitchMotor][2]=pitchMotorSinArray[pitch];
	/*setPWM(rollMotor,roll,rollMotorPWM);
	setPWM(pitchMotor,pitch,pitchMotorPWM);*/
	
}
void updateMotor(){
	/*
	PWM_A_MOTOR1 OCR2A
	PWM_B_MOTOR1 OCR1B
	PWM_C_MOTOR1 OCR1A

	PWM_A_MOTOR0 OCR0A
	PWM_B_MOTOR0 OCR0B
	PWM_C_MOTOR0 OCR2B
	*/
	OCR2B=motorPWM[0][0];
	OCR0B=motorPWM[0][1];
	OCR0A=motorPWM[0][2];

	OCR2A=motorPWM[1][0];
	OCR1A=motorPWM[1][1];
	OCR1B=motorPWM[1][2];
}