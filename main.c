#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "def.h"
#include "i2c.h"
#include "serial.h"
#include "mpu6050.h"
#include "imu.h"
#include "brushlessmotor.h"
#include "rc.h"

#include "config.h"

volatile uint8_t loopFlag=0;
//volatile uint8_t timer;
ISR(TIMER0_OVF_vect){
	//UDR='A';
	static uint8_t freqCounter=0;
	timer++;
	freqCounter++;
	if(freqCounter==31){
		loopFlag=1;
		freqCounter=0;
	}
}
/*ISR(PCINT1_vect){
	LED_OFF;
}*/

inline int32_t constrain_int32(int32_t x , int32_t l, int32_t h) {
	if (x <= l) {
		return l;
	} else if (x >= h) {
		return h;
	} else {
		return x;
	}
}
void writeCalibrationData(){
	//uint8_t eepromAddress=0x01;
	int8_t* calibData=(uint8_t*)accCalibrationData;
	for(uint8_t i=0;i<6;i++){
		eeprom_write_byte(eepromStartAddress+i,*(calibData+i));
	}
}
void readCalibrationData(){
	int8_t* calibData=(uint8_t*)accCalibrationData;
	for(uint8_t i=0;i<6;i++){
		*(calibData+i)=eeprom_read_byte(eepromStartAddress+i);
	}
}
/************************/
/* PID Controller       */
/************************/
// PID integer inplementation
//   DTms  ... sample period (ms)
//   DTinv ... sample frequency (Hz), inverse of DT (just to avoid division)
int32_t ComputePID(int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int32_t Kp, int16_t Ki, int32_t Kd){
	int32_t error = setPoint - in;
	int32_t Ierr;

	Ierr = error * Ki * DT_ms;
	Ierr = constrain_int32(Ierr, -(int32_t)32768, (int32_t)32767);
	*errorSum += Ierr;

	/*Compute PID Output*/
	int32_t out = (Kp * error) + *errorSum + Kd * (error - *errorOld) * Fs;
	*errorOld = error;

	out = out / 4096 / 8;

	return out;
}
int32_t _Kp=rollKp;
int32_t _Ki=rollKi;
int32_t _Kd=rollKd;
volatile uint8_t debugMode=0;
ISR(USART_RX_vect){
	char data=UDR;
	static uint8_t counter=0;
	char haha[8];
	
	if(data=='A'){
		cli();
		calibrateAcc();
		writeCalibrationData();
		sei();
	}else if(data='\r'||data=='\n'){
		counter++;
		if(counter==3){
			debugMode=1;
		}
	}/*else if(data==']'){
		_Kp+=10;
		sprintf(haha,"%d\r",_Kp);
		SerialWrite(haha);
	}else if(data=='['){
		_Kp-=10;
		sprintf(haha,"%d\r",_Kp);
		SerialWrite(haha);
	}else if(data=='l'){
		_Ki+=1;
		sprintf(haha,"%d\r",_Ki);
		SerialWrite(haha);
	}else if(data=='k'){
		_Ki-=1;
		sprintf(haha,"%d\r",_Ki);
		SerialWrite(haha);
	}else if(data=='.'){
		_Kd+=1;
		sprintf(haha,"%d\r",_Kd);
		SerialWrite(haha);
	}else if(data==','){
		_Kd-=1;
		sprintf(haha,"%d\r",_Kd);
		SerialWrite(haha);
	}*/
}
inline int32_t map(int32_t input, int32_t il, int32_t ih, int32_t ol, int32_t oh){
	return (input-il) * (oh-ol)/(ih-il) + ol;
}
void main(void){
	char debugStr[16];

	int32_t pitchSetpoint=10000;
	int32_t pitchErrorSum=0;
	int32_t pitchErrorOld=0;
	
	int32_t rollSetpoint=0;
	int32_t rollErrorSum=0;
	int32_t rollErrorOld=0;

	DDRB|=(1<<PB0);
	DDRD|=0x02;

	LED_OFF;

	SerialInit(115200);
	
	UBRRH=0x00;
	UBRRL=0x10;
	SerialWrite(".Init\r");

	initTimer();

	//TCCR1A=0x00;
	//TCCR1B=0x02;
	uint16_t t0,t1;

	sei();
	
	SerialWrite("Halo\r");
	_delay_ms(1000);
	i2c_init();
	readCalibrationData();
	mpu6050_init();
	if(!mpu6050_test()){
		SerialWrite(".IMU not found\r");
		while(1);
	}
	_delay_ms(100);
	SerialWrite(".Init sin array\r");
	initSinArray();
	SerialWrite(".Gyro calibration\r");
	calibrateGyro();
	//calibrateAcc();
	/*uint8_t version=eeprom_read_byte(0x00);
	if(version!=EEPROM_VERSION){
		writeCalibrationData();
	}*/
	initIMU();
	initRC();
	SerialWrite(".Ready\r");
	LED_ON;

	DDRC|=(0<<CH0_bit)|(0<<CH1_bit)|(0<<CH2_bit)|(1<<CH3_bit);
	//DDRD|=(1<<CH2_bit)|(1<<CH3_bit);

	uint8_t loopCounter=0;
	//uint8_t tes=0;
	while(1){
		//int16_t rawAccel[3],rawGyro[3];
		int32_t angle[2];

		//mpu6050_readAccel(rawAccel);
		if(loopFlag){ // total 780us (min)
			//updateAcc();
			//updateAccVector();
			//mpu6050_readGyro(rawGyro);
			//mpu6050_readAccel(rawAccel);
			//t0=TCNT1;
			setBit(PORTC,2);
			updateMotor(); // 2,6us
			#ifdef USE_GYRO
			updateGyroVector(); // 300us
			#endif

			estimateAttitude(angle); // 330us
			int8_t rollPID=ComputePID(angle[ROLL],  rollSetpoint, &rollErrorSum, &rollErrorOld, rollKp, rollKi, rollKd);  // 76us
			int8_t pitchPID=ComputePID(angle[PITCH],pitchSetpoint,&pitchErrorSum,&pitchErrorOld, pitchKp, pitchKi, pitchKd); // 76us
			setMotorPos(rollPID,pitchPID); // 4,3us
			clearBit(PORTC,2);
			//t1=TCNT1;
			//tes++;
			
			loopCounter++;
			switch(loopCounter){
				case 1:
					updateAcc();
					break;
				case 2:
					#ifdef USE_ACCEL
					updateAccVector();
					#endif
					break;
				case 3:
					if(debugMode)
					//sprintf(debugStr,"%5d\t",angle[PITCH]/1000);
					sprintf(debugStr,"%d\t",pitchSetpoint/1000);
					//sprintf(debugStr,"%5d\t",gyroADC[0]-gyroCalibrationData[0]);
					//sprintf(debugStr,"%5d\t",accADC[0]);
					//sprintf(debugStr,"%5d\t",accCalibrationData[0]);
					//sprintf(debugStr,"%5d\t",attitudeVector[2]);
					//sprintf(debugStr,"%5d\t",rawAccel[0]);
					break;
				case 4:
					if(debugMode)
					SerialWrite(debugStr);
					break;
				case 5:
					if(debugMode)
					//sprintf(debugStr,"%5d\t",accADC[2]);
					//sprintf(debugStr,"%5d\t",accCalibrationData[2]);
					sprintf(debugStr,"%5d\t",angle[ROLL]/1000);
					//sprintf(debugStr,"%d\t",rcInput[CH1]);
					break;
				case 6:
					if(debugMode)
					SerialWrite(debugStr);
					break;
				case 7:
					if(debugMode)
					sprintf(debugStr,"%dus\r",(t1-t0)>>1);
					break;
				case 8:
					if(debugMode)
					SerialWrite(debugStr);
					//LED_TOGGLE;
					evaluateRC();
					break;
				case 9:
					// RC: 1000-2000us
					#ifdef RC_PITCH
					//pitchSetpoint= (int32_t)(rcInput[RC_PITCH]-1000) * (PITCH_MAX-PITCH_MIN) + (PITCH_MIN*1000);
					pitchSetpoint= map(rcInput[RC_PITCH],1000,2000,PITCH_MIN*1000,PITCH_MAX*1000);
					#endif
					#ifdef RC_ROLL
					rollSetpoint=(rcInput[RC_ROLL]-1000) * (ROLL_MAX-ROLL_MIN) + ROLL_MIN*1000;
					#endif
					break;
				default:
					if(loopCounter==10) loopCounter=0;
					break;
			}
			loopFlag=0;
			
		}
		
		
		
		//sprintf(debugStr,"%5d\t",rawGyro[1]);
		//SerialWriteBuffer(debugStr);
		//sprintf(debugStr,"%5d\t",rawGyro[2]);
		//SerialWriteBuffer(debugStr);
		//
		//SerialWriteBuffer(debugStr);
		
		//_delay_ms(100);
	}
}