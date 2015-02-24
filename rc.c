#include <avr/io.h>
#include <avr/interrupt.h>
#include "rc.h"
#include "def.h"

volatile uint8_t startTime[4];
volatile uint8_t endTime[4];
int16_t rcInput[4]={0,0,0,0};
volatile uint8_t timer;

ISR(PCINT1_vect){
	uint8_t counter=timer;
	// cek rising edge
	if(PINC&(1<<CH1_bit)){
		startTime[CH1]=counter;
	}else{
		endTime[CH1]=counter;
		PCMSK1&=~_BV(CH1_PCINT);
		/*uint8_t pwm=counter-startTime[CH0];
		if((pwm>28)&&(pwm<69))
			rcInput[CH0]=(uint16_t)pwm<<5;*/
	}
//LED_OFF;
	/*if(PINC&(1<<CH1_bit)){
		startTime[CH0]=counter;
	}else{
		endTime[CH0]=counter;
		PCMSK1&=~_BV(CH1_PCINT);
	}*/
}

void initRC(){
	PCICR|=(1<<PCIE1);
	PCMSK1|=(1<<CH0_PCINT)|(1<<CH1_PCINT)|(0<<CH2_PCINT)|(0<<CH3_PCINT);
	//PCMSK2=(0<<CH2_PCINT)|(0<<CH3_PCINT);
	PCIFR=0x02;
	DDRC|=(0<<CH0_bit)|(0<<CH1_bit);
	PORTC|=(1<<CH0_bit)|(1<<CH1_bit);
}
void evaluateRC(){
	for(uint8_t i=0;i<2;i++){
		uint8_t pwm=endTime[i]-startTime[i];
		if((pwm>25)&&(pwm<70))
			rcInput[i]+=0.8*((pwm<<5)-rcInput[i]);
	}
	PCMSK1|=(1<<CH0_PCINT)|(1<<CH1_PCINT)|(0<<CH2_PCINT)|(0<<CH3_PCINT);
}