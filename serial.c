#include <avr/interrupt.h>
#include "serial.h"

char serialBuffer[256];
uint8_t bufferIdx=0;
uint8_t bufferSendIdx=0;
uint8_t serialReady=1;
ISR(USART_TX_vect){
	if(bufferIdx!=bufferSendIdx){
		serialReady=0;
		//waitForUDR;
		UDR=serialBuffer[bufferSendIdx];
		serialBuffer[bufferSendIdx]=0;
		bufferSendIdx++;
	}else{
		serialReady=1;
	}
}
void SerialWriteBuffer(char* str){
	uint8_t i=0;
	while(str[i]){
		if(serialReady){
			UDR=str[i];
		}else{
			serialBuffer[bufferIdx]=str[i];
			bufferIdx++;
		}
		i++;
	}
}
void SerialInit(uint32_t baud){
	UCSRA=0x02;
	UCSRB=0x98; // tanpa tx interrupt
	//UCSRB=0xD8; // dengan tx interrupt
	UCSRB|=(0<<UDRIE0)|(0<<TXCIE0);
	UCSRC=0x86;

	/*UCSR0A=0x02;
	UCSR0B=0xD8;
	UCSR0C=0x06;
	UBRR0H=0x00;
	UBRR0L=0x10;*/

	uint16_t ubrr_tmp=((F_CPU>>3)/baud)-1;
	UBRRL=(ubrr_tmp&0xFF);
	UBRRH=(ubrr_tmp>>8);
	uint8_t i=255;
	while(i--){
		serialBuffer[i]=0;
	}
}

void SerialWrite(char* str){
	uint8_t i=0;
	while(str[i]){
		while( !( UCSRA & (1<<UDRE) ));
		UDR=str[i];
		i++;
	}
}

