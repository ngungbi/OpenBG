#include <avr/io.h>
#include <stdint.h>

#ifndef _SERIAL_
#define _SERIAL_

#if defined(UBRR)||defined(UBRR0)
#define waitForUDR while( !( UCSR0A & (1<<UDRE0) ))
#endif

#if defined(UBRR0)
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UCSRC UCSR0C
#define UBRRL UBRR0L
#define UBRRH UBRR0H
#define UDR   UDR0

#define RXC  RXC0
#define TXC  TXC0
#define UDRE UDRE0
#define U2X  U2X0
#endif

void SerialInit(uint32_t baud);
void SerialWrite(char* str);
void SerialWriteBuffer(char* str);

#endif