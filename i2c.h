#include <avr/io.h>
#include <avr/pgmspace.h>
//#include <stdint.h>

#ifndef _I2C_H_
#define _I2C_H_

//#define INTERNAL_I2C_PULLUPS

#define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
#define I2C_SPEED 400000L 

#define ACK 1
#define NAK 0

extern uint16_t i2c_errors_count;

void i2c_init(void);
void i2c_start(uint8_t address);
void i2c_stop(void);
void i2c_write(uint8_t data );
uint8_t i2c_read(uint8_t ack);
uint8_t i2c_readAck();
uint8_t i2c_readNak(void);
void waitTransmissionI2C(void);
size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size);
size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size);

//void i2c_getSixRawADC(uint8_t add, uint8_t reg);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);

uint8_t i2c_errors(void);
void i2c_error_reset(void);

#endif