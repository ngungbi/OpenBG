#include <avr/io.h>
#include <stdint.h>

#ifndef _BRUSHLESS_MOTOR_
#define _BRUSHLESS_MOTOR_

#if reverseRoll==1
#define ROLL_MOTOR_A 2
#define ROLL_MOTOR_B 1
#define ROLL_MOTOR_C 0
#else
#define ROLL_MOTOR_A 0
#define ROLL_MOTOR_B 1
#define ROLL_MOTOR_C 2
#endif

#if reversePitch==1
#define PITCH_MOTOR_A 2
#define PITCH_MOTOR_B 1
#define PITCH_MOTOR_C 0
#else
#define PITCH_MOTOR_A 0
#define PITCH_MOTOR_B 1
#define PITCH_MOTOR_C 2
#endif

extern volatile uint8_t loopFlag;

void initTimer(void);
void initSinArray(void);
void setMotorPos(uint8_t roll, uint8_t pitch);
void updateMotor(void);
//ISR(TIMER0_OVF_vect);

#endif