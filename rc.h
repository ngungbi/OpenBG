// CH0: PC1
// CH1: PC2
// CH2: PD4
// CH3: PD7

#ifndef _RC_DECODE_
#define _RC_DECODE_

#define CH0 0
#define CH1 1
#define CH2 2
#define CH3 3

// CH0: PC3
// CH1: PC0
// CH2: PC1
// CH3: PC2

#define CH0_PCINT PCINT11
#define CH1_PCINT PCINT8
#define CH2_PCINT PCINT9
#define CH3_PCINT PCINT10

#define CH0_bit PC3
#define CH1_bit PC0
#define CH2_bit PC1
#define CH3_bit PC2

//ISR(PCINT1_vect);
extern int16_t rcInput[4];
extern volatile uint8_t timer;
void initRC();

#endif