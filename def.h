#define setBit(_port,_bit) (_port)|=(1<<(_bit))
#define clearBit(_port,_bit) (_port)&=~(1<<(_bit))

#define LED_ON  setBit(PORTB,0)
#define LED_OFF clearBit(PORTB,0)
#define LED_TOGGLE if (PORTB&(1<<0)) LED_OFF; else LED_ON;

#define eepromStartAddress 0x01