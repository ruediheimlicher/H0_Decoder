/*
 * i2c.c
 *
 * Created: 20.01.2018 12:50:54
 *  Author: Ulrich
 */ 

#include "i2c.h"

void i2c_init(void){
	I2C_PORT |= (1 << SDA_PIN | 1 << SCL_PIN);			//Port Pullup
	TWCR = 0;
	TWSR = 0;
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
	_delay_ms(50);
}

//***************************************************************************************
uint8_t i2c_start (uint8_t addr)
{
	uint16_t timeout = 0;

    TWCR = (1 << TWINT | 1 << TWSTA | 1 << TWEN);
    while(!(TWCR & (1<<TWINT))){
	    if((timeout++) > 1000) return 1;
	}
    TWDR = addr;
    TWCR = (1 << TWINT | 1 << TWEN);
	timeout = 0;
    while(!(TWCR & (1<<TWINT))){
	    if((timeout++) > 1000) return 1;
    }
	return 0;
}

//***************************************************************************************
uint8_t i2c_rep_start (uint8_t addr)
{
   return i2c_start(addr);
   
}
//***************************************************************************************
uint8_t i2c_write_byte (uint8_t byte){  
	uint16_t timeout = 0;
   uint8_t   twst;
    TWDR = byte;
    TWCR = (1 << TWINT | 1 << TWEN);
    while(!(TWCR & (1<<TWINT)))
    {
		if((timeout++) > 1000) return 1;
	}
   //TW_STATUS = TWSR & 0xF8
   // check value of TWI Status Register. Mask prescaler bits
   twst = TW_STATUS & 0xF8;

   if( twst != TW_MT_DATA_ACK) return twst;

	return 0;
}

//***************************************************************************************
uint8_t i2c_write_string(const char *s)
{
   register char c;
   uint8_t returnwert = 0;
   while ( (c = *s++) ) 
   {
      returnwert = i2c_write_byte(c);
      if(returnwert)
      {
         return returnwert;
      }
   }
   return 0;
}



//***************************************************************************************
uint8_t i2c_readNak(void)
{
	uint16_t timeout = 0;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){
		if((timeout++) > 1000) return 0;
	}
    return TWDR;
}

//***************************************************************************************
void i2c_stop (void){
    TWCR = (1 << TWINT | 1 << TWSTO | 1 << TWEN);
	TWCR = 0;
}
