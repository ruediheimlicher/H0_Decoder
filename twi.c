#include "twi.h"

uint16_t twicounter = 0;
void TWI_Init(void) {
	/* SCL CLOCK 
	* CPU 16Mhz CLK 100khz
	*/
   TWSR |= (1<<1);   
   _delay_ms(5);   
	TWBR = 0x32;
   _delay_ms(5);   
}

void TWI_Start(void) 
{
   twicounter=0;
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
   _delay_ms(5);   
	while(!(TWCR & (1<<TWINT)))
   {
      twicounter++;
      if (twicounter > 0xFE)
         return;
   }
}

void TWI_Transmit(unsigned char data) 
{
   twicounter=0;
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
   _delay_ms(5);   
	while(!(TWCR & (1<<TWINT)))
   {
      twicounter++;
      if (twicounter > 0xFE)
         return;
   }

}

void TWI_TransmitByAddr(unsigned char data, unsigned char addr) {
	TWI_Start();
   _delay_ms(5);   
	TWI_Transmit(addr);
   _delay_ms(5);   
	TWI_Transmit(data);
   _delay_ms(5);   
	TWI_Stop();
}

void TWI_Stop(void) {
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}
