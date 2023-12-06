/*
 * LCD_HD44780.c
 *
 * Created: 20.01.2018 12:51:11
 *  Author: Ulrich
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"
#include "i2c.h"

int main(void)
{
	uint8_t byte = 0;
	i2c_init();
	lcd_init();
	
	//PullUp der Tasten PCF8574 P4-P7
	i2c_start(0x40);
	i2c_byte(0xF0);
	i2c_stop();
			
	
	lcd_clear();
	lcd_write("TEST:");
	
	while(1){
		
		//Einlesen der Tasten PCF8574 P4-P7
		i2c_start(0x41);
		byte =((~i2c_readNak())>>4)&0x0F;
		i2c_stop();
				
		//Output to LCD
		lcd_home();
		lcd_write("Byte %2i",byte);
		_delay_ms(200);
	}
}