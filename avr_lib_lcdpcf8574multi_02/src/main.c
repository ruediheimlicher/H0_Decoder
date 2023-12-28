/*
lcdpcf8574multi lib sample

copyright (c) Davide Gironi, 2016

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/


#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "lcdpcf8574multi/lcdpcf8574multi.h"

#define UART_BAUD_RATE 2400
#include "uart/uart.h"

#define MAXDEVICEID 7

int main(void)
{

	//init uart
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );

    sei();

    uart_puts("starting...");

    //init lcd
    lcd_init(LCD_DISP_ON_BLINK);

	uint8_t deviceid = 0;

	//clear all the display
	for(deviceid=0; deviceid<=MAXDEVICEID; deviceid++) {
		lcd_clrscrm(deviceid);
	}

    //write the device number
	for(deviceid=0; deviceid<=MAXDEVICEID; deviceid++) {
		lcd_ledm(deviceid, 0);
		char bufdeviceid[10];
		itoa(deviceid, bufdeviceid, 10);
		lcd_gotoxym(deviceid, 0, 0);
		lcd_puts_pm(deviceid, PSTR("standy by"));
		lcd_gotoxym(deviceid, 10, 0);
		lcd_puts_pm(deviceid, PSTR("dev"));
		lcd_putsm(deviceid, bufdeviceid);
	}

	//test led on each device
	for(deviceid=0; deviceid<=MAXDEVICEID; deviceid++) {
		lcd_ledm(deviceid, 0);
		_delay_ms(100);
	}
	_delay_ms(100);
	for(deviceid=0; deviceid<=MAXDEVICEID; deviceid++) {
		lcd_ledm(deviceid, 1);
		_delay_ms(100);
	}
	_delay_ms(1000);
	for(deviceid=0; deviceid<=MAXDEVICEID; deviceid++) {
		lcd_ledm(deviceid, 0);
		_delay_ms(10);
	}
	_delay_ms(1000);

    while(1) {
    	deviceid = 0;
    	for(deviceid=0; deviceid<=MAXDEVICEID; deviceid++) {
    		//clear screen
    		lcd_clrscrm(deviceid);

            //lcd go home
            lcd_homem(deviceid);

            char bufdeviceid[10];
			itoa(deviceid, bufdeviceid, 10);

			//write device number
			lcd_gotoxym(deviceid, 10, 0);
			lcd_putcm(deviceid, 'd');
			lcd_puts_pm(deviceid, PSTR("ev"));
			lcd_putsm(deviceid, bufdeviceid);

			//test loop
			int i = 0;
			int line = 0;
			for(i=0; i<10; i++) {
				char buf[10];
				itoa(i, buf, 10);
				lcd_gotoxym(deviceid, 0, line);
				lcd_putsm(deviceid, "i= ");
				lcd_gotoxym(deviceid, 3, line);
				lcd_putsm(deviceid, buf);
				line++;
				line %= 2;
				uart_puts(buf);
				uart_puts("\r\n");
				_delay_ms(100);
			}

			lcd_clrscrm(deviceid);
			lcd_gotoxym(deviceid, 0, 0);
			lcd_puts_pm(deviceid, PSTR("standy by"));
			//write device number
			lcd_gotoxym(deviceid, 10, 0);
			lcd_putcm(deviceid, 'd');
			lcd_puts_pm(deviceid, PSTR("ev"));
			lcd_putsm(deviceid, bufdeviceid);
			lcd_gotoxym(deviceid, 0, 1);
			lcd_puts_pm(deviceid, PSTR("next display..."));
    	}
    }
}


