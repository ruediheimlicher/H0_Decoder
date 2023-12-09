/*
 * lcd.h
 *
 * Created: 20.01.2018 12:51:11
 *  Author: Ulrich
 */ 

#ifndef __LCD_H__
	#define __LCD_H__

	#include <inttypes.h>
	#include <avr/io.h>
	#include <avr/pgmspace.h>
	#include <stdlib.h>
	#include <stdarg.h>
	#include <ctype.h>
	#include <string.h>
	#include <util/delay.h>
	#include "i2c.h"

	#define LCD_I2C_ADDR	0x4E

	//Port Belegung am PCF8574 (1<<PORT)
	#define LCD_RS			(1<<0)
	#define LCD_RW			(1<<1)
	#define LCD_E			(1<<2)
	#define LCD_BACKLIGHT	(1<<3)

	#define LCD_D4			(1<<4)
	#define LCD_D5			(1<<5)
	#define LCD_D6			(1<<6)
	#define LCD_D7			(1<<7)

	#define LCD_WIDTH 20
	#define LCD_ADDR_LINE1	(0x80) // 128
	#define LCD_ADDR_LINE2	(0xC0) // 192
#define LCD_START_LINE1  0x00     /**< DDRAM address of first char of line 1 */
#define LCD_START_LINE2  0x40     /**< DDRAM address of first char of line 2 */

#define LCD_START_LINE3  0x14     /**< DDRAM address of first char of line 3 */
#define LCD_START_LINE4  0x54     /**< DDRAM address of first char of line 4 */
#define LCD_CGRAM             6      /* DB6: set CG RAM address             */
#define LCD_DDRAM             7      /* DB7: set DD RAM address             */


	void lcd_init(void);
	void lcd_clear(void);
	void lcd_home(void);
	void lcd_puts (char *str);
	void lcd_write_P (const char *Buffer,...);
void lcd_putc(char c);
   void lcd_gotoxy(uint8_t x, uint8_t y);
void lcd_putint16(uint16_t zahl);
void lcd_putint12(uint16_t zahl);
void lcd_putint2(uint8_t zahl) ;
void lcd_putint1(uint8_t zahl);
void lcd_putint(uint8_t zahl);
void lcd_clr_line(uint8_t Linie);
void lcd_write_char (char c) ;
void lcd_puts(char *str) ;
	#define lcd_write(format, args...)   lcd_write_P(PSTR(format) , ## args)

#endif
