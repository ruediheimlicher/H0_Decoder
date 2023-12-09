/*
 * lcd.c
 *
 * Created: 20.01.2018 12:51:11
 *  Author: Ulrich
 */ 

#include "lcd.h"

volatile unsigned char char_counter = 0;

// output data to lcd
void lcd_out (unsigned char c) {
	i2c_start(LCD_I2C_ADDR);
	i2c_write_byte(c | LCD_BACKLIGHT);
	i2c_stop();
}

//***************************************************************************************
// CD = Command or Data
void lcd_nibble_out(unsigned char c, unsigned char cd) {
	unsigned char out = 0;
	unsigned char rs = 0;
	
	if(cd) rs=LCD_RS;

	//4 upper bits
	if(c & (1<<4)) out |= LCD_D4;
	if(c & (1<<5)) out |= LCD_D5;
	if(c & (1<<6)) out |= LCD_D6;
	if(c & (1<<7)) out |= LCD_D7;
	lcd_out(out | rs | LCD_E);
	_delay_ms(1);
	lcd_out(out | rs);

	//4 lower bits
	out=0;
	if(c & (1<<0)) out |= LCD_D4;
	if(c & (1<<1)) out |= LCD_D5;
	if(c & (1<<2)) out |= LCD_D6;
	if(c & (1<<3)) out |= LCD_D7;
	lcd_out(out | rs | LCD_E);
	_delay_ms(1);
	lcd_out(out | rs );
}

//***************************************************************************************
// clear LCD
void lcd_clear() {
	lcd_nibble_out(0x01, 0); // clear display
	lcd_nibble_out(0x80, 0);
	char_counter = 0;
}


//***************************************************************************************
// LCD home
void lcd_home() {
	lcd_nibble_out(0x80, 0);
	char_counter = 0;
}

//***************************************************************************************
// Init LCD
void lcd_init() {
	unsigned char loop=3;

	while(loop--){
		lcd_out(LCD_D4 | LCD_D5 | LCD_E);
		_delay_ms(10);
		lcd_out(LCD_D4 | LCD_D5);
		_delay_ms(100);
	}

	// 4 bit mode
	lcd_out(LCD_D5 | LCD_E);
	_delay_ms(10);
	lcd_out(LCD_D5);
	_delay_ms(10);

	lcd_nibble_out(0x28, 0);
	lcd_nibble_out(0x0C, 0);

	lcd_clear();
}
//***************************************************************************************
//***************************************************************************************
void lcd_gotoxy(uint8_t x, uint8_t y)
{
   OSZIALO;
   switch (y)
   {
      case 0:
         lcd_nibble_out((1<<LCD_DDRAM)+LCD_START_LINE1+x,0);
         break;
      case 1:
         lcd_nibble_out((1<<LCD_DDRAM)+LCD_START_LINE2+x,0);
         break;
      case 2:
         lcd_nibble_out((1<<LCD_DDRAM)+LCD_START_LINE3+x,0);
      
         break;
      case 3:
         lcd_nibble_out((1<<LCD_DDRAM)+LCD_START_LINE4+x,0);
       
         break;
         
         
   }//switch
   OSZIAHI;
}
//***************************************************************************************
void lcd_putint(uint8_t zahl)
{
char string[4];
  int8_t i;                             // schleifenzähler
 
  string[3]='\0';                       // String Terminator
  for(i=2; i>=0; i--) 
  {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
lcd_puts(string);
}

//***************************************************************************************
void lcd_putint1(uint8_t zahl)   //einstellige Zahl
{
   //char string[5];
   char string[2];
   zahl%=10;                        //  hinterste Stelle
   string[1]='\0';                     // String Terminator
   string[0]=zahl +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
   lcd_puts(string);
}

//***************************************************************************************
void lcd_putint2(uint8_t zahl)   //zweistellige Zahl
{
   char string[3];
   int8_t i;                        // Schleifenzähler
   zahl%=100;                        // 2 hintere Stelle
   //  string[4]='\0';                     // String Terminator
   string[2]='\0';                     // String Terminator
   for(i=1; i>=0; i--) {
      string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
   lcd_puts(string);
}

//***************************************************************************************
void lcd_putint12(uint16_t zahl)
{
   char string[5];
   int8_t i;                             // schleifenzähler
   
   string[4]='\0';                       // String Terminator
   for(i=3; i>=0; i--)
   {
      string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
   lcd_puts(string);
}

//***************************************************************************************
void lcd_putint16(uint16_t zahl)
{
char string[8];
  int8_t i;                             // schleifenzähler
 
  string[7]='\0';                       // String Terminator
  for(i=6; i>=0; i--) 
  {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
lcd_puts(string);
}

//***************************************************************************************
void lcd_putc(char c)
{
   lcd_nibble_out(c,0);
 
}

//***************************************************************************************
// Linie Loeschen
void lcd_clr_line(uint8_t Linie)
{
   lcd_gotoxy(0,Linie);
   char string[LCD_WIDTH+1];
     int8_t i;                             // schleifenzähler
    
     string[LCD_WIDTH]='\0';                       // String Terminator
     for(i=LCD_WIDTH-1; i>=0; i--) 
     {
       string[i]=(' ') ;         // Modulo rechnen, dann den ASCII-Code von '0' addieren
       
     }
   OSZIBLO;
   lcd_puts(string);
   OSZIBHI;
   lcd_gotoxy(0,Linie);
   
   
   
   
/*
   _delay_ms(5);
   uint8_t i=0;
   for (i=0;i<LCD_WIDTH;i++)
   {
      lcd_nibble_out(' ',0);
      _delay_ms(100);
   }
   _delay_ms(5);
   lcd_gotoxy(0,Linie);
   _delay_ms(5);
 */
}   // Linie Loeschen

//***************************************************************************************

//***************************************************************************************

//***************************************************************************************

//***************************************************************************************

//***************************************************************************************


void lcd_write_char (char c) 
{
   /*
	if(char_counter == LCD_WIDTH) lcd_nibble_out(LCD_ADDR_LINE2,0);
	if(char_counter == (LCD_WIDTH*2)){
		lcd_nibble_out(LCD_ADDR_LINE1,0);
		char_counter = 0;
	}
	char_counter++;
   */
   
	lcd_nibble_out(c, 1);
}

//***************************************************************************************
void lcd_puts(char *str) 
{
	while (*str != 0)
   {
      /*
		if(char_counter == LCD_WIDTH) lcd_nibble_out(LCD_ADDR_LINE2,0);
		if(char_counter == (LCD_WIDTH*2)){
			lcd_nibble_out(LCD_ADDR_LINE1,0);
			char_counter = 0;
		}
       */
		char_counter++;
		lcd_nibble_out(*str++, 1);
	}
}

//***************************************************************************************
void lcd_write_P (const char *Buffer,...)
{
	va_list ap;
	va_start (ap, Buffer);	
	
	int format_flag;
	char str_buffer[10];
	char str_null_buffer[10];
	char move = 0;
	char Base = 0;
	int tmp = 0;
	char by;
	char *ptr;
		
	//Ausgabe der Zeichen
    for(;;){
		by = pgm_read_byte(Buffer++);
		if(by==0) break; // end of format string
            
		if (by == '%'){
            by = pgm_read_byte(Buffer++);
			if (isdigit(by)>0){                   
 				str_null_buffer[0] = by;
				str_null_buffer[1] = '\0';
				move = atoi(str_null_buffer);
                by = pgm_read_byte(Buffer++);
				}

			switch (by){
                case 's':
                    ptr = va_arg(ap,char *);
                    while(*ptr) { lcd_write_char(*ptr++); }
                    break;
				case 'b':
					Base = 2;
					goto ConversionLoop;
				case 'c':
					//Int to char
					format_flag = va_arg(ap,int);
					lcd_write_char (format_flag++);
					break;
				case 'i':
					Base = 10;
					goto ConversionLoop;
				case 'o':
					Base = 8;
					goto ConversionLoop;
				case 'x':
					Base = 16;
					//****************************
					ConversionLoop:
					//****************************
					itoa(va_arg(ap,int),str_buffer,Base);
					int b=0;
					while (str_buffer[b++] != 0){};
					b--;
					if (b<move){
						move -=b;
						for (tmp = 0;tmp<move;tmp++){
							str_null_buffer[tmp] = '0';
							}
						//tmp ++;
						str_null_buffer[tmp] = '\0';
						strcat(str_null_buffer,str_buffer);
						strcpy(str_buffer,str_null_buffer);
						}
					lcd_puts (str_buffer);
					move =0;
					break;
				}
		}else{
			lcd_write_char (by);	
		}
	}
	va_end(ap);
}
