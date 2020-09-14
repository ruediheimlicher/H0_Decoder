//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include "defines.h"

//#include "twislave.c"
#include "lcd.c"

#include "adc.c"

//***********************************
						
uint8_t  LOK_ADRESSE = 0xCC; //	11001100	Trinär
//									
//***********************************

/*
 commands
 LO     0x0202  // 0000001000000010
 OPEN   0x02FE  // 0000001011111110
 HI     0xFEFE  // 1111111011111110
 */


#define OUTPORT	PORTD		// Ausgang fuer Motor

#define INPORT   PORTD  // Input signal auf INT0
#define INPIN   PIND  // Input signal

#define DATAPIN  2 


#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR      DDRD
#define LOOPLED			5

#define INT0_RISING	   0
#define INT0_FALLING		1


void lcd_puts(const char *s);
//volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[buffer_size];

//uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit


//uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
//uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset


volatile uint8_t	INT0status=0x00;				
volatile uint8_t	signalstatus=0x00; // status TRIT
volatile uint8_t  pausestatus=0x00;


volatile uint8_t   address=0x00; 
volatile uint8_t   data=0x00;   


volatile uint16_t	HICounter=0;					//	Abstand der Impulspakete
volatile uint16_t	LOCounter=0x00;				//	Zaehler fuer Pause
volatile uint16_t	LOimpulsdauer=0x00;				//	Zaehler fuer Impuls



volatile uint16_t	HIimpulsdauer=00;			//	Dauer des LOimpulsdaueres Definitiv
volatile uint8_t	HIimpulsdauerPuffer=22;		//	Puffer fuer HIimpulsdauer
volatile uint8_t	HIimpulsdauerSpeicher=0;		//	Speicher  fuer HIimpulsdauer

volatile uint8_t   LOimpulsdauerOK=0;   

volatile uint16_t   pausecounter = 0; //  neue ädaten detektieren
volatile uint16_t   abstandcounter = 0; // zweites Paket detektieren

volatile uint8_t   tritposition = 0; // nummer des trit im Paket
volatile uint8_t   lokadresse = 0;
volatile uint8_t    lokstatus=0x00; // Funktion, Richtung

volatile uint8_t   oldlokdata = 0;
volatile uint8_t   lokdata = 0;

volatile uint8_t   speed = 0;

volatile uint8_t   oldfunktion = 0;
volatile uint8_t   funktion = 0;

volatile uint8_t   waitcounter = 0;


volatile uint8_t	Potwert=45;
			//	Zaehler fuer richtige Impulsdauer
uint8_t				LOimpulsdauerNullpunkt=23;
uint8_t				LOimpulsdauerSchrittweite=10;
//uint8_t				Servoposition[]={23,33,42,50,60};
// Richtung invertiert
volatile uint8_t				Servoposition[]={60,50,42,33,23};

volatile uint16_t	taktimpuls=0;

volatile uint16_t   motorPWM=0;


uint8_t EEMEM WDT_ErrCount;	// Akkumulierte WDT Restart Events







void slaveinit(void)
{
 	OSZIPORT |= (1<<OSZIA);	//Pin 6 von PORT D als Ausgang fuer OSZI A
	OSZIDDR |= (1<<OSZIA);	//Pin 7 von PORT D als Ausgang fuer SOSZI B
   OSZIPORT |= (1<<OSZIB);   //Pin 6 von PORT D als Ausgang fuer OSZI A
   OSZIDDR |= (1<<OSZIB);   //Pin 7 von PORT D als Ausgang fuer SOSZI B
	
	LOOPLEDPORT |=(1<<LOOPLED);
   LOOPLEDDDR |=(1<<LOOPLED);
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

   DDRB |= (1<<0); // Ausgang Oszi
   DDRB |= (1<<1); // Ausgang 
   PORTB |= (1<<1); // HI
	
   STATUSDDR |= (1<<ADDRESSOK); // Adresse ist OK
   STATUSPORT &= ~(1<<ADDRESSOK); // LO
   STATUSDDR |= (1<<DATAOK);  // Data ist OK
	STATUSPORT &= ~(1<<DATAOK); // LO

   STATUSDDR |= (1<<FUNKTIONOK);  // Data ist OK
   STATUSPORT &= ~(1<<FUNKTIONOK); // LO

   MOTORDDR |= (1<<MOTOROUT);  // Motor PWM
   MOTORPORT &= ~(1<<MOTOROUT); // LO

   
}




void int0_init(void)
{
   MCUCR = (1<<ISC00 | (1<<ISC01)); // raise int0 on rising edge
   GICR |= (1<<INT0); // enable external int0
 //  INT0status |= (1<<INT0_RISING);
   INT0status = 0;
}



void timer0 (void) 
{ 
// Timer fuer Exp
TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
//	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	
//   TCCR0 |= (1<<CS00); // no prescaler
//Timer fuer 	
   
//	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
	
	TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	TCNT0 = 0x00;					//Rücksetzen des Timers
  
}

/*
ISR(TIMER0_COMP_vect) 
{
   
}
*/
void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
//	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC
   TCCR2 |= (1<<CS00); // no prescaler
	//OC2 akt
//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<OCIE2);			//CTC Interrupt aktivieren

	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2 = wert;					//Setzen des Compare Registers auf HIimpulsdauer
} 

ISR(INT0_vect) 
{
   //OSZIATOG;
   if (INT0status == 0) // neue Daten beginnen
   {
      INT0status |= (1<<INT0_START);
      INT0status |= (1<<INT0_WAIT);
      OSZIALO; 
      OSZIBLO;
      
      PORTB &= ~(1<<1);
      
      pausecounter = 0; // pausen detektieren, reset fuer jedes HI
      abstandcounter = 0;// zweites Paket detektieren, 
      
      waitcounter = 0;
      tritposition = 0;
      lokadresse = 0;
      lokdata = 0;
      funktion = 0;
      
      HIimpulsdauer = 0;
      
      HICounter = 0;
      //  irq_falling_edge=0; // wait for next fall
      
      //    INT0status &= ~(1<<INT0_RISING);
      //MCUCR = (1<<ISC00); // raise int0 on falling edge
      //OSZIAHI;
   } 
   
   else // Data im Gang, neuer Interrupt
   {
      INT0status |= (1<<INT0_WAIT);
      PORTB &= ~(1<<1);
      pausecounter = 0;
      abstandcounter = 0; 
      waitcounter = 0;
      OSZIALO;
      //    HIimpulsdauer = HICounter;
      //   INT0status |= (1<<INT0_RISING); // wait for next rise
      //    MCUCR = (1<<ISC00) |(1<<ISC01); // raise int0 on rising edge
   }
}



ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
   
   //MOTORPORT ^= (1<<MOTOROUT);
   motorPWM++;
   if (motorPWM > speed)
   {
   //   MOTORPORT ^= (1<<MOTOROUT);
      MOTORPORT &= ~(1<<MOTOROUT);
   }
   if (motorPWM > 0xFF)
   {
      motorPWM = 0;
      MOTORPORT |= (1<<MOTOROUT);
    }
   
   if (INT0status & (1<<INT0_WAIT))
   {
      waitcounter++;
      if (waitcounter > 2)
      {
         INT0status &= ~(1<<INT0_WAIT);
         
         PORTB |= (1<<1);
         //uint8_t pos = tritposition & 0x01;
         if (tritposition < 8) // Adresse)
         {
            if (INPIN & (1<<DATAPIN)) // Pin HI, 
            {
               lokadresse |= (1<<tritposition); // bit ist 1
            }
            else // 
            {
               lokadresse &= ~(1<<tritposition); // bit ist 0
            }
            //tritposition ++;
         }
         
         else 
         {
            
            if (lokadresse == LOK_ADRESSE) // weitere Daten lesen ab tritposition 4
            {
               lokstatus |= (1<<ADDRESSBIT);
               
               STATUSPORT |= (1<<ADDRESSOK); // LED ON
               
               if (tritposition < 10)
               {
                  if (INPIN & (1<<DATAPIN)) // Pin HI, 
                  {
                     funktion |= (1<<(tritposition - 8)); // bit ist 1
                  }
                  else // 
                  {
                     funktion &= ~(1<<(tritposition - 8)); // bit ist 0
                  }

               }
               else
               {
                  if (INPIN & (1<<DATAPIN)) // Pin HI, 
                  {
                     lokdata |= (1<<(tritposition - 10)); // bit ist 1
                  }
                  else // 
                  {
                     lokdata &= ~(1<<(tritposition - 10)); // bit ist 0
                  }
               }
            }
            else // Lokadresse falsch, Daten verwerfen
            {
               lokstatus &= ~(1<<ADDRESSBIT); // Adresse falsch
                
               STATUSPORT &= ~(1<<ADDRESSOK); // LED OFF
          //     INT0status == 0;
          //     return;
            }
         }
         
         if (tritposition < 18)
         {
            tritposition ++;
         }
         else
         {
            if (funktion == oldfunktion) // funktion gleich wie vorher
            {
               STATUSPORT |= (1<<FUNKTIONOK);
               lokstatus ^= (1<<FUNKTIONBIT); // funktion zweimal OK
            }
            else
            {
               STATUSPORT &= ~(1<<FUNKTIONOK); // Funktion ungleich, warten bis ident
               //oldfunktion = funktion;
            }
           
            if (lokdata == oldlokdata)
            {
               STATUSPORT |= (1<<DATAOK);
               lokstatus |= (1<<DATABIT); // data zweimal OK
               
               if (lokdata == 0x03) // Richtung toggle
               {
                  lokstatus ^= (1<<RICHTUNGBIT);
                  MOTORPORT ^= (1<<MOTORDIR); // Richtung umpolen
               }
               else if (lokdata == 0)
               {
                  speed = 0;
               }
               else //if (lokdata > 3)
               {
                  switch (lokdata)
                  {
                  case 0x0C:
                     speed = 1;
                     break;
                  case 0x0F:
                     speed = 2;
                     break;
                  case 0x30:
                     speed = 3;
                     break;
                  case 0x33:
                     speed = 4;
                     break;
                  case 0x3C:
                     speed = 5;
                     break;
                  case 0x3F:
                     speed = 6;
                     break;
                  case 0xC0:
                     speed = 7;
                     break;
                  case 0xC3:
                     speed = 8;
                     break;
                  case 0xCC:
                     speed = 9;
                     break;
                  case 0xCF:
                     speed = 10;
                     break;
                  case 0xF0:
                     speed = 11;
                     break;
                  case 0xF3:
                     speed = 12;
                     break;
                  case 0xFC:
                     speed = 13;
                     break;
                  case 0xFF:
                     speed = 14;
                     break;
                     
                  }
                  speed *= 18;
                                    
               }
            }
            else
            {
               STATUSPORT &= ~(1<<DATAOK);
               lokstatus &= ~(1<<DATABIT); // data falsch
               oldlokdata = lokdata;
            }
            
            //tritposition = 0;
            //INT0status == 0;
         }
         
      }
   }
   //PORTB ^= (1<<0);
   
   //PORTB |= (1<<1);
   if (INPIN & (1<<DATAPIN)) // Pin HI, input   im Gang
   {
      HIimpulsdauer++; // zaehlen
   }
   else  // LO, input fertig, Bilanz
   {
      if (abstandcounter < 20)
      {
         abstandcounter++;
       }
      else //if (abstandcounter )
      {
         abstandcounter = 0;
         OSZIAHI;
      }
      
      if (pausecounter < 140)
      {
         pausecounter ++; // pausencounter incrementieren
         
      }
      else 
      {
        OSZIBHI; //pause detektiert
         pausecounter = 0;
         INT0status = 0; //Neue Daten abwarten
         return;
      }
      if (HIimpulsdauer < 4) // kurz
      {
         //PORTB |= (1<<1);
         //OSZIAHI;
         HIimpulsdauer =0;
      }
      else if (HIimpulsdauer > 8) // lang
      {
         //OSZIBHI;
         HIimpulsdauer = 0;
          HIimpulsdauer = 0;
      }
      
   } // input LO
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
//		TCCR2=0;
//		SERVOPORT &= ~(1<<SERVOPIN0);//	SERVOPIN0 zuruecksetzen
//		SERVOPORT &= ~(1<<5);// Kontrolle auf PIN D5 OFF
		//delay_ms(800);
		//lcd_clr_line(1);
		
}




void main (void) 
{
	slaveinit();
   
   int0_init();
	//PORT2 |=(1<<PC4);
	//PORTC |=(1<<PC5);
	//uint16_t ADC_Wert= readKanal(0);
	//sei();
	
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	_delay_ms(1000);
	lcd_cls();
	lcd_puts("H0-Interface\0");
	

	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
		
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	uint8_t Schalterposition=0;
   
	//timer0();
   
   timer2(4);
	
	//initADC(TASTATURPIN);
	
	//wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	


	uint8_t twierrcount=0;
	
	
	_delay_ms(800);
	//uint8_t tempWDT_Count=eeprom_read_byte(&WDT_ErrCount);


	//	Zaehler fuer Wartezeit nach dem Start
	uint16_t startdelay0=0x00AF;
	//uint16_t startdelay1=0;

	//Zaehler fuer Zeit von (SDA || SCL = LO)
	uint16_t twi_LO_count0=0;
	uint16_t twi_LO_count1=0;
	uint8_t SlaveStatus=0x00; //status

	//Zaehler fuer Zeit von (SDA && SCL = HI)
	uint16_t twi_HI_count0=0;

	//uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0); 
	//uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);
	/*
	eepromWDT_Count0: Zaehler der wdt-Resets mit restart. 
	
	eepromWDT_Count1: Zaehler fuer neuen wdt-Reset. Wenn wdt anspricht, wird der Zaheler erhoeht.
	Beim Restart wird bei anhaltendem LO auf SDA oder SCL gewartet.
	Wenn SCL und SDA beide HI sind, wird der Zaehler auf den Wert von eepromWDT_Count0 gesetzt 
	und der TWI-Slave gestartet.
	
	*/
/*
	// Ankuendigen, dass schon ein wdt erfolgte
	if (!(eepromWDT_Count0==eepromWDT_Count1))
	{
		lcd_gotoxy(18,1);
		lcd_puts("W\0");

	}
 */
   oldfunktion = 0x03; // 0x02
   oldlokdata = 0xCC;

   sei();

   lcd_gotoxy(0,1);
   lcd_puts("ADR ");
   lcd_puthex(LOK_ADRESSE);
   
   lcd_gotoxy(0,2);
   lcd_puts("adr ");

   lcd_gotoxy(0,3);
   lcd_puts("data ");



	while (1)
	{	
		wdt_reset();
				//Blinkanzeige
		loopcount0++;
		if (loopcount0==0x8FFF)
		{
			loopcount0=0;
			LOOPLEDPORT ^=(1<<LOOPLED);
         
         lcd_gotoxy(4,2);
 			lcd_puthex(lokadresse);
         
			//delay_ms(10);
         /*
         if (lokadresse == LOK_ADRESSE)
         {
            cli();
            lcd_gotoxy(8,2);
            lcd_puts("adr OK ");
            lcd_puthex(lokadresse);
            sei();
         }
       else 
       {
          cli();
          lcd_gotoxy(8,2);
          lcd_puts("adr Err ");
          lcd_puthex(lokadresse);
          sei();
       }
          */
         
         lcd_gotoxy(4,3);
         lcd_puthex(lokdata);
         lcd_putc(' ');
         lcd_putint(speed);
         
         
         uint16_t tempBuffer=0;
//			initADC(VORLAUF);
//			tempBuffer=readKanal(VORLAUF);
//			lcd_gotoxy(0,1);
//			lcd_puts("V\0");
			//lcd_putint(tempBuffer>>2);
//			lcd_put_tempbis99(tempBuffer>>2);
			
		}

		
			
		
		
	
			
		
		
	}//while


// return 0;
}
