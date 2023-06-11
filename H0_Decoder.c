//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include "defines.h"
#include <stdint.h>

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


//#define OUTPORT	PORTD		// Ausgang fuer Motor

//#define INPORT   PORTD  // Input signal auf INT0
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





//volatile uint16_t	HIimpulsdauer=0;			//	Dauer des LOimpulsdaueres Definitiv

volatile uint8_t	HIimpulsdauerPuffer=22;		//	Puffer fuer HIimpulsdauer
volatile uint8_t	HIimpulsdauerSpeicher=0;		//	Speicher  fuer HIimpulsdauer

volatile uint8_t   LOimpulsdauerOK=0;   

volatile uint8_t   pausecounter = 0; //  neue ädaten detektieren
volatile uint8_t   abstandcounter = 0; // zweites Paket detektieren

volatile uint8_t   tritposition = 0; // nummer des trit im Paket
volatile uint8_t   lokadresse = 0;

volatile uint8_t   lokadresseA = 0;
volatile uint8_t   lokadresseB = 0;

volatile uint8_t   deflokadresse = 0;

volatile uint8_t   lokstatus=0x00; // Funktion, Richtung

volatile uint8_t   oldlokdata = 0;
volatile uint8_t   lokdata = 0;
volatile uint8_t   deflokdata = 0;
//volatile uint16_t   newlokdata = 0;

volatile uint8_t   rawdataA = 0;
volatile uint8_t   rawdataB = 0;
//volatile uint32_t   oldrawdata = 0;

// ***
volatile uint8_t   rawfunktionA = 0;
volatile uint8_t   rawfunktionB = 0;


// ***
volatile uint8_t   speed = 0; // Motor

volatile uint8_t   dimm = 0; // LED dimmwert
volatile uint8_t   ledpwm = 0; // LED PWM

volatile uint8_t   oldfunktion = 0;
volatile uint8_t   funktion = 0;
volatile uint8_t   deffunktion = 0;
volatile uint8_t   waitcounter = 0;
volatile uint8_t   richtungcounter = 0; // delay fuer Richtungsimpuls

volatile uint8_t	Potwert=45;
			//	Zaehler fuer richtige Impulsdauer
//uint8_t				Servoposition[]={23,33,42,50,60};
// Richtung invertiert
//volatile uint8_t				Servoposition[]={60,50,42,33,23};

volatile uint16_t	taktimpuls=0;

volatile uint16_t   motorPWM=0;

uint8_t lastdir = 0;

uint8_t EEMEM WDT_ErrCount;	// Akkumulierte WDT Restart Events

volatile uint8_t   taskcounter = 0;

// linear
//volatile uint8_t   speedlookup[15] = {0,18,36,54,72,90,108,126,144,162,180,198,216,234,252};


//volatile uint8_t   speedlookup[15] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140};
// linear 100
//volatile uint8_t   speedlookup[15] = {0,7,14,21,28,35,42,50,57,64,71,78,85,92,100};

// linear 80
//volatile uint8_t   speedlookup[15] = {0,5,11,17,22,28,34,40,45,51,57,62,68,74,80};

// linear mit offset 30
//volatile uint8_t   speedlookup[15] = {0,33,37,40,44,47,51,55,58,62,65,69,72,76,80};

// linear mit offset 22
//volatile uint8_t   speedlookup[15] = {0,26,30,34,38,42,46,51,55,59,63,67,71,75,80};

// linear mit offset 20
volatile uint8_t   speedlookup[15] = {0,24,28,32,37,41,45,50,54,58,62,67,71,75,80};

// logarithmisch 180
//volatile uint8_t   speedlookup[14] = {0,46,73,92,106,119,129,138,146,153,159,165,170,175,180};

//log 160
//volatile uint8_t   speedlookup[14] = {0,40,64,81,95,105,114,122,129,136,141,146,151,155,160};

// log 140
//volatile uint8_t   speedlookup[14] = {0,35,56,71,83,92,100,107,113,119,123,128,132,136,140};

// log 100
//volatile uint8_t   speedlookup[14] = {0,25,40,51,59,66,71,76,81,85,88,91,94,97,100};

uint8_t loopledtakt = 0x40;


uint16_t adctemperatur = 0;

void slaveinit(void)
{
   STATUSDDR &= ~(1<<MEM);  // Eingang Mem-Status (last richtung)
   
   
   OSZIPORT |= (1<<OSZIA);   //Pin 6 von PORT D als Ausgang fuer OSZI A
   OSZIDDR |= (1<<OSZIA);   //Pin 7 von PORT D als Ausgang fuer SOSZI B
   OSZIPORT |= (1<<OSZIB);   //Pin 6 von PORT D als Ausgang fuer OSZI A
   OSZIDDR |= (1<<OSZIB);   //Pin 7 von PORT D als Ausgang fuer SOSZI B

   //OSZIPORT |= (1<<INT_0);   //
   //OSZIDDR |= (1<<INT_0);   //Pin 7 von PORT D als Ausgang fuer SOSZI B

   OSZIDDR |= (1<<PAKETA);
   OSZIPORT |= (1<<PAKETA);   //PAKETA
      //
   OSZIDDR |= (1<<PAKETB); 
   OSZIPORT |= (1<<PAKETB);   //PAKETB
   

   
   
   LOOPLEDPORT |=(1<<LOOPLED);
   LOOPLEDDDR |=(1<<LOOPLED);
   

   //LCD
   LCD_DDR |= (1<<LCD_RSDS_PIN);   //Pin 5 von PORT B als Ausgang fuer LCD
    LCD_DDR |= (1<<LCD_ENABLE_PIN);   //Pin 6 von PORT B als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_CLOCK_PIN);   //Pin 7 von PORT B als Ausgang fuer LCD

   
   TESTDDR |= (1<<TEST0); // test0
   TESTPORT |= (1<<TEST0); // HI
   
   
   STATUSDDR |= (1<<ADDRESSOK); // Adresse ist OK
   STATUSPORT &= ~(1<<ADDRESSOK); // LO
   //STATUSDDR |= (1<<DATAOK);  // Data ist OK
   //STATUSPORT &= ~(1<<DATAOK); // LO

   STATUSDDR |= (1<<FUNKTIONOK);  // Data ist OK
   STATUSPORT &= ~(1<<FUNKTIONOK); // LO

   
  

   
   MOTORDDR |= (1<<MOTORA_PIN);  // Motor A
   MOTORPORT &= ~(1<<MOTORA_PIN); // LO

   MOTORDDR |= (1<<MOTORB_PIN);  // Motor B
   MOTORPORT &= ~(1<<MOTORB_PIN); // LO

   
   DEVDDR |= (1<<MOTORDIR_PIN);  // Motor Dir
   DEVPORT &= ~(1<<MOTORDIR_PIN); // LO

 
   LAMPEDDR |= (1<<LAMPEA_PIN);  // Lampe A
   LAMPEPORT &= ~(1<<LAMPEA_PIN); // OFF

   LAMPEDDR |= (1<<LAMPEB_PIN);  // Lampe A
   LAMPEPORT &= ~(1<<LAMPEB_PIN); // OFF

   
   //initADC(MEM);
   

  
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
   TCCR2 |= (1<<CS00);     // no prescaler
	//OC2 akt
//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2); 			//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<OCIE2);			//CTC Interrupt aktivieren
  // TIMSK |=(1<<TOIE2);  
	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2 = wert;					//Setzen des Compare Registers auf HIimpulsdauer
} 

// MARK: INT0
ISR(INT0_vect) 
{
   //OSZIATOG;
   
   if (INT0status == 0) // neue Daten beginnen
   {
      INT0status |= (1<<INT0_START);
      INT0status |= (1<<INT0_WAIT); // delay, um Wert des Eingangs zum richtigen Zeitpunkt zu messen
      
      INT0status |= (1<<INT0_PAKET_A); // erstes Paket lesen
      OSZIPORT &= ~(1<<PAKETA); 
      //TESTPORT &= ~(1<<TEST2);
      //OSZIALO; 
      //OSZIBLO;
      
      
      pausecounter = 0; // pausen detektieren, reset fuer jedes HI
      abstandcounter = 0;// zweites Paket detektieren, 
      
      waitcounter = 0;
      tritposition = 0;
      lokadresse = 0;
      lokdata = 0;
      funktion = 0;
      
       
      // A85
      
      
      // A85
   } 
   
   else // Data im Gang, neuer Interrupt
   {
      INT0status |= (1<<INT0_WAIT);
      
      pausecounter = 0;
      abstandcounter = 0; 
      waitcounter = 0;
      //OSZIALO;
   }
}

// MARK: ISR Timer2

ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
   //OSZIBTOG;
    
   if (speed)
   {
      motorPWM++;
   }
   
   if ((motorPWM > speed) || (speed == 0)) // Impulszeit abgelaufen oder speed ist 0
   {
      
      {
         MOTORPORT |= (1<<MOTORA_PIN); // MOTORA_PIN HI
         MOTORPORT |= (1<<MOTORB_PIN); // MOTORB_PIN HI
      }
      
   }
   
   if (motorPWM >= 254) //ON, neuer Motorimpuls
   {
      if(DEVPORT & (1<<MOTORDIR_PIN)) // Richtungbit gesetzt
      {
         MOTORPORT |= (1<<MOTORA_PIN);
         MOTORPORT &= ~(1<<MOTORB_PIN);// MOTORB_PIN PWM, OFF
      }
      else 
      {
         MOTORPORT |= (1<<MOTORB_PIN);
         MOTORPORT &= ~(1<<MOTORA_PIN);// MOTORA_PIN PWM, OFF        
      }
      
      motorPWM = 0;
   }
   
   
// MARK: TIMER0 INT0
   if (INT0status & (1<<INT0_WAIT))
   {
      waitcounter++;
      if (waitcounter > 2)
      {
         INT0status &= ~(1<<INT0_WAIT);
         if (INT0status & (1<<INT0_PAKET_A))
         {
            if (tritposition < 8) // Adresse)
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseA |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseA &= ~(1<<tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10)
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionA |= (1<<(tritposition-8)); // bit ist 1
               }
               else // 
               {
                  rawfunktionA &= ~(1<<(tritposition-8)); // bit ist 0
               }
               
            }
            
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataA |= (1<<(tritposition-10)); // bit ist 1
               }
               else // 
               {
                  rawdataA &= ~(1<<(tritposition-10)); // bit ist 0
               }
            }
         }
         
         if (INT0status & (1<<INT0_PAKET_B))
         {
            if (tritposition < 8) // Adresse)
            {
               
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseB |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseB &= ~(1<<tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10) // bit 8,9: funktion
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionB |= (1<<(tritposition-8)); // bit ist 1
               }
               else // 
               {
                  rawfunktionB &= ~(1<<(tritposition-8)); // bit ist 0
               }
               
            }
            
            
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataB |= (1<<(tritposition-10)); // bit ist 1
               }
               else 
               {
                  rawdataB &= ~(1<<(tritposition-10)); // bit ist 0
               }
            }
            
            if (!(lokadresseB == LOK_ADRESSE))
            {
               
            }
         }
         
         // Paket anzeigen
         if (INT0status & (1<<INT0_PAKET_B))
         {
            //           TESTPORT |= (1<<TEST2);
         }
         if (INT0status & (1<<INT0_PAKET_A))
         {
            //           TESTPORT |= (1<<TEST1);
         }
         
         if (tritposition < 17)
         {
            tritposition ++;
         }
         else // Paket gelesen
         {
            // Paket A?
            if (INT0status & (1<<INT0_PAKET_A)) // erstes Paket, Werte speichern
            {
               oldfunktion = funktion;
               
               INT0status &= ~(1<<INT0_PAKET_A);   // Bit fuer erstes Paket weg
               INT0status |= (1<<INT0_PAKET_B);    // Bit fuer zweites Paket setzen
               tritposition = 0;
            }
            else if (INT0status & (1<<INT0_PAKET_B)) // zweites Paket, Werte testen
            {
               
               
// MARK: EQUAL
               if (lokadresseA && ((rawfunktionA == rawfunktionB) && (rawdataA == rawdataB) && (lokadresseA == lokadresseB))) // Lokadresse > 0 und Lokadresse und Data OK
               {
                  if (lokadresseB == LOK_ADRESSE)
                  {
                     // Daten uebernehmen
                     
                     lokstatus |= (1<<ADDRESSBIT); // mein Paket
                     
                     deflokadresse = lokadresseB;
                     //deffunktion = (rawdataB & 0x03); // bit 0,1 funktion als eigene var
                     deffunktion = rawfunktionB;
                     uint8_t speedcode = 0;
                     
                     if (deffunktion)
                     {
                        lokstatus |= (1<<FUNKTIONBIT);
                        
                     }
                     else
                     {
                        lokstatus &= ~(1<<FUNKTIONBIT);
                     }
                     
                     
                     for (uint8_t i=0;i<8;i++)
                     {
                        //if ((rawdataB & (1<<(2+i))))
                        if ((rawdataB & (1<<i)))
                        {
                           deflokdata |= (1<<i);
                        }
                        else 
                        {
                           deflokdata &= ~(1<<i);
                        }
                     }
                     
                     // Richtung
                     if (deflokdata == 0x03) // Wert 1, Richtung togglen
                     {
                        if (!(lokstatus & (1<<RICHTUNGBIT)))
                        {
                           lokstatus |= (1<<RICHTUNGBIT);
                           richtungcounter = 0xFF;
                           speed = 0;
                           DEVPORT ^= (1<<MOTORDIR_PIN); // Richtung umpolen
                           lokstatus |= (1<<CHANGEBIT);
                           taskcounter++;
                           if (lokstatus & (1<<FUNKTIONBIT))
                           {
                              taskcounter += 10;
                           }
                        }
                     }
                     else 
                     {  
                        lokstatus &= ~(1<<RICHTUNGBIT); 
// MARK: speed                           
                        switch (deflokdata)
                        {
                           case 0:
                              speedcode = 0;
                              break;
                           case 0x0C:
                              speedcode = 1;
                              break;
                           case 0x0F:
                              speedcode = 2;
                              break;
                           case 0x30:
                              speedcode = 3;
                              break;
                           case 0x33:
                              speedcode = 4;
                              break;
                           case 0x3C:
                              speedcode = 5;
                              break;
                           case 0x3F:
                              speedcode = 6;
                              break;
                           case 0xC0:
                              speedcode = 7;
                              break;
                           case 0xC3:
                              speedcode = 8;
                              break;
                           case 0xCC:
                              speedcode = 9;
                              break;
                           case 0xCF:
                              speedcode = 10;
                              break;
                           case 0xF0:
                              speedcode = 11;
                              break;
                           case 0xF3:
                              speedcode = 12;
                              break;
                           case 0xFC:
                              speedcode = 13;
                              break;
                           case 0xFF:
                              speedcode = 14;
                              break;
                           default:
                              speedcode = 0;
                              break;
                              
                        }
                        speed = speedlookup[speedcode];
                     }
                  }
                  else 
                  {
                     // aussteigen
                     //deflokdata = 0x17;
                     INT0status = 0;
                     return;
                  }
               }
               else 
               {
                  lokstatus &= ~(1<<ADDRESSBIT);
                  // aussteigen
                  //deflokdata = 0x13;
                  INT0status = 0;
                  return;
                  
               }
               
               INT0status |= (1<<INT0_END);
               //     OSZIPORT |= (1<<PAKETB);
               if (INT0status & (1<<INT0_PAKET_B))
               {
                  //               TESTPORT |= (1<<TEST2);
               }
            } // End Paket B
         }
         
      } // waitcounter > 2
   } // if INT0_WAIT
   
   if (INPIN & (1<<DATAPIN)) // Pin HI, input   im Gang
   {
      //      HIimpulsdauer++; // zaehlen
   }
   else  // LO, input fertig, Bilanz
   {
      if (abstandcounter < 20)
      {
         abstandcounter++;
      }
      else //if (abstandcounter ) // Paket 2
      {
         abstandcounter = 0;
         //   OSZIAHI;
      }
      
      if (pausecounter < 120)
      {
         pausecounter ++; // pausencounter incrementieren
      }
      else 
      {
         //OSZIBHI; //pause detektiert
         pausecounter = 0;
         INT0status = 0; //Neue Daten abwarten
         return;
      }
      
   } // input LO
}




void main (void) 
{
   
   
	slaveinit();
   lastdir = PINC;
   int0_init();
	
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	_delay_ms(1000);
	lcd_cls();
	lcd_puts("H0-Decoder A8_2");
	
   
	//timer0();
   
   timer2(4);
	
	//initADC(TASTATURPIN);
	
	
	//uint16_t loopcount0=0;
   uint8_t loopcount0=0;
   uint8_t loopcount1=0;

	
	_delay_ms(800);

   
   oldfunktion = 0x03; // 0x02
   oldlokdata = 0xCE;

   sei();

   lcd_gotoxy(0,1);
   lcd_puts("ADR ");
   lcd_puthex(LOK_ADRESSE);
   
   //lcd_gotoxy(0,2);
   lcd_puts(" adrIN");

   

	while (1)
   {	
      wdt_reset();
      //Blinkanzeige
      loopcount0++;
      
      if (lokstatus & (1<<CHANGEBIT))
      {
         lcd_gotoxy(12,2);
         lcd_putint(taskcounter);
         lokstatus &= ~(1<<CHANGEBIT);
      }
      
      
      if (loopcount0 >= loopledtakt)
      {
         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         
         
         
         loopcount1++;
         if (loopcount1 >= 4)
         {
            loopcount1 = 0;
            
            adctemperatur = readKanal(ADC_PIN);
            //temperatur = 8;
            lcd_gotoxy(0,2);
            lcd_putint12(adctemperatur);
            if (adctemperatur < MIN_ADCTEMPERATUR )
            {
               adctemperatur = MIN_ADCTEMPERATUR;
            }
            uint8_t temperatur = 0;
            for (uint8_t index = 0;index < KENNLINIE_SIZE; index++)
            {
               if (adctemperatur > kennlinie[index] )
               {
                  temperatur = index + 15;
                  break;
               }
            }
            lcd_gotoxy(8,2);
            lcd_putint2(temperatur);
            
         }   
         lcd_gotoxy(13,1);
         lcd_puthex(deflokadresse);
         
          
         if (deflokadresse == LOK_ADRESSE)
         {
            
            lcd_gotoxy(17,1);
            lcd_puts("OK ");
            //lcd_puthex(deflokadresse);
            
         }
         else 
         {
            lcd_gotoxy(17,1);
            lcd_puts("Err");
            //lcd_puthex(lokadresse);
            
         }
         
// MARK: LCD
         //uint8_t a = ((deflokdata >>4)& 0xFF);
         /*
          lcd_gotoxy(0,2);
          lcd_puthex(a);
          lcd_putc(' ');
          a = ((deflokdata >>8)& 0xFF);
          lcd_puthex(a);
          lcd_putc(' ');
          a = ((deflokdata >>12)& 0xFF);
          lcd_puthex(a);
          lcd_putc(' ');
          lcd_puthex(newlokdata);
          lcd_putc(' ');
          
          lcd_putc('*');
          lcd_puthex(oldlokdata);
          lcd_putc(' ');
          
          //lcd_puthex(rawdataA>>4);
          */
         
         lcd_gotoxy(0,2);
         // uint32_t b = 0xFFFFFFFF;
         /*
          for (uint8_t i=0;i<18;i++)
          {
          if (deflokdata & (1<<i))
          {
          lcd_putc('1');
          }
          else
          {
          lcd_putc('0');
          }
          
          }
          */
         
         // Lampen einstellen
         if(DEVPIN & (1<<MOTORDIR_PIN))
         {
            if (lokstatus & (1<<FUNKTIONBIT))
            {
               LAMPEPORT |=(1<<LAMPEA_PIN);
               LAMPEPORT &= ~(1<<LAMPEB_PIN);
            }
            else
            {
               LAMPEPORT &= ~(1<<LAMPEA_PIN);
               LAMPEPORT &= ~(1<<LAMPEB_PIN);
            }
         }
         else
         {
            if (lokstatus & (1<<FUNKTIONBIT))
            {
               LAMPEPORT |=(1<<LAMPEB_PIN);
               LAMPEPORT &= ~(1<<LAMPEA_PIN);
            }
            else 
            {
               LAMPEPORT &= ~(1<<LAMPEA_PIN);
               LAMPEPORT &= ~(1<<LAMPEB_PIN);
            }
         }// if (DEVPIN & (1<<MOTORDIR_PIN))

         
         lcd_gotoxy(0,3);
         lcd_puthex(deflokdata);
         lcd_putc(' ');
         
         if (lokstatus & (1<<FUNKTIONBIT))
         //if ( STATUSPIN & (1<<LAMPEA_PIN))
         {
            lcd_putc('1');
            
         }
         else
         {
            lcd_putc('0');
         }
         lcd_putc(' ');
         lcd_putint(speed);
         lcd_putc(' ');
         //if (lokstatus &(1<<RICHTUNGBIT))
         if (DEVPIN & (1<<MOTORDIR_PIN))
         {
            
            lcd_putc('V');
            //LAMPEPORT |=(1<<LAMPEA_PIN);
            //LAMPEPORT &=(1<<LAMPEB_PIN);
         }
         else
         {
            lcd_putc('R');
            //LAMPEPORT |=(1<<LAMPEB_PIN);
            //LAMPEPORT &=(1<<LAMPEA_PIN);
         }
         
         //lastdir = PINC & (1<<MEM); // aktueller Wert
         lcd_putc(' ');
         lcd_putint(lokstatus);


         lcd_putc(' ');
         lcd_putint(lastdir);
         
          
      } // loopcount0 >= loopledtakt
      
   }//while


// return 0;
}
