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
#include <util/twi.h>
//#include "i2c.c"


#include "lcd.c"

//#include "twislave.c"
#include "lcd_a.c"

#include "lcd_I2C.h"

//#include "lcd_o.h"
//https://github.com/Sylaina/oled-display/tree/master

#include "adc.c"
#define TW_STATUS   (TWSR & TW_STATUS_MASK)


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



volatile uint16_t MEMBuffer = 0;

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

volatile uint8_t     oldspeed = 0;
volatile uint8_t     newspeed = 0;
volatile uint8_t     startspeed = 0; // Anlaufimpuls
volatile uint8_t     speedcode = 0;
volatile int8_t      speedintervall = 0;


// ***
volatile uint8_t   speed = 0;

volatile uint8_t   oldfunktion = 0;
volatile uint8_t   funktion = 0;
volatile uint8_t   deffunktion = 0;
volatile uint8_t   waitcounter = 0;
volatile uint8_t   richtungcounter = 0; // delay fuer Richtungsimpuls

volatile uint8_t     pwmpin = MOTORA_PIN;           // Motor PWM
volatile uint8_t     richtungpin = MOTORB_PIN;      // Motor Richtung

volatile uint8_t	Potwert=45;
			//	Zaehler fuer richtige Impulsdauer
//uint8_t				Servoposition[]={23,33,42,50,60};
// Richtung invertiert
//volatile uint8_t				Servoposition[]={60,50,42,33,23};

volatile uint16_t	taktimpuls=0;

volatile uint16_t   motorPWM=0;

volatile uint8_t ledstatus = 0;

uint8_t EEMEM WDT_ErrCount;	// Akkumulierte WDT Restart Events



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

volatile uint8_t   lastDIR =  0;
uint8_t loopledtakt = 0x40;
uint8_t refreshtakt = 0x45;
uint16_t speedchangetakt = 0x350; // takt fuer beschleunigen/bremsen


volatile uint8_t loktyptable[4];

volatile uint8_t speedindex = 7;

volatile uint8_t   maxspeed =  252;//prov.

uint16_t loopcounter0;
uint16_t loopcounter1;


void slaveinit(void)
{
   
 	OSZIPORT |= (1<<OSZIA);	//Pin 6 von PORT D als Ausgang fuer OSZI A
	OSZIDDR |= (1<<OSZIA);	//Pin 7 von PORT D als Ausgang fuer SOSZI B
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
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

   
   TESTDDR |= (1<<TEST0); // test0
   TESTPORT |= (1<<TEST0); // HI
   
  
   
   MOTORDDR |= (1<<MOTORA_PIN);  // Motor A
   MOTORPORT &= ~(1<<MOTORA_PIN); // LO

   MOTORDDR |= (1<<MOTORB_PIN);  // Motor B
   MOTORPORT &= ~(1<<MOTORB_PIN); // LO

   
   //MOTORDDR |= (1<<MOTORDIR);  // Motor Dir
   //MOTORPORT &= ~(1<<MOTORDIR); // LO


   LAMPEDDR |= (1<<LAMPE);  // Data ist OK
   LAMPEPORT &= ~(1<<LAMPE); // LO

   initADC(MEM);
   
   // TWI
   
   DDRC |= (1<<5);   //Pin 0 von PORT C als Ausgang (SCL)
   PORTC |= (1<<5);   //   ON
   DDRC |= (1<<4);   //Pin 1 von PORT C als Ausgang (SDA)
   PORTC |= (1<<4);   //   ON

      
}




void int0_init(void)
{
   EICRA |= (1 << ISC00) | (1 << ISC01);  // Trigger interrupt on rising edge
   EIMSK |= (1 << INT0);  // Enable external interrupt INT0

 //  INT0status |= (1<<INT0_RISING);
   INT0status = 0;
}


void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
//	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2A |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC
   TCCR2B |= (1<<CS00); // no prescaler
	//OC2 akt


	TIMSK2 |= (1<<OCIE2A);			//CTC Interrupt aktivieren
   TIMSK2 |=(1<<TOIE2);        //interrupt on Compare Match A
	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2A = wert;					//Setzen des Compare Registers auf HIimpulsdauer
} 

#pragma mark INT0
ISR(INT0_vect) 
{
   OSZIATOG;
   if (INT0status == 0) // neue Daten beginnen
   {
      INT0status |= (1<<INT0_START);
      INT0status |= (1<<INT0_WAIT); // delay, um Wert des Eingangs zum richtigen Zeitpunkt zu messen
      
      INT0status |= (1<<INT0_PAKET_A); // erstes Paket lesen
      OSZIPORT &= ~(1<<PAKETA); 
      //TESTPORT &= ~(1<<TEST2);
      OSZIALO; 
      
      
  //    OSZIPORT &= ~(1<<INT_0);
      
      pausecounter = 0; // pausen detektieren, reset fuer jedes HI
      abstandcounter = 0;// zweites Paket detektieren, 
      
      waitcounter = 0;
      tritposition = 0;
      lokadresse = 0;
      lokdata = 0;
      funktion = 0;
      
  //    rawdataA = 0;
  //    rawdataB = 0;

      
//      HIimpulsdauer = 0;
      
      // A85
 
      
      // A85
   } 
   
   else // Data im Gang, neuer Interrupt
   {
      INT0status |= (1<<INT0_WAIT);
      
      
  //    OSZIPORT &= ~(1<<INT_0);
      pausecounter = 0;
      abstandcounter = 0; 
      waitcounter = 0;
      OSZIALO;
      //   INT0status |= (1<<INT0_RISING); // wait for next rise
      //    MCUCR = (1<<ISC00) |(1<<ISC01); // raise int0 on rising edge
   }
}

// MARK: ISR Timer2

ISR(TIMER2_COMPA_vect) // Schaltet Impuls an SERVOPIN0 aus
{
   //return;
   //OSZIATOG;
   if (speed)
   {
      motorPWM++;
   }
   if ((motorPWM > speed) || (speed == 0)) // Impulszeit abgelaufen oder speed ist 0
   {
      //MOTORPORT |= (1<<MOTORA_PIN); // OFF, Motor ist active LO
      MOTORPORT |= (1<<pwmpin); 
   }
   
   if (motorPWM >= 254) //ON, neuer Motorimpuls
   {
      //MOTORPORT &= ~(1<<MOTORA_PIN);
      MOTORPORT &= ~(1<<pwmpin);
      motorPWM = 0;
   }
   
   
#pragma mark TIMER0 INT0
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
               
               INT0status &= ~(1<<INT0_PAKET_A); // Bit fuer erstes Paket weg
               INT0status |= (1<<INT0_PAKET_B); // Bit fuer zweites Paket setzen
               tritposition = 0;
            }
            else if (INT0status & (1<<INT0_PAKET_B)) // zweites Paket, Werte testen
            {
               
               
   // MARK:  EQUAL
               if (lokadresseA && ((rawfunktionA == rawfunktionB) && (rawdataA == rawdataB) && (lokadresseA == lokadresseB))) // Lokadresse > 0 und Lokadresse und Data OK
               {
                  if (lokadresseB == LOK_ADRESSE)
                  {
                     // Daten uebernehmen
                      
                     lokstatus |= (1<<ADDRESSBIT);
                     deflokadresse = lokadresseB;
                     //deffunktion = (rawdataB & 0x03); // bit 0,1 funktion als eigene var
                     deffunktion = rawfunktionB;
                     //uint8_t speedcode = 0;
                     
                     if (deffunktion)
                     {
                        lokstatus |= (1<<FUNKTIONBIT);
                        ledstatus |= (1<<LED_CHANGEBIT); // change setzen
                        //LAMPEPORT |= (1<<LAMPE);
                        
                        OSZIBLO;
                     }
                     else
                     {
                        lokstatus &= ~(1<<FUNKTIONBIT);
                        ledstatus |= (1<<LED_CHANGEBIT); // led-change setzen
                        //LAMPEPORT &= ~(1<<LAMPE);
                        OSZIBHI;
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
                           //MOTORPORT ^= (1<<MOTORDIR); // Richtung umpolen
                           
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
                        //speed = speedlookup[speedcode];
                        if(speedcode && (speedcode < 2) && !(lokstatus & (1<<STARTBIT))  && !(lokstatus & (1<<RUNBIT))) // noch nicht gesetzt
                        {
                           //startspeed = speedlookuptable[speedindex][speedcode] + 1; // kleine Zugabe
                           startspeed = speedlookup[speedcode] + 1; // kleine Zugabe
                           
                           lokstatus |= (1<<STARTBIT);
                        }

                       oldspeed = speed; // behalten
                    
                       speedintervall = (newspeed - speed)>>2; // 4 teile
                        
                       
                        //newspeed = speedlookuptable[speedindex][speedcode]; // zielwert
                        newspeed = speedlookup[speedcode]; // zielwert
                       
                        
                        if(speedcode > 0)
                        {
                           lokstatus |= (1<<RUNBIT); // lok in bewegung
                        }
                        else
                        {
                           lokstatus &= ~(1<<RUNBIT); // lok steht still
                        }

                     }
                     //MEMBuffer=(readKanal(MEM));
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
         //     OSZIPORT |= (1<<PAKETA); 
         //    OSZIPORT &= ~(1<<PAKETB);   
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
   
//   int0_init();
	
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	_delay_ms(100);
	lcd_cls();
	lcd_puts("H0-Decoder A328");
	
   
	//timer0();
   
//   timer2(4);
	
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

   //lcd_gotoxy(0,2);
   
   //lcd_gotoxy(0,3);
   //lcd_puts("data ");
   
   
 //  TWI_Init();
   _delay_ms(100);
   LCD_init();
   uint8_t counter = 0;
   uint8_t byte = 0;
   uint8_t lcdcounter = 0;
	while (1)
   {   
      // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      wdt_reset();
      {
         //PORTA &= ~(1<<PA4); // LED on
         if(lokstatus & (1<<FUNKTIONBIT))
         {
            /*
             if(dimmcounter == 3)
             {
             LAMPEPORT |= (1<<ledonpin); // Lampe-PWM  ON
             
             }
             dimmcounter++;
             if(dimmcounter > 32)
             {
             LAMPEPORT &= ~(1<<ledonpin); // Lampe-PWM  OFF
             dimmcounter = 0;
             }
             */
         }
         
         //continue;
         
         loopcount1++;
         if (loopcount1 >= speedchangetakt)
         {
   
            //Output to LCD
             lcdcounter++;
            //LOOPLEDPORT ^= (1<<LOOPLED); // Kontrolle lastDIR
            loopcount1 = 0;
            //OSZIATOG;
            
            
            // speed var
            if((newspeed > oldspeed)) // beschleunigen, speedintervall positiv
            {
               if(speed < (newspeed - speedintervall))
               {
                  if((startspeed > speed) && (lokstatus & (1<<STARTBIT))) // Startimpuls
                  {
                     speed = startspeed;
                     lokstatus &= ~(1<<STARTBIT);
                  }
                  
                  speed += speedintervall;
               }
               else 
               {
                  speed = newspeed;
               }
            }
            else if((newspeed < oldspeed)) // bremsen, speedintervall negativ
            {
               if((speed > newspeed) && ((speed + 2*speedintervall) > 0))
               {
                  speed += 2*speedintervall;
               }
               else 
               {
                  speed = newspeed;
               }
            }
            // end speed var
            
         } // loopcount1 >= speedchangetakt
         
      }// Source OK
      
      
      loopcount0++;
      if (loopcount0>=refreshtakt)
      {
         //OSZIATOG;
         //LOOPLEDPORT ^= (1<<LOOPLED); 
         
         loopcount0=0;
         loopcounter1++;
             if (loopcounter1 > MAXLOOP1)
             {
                loopcounter1=0;
                LOOPLEDPORT ^= (1<<LOOPLED);
                counter++;
                
   
  
                lcdcounter++;

                //lcd_gotoxy(16,1);
                //lcd_putint(counter);
               // LCD_setPosition(0,1);
               // LCD_sendString("hallo");
             }

         if(lokstatus & (1<<LOK_CHANGEBIT)) // Motor-Pins tauschen
         {
            if(pwmpin == MOTORA_PIN)
            {
               pwmpin = MOTORB_PIN;
               richtungpin = MOTORA_PIN;
               //ledonpin = LAMPEB_PIN;
              // ledoffpin = LAMPEA_PIN;
               
               if(lokstatus & (1<<FUNKTIONBIT))
               {
                  
                  LAMPEPORT &= ~(1<<LAMPEB_PIN); // Lampe B OFF
                  LAMPEPORT |= (1<<LAMPEA_PIN); // Lampe A OFF
               }
               else
               {
                  // beide lampen OFF
                  LAMPEPORT &= ~(1<<LAMPEB_PIN); // Lampe B OFF
                  LAMPEPORT &= ~(1<<LAMPEA_PIN); // Lampe A OFF

               }

            }
            else // auch default
            {
               pwmpin = MOTORA_PIN;
               richtungpin = MOTORB_PIN;
               //ledonpin = LAMPEA_PIN;
               //ledoffpin = LAMPEB_PIN;
               if(lokstatus & (1<<FUNKTIONBIT))
               {
                  
                  LAMPEPORT |= (1<<LAMPEB_PIN); // Lampe B OFF
                  LAMPEPORT &= ~(1<<LAMPEA_PIN); // Lampe A OFF
               }
               else 
               {
                  // beide lampen OFF
                  LAMPEPORT &= ~(1<<LAMPEB_PIN); // Lampe B OFF
                  LAMPEPORT &= ~(1<<LAMPEA_PIN); // Lampe A OFF
               }

            }
            MOTORPORT |= (1<<richtungpin); // Richtung setzen
            
            lokstatus &= ~(1<<LOK_CHANGEBIT);
            
         } // if changebit
         
         
         
         // Lampen einstellen
         if(ledstatus & (1<<LED_CHANGEBIT))
         {
            if(lokstatus & (1<<FUNKTIONBIT))
            {
            //   LAMPEPORT |= (1<<ledonpin); // Lampe  ON
             //  LAMPEPORT &= ~(1<<ledoffpin); // // Lampe  OFF
               
            }
            else
            {
               // beide lampen OFF
            //   LAMPEPORT &= ~(1<<LAMPEB_PIN); // Lampe B OFF
            //   LAMPEPORT &= ~(1<<LAMPEA_PIN); // Lampe A OFF
            }
            ledstatus &= ~(1<<LED_CHANGEBIT);
         }

         
         
         
     /*        
         if (deflokadresse == LOK_ADRESSE)
         {
            //OSZIATOG;
         }
         else
         {
            //OSZIAHI;
         }
      */   
         
      }  // loopcount0>=refreshtakt
      
      //OSZIAHI;
   }//while


// return 0;
}
