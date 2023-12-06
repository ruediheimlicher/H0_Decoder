//
//  defines.h
//  H0_Decoder
//
//  Created by Ruedi Heimlicher on 11.09.2020.
//

#ifndef defines_h
#define defines_h

#define SHORT 0 // Abstand im doppelpaket
#define LONG 2 // Abstand zwischen Daten

#define OSZIPORT   PORTD      // Ausgang fuer Servo
#define OSZIDDR   DDRD



#define OSZIA 6            // 
#define OSZIB 7            // Impuls für Servo

#define INT_0   4

#define PAKETA   0
#define PAKETB   1

#define OSZIALO OSZIPORT &= ~(1<<OSZIA)
#define OSZIAHI OSZIPORT |= (1<<OSZIA)
#define OSZIATOG OSZIPORT ^= (1<<OSZIA)

#define OSZIBLO OSZIPORT &= ~(1<<OSZIB)
#define OSZIBHI OSZIPORT |= (1<<OSZIB)
#define OSZIBTOG OSZIPORT ^= (1<<OSZIB)

#define TESTPORT        PORTB
#define TESTDDR        DDRB

#define TEST0     0
#define TEST1     1



#define I2CPORT   PORTC
#define I2CDDR    DDRC


// Pins
#define FUNKTIONOK   2
#define ADDRESSOK    3
#define DATAOK       4

#define MOTORPORT   PORTB
#define MOTORDDR    DDRB
#define MOTORPIN    PINB

#define MOTORA_PIN      2
#define MOTORB_PIN      3

// lokstatus-Bits
#define FUNKTION     0
#define OLDFUNKTION  1
#define FUNKTIONSTATUS 2




#define ADDRESSBIT   0
#define STARTBIT        1 // Startimpuls
#define DATABIT      2
#define PROGBIT         3 // Programmiermodus
#define FUNKTIONBIT  4
#define RUNBIT  5
#define RICHTUNGBIT  6
#define LOK_CHANGEBIT       7  


#define STARTDELAY      100

#define STARTWAIT 100


#define TRIT0 0
#define TRIT1 1
#define TRIT2 2
#define TRIT3 3
#define TRIT4 4

#define HI_IMPULSDAUER 10
#define LO_IMPULSDAUER 20

#define INT0_START   0
#define INT0_END   1
#define INT0_WAIT 2

#define INT0_PAKET_A 4
#define INT0_PAKET_B 5


#define LAMPEPORT PORTD
#define LAMPEDDR  DDRD
#define LAMPEPIN   PIND

#define LED_CHANGEBIT       7  
#define LAMPEA_PIN      1 
#define LAMPEB_PIN      0


#define LAMPE         3
#define MEM           6 // Eingang fuer last richtung (Kondensator)

#define LAMPEMAX 0x40 // 50%

#define FIRSTRUN_END 80

#define MAXLOOP0 0x0AFE
#define MAXLOOP1 0x0AFF

#endif /* defines_h */
