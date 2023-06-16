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

#define INT_0_PIN   4

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
#define TEST2     2


#define STATUSPORT   PORTC
#define STATUSDDR    DDRC
#define STATUSPIN    PINC

// Pins
#define TEMPERATURPIN  0 // Diode gegen masse
#define FUNKTIONOK   2
#define ADDRESSOK    3
#define DATAOK       4

#define MOTORPORT   PORTB
#define MOTORDDR    DDRB
#define MOTORPIN    PINB

#define MOTORA_PIN       2
#define MOTORB_PIN       3

// PINs fuer device: Richtung, 
#define DEVPORT   PORTB
#define DEVDDR    DDRB
#define DEVPIN    PINB
#define MOTORDIR_PIN     1

// lokstatus-Bits



#define ADDRESSBIT   0
#define DATABIT      2
#define FUNKTIONBIT  4
#define OLDRICHTUNGBIT  5
#define RICHTUNGBIT  6
#define CHANGEBIT      7

#define TRIT0 0
#define TRIT1 1
#define TRIT2 2
#define TRIT3 3
#define TRIT4 4

//#define HI_IMPULSDAUER 10
//#define LO_IMPULSDAUER 20

#define INT0_START   0
#define INT0_END   1
#define INT0_WAIT 2

#define INT0_PAKET_A 4
#define INT0_PAKET_B 5

#define LAMPEPORT PORTD
#define LAMPEDDR  DDRD
#define LAMPEPIN  PIND
#define LEDPWM    50


#define LAMPEA_PIN          3
#define LAMPEB_PIN          4
#define MEM             0 // Eingang fuer last richtung (Kondensator)

#define ADC_PIN 0       // Temperaturmessung Diode Anode mit Pullup
#define ADC_GND_PIN 1   // Temperaturmessung Diode Kathode: LOW


#define KENNLINIE_SIZE 50
#define MAX_TEMPERATUR 60
#define MAX_ADCTEMPERATUR 224
#define MIN_ADCTEMPERATUR 194
uint8_t kennlinie[KENNLINIE_SIZE] = {233,232,231,230,229,229,228,227,226,225,225,224,223,222,221,221,220,219,218,217,217,216,215,214,213,213,212,211,210,209,209,208,207,206,205,205,204,203,202,201,201,200,199,198,197,197,196,195,194,193,193};


#endif /* defines_h */
