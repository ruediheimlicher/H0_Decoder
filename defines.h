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

#define OSZIALO OSZIPORT &= ~(1<<OSZIA)
#define OSZIAHI OSZIPORT |= (1<<OSZIA)
#define OSZIATOG OSZIPORT ^= (1<<OSZIA)

#define OSZIBLO OSZIPORT &= ~(1<<OSZIB)
#define OSZIBHI OSZIPORT |= (1<<OSZIB)
#define OSZIBTOG OSZIPORT ^= (1<<OSZIB)

#define STATUSPORT   PORTC
#define STATUSDDR    DDRC

#define FUNKTIONOK   2
#define ADDRESSOK    3
#define DATAOK       4

#define MOTORPORT   PORTB
#define MOTORDDR    DDRB

#define MOTOROUT      3
#define MOTORDIR      4

// lokstatus
#define FUNKTION     0
#define OLDFUNKTION  1
#define FUNKTIONSTATUS 2



#define ADDRESSBIT   0
#define DATABIT      2
#define FUNKTIONBIT  4
#define RICHTUNGBIT  6

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

#endif /* defines_h */
