/*
 * i2c.h
 *
 * Created: 20.01.2018 12:51:11
 *  Author: Ulrich
 */ 

#ifndef I2C_H_
	#define I2C_H_

	#include <avr/io.h>
	#include <util/delay.h>
	
	//I2C
	#define SDA_PIN			PC4 //PIN am Atmel
	#define SCL_PIN			PC5 //PIN am Atmel
	#define I2C_PORT		PORTC //PORT am Atmel
	#define SCL_CLOCK		100000UL

	void i2c_init(void);
	uint8_t i2c_start (uint8_t);
   uint8_t i2c_rep_start (uint8_t addr);
	uint8_t i2c_write_byte (uint8_t);  
	uint8_t i2c_readNak(void);
	void i2c_stop (void);

#endif /* I2C_H_ */
