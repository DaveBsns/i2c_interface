#ifndef LIBRARY_H
#define LIBRARY_H

#include <stdio.h>
#include <util/delay.h>
#include <lib/hd44780pcf8574.h>
#include <compat/twi.h>

#define F_CPU 4000000UL

/* I2C clock in Hz */
#define SCL_CLOCK  100000L

void i2c_initialize(void);
unsigned char i2c_start(unsigned char address);
void blinki_blink(void);
unsigned char i2c_readAck(void);
void i2c_start_wait(unsigned char address);
unsigned char i2c_write(unsigned char data);
void i2c_stop(void);
unsigned char i2c_rep_start(unsigned char address);
unsigned char i2c_readNak(void);

#endif