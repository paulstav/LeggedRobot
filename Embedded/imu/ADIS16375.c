#include <stdint.h>
#include <stdbool.h>

#include "ADIS16375.h"
#include "spi.h"

#include "utils/uartstdio.h"

#define debug_printf UARTprintf

void ADIS16375_wake(ADIS16375 *this){
    this->_cs(LOW);
    this->_delay_cycle(80);
    this->_cs(HIGH);
}

double ADIS16375_signed_double(unsigned char nbits, unsigned int num){
  unsigned int mask, padding;
  // select correct mask
  mask = 1 << (nbits -1);
  
  // if MSB is 1, then number is negative, so invert it and add one
  // if MSB is 0, then just return the number 
  return (num & mask)?( -1.0 * (~(num | 0xFF << nbits)  + 1) ):( 1.0 * num );
}

unsigned int ADIS16375_twos_comp(double num){
  unsigned int raw;
  
  if(num < 0){
    raw = ~((unsigned int)(-num) - 1);
  }else{
    raw = (unsigned int)num;
  }
  return raw;
}

unsigned int ADIS16375_device_id(ADIS16375 *this){
  // Read 14 bits from the PROD_ID register
  return ADIS16375_read(this, 16, PROD_ID);
}

void ADIS16375_Init(ADIS16375 *this, void (* _delay_cycle)(unsigned long), void (* _cs)(unsigned char), void (* _spi_setup)(), unsigned short (* _spi_write)(unsigned short))
{
	this->_cs = _cs;
	this->_spi_setup = _spi_setup;
	this->_spi_write = _spi_write;
	this->_delay_cycle = _delay_cycle;

	// Begin SPI
	this->_spi_setup();
	// Initialize CS pin to be high
	this->_cs(HIGH);
	// Wake device up, incase it's sleeping
	ADIS16375_wake(this);
}


void ADIS16375_burst_read(ADIS16375 *this)
{
  unsigned char bits[11] = {12, 14, 14, 14, 14, 14, 14, 12, 12, 12, 12};
  unsigned char offset_bin[11] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  int i;
  unsigned char upper,lower,mask;
  unsigned int raw;
  double scale[11] = {2.418e-3, 0.05, 0.05, 0.05, 1, 1, 1, 0.136, 0.136, 0.136, 805.8e-6};
  double add[11] = {0, 0, 0, 0, 0, 0, 0, 25, 25, 25, -1.65};
  
  
  this->_cs(LOW);
  
  this->_spi_write(0x3E);
  this->_spi_write(0x00);
  this->_delay_cycle(8);
  
  for(i = 0; i < 11; i++)
  {
    upper = this->_spi_write(0x00);
    lower = this->_spi_write(0x00);
    mask = 0xFF >> (16 - bits[i]);
    raw = ( ( upper & mask ) << 8 ) | ( lower );
    this->sensor[i] = ( ( offset_bin[i] )?( raw ):( ADIS16375_signed_double( bits[i], raw ) ) ) * scale[i] + add[i];
    this->_delay_cycle(8);
  }
   
  this->_cs(HIGH);
}


unsigned int ADIS16375_read(ADIS16375 *this, unsigned char nbits, unsigned short reg){
  // initialize variables
  unsigned short rshort, mask, measure; 

  // Get upper and lower unsigned chars
  this->_cs(LOW);
  this->_spi_write(reg);
  this->_cs(HIGH);
  this->_delay_cycle(1);
  this->_cs(LOW);
  rshort = this->_spi_write(0x0000);
  this->_cs(HIGH);

  // calculate mask
  mask = 0xFFFF >> (16 - nbits);
  

  // Combine upper and lower, and return
  measure= rshort & mask;

  return measure;
}

void ADIS16375_write(ADIS16375 *this, unsigned char reg, unsigned int value){
  // set lower byte
  this->_cs(LOW);
  this->_spi_write(reg | 0x80);
  this->_spi_write(value & 0x00FF);
  this->_cs(HIGH);
  this->_delay_cycle(8);
  // set upper byte
  this->_cs(LOW);
  this->_spi_write( (reg + 1) | 0x80 );
  this->_spi_write( value >> 8 );
  this->_cs(HIGH);
}