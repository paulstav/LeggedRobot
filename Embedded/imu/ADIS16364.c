
#include <stdint.h>
#include <stdbool.h>

#include "ADIS16364.h"
#include "spi.h"

#include "utils/uartstdio.h"

#define debug_printf UARTprintf

void ADIS16364_Init(ADIS16364 *this, void (* _delay_cycle)(unsigned long), void (* _cs)(unsigned char), void (* _spi_setup)(), unsigned char (* _spi_write)(unsigned char))
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
	ADIS16364_wake(this);
}


void ADIS16364_burst_read(ADIS16364 *this)
{
  double sensor[11];
  unsigned char bits[11] = {12, 14, 14, 14, 14, 14, 14, 12, 12, 12, 12};
  unsigned char offset_bin[11] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  int i;
  unsigned char upper,lower,mask;
  int16_t raw;
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
    sensor[i] = ( ( offset_bin[i] )?( raw ):( ADIS16364_signed_double( bits[i], raw ) ) ) * scale[i] + add[i];
    this->_delay_cycle(8);
  }
   
  this->_cs(HIGH);
}

int16_t ADIS16364_read(ADIS16364 *this, unsigned char nbits, unsigned char reg){
  // initialize variables
  unsigned char upper, lower, mask; 
  int16_t measure;

  // Get upper and lower unsigned chars
  this->_cs(LOW);
  this->_delay_cycle(2);
  this->_spi_write(reg);
  this->_spi_write(0x00);
  this->_delay_cycle(2);
  this->_cs(HIGH);
  this->_delay_cycle(370);
  this->_cs(LOW);
  this->_delay_cycle(2);
  upper = this->_spi_write(0x00);
  lower = this->_spi_write(0x00);
  this->_delay_cycle(2);
  this->_cs(HIGH);

  // calculate mask
  mask = 0xFF >> (16 - nbits);
  

  // Combine upper and lower, and return
  measure=( ( upper & mask ) << 8 ) | ( lower );

  return measure;
}

void ADIS16364_write(ADIS16364 *this, unsigned char reg, int16_t value){
  // set lower byte
  this->_cs(LOW);
  this->_delay_cycle(2);
  this->_spi_write(reg | 0x80);
  this->_spi_write(value & 0x00FF);
  this->_delay_cycle(2);
  this->_cs(HIGH);
  this->_delay_cycle(370);
  // set upper byte
  this->_cs(LOW);
  this->_delay_cycle(2);
  this->_spi_write( (reg + 1) | 0x80 );
  this->_spi_write( value >> 8 );
  this->_delay_cycle(2);
  this->_cs(HIGH);
}

int16_t ADIS16364_signed_double(unsigned char nbits, int16_t num){
  int16_t mask, padding, neg, sense=0,raw;
  int16_t measure = 0;
  
  raw = num;
  
  // select correct mask
  //mask = 1 << (nbits -1);
  
  neg = 1 << (nbits-1);

  mask = 0xFFFF >> (16-nbits);  //16-nbits
  raw = (raw & mask);
  
  if (raw & neg)
  {
    sense = -1*( ~ (raw | (0xFFFFF << nbits))+1);
  }
  else
  {
    sense = raw;
  }
  
  measure = sense;
  
  return measure;
  
  // if MSB is 1, then number is negative, so invert it and add one
  // if MSB is 0, then just return the number 
  //return (num & mask)?( -1.0 * (~(num | 0xFFFF << nbits)  + 1) ):( 1.0 * num );
}

int16_t ADIS16364_twos_comp(double num){
  int16_t raw;
  
  if(num < 0){
    raw = ~((int16_t)(-num) - 1);
  }else{
    raw = (int16_t)num;
  }
  return raw;
}

// Read accelerometer data
void ADIS16364_readAccData(ADIS16364 *this, int16_t* accX, int16_t* accY, int16_t* accZ)
{
  int16_t tmp;
  
  *accX = ADIS16364_signed_double(14,ADIS16364_read(this, 14, XACCL_OUT));
  tmp = ADIS16364_signed_double(14,ADIS16364_read(this, 14, YACCL_OUT));
  *accY = ADIS16364_signed_double(14,ADIS16364_read(this, 14, YACCL_OUT));
  tmp = ADIS16364_signed_double(14,ADIS16364_read(this, 14, ZACCL_OUT));
  *accZ = ADIS16364_signed_double(14,ADIS16364_read(this, 14, ZACCL_OUT));
  tmp = ADIS16364_signed_double(14,ADIS16364_read(this, 14, XACCL_OUT));
}

// Read gyroscope data
void ADIS16364_readGyroData(ADIS16364 *this, int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ)
{
  int16_t tmp;
  *gyroX = ADIS16364_signed_double(14,ADIS16364_read(this, 14, XGYRO_OUT));
  tmp = ADIS16364_signed_double(14,ADIS16364_read(this, 14, YGYRO_OUT));
  *gyroY = ADIS16364_signed_double(14,ADIS16364_read(this, 14, YGYRO_OUT));
  tmp = ADIS16364_signed_double(14,ADIS16364_read(this, 14, ZGYRO_OUT));
  *gyroZ = ADIS16364_signed_double(14,ADIS16364_read(this, 14, ZGYRO_OUT));
  tmp = ADIS16364_signed_double(14,ADIS16364_read(this, 14, XGYRO_OUT));
}

int16_t ADIS16364_device_id(ADIS16364 *this){
  // Read 14 bits from the PROD_ID register
  return ADIS16364_read(this, 14, PROD_ID);
}

void ADIS16364_wake(ADIS16364 *this){
    this->_cs(LOW);
    this->_delay_cycle(80);
    this->_cs(HIGH);
}

void ADIS16364_factory_reset(ADIS16364 *this){
	ADIS16364_write(this, GLOB_CMD, 0x0002);
	this->_delay_cycle(50);
}
