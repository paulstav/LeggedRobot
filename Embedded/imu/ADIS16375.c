#include <stdint.h>
#include <stdbool.h>

#include "ADIS16375.h"
#include "spi.h"

#include "utils/uartstdio.h"

#define debug_printf UARTprintf

extern uint8_t imuDataReady;

void ADIS16375_wake(ADIS16375 *this){
    this->_cs(LOW);
    this->_delay_cycle(1000);
    this->_cs(HIGH);
    this->_delay_cycle(21000);
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

int16_t ADIS16375_device_id(ADIS16375 *this)
{
  return ADIS16375_read(this, 16, ADIS16375_REG_PROD_ID);
}

int16_t ADIS16375_status(ADIS16375 *this)
{
  return ADIS16375_read(this, 16, ADIS16375_REG_SYS_E_FLA);
}

int16_t ADIS16375_temp(ADIS16375 *this)
{
  return ADIS16375_read(this, 16, ADIS16375_REG_TEMP_OUT);
}

void ADIS16375_Init(ADIS16375 *this, void (* _delay_cycle)(unsigned long), void (* _cs)(unsigned char), void (* _rst)(unsigned char), void (* _spi_setup)(), int16_t (* _spi_write)(int16_t))
{
	this->_cs = _cs;
        this->_rst = _rst;
	this->_spi_setup = _spi_setup;
	this->_spi_write = _spi_write;
	this->_delay_cycle = _delay_cycle;
        this->cur_page = 0x00;

	// Begin SPI
	this->_spi_setup();
	// Initialize CS pin to be high
	this->_cs(HIGH);
	// Wake device up, incase it's sleeping
	ADIS16375_wake(this);
        
        // Software Reset
        //ADIS16375_write(this, ADIS16375_REG_GLOB_CMD, 0x8000);
        //this->_delay_cycle(60000000);
        
        // H/W Reset
        this->_cs(LOW);
        this->_delay_cycle(10);
        this->_cs(HIGH);
        this->_delay_cycle(60000000);
        
        // Wake device up, incase it's sleeping
	ADIS16375_wake(this);
        
        imuDataReady = 0;
}

void ADIS16375_CheckPageChange(ADIS16375 *this, int16_t reg)
{
  int16_t page2go = 0;
  int16_t page_command = 0x0000;
  
  page2go = (reg & 0x00FF);
  
  if(page2go != this->cur_page)
  {
    page_command = 0x8000 + page2go;
    this->cur_page = page2go;
    this->_cs(LOW);
    this->_delay_cycle(2);
    this->_spi_write(page_command);
    this->_delay_cycle(2);
    this->_cs(HIGH);
  }
}

void ADIS16375_readGyroData(ADIS16375 *this, int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ)
{
  int16_t reg = 0;
  
  reg = ADIS16375_REG_X_GYRO_OUT;
  ADIS16375_CheckPageChange(this,reg);
  
  this->_cs(LOW);
  this->_delay_cycle(1);
  this->_spi_write(reg);
  this->_delay_cycle(1);
  this->_cs(HIGH);
  this->_delay_cycle(80);
  reg = ADIS16375_REG_Y_GYRO_OUT;
  this->_cs(LOW);
  this->_delay_cycle(1);
  *gyroX = this->_spi_write(reg);
  this->_delay_cycle(1);
  this->_cs(HIGH);
  this->_delay_cycle(80);
  reg = ADIS16375_REG_Z_GYRO_OUT;
  this->_cs(LOW);
  this->_delay_cycle(1);
  *gyroY = this->_spi_write(reg);
  this->_delay_cycle(1);
  this->_cs(HIGH);
  this->_delay_cycle(80);
  reg = ADIS16375_REG_X_GYRO_OUT;
  this->_cs(LOW);
  this->_delay_cycle(1);
  *gyroZ = this->_spi_write(reg);
  this->_delay_cycle(1);
  this->_cs(HIGH);
}

void ADIS16375_readAccData(ADIS16375 *this, int16_t* accX, int16_t* accY, int16_t* accZ)
{
  int16_t reg = 0;
  
  reg = ADIS16375_REG_X_ACCEL_OUT;
  ADIS16375_CheckPageChange(this,reg);
  
  this->_cs(LOW);
  this->_delay_cycle(1);
  this->_spi_write(reg);
  this->_delay_cycle(1);
  this->_cs(HIGH);
  this->_delay_cycle(80);
  reg = ADIS16375_REG_Y_ACCEL_OUT;
  this->_cs(LOW);
  this->_delay_cycle(1);
  *accX = this->_spi_write(reg);
  this->_delay_cycle(1);
  this->_cs(HIGH);
  this->_delay_cycle(80);
  reg = ADIS16375_REG_Z_ACCEL_OUT;
  this->_cs(LOW);
  this->_delay_cycle(1);
  *accY = this->_spi_write(reg);
  this->_delay_cycle(1);
  this->_cs(HIGH);
  this->_delay_cycle(80);
  reg = ADIS16375_REG_X_ACCEL_OUT;
  this->_cs(LOW);
  this->_delay_cycle(1);
  *accZ = this->_spi_write(reg);
  this->_delay_cycle(1);
  this->_cs(HIGH);
}

int16_t ADIS16375_read(ADIS16375 *this, unsigned char nbits, int16_t reg){
  // initialize variables
  int16_t rshort/*, mask, measure*/; 

  ADIS16375_CheckPageChange(this,reg);
  
  // Get upper and lower unsigned chars
  this->_cs(LOW);
  this->_delay_cycle(1);
  rshort = this->_spi_write(reg);
  this->_delay_cycle(1);
  this->_cs(HIGH);
  this->_delay_cycle(80);
  this->_cs(LOW);
  this->_delay_cycle(1);
  rshort = this->_spi_write(0x0000);
  this->_delay_cycle(1);
  this->_cs(HIGH);

  // calculate mask
  //mask = 0xFFFF >> (16 - nbits);
  

  // Combine upper and lower, and return
  //measure= rshort & mask;

  //return measure;
  return rshort;
}

void ADIS16375_write(ADIS16375 *this, int16_t reg, int16_t value)
{
  int16_t write_val1 = 0 , write_val2 = 0;
  
  write_val1 = (reg  & 0xFF00) + ((value >> 8) & 0x00FF) ;
  
  write_val2 = ((reg & 0xFF00) + 0x0100) + (value & 0x00FF) ;
  
  ADIS16375_CheckPageChange(this,reg);
  
  // set lower byte
  this->_cs(LOW);
  this->_delay_cycle(2);
  this->_spi_write(write_val1 | 0x8000);
  this->_delay_cycle(2);
  this->_cs(HIGH);
  this->_delay_cycle(80);
  this->_cs(LOW);
  this->_delay_cycle(2);
  this->_spi_write(write_val2 | 0x8000);
  this->_delay_cycle(2);
  //this->_spi_write(value & 0x00FF);
  //this->_cs(HIGH);
  //this->_delay_cycle(8);
  // set upper byte
  //this->_cs(LOW);
  //this->_spi_write( (reg + 1) | 0x80 );
  //this->_spi_write( value >> 8 );
  this->_cs(HIGH);
}