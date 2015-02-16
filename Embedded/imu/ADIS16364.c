
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
    this->sensor[i] = ( ( offset_bin[i] )?( raw ):( ADIS16364_signed_double( bits[i], raw ) ) ) * scale[i] + add[i];
    this->_delay_cycle(8);
  }
   
  this->_cs(HIGH);
}

void ADIS16364_debug(ADIS16364 *this)
{
	// print all readable registers
	debug_printf("Device ID:");
	debug_printf("%d\n",ADIS16364_device_id(this));

	// perform burst read
	//ADIS16364_burst_read(this);

	debug_printf("Supply Voltage: ");
	debug_printf("%d",this->sensor[SUPPLY]);
	debug_printf(" V\n");

	debug_printf("Gyroscope: ");
	debug_printf("(");
	debug_printf("%d",this->sensor[XGYRO]);
	debug_printf(", ");
	debug_printf("%d",this->sensor[YGYRO]);
	debug_printf(", ");
	debug_printf("%d",this->sensor[ZGYRO]);
	debug_printf(") deg / s\n");

	debug_printf("Accelerometer: ");
	debug_printf("(");
	debug_printf("%d",this->sensor[XACCEL]);
	debug_printf(", ");
	debug_printf("%d",this->sensor[YACCEL]);
	debug_printf(", ");
	debug_printf("%d",this->sensor[ZACCEL]);
	debug_printf(") mg\n");

	debug_printf("Temperature: ");
	debug_printf("(");
	debug_printf("%d",this->sensor[XTEMP]);
	debug_printf(", ");
	debug_printf("%d",this->sensor[YTEMP]);
	debug_printf(", ");
	debug_printf("%d",this->sensor[ZTEMP]);
	debug_printf(") dec C\n");

	debug_printf("Analog Input: ");
	debug_printf("%d",this->sensor[ANALOG]);
	debug_printf(" V\n");

	debug_printf("Gyro Offset: ");
	debug_printf("(");
	debug_printf("%d",ADIS16364_x_gyro_offset_get(this));
	debug_printf(", ");
	debug_printf("%d",ADIS16364_y_gyro_offset_get(this));
	debug_printf(", ");
	debug_printf("%d",ADIS16364_z_gyro_offset_get(this));
	debug_printf(") deg / s\n");

	debug_printf("Accel Offset: ");
	debug_printf("(");
	debug_printf("%d",ADIS16364_x_accel_offset_get(this));
	debug_printf(", ");
	debug_printf("%d",ADIS16364_y_accel_offset_get(this));
	debug_printf(", ");
	debug_printf("%d",ADIS16364_z_accel_offset_get(this));
	debug_printf(") mg\n");
}

unsigned int ADIS16364_read(ADIS16364 *this, unsigned char nbits, unsigned char reg){
  // initialize variables
  unsigned char upper, lower, mask; 
  double measure;

  // Get upper and lower unsigned chars
  this->_cs(LOW);
  this->_spi_write(reg);
  this->_spi_write(0x00);
  this->_cs(HIGH);
  this->_delay_cycle(1);
  this->_cs(LOW);
  upper = this->_spi_write(0x00);
  lower = this->_spi_write(0x00);
  this->_cs(HIGH);

  // calculate mask
  mask = 0xFF >> (16 - nbits);
  

  // Combine upper and lower, and return
  measure=( ( upper & mask ) << 8 ) | ( lower );

  return measure;
}

void ADIS16364_write(ADIS16364 *this, unsigned char reg, unsigned int value){
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

double ADIS16364_signed_double(unsigned char nbits, unsigned int num){
  unsigned int mask, padding;
  // select correct mask
  mask = 1 << (nbits -1);
  
  // if MSB is 1, then number is negative, so invert it and add one
  // if MSB is 0, then just return the number 
  return (num & mask)?( -1.0 * (~(num | 0xFF << nbits)  + 1) ):( 1.0 * num );
}

unsigned int ADIS16364_twos_comp(double num){
  unsigned int raw;
  
  if(num < 0){
    raw = ~((unsigned int)(-num) - 1);
  }else{
    raw = (unsigned int)num;
  }
  return raw;
}

unsigned int ADIS16364_device_id(ADIS16364 *this){
  // Read 14 bits from the PROD_ID register
  return ADIS16364_read(this, 14, PROD_ID);
}

double ADIS16364_x_gyro_offset_get(ADIS16364 *this){
  unsigned int raw_value = ADIS16364_read(this, 13, XGYRO_OFF);
  return ADIS16364_signed_double(13, raw_value)*0.0125;
}

double ADIS16364_y_gyro_offset_get(ADIS16364 *this){
  unsigned int raw_value = ADIS16364_read(this, 13, YGYRO_OFF);
  return ADIS16364_signed_double(13, raw_value)*0.0125;
}

double ADIS16364_z_gyro_offset_get(ADIS16364 *this){
  unsigned int raw_value = ADIS16364_read(this, 13, ZGYRO_OFF);
  return ADIS16364_signed_double(13, raw_value)*0.0125;
}

double ADIS16364_x_accel_offset_get(ADIS16364 *this){
  unsigned int raw_value = ADIS16364_read(this, 12, XACCL_OFF);
  return ADIS16364_signed_double(12, raw_value);
}

double ADIS16364_y_accel_offset_get(ADIS16364 *this){
  unsigned int raw_value = ADIS16364_read(this, 12, YACCL_OFF);
  return ADIS16364_signed_double(12, raw_value);
}

double ADIS16364_z_accel_offset_get(ADIS16364 *this){
  unsigned int raw_value = ADIS16364_read(this, 12, ZACCL_OFF);
  return ADIS16364_signed_double(12, raw_value);
}

void ADIS16364_x_gyro_offset_set(ADIS16364 *this, double value){
	ADIS16364_write(this, XGYRO_OFF, ADIS16364_twos_comp(value/0.0125));
}

void ADIS16364_y_gyro_offset_set(ADIS16364 *this, double value){
	ADIS16364_write(this, YGYRO_OFF, ADIS16364_twos_comp(value/0.0125));
}

void ADIS16364_z_gyro_offset_set(ADIS16364 *this, double value){
	ADIS16364_write(this, ZGYRO_OFF,ADIS16364_twos_comp(value/0.0125));
}

void ADIS16364_x_accel_offset_set(ADIS16364 *this, double value){
	ADIS16364_write(this, XACCL_OFF, ADIS16364_twos_comp(value));
}

void ADIS16364_y_accel_offset_set(ADIS16364 *this, double value){
	ADIS16364_write(this, YACCL_OFF, ADIS16364_twos_comp(value));
}

void ADIS16364_z_accel_offset_set(ADIS16364 *this, double value){
	ADIS16364_write(this, ZACCL_OFF, ADIS16364_twos_comp(value));
}

void ADIS16364_gyro_null(ADIS16364 *this){
  ADIS16364_write(this, GLOB_CMD, 0x0001);
 this->_delay_cycle(50);
}

void ADIS16364_gyro_prec_null(ADIS16364 *this){
  ADIS16364_write(this, GLOB_CMD, 0x0010);
  this->_delay_cycle(30000);
}

void ADIS16364_sleep(ADIS16364 *this){
	ADIS16364_write(this, SLP_CNT, 0x0100);
}

void ADIS16364_sleep_dur(ADIS16364 *this, double dur){
	ADIS16364_write(this, SLP_CNT, ( (unsigned int)(dur / 0.5) ) & 0x00FF);
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
