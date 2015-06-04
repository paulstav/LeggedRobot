#include <stdint.h>
#include <stdbool.h>

#include "ForceSensor.h"
#include "spi.h"

#include "utils/uartstdio.h"

// External variable that acts as flag when interrupt form Force Sensor is received
extern uint8_t forceDataReady;

// Initialization function for the force sensor
void FORCESENSOR_Init(FORCESENSOR *this, void (* _delay_cycle)(unsigned long), void (* _cs)(unsigned char), void (* _spi_setup)(), uint8_t (* _spi_write)(uint8_t))
{
        // Assign the proper funtion pointers
	this->_cs = _cs;
        this->_spi_setup = _spi_setup;
	this->_spi_write = _spi_write;
	this->_delay_cycle = _delay_cycle;
	// Begin SPI
	this->_spi_setup();
	// Initialize CS pin to be high
	this->_cs(HIGH);
        
        // Indicate no data ready
        forceDataReady = 0;
}

// Funtion that read all 6 measurements from the Force Sensor
// Needs a uint16_t [6] vector to place the read values
// each value/measurement is 16 bit long so 2 spi reads are required
void ReadForceValues(FORCESENSOR *this, uint16_t *_values)
{
  uint8_t cTemp = 'm';
  uint8_t * cVal = (uint8_t*)_values;
  
  this->_cs(LOW);
  this->_delay_cycle(100);
  // Send the data retrieve command
  cTemp = this->_spi_write(cTemp);
  cTemp = 0;
  
  // Get first measurement
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  
  // Get second measurement
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  
  // Get third measurement
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  
  // Get fourth measurement
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  
  // Get fifth measurement
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  
  // Get sixth measurement
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  cVal++;
  this->_delay_cycle(100);
  *cVal = this->_spi_write(cTemp);
  this->_delay_cycle(100);

  this->_cs(HIGH);
}