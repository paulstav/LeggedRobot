#ifndef FORCESENSOR_H
#define FORCESENSOR_H

#include <stdint.h>

// Force sensor struct
// This struct has pointers to all functions needed to communicate
// with the Force Sensor
typedef struct {
        // Generic Delay Function
        void (* _delay_cycle)(unsigned long);
        // Function to control CS
	void (* _cs)(unsigned char);
        // SPI initialization function
        void (* _spi_setup)();
        // SPI transaction funtion, write 1 byte, read 1 byte
	uint8_t (* _spi_write)(uint8_t);

}FORCESENSOR;

// Initialization function for the force sensor
void FORCESENSOR_Init(FORCESENSOR *this, void (* _delay_cycle)(unsigned long), void (* _cs)(unsigned char), void (* _spi_setup)(), uint8_t (* _spi_write)(uint8_t));

// Funtion that read all 6 measurements from the Force Sensor
void ReadForceValues(FORCESENSOR *this, uint16_t *_values);
#endif
