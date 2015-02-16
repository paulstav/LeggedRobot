#ifndef ADIS16375_h
#define ADIS16375_h


#define PROD_ID     0x7E00  // Product identification, ADIS16375


typedef struct {

	double sensor[11];

	void (* _delay_cycle)(unsigned long);
	void (* _cs)(unsigned char);
	void (* _spi_setup)();
	unsigned short (* _spi_write)(unsigned short);

}ADIS16375;

void ADIS16375_Init(ADIS16375 *this, void (* _delay_cycle)(unsigned long), void (* _cs)(unsigned char), void (* _spi_setup)(), unsigned short (* _spi_write)(unsigned short));

// Burst read
void ADIS16375_burst_read(ADIS16375 *this);

// Regsiter read/write, and two's complement converters
unsigned int ADIS16375_read(ADIS16375 *this, unsigned char nbits, unsigned short reg);
void ADIS16375_write(ADIS16375 *this, unsigned char reg, unsigned int value);

#endif