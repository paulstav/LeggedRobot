#ifndef ADIS16364_h
#define ADIS16364_h

// convient macros for accessing burst mode sensor data
#define SUPPLY 0
#define XGYRO  1
#define YGYRO  2
#define ZGYRO  3
#define XACCEL 4
#define YACCEL 5
#define ZACCEL 6
#define XTEMP  7
#define YTEMP  8
#define ZTEMP  9
#define ANALOG 10

// Memory map
#define FLASH_CNT   0x00  // Flash memory write count  N/A
#define SUPPLY_OUT  0x02  // Power supply measurement  See Table 9
#define XGYRO_OUT   0x04  // X-axis gyroscope output  See Table 9
#define YGYRO_OUT   0x06  // Y-axis gyroscope output  See Table 9
#define ZGYRO_OUT   0x08  // Z-axis gyroscope output  See Table 9
#define XACCL_OUT   0x0A  // X-axis accelerometer output  See Table 9
#define YACCL_OUT   0x0C  // Y-axis accelerometer output  See Table 9
#define ZACCL_OUT   0x0E  // Z-axis accelerometer output  See Table 9
#define XTEMP_OUT   0x10  // X-axis gyroscope temperature output  See Table 9
#define YTEMP_OUT   0x12  // Y-axis gyroscope temperature output  See Table 9
#define ZTEMP_OUT   0x14  // Z-axis gyroscope temperature output  See Table 9
#define AUX_ADC     0x16  // Auxiliary ADC output  See Table 9
#define XGYRO_OFF   0x1A  // X-axis gyroscope bias offset factor  See Table 15
#define YGYRO_OFF   0x1C  // Y-axis gyroscope bias offset factor  See Table 15
#define ZGYRO_OFF   0x1E  // Z-axis gyroscope bias offset factor  See Table 15
#define XACCL_OFF   0x20  // X-axis acceleration bias offset factor  See Table 16
#define YACCL_OFF   0x22  // Y-axis acceleration bias offset factor  See Table 16
#define ZACCL_OFF   0x24  // Z-axis acceleration bias offset factor  See Table 16
#define ALM_MAG1    0x26  // Alarm 1 amplitude threshold  See Table 27
#define ALM_MAG2    0x28  // Alarm 2 amplitude threshold  See Table 27
#define ALM_SMPL1   0x2A  // Alarm 1 sample size  See Table 28
#define ALM_SMPL2   0x2C  // Alarm 2 sample size  See Table 28
#define ALM_CTRL    0x2E  // Alarm control  See Table 29
#define AUX_DAC     0x30  // Auxiliary DAC data  See Table 23
#define GPIO_CTRL   0x32  // Auxiliary digital input/output control  See Table 21
#define MSC_CTRL    0x34  // Data ready, self-test, miscellaneous  See Table 22
#define SMPL_PRD    0x36  // Internal sample period (rate) control  See Table 18
#define SENS_AVG    0x38  // Dynamic range and digital filter control  See Table 20
#define SLP_CNT     0x3A  // Sleep mode control  See Table 19
#define DIAG_STAT   0x3C  // System status  See Table 26
#define GLOB_CMD    0x3E  // System commands  See Table 17
#define LOT_ID1     0x52  // Lot Identification Code 1  See Table 32
#define LOT_ID2     0x54  // Lot Identification Code 2  See Table 32
#define PROD_ID     0x56  // Product identification, ADIS16364  See Table 32
#define SERIAL_NUM  0x58  // Serial number  See Table 32

typedef struct {

	double sensor[11];

	void (* _delay_cycle)(unsigned long);
	void (* _cs)(unsigned char);
	void (* _spi_setup)();
	unsigned char (* _spi_write)(unsigned char);

}ADIS16364;

void ADIS16364_Init(ADIS16364 *this, void (* _delay_cycle)(unsigned long), void (* _cs)(unsigned char), void (* _spi_setup)(), unsigned char (* _spi_write)(unsigned char));

// Burst read
void ADIS16364_burst_read(ADIS16364 *this);

// debug
void ADIS16364_debug(ADIS16364 *this);

// Regsiter read/write, and two's complement converters
unsigned int ADIS16364_read(ADIS16364 *this, unsigned char nbits, unsigned char reg);
void ADIS16364_write(ADIS16364 *this, unsigned char reg, unsigned int value);
double ADIS16364_signed_double(unsigned char nbits, unsigned int num);
unsigned int ADIS16364_twos_comp(double num);

// Get methods
unsigned int ADIS16364_device_id(ADIS16364 *this);
double ADIS16364_x_gyro_offset_get(ADIS16364 *this);
double ADIS16364_y_gyro_offset_get(ADIS16364 *this);
double ADIS16364_z_gyro_offset_get(ADIS16364 *this);
double ADIS16364_x_accel_offset_get(ADIS16364 *this);
double ADIS16364_y_accel_offset_get(ADIS16364 *this);
double ADIS16364_z_accel_offset_get(ADIS16364 *this);

// Offset set methods and Gyro null
void ADIS16364_x_gyro_offset_set(ADIS16364 *this, double value);
void ADIS16364_y_gyro_offset_set(ADIS16364 *this, double value);
void ADIS16364_z_gyro_offset_set(ADIS16364 *this, double value);
void ADIS16364_x_accel_offset_set(ADIS16364 *this, double value);
void ADIS16364_y_accel_offset_set(ADIS16364 *this, double value);
void ADIS16364_z_accel_offset_set(ADIS16364 *this, double value);
void ADIS16364_gyro_null(ADIS16364 *this);        // Not Tested
void ADIS16364_gyro_prec_null(ADIS16364 *this);   // Not Tested

// Power managment
void ADIS16364_sleep(ADIS16364 *this);            // Not Tested
void ADIS16364_sleep_dur(ADIS16364 *this, double dur);  // Not Tested
void ADIS16364_wake(ADIS16364 *this);             // Not Tested

// System Settings
void ADIS16364_factory_reset(ADIS16364 *this);

#endif
