#ifndef ADIS16375_H
#define ADIS16375_H

#include <stdint.h>

#define ADIS16375_PAGE_SIZE 0x80

#define ADIS16375_REG(page, reg) (((reg << 8) & 0xFF00) + (page))
 
#define ADIS16375_REG_PAGE_ID 0x00 /* Same address on each page */

#define ADIS16375_REG_SEQ_CNT                   ADIS16375_REG(0x00, 0x06)
#define ADIS16375_REG_SYS_E_FLA                 ADIS16375_REG(0x00, 0x08)
#define ADIS16375_REG_DIAG_STS                  ADIS16375_REG(0x00, 0x0A)
#define ADIS16375_REG_ALM_STS                   ADIS16375_REG(0x00, 0x0C)
#define ADIS16375_REG_TEMP_OUT                  ADIS16375_REG(0x00, 0x0E)
#define ADIS16375_REG_X_GYRO_OUT                ADIS16375_REG(0x00, 0x12)
#define ADIS16375_REG_Y_GYRO_OUT                ADIS16375_REG(0x00, 0x16)
#define ADIS16375_REG_Z_GYRO_OUT                ADIS16375_REG(0x00, 0x1A)
#define ADIS16375_REG_X_ACCEL_OUT               ADIS16375_REG(0x00, 0x1E)
#define ADIS16375_REG_Y_ACCEL_OUT               ADIS16375_REG(0x00, 0x22)
#define ADIS16375_REG_Z_ACCEL_OUT               ADIS16375_REG(0x00, 0x26)
#define ADIS16375_REG_Z_ACCEL_LOW               ADIS16375_REG(0x00, 0x24)

#define ADIS16375_REG_X_DELTAANG_L_OUT          ADIS16375_REG(0x00, 0x40)
#define ADIS16375_REG_X_DELTAANG_H_OUT          ADIS16375_REG(0x00, 0x42)
#define ADIS16375_REG_Y_DELTAANG_L_OUT          ADIS16375_REG(0x00, 0x44)
#define ADIS16375_REG_Y_DELTAANG_H_OUT          ADIS16375_REG(0x00, 0x46)
#define ADIS16375_REG_Z_DELTAANG_L_OUT          ADIS16375_REG(0x00, 0x48)
#define ADIS16375_REG_Z_DELTAANG_H_OUT          ADIS16375_REG(0x00, 0x4A)

#define ADIS16375_REG_X_DELTAVEL_L_OUT          ADIS16375_REG(0x00, 0x4C)
#define ADIS16375_REG_X_DELTAVEL_H_OUT          ADIS16375_REG(0x00, 0x4E)
#define ADIS16375_REG_Y_DELTAVEL_L_OUT          ADIS16375_REG(0x00, 0x50)
#define ADIS16375_REG_Y_DELTAVEL_H_OUT          ADIS16375_REG(0x00, 0x52)
#define ADIS16375_REG_Z_DELTAVEL_L_OUT          ADIS16375_REG(0x00, 0x54)
#define ADIS16375_REG_Z_DELTAVEL_H_OUT          ADIS16375_REG(0x00, 0x56)

#define ADIS16375_REG_PROD_ID                   ADIS16375_REG(0x00, 0x7E)
 
#define ADIS16375_REG_X_GYRO_OFF_L              ADIS16375_REG(0x02, 0x10)
#define ADIS16375_REG_X_GYRO_OFF_H              ADIS16375_REG(0x02, 0x12)
#define ADIS16375_REG_Y_GYRO_OFF_L              ADIS16375_REG(0x02, 0x14)
#define ADIS16375_REG_Y_GYRO_OFF_H              ADIS16375_REG(0x02, 0x16)
#define ADIS16375_REG_Z_GYRO_OFF_L              ADIS16375_REG(0x02, 0x18)
#define ADIS16375_REG_Z_GYRO_OFF_H              ADIS16375_REG(0x02, 0x1A)
#define ADIS16375_REG_X_ACC_OFF_L               ADIS16375_REG(0x02, 0x1C)
#define ADIS16375_REG_X_ACC_OFF_H               ADIS16375_REG(0x02, 0x1E)
#define ADIS16375_REG_Y_ACC_OFF_L               ADIS16375_REG(0x02, 0x20)
#define ADIS16375_REG_Y_ACC_OFF_H               ADIS16375_REG(0x02, 0x22)
#define ADIS16375_REG_Z_ACC_OFF_L               ADIS16375_REG(0x02, 0x24)
#define ADIS16375_REG_Z_ACC_OFF_H               ADIS16375_REG(0x02, 0x26)
#define ADIS16375_REG_X_GYRO_SCALE              ADIS16375_REG(0x02, 0x40)
#define ADIS16375_REG_Y_GYRO_SCALE              ADIS16375_REG(0x02, 0x42)
#define ADIS16375_REG_Z_GYRO_SCALE              ADIS16375_REG(0x02, 0x44)
#define ADIS16375_REG_X_ACCEL_SCALE             ADIS16375_REG(0x02, 0x46)
#define ADIS16375_REG_Y_ACCEL_SCALE             ADIS16375_REG(0x02, 0x48)
#define ADIS16375_REG_Z_ACCEL_SCALE             ADIS16375_REG(0x02, 0x4A)

#define ADIS16375_REG_FLASH_CNT_L               ADIS16375_REG(0x02, 0x78)
#define ADIS16375_REG_FLASH_CNT_H               ADIS16375_REG(0x02, 0x7A)
#define ADIS16375_REG_GLOB_CMD                  ADIS16375_REG(0x03, 0x02)
#define ADIS16375_REG_FNCTIO_CTRL               ADIS16375_REG(0x03, 0x06)
#define ADIS16375_REG_GPIO_CTRL                 ADIS16375_REG(0x03, 0x08)
#define ADIS16375_REG_NULL_CFG                  ADIS16375_REG(0x03, 0x0E)
#define ADIS16375_REG_GEN_CFG                   ADIS16375_REG(0x03, 0x0A)
#define ADIS16375_REG_SLP_CNT                   ADIS16375_REG(0x03, 0x10)
#define ADIS16375_REG_DEC_RATE                  ADIS16375_REG(0x03, 0x0C)

#define ADIS16375_REG_ALM_CNFG1                 ADIS16375_REG(0x03, 0x20)
#define ADIS16375_REG_ALM_CNFG2                 ADIS16375_REG(0x03, 0x22)
#define ADIS16375_REG_XG_ALM_MAGN               ADIS16375_REG(0x03, 0x40)
#define ADIS16375_REG_YG_ALM_MAGN               ADIS16375_REG(0x03, 0x42)
#define ADIS16375_REG_ZG_ALM_MAGN               ADIS16375_REG(0x03, 0x44)
#define ADIS16375_REG_XA_ALM_MAGN               ADIS16375_REG(0x03, 0x46)
#define ADIS16375_REG_YA_ALM_MAGN               ADIS16375_REG(0x03, 0x48)
#define ADIS16375_REG_ZA_ALM_MAGN               ADIS16375_REG(0x03, 0x4A)


typedef struct {

	double sensor[11];
        unsigned char cur_page;

	void (* _delay_cycle)(unsigned long);
	void (* _cs)(unsigned char);
        void (* _rst)(unsigned char);
	void (* _spi_setup)();
	int16_t (* _spi_write)(int16_t);

}ADIS16375;

void ADIS16375_Init(ADIS16375 *this, void (* _delay_cycle)(unsigned long), void (* _cs)(unsigned char), void (* _rst)(unsigned char), void (* _spi_setup)(), int16_t (* _spi_write)(int16_t));

int16_t ADIS16375_status(ADIS16375 *this);
int16_t ADIS16375_device_id(ADIS16375 *this);
int16_t ADIS16375_temp(ADIS16375 *this);
void ADIS16375_wake(ADIS16375 *this);
void ADIS16375_readAccData(ADIS16375 *this, int16_t* accX, int16_t* accY, int16_t* accZ);
void ADIS16375_readGyroData(ADIS16375 *this, int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ);
void ADIS16375_readDeltaVel(ADIS16375 *this, int16_t* deltaX, int16_t* deltaY, int16_t* deltaZ);
void ADIS16375_readDeltaAngle(ADIS16375 *this, int16_t* deltaX, int16_t* deltaY, int16_t* deltaZ);
// Regsiter read/write, and two's complement converters
short ADIS16375_read(ADIS16375 *this, unsigned char nbits, int16_t reg);
void ADIS16375_write(ADIS16375 *this, int16_t, int16_t value);

#endif