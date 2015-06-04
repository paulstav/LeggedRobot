#ifndef __SPI_H__
#define __SPI_H__

#define HIGH 0xFF
#define LOW 0x00

// Pinout for IMU Connection
#define IMU_SPI_PERIPH		SYSCTL_PERIPH_SSI2
#define IMU_SPI_PORT_PERIPH	SYSCTL_PERIPH_GPIOD
#define IMU_SPI_SSI_BASE	SSI2_BASE
#define IMU_SPI_PORT_BASE 	GPIO_PORTD_BASE
#define IMU_SPI_CLK_PIN 	GPIO_PIN_3
#define IMU_SPI_RX_PIN 		GPIO_PIN_0
#define IMU_SPI_TX_PIN 		GPIO_PIN_1
#define IMU_SPI_CLK_MUX		GPIO_PD3_SSI2CLK
#define IMU_SPI_RX_MUX		GPIO_PD0_SSI2XDAT1
#define IMU_SPI_TX_MUX		GPIO_PD1_SSI2XDAT0

#define IMU_CS_PERIPH		SYSCTL_PERIPH_GPION
#define IMU_CS_PORT_BASE 	GPIO_PORTN_BASE
#define IMU_CS_PIN	        GPIO_PIN_2

#define IMU_RST_PERIPH		SYSCTL_PERIPH_GPIOC
#define IMU_RST_PORT_BASE 	GPIO_PORTC_BASE
#define IMU_RST_PIN	        GPIO_PIN_7

#define IMU_IRQ_PERIPH		SYSCTL_PERIPH_GPION
#define IMU_IRQ_PORT_BASE 	GPIO_PORTN_BASE
#define IMU_IRQ_PIN		GPIO_PIN_3
#define IMU_IRQ_INT		INT_GPION

// Pinout for Force Sensor Connection
// Pinout is the same as IMU as in this version 
// each force sensor is attached to a Leg board
// IMU has it's own dedicated board

#define FORCE_SPI_PERIPH	SYSCTL_PERIPH_SSI2
#define FORCE_SPI_PORT_PERIPH	SYSCTL_PERIPH_GPIOD
#define FORCE_SPI_SSI_BASE	SSI2_BASE
#define FORCE_SPI_PORT_BASE 	GPIO_PORTD_BASE
#define FORCE_SPI_CLK_PIN 	GPIO_PIN_3
#define FORCE_SPI_RX_PIN 	GPIO_PIN_0
#define FORCE_SPI_TX_PIN 	GPIO_PIN_1
#define FORCE_SPI_CLK_MUX	GPIO_PD3_SSI2CLK
#define FORCE_SPI_RX_MUX	GPIO_PD0_SSI2XDAT1
#define FORCE_SPI_TX_MUX	GPIO_PD1_SSI2XDAT0

#define FORCE_CS_PERIPH		SYSCTL_PERIPH_GPION
#define FORCE_CS_PORT_BASE 	GPIO_PORTN_BASE
#define FORCE_CS_PIN	        GPIO_PIN_2

#define FORCE_IRQ_PERIPH	SYSCTL_PERIPH_GPION
#define FORCE_IRQ_PORT_BASE 	GPIO_PORTN_BASE
#define FORCE_IRQ_PIN		GPIO_PIN_3
#define FORCE_IRQ_INT		INT_GPION


// SPI implmentation 8 bit width
void init_spi();
void init_spi_imu();
uint8_t SpiTransfer(uint8_t _byte);

// SPI implmentation 16 bit width
void init_spi16();
int16_t SpiTransfer16(int16_t _byte);

// Control CS pin for IMU
void IMU_CS(uint8_t HighLow);

// Control RST pin for IMU

void IMU_RST(uint8_t HighLow);
// Control CS pin for Force Sensor
void FORCE_CS(uint8_t HighLow);

#endif
