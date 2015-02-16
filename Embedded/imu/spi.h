#ifndef __SPI_H__
#define __SPI_H__

#define HIGH 0xFF
#define LOW 0x00

#define IMU_SPI_PERIPH		SYSCTL_PERIPH_SSI0
#define IMU_SPI_SSI_BASE	SSI0_BASE
#define IMU_SPI_PORT_BASE 	GPIO_PORTA_BASE
#define IMU_SPI_CLK_PIN 	GPIO_PIN_2
#define IMU_SPI_RX_PIN 		GPIO_PIN_5
#define IMU_SPI_TX_PIN 		GPIO_PIN_4
#define IMU_SPI_CLK_MUX		GPIO_PA2_SSI0CLK
#define IMU_SPI_RX_MUX		GPIO_PA5_SSI0XDAT1
#define IMU_SPI_TX_MUX		GPIO_PA4_SSI0XDAT0

#define IMU_CS_PERIPH		SYSCTL_PERIPH_GPIOA
#define IMU_CS_PORT_BASE 	GPIO_PORTA_BASE
#define IMU_CS_PIN	        GPIO_PIN_3


uint8_t SpiTransfer(uint8_t _byte);
uint16_t SpiTransfer16(uint16_t _byte);
void init_spi();
void init_spi16();
void IMU_CS(uint8_t HighLow);

#endif
