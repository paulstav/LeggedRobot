#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"

#include "spi.h"

// External variable that hold the system clock speed in Hz
extern uint32_t g_ui32SysClock;

// Write and Read one byte from SPI
uint8_t SpiTransfer(uint8_t _byte)
{
	uint32_t rbyte = 0;
        
        // Wait for bus to be free
        while(MAP_SSIBusy(FORCE_SPI_SSI_BASE))
	{
		MAP_SysCtlDelay(1);
	}

        // Write the data to buffer
	MAP_SSIDataPut(FORCE_SPI_SSI_BASE, _byte);

        // Wait for bus to be free
	while(MAP_SSIBusy(FORCE_SPI_SSI_BASE))
	{
		MAP_SysCtlDelay(1);
	}

        // Read data from the buffer
	MAP_SSIDataGet(FORCE_SPI_SSI_BASE, &rbyte);

        // Return the byte read
	return (uint8_t) rbyte;
}

// Write and Read one 16 bit word from SPI
int16_t SpiTransfer16(int16_t _byte)
{
	uint32_t rbyte = 0;
        
        // Wait for bus to be free
        while(MAP_SSIBusy(IMU_SPI_SSI_BASE))
	{
		MAP_SysCtlDelay(1);
	}

        // Write data from the buffer
	MAP_SSIDataPut(IMU_SPI_SSI_BASE, _byte);

        // Wait for bus to be free
	while(MAP_SSIBusy(IMU_SPI_SSI_BASE))
	{
		MAP_SysCtlDelay(1);
	}

        // Read data from the buffer
	MAP_SSIDataGet(IMU_SPI_SSI_BASE, &rbyte);

        // Return the word read
	return (int16_t) rbyte;
}


// SPI initializaztion function for 8 bit word length
void init_spi()
{
  unsigned long tmpC = 0;
  
  // Enable and Reset the involved peripherals
  MAP_SysCtlPeripheralEnable(FORCE_SPI_PERIPH);

  MAP_SysCtlPeripheralEnable(FORCE_CS_PERIPH);
  
  MAP_SysCtlPeripheralEnable(FORCE_SPI_PORT_PERIPH);

  SysCtlPeripheralReset(FORCE_SPI_PERIPH); 

  SysCtlPeripheralReset(FORCE_CS_PERIPH);
  
  // CS Setup
  MAP_GPIOPinTypeGPIOOutput(FORCE_CS_PORT_BASE, FORCE_CS_PIN);
  MAP_GPIOPadConfigSet(FORCE_CS_PORT_BASE, FORCE_CS_PIN, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_STD_WPU);
  MAP_GPIOPinWrite(FORCE_CS_PORT_BASE, FORCE_CS_PIN, FORCE_CS_PIN);
  
  // Set proper drive strength and internal pull-down resistor to CLK pin
  MAP_GPIOPadConfigSet(FORCE_SPI_PORT_BASE, FORCE_SPI_CLK_PIN, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_STD_WPD);
  // Set proper drive strength and set RX pin as Open-Drain
  MAP_GPIOPadConfigSet(FORCE_SPI_PORT_BASE, FORCE_SPI_RX_PIN, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_OD);

  // SPI pin Configuration
  GPIOPinConfigure(FORCE_SPI_CLK_MUX);
  GPIOPinConfigure(FORCE_SPI_RX_MUX);
  GPIOPinConfigure(FORCE_SPI_TX_MUX);

  MAP_GPIOPinTypeSSI(FORCE_SPI_PORT_BASE, FORCE_SPI_TX_PIN | FORCE_SPI_RX_PIN | FORCE_SPI_CLK_PIN);
  
  // Disable SSI module before configuration
  MAP_SSIDisable(FORCE_SPI_SSI_BASE);
  //SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);
  //HWREG(SSI0_BASE + SSI_O_CC) = SSI_CLOCK_SYSTEM;

  // SPI configuration - Mode 0, 1 MHz, 8 bit word length
  MAP_SSIConfigSetExpClk(FORCE_SPI_SSI_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);

  // Enable the SPI module
  MAP_SSIEnable(FORCE_SPI_SSI_BASE);
  
  // Flush the SPI Rx buffer
  while(MAP_SSIDataGetNonBlocking(FORCE_SPI_SSI_BASE, &tmpC))
  {
  }
}

// SPI initializaztion function for 16 bit word length
void init_spi16()
{
  unsigned long tmpC = 0;
  
  // Enable and Reset the involved peripherals
  MAP_SysCtlPeripheralEnable(IMU_SPI_PERIPH);
  
  MAP_SysCtlPeripheralEnable(IMU_SPI_PORT_PERIPH);

  SysCtlPeripheralReset(IMU_SPI_PERIPH);

  MAP_SysCtlPeripheralEnable(IMU_CS_PERIPH);
  
  MAP_SysCtlPeripheralEnable(IMU_RST_PERIPH);

  SysCtlPeripheralReset(IMU_CS_PERIPH);
  
  // CS Setup
  MAP_GPIOPinTypeGPIOOutput(IMU_CS_PORT_BASE, IMU_CS_PIN);
  MAP_GPIOPadConfigSet(IMU_CS_PORT_BASE, IMU_CS_PIN, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_STD_WPU);
  MAP_GPIOPinWrite(IMU_CS_PORT_BASE, IMU_CS_PIN, IMU_CS_PIN);
  
  // Set proper drive strength and internal pull-down resistor to CLK pin
  MAP_GPIOPadConfigSet(IMU_SPI_PORT_BASE, IMU_SPI_CLK_PIN, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_STD_WPD);
  
  // Set proper drive strength and set RX pin as Open-Drain
  MAP_GPIOPadConfigSet(IMU_SPI_PORT_BASE, IMU_SPI_RX_PIN, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_OD);
  
  // RST Setup
  MAP_GPIOPinTypeGPIOOutput(IMU_RST_PORT_BASE, IMU_RST_PIN);
  MAP_GPIOPadConfigSet(IMU_RST_PORT_BASE, IMU_RST_PIN, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_STD);
  MAP_GPIOPinWrite(IMU_RST_PORT_BASE, IMU_RST_PIN, IMU_RST_PIN);
 
  // SPI pin Configuration
  GPIOPinConfigure(IMU_SPI_CLK_MUX);
  GPIOPinConfigure(IMU_SPI_RX_MUX);
  GPIOPinConfigure(IMU_SPI_TX_MUX);

  MAP_GPIOPinTypeSSI(IMU_SPI_PORT_BASE, IMU_SPI_TX_PIN | IMU_SPI_RX_PIN | IMU_SPI_CLK_PIN);

  // Disable SSI module before configuration
  MAP_SSIDisable(IMU_SPI_SSI_BASE);
  //SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);
  //HWREG(SSI0_BASE + SSI_O_CC) = SSI_CLOCK_SYSTEM;

  // SPI configuration - Mode 3, 8 MHz, 16 bit word length
  MAP_SSIConfigSetExpClk(IMU_SPI_SSI_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 8000000, 16);

  // Enable the SPI module
  MAP_SSIEnable(IMU_SPI_SSI_BASE);
  
  // Flush the SPI Rx buffer
  while(MAP_SSIDataGetNonBlocking(IMU_SPI_SSI_BASE, &tmpC))
  {
  }
}

// Drive the IMU CS pin low when input is 0, high otherwise
void IMU_CS(uint8_t HighLow)
{
	if(!HighLow)
		MAP_GPIOPinWrite(IMU_CS_PORT_BASE, IMU_CS_PIN, 0);
	else
		MAP_GPIOPinWrite(IMU_CS_PORT_BASE, IMU_CS_PIN, IMU_CS_PIN);
}

// Drive the IMU RST pin low when input is 0, high otherwise
void IMU_RST(uint8_t HighLow)
{
	if(!HighLow)
		MAP_GPIOPinWrite(IMU_RST_PORT_BASE, IMU_RST_PIN, 0);
	else
		MAP_GPIOPinWrite(IMU_RST_PORT_BASE, IMU_RST_PIN, IMU_RST_PIN);
}

// Drive the Force Sensor CS pin low when input is 0, high otherwise
void FORCE_CS(uint8_t HighLow)
{
	if(!HighLow)
		MAP_GPIOPinWrite(FORCE_CS_PORT_BASE, FORCE_CS_PIN, 0);
	else
		MAP_GPIOPinWrite(FORCE_CS_PORT_BASE, FORCE_CS_PIN, FORCE_CS_PIN);
}







