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

extern uint32_t g_ui32SysClock;

uint8_t SpiTransfer(uint8_t _byte)
{
	uint32_t rbyte = 0;
        
        while(MAP_SSIBusy(IMU_SPI_SSI_BASE))
	{
		MAP_SysCtlDelay(1);
	}

	MAP_SSIDataPut(IMU_SPI_SSI_BASE, _byte);

	while(MAP_SSIBusy(IMU_SPI_SSI_BASE))
	{
		MAP_SysCtlDelay(1);
	}

	MAP_SSIDataGet(IMU_SPI_SSI_BASE, &rbyte);

	return (uint8_t) rbyte;
}

uint16_t SpiTransfer16(uint16_t _byte)
{
	uint32_t rbyte = 0;
        
        while(MAP_SSIBusy(IMU_SPI_SSI_BASE))
	{
		MAP_SysCtlDelay(1);
	}

	MAP_SSIDataPut(IMU_SPI_SSI_BASE, _byte);

	while(MAP_SSIBusy(IMU_SPI_SSI_BASE))
	{
		MAP_SysCtlDelay(1);
	}

	MAP_SSIDataGet(IMU_SPI_SSI_BASE, &rbyte);

	return (uint8_t) rbyte;
}



void init_spi()
{
  unsigned long tmpC = 0;
  
  MAP_SysCtlPeripheralEnable(IMU_SPI_PERIPH);

  //SysCtlPeripheralReset(IMU_SPI_PERIPH);

  MAP_SysCtlPeripheralEnable(IMU_CS_PERIPH);

  //SysCtlPeripheralReset(IMU_CS_PERIPH);

  GPIOPinConfigure(IMU_SPI_CLK_MUX);
  GPIOPinConfigure(IMU_SPI_RX_MUX);
  GPIOPinConfigure(IMU_SPI_TX_MUX);



  MAP_GPIOPinTypeSSI(IMU_SPI_PORT_BASE, IMU_SPI_TX_PIN | IMU_SPI_RX_PIN | IMU_SPI_CLK_PIN);
  MAP_GPIOPinTypeGPIOOutput(IMU_CS_PORT_BASE, IMU_CS_PIN);
  MAP_GPIOPadConfigSet(IMU_CS_PORT_BASE, IMU_CS_PIN, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_STD_WPD);
  MAP_GPIOPinWrite(IMU_CS_PORT_BASE, IMU_CS_PIN, IMU_CS_PIN);

  //MAP_SysCtlPeripheralReset(IMU_SPI_PERIPH);

  MAP_SSIDisable(IMU_SPI_SSI_BASE);
  //SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);
  //HWREG(SSI0_BASE + SSI_O_CC) = SSI_CLOCK_SYSTEM;

  MAP_SSIConfigSetExpClk(IMU_SPI_SSI_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 1000000, 8);

  MAP_SSIEnable(IMU_SPI_SSI_BASE);
  
  while(MAP_SSIDataGetNonBlocking(IMU_SPI_SSI_BASE, &tmpC))
  {
  }
}

void init_spi16()
{
  unsigned long tmpC = 0;
  
  MAP_SysCtlPeripheralEnable(IMU_SPI_PERIPH);

  //SysCtlPeripheralReset(IMU_SPI_PERIPH);

  MAP_SysCtlPeripheralEnable(IMU_CS_PERIPH);

  //SysCtlPeripheralReset(IMU_CS_PERIPH);

  GPIOPinConfigure(IMU_SPI_CLK_MUX);
  GPIOPinConfigure(IMU_SPI_RX_MUX);
  GPIOPinConfigure(IMU_SPI_TX_MUX);



  MAP_GPIOPinTypeSSI(IMU_SPI_PORT_BASE, IMU_SPI_TX_PIN | IMU_SPI_RX_PIN | IMU_SPI_CLK_PIN);
  MAP_GPIOPinTypeGPIOOutput(IMU_CS_PORT_BASE, IMU_CS_PIN);
  MAP_GPIOPadConfigSet(IMU_CS_PORT_BASE, IMU_CS_PIN, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_STD_WPD);
  MAP_GPIOPinWrite(IMU_CS_PORT_BASE, IMU_CS_PIN, IMU_CS_PIN);

  //MAP_SysCtlPeripheralReset(IMU_SPI_PERIPH);

  MAP_SSIDisable(IMU_SPI_SSI_BASE);
  //SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);
  //HWREG(SSI0_BASE + SSI_O_CC) = SSI_CLOCK_SYSTEM;

  MAP_SSIConfigSetExpClk(IMU_SPI_SSI_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 12000000, 16);

  MAP_SSIEnable(IMU_SPI_SSI_BASE);
  
  while(MAP_SSIDataGetNonBlocking(IMU_SPI_SSI_BASE, &tmpC))
  {
  }
}

void IMU_CS(uint8_t HighLow)
{
	if(!HighLow)
		MAP_GPIOPinWrite(IMU_CS_PORT_BASE, IMU_CS_PIN, ~IMU_CS_PIN);
	else
		MAP_GPIOPinWrite(IMU_CS_PORT_BASE, IMU_CS_PIN, IMU_CS_PIN);
}


/*
void SpiPut(uint8_t _byte)
{
	//uint32_t rbyte = 0;

	SSIDataPut(IMU_SPI_SSI_BASE, _byte);

	while(SSIBusy(IMU_SPI_SSI_BASE))
	{
		SysCtlDelay(1);
	}
}

uint8_t SpiGet()
{
	uint32_t rbyte = 0;
	SSIDataGet(IMU_SPI_SSI_BASE, &rbyte);

	while(SSIBusy(IMU_SPI_SSI_BASE))
	{
	SysCtlDelay(1);
	}

	return (uint8_t) rbyte;
}
*/







