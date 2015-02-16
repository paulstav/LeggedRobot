
#include <ADIS16364.h>
#include <spi.h>
#include <driverlib/sysctl.h>
#include <utils/uartstdio.h>





void onecycledelay()
{
	SysCtlDelay(1); // Tiva C series specific
}

void main()
{
	/*ADIS16364 myIMU;

    ADIS16364_Init(&myIMU, onecycledelay, IMU_CS, init_spi, SpiTransfer);
	
	ADIS16364_read(&myIMU, 8, 0x04);*/
	 UARTConfig();
	 UARTprintf("hello");





}
