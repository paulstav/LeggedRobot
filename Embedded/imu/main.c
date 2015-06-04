#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "drivers/pinout.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/flash.h"
#include "driverlib/systick.h"
#include "utils/locator.h"
#include "utils/lwiplib.h"
#include "utils/ustdlib.h"
#include "inc/hw_pwm.h"
#include "driverlib/pwm.h"
#include "inc/hw_qei.h"
#include "driverlib/qei.h"
#include "driverlib/timer.h"

#ifdef DEV_ADIS16375
#include "ADIS16375.h"
#endif
#ifdef DEV_ADIS16364
#include "ADIS16364.h"
#endif
#include "ForceSensor.h"
#include "spi.h"

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)

//*****************************************************************************
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
//
//*****************************************************************************
#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

//*****************************************************************************
//
// The current IP address.
//
//*****************************************************************************
uint32_t g_ui32IPAddress;
//*****************************************************************************
//
// The system clock frequency.
//
//*****************************************************************************

uint32_t g_ui32SysClock;

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

#ifdef ENABLE_ETHERNET
// Initialize the UDP receive pcb
struct udp_pcb * udp_init_r(void);
// Send data over UDP
void udp_send_data(void* sbuf, u16_t len);
// Callback for UDP data reception
void udp_receive_data(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port);
// The variable that hold the UDP receive pcb
struct udp_pcb *Rpcb;
// Variables assgined to the controller pc IP and current board IP
struct ip_addr controller_ip, board_ip;
// Flag that is raised when the IP is assigned
volatile uint8_t gotIP = 0;
// Variables for lwip configuration
unsigned long device_ip,device_subnet,device_gateway;
#endif

// Flags raised when events for encoder send and pwm set are active
bool sendEncoder, setPWMvalue;
// Variable for received PWM command
int8_t pwmValue;

#ifdef ENABLE_IMU   

#ifdef DEV_ADIS16375
// Function that configures the interrupt detection of ADIS16375 IMU
void ConfigureADIS16375Int(void);
// ADIS16375 object
ADIS16375 myIMU;
#endif

#ifdef DEV_ADIS16364
// Function that configures the interrupt detection of ADIS16364 IMU
void ConfigureADIS16364Int(void);
// ADIS16364 object
ADIS16364 myIMU;
#endif

// Flag for IMU Data ready on interrupt
uint8_t imuDataReady = 0;
// Variables that hold the measurements received from the IMU
int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, delta_x, delta_y, delta_z, dv_x, dv_y, dv_z, temp_out;
double dval_x, dval_y, dval_z, temp, deltaAccX, deltaAccY, deltaAccZ;
#endif

#ifdef ENABLE_FORCE
// Function that configures the interrupt detection from the Force Sensor
void ConfigureForceSensorInt(void);
// Flag for Force Sensor Data ready on interrupt
uint8_t forceDataReady;
// Force Sensor object
FORCESENSOR myForce;
// Variable that hold the received measurements from the Force Sensor
uint16_t forceValues[6] = {0,0,0,0,0,0};
#endif

#ifdef ENABLE_ETHERNET
// Display the input IP address on UART
void DisplayIPAddress(uint32_t ui32Addr)
{
    char pcBuf[16];

    //
    // Convert the IP Address into a string.
    //
    usprintf(pcBuf, "%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,
            (ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);

    //
    // Display the string.
    //
#ifdef ENABLE_UART
    UARTprintf(pcBuf);
#endif
    
}
#endif

#ifdef ENABLE_ETHERNET
// Ethernet lwip interrupt handler
void lwIPHostTimerHandler(void)
{
    uint32_t ui32Idx, ui32NewIPAddress;

    //
    // Get the current IP address.
    //
    ui32NewIPAddress = lwIPLocalIPAddrGet();

    //
    // See if the IP address has changed.
    //
    if(ui32NewIPAddress != g_ui32IPAddress)
    {
        //
        // See if there is an IP address assigned.
        //
        if(ui32NewIPAddress == 0xffffffff)
        {
            //
            // Indicate that there is no link.
            //
            //UARTprintf("Waiting for link.\n");
        }
        else if(ui32NewIPAddress == 0)
        {
            //
            // There is no IP address, so indicate that the DHCP process is
            // running.
            //
            //UARTprintf("Waiting for IP address.\n");
        }
        else
        {
            //
            // Display the new IP address.
            //
#ifdef ENABLE_UART
            UARTprintf("IP Address: ");
            DisplayIPAddress(ui32NewIPAddress);
            UARTprintf("\n");
#endif
            // Set the gotIP flag once IP is assigned
            gotIP = 1;
        }

        //
        // Save the new IP address.
        //
        g_ui32IPAddress = ui32NewIPAddress;

        //
        // Turn GPIO off.
        //
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
    }

    //
    // If there is not an IP address.
    //
    if((ui32NewIPAddress == 0) || (ui32NewIPAddress == 0xffffffff))
    {
        //
        // Loop through the LED animation.
        //

        for(ui32Idx = 1; ui32Idx < 17; ui32Idx++)
        {

            //
            // Toggle the GPIO
            //
            MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,
                    (MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1) ^
                     GPIO_PIN_1));

            SysCtlDelay(g_ui32SysClock/(ui32Idx << 1));
        }
    }
}
#endif

//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Call the lwIP timer handler.
    //
#ifdef ENABLE_ETHERNET
    lwIPTimer(SYSTICKMS);
#endif
}

#ifdef ENABLE_UART
// Configure the UART peripheral
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
    
#ifdef UART_BUFFERED
    UARTEchoSet(false);
#endif
}
#endif

// Generic delay function
void cyclesdelay(unsigned long cycles)
{
	MAP_SysCtlDelay(cycles); // Tiva C series specific
}

#ifdef ENABLE_MOTOR
// Setup the PWM peripheral
void SetupPWM()
{
  SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  GPIOPinConfigure(GPIO_PG0_M0PWM4);
  GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
  
  PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);

  //
  // Set the PWM period to 1000Hz.  To calculate the appropriate parameter
  // use the following equation: N = (1 / f) * SysClk.  Where N is the
  // function parameter, f is the desired frequency, and SysClk is the
  // system clock frequency.
  // In this case you get: (1 / 20000Hz) * 120MHz = 6000 cycles.  Note that
  // the maximum period you can set is 2^16.
  // TODO: modify this calculation to use the clock frequency that you are
  // using.
  //
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 6000);
  
  // Configure Direction pin
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);
  MAP_GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_4, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_STD_WPD);
  MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0);

}

// Fucntion to set the PWM output given the duty cycle
// PWM can range from -100 to 100, if the value is negative
// we reverse the motion by setting the DIR pin low for the drive
// positive direction correspond to DIR pin being high
int8_t SetPWMDuty(int8_t duty)
{
  // If duty cycle is 0 , disable the PWM generator and output
  if(!duty)
  {
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
    PWMGenDisable(PWM0_BASE, PWM_GEN_2);
  }
  else
  {
    // Set DIR pin accordingly
    if(duty < 0)
    {
      MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0);
      duty = 100 - (100 + duty);
    }
    else
      MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, GPIO_PIN_4);
    
    if(duty == 100)
      duty = 95;
    
    // Set the PWM pulse width (duty cycle)
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,
                   (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) / 100) * (uint32_t)duty);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
  }
  // Return the set duty cycle
  return duty;
}

// 5 KHz timer that sets the flag for encoder value transmission
void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    // Set the flag
    sendEncoder = true;

}
#endif

#ifdef ENABLE_IMU

#ifdef DEV_ADIS16375
// ADIS16375 Interrupt handler
void IntADIS16375(void)
{
  uint32_t status;
  
  // Clear the interrupt flag
  status = GPIOIntStatus(IMU_IRQ_PORT_BASE, true);
  
  // Set the appropriate flag
  imuDataReady = 1;
  
  // Read the desired data from the IMU
  ADIS16375_readAccData(&myIMU, &accel_x, &accel_y, &accel_z);
  ADIS16375_readGyroData(&myIMU, &gyro_x, &gyro_y, &gyro_z);
  //ADIS16375_readDeltaAngle(&myIMU, &delta_x, &delta_y, &delta_z);
  //ADIS16375_readDeltaVel(&myIMU, &dv_x, &dv_y, &dv_z);
  
  // Value conversion for delta angle displacement
  // Delta angles need to be accumulated to get proper euler angle values
  /*dval_x = (delta_x*1.0)*0.005493;
  dval_y = (delta_y*1.0)*0.005493;
  dval_z = (delta_z*1.0)*0.005493;
  
  deltaAccX += dval_x;
  deltaAccY += dval_y;
  deltaAccZ += dval_z;*/
  
  GPIOIntClear(IMU_IRQ_PORT_BASE, status);
}

// Configure the ADIS16375 interrupt pin 
void ConfigureADIS16375Int(void)
{
  SysCtlPeripheralEnable(IMU_IRQ_PERIPH);
  GPIOPinTypeGPIOInput(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
  GPIOIntTypeSet(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN, GPIO_RISING_EDGE);
  GPIOIntRegister(IMU_IRQ_PORT_BASE, IntADIS16375);
  //GPIOIntEnable(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
  IntEnable(IMU_IRQ_INT);
}
#endif

#ifdef DEV_ADIS16364
// ADIS16364 Interrupt handler
void IntADIS16364(void)
{
  uint32_t status;
  
  // Clear the interrupt flag
  status = GPIOIntStatus(IMU_IRQ_PORT_BASE, true);
  
  // Set the appropriate flag
  imuDataReady = 1;
  
  // Read the desired data from the IMU
  ADIS16364_readAccData(&myIMU, &accel_x, &accel_y, &accel_z);
  ADIS16364_readGyroData(&myIMU, &gyro_x, &gyro_y, &gyro_z);
  
  GPIOIntClear(IMU_IRQ_PORT_BASE, status);
}

// Configure the ADIS16364 interrupt pin 
void ConfigureADIS16364Int(void)
{
  SysCtlPeripheralEnable(IMU_IRQ_PERIPH);
  GPIOPinTypeGPIOInput(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
  GPIOIntTypeSet(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN, GPIO_RISING_EDGE);
  GPIOIntRegister(IMU_IRQ_PORT_BASE, IntADIS16364);
  //GPIOIntEnable(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
  IntEnable(IMU_IRQ_INT);
}
#endif

#endif

#ifdef ENABLE_FORCE
// Force Sensor data ready interrupt handler
void IntForceSensor(void)
{
  uint32_t status;
  
  // Clear the interrupt
  status = GPIOIntStatus(FORCE_IRQ_PORT_BASE, true);
  
  // Set the appropriate flag
  forceDataReady = 1;
  
  // Read measurements from the Force sensor
  ReadForceValues(&myForce, forceValues);
  
  GPIOIntClear(FORCE_IRQ_PORT_BASE, status);
}

// Configure the Force Sensor interrupt pin
void ConfigureForceSensorInt(void)
{
  SysCtlPeripheralEnable(FORCE_IRQ_PERIPH);
  GPIOPinTypeGPIOInput(FORCE_IRQ_PORT_BASE, FORCE_IRQ_PIN);
  GPIOIntTypeSet(FORCE_IRQ_PORT_BASE, FORCE_IRQ_PIN, GPIO_RISING_EDGE);
  GPIOIntRegister(FORCE_IRQ_PORT_BASE, IntForceSensor);
  //GPIOIntEnable(FORCE_IRQ_PORT_BASE, FORCE_IRQ_PIN);
  IntEnable(FORCE_IRQ_INT);
}
#endif

// Main application
int main(void)
{
   uint32_t status;
   uint8_t printIMU = 0;
   // UART buffer
   uint8_t charUART[256];
   // The first time the IMU gives an interrupt we can set the BIAS NULL
   // command for auto-bias correction on accelerometer and gyroscope data
   uint8_t firstIMU = 0;
    
#ifdef ENABLE_UART
    // Character that is used to receive commands from the UART
    // Used only for debugging
    unsigned char uCom = 0;
#endif
    
#ifdef ENABLE_ETHERNET
    uint32_t ui32User0, ui32User1;
    uint8_t pui8MACArray[8];
    // UDP Send buffer
    uint8_t sendUDP[128];
    // Hold the number of transmission (used for debugging)
    uint32_t sends = 0;
#endif
    
    // Variable to hold the read encoder value
    int32_t encoderPos = 0;   

    // Initialize the application flags
#ifdef ENABLE_MOTOR
    sendEncoder = false;
    setPWMvalue = false;
    pwmValue = 0;
#endif

#ifdef ENABLE_ETHERNET
    gotIP = 0;
    
    // Set the proper values for lwip configuration based on board selection
#if defined(BOARD_FL)
    // 192.168.1.10
    device_ip = 0xC0A8010A;
    IP4_ADDR(&board_ip, 0xC0,0xA8,0x01,0x0A);
#elif defined(BOARD_FR)
    // // 192.168.1.11
    device_ip = 0xC0A8010B;
    IP4_ADDR(&board_ip, 0xC0,0xA8,0x01,0x0B);
#elif defined(BOARD_BL)
    // // 192.168.1.12
    device_ip = 0xC0A8010C;
    IP4_ADDR(&board_ip, 0xC0,0xA8,0x01,0x0C);
#elif defined(BOARD_BR)
    // // 192.168.1.13
    device_ip = 0xC0A8010D;
    IP4_ADDR(&board_ip, 0xC0,0xA8,0x01,0x0D);
#elif defined(BOARD_IMU)
    // // 192.168.1.14
    device_ip = 0xC0A8010E;
    IP4_ADDR(&board_ip, 0xC0,0xA8,0x01,0x0E);
#elif defined(BOARD_FORCE)
    // // 192.168.1.15
    device_ip = 0xC0A8010F;
    IP4_ADDR(&board_ip, 0xC0,0xA8,0x01,0x0F);
#else
    // // 192.168.1.10
    device_ip = 0xC0A8010A;
    IP4_ADDR(&board_ip, 0xC0,0xA8,0x01,0x0A);
#endif

    // 255.255.255.0
    device_subnet = 0xFFFFFF00;

    // 192.168.1.1
    device_gateway = 0xC0A80101;

    // 192.168.1.22
    IP4_ADDR(&controller_ip, 0xC0,0xA8,0x01,0x16);

#endif
    
    // Start the system clock (120 MHz)
    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
    
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);
#ifdef ENABLE_ETHERNET
    // Set pins for ethernet functionality
    PinoutSet(true, false);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
#else
    PinoutSet(false, false);
#endif

#ifdef ENABLE_UART    
    ConfigureUART();
#endif
    
#ifdef ENABLE_ETHERNET
    // Initialize the SysTick timer
    MAP_SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
    MAP_SysTickEnable();
    MAP_SysTickIntEnable();
    
    MAP_FlashUserGet(&ui32User0, &ui32User1);
    if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
    {
#ifdef ENABLE_UART
        UARTprintf("No MAC programmed!\n");
#endif
        while(1)
        {
        }
    }

#ifdef ENABLE_UART
    UARTprintf("Waiting for IP.\n");
#endif
    
    pui8MACArray[0] = ((ui32User0 >>  0) & 0xff);
    pui8MACArray[1] = ((ui32User0 >>  8) & 0xff);
    pui8MACArray[2] = ((ui32User0 >> 16) & 0xff);
    pui8MACArray[3] = ((ui32User1 >>  0) & 0xff);
    pui8MACArray[4] = ((ui32User1 >>  8) & 0xff);
    pui8MACArray[5] = ((ui32User1 >> 16) & 0xff);

    // lwIP stack initialization
    //lwIPInit(g_ui32SysClock, pui8MACArray, 0, 0, 0, IPADDR_USE_DHCP);
    lwIPInit(g_ui32SysClock, pui8MACArray, device_ip, device_subnet, device_gateway, IPADDR_USE_STATIC);

    MAP_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
#endif
    
    MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);

    // Wait until an IP is assigned 
#ifdef ENABLE_ETHERNET    
    while(gotIP == 0)
    	SysCtlDelay(120);
#endif
    
#ifdef ENABLE_UART
    UARTprintf("Initializing...\n");
#endif
   
    // Configure motor interface modules
#ifdef ENABLE_MOTOR
    // Setup the PWM generator
    SetupPWM();
    
    // QEI Setup
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    GPIOPinConfigure(GPIO_PL3_IDX0);
    GPIOPinConfigure(GPIO_PL1_PHA0);
    GPIOPinConfigure(GPIO_PL2_PHB0);
    GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B /*| QEI_CONFIG_RESET_IDX */| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 0xFFFFFFFF);
    //
    // Enable the quadrature encoder.
    //
    QEIEnable(QEI0_BASE);
    QEIPositionSet(QEI0_BASE,(0x00000001 << 31));
    //
    // Delay for some time...
    //
    SysCtlDelay(12000);
#endif

    // Initialize the UDP receive pcb
#ifdef ENABLE_ETHERNET
    Rpcb = udp_init_r();
#endif

    // IMU Initializaztion
#ifdef ENABLE_IMU
    
#ifdef DEV_ADIS16375
    ADIS16375_Init(&myIMU, cyclesdelay, IMU_CS, IMU_RST, init_spi16, SpiTransfer16);    
#ifdef ENABLE_UART
    //UARTprintf("Prod ID : 0x%X\n",ADIS16375_device_id(&myIMU));
    //ADIS16375_write(&myIMU,ADIS16375_REG_GLOB_CMD,0x8000);  
#endif
    //ADIS16375_debug(&myIMU);
    ConfigureADIS16375Int();
    //status = GPIOIntStatus(IMU_IRQ_PORT_BASE, true);
    
    // Clear interrupt flag just to be safe
    GPIOIntClear(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
    GPIOIntEnable(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
    
    //ADIS16375_wake(&myIMU);
    //init_spi16();
    //UARTprintf("Prod ID : 0x%X\n",ADIS16375_device_id(&myIMU));
#endif
    
#ifdef DEV_ADIS16364
    ADIS16364_Init(&myIMU, cyclesdelay, IMU_CS, init_spi_imu, SpiTransfer);    
#ifdef ENABLE_UART
    //UARTprintf("Prod ID : 0x%X\n",ADIS16375_device_id(&myIMU));
    //ADIS16375_write(&myIMU,ADIS16375_REG_GLOB_CMD,0x8000);  
#endif
    //ADIS16375_debug(&myIMU);
    ConfigureADIS16364Int();
    //status = GPIOIntStatus(IMU_IRQ_PORT_BASE, true);
    
    // Clear interrupt flag just to be safe
    GPIOIntClear(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
    GPIOIntEnable(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
    
    ADIS16364_wake(&myIMU);
    //init_spi16();
    //UARTprintf("Prod ID : 0x%X\n",ADIS16375_device_id(&myIMU));
#endif
    
#endif
    
    // Force Sensor Initialization
#ifdef ENABLE_FORCE
    FORCESENSOR_Init(&myForce, cyclesdelay, FORCE_CS, init_spi, SpiTransfer);   
    ConfigureForceSensorInt();   
    GPIOIntClear(FORCE_IRQ_PORT_BASE, FORCE_IRQ_PIN);
    GPIOIntEnable(FORCE_IRQ_PORT_BASE, FORCE_IRQ_PIN);
#endif
    
    // Configure 5 KHz tier for encoder count acquisition
#ifdef ENABLE_MOTOR   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock/5000);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
#endif
    
    // Application Main Loop
    while(1)
    {
#ifdef ENABLE_FORCE
      // If data ready from force sensor output the data
      if(forceDataReady == 1)
      {
        forceDataReady = 0;
#ifdef ENABLE_ETHERNET
          sendUDP[0] = 0x47;
          memcpy(&sendUDP[1],(uint8_t*)forceValues,12);
          udp_send_data((void*)sendUDP,13);
#endif
#ifdef ENABLE_UART
        UARTprintf("Read : %u %u %u %u %u %u\n",forceValues[0],forceValues[1],forceValues[2],forceValues[3],forceValues[4],forceValues[5]);
#endif    
      }
#endif
#ifdef ENABLE_IMU
      // If data ready from IMU output the data
      // On first interrupt only we check the product ID
      // and are able to set the configuration for proper delta angle calculation
      if(imuDataReady == 1)
      {
        if(!firstIMU)
        {
          firstIMU = 1;
#ifdef DEV_ADIS16375
          UARTprintf("Prod ID : 0x%X\n",ADIS16375_device_id(&myIMU));
#endif
          
#ifdef DEV_ADIS16364
          ADIS16364_wake(&myIMU);
          UARTprintf("Prod ID : 0x%X\n",ADIS16364_device_id(&myIMU));
          ADIS16364_factory_reset(&myIMU);
#endif
          
#ifdef DEV_ADIS16375
          // Resore factory calibration on strat-up
          ADIS16375_write(&myIMU, ADIS16375_REG_GLOB_CMD, 0x4000);
          // Configure the ADIS16375 IMU
          //MAP_SysCtlDelay(40000*100);
          
          // Set the decimation coefficient
          //ADIS16375_write(&myIMU, ADIS16375_REG_DEC_RATE, DECIMATION_COEF);
          
          // Set configuration for the BIAS estimator
          ADIS16375_write(&myIMU, ADIS16375_REG_NULL_CFG, 0x0A07);
          
          // Load values for bias correction (BIAS NULL command)
          //ADIS16375_write(&myIMU, ADIS16375_REG_GLOB_CMD, 0x0100);
          
          //GPIOIntDisable(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
#endif
          imuDataReady = 0;  
          deltaAccX = deltaAccY = deltaAccZ = 0.0;
        }
        else
        {
          imuDataReady = 0;
#ifdef ENABLE_ETHERNET
#ifdef DEV_ADIS16375
          sendUDP[0] = 0x43;
#endif
#ifdef DEV_ADIS16364
          sendUDP[0] = 0x44;
#endif
          memcpy(&sendUDP[1],(uint8_t*)(&accel_x),2);
          memcpy(&sendUDP[3],(uint8_t*)(&accel_y),2);
          memcpy(&sendUDP[5],(uint8_t*)(&accel_z),2);
          memcpy(&sendUDP[7],(uint8_t*)(&gyro_x),2);
          memcpy(&sendUDP[9],(uint8_t*)(&gyro_y),2);
          memcpy(&sendUDP[11],(uint8_t*)(&gyro_z),2);
          udp_send_data((void*)sendUDP,13);
#endif
          // Handle the recieved IMU measrements
#ifdef ENABLE_UART
          if(printIMU)
          {
            printIMU = 0;
#ifdef DEV_ADIS16375
            UARTprintf("ACC_X_OUT : %d 0x%X\n",accel_x,accel_x);
            UARTprintf("ACC_Y_OUT : %d 0x%X\n",accel_y,accel_y);
            UARTprintf("ACC_Z_OUT : %d 0x%X\n",accel_z,accel_z);
            
            UARTprintf("GYRO_X_OUT : %d 0x%X\n",gyro_x,gyro_x);
            UARTprintf("GYRO_Y_OUT : %d 0x%X\n",gyro_y,gyro_y);
            UARTprintf("GYRO_Z_OUT : %d 0x%X\n",gyro_z,gyro_z);
            
            dval_x = (gyro_x*1.0)*0.013108;
            dval_y = (gyro_y*1.0)*0.013108;
            dval_z = (gyro_z*1.0)*0.013108;
            
            sprintf(charUART, "GYRO X : %lf\n", dval_x);
            UARTprintf("%s",charUART);
            sprintf(charUART, "GYRO Y : %lf\n", dval_y);
            UARTprintf("%s",charUART);
            sprintf(charUART, "GYRO Z : %lf\n", dval_z);
            UARTprintf("%s",charUART);
            
            dval_x = (accel_x*1.0)*0.8192;
            dval_y = (accel_y*1.0)*0.8192;
            dval_z = (accel_z*1.0)*0.8192;
            
            sprintf(charUART, "ACC X : %lf\n", dval_x);
            UARTprintf("%s",charUART);
            sprintf(charUART, "ACC Y : %lf\n", dval_y);
            UARTprintf("%s",charUART);
            sprintf(charUART, "ACC Z : %lf\n", dval_z);
            UARTprintf("%s",charUART);
            
            /*dval_x = (delta_x*1.0)*0.005493;
            dval_y = (delta_y*1.0)*0.005493;
            dval_z = (delta_z*1.0)*0.005493;
            
            sprintf(charUART, "DELTA X : 0x%X %lf\n", delta_x, dval_x);
            UARTprintf("%s",charUART);
            sprintf(charUART, "DELTA Y : 0x%X %lf\n", delta_y, dval_y);
            UARTprintf("%s",charUART);
            sprintf(charUART, "DELTA Z : 0x%X %lf\n", delta_z, dval_z);
            UARTprintf("%s",charUART);
            
            dval_x = (dv_x*1.0)*3.0518;
            dval_y = (dv_y*1.0)*3.0518;
            dval_z = (dv_z*1.0)*3.0518;
            
            sprintf(charUART, "DELTA VEL X : %lf\n", dval_x);
            UARTprintf("%s",charUART);
            sprintf(charUART, "DELTA VEL Y : %lf\n", dval_y);
            UARTprintf("%s",charUART);
            sprintf(charUART, "DELTA VEL Z : %lf\n", dval_z);
            UARTprintf("%s",charUART);*/
#endif
            
#ifdef DEV_ADIS16364
            UARTprintf("ACC_X_OUT : %d 0x%X\n",accel_x,accel_x);
            UARTprintf("ACC_Y_OUT : %d 0x%X\n",accel_y,accel_y);
            UARTprintf("ACC_Z_OUT : %d 0x%X\n",accel_z,accel_z);
            
            UARTprintf("GYRO_X_OUT : %d 0x%X\n",gyro_x,gyro_x);
            UARTprintf("GYRO_Y_OUT : %d 0x%X\n",gyro_y,gyro_y);
            UARTprintf("GYRO_Z_OUT : %d 0x%X\n",gyro_z,gyro_z);
            
            dval_x = gyro_x*0.05;
            dval_y = gyro_y*0.05;
            dval_z = gyro_z*0.05;
            
            sprintf(charUART, "GYRO X : %lf\n", dval_x);
            UARTprintf("%s",charUART);
            sprintf(charUART, "GYRO Y : %lf\n", dval_y);
            UARTprintf("%s",charUART);
            sprintf(charUART, "GYRO Z : %lf\n", dval_z);
            UARTprintf("%s",charUART);
            
            dval_x = accel_x*1.0;
            dval_y = accel_y*1.0;
            dval_z = accel_z*1.0;
            
            sprintf(charUART, "ACC X : %lf\n", dval_x);
            UARTprintf("%s",charUART);
            sprintf(charUART, "ACC Y : %lf\n", dval_y);
            UARTprintf("%s",charUART);
            sprintf(charUART, "ACC Z : %lf\n", dval_z);
            UARTprintf("%s",charUART);
#endif
          }
        }
#endif
      }
#endif
      // UART command interface for debugging
#ifdef ENABLE_UART
#ifdef UART_BUFFERED
      if(UARTRxBytesAvail()>0)
      {
        uCom = UARTgetc();
        UARTFlushRx();
      }
#endif
      switch(uCom)
      {
#ifdef ENABLE_IMU
      case '1' : 
        // Get IMU product ID
#ifdef DEV_ADIS16375
        UARTprintf("Prod ID : 0x%X\n",ADIS16375_device_id(&myIMU));
#endif
#ifdef DEV_ADIS16364
        UARTprintf("Prod ID : 0x%X\n",ADIS16364_device_id(&myIMU));
#endif
        break;
      case '2': 
        // Reset IMU measurement data
        imuDataReady = 0;
        accel_x = accel_y = accel_z = 0;
        deltaAccX = deltaAccY = deltaAccZ = 0.0;
        status = GPIOIntStatus(IMU_IRQ_PORT_BASE, true);
        GPIOIntClear(IMU_IRQ_PORT_BASE, status);
        GPIOIntEnable(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
        break;
      case '3': 
        // Get data directly from IMU without waiting for interrupt  
#ifdef DEV_ADIS16375
        ADIS16375_readAccData(&myIMU, &accel_x, &accel_y, &accel_z);
        ADIS16375_readGyroData(&myIMU, &gyro_x, &gyro_y, &gyro_z);
        ADIS16375_readDeltaAngle(&myIMU, &delta_x, &delta_y, &delta_z);
        ADIS16375_readDeltaVel(&myIMU, &dv_x, &dv_y, &dv_z);
#endif
        
        dval_x = (gyro_x*1.0)*0.013108;
        dval_y = (gyro_y*1.0)*0.013108;
        dval_z = (gyro_z*1.0)*0.013108;
        
        sprintf(charUART, "GYRO X : %lf\n", dval_x);
        UARTprintf("%s",charUART);
        sprintf(charUART, "GYRO Y : %lf\n", dval_y);
        UARTprintf("%s",charUART);
        sprintf(charUART, "GYRO Z : %lf\n", dval_z);
        UARTprintf("%s",charUART);
        
        dval_x = (accel_x*1.0)*0.8192;
        dval_y = (accel_y*1.0)*0.8192;
        dval_z = (accel_z*1.0)*0.8192;
        
        sprintf(charUART, "ACC X : %lf\n", dval_x);
        UARTprintf("%s",charUART);
        sprintf(charUART, "ACC Y : %lf\n", dval_y);
        UARTprintf("%s",charUART);
        sprintf(charUART, "ACC Z : %lf\n", dval_z);
        UARTprintf("%s",charUART);
        
        /*dval_x = (delta_x*1.0)*0.005493;
        dval_y = (delta_y*1.0)*0.005493;
        dval_z = (delta_z*1.0)*0.005493;
        
        sprintf(charUART, "DELTA X : %lf\n", dval_x);
        UARTprintf("%s",charUART);
        sprintf(charUART, "DELTA Y : %lf\n", dval_y);
        UARTprintf("%s",charUART);
        sprintf(charUART, "DELTA Z : %lf\n", dval_z);
        UARTprintf("%s",charUART);
        
        dval_x = (dv_x*1.0)*3.0518;
        dval_y = (dv_y*1.0)*3.0518;
        dval_z = (dv_z*1.0)*3.0518;
        
        sprintf(charUART, "DELTA VEL X : %lf\n", dval_x);
        UARTprintf("%s",charUART);
        sprintf(charUART, "DELTA VEL Y : %lf\n", dval_y);
        UARTprintf("%s",charUART);
        sprintf(charUART, "DELTA VEL Z : %lf\n", dval_z);
        UARTprintf("%s",charUART);*/
        break;
      case '4' :
        // Output received IMU data 
        printIMU = 1;
        break;
#ifdef DEV_ADIS16375
      case '5' : 
        // Read and display IMU internal temperature
        temp_out = ADIS16375_temp(&myIMU);
        temp = (temp_out*1.0)*0.00565 + 25.0;
        sprintf(charUART, "%lf", temp);
        UARTprintf("Temp : 0x%X %s\n",temp_out,charUART);
        break;
      case '6' :
        // Output bias coefficients
        UARTprintf("X_GYRO_OFF_L : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_X_GYRO_OFF_L));
        UARTprintf("X_GYRO_OFF_H : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_X_GYRO_OFF_H));
        UARTprintf("Y_GYRO_OFF_L : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Y_GYRO_OFF_L));
        UARTprintf("Y_GYRO_OFF_H : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Y_GYRO_OFF_H));
        UARTprintf("Z_GYRO_OFF_L : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Z_GYRO_OFF_L));
        UARTprintf("Z_GYRO_OFF_H : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Z_GYRO_OFF_H));
        UARTprintf("X_ACC_OFF_L : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_X_ACC_OFF_L));
        UARTprintf("X_ACC_OFF_H : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_X_ACC_OFF_H));
        UARTprintf("Y_ACC_OFF_L : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Y_ACC_OFF_L));
        UARTprintf("Y_ACC_OFF_H : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Y_ACC_OFF_H));
        UARTprintf("Z_ACC_OFF_L : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Z_ACC_OFF_L));
        UARTprintf("Z_ACC_OFF_H : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Z_ACC_OFF_H));
        UARTprintf("X_GYRO_SCALE : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_X_GYRO_SCALE));
        UARTprintf("Y_GYRO_SCALE : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Y_GYRO_SCALE));
        UARTprintf("Z_GYRO_SCALE : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Z_GYRO_SCALE));
        UARTprintf("X_ACC_SCALE : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_X_ACCEL_SCALE));
        UARTprintf("Y_ACC_SCALE : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Y_ACCEL_SCALE));
        UARTprintf("Z_ACC_SCALE : 0x%X\n",ADIS16375_read(&myIMU, 16, ADIS16375_REG_Z_ACCEL_SCALE));
        UARTprintf("GEN_CONFIG : 0x%X\n",(ADIS16375_read(&myIMU, 16, ADIS16375_REG_GEN_CFG) & 0x00FF));
        UARTprintf("NULL_CONFIG : 0x%X\n",(ADIS16375_read(&myIMU, 16, ADIS16375_REG_NULL_CFG) & 0x3FFF));
        UARTprintf("DEC_RATE : 0x%X\n",(ADIS16375_read(&myIMU, 16, ADIS16375_REG_DEC_RATE) & 0x07FF));
        break;
      case '7':
        // Load calibration values
        ADIS16375_write(&myIMU, ADIS16375_REG_GLOB_CMD, 0x0100);
        break;
      case '8': 
        // Output delta angle from stored accumulated data
        sprintf(charUART, "DELTA X : %lf\n", deltaAccX);
        UARTprintf("%s",charUART);
        sprintf(charUART, "DELTA Y : %lf\n", deltaAccY);
        UARTprintf("%s",charUART);
        sprintf(charUART, "DELTA Z : %lf\n", deltaAccZ);
        UARTprintf("%s",charUART);
        break;
#endif
      case 'w':
        // IMU wakeup
#ifdef DEV_ADIS16375
        ADIS16375_wake(&myIMU);
#endif
#ifdef DEV_ADIS16364
        ADIS16364_wake(&myIMU);
#endif
        break;
#endif
      default : break;
      }
      uCom = 0;
#endif
      
#ifdef ENABLE_MOTOR
      // If we received a PWM command set the corresponding PWM duty cyle and direction
      if(setPWMvalue == true)
      {
        //UARTprintf("Setting pwm to %d\n",pwmValue);
        SetPWMDuty(pwmValue);
        setPWMvalue = false;
        /*sendUDP[0] = 0x41;
        sendUDP[1] = pwmValue;
        udp_send_data((void*)sendUDP,2);*/
      }
      
      // If send encoder value event occurs, send data via UDP
      if(sendEncoder == true)
      {
        //sends++;
        encoderPos = (QEIPositionGet(QEI0_BASE)- (0x00000001 << 31));
        //if(sends == 5000)
        //{
          //UARTprintf("Position %d\n",encoderPos);
          //sends = 0;
        //}
        sendEncoder = false;
#ifdef ENABLE_ETHERNET
        sendUDP[0] = 0x42;
        memcpy(&sendUDP[1],(uint8_t*)(&encoderPos),4);
        udp_send_data((void*)sendUDP,5);
#endif
      }
#endif
    }
}

#ifdef ENABLE_ETHERNET

// Initializze the UDP receive pcb
struct udp_pcb * udp_init_r(void)
{
  //err_t err;
  struct udp_pcb *pcb_r;
  pcb_r = udp_new();

  // Bind to given port , receive from any IP
  udp_bind(pcb_r, IP_ADDR_ANY, PORT_R);

#ifdef ENABLE_UART
  UARTprintf("UDP to receive at port %d...\n", PORT_R);
#endif
  
  // Set the receive data callback
  udp_recv(pcb_r, udp_receive_data, NULL);

  return pcb_r;
}

void udp_receive_data(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
    char * pPointer;

    //struct pbuf *p1;

    if (p != NULL)
    {
      //UARTwrite((char*)(p->payload), p->len);
      //UARTprintf("R : %s\n",(char*)(p->payload));

      pPointer = (char*)(p->payload);

     /* p1 = pbuf_alloc(PBUF_TRANSPORT,8,PBUF_RAM);
      memcpy (p1->payload, pData, 8);
      udp_send(pcb, p1);
      pbuf_free(p1);*/

      /*if(pPointer[0] == 0x31)
      {
    	  udp_send_data((void*)pData, 68);
      }*/
      // If we received PWM commnad (0x31 command byte)
      // extract the transmitted value
      if(pPointer[0] == 0x31)
      {
        pwmValue = pPointer[1];
        setPWMvalue = true;
      }
      
      /*if(pPointer[0] == 0x32)
      {
        sendEncoder = true;
      }*/
      
      pbuf_free(p);
    }
}

// Send data over UDP to the defined port
void udp_send_data(void* sbuf, u16_t len)
{
  struct pbuf *p;
  err_t err;

  p = pbuf_alloc(PBUF_TRANSPORT,len,PBUF_RAM);
  memcpy (p->payload, sbuf, len);
  err = udp_sendto(Rpcb, p, &controller_ip, PORT_S);

  pbuf_free(p);
}

#endif
