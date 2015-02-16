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


#include "ADIS16375.h"
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

struct udp_pcb * udp_init_s(void);
struct udp_pcb * udp_init_r(void);
void udp_send_data(void* sbuf, u16_t len);
void udp_receive_data(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port);
struct udp_pcb *Spcb;
struct udp_pcb *Rpcb;
struct ip_addr controller_ip, board_ip;
volatile uint8_t gotIP = 0;
unsigned long device_ip,device_subnet,device_gateway;
struct pbuf *pi;
char pData[68];
bool sendEncoder, setPWMvalue;
uint32_t timerEncoder = 0;
uint8_t pwmValue;

//*****************************************************************************
//
// Display an lwIP type IP Address.
//
//*****************************************************************************
void
DisplayIPAddress(uint32_t ui32Addr)
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
    UARTprintf(pcBuf);
}

//*****************************************************************************
//
// Required by lwIP library to support any host-related timer functions.
//
//*****************************************************************************
void
lwIPHostTimerHandler(void)
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
            UARTprintf("IP Address: ");
            DisplayIPAddress(ui32NewIPAddress);
            //UARTprintf("\nOpen a browser and enter the IP address.\n");
            UARTprintf("\n");
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
    lwIPTimer(SYSTICKMS);
    
    //timerEncoder++;
    //if(timerEncoder == 10)
    //{
      //timerEncoder = 0;
      //sendEncoder = true;
    //}
}

void
ConfigureUART(void)
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
}

void cyclesdelay(unsigned long cycles)
{
	MAP_SysCtlDelay(cycles); // Tiva C series specific
}

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
  // In this case you get: (1 / 1000Hz) * 120MHz = 1200000 cycles.  Note that
  // the maximum period you can set is 2^16.
  // TODO: modify this calculation to use the clock frequency that you are
  // using.
  //
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 120000);

}

uint8_t SetPWMDuty(uint8_t duty)
{
  if(!duty)
  {
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
    PWMGenDisable(PWM0_BASE, PWM_GEN_2);
  }
  else
  {
    if(duty == 100)
      duty = 95;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,
                   (PWMGenPeriodGet(PWM0_BASE, PWM_OUT_4) / 100) * (uint32_t)duty);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
  }
  
  return duty;
}

void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    sendEncoder = true;

}

int
main(void)
{
    ADIS16375 myIMU;
    uint32_t ui32User0, ui32User1;
    uint8_t pui8MACArray[8];
    uint32_t encoderPos = 0;
    uint8_t sendUDP[128];
    uint32_t sends = 0;

    gotIP = 0;
    timerEncoder = 0;
    
    sendEncoder = false;
    setPWMvalue = false;
    pwmValue = 0;
    
    // 147.102.100.130
    device_ip = 0x93666482;

    // 255.255.255.0
    device_subnet = 0xFFFFFF00;

    // 147.102.100.200
    device_gateway = 0x93666401;
    
    // 147.102.100.144
    //IP4_ADDR(&controller_ip, 0x93,0x66,0x64,0x87);

    // 147.102.100.251
    IP4_ADDR(&controller_ip, 0x93,0x66,0x64,0xFB);

    // 147.102.100.130
    IP4_ADDR(&board_ip, 0x93,0x66,0x64,0x82);

    memset(pData,0x31,68);
    
    //
    // Make sure the main oscillator is enabled because this is required by
    // the PHY.  The system must have a 25MHz crystal attached to the OSC
    // pins. The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
    // frequency is 10MHz or higher.
    //
    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
    
    //
    // Set the clocking to run directly from the crystal at 120MHz.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);

    //
    // Configure the device pins.
    //
    PinoutSet(true, false);
    
    //
    // Initialize the UART.
    //
    ConfigureUART();
    
    //
    // Configure Port N1 for as an output for the animation LED.
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // Initialize LED to OFF (0)
    //
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIO_PIN_1);

    //
    // Configure SysTick for a periodic interrupt.
    //
    MAP_SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
    MAP_SysTickEnable();
    MAP_SysTickIntEnable();
    
    //
    // Configure the hardware MAC address for Ethernet Controller filtering of
    // incoming packets.  The MAC address will be stored in the non-volatile
    // USER0 and USER1 registers.
    //
    MAP_FlashUserGet(&ui32User0, &ui32User1);
    if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
    {
        //
        // We should never get here.  This is an error if the MAC address has
        // not been programmed into the device.  Exit the program.
        // Let the user know there is no MAC address
        //
        UARTprintf("No MAC programmed!\n");
        while(1)
        {
        }
    }

    //
    // Tell the user what we are doing just now.
    //
    UARTprintf("Waiting for IP.\n");

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //
    pui8MACArray[0] = ((ui32User0 >>  0) & 0xff);
    pui8MACArray[1] = ((ui32User0 >>  8) & 0xff);
    pui8MACArray[2] = ((ui32User0 >> 16) & 0xff);
    pui8MACArray[3] = ((ui32User1 >>  0) & 0xff);
    pui8MACArray[4] = ((ui32User1 >>  8) & 0xff);
    pui8MACArray[5] = ((ui32User1 >> 16) & 0xff);

    //
    // Initialize the lwIP library, using DHCP.
    //
    //lwIPInit(g_ui32SysClock, pui8MACArray, 0, 0, 0, IPADDR_USE_DHCP);
    lwIPInit(g_ui32SysClock, pui8MACArray, device_ip, device_subnet, device_gateway, IPADDR_USE_STATIC);

    //
    // Set the interrupt priorities.  We set the SysTick interrupt to a higher
    // priority than the Ethernet interrupt to ensure that the file system
    // tick is processed if SysTick occurs while the Ethernet handler is being
    // processed.  This is very likely since all the TCP/IP and HTTP work is
    // done in the context of the Ethernet interrupt.
    //
    MAP_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
    MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);
    
    while(gotIP == 0)
    	SysCtlDelay(120);
    
    UARTprintf("Initializing...\n");
    
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
    QEIPositionSet(QEI0_BASE,0);
    //
    // Delay for some time...
    //
    SysCtlDelay(12000);

    //pi = pbuf_alloc(PBUF_TRANSPORT,8,PBUF_RAM);

    Rpcb = udp_init_r();
    //Spcb = udp_init_s();

    //ADIS16375_Init(&myIMU, cyclesdelay, IMU_CS, init_spi16, SpiTransfer16);
    
    //UARTprintf("Prod ID : %d\n",ADIS16375_device_id(&myIMU));
    //ADIS16375_debug(&myIMU);
    
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock/5000);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    while(1)
    {
      if(setPWMvalue == true)
      {
        UARTprintf("Setting pwm to %d\n",pwmValue);
        SetPWMDuty(pwmValue);
        setPWMvalue = false;
        sendUDP[0] = 0x41;
        sendUDP[1] = pwmValue;
        udp_send_data((void*)sendUDP,2);
      }
      
      if(sendEncoder == true)
      {
        //sends++;
        encoderPos = QEIPositionGet(QEI0_BASE);
        //if(sends == 5000)
        //{
          //UARTprintf("Position %d\n",encoderPos);
          //sends = 0;
        //}
        sendEncoder = false;
        sendUDP[0] = 0x42;
        memcpy(&sendUDP[1],(uint8_t*)(&encoderPos),4);
        udp_send_data((void*)sendUDP,5);
      }
    }
}

struct udp_pcb * udp_init_s(void)
{
  err_t err;
  struct udp_pcb *pcb_s;

  pcb_s = udp_new();
  //UARTprintf("Init pcb = %d\n",pcb);
  udp_bind(pcb_s, IP_ADDR_ANY, 2014);
  //udp_bind(pcb_s, &board_ip, 2012);

  err = udp_connect(pcb_s, &controller_ip, 2014);


  if(err != ERR_OK)
    UARTprintf("Error connecting to controller.\n");
  else
  {
	  UARTprintf("UDP to send at port 2014...\n");
  }
  return pcb_s;
}

struct udp_pcb * udp_init_r(void)
{
  //err_t err;
  struct udp_pcb *pcb_r;
  pcb_r = udp_new();

  udp_bind(pcb_r, IP_ADDR_ANY, 2013);

  UARTprintf("UDP to receive at port 2013...\n");

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

void udp_send_data(void* sbuf, u16_t len)
{
  struct pbuf *p;
  err_t err;

  p = pbuf_alloc(PBUF_TRANSPORT,len,PBUF_RAM);
  memcpy (p->payload, sbuf, len);
  err = udp_sendto(Rpcb, p, &controller_ip, 2014);
  //err = udp_send(Spcb, p);

  pbuf_free(p);
}
