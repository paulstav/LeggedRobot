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

#ifdef ENABLE_ETHERNET
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
#endif

bool sendEncoder, setPWMvalue;
int8_t pwmValue;

#ifdef ENABLE_IMU    
ADIS16375 myIMU;
uint8_t imuDataReady = 0;
int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp_out;
double dval_x, dval_y, dval_z, temp;
#endif

#ifdef ENABLE_ETHERNET
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
#ifdef ENABLE_UART
    UARTprintf(pcBuf);
#endif
    
}
#endif

#ifdef ENABLE_ETHERNET
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
#ifdef ENABLE_UART
            UARTprintf("IP Address: ");
            DisplayIPAddress(ui32NewIPAddress);
            UARTprintf("\n");
#endif
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
    //timerEncoder++;
    //if(timerEncoder == 10)
    //{
      //timerEncoder = 0;
      //sendEncoder = true;
    //}
}

#ifdef ENABLE_UART
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
    
#ifdef UART_BUFFERED
    UARTEchoSet(false);
#endif
}
#endif

void cyclesdelay(unsigned long cycles)
{
	MAP_SysCtlDelay(cycles); // Tiva C series specific
}

#ifdef ENABLE_MOTOR
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
  // In this case you get: (1 / 10000Hz) * 120MHz = 12000 cycles.  Note that
  // the maximum period you can set is 2^16.
  // TODO: modify this calculation to use the clock frequency that you are
  // using.
  //
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 12000);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);
  MAP_GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_4, GPIO_STRENGTH_8MA,
                                GPIO_PIN_TYPE_STD_WPD);
  MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0);

}

int8_t SetPWMDuty(int8_t duty)
{
  if(!duty)
  {
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
    PWMGenDisable(PWM0_BASE, PWM_GEN_2);
  }
  else
  {
    if(duty < 0)
    {
      MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0);
      duty = 100 - (100 + duty);
    }
    else
      MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, GPIO_PIN_4);
    
    if(duty == 100)
      duty = 95;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,
                   (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) / 100) * (uint32_t)duty);
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
#endif

#ifdef ENABLE_IMU
void IntADIS16375(void)
{
  uint32_t status;
  
  status = GPIOIntStatus(IMU_IRQ_PORT_BASE, true);
  
  imuDataReady = 1;
  
  /*accel_x = ADIS16375_read(&myIMU, 16, ADIS16375_REG_X_ACCEL_OUT);
  accel_y = ADIS16375_read(&myIMU, 16, ADIS16375_REG_Y_ACCEL_OUT);
  accel_z = ADIS16375_read(&myIMU, 16, ADIS16375_REG_Z_ACCEL_OUT);*/
  
  ADIS16375_readAccData(&myIMU, &accel_x, &accel_y, &accel_z);
  ADIS16375_readGyroData(&myIMU, &gyro_x, &gyro_y, &gyro_z);
  
  GPIOIntClear(IMU_IRQ_PORT_BASE, status);
}


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

int
main(void)
{
   uint32_t status;
   uint8_t charUART[128];
    
#ifdef ENABLE_UART
    unsigned char uCom = 0;
#endif
    
#ifdef ENABLE_ETHERNET
    uint32_t ui32User0, ui32User1;
    uint8_t pui8MACArray[8];
    uint8_t sendUDP[128];
    uint32_t sends = 0;
#endif
    
    uint32_t encoderPos = 0;   

#ifdef ENABLE_MOTOR
    sendEncoder = false;
    setPWMvalue = false;
    pwmValue = 0;
#endif

#ifdef ENABLE_ETHERNET
    gotIP = 0;
    
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

    memset(pData,0x31,68);
#endif
    
    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
    
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);
#ifdef ENABLE_ETHERNET
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

    //lwIPInit(g_ui32SysClock, pui8MACArray, 0, 0, 0, IPADDR_USE_DHCP);
    lwIPInit(g_ui32SysClock, pui8MACArray, device_ip, device_subnet, device_gateway, IPADDR_USE_STATIC);

    MAP_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
#endif
    
    MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);

#ifdef ENABLE_ETHERNET    
    while(gotIP == 0)
    	SysCtlDelay(120);
#endif
    
#ifdef ENABLE_UART
    UARTprintf("Initializing...\n");
#endif
   
#ifdef ENABLE_MOTOR
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
#endif

#ifdef ENABLE_ETHERNET
    Rpcb = udp_init_r();
    //Spcb = udp_init_s();
#endif

#ifdef ENABLE_IMU
    // Only for Boosterpack 2 SSI3 connection
    //MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    
    ADIS16375_Init(&myIMU, cyclesdelay, IMU_CS, IMU_RST, init_spi16, SpiTransfer16);
    
#ifdef ENABLE_UART
    UARTprintf("Prod ID : 0x%X\n",ADIS16375_device_id(&myIMU));
    //ADIS16375_write(&myIMU,ADIS16375_REG_GLOB_CMD,0x8000);  
#endif
    //ADIS16375_debug(&myIMU);
    ConfigureADIS16375Int();
#endif
    
#ifdef ENABLE_MOTOR   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock/5000);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
#endif
    
    while(1)
    {
#ifdef ENABLE_IMU
      if(imuDataReady == 1)
      {
        GPIOIntDisable(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
        imuDataReady = 2;
#ifdef ENABLE_UART
        UARTprintf("ACC_X_OUT : %d 0x%X\n",accel_x,accel_x);
        UARTprintf("ACC_Y_OUT : %d 0x%X\n",accel_y,accel_y);
        UARTprintf("ACC_Z_OUT : %d 0x%X\n",accel_z,accel_z);
        
        UARTprintf("GYRO_X_OUT : %d 0x%X\n",gyro_x,gyro_x);
        UARTprintf("GYRO_Y_OUT : %d 0x%X\n",gyro_y,gyro_y);
        UARTprintf("GYRO_Z_OUT : %d 0x%X\n",gyro_z,gyro_z);
        
        dval_x = (accel_x*1.0)*0.8192;
        dval_y = (accel_y*1.0)*0.8192;
        dval_z = (accel_z*1.0)*0.8192;
        
        sprintf(charUART, "X : %lf\n", dval_x);
        UARTprintf("%s",charUART);
        sprintf(charUART, "Y : %lf\n", dval_y);
        UARTprintf("%s",charUART);
        sprintf(charUART, "Z : %lf\n", dval_z);
        UARTprintf("%s",charUART);
#endif
      }
      /*if((UART_TX_BUFFER_SIZE == UARTTxBytesFree()) && (imuDataReady == 2))
      {
        imuDataReady = 3;
        status = GPIOIntStatus(IMU_IRQ_PORT_BASE, true);
        GPIOIntClear(IMU_IRQ_PORT_BASE, status);
        GPIOIntEnable(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
      }*/
#endif
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
        UARTprintf("Prod ID : 0x%X\n",ADIS16375_device_id(&myIMU));
        break;
      case '2': 
        imuDataReady = 0;
        accel_x = accel_y = accel_z = 0;
        status = GPIOIntStatus(IMU_IRQ_PORT_BASE, true);
        GPIOIntClear(IMU_IRQ_PORT_BASE, status);
        GPIOIntEnable(IMU_IRQ_PORT_BASE, IMU_IRQ_PIN);
        break;
      case '3': 
        /*accel_x = ADIS16375_read(&myIMU, 16, ADIS16375_REG_X_ACCEL_OUT);
        accel_y = ADIS16375_read(&myIMU, 16, ADIS16375_REG_Y_ACCEL_OUT);
        accel_z = ADIS16375_read(&myIMU, 16, ADIS16375_REG_Z_ACCEL_OUT);*/
        
        ADIS16375_readAccData(&myIMU, &accel_x, &accel_y, &accel_z);
        ADIS16375_readGyroData(&myIMU, &gyro_x, &gyro_y, &gyro_z);
        
        UARTprintf("ACC_X_OUT : %d 0x%X\n",accel_x,accel_x);
        UARTprintf("ACC_Y_OUT : %d 0x%X\n",accel_y,accel_y);
        UARTprintf("ACC_Z_OUT : %d 0x%X\n",accel_z,accel_z);
        
        UARTprintf("GYRO_X_OUT : %d 0x%X\n",gyro_x,gyro_x);
        UARTprintf("GYRO_Y_OUT : %d 0x%X\n",gyro_y,gyro_y);
        UARTprintf("GYRO_Z_OUT : %d 0x%X\n",gyro_z,gyro_z);
        
        dval_x = (accel_x*1.0)*0.8192;
        dval_y = (accel_y*1.0)*0.8192;
        dval_z = (accel_z*1.0)*0.8192;
        
        sprintf(charUART, "X : %lf\n", dval_x);
        UARTprintf("%s",charUART);
        sprintf(charUART, "Y : %lf\n", dval_y);
        UARTprintf("%s",charUART);
        sprintf(charUART, "Z : %lf\n", dval_z);
        UARTprintf("%s",charUART);
        break;
      case '4' : 
        UARTprintf("Status : 0x%X\n",ADIS16375_status(&myIMU));
        break;
      case '5' : 
        temp_out = ADIS16375_temp(&myIMU);
        temp = (temp_out*1.0)*0.00565 + 25.0;
        sprintf(charUART, "%lf", temp);
        UARTprintf("Temp : 0x%X %s\n",temp_out,charUART);
        break;
#endif
      default : break;
      }
      uCom = 0;
#endif
      
#ifdef ENABLE_MOTOR
      if(setPWMvalue == true)
      {
        //UARTprintf("Setting pwm to %d\n",pwmValue);
        SetPWMDuty(pwmValue);
        setPWMvalue = false;
        /*sendUDP[0] = 0x41;
        sendUDP[1] = pwmValue;
        udp_send_data((void*)sendUDP,2);*/
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

struct udp_pcb * udp_init_s(void)
{
  err_t err;
  struct udp_pcb *pcb_s;

  pcb_s = udp_new();
  //UARTprintf("Init pcb = %d\n",pcb);
  udp_bind(pcb_s, IP_ADDR_ANY, PORT_S);
  //udp_bind(pcb_s, &board_ip, 2012);

  err = udp_connect(pcb_s, &controller_ip, PORT_S);

#ifdef ENABLE_UART
  if(err != ERR_OK)
    UARTprintf("Error connecting to controller.\n");
  else
  {
	  UARTprintf("UDP to send at port %d...\n",PORT_S);
  }
#endif
  return pcb_s;
}

struct udp_pcb * udp_init_r(void)
{
  //err_t err;
  struct udp_pcb *pcb_r;
  pcb_r = udp_new();

  udp_bind(pcb_r, IP_ADDR_ANY, PORT_R);

#ifdef ENABLE_UART
  UARTprintf("UDP to receive at port %d...\n", PORT_R);
#endif
  
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

#endif
