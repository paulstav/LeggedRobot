#ifndef __MAIN_H__
#define __MAIN_H__

//#define ENABLE_ETHERNET
//#define ENABLE_UART
//#define ENABLE_IMU
//#define ENABLE_MOTOR


//#define BOARD_FL
//#define BOARD_FR
#define BOARD_BL
//#define BOARD_BR

#if defined(BOARD_FL)
#define PORT_S 2014
#define PORT_R 2013
#elif defined(BOARD_FR)
#define PORT_S 2016
#define PORT_R 2015
#elif defined(BOARD_BL)
#define PORT_S 2018
#define PORT_R 2017
#elif defined(BOARD_BR)
#define PORT_S 2020
#define PORT_R 2019
#else
#define PORT_S 2012
#define PORT_R 2011
#endif

#define DECIMATION_COEF         0x3000

#endif