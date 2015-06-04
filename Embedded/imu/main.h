#ifndef __MAIN_H__
#define __MAIN_H__

// Definition for the Leg to which the board is connected

//#define BOARD_FL
//#define BOARD_FR
//#define BOARD_BL
//#define BOARD_BR
//#define BOARD_IMU
//#define BOARD_FORCE

//#define DEV_ADIS16375
#define DEV_ADIS16364

// Definition of UDP SEnd/Receive ports depending on board

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
#elif defined(BOARD_IMU)
#define PORT_S 2022
#define PORT_R 2021
#elif defined(BOARD_FORCE)
#define PORT_S 2024
#define PORT_R 2023
#else
#define PORT_S 2012
#define PORT_R 2011
#endif

// Decimation coefficient for IMU data rate and Euler angle calculation
#define DECIMATION_COEF         0x3000

#endif

