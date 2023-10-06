/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/


#ifndef _DRV_UART_H_
#define _DRV_UART_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "source/protocols/uart.h"

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define BAUDRATE_DEFAULT 9600
#define DATA_TYPE_SIZE 1
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct{
 	char dataType[DATA_TYPE_SIZE]; // el identificador
 	uint8_t value[3]; // el valor  ... 255 -> '2' '5' '5'
 					//  valor , valor 2 , cal

 }package_t;


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 void initUART(void);
 package_t receivePackage(void);
 void sendPackage(package_t package);


#endif // _DRV_UART_H_








