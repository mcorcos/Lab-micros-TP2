
/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "drv_UART.h"

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


#define DEFAULT_ID 0

#define SIZE_OF_PACKAGE 1
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

 void initUART(uint8_t id, uart_cfg_t configure){
	 uartInit(id, configure);
 }

 package_t receivePackage(void){
	 package_t package;
	 if(uartIsRxMsg(DEFAULT_ID)){
		 if(uartGetRxMsgLength(DEFAULT_ID) >= SIZE_OF_PACKAGE  ){
			 uartReadMsg(DEFAULT_ID,package.data, SIZE_OF_PACKAGE);
		 }
	 }
	 return package;

 }

 void sendPackage(package_t package){
	 if(uartIsTxMsgComplete(DEFAULT_ID)){
		 uartWriteMsg(DEFAULT_ID, package.data, SIZE_OF_PACKAGE);
	 }
	 else{


	 }


 }
