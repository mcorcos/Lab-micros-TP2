/***************************************************************************//**
  @file     drv_I2C.c
  @brief    Board management
  @author   G4
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "drv_I2C.h"

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
//ID con el que vamos a trabajar
#define DEFAULT_ID 0

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

 void initUART(void){
	 uint8_t id = DEFAULT_ID;
	 I2CInit(id);
 }

 package_t receivePackage(void){
	 package_t package;
	 if(uartIsRxMsg(DEFAULT_ID)){
		 if(uartGetRxMsgLength(DEFAULT_ID) >= SIZE_OF_PACKAGE  ){
			 uartReadMsg(DEFAULT_ID,package.dataType, SIZE_OF_PACKAGE);
		 }
	 }
	 return package;

 }

 void sendPackage(package_t package){
	 if(uartIsTxMsgComplete(DEFAULT_ID)){
		 uartWriteMsg(DEFAULT_ID, package.dataType, SIZE_OF_PACKAGE);
	 }
	 else{


	 }


 }
