/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/



/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "drivers/drv_DEVBOARD.h"

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/



 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
/*******************************************************************************
 * function PROTOTYPES WITH local SCOPE
 ******************************************************************************/
/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

void init_DEVBOARD(void){

#if DEBUG_MODE
	gpioMode(DEBUG_PIN_1,OUTPUT);
	gpioWrite(DEBUG_PIN_1,LED_OFF_DEVBOARD);

	gpioMode(DEBUG_PIN_2,OUTPUT);
	gpioWrite(DEBUG_PIN_2,LED_OFF_DEVBOARD);

	gpioMode(DEBUG_PIN_3,OUTPUT);
	gpioWrite(DEBUG_PIN_3,LED_OFF_DEVBOARD);

	gpioMode(DEBUG_PIN_4,OUTPUT);
	gpioWrite(DEBUG_PIN_4,LED_OFF_DEVBOARD);

#endif

#if ERROR_MODE
	gpioMode(DEBUG_PIN_1,OUTPUT);
	gpioWrite(DEBUG_PIN_1,LED_OFF_DEVBOARD);

	gpioMode(DEBUG_PIN_2,OUTPUT);
	gpioWrite(DEBUG_PIN_2,LED_OFF_DEVBOARD);

	gpioMode(ERROR_PIN_1,OUTPUT);
	gpioWrite(ERROR_PIN_1,LED_OFF_DEVBOARD);

	gpioMode(ERROR_PIN_2,OUTPUT);
	gpioWrite(ERROR_PIN_2,LED_OFF_DEVBOARD);

#endif
}

//primer pin de error


#if ERROR_MODE
 void turnOn_ErrorLed_1(void){
	 gpioWrite(ERROR_PIN_1,LED_ACTIVE_DEVBOARD);
 }

 void turnOff_ErrorLed_1(void){
	 gpioWrite(ERROR_PIN_1,LED_OFF_DEVBOARD);
 }

 //primer pin de error

  void turnOn_ErrorLed_2(void){
 	 gpioWrite(ERROR_PIN_2,LED_ACTIVE_DEVBOARD);
  }

  void turnOff_ErrorLed_2(void){
 	 gpioWrite(ERROR_PIN_2,LED_OFF_DEVBOARD);
  }

#endif




#if DEBUG_MODE //-------------------

 void turnOn_DebugLed_1(void){
	 gpioWrite(DEBUG_PIN_1,LED_ACTIVE_DEVBOARD);
 }

 void turnOff_DebugLed_1(void){
	 gpioWrite(DEBUG_PIN_1,LED_OFF_DEVBOARD);
 }




 void turnOn_DebugLed_2(void){
	 gpioWrite(DEBUG_PIN_2,LED_ACTIVE_DEVBOARD);
 }

 void turnOff_DebugLed_2(void){
	 gpioWrite(DEBUG_PIN_2,LED_OFF_DEVBOARD);
 }


 void turnOn_DebugLed_3(void){
	 gpioWrite(DEBUG_PIN_3,LED_ACTIVE_DEVBOARD);
 }

 void turnOff_DebugLed_3(void){
	 gpioWrite(DEBUG_PIN_3,LED_OFF_DEVBOARD);
 }

 void turnOn_DebugLed_4(void){
	 gpioWrite(DEBUG_PIN_4,LED_ACTIVE_DEVBOARD);
 }

 void turnOff_DebugLed_4(void){
	 gpioWrite(DEBUG_PIN_4,LED_OFF_DEVBOARD);
 }

#endif //-----------------
