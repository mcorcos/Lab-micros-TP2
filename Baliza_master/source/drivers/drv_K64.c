/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/


#include "MCAL/board.h"
#include "MCAL/gpio.h"
#include <stdbool.h>

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
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


 void init_K64Leds(void){

	 gpioMode (PIN_LED_RED,OUTPUT);
	 gpioWrite(PIN_LED_RED,LED_OFF); //  apago led

	 gpioMode (PIN_LED_GREEN,OUTPUT);
	 gpioWrite(PIN_LED_GREEN,LED_OFF); // apago led

	 gpioMode (PIN_LED_BLUE,OUTPUT);
	 gpioWrite(PIN_LED_BLUE,LED_OFF); //apago led


 }


//Funciones identicas para todos los leds , prender, apagar y toglear leds
// TODO: PRENDER 5_SEC algun LED

 void turnOn_RedLed(void){
	 gpioWrite(PIN_LED_RED,false);
 }
 void turnOn_GreenLed(void){
	 gpioWrite(PIN_LED_GREEN,false);
 }
 void turnOn_BlueLed(void){
	 gpioWrite(PIN_LED_BLUE,false);
 }

 void turnOff_RedLed(void){
	 gpioWrite(PIN_LED_RED,true);
 }
 void turnOff_GreenLed(void){
	 gpioWrite(PIN_LED_GREEN,true);
 }
 void turnOff_BlueLed(void){
	 gpioWrite(PIN_LED_BLUE,true);
 }


 void toggle_RedLed(void){
	 gpioToggle(PIN_LED_RED);
 }

 void toggle_GreenLed(void){
	 gpioToggle(PIN_LED_GREEN);
 }

 void toggle_BlueLed(void){
	 gpioToggle(PIN_LED_BLUE);
 }






