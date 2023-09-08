/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/


#ifndef _drv_K64_H_
#define _drv_K64_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/


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

 void init_K64Leds(void);

 void turnOn_RedLed(void);
 void turnOn_GreenLed(void);
 void turnOn_BlueLed(void);

 void turnOff_RedLed(void);
 void turnOff_GreenLed(void);
 void turnOff_BlueLed(void);

 void toggle_RedLed(void);
 void toggle_GreenLed(void);
 void toggle_BlueLed(void);


#endif // _drv_K64_H_








