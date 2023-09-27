/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
  @date 	Sep 27, 2023
 
 ******************************************************************************/

#ifndef DRIVERS_DRV_CAN_H_
#define DRIVERS_DRV_CAN_H_


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "protocols/CAN.h"

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
 
 

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef struct
{
	uint16_t boardID;
	uint8_t rolling;
	uint8_t tilt;
	uint8_t orientation;
}Measurement;
/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

CAN_STATUS initBoardsCan(void);
uint8_t receiveCAN(Measurement measurements);
uint8_t sendCan(Measurement measurements);
 
#endif /*  DRIVERS_DRV_CAN_H_ */
