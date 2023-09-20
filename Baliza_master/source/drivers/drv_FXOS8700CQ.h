/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
  @date 	Sep 18, 2023
 
 ******************************************************************************/

#ifndef DRIVERS_DRV_FXOS8700CQ_H_
#define DRIVERS_DRV_FXOS8700CQ_H_


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "timers\timer.h"
#include "protocols\I2C.h"

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
 
 

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef enum{
	WORKING,
	FINISHED,
}I2C_COM_CONTROL;

typedef enum{
	SENSOR_ERROR,
	SENSOR_READY,
	SENSOR_OK,
	SENSOR_INITIALIZED,
	SENSOR_NOT_INITIALIZED,
}SENSOR_CONTROL;
/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


void initSensor(void);
SENSOR_CONTROL configSensor(void);






 
#endif /*  DRIVERS_DRV_FXOS8700CQ_H_ */
