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
#include <stdint.h>
#include"MCAL/gpio.h"
#include "MCAL/board.h"
#include "drv_I2C.h"

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



typedef struct{
	float x;
	float y;
	float z;
}rawdata_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


SENSOR_CONTROL initSensor(void);
SENSOR_CONTROL configSensor(void);

rawdata_t getMagData(void);
rawdata_t getAccData(void);
void callbackRead(void);
void ReadAccelMagnData(void);
I2C_COM_CONTROL getStatus(void);
 
#endif /*  DRIVERS_DRV_FXOS8700CQ_H_ */
