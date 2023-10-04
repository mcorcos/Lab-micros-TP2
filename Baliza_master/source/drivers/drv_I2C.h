/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
  @date 	Sep 28, 2023
 
 ******************************************************************************/

#ifndef DRIVERS_DRV_I2C_H_
#define DRIVERS_DRV_I2C_H_


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "protocols/I2C.h"
#include "MCAL/board.h"
 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
 
 typedef void (* ptrToFun)( void);

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 void initI2c(void);
 void loadCallback(ptrToFun callback_);
 void i2cCommunicationHandler( uint8_t adress_register_,uint8_t * data,uint8_t size,I2C_MODE mode);
 I2C_FAULT faultGetter(void);



#endif /*  DRIVERS_DRV_I2C_H_ */
