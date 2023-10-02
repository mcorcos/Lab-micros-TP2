/***************************************************************************//**
  @file     I2C.h
  @brief    I2C protocol
  @author   G4
 ******************************************************************************/
 
 
#ifndef _I2C_H_
#define _I2C_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "MCAL/board.h"
#include "MCAL/gpio.h"
 
 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define PIN_SCL PIN2NUM(PIN_I2C_SCL)
 #define PIN_SDA PIN2NUM(PIN_I2C_SDA)
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef void (* ptrToFun)( void);

typedef uint8_t I2CBYTE;

typedef struct{
    int address;
    int acknowledge;
}slave_t;

typedef enum{
	I2C_READ,
	I2C_WRITE
}I2C_MODE;


typedef enum {
	I2C_STATE_NONE = 0,
	I2C_STATE_WRITE_DATA,
	I2C_STATE_WRITE_ADRESS_REGISTER,
	I2C_STATE_RSTART,
	I2C_STATE_DUMMY,
	I2C_STATE_READ_DATA,
	I2C_STATE_ERROR,
	I2C_STATE_NO_ERROR
} I2C_STATE;

typedef enum{
	FLAG_TRANSMISSION,
	FLAG_OK
}I2C_FLAG;


typedef enum{
	I2C_FAULT_END_READING,
	I2C_FAULT_END_WRITING,
	I2C_FAULT_NO_ACK
}I2C_FAULT;




typedef struct{
	uint8_t address_r;
	uint8_t address_w;
	uint8_t address_register;
	uint8_t state ;
	uint8_t flag;
	uint8_t mode;
	uint8_t fault;
	uint8_t id;
	uint8_t *	data;
	uint8_t	dataSize;
	uint8_t indexData;
	ptrToFun callback;
	uint16_t freq;

}I2C_CONFIG;
/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 /**
  * @brief Initializer for the I2C.
  */
void initI2C(I2C_CONFIG * i2cConfig);

/**
 * @brief Initializer for the I2C dfault config.
 */
void i2cDefaultConfig(I2C_CONFIG * i2cConfig, uint8_t address, uint16_t frequency);



 /**
  * @brief leer o escribir en I2C
  * @return returns state
  */
void i2cWriteAndRead(I2C_CONFIG  i2cConfig, I2C_MODE mode); //Blocking





bool i2cStartCommunication(I2C_CONFIG * i2cConfig , I2C_MODE mode); // orioginalmente era local de I2c.c ,
																			//pero la tengo que sacar para DRV_I2C para poder comenzar una comunicacion y que luego la siga
																			// el handler de interrupts


 
#endif // _I2C_H_
 
 
 
 
 
 
 
 
 
