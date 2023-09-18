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
 
 
 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
//Pines SDA y SCL
#define PIN_SDA 25
#define PIN_SCL 24

 
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef struct{
    int address;
    int acknowledge;
}slave_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 /**
  * @brief Initializer for the I2C.
  */
void initI2C(void);

 /**
  * @brief Read for I2C.
  * @return returns information that is read.
  */
int readI2C(void);

 /**
  * @brief Write for I2C.
  * @param information that is written.
  */
void writeI2C(int information);
 
 
#endif // _I2C_H_
 
 
 
 
 
 
 
 
 