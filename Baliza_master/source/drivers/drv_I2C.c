/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <drivers/drv_I2C.h>

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
 
 
static I2C_CONFIG i2cConfig;
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 
 
void initI2c(void){
	i2cDefaultConfig(FXOS8700CQ_SLAVE_ADDR,  48800); //48800Hz. El callback es el que lee los datos desde el  chip
	initI2C();
}


void loadCallback(ptrToFun callback_){
	i2cLoadCallback(callback_);
}

void i2cCommunicationHandler(uint8_t adress_register_,uint8_t * data_,uint8_t size,I2C_MODE mode,ptrToFun callback){



	if(size==1){


		i2cWriteAndRead( mode , adress_register_ , data_ , size); //Leo o escribo segun corresponda
		//Cuando termine , llamo a
		callback();
	}
	else{

		i2cWriteAndRead( mode , adress_register_ , data_ , size); //Leo o escribo segun corresponda
	}
}








