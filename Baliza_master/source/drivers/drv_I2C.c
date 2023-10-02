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
 
 

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
static I2C_CONFIG i2cConfig;
/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 
 
void initI2c(void){
	i2cDefaultConfig(&i2cConfig,FXOS8700CQ_SLAVE_ADDR,  48800); //48800Hz. El callback es el que lee los datos desde el  chip
	initI2C(&i2cConfig);
}


void loadCallback(ptrToFun callback_){
	i2cConfig.callback = callback_;
}

void i2cCommunicationHandler(uint8_t adress_register_,uint8_t * data_,uint8_t size,I2C_MODE mode,ptrToFun callback){



	if(size==1){

		i2cConfig.dataSize = size;
		i2cConfig.address_register = adress_register_;
		i2cConfig.data[0] = (*data_);

		i2cWriteAndRead(i2cConfig, mode); //Leo o escribo segun corresponda
		//Cuando termine , llamo a
		callback();
	}
	else{
		i2cConfig.address_register = adress_register_;
		i2cConfig.data[0] = 0;
		i2cConfig.dataSize = size;
		i2cStartCommunication(&i2cConfig , mode);
	}
}








