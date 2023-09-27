/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <drivers/drv_CAN.h>


 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
 
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
static canFrame_t MbBuffer[CAN_ID_COUNT];
static uint8_t i =0 ;
void callback(canFrame_t frame);
/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 
CAN_STATUS initBoardsCan(void){


	canConfig_t config;

	defaultCANConfig(&config);


	if(initCAN(&config)!=CAN_READY){
		return CAN_ERR;
	}


	configRxMB(MY_MB_INDEX +1 ,MY_ID + 1);
	enableCanInterrup( MY_MB_INDEX +1,callback);


}













 uint8_t sendCan(Measurement measurements){

	canFrame_t frame;

	frame.ID  = MY_ID;
	frame.dataWord0 = measurements.rolling;
	frame.dataWord1 = measurements.tilt;
	frame.length = 31;

	// writes TX MB to transmir CAN
	STATUS_TRANSMIT status = transmitCan(MY_MB_INDEX,frame);
	if(status == TRANSMIT_OK){
		return CAN_TRANSMIT_OK;
	}
	else{
		return CAN_TRANSMIT_FAIL;
	}


 }


 uint8_t receiveCAN(Measurement measurements){

	canFrame_t frame;

	frame = MbBuffer[i++];

	measurements.boardID = frame.ID;
	measurements.rolling = frame.dataByte0; //dummy , desp hay q igualar todo bien
	measurements.tilt = frame.dataByte0;
	measurements.orientation = frame.dataByte0;



	if(i == CAN_ID_COUNT ){
		i = 0;

	}

	return CAN_RECEIVE_OK;


 }


 void callback(canFrame_t frame){

	 uint8_t id = frame.ID;

	 MbBuffer[id] = frame;

 }


