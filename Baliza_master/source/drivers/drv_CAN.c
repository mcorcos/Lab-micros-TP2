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
#define BASE_ID 256
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
static canFrame_t MbBuffer[CAN_ID_COUNT];
static uint8_t iterator  = 0 ;
void callback(canFrame_t *frame);
static 	canFrame_t frameTx;
/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 
CAN_STATUS initBoardsCan(void){


	canConfig_t config;

	defaultCANConfig(&config);


	if(initCAN(&config)!=CAN_READY){
		return CAN_ERR;
	}

	configRxMB(MY_MB_INDEX  ,MY_ID );
	enableCanInterrup( MY_MB_INDEX ,callback);

	configRxMB(MY_MB_INDEX +1 ,MY_ID + 1);
	enableCanInterrup( MY_MB_INDEX +1,callback);

	configRxMB(MY_MB_INDEX +2 ,MY_ID +2 );
	enableCanInterrup( MY_MB_INDEX+2 ,callback);


	configRxMB(MY_MB_INDEX +4 ,MY_ID - 1);
	enableCanInterrup( MY_MB_INDEX +4,callback);

	configRxMB(MY_MB_INDEX +5 ,MY_ID -2 );
	enableCanInterrup( MY_MB_INDEX+5 ,callback);

	configRxMB(MY_MB_INDEX +6 ,MY_ID -3);
	enableCanInterrup( MY_MB_INDEX +6,callback);

	configRxMB(MY_MB_INDEX +7 ,MY_ID -4);
	enableCanInterrup( MY_MB_INDEX +7,callback);
}













 uint8_t sendCan(packageCan_t *package){



	 frameTx.ID  = 0x104;

	 frameTx.dataByte0 = package[0].dataType[0];
	 frameTx.dataByte1 = package[0].sign;
	 frameTx.dataByte2 = package[0].value[0];
	 frameTx.dataByte3 = package[0].value[1];
	 frameTx.dataByte4 = package[0].value[2];


	 frameTx.length = 5;

	// writes TX MB to transmir CAN
	STATUS_TRANSMIT status = transmitCan(MY_MB_INDEX,&frameTx);
	if(status == TRANSMIT_OK){
		return CAN_TRANSMIT_OK;
	}
	else{
		return CAN_TRANSMIT_FAIL;
	}


 }


 uint8_t receiveCAN(Measurement *measurements){



	measurements->boardID = MbBuffer[iterator++].ID;
	measurements->rolling = MbBuffer[iterator++].dataByte0; //dummy , desp hay q igualar todo bien
	measurements->tilt = MbBuffer[iterator++].dataByte0;
	measurements->orientation = MbBuffer[iterator++].dataByte0;



	if(iterator == CAN_ID_COUNT ){
		iterator = 0;

	}

	return CAN_RECEIVE_OK;


 }


 void callback(canFrame_t *frame){

	 uint16_t id = frame->ID - BASE_ID;

	 MbBuffer[id] = *frame;

 }


