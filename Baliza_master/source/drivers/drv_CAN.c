/***************************************************************************//**
  @file     drv_CAN.c
  @brief    Driver for CAN protocol
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
#define IS_ID_OK(id) (((id)<7)&&((id)>=0)&&((id)!=4))

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
static canFrame_t MbBuffer[CAN_ID_COUNT];
void callback(canFrame_t *frame, CAN_STATUS s);
static 	canFrame_t frameTx;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH LOCAL SCOPE
 ******************************************************************************/
uint8_t parsePackage(Measurement *measurements, uint8_t idx_mb_buffer);
int16_t charsToInt16(uint32_t length, uint8_t *chars);

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 
CAN_STATUS initBoardsCan(void){

	canConfig_t config;

	defaultCANConfig(&config);

	if(initCAN(&config)!=CAN_READY){
		return CAN_ERR;
	}

	configRxMB(0, 0x100);
	enableCanInterrup(0, callback);

	configRxMB(1 , 0x101);
	enableCanInterrup(1, callback);

	configRxMB(2, 0x102 );
	enableCanInterrup(2, callback);

	configRxMB(3, 0x103);
	enableCanInterrup(3, callback);

	configRxMB(4, 0x104);
	enableCanInterrup(4, callback);

	configRxMB(5, 0x105);
	enableCanInterrup(5, callback);

	configRxMB(6, 0x106);
	enableCanInterrup(6, callback);


	return CAN_READY;
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


 uint8_t receiveCAN(Measurement *measurements, uint8_t idx_mb_buffer){
	 uint8_t change_r;
	 if(IS_ID_OK((MbBuffer[idx_mb_buffer].ID - 0x100))){
		 //Para obtener el ID en formato de uint16_t y con valores de una unidad se le resta 256 para normalizarlo
		 measurements->boardID = MbBuffer[idx_mb_buffer].ID - 0x100;
		 change_r = parsePackage(measurements, idx_mb_buffer);
	 }
	 return change_r;
 }



 uint8_t parsePackage(Measurement *measurements, uint8_t idx_mb_buffer){
	 uint8_t angleID = MbBuffer[idx_mb_buffer].dataByte0;
	 uint8_t change_p = CHANGE_NONE;
	 switch(angleID){
	 	 case 'R':
	 		measurements->rolling = charsToInt16(MbBuffer[idx_mb_buffer].length, &(MbBuffer[idx_mb_buffer].dataByte1));
	 		change_p = CHANGE_R;
	 		break;
	 	 case 'C':
	 		measurements->tilt = charsToInt16(MbBuffer[idx_mb_buffer].length, &(MbBuffer[idx_mb_buffer].dataByte1));
	 		change_p = CHANGE_C;
	 		break;
	 	 case 'O':
	 		measurements->orientation = charsToInt16(MbBuffer[idx_mb_buffer].length, &(MbBuffer[idx_mb_buffer].dataByte1));
	 		change_p = CHANGE_O;
	 		break;
	 	 default:
	 		change_p = CHANGE_NONE;
	 		 break;
	 }
	 return change_p;
 }

int16_t charsToInt16(uint32_t length, uint8_t * chars) {
	int16_t valor = 0;
	uint8_t contador = 0;
	int16_t signo = 1;
	char dato = chars[contador];
	switch(dato){
		case '+':
			signo = 1;
			contador++;
			break;
		case '-':
			signo = -1;
			contador++;
			break;
		default:
			break;
	}
	for(; contador < length-1; contador++){
		valor = 10*valor + chars[contador] - '0';
	}
	return (valor*signo);
 }

 void callback(canFrame_t *frame, CAN_STATUS s){
	 if(s != READ_FAIL){
		 if(IS_ID_OK((frame->ID)-0x100)){
			 MbBuffer[(frame->ID)-(0x100)] = *frame;
		 }
	 }
 }


