
/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/


 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define BAUDRATE_DEFAULT 9600
#define DEFAULT_ID 3

#define SIZE_OF_PACKAGE 1
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef struct{
	char data = '0';
}package_t;
/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

 void initUART(uint9_t id, uart_cfg_t configure){
	 uart_cfg_t config_uart = {BAUDRATE_DEFAULT,0};
	 uartInit(DEFAULT_ID, config_uart);
 }
 package_t receivePackage(void){
	 package_t package;
	 if(uartIsRxMsg(DEFAULT_ID)){
		 if(uartGetRxMsgLength(DEFAULT_ID) >= SIZE_OF_PACKAGE  ){
			 uartReadMsg(DEFAULT_ID,package.data, SIZE_OF_PACKAGE);
		 }
	 }
	 return package;

 }
 void sendPackage(package_t package){
	 if(uartIsTxMsgComplete(DEFAULT_ID)){
		 uartWriteMsg(DEFAULT_ID, (char*)&package, PACKAGE_SIZE);
	 }
	 else{

	 }
 }
