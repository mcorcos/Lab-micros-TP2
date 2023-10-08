/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
  @date 	Sep 14, 2023
 
 ******************************************************************************/

#ifndef PROTOCOLS_CAN_H_
#define PROTOCOLS_CAN_H_



 /**
  * @brief CAN driver
  * FREEDOM k64 CAN driver.Uses CAN0

 */

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdint.h>
#include "MCAL/board.h"
 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define MAX_BYTES 8

#define MY_ID (0x104) //104
#define MY_MB_INDEX 4
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
 typedef enum{ // Los codigos de cada MB. leer pagina 1432 para mas info
 	INACTIVE_RX = 0b0000,
 	 EMPTY_RX    = 0b0100,
 	 FULL_RX     = 0b0010,
 	 OVERRUN_RX  = 0b0011,
 	 ANSWER_RX  = 0b1010,
 	 BUSY_RX     = 0b0001
 }RX_CODE;

 typedef enum{
 	INACTIVE_TX = 0b1000,
	ABORT_TX    = 0b1001,
 	ACTIVE_TX = 0b1100,
	TANSWER_TX  = 0b1110

 }TX_CODES;

 typedef enum{
 	READ_FAIL,
 	READ_OK
 }STATUS_READ;

 typedef enum{
 	TRANSMIT_FAIL,
	TRANSMIT_OK
 }STATUS_TRANSMIT;

 typedef enum{
	 CAN_READY,
	 CAN_ERR,
	 CAN_TRANSMIT_FAIL,
	 CAN_TRANSMIT_OK,
	 CAN_RECEIVE_FAIL,
	 CAN_RECEIVE_OK
 }CAN_STATUS;
 
 typedef struct{
	uint8_t PRESDIV;
	uint8_t PROPSEG;
	uint8_t PSEG1;
	uint8_t PSEG2;

 }canTiming_t;

 typedef struct{
	uint8_t clock;
	uint32_t baudrate;
	canTiming_t timing;

 }canConfig_t;


typedef struct{
	uint32_t ID;
	uint32_t length;
	union{
		struct{
			uint32_t dataWord0;
			uint32_t dataWord1;
		};
		struct{
			uint8_t dataByte0;
			uint8_t dataByte1;
			uint8_t dataByte2;
			uint8_t dataByte3;
			uint8_t dataByte4;
			uint8_t dataByte5;
			uint8_t dataByte6;
			uint8_t dataByte7;
		};
		uint8_t data[8];
	};
}canFrame_t;

 typedef enum{
	 CAN_OSC_CLOCK = 0,
	 CAN_PERIPHERAL_CLOCK = 1
 }CAN_CLOCKS;




 typedef void (*ptrToFunction)(canFrame_t* frame, CAN_STATUS s);


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

 /**
  * @brief Initialize CAN driver
  * @param configuration of CAN
  * @param Clock source to use
  * @return Status of CAN
 */
 CAN_STATUS initCAN( canConfig_t * config);


 /**
  * @brief Enable CAN
  *
  */
 void enableCAN();

 /**
  * @brief Disable CAN
  *
  */
 void disableCAN();




 /**
  * @brief transmit CAN Message Buffer
  *
  */
 uint8_t transmitCan(uint8_t MB_ID,canFrame_t *frame);
 
 /**
  * @brief read CAN Message Buffer
  *
  */
 STATUS_READ readCAN(uint8_t MB_ID, canFrame_t *frame);
 
 /**
   * @brief
   *
   */
 void  configRxMB( uint8_t mb_id, uint32_t ID);
 /**
   * @brief
   *
   */
 void defaultCANConfig(canConfig_t * config);



 /**
   * @brief
   *
   */
 void enableCanInterrup( uint8_t mb_id, ptrToFunction callback);

#endif /*  PROTOCOLS_CAN_H_ */
