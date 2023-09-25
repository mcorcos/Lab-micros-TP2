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

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


 typedef enum{
	 CAN_READY,
	 CAN_ERR

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




 typedef enum{
	 CAN_OSC_CLOCK = 0,
	 CAN_PERIPHERAL_CLOCK = 1
 }CAN_CLOCKS;
 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

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
 void transmitCan(uint8_t MB_ID, uint8_t id, uint8_t buffer[],uint8_t cantBytes);

 
 
#endif /*  PROTOCOLS_CAN_H_ */
