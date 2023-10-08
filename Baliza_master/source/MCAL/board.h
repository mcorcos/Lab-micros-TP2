/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _BOARD_H_
#define _BOARD_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "gpio.h"




/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/***** BOARD defines **********************************************************/

//Defino todos los pines de la placa

#define PIN_PTC_10		PORTNUM2PIN(PC,10)
#define PIN_PTC_11		PORTNUM2PIN(PC,11)
#define PIN_PTC_13		PORTNUM2PIN(PC,13)


#define PIN_PTB_10		PORTNUM2PIN(PC,10)
#define PIN_PTB_11		PORTNUM2PIN(PB,11)
#define PIN_PTB_18		PORTNUM2PIN(PB,18)
#define PIN_PTB_19		PORTNUM2PIN(PB,19)
#define PIN_PTB_24		PORTNUM2PIN(PB,24)
#define PIN_PTB_25		PORTNUM2PIN(PB,25)


#define PIN_PTE_24		PORTNUM2PIN(PE,24)
#define PIN_PTE_25		PORTNUM2PIN(PE,25)



// On Board User LEDs
#define PIN_LED_RED     PORTNUM2PIN(PB,22)
#define PIN_LED_GREEN   PORTNUM2PIN(PE,26)
#define PIN_LED_BLUE    PORTNUM2PIN(PB,21) // PTB21

//Para el uso de los leds de la kinetis
#define LED_ACTIVE      LOW
#define LED_OFF			HIGH

//Para el uso de los leds de la DEVBOARD
#define LED_ACTIVE_DEVBOARD	HIGH
#define LED_OFF_DEVBOARD	LOW


// On Board User Switches
#define PIN_SW2         PORTNUM2PIN(PC,6)
#define PIN_SW3         PORTNUM2PIN(PA,4)


//UART
#define UART0_TX_PIN 	17   //PTB17
#define UART0_RX_PIN 	16   //PTB16

//CAN

#define CAN0_RX_PIN	PIN_PTB_19
#define CAN0_TX_PIN PIN_PTB_18


//I2C
#define PIN_INT2_FXOS8700XQ	PIN_PTC_13
#define PIN_I2C_SCL PIN_PTE_24
#define PIN_I2C_SDA PIN_PTE_25


//DEBUG ( pines para la DEVBOARD)

#if DEBUG_MODE
#define DEBUG_PIN_1 PIN_PTB_10
#define DEBUG_PIN_2 PIN_PTB_11
#define DEBUG_PIN_3 PIN_PTC_11
#define DEBUG_PIN_4 PIN_PTC_10
#endif

#if ERROR_MODE
#define DEBUG_PIN_1 PIN_PTB_10
#define DEBUG_PIN_2 PIN_PTB_11
#define ERROR_PIN_1 PIN_PTC_11
#define ERROR_PIN_2 PIN_PTC_10
#endif



 // FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR 0x1D



// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS 		0x00
#define FXOS8700CQ_OUT_X_MSB 	0x01
#define FXOS8700CQ_WHOAMI 		0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 	0x2A
#define FXOS8700CQ_CTRL_REG2 	0x2B
#define FXOS8700CQ_M_CTRL_REG1 	0x5B
#define FXOS8700CQ_M_CTRL_REG2 	0x5C
#define FXOS8700CQ_M_CTRL_REG4	0x2D   // Interrupts enable
#define FXOS8700CQ_M_CTRL_REG5 	0x2E   //rputing of the interrupts on INT1/2
#define FXOS8700CQ_WHOAMI_VAL 	0xC7


#define DATA_READY_ENABLE 0b00000001
#define INTERRUPT_ROUTED_INT2 0b00000000

// number of bytes to be read from the FXOS8700CQ
#define FXOS8700CQ_READ_LEN 	13







/*******************************************************************************
 ******************************************************************************/

#endif // _BOARD_H_
