/***************************************************************************//**
  @file     gpio.h
  @brief    Simple GPIO Pin services, similar to Arduino
  @author   Nicol�s Magliola
 ******************************************************************************/

#ifndef _GPIO_H_
#define _GPIO_H_



/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "MK64F12.h"
#include "hardware.h"






/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
// Convert port and number into pin ID
// Ex: PTB5  -> PORTNUM2PIN(PB,5)  -> 0x25
//     PTC22 -> PORTNUM2PIN(PC,22) -> 0x56
#define PORTNUM2PIN(p,n)    (((p)<<5) + (n))
#define PIN2PORT(p)         (((p)>>5) & 0x07)
#define PIN2NUM(p)          ((p) & 0x1F)




//Puertos
#define PORT_COUNT 5


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
// Ports
enum { PA, PB, PC, PD, PE };

typedef void (*pinIrqFun_t)(void);


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


/**
 * @brief Clears ISF Flag
 * @param pin the pin whose ISF you want to clear
 * @param mode INPUT, OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN.
 */
void PORT_ClearInterruptFlag (uint8_t pin_num,uint8_t PORT);



/**
 * @brief PORTS iRQ handler, its called automatically
 * @param void
 * @returns interruption Void
 */

__ISR__ PORTA_IRQHandler(void);
/**
 * @brief PORTS iRQ handler, its called automatically
 * @param void
 * @returns interruption Void
 */
__ISR__ PORTB_IRQHandler(void);
/**
 * @brief PORTS iRQ handler, its called automatically
 * @param void
 * @returns interruption Void
 */
__ISR__ PORTC_IRQHandler(void);
/**
 * @brief PORTS iRQ handler, its called automatically
 * @param void
 * @returns interruption Void
 */
__ISR__ PORTD_IRQHandler(void);
/**
 * @brief PORTS iRQ handler, its called automatically
 * @param void
 * @returns interruption Void
 */
__ISR__ PORTE_IRQHandler(void);



/**
 * @brief PORTS iRQ handler, its called automatically
 * @param PORT PORT in which the interruption has happened
 * @returns return pin
 */

uint8_t FindPinNum(uint8_t PORT);



















#endif // _GPIO_H_
