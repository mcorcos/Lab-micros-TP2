/***************************************************************************//**
  @file     gpio.h
  @brief    Simple GPIO Pin services, similar to Arduino
  @author   g4
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "IRQ.h"
#include "gpio.h"

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
static PORT_Type* PORT_PTRS[] = PORT_BASE_PTRS;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

extern pinIrqFun_t PTRS_TO_FUN[PORT_PCR_COUNT*PORT_COUNT] ;

/**
 * @brief Clears ISF Flag
 * @param pin the pin whose ISF you want to clear
 * @param mode INPUT, OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN.
 */
void PORT_ClearInterruptFlag (uint8_t pin_num,uint8_t PORT){
	PORT_Type* port = PORT_PTRS[PORT] ;
	port->PCR[pin_num] |=   1<<PORT_PCR_ISF_SHIFT;
}

__ISR__ PORTA_IRQHandler(void){
	uint8_t pin_num = FindPinNum(PA);
	PORT_ClearInterruptFlag(pin_num,PA);
	 ( *PTRS_TO_FUN[(0*PORT_PCR_COUNT) + pin_num] )();
}
__ISR__ PORTB_IRQHandler(void){
	uint8_t pin_num = FindPinNum(PB);
	PORT_ClearInterruptFlag(pin_num,PB);
	( *PTRS_TO_FUN[(1*PORT_PCR_COUNT) + pin_num] )();
}
__ISR__ PORTC_IRQHandler(void){
	uint8_t pin_num = FindPinNum(PC);
	PORT_ClearInterruptFlag(pin_num,PC);
	( *PTRS_TO_FUN[(2*PORT_PCR_COUNT) + pin_num] )();
}
__ISR__ PORTD_IRQHandler(void){
	uint8_t pin_num = FindPinNum(PD);
	PORT_ClearInterruptFlag(pin_num,PD);
	( *PTRS_TO_FUN[(3*PORT_PCR_COUNT) + pin_num] )();
}
__ISR__ PORTE_IRQHandler(void){
	uint8_t pin_num = FindPinNum(PE);
	PORT_ClearInterruptFlag(pin_num,PE);
	( *PTRS_TO_FUN[(4*PORT_PCR_COUNT) + pin_num] )();
}




/**
 * @brief PORTS iRQ handler, its called automatically
 * @param PORT PORT in which the interruption has happened
 * @returns return pin
 */

uint8_t FindPinNum(uint8_t PORT){

	PORT_Type* port = PORT_PTRS[PORT] ;
	uint8_t pin_num=0;
	uint32_t ISFR = port->ISFR;
	for(;pin_num<32;pin_num++){
		ISFR = ISFR >>1;
		if(ISFR == 0 ){ // es el pin que busco. el isfr es un arrego de 32 bits siendo cada uno de ellos el pin que se levanto
			return pin_num;
		}
	}
	return -1;



}



