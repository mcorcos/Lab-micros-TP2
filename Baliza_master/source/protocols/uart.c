/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/



/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "MCAL/gpio.h"
#include "protocols/uart.h"
#include "hardware.h"

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define BAUD_RATE_DEFAULT 9600
#define BUFFER_SIZE 32

#define UART_RX_PIN	16 //Default 16 y 17
#define UART_TX_PIN	17

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef struct {
	char buffer[BUFFER_SIZE];
	char* buffer_ptr;
	uint32_t len;
	bool done;
} buffer_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

static const IRQn_Type IRQn[] = {UART0_RX_TX_IRQn, UART1_RX_TX_IRQn, UART2_RX_TX_IRQn, UART3_RX_TX_IRQn, UART4_RX_TX_IRQn, UART5_RX_TX_IRQn};
static uint32_t SIM_MASK[] = {SIM_SCGC4_UART0_MASK, SIM_SCGC4_UART1_MASK, SIM_SCGC4_UART2_MASK, SIM_SCGC4_UART3_MASK, SIM_SCGC1_UART4_MASK , SIM_SCGC1_UART5_MASK };
static SIM_Type* SIM_PTR = SIM;
static  UART_Type * UART_POINTERS[] = UART_BASE_PTRS;
static PORT_Type* PORT_PTRS[] =  { PORTC, PORTC, PORTC, PORTC, PORTC , PORTC };

static buffer_t rx[];
static buffer_t tx[];

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
void uartInit (uint8_t id, uart_cfg_t config){

	UART_Type* const uart_p = PORT_PTRS[id];
	PORT_Type* const port = UART_POINTERS[id] ;

	//Clock Gating
	if((id >= 0) && (id<UART_CANT_IDS)){
		if(id<4){
				SIM_PTR->SCGC4 |= SIM_MASK[id];
		}
		else{
				SIM_PTR->SCGC1 |= SIM_MASK[id];
		}
	}

	NVIC_EnableIRQ(IRQn[id]);

	//UART0 Set UART Speed
	UART_SetBaudRate(uart_p, config.baudrate);

	//Enable UART0 Xmiter and Rcvr
	uart_p->C2=UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK; //     transmiter full interrupt // receiver full interrupt //  receiver full interrupt
	//Que hace nose
	uart_p->C5 &= ~UART_C5_TDMAS_MASK;

	//SETEO LOS PCR

	port->PCR[UART_RX_PIN(id)] = (uint32_t)0; //clear
	port->PCR[UART_RX_PIN(id)] = PORT_PCR_MUX(PORT_mAlt3); //Alt3
	port->PCR[UART_TX_PIN(id)] = (uint32_t)0; //clear
	port->PCR[UART_TX_PIN(id)] = PORT_PCR_MUX(PORT_mAlt3); //Alt3

	// creo los buffers
		for(int id=0; id<UART_CANT_IDS; id++){
			rx[id].buffer_ptr = rx[id].buffer;
			tx[id].buffer_ptr = tx[id].buffer;
			tx[id].done = true;
		}


}


uint8_t uartIsRxMsg(uint8_t id){
	return rx[id].done;
}


uint8_t uartGetRxMsgLength(uint8_t id){
	return rx[id].len;
}


uint8_t uartReadMsg(uint8_t id, char* msg, uint8_t cant){


	if (rx[id].read == false) {
			return 0;
		}

	for(int i=0 ; i<cant && i<(rx[id].len); i++) {
		*(msg+i) = *rx[id].reader_ptr;
		rx[id].reader_ptr++;
		rx[id].len--;
		if(rx[id].reader_ptr - rx[id].buffer >= BUFFER_SIZE) {
			rx[id].reader_ptr = rx[id].buffer;
		}
	}

}

