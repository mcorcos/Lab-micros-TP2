/***************************************************************************//**
  @file     uart.c
  @brief    UART driver
  @author   G4
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "MCAL/gpio.h"
#include "protocols/uart.h"
#include "hardware.h"
#include "MK64F12.h" //Tiene el typedef UART_Type

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef struct {
	char buffer[BUFFER_SIZE];
	char* buffer_ptr;
	char* reader_ptr;
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
static PORT_Type* PORT_PTRS[] =  { PORTB, PORTB, PORTB, PORTB, PORTB , PORTB };

static buffer_t rx[UART_CANT_IDS];
static buffer_t tx[UART_CANT_IDS];


/*******************************************************************************
 *  FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
/**
 * @brief Sets UART BaudRate.
 * @param uart pointer to uart structure to set baudrate.
 * @param baudrate chosen baudrate to set.
 * @return void.
 */
void UART_SetBaudRate(UART_Type *uart, uint32_t baudrate);

/*******************************************************************************
 *  GLOBAL FUNCTIONS DEFINITIONS
 ******************************************************************************/
void uartInit (uint8_t id , uart_cfg_t config){

	PORT_Type* const port = PORT_PTRS[id];
	UART_Type* const uart_p = UART_POINTERS[id] ;
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
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



	//SETEO LOS PCR

	port->PCR[UART_RX_PIN] = 0x0; //clear
	port->PCR[UART_RX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); //Alt3
	port->PCR[UART_TX_PIN] = 0x0; //clear
	port->PCR[UART_TX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); //Alt3

	//UART0 Set UART Speed
	UART_SetBaudRate(uart_p, config.baudrate);

	//Enable UART0 Xmiter and Rcvr
	uart_p->C2=UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK; //     transmiter full interrupt // receiver full interrupt //  receiver full interrupt
	//Que hace nose
	uart_p->C5 &= ~UART_C5_TDMAS_MASK;

	// creo los buffers
		for(int id=0; id<UART_CANT_IDS; id++){
			rx[id].buffer_ptr = rx[id].buffer;
			rx[id].reader_ptr = rx[id].buffer;
			tx[id].buffer_ptr = tx[id].buffer;
			tx[id].reader_ptr = tx[id].buffer;
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


	if (rx[id].done == false) {
			return 0;
		}

	uint32_t len = rx[id].len;

	for(int i=0 ; i<cant && i<len; i++) {
		msg[i] = *rx[id].reader_ptr;  //copio miembro a miembro
		rx[id].reader_ptr++; //aumento el puntero de mi buffer RX
		if(rx[id].reader_ptr - rx[id].buffer >= BUFFER_SIZE) { //
			rx[id].reader_ptr = rx[id].buffer;
		}
		rx[id].len--; // pues disminuye la longitud del buffer
	}
	rx[id].done = false;
	// me fijo si se mando la cantidad esperada
	if(len>cant){
		return cant;
	}
	else{
		return len;
	}
}





uint8_t uartWriteMsg(uint8_t id, const char* msg, uint8_t cant){
	UART_Type* const uart_p = UART_POINTERS[id];
	uint16_t i=0;
	uint32_t len = tx[id].len;
	for(i=0; i < cant && i< BUFFER_SIZE - len;i++){	//Lo que pase primero, que se llegue a cant o a
		*tx[id].reader_ptr = msg[i];				//llenar el buffer
		tx[id].reader_ptr++;
		tx[id].len++;
		if(tx[id].reader_ptr - tx[id].buffer >= BUFFER_SIZE){
			tx[id].reader_ptr = tx[id].buffer;
		}
	}
	tx[id].done = false;
	//enable transmition
	uart_p->C2 |= UART_C2_TIE_MASK;
	return i;


}


uint8_t uartIsTxMsgComplete(uint8_t id){

	return tx[id].done;

}


/*---------------------------------------------------------------------------------------
 *
 * LOCAL FUNCTIONS
 *
 * -----------------------------------------------------------*/




void UART_SetBaudRate (UART_Type *uart, uint32_t baudrate)
{
	uint16_t sbr, brfa;
	uint32_t clock;

	clock = ((uart == UART0) || (uart == UART1))?(__CORE_CLOCK__):(__CORE_CLOCK__ >> 1); //LOS dso primeros uart usan clock , los otros clock/2

	//baudrate = ((baudrate == 0)?(UART_HAL_DEFAULT_BAUDRATE):
	//	((baudrate > 0x1FFF)?(UART_HAL_DEFAULT_BAUDRATE):(baudrate)));

	sbr = clock / (baudrate << 4);               // sbr = clock/(Baudrate x 16)
	brfa = (clock << 1) / baudrate - (sbr << 5); // brfa = 2*Clock/baudrate - 32*sbr

	uart->BDH = UART_BDH_SBR(sbr >> 8);
	uart->BDL = UART_BDL_SBR(sbr);
	uart->C4 = (uart->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}



void UARTX_IRQ_HANDLER(uint8_t id){

	UART_Type* const uart_p = UART_POINTERS[id];

	//interrupcion fue por el transmisor
	if ((uart_p->S1 & UART_S1_TDRE_MASK) == UART_S1_TDRE_MASK ) {
		if(tx[id].reader_ptr != tx[id].buffer_ptr){
			uart_p->D = *tx[id].buffer_ptr; // Escribo en el registro
			tx[id].buffer_ptr++;
			tx[id].len--;
			if(tx[id].buffer_ptr - tx[id].buffer == BUFFER_SIZE) {
				tx[id].buffer_ptr = tx[id].buffer;
			}
		}
		else {
			tx[id].done = true;
			// disable transmission
			uart_p->C2 &= ~UART_C2_TIE_MASK;
			//UARTX->C2 |= UART_C2_RIE_MASK;
		}


	}

	//interrupcion fue por el receptor
	if ( (uart_p->S1 & UART_S1_RDRF_MASK) == UART_S1_RDRF_MASK ) {
		// reading S1 with RDRF set and then reading D clears S1[RDRF]
		*rx[id].buffer_ptr = uart_p->D; // Guardo el byte recibido
		rx[id].buffer_ptr++;
		rx[id].len = (rx[id].len + 1) % BUFFER_SIZE; //Buffer circular
		if(rx[id].buffer_ptr - rx[id].buffer == BUFFER_SIZE){
			rx[id].buffer_ptr = rx[id].buffer;
		}
		rx[id].done = true;
		//uart_p->C2 &= ~UART_C2_RIE_MASK;
	}
}
/*
// reading S1 with TDRE set clears S1[TDRE]
        if(tx[id].writer_ptr - tx[id].buffer_ptr != 0){
            UARTX->D = *tx[id].buffer_ptr; // Transmito
            tx[id].buffer_ptr++;
            tx[id].len--;
            if(tx[id].buffer_ptr - tx[id].buffer == BUFFER_SIZE) {
                tx[id].buffer_ptr = tx[id].buffer;
            }
        }
        else {
            tx[id].done = true;
            // disable transmission
            UARTX->C2 &= ~UART_C2_TIE_MASK;
            //UARTX->C2 |= UART_C2_RIE_MASK;
        }
*/
//Hago el control de interrupcinoes
__ISR__ UART0_RX_TX_IRQHandler(void){
	UARTX_IRQ_HANDLER(0);
}
__ISR__ UART1_RX_TX_IRQHandler(void){
	UARTX_IRQ_HANDLER(1);
}
__ISR__ UART2_RX_TX_IRQHandler(void){
	UARTX_IRQ_HANDLER(2);
}
__ISR__ UART3_RX_TX_IRQHandler(void){
	UARTX_IRQ_HANDLER(3);
}
__ISR__ UART4_RX_TX_IRQHandler(void){
	UARTX_IRQ_HANDLER(4);
}
__ISR__ UART5_RX_TX_IRQHandler(void){
	UARTX_IRQ_HANDLER(5);
}
