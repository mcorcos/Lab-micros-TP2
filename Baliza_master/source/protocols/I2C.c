/***************************************************************************//**
  @file     I2C.c
  @brief    I2C protocol
  @author   G4
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "I2C.h"
#include "MCAL/gpio.h"
#include "hardware.h"
#include "MK64F12.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
 //Defines de modo de I2C
 #define MASTER 0
 #define SLAVE  1
 
 //Frames 
#define WRITE   0
#define READ    1

//Cantidad de slaves y masters 
#define I2C_CANT_SLAVES 1
#define I2C_CANT_MASTERS 1

//Defines para el INIT
#define I2C_CANT_IDS 2 //Tenemos I2C 0, 1 y 2

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
//Posibles estados
enum {BUS_FREE, START, ADDRESS, FRAME, SEND_BYTES, STOP, REPEATED_START};

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
static PORT_Type* PORT_PTRS[] =  {PORTE, PORTE};
static  I2C_Type * I2C_POINTERS[] = I2C_BASE_PTRS;
static SIM_Type* SIM_PTR = SIM;
static uint32_t SIM_MASK[] = {SIM_SCGC4_I2C0_MASK, SIM_SCGC4_I2C1_MASK, SIM_SCGC1_I2C2_MASK};
static const IRQn_Type IRQn[] = {I2C0_IRQn, I2C1_IRQn, I2C2_IRQn};


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH LOCAL SCOPE
 ******************************************************************************/
 

 /*******************************************************************************
 * FUNCTION DEFINITIONS WITH GLOBAL SCOPE
 *******************************************************************************/
void initI2C(uint8_t id){
    PORT_Type* const port = PORT_PTRS[id];
	I2C_Type* const i2c_ptr = I2C_POINTERS[id] ;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
    //Clock Gating
	if((id >= 0) && (id<I2C_CANT_IDS)){
		if(id==2){
				SIM_PTR->SCGC1 |= SIM_MASK[id];
		}
		else{
				SIM_PTR->SCGC4 |= SIM_MASK[id];
		}
	}

	NVIC_EnableIRQ(IRQn[id]);

	//SETEO LOS PCR
	port->PCR[PIN_SDA] = 0x0; //clear
	port->PCR[PIN_SDA] |= PORT_PCR_MUX(PORT_mAlt3); //Alt3
	port->PCR[PIN_SCL] = 0x0; //clear
	port->PCR[PIN_SCL] |= PORT_PCR_MUX(PORT_mAlt3); //Alt3

	//Enable UART0 Xmiter and Rcvr
	//uart_p->C2=UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK; //     transmiter full interrupt // receiver full interrupt //  receiver full interrupt
	//Que hace nose
	//uart_p->C5 &= ~UART_C5_TDMAS_MASK;

}

int readI2C(void){

}

void writeI2C(int information){ //Puse int pero para poner algo :P

}



void i2cCommunication(uint8_t slave_adress,uint8_t slave_register,I2CBYTE * databyte,uint8_t len ,uint8_t mode , ptrToFun  callback_){

}

 /*******************************************************************************
 * FUNCTION DEFINITIONS WITH LOCAL SCOPE
 ******************************************************************************/
