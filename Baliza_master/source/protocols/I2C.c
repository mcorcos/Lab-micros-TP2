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


#define I2C_CLEAR_IRQ_FLAG       (I2C0->S |= I2C_S_IICIF_MASK)

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

static SIM_Type* SIM_PTR = SIM;
static uint32_t SIM_MASK[] = {SIM_SCGC4_I2C0_MASK, SIM_SCGC4_I2C1_MASK, SIM_SCGC1_I2C2_MASK};
static const IRQn_Type IRQn[] = {I2C0_IRQn, I2C1_IRQn, I2C2_IRQn};
static I2C_CONFIG isrI2cConfig;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH LOCAL SCOPE
 ******************************************************************************/



static bool i2cEndCommunication( I2C_FAULT fault_);
void i2cCommunication(void);

/*******************************************************************************
 * FUNCTION DEFINITIONS WITH GLOBAL SCOPE
 *******************************************************************************/





void initI2C(void){



	//Enable porte clock gating bc i2c appears on PORTE
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	SIM->SCGC4 |=  SIM_SCGC4_I2C0_MASK;



	//Configure clock
	I2C0->F = 0x97;

	I2C0->C1 = 0x00; // I2C Control Register 1

	I2C0->C1 |= I2C_C1_IICIE_MASK; // Enables I2C interrupt requests.
	I2C0->C1 |= I2C_C1_IICEN_MASK; // Enables I2C module operation.




	NVIC_EnableIRQ(I2C0_IRQn);


	//Set Mux
	(PORTE->PCR)[PIN_SDA] =0;
	(PORTE->PCR)[PIN_SDA] &= ~PORT_PCR_MUX_MASK;
	(PORTE->PCR)[PIN_SDA] |= PORT_PCR_MUX(PORT_mAlt5);

	(PORTE->PCR)[PIN_SCL] =0;
	(PORTE->PCR)[PIN_SCL] &= ~PORT_PCR_MUX_MASK;
	(PORTE->PCR)[PIN_SCL] |= PORT_PCR_MUX(PORT_mAlt5);



	(PORTE->PCR)[PIN_SDA] |= PORT_PCR_ODE_MASK;//Set open drain
	(PORTE->PCR)[PIN_SCL] |= PORT_PCR_ODE_MASK;

	(PORTE->PCR)[PIN_SDA] |= (HIGH << PORT_PCR_PE_SHIFT);//Enable
	(PORTE->PCR)[PIN_SCL] |= (HIGH << PORT_PCR_PE_SHIFT);

	(PORTE->PCR)[PIN_SDA] |= (HIGH << PORT_PCR_PS_SHIFT);//Pull Up
	(PORTE->PCR)[PIN_SCL] |= (HIGH << PORT_PCR_PS_SHIFT);


}



void i2cDefaultConfig( uint8_t address) {
	isrI2cConfig.address_r = (address<<1) | 0x01;   // OR con 0b00000001
	isrI2cConfig.address_w = (address<<1) & 0xFE;   // AND con 0b11111110 bc cuando termina en 0 es para escribir
	isrI2cConfig.state = I2C_STATE_NONE;
	isrI2cConfig.flag = FLAG_OK;
	isrI2cConfig.id = 0;
	isrI2cConfig.mode = I2C_READ;
	isrI2cConfig.indexData = 0;
}



void i2cLoadCallback(ptrToFun callback_){
	isrI2cConfig.callback = callback_;
}


void i2cWriteAndRead( I2C_MODE mode , uint8_t adress_register_ , uint8_t * data_ , uint8_t size){



	if((I2C0->S & I2C_S_BUSY_MASK) != 1 ) //CHECK IF BUS BUSY
	{

		isrI2cConfig.address_register = adress_register_;
		isrI2cConfig.data = data_;
		isrI2cConfig.dataSize = size;


		if(mode == I2C_WRITE){

			isrI2cConfig.fault = I2C_FAULT_NO_FAULT;

			isrI2cConfig.mode = I2C_WRITE;	// Set write mode in control structure

			isrI2cConfig.indexData = 0;

			isrI2cConfig.flag = FLAG_TRANSMISSION;	// Set transmission in progress flag

			isrI2cConfig.state = I2C_STATE_WRITE_ADRESS_REGISTER;	// El proximo estado tiene que ser escribir la direccion del Chp que me interesa

			I2C0->C1 |= (I2C_C1_TX_MASK) ;		//Tx MODE

			I2C0->C1 |= (I2C_C1_MST_MASK);  //When MST is changed from 0 to 1, a START signal is generated on the bus and master mode is selected.

			I2C0->D = isrI2cConfig.address_w;		// Send the desired address to the bus + write bit

		}

		else if(mode == I2C_READ){

			isrI2cConfig.fault = I2C_FAULT_NO_FAULT;

			isrI2cConfig.mode = I2C_READ;	// Set write mode in control structure

			isrI2cConfig.indexData = 0;

			isrI2cConfig.flag = FLAG_TRANSMISSION;	// Set transmission in progress flag


			isrI2cConfig.state = I2C_STATE_WRITE_ADRESS_REGISTER;	// El proximo estado tiene que ser escribir la direccion del Chp que me interesa
			I2C0->C1 |= (I2C_C1_TX_MASK) ;		//Tx MODE
			I2C0->C1 |= (I2C_C1_MST_MASK);  //When MST is changed from 0 to 1, a START signal is generated on the bus and master mode is selected.


			I2C0->D = isrI2cConfig.address_w;		// Send the desired address to the bus + write bit
		}
	}
	else{
		isrI2cConfig.fault = I2C_FAULT_BUS_BUSY;
		isrI2cConfig.callback();
	}


}




void i2cCommunication(){ //Es la misma para leer y escribir, diferencia segun modo de operacion. mini FSM que controla bien el proceso de hablar entre chips

	I2C_CLEAR_IRQ_FLAG;

	uint8_t state = isrI2cConfig.state;
	uint8_t mode = isrI2cConfig.mode;

	if(mode == I2C_READ){

		switch(state){ //Depende en que estado esta la i2c

					case I2C_STATE_WRITE_ADRESS_REGISTER: //escribo la direccion que quiero leer
					{
						if(((I2C0->S & I2C_S_RXAK_MASK) == 0))	//recibi un ACK
						{
							// Write register address and switch to receive mode
							isrI2cConfig.state = I2C_STATE_RSTART;
							I2C0->D = isrI2cConfig.address_register;

						}
						else
							i2cEndCommunication(I2C_FAULT_NO_ACK);
						break;
					}
					case I2C_STATE_RSTART:
					{
						I2C0->C1 |= 1<<I2C_C1_RSTA_SHIFT; //Writing 1 to this bit generates a repeated START condition provided it is the current master.
						I2C0->D = isrI2cConfig.address_r;
						isrI2cConfig.state = I2C_STATE_DUMMY; //Ahora tengo que leer un dummy
						break;
					}
					case I2C_STATE_DUMMY:
					{

						I2C0->C1 &= ~(1<<I2C_C1_TX_SHIFT); // Pongo en estado RX

						if(isrI2cConfig.indexData == (isrI2cConfig.dataSize-1))	// estoy parado en el ultimo byte para mandar
							I2C0->C1 |= 1<<I2C_C1_TXAK_SHIFT; //NO acknowledge signal is sent to the bus on the following receiving byte (if FACK is cleared) or the
															//current receiving byte
						else
							I2C0->C1 &= ~(1<<I2C_C1_TXAK_SHIFT); //An acknowledge signal is sent to the bus on the following receiving byte (if FACK is cleared) or the
																//current receiving byte

						I2C0->D; //dummy reading

						isrI2cConfig.state = I2C_STATE_READ_DATA; //Cambio el estado a Leer DATA
						break;
					}
					case I2C_STATE_READ_DATA:
					{
						if(isrI2cConfig.indexData == (isrI2cConfig.dataSize-1))	// es el ultimo byte a leer

							i2cEndCommunication(I2C_FAULT_NO_FAULT);

						else if(isrI2cConfig.indexData == isrI2cConfig.dataSize-2)	//Aca es momento de mandar el NACK
							I2C0->C1 |= 1<<I2C_C1_TXAK_SHIFT;					//Mando un No acknowledge

						isrI2cConfig.data[isrI2cConfig.indexData++] = I2C0->D;
						break;
					}
					default:
					break;
				}
	}
	else if(mode == I2C_WRITE){

		switch(state){

			case I2C_STATE_WRITE_ADRESS_REGISTER: //Escribir la direccion del registro

				if((I2C0->S & I2C_S_RXAK_MASK) == 0)//Recivi un ACK (si es 0)
				{
					I2C0->D = isrI2cConfig.address_register;	//escribo la direccion del registro
					isrI2cConfig.state = I2C_STATE_WRITE_DATA; //Ahora el estado que acontece es escribir la data
				}
				else
					i2cEndCommunication(I2C_FAULT_NO_ACK);
				break;

			case I2C_STATE_WRITE_DATA:

				if(isrI2cConfig.indexData == isrI2cConfig.dataSize)	// si estoy al final de la data que quiero mandar, termina la transmicion
					i2cEndCommunication(I2C_FAULT_NO_FAULT);
				else
				{
					if((I2C0->S & I2C_S_RXAK_MASK) == 0)	//recibi un ACK
					{
						I2C0->D  = isrI2cConfig.data[isrI2cConfig.indexData++]; //escribo el arreglo de bytes. mi iterador es el data index
					}
					else //hubo error
						i2cEndCommunication(I2C_FAULT_NO_ACK); //Incomplete
				}
				break;

			default:
				break;

		}


	}

}


static bool i2cEndCommunication( I2C_FAULT fault_){

	I2C0->C1 &= ~(1<<I2C_C1_MST_SHIFT); //STOP SIGNAL

	isrI2cConfig.fault = fault_; //Que falta fue

	isrI2cConfig.state = I2C_STATE_NONE;

	isrI2cConfig.flag = FLAG_OK;

	isrI2cConfig.callback(); //esta funcion es para que el usuario del sensor pueda ejecutar un callback para
	//						levantar un flag y segui con la configuracion
	return true;

}


__ISR__ I2C0_IRQHandler (void){
	if(I2C0->S & I2C_S_TCF_MASK ){
		i2cCommunication();
	}
}



I2C_FAULT getFault(void){
	return isrI2cConfig.fault;
}

