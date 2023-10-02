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
 static uint8_t buffer[12];



static bool i2cEndCommunication(I2C_CONFIG * i2cConfig , I2C_FAULT fault_);
void i2cCommunication(I2C_CONFIG * i2cConfig);

/*******************************************************************************
 * FUNCTION DEFINITIONS WITH GLOBAL SCOPE
 *******************************************************************************/





void initI2C(I2C_CONFIG * i2cConfig){



	//Enable porte clock gating bc i2c appears on PORTE
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    //Clock Gating
	if(((i2cConfig->id) >= 0) && ((i2cConfig->id)<I2C_CANT_IDS)){
		if((i2cConfig->id)==2){
				SIM_PTR->SCGC1 |= SIM_MASK[i2cConfig->id];
		}
		else{
				SIM_PTR->SCGC4 |= SIM_MASK[i2cConfig->id];
		}
	}

	I2C0->F = i2cConfig->freq;// frecuencia


	NVIC_EnableIRQ(IRQn[i2cConfig->id]); // Enable NVIC interrupts


	// Write I2C0 Control Register 1
	I2C0->C1 = 0x00;
	I2C0->C1 |= (I2C_C1_IICEN_MASK | I2C_C1_IICEN_MASK);	// enable i2c & i2c interrupts

	PORTE->PCR[PIN_SCL] = 0;
 	PORTE->PCR[PIN_SCL] &= ~PORT_PCR_MUX_MASK;
 	PORTE->PCR[PIN_SCL] |= PORT_PCR_MUX(PORT_mAlt5);
 	PORTE->PCR[PIN_SCL] |= (1<<PORT_PCR_ODE_SHIFT);

 	PORTE->PCR[PIN_SDA] &= ~PORT_PCR_MUX_MASK;
 	PORTE->PCR[PIN_SDA] |= PORT_PCR_MUX(PORT_mAlt5);
 	PORTE->PCR[PIN_SDA] |= (1<<PORT_PCR_ODE_SHIFT);

 	I2C0->C1 |= 1<<I2C_C1_TX_SHIFT; //setear como modo TX
}



void i2cDefaultConfig(I2C_CONFIG * i2cConfig, uint8_t address, uint16_t frequency) {
	i2cConfig->address_r = (address<<1) | 0x01;   // OR con 0b00000001
	i2cConfig->address_w = (address<<1) & 0xFE;   // AND con 0b11111110 bc cuando termina en 0 es para escribir
	i2cConfig->state = I2C_STATE_NONE;
	i2cConfig->flag = FLAG_OK;
	i2cConfig->id = 0;
	i2cConfig->mode = I2C_READ;
	i2cConfig->indexData = 0;
	i2cConfig->freq = frequency;
	i2cConfig->data = buffer;
}






void i2cWriteAndRead(I2C_CONFIG  i2cConfig, I2C_MODE mode){


	if(mode == I2C_WRITE)
		while(i2cStartCommunication(&i2cConfig ,I2C_WRITE) == false)	{ //devuelve false cuando esta BUSY. entonces espera a que se libere para comenzar la comunicacion

		}
	else if(mode == I2C_READ)
		while(i2cStartCommunication(&i2cConfig ,I2C_READ) == false) 	{

		}

	// si pude avanzar sin tener errores de i2c
	while(i2cConfig.flag == FLAG_TRANSMISSION) //Mientras este en transmision
	{
		//while((I2C0->S & I2C_S_IICIF_MASK)==0); //espero a tener una interrupcion
		//i2cCommunication(&i2cConfig); //mini fsm que lleva adelante la comunicacion
	}
	while(((I2C0->S & I2C_S_BUSY_MASK) != 0));		// espero hasta que este libre .si esta en 0 esta en idle . en 1 esta busy


}



bool i2cStartCommunication(I2C_CONFIG * i2cConfig , I2C_MODE mode){



	if((I2C0->S & I2C_S_BUSY_MASK) != 0){ //Si el Bus esta ocupado que vuelva
		return false;
	}
	else //Seteo el primer estado de la comunicacion
	{
		i2cConfig->mode = mode;	// Set write mode in control structure
		i2cConfig->indexData = 0;
		i2cConfig->flag = FLAG_TRANSMISSION;	// Set transmission in progress flag
		i2cConfig->state = I2C_STATE_WRITE_ADRESS_REGISTER;	// El proximo estado tiene que ser escribir la direccion del Chp que me interesa
		I2C0->C1 |= (1<<I2C_C1_TX_SHIFT);		//Tx MODE
		I2C0->C1 |= 1<<I2C_C1_MST_SHIFT;		//When MST is changed from 0 to 1, a START signal is generated on the bus and master mode is selected.
		I2C0->D = i2cConfig->address_w;		// Send the desired address to the bus + write bit

		isrI2cConfig = *i2cConfig; //Esto es solamente para cuando tengo mas de un byte para leer. seria no bloqueante y por medio de interrupciones

		return true;
	}

}


void i2cCommunication(I2C_CONFIG * i2cConfig){ //Es la misma para leer y escribir, diferencia segun modo de operacion. mini FSM que controla bien el proceso de hablar entre chips

	//Primero borro la interrupcion
	I2C0->S |= 1<<I2C_S_IICIF_SHIFT;

	uint8_t state = i2cConfig->state;
	uint8_t mode = i2cConfig->mode;
	if(mode == I2C_READ){

		switch(state){ //Depende en que estado esta la i2c

					case I2C_STATE_WRITE_ADRESS_REGISTER: //escribo la direccion que quiero leer
					{
						if(((I2C0->S & I2C_S_RXAK_MASK) == 0))	//recibi un ACK
						{
							// Write register address and switch to receive mode
							I2C0->D = i2cConfig->address_register;
							i2cConfig->state = I2C_STATE_RSTART;
						}
						else
							i2cEndCommunication(i2cConfig,I2C_FAULT_NO_ACK); //Devuelve un bool?
						break;
					}
					case I2C_STATE_RSTART:
					{
						I2C0->C1 |= 1<<I2C_C1_RSTA_SHIFT; //Writing 1 to this bit generates a repeated START condition provided it is the current master.
						I2C0->D = i2cConfig->address_r;
						i2cConfig->state = I2C_STATE_DUMMY; //Ahora tengo que leer un dummy
						break;
					}
					case I2C_STATE_DUMMY:
					{

						I2C0->C1 &= ~(1<<I2C_C1_TX_SHIFT); // Pongo en estado RX

						if(i2cConfig->indexData == (i2cConfig->dataSize-1))	// estoy parado en el ultimo byte para mandar
							I2C0->C1 |= 1<<I2C_C1_TXAK_SHIFT; //NO acknowledge signal is sent to the bus on the following receiving byte (if FACK is cleared) or the
															//current receiving byte
						else
							I2C0->C1 &= ~(1<<I2C_C1_TXAK_SHIFT); //An acknowledge signal is sent to the bus on the following receiving byte (if FACK is cleared) or the
																//current receiving byte

						uint8_t pp = I2C0->D; //dummy reading

						i2cConfig->state = I2C_STATE_READ_DATA; //Cambio el estado a Leer DATA
						break;
					}
					case I2C_STATE_READ_DATA:
					{
						if(i2cConfig->indexData == (i2cConfig->dataSize-1))	// es el ultimo byte a leer

							i2cEndCommunication(i2cConfig,I2C_FAULT_END_READING);

						else if(i2cConfig->indexData == i2cConfig->dataSize-2)	//Aca es momento de mandar el NACK
							I2C0->C1 |= 1<<I2C_C1_TXAK_SHIFT;					//Mando un No acknowledge

						i2cConfig->data[i2cConfig->indexData++] = I2C0->D;
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
					I2C0->D = i2cConfig->address_register;	//escribo la direccion del registro
					i2cConfig->state = I2C_STATE_WRITE_DATA; //Ahora el estado que acontece es escribir la data
				}
				else
					i2cEndCommunication(i2cConfig,I2C_FAULT_NO_ACK);
				break;

			case I2C_STATE_WRITE_DATA:

				if(i2cConfig->indexData == i2cConfig->dataSize)	// si estoy al final de la data que quiero mandar, termina la transmicion
					i2cEndCommunication(i2cConfig,I2C_FAULT_END_WRITING);
				else
				{
					if((I2C0->S & I2C_S_RXAK_MASK) == 0)	//recibi un ACK
					{
						I2C0->D  = i2cConfig->data[i2cConfig->indexData++]; //escribo el arreglo de bytes. mi iterador es el data index
					}
					else //hubo error
						i2cEndCommunication(i2cConfig,I2C_FAULT_NO_ACK); //Incomplete
				}
				break;

			default:
				break;

		}


	}

}


static bool i2cEndCommunication(I2C_CONFIG * i2cConfig , I2C_FAULT fault_){

	I2C0->C1 &= ~(1<<I2C_C1_MST_SHIFT);
	i2cConfig->state = I2C_STATE_NONE;
	i2cConfig->flag = FLAG_OK;
	i2cConfig->fault = fault_;
	//Aca se tiene que llamar a una fuencion que parsee los datos en el exterior
	i2cConfig->callback();
	return true;

}



void I2C0_IRQHandler(void) //Este handler va a ser usador cuando el slave tenga informacion en data ready
{

	i2cCommunication(&isrI2cConfig);
}




