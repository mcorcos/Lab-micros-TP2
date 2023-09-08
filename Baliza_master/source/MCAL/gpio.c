/***************************************************************************//**
  @file     gpio.h
  @brief    Simple GPIO Pin services, similar to Arduino
  @author   Manuel Corcos
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "gpio.h"


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


// para iniciar el clock de cada puerto

static uint32_t SIM_MASK[] = {SIM_SCGC5_PORTA_MASK, SIM_SCGC5_PORTB_MASK, SIM_SCGC5_PORTC_MASK, SIM_SCGC5_PORTD_MASK, SIM_SCGC5_PORTE_MASK };
static SIM_Type* SIM_PTR = SIM;


//Para facilitar el acceso a pines
static GPIO_Type* GPIO_PTRS[] = GPIO_BASE_PTRS;
static PORT_Type* PORT_PTRS[] = PORT_BASE_PTRS;

//Creo arreglo para asignar interrupciones corresponientes de cada puerto
static uint8_t PORTS_IRQ[] = PORT_IRQS ;


pinIrqFun_t PTRS_TO_FUN[PORT_PCR_COUNT*PORT_COUNT] ;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*
 * @brief Configures the specified pin to behave either as an input or an output
 * @param pin the pin whose mode you wish to set (according PORTNUM2PIN)
 * @param mode INPUT, OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN.
 */
void gpioMode (pin_t pin, uint8_t mode){
	//init clock
	SIM_PTR->SCGC5 |= SIM_MASK[PIN2PORT(pin)];
	// obtain gpio and port
	GPIO_Type* gpio = GPIO_PTRS[PIN2PORT(pin)] ;
	PORT_Type* port = PORT_PTRS[PIN2PORT(pin)] ;
	//set pin as gpio
	port->PCR[PIN2NUM(pin)] = 0x00;
	port->PCR[PIN2NUM(pin)] |= PORT_PCR_MUX(PORT_mGPIO);

	switch(mode){
		case INPUT:   /* QUIERO PONER UN 0 POR QUE ES UN INPUT*/
			gpio->PDDR &=   ~(1<<PIN2NUM(pin)); /*se puede resumir a eso*/
			break;
		case OUTPUT: /* QUIERO PONER UN 1 POR QUE ES UN INPUT*/
			gpio->PDDR |=  ( 1 << PIN2NUM(pin)); /* se resume muchisimo, solo tenes que prender un bit, osea or con 1 con ese bit en esa posicion , osea sin mascara.*/
		case INPUT_PULLUP:

			break;
		case INPUT_PULLDOWN:
			break;
	}



}



/**
 * @brief Configures how the pin reacts when an IRQ event ocurrs
 * @param pin the pin whose IRQ mode you wish to set (according PORTNUM2PIN)
 * @param irqMode disable, risingEdge, fallingEdge or bothEdges
 * @param irqFun function to call on pin event
 * @return Registration succeed
 */
bool gpioIRQ (pin_t pin, uint8_t irqMode, pinIrqFun_t irqFun){
	PORT_Type* port = PORT_PTRS[PIN2PORT(pin)];
	if(irqMode < CANT_PORTS_IRQ_MODES){// chequeo que el modo sea correcto, que no sea mayor a 12
		NVIC_EnableIRQ(PORTS_IRQ[PIN2PORT(pin)]); //Enable PORTA interrupt on NVIC
		//NVIC_SetPriority(PORTS_IRQ[PIN2PORT(pin)],0);
		port->PCR[PIN2NUM(pin)]|=PORT_PCR_IRQC(irqMode);
		PTRS_TO_FUN[(PIN2PORT(pin)*PORT_PCR_COUNT) + PIN2NUM(pin)] = irqFun;
		return 1;

	}
	return 0;
}




/*
 * @brief Write a HIGH or a LOW value to a digital pin
 * @param pin the pin to write (according PORTNUM2PIN)
 * @param val Desired value (HIGH or LOW)
 */
void gpioWrite(pin_t pin, bool value){
	GPIO_Type* gpio = GPIO_PTRS[PIN2PORT(pin)] ;
	if(!value){
		gpio->PDOR &= ~ ( 1 << PIN2NUM(pin));   /* ACA HAGO AND CON UNA MASCARA DE 111101111 , DONDE EL 0 ES EL PIN A MODIFICAR. DESPUES HAGO UNA OR CON EL 00000X0000, ENTONCES VOY A MODIFICAR ESE MISMO PIN CON LA X QUE CORRESPONDA */
	}
	else{
		gpio->PDOR |= ( 1 << PIN2NUM(pin)) ;
	}

}




/*
 * @brief Toggle the value of a digital pin (HIGH<->LOW)
 * @param pin the pin to toggle (according PORTNUM2PIN)
 */
void gpioToggle (pin_t pin){
	GPIO_Type* gpio = GPIO_PTRS[PIN2PORT(pin)];
	if(  (gpio->PDOR & (1<<PIN2NUM(pin))) == (1<<PIN2NUM(pin)) ) {
		gpio->PDOR &= ~(1<<PIN2NUM(pin)); //Pongo un cero en toggle si esta prendido
	}

	else{
		gpio->PDOR |= (1<<PIN2NUM(pin));
	}
}
/*
 * @brief Reads the value from a specified digital pin, either HIGH or LOW.
 * @param pin the pin to read (according PORTNUM2PIN)
 * @return HIGH or LOW
 */
/*bool gpioRead (pin_t pin) {

	GPIO_Type* gpio = GPIO_PTRS[PIN2PORT(pin)];
	return (( gpio->PDIR &  (1<<PIN2NUM(pin)) ) == (1<<PIN2NUM(pin)));

}*/

bool gpioRead (pin_t pin) {

	GPIO_Type* gpio = GPIO_PTRS[PIN2PORT(pin)];
	return (( gpio->PDIR &  (1<<PIN2NUM(pin)) ) == 1<<PIN2NUM(pin));

}


