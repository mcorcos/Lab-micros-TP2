/*
 * main.c
 *
 *  Created on: May 1, 2015
 *      Author: Juan Pablo VEGA - Laboratorio de Microprocesadores
 */

#include "hardware.h"
#include "source/drivers/drv_UART.h"


#define __FOREVER__ 	for(;;)

#define PIN_RED_LED 		22     	//PTB22
#define PIN_BLUE_LED 		21     	//PTB21
#define PIN_GREEN_LED 		26 	   	//PTE26
#define PIN_PUSH_BUTTON  	4 		//PTA4

typedef enum
{
	PORT_mAnalog,
	PORT_mGPIO,
	PORT_mAlt2,
	PORT_mAlt3,
	PORT_mAlt4,
	PORT_mAlt5,
	PORT_mAlt6,
	PORT_mAlt7,

} PORTMux_t;

typedef enum
{
	PORT_eDisabled				= 0x00,
	PORT_eDMARising				= 0x01,
	PORT_eDMAFalling			= 0x02,
	PORT_eDMAEither				= 0x03,
	PORT_eInterruptDisasserted	= 0x08,
	PORT_eInterruptRising		= 0x09,
	PORT_eInterruptFalling		= 0x0A,
	PORT_eInterruptEither		= 0x0B,
	PORT_eInterruptAsserted		= 0x0C,
} PORTEvent_t;

#define BALIZA_DELAY	4000000UL

void idle(void);
void delayLoop (uint32_t veces);

int main (void)
{
 uint32_t temp;
 uint32_t pin;
 uint32_t data;

package_t uart_data;

 	 	 	 hw_Init ();
			//Enable clocking for port B

			SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
			SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

			PORTA->PCR[4] |= PORT_PCR_ISF_MASK;  //Clear mask
			NVIC_EnableIRQ(PORTA_IRQn);


			//Configure port control register pin 22 PORTB

		/*	PORTB->PCR[22]=0x0; //Clear all bits
			PORTB->PCR[22]|=PORT_PCR_MUX(PORT_mGPIO); 		//Set MUX to GPIO
			PORTB->PCR[22]|=PORT_PCR_DSE(1);          		//Drive strength enable
			PORTB->PCR[22]|=PORT_PCR_IRQC(PORT_eDisabled);  //Disable interrupts
		*/



/////////// Multiple Pins at once on PORT B

			temp =PORT_GPCHR_GPWE((1<<(21-16))|(1<<(22-16))); // Which Pins
			// Now set pin properties
			temp|=PORT_GPCHR_GPWD(PORT_PCR_MUX(PORT_mGPIO));  //Set MUX to GPIO
			temp|=PORT_GPCHR_GPWD(PORT_PCR_DSE(1));			  //Drive strength enable
			PORTB->GPCHR=temp;


/////////// Input Pin 4
			PORTA->PCR[4]=0x0; //Clear
			PORTA->PCR[4]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO
			PORTA->PCR[4]|=PORT_PCR_PE(1);          		       //Pull UP/Down  Enable
			PORTA->PCR[4]|=PORT_PCR_PS(1);          		       //Pull UP
			//PORTA->PCR[4]|=PORT_PCR_IRQC(PORT_eInterruptFalling);  //Enable Falling edge interrupts
			PORTA->PCR[4]|=PORT_PCR_IRQC(PORT_eInterruptRising);  //Enable Rising edge interrupts

			//////////

			PTB->PSOR = (1<<21)|(1<<22);

			// Configure GPIO registers

			// Configurar como salida pin 22 y pin 21 PTB
			PTB->PDDR |= (1<<21)|(1<<22);
			// Configurar como entrada pin 4 PTA
			PTA->PDDR |= (0<<4);

			SysTick_Init();

			uart_cfg_t config_uart = {BAUDRATE_DEFAULT,0};
			//UART_Init();
			initUART(0, config_uart);
			char textMessage [] = "Hola mundo!";

			// Enable interrupts
			hw_EnableInterrupts();
			for(uint8_t i =0; i<sizeof(textMessage);i++){
				package_t pack = {.data[0] = textMessage[i]};
				pack.data[0] = textMessage[i];
				sendPackage(pack);
			}

			__FOREVER__
			{

				uart_data=receivePackage();
				sendPackage(uart_data);

			}




}


__ISR__  PORTA_IRQHandler(void)
{

	// Clear port IRQ flag


		PORTA->PCR[4] |= PORT_PCR_ISF_MASK;

		PTB->PCOR = (1<<21)|(1<<22);



}


void idle(void)
{

}


void delayLoop (uint32_t veces)
{
	while (veces--);
}


__ISR__  SysTick_Handler (void)
{
	static uint32_t speed=4;  // 0.5 seg @ tick =125ms

	if (speed==0)
	{

		PTB->PTOR = (1<<21)|(1<<22);
		speed=4;

	}

	speed--;


}

void SysTick_Init (void)
{
	SysTick->CTRL = 0x00;
	SysTick->LOAD = 12500000L-1; //12499999L; // <= 125 ms @ 100Mhz
	SysTick->VAL  = 0x00;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

