/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "MCAL/board.h"
#include "MCAL/gpio.h"
#include "CAN.h"
 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
 
 

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef enum{ // Los codigos de cada MB. leer pagina 1432 para mas info
	INACTIVE_RX = 0b0000,
	 EMPTY_RX    = 0b0100,
	 FULL_RX     = 0b0010,
	 OVERRUN_RX  = 0b0011,
	 ANSWER_RX  = 0b1010,
	 BUSY_RX     = 0b0001
}RX_CODE;
/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 void configureCANClock(canConfig_t * config);
 void defaultCANConfig(canConfig_t * config);
 void configureIndivRxMask(void);
/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
 
 CAN_STATUS initCAN(canConfig_t * config){

		defaultCANConfig(config);

		SIM->SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK; /// CLOCK enable for CAN0

		enableCAN(); //Enable of CAN0

		CAN0->MCR |= CAN_MCR_SOFTRST(1); /// mask 4 soft reset
		while(((CAN0->MCR)&CAN_MCR_SOFTRST_MASK)!= CAN_MCR_SOFTRST_MASK); // wait until achieves soft reset

		PORT_Type * PORTS[] = PORT_BASE_PTRS; //CONFIGURA MUX OF PORTB18 & PORTB19 as CANBUS
		PORTS[PIN2PORT(CAN0_RX_PIN)]->PCR[PIN2NUM(CAN0_RX_PIN)] = PORT_PCR_MUX(PORT_mAlt2); // configuration of ports to CAN
		PORTS[PIN2PORT(CAN0_TX_PIN)]->PCR[PIN2NUM(CAN0_TX_PIN)] = PORT_PCR_MUX(PORT_mAlt2);

		disableCAN(); //Disable CAN for write into CTR1 and MCR  the clock config
		configureCANClock(config); // PLL clock or Crystal Clock

		enableCAN(); // Enable CAN
		while((CAN0->MCR & CAN_MCR_FRZACK_MASK)!=CAN_MCR_FRZACK_MASK); //wait til CAN freezes . it enters freezing bc SoftStart resets FRZ and HLT to 1

		configureIndivRxMask();

		for(int i=0; i<CAN_ID_COUNT; i++){ // Buffers reset
			CAN0->MB[i].CS = ( CAN0->MB[i].CS &= ~CAN_CS_CODE_MASK ); // Limpio la informacion preexistente en esa posicion
			CAN0->MB[i].CS |= CAN_CS_CODE(INACTIVE_RX); // Code set
			CAN0->MB[i].ID = CAN_ID_STD(0); // id = 0
			CAN0->RXIMR[i] = 0xFFFFFFFF; // Masks RESET
		}
		return CAN_READY;
}



 void enableCAN()
 {
 	CAN0->MCR &= ~CAN_MCR_MDIS_MASK; // to clear this register is to enable CAN
 	while(CAN0->MCR & CAN_MCR_LPMACK_MASK); // As the reference manual suggests, wait until CAN goes out of low power mode. MCR in LPMACK has to be  0
 }


 void disableCAN()
 {
 	CAN0->MCR |= CAN_MCR_MDIS_MASK; // to Set this register is to disable CAN
 	// Wait until CAN module is out of low power mode (MCR[LPM_ACK] = 0).
 	while((CAN0->MCR & CAN_MCR_LPMACK_MASK)!=CAN_MCR_LPMACK_MASK);  // As the reference manual suggests, wait until CAN goes low power mode. MCR in LPMACK has to be  1
 }







 void configureCANClock(canConfig_t * config){
	 if( config->clock == CAN_OSC_CLOCK){
		 CAN0->CTRL1 = CAN_CTRL1_CLKSRC(0);
	 }
	 else if( config->clock == CAN_PERIPHERAL_CLOCK){
		 CAN0->CTRL1 = CAN_CTRL1_CLKSRC(1);
	 }

 }

void defaultCANConfig(canConfig_t * config){
	config->clock = CAN_OSC_CLOCK;
	config->baudrate = 125000; //  default number 4 can
	config->timing.PRESDIV = 0x13;
	config->timing.PROPSEG = 0x07;
	config->timing.PSEG1 = 0x07;
	config->timing.PSEG2 = 0x02;
}


void configureIndivRxMask(void){
	// TODO se podria agregar otro control de datos recibidos.
	CAN0->MCR |= CAN_MCR_IRMQ_MASK; // USO MB con mascaras que me da el micro
}
