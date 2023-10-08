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


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
void configureCANClock(canConfig_t * config);
 void defaultCANConfig(canConfig_t * config);
 void configureIndivRxMask(void);

 static canFrame_t frameRx;
 static ptrToFunction callbacks[CAN_ID_COUNT];

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
		PORTS[PIN2PORT(CAN0_RX_PIN)]->PCR[PIN2NUM(CAN0_RX_PIN)] = PORT_PCR_MUX(PORT_mAlt2) |PORT_PCR_PE_MASK |PORT_PCR_PS_MASK; // configuration of ports to CAN
		PORTS[PIN2PORT(CAN0_TX_PIN)]->PCR[PIN2NUM(CAN0_TX_PIN)] = PORT_PCR_MUX(PORT_mAlt2) |PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

		disableCAN(); //Disable CAN for write into CTR1 and MCR  the clock config


		enableCAN(); // Enable CAN

		while((CAN0->MCR & CAN_MCR_FRZACK_MASK)!=CAN_MCR_FRZACK_MASK); //wait til CAN freezes . it enters freezing bc SoftStart resets FRZ and HLT to 1

		CAN0->CTRL1 = 0;
		CAN0->CTRL1 |= CAN_CTRL1_PROPSEG(1) | CAN_CTRL1_PRESDIV(48)
		 					| CAN_CTRL1_PSEG1(2) | CAN_CTRL1_PSEG2(1) | CAN_CTRL1_RJW(0)
		 					| CAN_CTRL1_LBUF(0);
		//CAN0->CTRL2 = 0xb40000;
		//CAN0->TIMER = 0x7;

		//configureIndivRxMask();

		for(int i=0; i<CAN_ID_COUNT; i++){ // Buffers reset
			//CAN0->MB[i].CS = ( CAN0->MB[i].CS &= ~CAN_CS_CODE_MASK ); // Limpio la informacion preexistente en esa posicion
			//CAN0->MB[i].CS |= CAN_CS_CODE(INACTIVE_RX); // Code set
			//CAN0->MB[i].ID = CAN_ID_STD(0); // id = 0
			//CAN0->RXIMR[i] = 0xFFFFFFFF; // Masks RESET
		}


		//Exit Freeze

		CAN0->MCR &= ~CAN_MCR_HALT_MASK;
		//esperar a que se freezee..
		while((CAN0->MCR & CAN_MCR_FRZACK_MASK)==CAN_MCR_FRZACK_MASK);



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


 void enableCanInterrup( uint8_t mb_id, ptrToFunction callback){
 	NVIC_EnableIRQ(CAN0_ORed_Message_buffer_IRQn);
 	CAN0->IMASK1 |= (1UL<<mb_id);
 	callbacks[mb_id] = callback;
 }




 void configureCANClock(canConfig_t * config){




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

void  configRxMB( uint8_t mb_id, uint32_t ID){
	/// inactivate Mailbox
	//CAN0->MB[mb_id].CS = ( CAN0->MB[mb_id].CS &= ~CAN_CS_CODE_MASK ) | CAN_CS_CODE(INACTIVE_RX);

	///Pongo el ID
	CAN0->MB[mb_id].ID = CAN_ID_STD(ID);

	/// Write the EMPTY code to the CODE field of the Control and Status word to activate the Mailbox.
	CAN0->MB[mb_id].CS = CAN_CS_CODE(EMPTY_RX);
}

uint8_t transmitCan(uint8_t MB_ID,canFrame_t *frame){


	/// check if IFLAG its asserted and write 1 to clear it
	if(CAN0->IFLAG1& (1<<MB_ID))
		CAN0->IFLAG1 |= (1<<MB_ID);


	//write ID
	CAN0->MB[MB_ID].ID = CAN_ID_STD(frame->ID);

	/// Write INACTIVE
	CAN0->MB[MB_ID].CS = CAN_CS_CODE(INACTIVE_TX);

	// Write data bytes
	CAN0->MB[MB_ID].WORD0 = CAN_WORD0_DATA_BYTE_0(frame->dataByte0) |
	                    	CAN_WORD0_DATA_BYTE_1(frame->dataByte1) |
	                    	CAN_WORD0_DATA_BYTE_2(frame->dataByte2) |
	                    	CAN_WORD0_DATA_BYTE_3(frame->dataByte3);
	CAN0->MB[MB_ID].WORD1 = CAN_WORD1_DATA_BYTE_4(frame->dataByte4) |
	                        CAN_WORD1_DATA_BYTE_5(frame->dataByte5) |
	                    	CAN_WORD1_DATA_BYTE_6(frame->dataByte6) |
	                    	CAN_WORD1_DATA_BYTE_7(frame->dataByte7);

	//write code and length

	CAN0->MB[MB_ID].CS = CAN_CS_CODE(ACTIVE_TX) |  CAN_CS_DLC(frame->length);

	return TRANSMIT_OK;
}


STATUS_READ readCAN(uint8_t MB_ID, canFrame_t *frame){

	uint8_t return_value;

	/// Check if the BUSY bit is 0
	if(CAN0->MB[MB_ID].CS>>CAN_CS_CODE_SHIFT & BUSY_RX) //RXbusy es 1
		return_value = READ_FAIL;

	//SRV: Serviced MB. MB was read and unlocked by reading TIMER or other MB.
	CAN0->TIMER;

	//Check if mailbox its active

	uint32_t code = (CAN0->MB[MB_ID].CS & CAN_CS_CODE_MASK)>>CAN_CS_CODE_SHIFT;
	switch(code){

		//Si esta full
		case FULL_RX:
			frame->ID = (CAN0->MB[MB_ID].ID & CAN_ID_STD_MASK)>>CAN_ID_STD_SHIFT;

			frame->dataWord0 =  ((CAN0->MB[MB_ID].WORD0 & CAN_WORD0_DATA_BYTE_0_MASK)>>24)|
								((CAN0->MB[MB_ID].WORD0 & CAN_WORD0_DATA_BYTE_1_MASK)>>8)|
								((CAN0->MB[MB_ID].WORD0 & CAN_WORD0_DATA_BYTE_2_MASK)<<8)|
								((CAN0->MB[MB_ID].WORD0 & CAN_WORD0_DATA_BYTE_3_MASK)<<24);

			frame->dataWord1 =  ((CAN0->MB[MB_ID].WORD1 & CAN_WORD1_DATA_BYTE_4_MASK)>>24)|
								((CAN0->MB[MB_ID].WORD1 & CAN_WORD1_DATA_BYTE_5_MASK)>>8)|
								((CAN0->MB[MB_ID].WORD1 & CAN_WORD1_DATA_BYTE_6_MASK)<<8)|
								((CAN0->MB[MB_ID].WORD1 & CAN_WORD1_DATA_BYTE_7_MASK)<<24);

			frame->length = (CAN0->MB[MB_ID].CS & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT;

			// Write the EMPTY code (0b0100) to the CODE
			CAN0->MB[MB_ID].CS = CAN_CS_CODE(EMPTY_RX);

			return_value = READ_OK;
			break;

		case EMPTY_RX:
			return_value = READ_FAIL;
			break;
	}

	///IFLAG registers reset by writing 1
	CAN0->IFLAG1 |= (1<<MB_ID);


	return return_value;

}






void CAN0_ORed_Message_buffer_IRQHandler(void){




	for(int i=0; i<CAN_ID_COUNT; i++)
	{
		if( CAN0->IFLAG1 & (1<<i) ) //Cuando exista una flag levantada , el numerode bit es cual id esta levantado
		{
			if(((CAN0->MB[i].CS&CAN_CS_CODE_MASK)>>CAN_CS_CODE_SHIFT)==FULL_RX) //Cuando un MB este lleno , descargalo
			{
				frameRx.dataWord0 = 0x00000000;
				frameRx.dataWord1 = 0x00000000;

				CAN_STATUS s = readCAN(i,&frameRx);
				callbacks[i](&frameRx);

			}
		}
	}
}

