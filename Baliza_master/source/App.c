/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Grupo 4 Laboratorio de Microprocesadores:
  	  	  	Corcos, Manuel
  	  	  	Lesiuk, Alejandro
  	  	  	Paget, Milagros
  	  	  	Voss, Maria de Guadalupe
  ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "MCAL/board.h"
#include "timers/timer.h" // Llamo para Init timer
#include "drivers/drv_DEVBOARD.h" // Placa creada por el grupo con leds para debuggin
#include "drivers/drv_K64.h" // creamos funciones para acceder a switches y demas de la kinetis
#include "drivers/drv_UART.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
//ID de UART0
#define DEFAULT_ID 0

//TIMERS MS DEFINE
#define TIMER_TX_MS 10
#define TIMER_RX_MS 100


//CANTIDADES
#define CANT_DISP 7
#define CANT_PACKAGE 4
#define SIZE_OF_PACKAGE 2
/*******************************************************************************
 *******************************************************************************
                        STRUCTS & TYPEDFS
 *******************************************************************************
 ******************************************************************************/

typedef struct{
	uint8_t id;
	int16_t rolling;
	int16_t tilt;
	int16_t orientation;
}dispositive_t;


/*******************************************************************************
 *******************************************************************************
                        GLOBAL VARS DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static tim_id_t timerTx;
static tim_id_t timerRx;

static dispositive_t dispositives[CANT_DISP];
static dispositive_t buffer[CANT_DISP];

static package_t message4CPU[CANT_DISP][CANT_PACKAGE];
static package_t * messagePointer = &(message4CPU[0][0]);


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION PROTOTYPES
 *******************************************************************************
 ******************************************************************************/

void callbackTimerTx(void);
void callbackTimerRx(void);

void initDispositives(void);
void updateDispositives(void);
void updateMessage4CPU(uint8_t i);
void initBuffer(void);

/*******************************************************************************
 *******************************************************************************
						GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{

	init_K64Leds(); //init drv de los leds de la kinetis
	timerInit(); // init de timer
	init_DEVBOARD(); // init de la placa shield creada por el grupo
	initUART(); //Init UART

	//inicio los timers de UART
	timerTx = timerGetId();
	timerRx = timerGetId();


    timerStart(timerTx, TIMER_MS2TICKS(TIMER_TX_MS), TIM_MODE_PERIODIC, callbackTimerTx);
    timerStart(timerRx, TIMER_MS2TICKS(TIMER_RX_MS), TIM_MODE_PERIODIC, callbackTimerRx);

    initDispositives();
    initBuffer();
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	// updateo las posiciones de mi placa y de las demas
	updateDispositives();



}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTIONS DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


void updateDispositives(void){
	for(uint8_t i=0; i < CANT_DISP ; i++){
		dispositives[i].rolling = buffer[i].rolling;
		dispositives[i].tilt = buffer[i].tilt;
		dispositives[i].orientation = buffer[i].orientation;
		updateMessage4CPU(i);
	}
}

void updateMessage4CPU(uint8_t i){
	message4CPU[i][0].dataType[0] = 'I';//poner el ID
	message4CPU[i][0].value = i+'0'; //poner el value
	if(buffer[i].rolling >= 0){
		message4CPU[i][1].dataType[0] = '+';//poner el + o -
		message4CPU[i][1].value = buffer[i].rolling;
	}
	else{
		message4CPU[i][1].dataType[0] = '-';//poner el + o -
		message4CPU[i][1].value = (-1) * buffer[i].rolling;
	}
	if(buffer[i].tilt >= 0){
		message4CPU[i][2].dataType[0] = '+';//poner el + o -
		message4CPU[i][2].value = buffer[i].tilt;
	}
	else{
		message4CPU[i][2].dataType[0] = '-';//poner el + o -
		message4CPU[i][2].value = (-1) * buffer[i].tilt;
	}
	if(buffer[i].orientation >= 0){
		message4CPU[i][3].dataType[0] = '+';//poner el + o -
		message4CPU[i][3].value = buffer[i].orientation;
	}
	else{
		message4CPU[i][3].dataType[0] = '-';//poner el + o -
		message4CPU[i][3].value = (-1) * buffer[i].orientation;
	}
}

void initDispositives(void){ // inicio los ids  de los dispositives
	for(uint8_t i=0;i<CANT_DISP;i++){
		dispositives[i].id = i + '0';
	}
}

void initBuffer(void){
	for(uint8_t i=0;i<CANT_DISP;i++){
		buffer[i].id = i + '0';
		buffer[i].rolling='A';
		buffer[i].tilt='B';
		buffer[i].orientation='C';
	}
}


void callbackTimerTx(void){ //callback para transmision de datos por UART

	sendPackage(*(messagePointer++) ); //manda el siguiente paquete

	if(messagePointer == &message4CPU[CANT_DISP-1][CANT_PACKAGE-1]){ // SI SE TERMINA EL ARREGLO QUE VUELVA AL PRINCIPIO
		messagePointer = &message4CPU[0][0];
	}
}


void callbackTimerRx(void){ //Callback para recepecion de datos de UART

}
