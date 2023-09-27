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

#include <math.h>

#include "MCAL/board.h"
#include "timers/timer.h" // Llamo para Init timer
#include "drivers/drv_DEVBOARD.h" // Placa creada por el grupo con leds para debuggin
#include "drivers/drv_K64.h" // creamos funciones para acceder a switches y demas de la kinetis
#include "drivers/drv_UART.h"
#include "drivers/drv_FXOS8700CQ.h"
#include "drivers/drv_CAN.h"

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
static Measurement measurament;

//Timers para UART. Para transmitir parte del mensaje cada X ms
//Para chequear si se ingreso algo por teclado.
static tim_id_t timerTx;
static tim_id_t timerRx;


//Dispositives es el arreglo de informacion donde se guarda la info de las placas, y el mismo se manda a la cpu por medio de msg4CPU
//Buffer es el arreglo donde guardo la informacion proviniente de CAN y de Acc y magnetometro
static dispositive_t dispositives[CANT_DISP];
static dispositive_t buffer[CANT_DISP];


//Mensaje que se envia a la computadora por UART
static package_t message4CPU[CANT_DISP][CANT_PACKAGE+1]; // +1 its used to add '/n'
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
Measurement normalize(rawdata_t accel,rawdata_t magnet);

void receiveBoardsPos(void);
void sendPos2Boards(void);
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


    timerStart(timerTx, TIMER_MS2TICKS(TIMER_TX_MS), TIM_MODE_PERIODIC, callbackTimerTx); //info para los timers
    timerStart(timerRx, TIMER_MS2TICKS(TIMER_RX_MS), TIM_MODE_PERIODIC, callbackTimerRx);

    initDispositives(); //init a la info de las placas
    initBuffer(); // init para los buffers con valor dummy
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	// updateo las posiciones de mi placa y de las demas
	updateDispositives();
	receiveBoardsPos();
	sendPos2Boards();




}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTIONS DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


void updateDispositives(void){  //Actualizo la informacion de los buffers hacia el
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

	if(dispositives[i].rolling >= 0){
		message4CPU[i][1].dataType[0] = '+';//poner el + o -
		message4CPU[i][1].value = dispositives[i].rolling;
	}
	else{
		message4CPU[i][1].dataType[0] = '-';//poner el + o -
		message4CPU[i][1].value = (-1) * dispositives[i].rolling;
	}

	if(dispositives[i].tilt >= 0){
		message4CPU[i][2].dataType[0] = '+';//poner el + o -
		message4CPU[i][2].value = dispositives[i].tilt;
	}
	else{
		message4CPU[i][2].dataType[0] = '-';//poner el + o -
		message4CPU[i][2].value = (-1) * dispositives[i].tilt;
	}

	if(dispositives[i].orientation >= 0){
		message4CPU[i][3].dataType[0] = '+';//poner el + o -
		message4CPU[i][3].value = dispositives[i].orientation;
	}
	else{
		message4CPU[i][3].dataType[0] = '-';//poner el + o -
		message4CPU[i][3].value = (-1) * dispositives[i].orientation;
	}

	message4CPU[i][4].dataType[0] = 'T'; // T de terminador
	message4CPU[i][4].value = '\n'; // Pongo el terminador para que la aplicacion sepa que termino el msg !
}

void initDispositives(void){ // inicio los ids  de los dispositives
	for(uint8_t i=0;i<CANT_DISP;i++){
		dispositives[i].id = i + '0'; // Quiero poner el numero i en char, por eso le sumo el comienzo de los numeros que es'0'
	}
}

void initBuffer(void){
	for(uint8_t i=0;i<CANT_DISP;i++){
		buffer[i].id = i + '0'; // Quiero poner el numero i en char, por eso le sumo el comienzo de los numeros que es'0'
		buffer[i].rolling = 'A'; //Dummy letter para chequear que funciona
		buffer[i].tilt = 'B';
		buffer[i].orientation = 'C';
	}
}


void callbackTimerTx(void){ //callback para transmision de datos por UART




	if(messagePointer == &message4CPU[CANT_DISP-1][CANT_PACKAGE+1]){ // SI SE TERMINA EL ARREGLO QUE VUELVA AL PRINCIPIO
		messagePointer = &message4CPU[0][0];						// es 5 el limite para que se pueda imprimir la direccion 4
	}
	sendPackage(*(messagePointer++) ); //manda el siguiente paquete
}


void callbackTimerRx(void){ //Callback para recepecion de datos de UART

}


void receiveBoardsPos(void){
	receiveCAN(measurament);
	uint8_t id = measurament.boardID;

	buffer[id].rolling = measurament.rolling;
	buffer[id].tilt = measurament.tilt;
	buffer[id].orientation = measurament.orientation;

}
void sendPos2Boards(void){

	rawdata_t accel = getAccData();
	rawdata_t magnet = getMagData();

	measurament = normalize(accel , magnet);
	sendCan(measurament);

	//Updateo los datos de mi placa (Buffer[0])
	buffer[0].rolling = measurament.rolling;
	buffer[0].tilt = measurament.tilt;
	buffer[0].orientation = measurament.orientation;
}


Measurement normalize(rawdata_t accel,rawdata_t magnet){
	Measurement m;

	float normalize = sqrt(accel.x*accel.x+accel.y*accel.y+accel.z*accel.z); // suma de los cuadrados. Modulo

	accel.y/=normalize;
	accel.x/=normalize;
	accel.z/=normalize;

	m.rolling=(int)((180/3.1415)*atan2(accel.y,sqrt(accel.x*accel.x+accel.z*accel.z)));//conversion de radianes a grados
	m.tilt=(int)((180/3.1415)*atan2(accel.x,sqrt(accel.y*accel.y+accel.z*accel.z)));

	return m;
}



