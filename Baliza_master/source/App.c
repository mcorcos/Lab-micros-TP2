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
#include "drivers/drv_I2C.h"
#include "drivers/drv_CAN.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
//ID de UART0
#define DEFAULT_ID 0

//TIMERS MS DEFINE
#define TIMER_TX_MS 10
#define TIMER_EVERY1500_MS 1500
#define TIMER_UPDATE_MS 50
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
static tim_id_t timerCanEvery1500mSecTx;
static tim_id_t timerCanUpdateMovement_R_Tx;
static tim_id_t timerCanUpdateMovement_C_Tx;
//static tim_id_t timerCanUpdateMovement_O_Tx;


//Dispositives es el arreglo de informacion donde se guarda la info de las placas, y el mismo se manda a la cpu por medio de msg4CPU
//bufferDisp es el arreglo donde guardo la informacion proviniente de CAN y de Acc y magnetometro
static dispositive_t dispositives[CANT_DISP];
static dispositive_t bufferDisp[CANT_DISP];


//Mensaje que se envia a la computadora por UART
static package_t message4CPU[CANT_DISP][CANT_PACKAGE+1]; // +1 its used to add '/n'
static package_t * messagePointer = &(message4CPU[0][0]);


static packageCan_t canPackageTx[3];
static packageCan_t tempPackage; //temporal para enviat
static int16_t currentPackage; //temporal para enviat
static int16_t previousPackage; //temporal para enviat


static bool sensorInit = false;

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
void byteToChars(unsigned char byte, uint8_t *result);
void receiveBoardsPos(void);
void sendPos2Boards(void);
void preparePackage(Measurement measurament);
void sendCompletePackageCan(void);
void sendUpdatedPackageCan_R_(void);
void sendUpdatedPackageCan_C_(void);
//void sendUpdatedPackageCan_O_(void);

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
	initI2c();
	initBoardsCan();
	SENSOR_CONTROL sensorStatus = initSensor();

	while(!sensorInit){

		if(sensorStatus == SENSOR_INITIALIZED){
			sensorInit = true;
		}
		else{
			initI2c();
			sensorStatus = initSensor();
		}

	}



	//inicio los timers de UART
	timerTx = timerGetId();
	timerRx = timerGetId();
	timerCanEvery1500mSecTx = timerGetId();
	timerCanUpdateMovement_R_Tx = timerGetId();
	timerCanUpdateMovement_C_Tx = timerGetId();

    timerStart(timerTx, TIMER_MS2TICKS(TIMER_TX_MS), TIM_MODE_PERIODIC, callbackTimerTx); //info para los timers
    timerStart(timerCanEvery1500mSecTx, TIMER_MS2TICKS(TIMER_EVERY1500_MS), TIM_MODE_PERIODIC, sendCompletePackageCan); //info para los timers
    timerStart(timerCanUpdateMovement_R_Tx, TIMER_MS2TICKS(TIMER_UPDATE_MS), TIM_MODE_PERIODIC, sendUpdatedPackageCan_R_); //info para los timers
    timerStart(timerCanUpdateMovement_C_Tx, TIMER_MS2TICKS(TIMER_UPDATE_MS), TIM_MODE_PERIODIC, sendUpdatedPackageCan_C_); //info para los timers
   // timerStart(timerCanUpdateMovement_O_Tx, TIMER_MS2TICKS(TIMER_UPDATE_MS), TIM_MODE_PERIODIC, sendUpdatedPackageCan_O_); //info para los timers

   // timerStart(timerRx, TIMER_MS2TICKS(TIMER_RX_MS), TIM_MODE_PERIODIC, callbackTimerRx);

    initDispositives(); //init a la info de las placas
    initBuffer(); // init para los buffers con valor dummy
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{

	// updateo las posiciones de mi placa y de las demas
	//updateDispositives();
	sendPos2Boards();


	ReadAccelMagnData();
	receiveBoardsPos();





}


/*******************************************************************************
 *******************************************************************************
						LOCAL FUNCTIONS DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


void updateDispositives(void){  //Actualizo la informacion de los buffers hacia el
	for(uint8_t i=0; i < CANT_DISP ; i++){
		dispositives[i].rolling = bufferDisp[i].rolling;
		dispositives[i].tilt = bufferDisp[i].tilt;
		dispositives[i].orientation = bufferDisp[i].orientation;
		updateMessage4CPU(i);
	}
}

void updateMessage4CPU(uint8_t i){
	message4CPU[i][0].dataType[0] = 'I';//poner el ID
	byteToChars( i ,message4CPU[i][0].value);

	if(dispositives[i].rolling >= 0){
		message4CPU[i][1].dataType[0] = '+';//poner el + o -
		byteToChars( dispositives[i].rolling,message4CPU[i][1].value );
	}
	else{
		message4CPU[i][1].dataType[0] = '-';//poner el + o -
		byteToChars((-1) * dispositives[i].rolling,message4CPU[i][1].value );
	}

	if(dispositives[i].tilt >= 0){
		message4CPU[i][2].dataType[0] = '+';//poner el + o -
		byteToChars( dispositives[i].tilt,message4CPU[i][2].value );

	}
	else{
		message4CPU[i][2].dataType[0] = '-';//poner el + o -
		byteToChars( (-1) *dispositives[i].tilt,message4CPU[i][2].value );

	}

	if(dispositives[i].orientation >= 0){
		message4CPU[i][3].dataType[0] = '+';//poner el + o -
		byteToChars( dispositives[i].orientation ,message4CPU[i][3].value);
	}
	else{
		message4CPU[i][3].dataType[0] = '-';//poner el + o -
		byteToChars( (-1) * dispositives[i].orientation ,message4CPU[i][3].value);
	}

	message4CPU[i][4].dataType[0] = 'T'; // T de terminador
	message4CPU[i][4].value[0] = '\n';// Pongo el terminador para que la aplicacion sepa que termino el msg !
	message4CPU[i][4].value[1] = '\n';
	message4CPU[i][4].value[2] = '\n';
}

void initDispositives(void){ // inicio los ids  de los dispositives
	for(uint8_t i=0;i<CANT_DISP;i++){
		dispositives[i].id = i + '0'; // Quiero poner el numero i en char, por eso le sumo el comienzo de los numeros que es'0'
	}
}

void initBuffer(void){
	for(uint8_t i=0;i<CANT_DISP;i++){
		bufferDisp[i].id = i + '0'; // Quiero poner el numero i en char, por eso le sumo el comienzo de los numeros que es'0'
		bufferDisp[i].rolling = 'A'; //Dummy letter para chequear que funciona
		bufferDisp[i].tilt = 'B';
		bufferDisp[i].orientation = 'C';
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
	receiveCAN(&measurament);
	uint8_t id  = measurament.boardID;

	bufferDisp[id].rolling = measurament.rolling;
	bufferDisp[id].tilt = measurament.tilt;
	bufferDisp[id].orientation = measurament.orientation;

}
void sendPos2Boards(void){ //antes aca se enviaba Can, queda cambiarle el nombre a la funcion como "update my boards position"

	rawdata_t accel = getAccData();
	rawdata_t magnet = getMagData();

	measurament = normalize(accel , magnet);
	preparePackage(measurament);

	//Updateo los datos de mi placa (bufferDisp[0])
	bufferDisp[4].rolling = measurament.rolling ;
	bufferDisp[4].tilt = measurament.tilt ;
	//bufferDisp[0].orientation = measurament.orientation ;
}


Measurement normalize(rawdata_t accel,rawdata_t magnet){

	Measurement m;

    // Calcular la magnitud de la aceleración (norma)
    float normalize = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

    // Normalizar los valores de aceleración
    accel.x /= normalize;
    accel.y /= normalize;
    accel.z /= normalize;

    // Calcular el ángulo de inclinación (tilt) en grados
    m.rolling = (int)(atan2(accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * (180.0 / M_PI));




    // Calcular el ángulo de balanceo (roll) en grados
    m.tilt = (int)(atan2(accel.y, accel.z) * (180.0 / M_PI));

    // Ajustar los ángulos al rango de -179° a 180°
    if (m.tilt > 180) {
        m.tilt -= 360;
    } else if (m.tilt < -179) {
        m.tilt += 360;
    }


    if(accel.x>0){
    	if(accel.z<0){
    		m.rolling = 90 + (90 - m.rolling);
    	}
    }
    if(accel.x<0){
    	if(accel.z<0){
    		m.rolling = -90 + (-90 - m.rolling);
    	}
    }

    if (m.rolling > 180) {
        m.rolling -= 360;
    } else if (m.rolling < -179) {
        m.rolling += 360;
    }

    return m;
}



void byteToChars(unsigned char byte, uint8_t *result) {
    if (byte > 99) {
        result[0] = '0' + (byte / 100);        // Obtener el dígito más significativo
        result[1] = '0' + ((byte % 100) / 10); // Obtener el segundo dígito
        result[2] = '0' + (byte % 10);         // Obtener el dígito menos significativo
    } else {
        result[0] = '0';
        if (byte > 9) {
            result[1] = '0' + (byte / 10);
            result[2] = '0' + (byte % 10);
        } else {
            result[1] = '0';
            result[2] = '0' + byte;
        }
    }
}



void preparePackage(Measurement measurament){

	canPackageTx[0].dataType[0] = 'R';

	if(measurament.rolling>=0){
		canPackageTx[0].sign = '+';
		byteToChars(measurament.rolling , canPackageTx[0].value);

	}
	else{
		canPackageTx[0].sign  = '-' ;
		byteToChars((-1)*measurament.rolling , canPackageTx[0].value);

	}

	canPackageTx[1].dataType[0] = 'C';

	if(measurament.tilt>=0){
		canPackageTx[1].sign  = '+';
		byteToChars(measurament.tilt , canPackageTx[1].value);
	}
	else{
		canPackageTx[1].sign  = '-' ;
		byteToChars((-1)*measurament.tilt , canPackageTx[1].value);
	}

}


void sendCompletePackageCan(void){

	tempPackage.dataType[0] = canPackageTx[0].dataType[0]; //Mando el primero
	tempPackage.sign = canPackageTx[0].sign;
	tempPackage.value[0] = canPackageTx[0].value[0];
	tempPackage.value[1] = canPackageTx[0].value[1];
	tempPackage.value[2] = canPackageTx[0].value[2];

	sendCan(&tempPackage);

	tempPackage.dataType[0] = canPackageTx[1].dataType[0]; //Mando el segundo
	tempPackage.sign = canPackageTx[1].sign;
	tempPackage.value[0] = canPackageTx[1].value[0];
	tempPackage.value[1] = canPackageTx[1].value[1];
	tempPackage.value[2] = canPackageTx[1].value[2];

	sendCan(&tempPackage);

}

void sendUpdatedPackageCan_R_(void){

	currentPackage = bufferDisp[0].rolling ;

	if(previousPackage != currentPackage){

		previousPackage = bufferDisp[0].rolling ;

		tempPackage.dataType[0] = canPackageTx[0].dataType[0]; //Mando el primero
		tempPackage.sign = canPackageTx[0].sign;
		tempPackage.value[0] = canPackageTx[0].value[0];
		tempPackage.value[1] = canPackageTx[0].value[1];
		tempPackage.value[2] = canPackageTx[0].value[2];
		sendCan(&tempPackage);
	}

}


void sendUpdatedPackageCan_C_(void){

	currentPackage = bufferDisp[0].tilt;;

	if(previousPackage != currentPackage){

		previousPackage = bufferDisp[0].tilt; ;

		tempPackage.dataType[0] = canPackageTx[1].dataType[0]; //Mando el segundo
		tempPackage.sign = canPackageTx[1].sign;
		tempPackage.value[0] = canPackageTx[1].value[0];
		tempPackage.value[1] = canPackageTx[1].value[1];
		tempPackage.value[2] = canPackageTx[1].value[2];

		sendCan(&tempPackage);
	}

}

/*
void sendUpdatedPackageCan_O_(void){


	sendCan(canPackageTx);


}*/





