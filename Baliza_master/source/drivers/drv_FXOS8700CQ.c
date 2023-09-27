/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   G4
 ******************************************************************************/

/* 

10.1.2 I2C read/write operations
  Single-byte read
    The master (or MCU) transmits a start condition (ST) to the FXOS8700CQ, followed by
    the slave address, with the R/W bit set to “0” for a write, and the FXOS8700CQ sends an
    acknowledgement. Then the master (or MCU) transmits the address of the register to read
    and the FXOS8700CQ sends an acknowledgement. The master (or MCU) transmits a
    repeated start condition (SR), followed by the slave address with the R/W bit set to “1” for a
    read from the previously selected register. The FXOS8700CQ then acknowledges and
    transmits the data from the requested register. The master does not acknowledge (NAK)
    the transmitted data, but transmits a stop condition to end the data transfer.
    Multiple-byte read
    When performing a multi-byte or “burst” read, the FXOS8700CQ automatically increments
    the register address read pointer after a read command is received. Therefore, after
    following the steps of a single-byte read, multiple bytes of data can be read from
    sequential registers after each FXOS8700CQ acknowledgment (AK) is received until a no
    acknowledge (NAK) occurs from the master followed by a stop condition (SP) signaling an
    end of transmission.
  Single-byte write
    To start a write command, the master transmits a start condition (ST) to the FXOS8700CQ,
    followed by the slave address with the R/W bit set to “0” for a write, and the FXOS8700CQ
    sends an acknowledgement. Then the master (or MCU) transmits the address of the
    register to write to, and the FXOS8700CQ sends an acknowledgement. Then the master
    (or MCU) transmits the 8-bit data to write to the designated register and the
    FXOS8700CQ sends an acknowledgement that it has received the data. Since this
    transmission is complete, the master transmits a stop condition (SP) to end the data
    transfer. The data sent to the FXOS8700CQ is now stored in the appropriate register. */



/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdint.h>
#include "drv_FXOS8700CQ.h"
#include "protocols/i2c.h"

 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR 0x1D



// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS 		0x00
#define FXOS8700CQ_WHOAMI 		0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 	0x2A
#define FXOS8700CQ_M_CTRL_REG1 	0x5B
#define FXOS8700CQ_M_CTRL_REG2 	0x5C
#define FXOS8700CQ_WHOAMI_VAL 	0xC7

// number of bytes to be read from the FXOS8700CQ
#define FXOS8700CQ_READ_LEN 	13

//Conversiones
#define ACC_CONVERSION	(0.000488)		//G
#define	MAG_CONVERSION	(0.1)		//uT

 /*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/







typedef struct{
	uint8_t status;
}i2c;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
static rawdata_t accelerometer ;
static rawdata_t magnet ;
static i2c i2cSensor;
static I2CBYTE buffer[FXOS8700CQ_READ_LEN]; // read buffer


void callbackIsFinished(void);
SENSOR_CONTROL callbackRead(void);
void callreadDataFromSensorback(void);
/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/



void initSensor(void){
	//configuracion del sensor
	configSensor();
	//Create a timer
	timerInit();
	tim_id_t timer = timerGetId();
	timerStart(timer, TIMER_MS2TICKS(200),TIM_MODE_PERIODIC,callreadDataFromSensorback);
}





SENSOR_CONTROL configSensor(void){

	I2CBYTE databyte;

    // read and check the FXOS8700CQ WHOAMI register
    // 				slave address to write , slave intern register to W or R , tamanio  , READ/WRITE , Ptr to Fun
    i2cCommunication(FXOS8700CQ_SLAVE_ADDR,FXOS8700CQ_WHOAMI,&databyte,(uint8_t)1,I2C_READ,callbackIsFinished);
    while(i2cSensor.status == WORKING ); // bloqueante, espero a que el mensaje anterior se termine de desarrollar
    i2cSensor.status = WORKING;
    if (databyte != FXOS8700CQ_WHOAMI_VAL) //Error en el sensor
    	return SENSOR_ERROR;

     /*write 0000 0000 = 0x00 to accelerometer control register 1 to place FXOS8700CQ into standby
     [7-1] = 0000 000
     [0]: active=0*/

    databyte = 0x00;

    i2cCommunication(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1, &databyte, 1,	I2C_WRITE, callbackIsFinished);

    while(i2cSensor.status == WORKING ); // bloqueante, espero a que el mensaje anterior se termine de desarrollar
    i2cSensor.status = WORKING;

    /* write 0001 1111 = 0x1F to magnetometer control register 1
     [7]: m_acal=0: auto calibration disabled
     [6]: m_rst=0: no one-shot magnetic reset
     [5]: m_ost=0: no one-shot magnetic measurement
     [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
     [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active*/

    databyte = 0x1F;
    i2cCommunication(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_M_CTRL_REG1, &databyte, 1,	I2C_WRITE, callbackIsFinished);
    while(i2cSensor.status == WORKING ); // bloqueante, espero a que el mensaje anterior se termine de desarrollar
    i2cSensor.status = WORKING;

     /*write 0010 0000 = 0x20 to magnetometer control register 2
     [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the
     accelerometer registers
     [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
     [3]: m_maxmin_dis_ths=0
     [2]: m_maxmin_rst=0
     [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle*/

    databyte = 0x20;

    i2cCommunication(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_M_CTRL_REG2, &databyte, 1,	I2C_WRITE, callbackIsFinished);
    while(i2cSensor.status == WORKING ); // bloqueante, espero a que el mensaje anterior se termine de desarrollar
    i2cSensor.status = WORKING;

    /* write 0000 0001= 0x01 to XYZ_DATA_CFG register
     [4]: hpf_out=0
     [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB*/

    databyte = 0x01;
    i2cCommunication(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_XYZ_DATA_CFG, &databyte, 1,I2C_WRITE, callbackIsFinished);
    while(i2cSensor.status == WORKING ); // bloqueante, espero a que el mensaje anterior se termine de desarrollar
    i2cSensor.status = WORKING;

    /* write 0000 1101 = 0x0D to accelerometer control register 1
     [7-6]: aslp_rate=00
     [5-3]: dr=001 for 200Hz data rate (when in hybrid mode)
     [2]: lnoise=1 for low noise mode
     [1]: f_read=0 for normal 16 bit reads
     [0]: active=1 to take the part out of standby and enable sampling*/

    databyte = 0x0D;
    i2cCommunication(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1, &databyte, 1,I2C_WRITE, callbackIsFinished);

    while(i2cSensor.status == WORKING ); // bloqueante, espero a que el mensaje anterior se termine de desarrollar
    i2cSensor.status = SENSOR_INITIALIZED; //El sensor esta preparado para ser usado

    return SENSOR_INITIALIZED; //  si todo salio bien en la init
}




void callreadDataFromSensorback(void){
					// la di4reccion del esclavo , la direccion de memoria a la cual leer o escribir, el tamanio del arreglo a leer o escribir, el modo de i2c , y la funcion a llamarse
	i2cCommunication(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_STATUS, buffer,FXOS8700CQ_READ_LEN, I2C_READ, callbackRead); // llamo a la funcion de i2cComm

}


void callbackIsFinished(void){
	i2cSensor.status = FINISHED;
}


SENSOR_CONTROL callbackRead(void){ //Funcion llamada desde el i2cCommunications. pasa los datos del buffer hacia las estructuras

	if(i2cSensor.status == SENSOR_INITIALIZED){


		int16_t accelerometerXAxis = (int16_t)(((buffer[1] << 8) | buffer[2])) >> 2; //paso de dos uint8  a una palabra de 14 bits
		int16_t accelerometerYAxis = (int16_t)(((buffer[3] << 8) | buffer[4])) >> 2;
		int16_t accelerometerZAxis = (int16_t)(((buffer[5] << 8) | buffer[6])) >> 2;

		int16_t magnetXAxis = (buffer[7] << 8) | buffer[8];
		int16_t magnetYAxis = (buffer[9] << 8) | buffer[10];
		int16_t magnetZAxis = (buffer[11] << 8) | buffer[12];

		accelerometer.x =accelerometerXAxis*ACC_CONVERSION; //cargo los datos raw en accel y magnet
		accelerometer.y =accelerometerYAxis*ACC_CONVERSION;
		accelerometer.z =accelerometerZAxis*ACC_CONVERSION;
		magnet.x = magnetXAxis*MAG_CONVERSION;
		magnet.y = magnetYAxis*MAG_CONVERSION;
		magnet.z = magnetZAxis*MAG_CONVERSION;

		return SENSOR_OK; //devuelvo ok

	}
	else{ // no esta inicializado aun

		return SENSOR_NOT_INITIALIZED; //devuelvo que aun no esta init

	}
}


rawdata_t getMagData(void){
	return magnet;
}
rawdata_t getAccData(void){
	return accelerometer;
}
