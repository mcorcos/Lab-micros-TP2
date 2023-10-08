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

#include "drv_FXOS8700CQ.h"
 /*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


//Conversiones
#define ACC_CONVERSION	(0.000488)		//0.488 mG/LSB or 1/2048
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
static uint32_t i;

void callbackIsFinished(void);
void dataIsReady(void);
/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/



SENSOR_CONTROL initSensor(void){
	//configuracion del sensor
	SENSOR_CONTROL sensor1 = configSensor();
	return sensor1;
}





SENSOR_CONTROL configSensor(void){

	I2CBYTE databyte;
	loadCallback(callbackIsFinished);

	databyte = 0x00;

    // read and check the FXOS8700CQ WHOAMI register
    // 				slave address to write , slave intern register to W or R , tamanio  , READ/WRITE , Ptr to Fun
    i2cCommunicationHandler(FXOS8700CQ_WHOAMI,&databyte,(uint8_t)1,I2C_READ);

    for (i = 0; i < 1000000; i++) {
        // Este bucle creará un retraso aproximado de 1 segundo
    }

    while(i2cSensor.status == WORKING ){ // bloqueante, espero a que el mensaje anterior se termine de desarrollar

		if( getFault() != I2C_FAULT_NO_FAULT)
		{
			return (SENSOR_ERROR);
		}



    }
    i2cSensor.status = WORKING;
    if(databyte != FXOS8700CQ_WHOAMI_VAL){
    	return (SENSOR_ERROR);
    }
     /*write 0000 0000 = 0x00 to accelerometer control register 1 to place FXOS8700CQ into standby
     [7-1] = 0000 000
     [0]: active=0*/

    databyte = 0x00;

    i2cCommunicationHandler( FXOS8700CQ_CTRL_REG1, &databyte, 1,I2C_WRITE);

    for (i = 0; i < 1000000; i++) {
        // Este bucle creará un retraso aproximado de 1 segundo
    }

    while(i2cSensor.status == WORKING ){ // bloqueante, espero a que el mensaje anterior se termine de desarrollar

		if( getFault() != I2C_FAULT_NO_FAULT)
		{
			return (SENSOR_ERROR);
		}

    }
    i2cSensor.status = WORKING;

    /* write 0001 1111 = 0x1F to magnetometer control register 1
     [7]: m_acal=0: auto calibration disabled
     [6]: m_rst=0: no one-shot magnetic reset
     [5]: m_ost=0: no one-shot magnetic measurement
     [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
     [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active*/

    databyte = 0x1F;
    i2cCommunicationHandler( FXOS8700CQ_M_CTRL_REG1, &databyte, 1,	I2C_WRITE);

    for (i = 0; i < 1000000; i++) {
        // Este bucle creará un retraso aproximado de 1 segundo
    }

    while(i2cSensor.status == WORKING ){ // bloqueante, espero a que el mensaje anterior se termine de desarrollar

		if( getFault() != I2C_FAULT_NO_FAULT)
		{
			return (SENSOR_ERROR);
		}

    }
    i2cSensor.status = WORKING;

     /*write 0010 0000 = 0x20 to magnetometer control register 2
     [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the
     accelerometer registers
     [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
     [3]: m_maxmin_dis_ths=0
     [2]: m_maxmin_rst=0
     [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle*/

    databyte = 0x20;

    i2cCommunicationHandler( FXOS8700CQ_M_CTRL_REG2, &databyte, 1,	I2C_WRITE);

    for (i = 0; i < 1000000; i++) {
        // Este bucle creará un retraso aproximado de 1 segundo
    }

    while(i2cSensor.status == WORKING ){ // bloqueante, espero a que el mensaje anterior se termine de desarrollar

		if( getFault() != I2C_FAULT_NO_FAULT)
		{
			return (SENSOR_ERROR);
		}

    }
    i2cSensor.status = WORKING;


   /* // Voy a hacer el Enable del pin de Data Ready que esta en el REG4 del chip

	databyte=DATA_READY_ENABLE;

    i2cCommunicationHandler( FXOS8700CQ_M_CTRL_REG4, &databyte, 1,	I2C_WRITE);

    for (i = 0; i < 1000000; i++) {
        // Este bucle creará un retraso aproximado de 1 segundo
    }

    while(i2cSensor.status == WORKING ){ // bloqueante, espero a que el mensaje anterior se termine de desarrollar

		if( getFault() != I2C_FAULT_NO_FAULT)
		{
			return (SENSOR_ERROR);
		}

    }
    i2cSensor.status = WORKING;

    // Voy a hacer el routing de la interrupcion de data ready a el PC13 (datasheet de la kinetis y del chip)

	databyte=INTERRUPT_ROUTED_INT2;

    i2cCommunicationHandler( FXOS8700CQ_M_CTRL_REG5, &databyte, 1,	I2C_WRITE);

    for (i = 0; i < 1000000; i++) {
        // Este bucle creará un retraso aproximado de 1 segundo
    }

    while(i2cSensor.status == WORKING ){ // bloqueante, espero a que el mensaje anterior se termine de desarrollar

		if( getFault() != I2C_FAULT_NO_FAULT)
		{
			return (SENSOR_ERROR);
		}

    }
    i2cSensor.status = WORKING;*/


    /* write 0000 0001= 0x01 to XYZ_DATA_CFG register
     [4]: hpf_out=0
     [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB*/

    databyte = 0x01;
    i2cCommunicationHandler( FXOS8700CQ_XYZ_DATA_CFG, &databyte, 1,I2C_WRITE);

    for (i = 0; i < 1000000; i++) {
        // Este bucle creará un retraso aproximado de 1 segundo
    }

    while(i2cSensor.status == WORKING ){ // bloqueante, espero a que el mensaje anterior se termine de desarrollar

		if( getFault() != I2C_FAULT_NO_FAULT)
		{
			return (SENSOR_ERROR);
		}

    }
    i2cSensor.status = WORKING;

    /* write 0000 1101 = 0x0D to accelerometer control register 1
     [7-6]: aslp_rate=00
     [5-3]: dr=001 for 200Hz data rate (when in hybrid mode)
     [2]: lnoise=1 for low noise mode
     [1]: f_read=0 for normal 16 bit reads
     [0]: active=1 to take the part out of standby and enable sampling*/

    databyte = 0x0D;
    i2cCommunicationHandler( FXOS8700CQ_CTRL_REG1, &databyte, 1,I2C_WRITE);

    for (i = 0; i < 1000000; i++) {
        // Este bucle creará un retraso aproximado de 1 segundo
    }

    while(i2cSensor.status == WORKING ){ // bloqueante, espero a que el mensaje anterior se termine de desarrollar

		if( getFault() != I2C_FAULT_NO_FAULT)
		{
			return (SENSOR_ERROR);
		}

    }
    //Voy a setear el pin PC13 como entradad de interrupciones (active low default)

    //gpioMode (PIN_INT2_FXOS8700XQ, INPUT);
   //gpioIRQ (PIN_INT2_FXOS8700XQ, PORT_eInterruptFalling, dataIsReady);

    i2cSensor.status = SENSOR_INITIALIZED; //El sensor esta preparado para ser usado

    return SENSOR_INITIALIZED; //  si todo salio bien en la init
}






void callbackIsFinished(void){
	i2cSensor.status = FINISHED;
}


void dataIsReady(void){

	    i2cCommunicationHandler( FXOS8700CQ_OUT_X_MSB, buffer,FXOS8700CQ_READ_LEN,I2C_READ);
}

void callbackRead(void){ //Funcion llamada desde el i2cCommunications. pasa los datos del buffer hacia las estructuras


	int16_t accelerometerXAxis = (int16_t)(((buffer[1] << 8) | buffer[2])) >> 2; //paso de dos uint8  a una palabra de 14 bits
	int16_t accelerometerYAxis = (int16_t)(((buffer[3] << 8) | buffer[4])) >> 2;
	int16_t accelerometerZAxis = (int16_t)(((buffer[5] << 8) | buffer[6])) >> 2;

	int16_t magnetXAxis = (buffer[7] << 8) | buffer[8];
	int16_t magnetYAxis = (buffer[9] << 8) | buffer[10];
	int16_t magnetZAxis = (buffer[11] << 8) | buffer[12];

	accelerometer.x =accelerometerXAxis; //cargo los datos raw en accel y magnet
	accelerometer.y =accelerometerYAxis;
	accelerometer.z =accelerometerZAxis;
	magnet.x = magnetXAxis*MAG_CONVERSION;
	magnet.y = magnetYAxis*MAG_CONVERSION;
	magnet.z = magnetZAxis*MAG_CONVERSION;

	i2cSensor.status = FINISHED;

}



rawdata_t getMagData(void){
	return magnet;
}
rawdata_t getAccData(void){
	return accelerometer;
}


void ReadAccelMagnData(void)
{
	// read FXOS8700CQ_READ_LEN=13 bytes ( six channels of data)
	if(i2cSensor.status != WORKING ){ // bloqueante, espero a que el mensaje anterior se termine de desarrollar
		i2cSensor.status = WORKING ;
		loadCallback(callbackRead);
		i2cCommunicationHandler( FXOS8700CQ_STATUS, buffer,FXOS8700CQ_READ_LEN,I2C_READ);

	}



}


I2C_COM_CONTROL getStatus(void){
	return i2cSensor.status;
}

