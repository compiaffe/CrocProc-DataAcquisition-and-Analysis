// Dataacquisition and analysis
// by Raphael Nagel
// 10032284 - University of the West of England, Bristol
// Created 24 Januar 2013

// This software contacts the MPL3115A2 barometer via I2C and reads a single barometer value with 20bits
// accuracy. It calculates the averages over a 30 sample long period


#include <I2C.h>
#include <DigitalIO.h>

#define CP_OK 1
#define CP_ERROR 0


#define baro 0x60
//#define DEBUG 1
//soft i2c
#define baro_read 0xC1 /*needed for the soft serial i2c*/
#define baro_write 0xC0
#define NACK  1
#define ACK  0

/*Barometer registers*/
#define DR_STATUS 0x00
#define OUT_P_MSB  0x01
#define OUT_P_CSB 0x02
#define OUT_P_LSB 0x03
#define CTRL_REG1 0x26 
#define PT_DATA_CFG 0x13
#define set_OST 0x02
#define set_OST_OS 0x02 //1 OS rate
//0x02 -  1xOS: 90-125Hz
//0x0A -  2xOS: 70Hz
//0x12 -  4xOS: 45Hz
//0x1A -  8xOS: 26.47Hz-30Hz
//0x22 - 16xOS: 16.13Hz

//#define set_OST_OS 0x3A
#define set_PDEFE_DREM 0x3
#define DATA_READY_PRESSURE 0x4



//the raw input value in binary
struct triple{
  unsigned long s0;
  unsigned long s1;
  unsigned long s2;
  unsigned long t;

} 
tri_raw;

unsigned int last_raw = 0;
int derivative;

unsigned int old_time = 0;
unsigned int new_time = 0;

/*Set Soft I2C pins */
const uint8_t SDA_PIN_0 = A4;
const uint8_t SCL_PIN_0 = A5;
const uint8_t SDA_PIN_1 = A2;
const uint8_t SCL_PIN_1 = A3;
const uint8_t SDA_PIN_2 = A0;
const uint8_t SCL_PIN_2 = A1;

/*********  SOFT I2C setup  ****************************************/
SoftI2cMaster i2c_0(SCL_PIN_0, SDA_PIN_0);
SoftI2cMaster i2c_1(SCL_PIN_1, SDA_PIN_1);
SoftI2cMaster i2c_2(SCL_PIN_2, SDA_PIN_2);


/********************FUNCTION PROTOTYPES************************************/

unsigned int Soft_TripleBarometerRead(triple *tri_raw);

/******************** VOID SETUP ******************************************/
void setup()
{    
  Serial.begin(57600);  // start serial for output

  /******************** Soft I2C *******************************************/

//  while (!Serial);
//
//  if (!digitalRead(SDA_PIN_1) && !digitalRead(SCL_PIN_1) && !digitalRead(SDA_PIN_2) && !digitalRead(SCL_PIN_2) && !digitalRead(SCL_PIN_0) && !digitalRead(SCL_PIN_0)) {
//    Serial.println("External pull-up resistors appear to be missing.");
//    Serial.println("Many false responses may be detected.");
//    Serial.println("Type any character to continue.");
//
//    while (!Serial.available());
//    Serial.println();
//
//  }

  /*Set up the data ready flags...*/
  i2c_0.start();
  i2c_0.write(baro_write);
  i2c_0.write(PT_DATA_CFG);
  i2c_0.write(set_PDEFE_DREM);
  i2c_0.stop();

  i2c_1.start();
  i2c_1.write(baro_write);
  i2c_1.write(PT_DATA_CFG);
  i2c_1.write(set_PDEFE_DREM);
  i2c_1.stop();


  i2c_2.start();
  i2c_2.write(baro_write);
  i2c_2.write(PT_DATA_CFG);
  i2c_2.write(set_PDEFE_DREM);
  i2c_2.stop();


}
/******************** VOID Loop ******************************************/

void loop(){
#ifdef DEBUG
  Serial.println("Hello World!");
#endif

  Soft_TripleBarometerRead(&tri_raw);
  Serial.print(tri_raw.t,DEC);
  Serial.print(' ');
  Serial.print(tri_raw.s0,DEC);
  Serial.print(' ');
  Serial.print(tri_raw.s1,DEC);
  Serial.print(' ');
  Serial.print(tri_raw.s2,DEC);
  Serial.println(" ");


  //****************************   measure the time taken... ******************
  /* old_time = new_time;
   new_time = micros();
   Serial.print("us: ");
   Serial.println((new_time-old_time),DEC);*/

  /* find the derivative.....*/
  //last_raw = raw;
  // derivative = raw-last_raw;

}


unsigned int Soft_TripleBarometerRead(triple *tri_raw){
  /*Temporary output data*/
  unsigned long Data[3];
  unsigned int buf;


  //************************************** sent barometers a read command ******************************
#ifdef DEBUG
  Serial.println("Sending 1st baro command");
#endif 
  //initiates a single barometer read on the first Barometer 
  i2c_0.start();
  i2c_0.write(baro_write);
  i2c_0.write(CTRL_REG1);
  i2c_0.write(set_OST_OS);
  i2c_0.stop();

#ifdef DEBUG
  Serial.println("Sending 2nd baro command");
#endif 
  //initiates a single barometer read on the second Barometer
  i2c_1.start();
  i2c_1.write(baro_write);
  i2c_1.write(CTRL_REG1);
  i2c_1.write(set_OST_OS);
  i2c_1.stop();
#ifdef DEBUG
  Serial.println("Sending 3rd baro command");
#endif 
  tri_raw->t = micros();

  //initiates a single barometer read on the third Barometer
  i2c_2.start();
  i2c_2.write(baro_write);
  i2c_2.write(CTRL_REG1);
  i2c_2.write(set_OST_OS);
  i2c_2.stop();

  //delay(500);
  //************************************* Check if data ready **************************************  
#ifdef DEBUG
  Serial.println("Checking 1st data ready");
#endif 
  i2c_0.start();  //read the status of the acquisition on the first baro 1111111111111111111111111111111111111
  i2c_0.write(baro_write);
  i2c_0.write(DR_STATUS);
  i2c_0.start();
  i2c_0.write(baro_read);
  buf = i2c_0.read(NACK);
  i2c_0.stop();

  while((buf&DATA_READY_PRESSURE) == 0){  //While it isnt ready, read the status register again
    i2c_0.start();
    i2c_0.write(baro_write);
    i2c_0.write(DR_STATUS);
    i2c_0.start();
    i2c_0.write(baro_read);
    buf = i2c_0.read(NACK);
    i2c_0.stop();
  }  
#ifdef DEBUG
  Serial.println("Checking 2nd data ready");
#endif 

  i2c_1.start();  //read the status of the acquisition on the second baro 2222222222222222222222222222
  i2c_1.write(baro_write);
  i2c_1.write(DR_STATUS);
  i2c_1.start();
  i2c_1.write(baro_read);
  //buf = i2c.read(ACK);
  buf = i2c_1.read(NACK);
  i2c_1.stop();

  while((buf&DATA_READY_PRESSURE) == 0){  //While it isnt ready, read the status register again
    i2c_1.start();
    i2c_1.write(baro_write);
    i2c_1.write(DR_STATUS);
    i2c_1.start();
    i2c_1.write(baro_read);
    buf = i2c_1.read(NACK);
    i2c_1.stop();
  }
#ifdef DEBUG
  Serial.println("Checking 3rd data ready");
#endif 
  i2c_1.start();  //read the status of the acquisition on the third baro 33333333333333333333333333333
  i2c_1.write(baro_write);
  i2c_1.write(DR_STATUS);
  i2c_1.start();
  i2c_1.write(baro_read);
  //buf = i2c.read(ACK);
  buf = i2c_1.read(NACK);
  i2c_1.stop();

  while((buf&DATA_READY_PRESSURE) == 0){  //While it isnt ready, read the status register again
    i2c_1.start();
    i2c_1.write(baro_write);
    i2c_1.write(DR_STATUS);
    i2c_1.start();
    i2c_1.write(baro_read);
    buf = i2c_1.read(NACK);
    i2c_1.stop();
  }
  //************************************ Finally read the data *************************************   
#ifdef DEBUG
  Serial.println("read 1st data ");
#endif 

  i2c_0.start(); //read from 2nd baro       1111111111111111111111
  i2c_0.write(baro_write);
  i2c_0.write(OUT_P_MSB);
  i2c_0.start();
  i2c_0.write(baro_read);

  Data[2] = i2c_0.read(ACK);
  Data[1] = i2c_0.read(ACK);
  Data[0] = i2c_0.read(NACK);
  i2c_0.stop();
#ifdef OUTPUT_DEBUG
  Serial.println((Data[2]<<10),BIN);
  Serial.println((Data[1]<<2),BIN);
  Serial.println((Data[0]>>6),BIN);

#endif
  //The bitmap of the value received from the barometer however places these at the 4 MSB positions of the 8-bit word received. 
  //The lower 4-bits are '0'. THus we rightshift to get rid of these.
  //CHANGED: we ignore the fraction now...
  //Serial.println(((MSB_Data<<10)|(CSB_Data<<2)|LSB_Data>>6),DEC);

  tri_raw->s0 = (unsigned long)((Data[2]<<10)|(Data[1]<<2)|(Data[0]>>6));   //all output data put together. 

#ifdef DEBUG
  Serial.println("read 2nd data ");
#endif 

  i2c_1.start(); //read from 2nd baro       2222222222222222222222222
  i2c_1.write(baro_write);
  i2c_1.write(OUT_P_MSB);
  i2c_1.start();
  i2c_1.write(baro_read);
  //i2c.read(ACK);
  Data[2] = i2c_1.read(ACK);
  Data[1] = i2c_1.read(ACK);
  Data[0] = i2c_1.read(NACK);
  i2c_1.stop();
  tri_raw->s1 = (unsigned long)((Data[2]<<10)|(Data[1]<<2)|(Data[0]>>6));   //all output data put together. 

#ifdef DEBUG
  Serial.println("read 3rd data ");
#endif 

  i2c_2.start(); //read from 3rd baro       333333333333333333333333
  i2c_2.write(baro_write);
  i2c_2.write(OUT_P_MSB);
  i2c_2.start();
  i2c_2.write(baro_read);
  //i2c.read(ACK);
  Data[2] = i2c_2.read(ACK);
  Data[1] = i2c_2.read(ACK);
  Data[0] = i2c_2.read(NACK);

  i2c_2.stop();

  tri_raw->s2 = (unsigned long)((Data[2]<<10)|(Data[1]<<2)|(Data[0]>>6));   //all output data put together. 

}












