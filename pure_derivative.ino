// Dataacquisition and analysis
// by Raphael Nagel
// 10032284 - University of the West of England, Bristol
// Created 24 Januar 2013

// This software contacts the MPL3115A2 barometer via I2C and reads a single barometer value with 20bits
// accuracy. It calculates the averages over a 30 sample long period


#include <I2C.h>
#define CP_OK 1
#define CP_ERROR 0


#define baro 0x60

/*Barometer registers*/
#define DR_STATUS 0x00
#define OUT_P_MSB  0x01
#define OUT_P_CSB 0x02
#define OUT_P_LSB 0x03
#define CTRL_REG1 0x26 
#define PT_DATA_CFG 0x13
#define set_OST 0x02
#define set_OST_OS 0x3A
#define set_PDEFE_DREM 0x3
#define DATA_READY_PRESSURE 0x4



unsigned long raw = 0; //the raw input value in binary
unsigned int last_raw = 0;
int derivative;

int output = 0; //the output value after it has been worked - in binary

unsigned int counter = 1;
unsigned int i;
unsigned int delay_val = 1800;
unsigned int time = 0;

/********************FUNCTION PROTOTYPES************************************/

//unsigned int SingleBarometerRead(unsigned long *);


/******************** VOID SETUP ******************************************/
void setup()
{
  I2c.begin();        // join i2c bus (address optional for master)
  TWBR = 12;         //set the I2C frequency to 400kHz 
  I2c.pullup(1);
  I2c.write(baro,PT_DATA_CFG,set_PDEFE_DREM); //turn on data ready flags
  Serial.begin(9600);  // start serial for output
}


/******************** VOID Loop ******************************************/

void loop()
{

  time = micros(); //store the start time of our reads
  last_raw = raw;
  SingleBarometerRead(&raw);


  /* find the derivative.....*/
  derivative = last_raw-raw;

  /************************************************************ OUTPUT ********************************************/
  Serial.println(raw,DEC);

  /************************************************************ OUTPUT ********************************************/

  //  PrintSignedNumber(derivative);
}






unsigned int SingleBarometerRead(unsigned long *raw){


  /*Temporary output data*/
  unsigned long MSB_Data = 0; 
  unsigned long CSB_Data = 0; 
  unsigned long LSB_Data = 0; 


  I2c.write(baro,CTRL_REG1,set_OST_OS); //initiates a single barometer read.  
  
  /*Check for finished acquisition by checking the dataready flag*/
  delay(500);
  I2c.read(baro,DR_STATUS,1);
  while((I2c.receive()&DATA_READY_PRESSURE) == 0){ 
    I2c.read(baro,DR_STATUS,1);
  }


  I2c.read(baro,OUT_P_MSB,3);
  MSB_Data = I2c.receive();//these are the 8 MSB out of the total of 20-bit
  CSB_Data = I2c.receive();//these are the 8 CSB out of a total of 20-bits
  LSB_Data = I2c.receive();//these are the 4 LSB of the 20-bit output. 
  //The bitmap of the value received from the barometer however places these at the 4 MSB positions of the 8-bit word received. 
  //The lower 4-bits are '0'. THus we rightshift to get rid of these.
  //CHANGED: we ignore the fraction now...
  //Serial.println(((MSB_Data<<10)|(CSB_Data<<2)|LSB_Data>>6),DEC);


  *raw = (unsigned long)((MSB_Data<<10)|(CSB_Data<<2)|LSB_Data>>6);   //all output data put together. 
  if(raw != 0){
    return CP_OK;
  }
  else{
    return CP_ERROR;
  }

}






