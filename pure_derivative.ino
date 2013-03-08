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
unsigned int old_time = 0;
unsigned int new_time = 0;
unsigned int x;
unsigned int buf;

/*Set Soft I2C pins */
const uint8_t SDA_PIN_1 = A2;
const uint8_t SCL_PIN_1 = A3;
const uint8_t SDA_PIN_2 = A0;
const uint8_t SCL_PIN_2 = A1;

/*********  SOFT I2C setup  ****************************************/
        SoftI2cMaster i2c_1(SCL_PIN_1, SDA_PIN_1);
        SoftI2cMaster i2c_2(SCL_PIN_2, SDA_PIN_2);


/********************FUNCTION PROTOTYPES************************************/

unsigned int SingleBarometerRead(unsigned long *);
unsigned int Soft_SingleBarometerRead_1(unsigned long *raw);
unsigned int Soft_SingleBarometerRead_2(unsigned long *raw);

/******************** VOID SETUP ******************************************/
void setup()
{    
  I2c.begin();        // join i2c bus (address optional for master)
  //TWBR = 12;         //set the I2C frequency to 400kHz 
  //I2c.pullup(0);
  I2c.write(baro,PT_DATA_CFG,set_PDEFE_DREM); //turn on data ready flags
  Serial.begin(9600);  // start serial for output
  
/******************** Soft I2C *******************************************/

while (!Serial);
  
  if (!digitalRead(SDA_PIN_1) && !digitalRead(SCL_PIN_1) && !digitalRead(SDA_PIN_2) && !digitalRead(SCL_PIN_2) && !digitalRead(A4) && !digitalRead(A5)) {
    Serial.println("External pull-up resistors appear to be missing.");
    Serial.println("Many false responses may be detected.");
    Serial.println("Type any character to continue.");

    while (!Serial.available());
    Serial.println();

  }

}
/******************** VOID Loop ******************************************/

void loop(){

    for(x = 1; x<4;x++){  //test all sensors
      switch(x){
      case 1:
        SingleBarometerRead(&raw);
        Serial.print("Sensor 1: ");
        Serial.print(raw,DEC);
       
        old_time = new_time;
        new_time = micros();
        Serial.print(" - ");
        Serial.print((old_time-new_time),DEC);
        Serial.println("us");

                break;
 
      case 2: 
        Soft_SingleBarometerRead_1(&raw);
        Serial.print("Sensor 2: ");
        Serial.print(raw,DEC);
        
        old_time = new_time;
        new_time = micros();
        Serial.print(" - ");
        Serial.print((old_time-new_time),DEC);
        Serial.println("us");

                break;
 
      case 3: 
        Soft_SingleBarometerRead_2(&raw);
        Serial.print("Sensor 3: ");
        Serial.print(raw,DEC);
       
        old_time = new_time;
        new_time = micros();
        Serial.print(" - ");
        Serial.print((old_time-new_time),DEC);
        Serial.println("us");
        
                break;  
      default:    
                break;                
      } 
     
     
        //time = micros(); //store the start time of our reads
        //last_raw = raw;
    
      /* find the derivative.....*/
       // derivative = last_raw-raw;

  }

}


unsigned int SingleBarometerRead(unsigned long *raw){


  /*Temporary output data*/
  unsigned long MSB_Data = 0; 
  unsigned long CSB_Data = 0; 
  unsigned long LSB_Data = 0; 


  I2c.write(baro,CTRL_REG1,set_OST_OS); //initiates a single barometer read.  
  
  /*Check for finished acquisition by checking the dataready flag*/
  //delay(500);
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

unsigned int Soft_SingleBarometerRead_1(unsigned long *raw){


  /*Temporary output data*/
  unsigned long Data[3];

//initiates a single barometer read. 
  i2c_1.start();
  i2c_1.write(baro_write);
  i2c_1.write(CTRL_REG1);
  i2c_1.write(set_OST_OS);
  i2c_1.stop();
  //buf = CTRL_REG1;
  //i2c.transfer(baro_write,&buf,1,I2C_CONTINUE); //send the barometer address and the register address
  //buf = set_OST_OS; 
  //i2c.transferContinue(&buf,1,I2C_STOP);//send the data to be written into the register

  
  /*Check for finished acquisition by checking the dataready flag*/
  //delay(500);
  
  //read the status of the acquisition -  I2c.read(baro_read,DR_STATUS,1);
  
  i2c_1.start();
  i2c_1.write(baro_write);
  i2c_1.write(DR_STATUS);
  i2c_1.start();
  i2c_1.write(baro_read);
  //buf = i2c.read(ACK);
  buf = i2c_1.read(NACK);
  i2c_1.stop();
    
  while((buf&DATA_READY_PRESSURE) == 0){ 
  i2c_1.start();
  i2c_1.write(baro_write);
  i2c_1.write(DR_STATUS);
  i2c_1.start();
  i2c_1.write(baro_read);
  //buf = i2c.read(ACK);
  buf = i2c_1.read(NACK);
  i2c_1.stop();
 //Serial.println(buf,BIN);
  }
   
   i2c_1.start();
   i2c_1.write(baro_write);
   i2c_1.write(OUT_P_MSB);
   i2c_1.start();
   i2c_1.write(baro_read);
   //i2c.read(ACK);
   Data[2] = i2c_1.read(ACK);
   Data[1] = i2c_1.read(ACK);
   Data[0] = i2c_1.read(NACK);
   i2c_1.stop();
  // buf = OUT_P_MSB;
  //i2c.transfer(baro_write,&buf,1,I2C_REP_START); //send the barometer address and the register address
  //i2c.transfer(baro_read,&Data,3,I2C_STOP); //read the value into the buffer

  //The bitmap of the value received from the barometer however places these at the 4 MSB positions of the 8-bit word received. 
  //The lower 4-bits are '0'. THus we rightshift to get rid of these.
  //CHANGED: we ignore the fraction now...
  //Serial.println(((MSB_Data<<10)|(CSB_Data<<2)|LSB_Data>>6),DEC);


  *raw = (unsigned long)((Data[2]<<10)|(Data[1]<<2)|Data[0]>>6);   //all output data put together. 
  if(raw != 0){
    return CP_OK;
  }
  else{
    return CP_ERROR;
  }

}

unsigned int Soft_SingleBarometerRead_2(unsigned long *raw){


  /*Temporary output data*/
  unsigned long Data[3];

//initiates a single barometer read. 
  i2c_1.start();
  i2c_1.write(baro_write);
  i2c_1.write(CTRL_REG1);
  i2c_1.write(set_OST_OS);
  i2c_1.stop();
  //buf = CTRL_REG1;
  //i2c.transfer(baro_write,&buf,1,I2C_CONTINUE); //send the barometer address and the register address
  //buf = set_OST_OS; 
  //i2c.transferContinue(&buf,1,I2C_STOP);//send the data to be written into the register

  
  /*Check for finished acquisition by checking the dataready flag*/
  //delay(500);
  
  //read the status of the acquisition -  I2c.read(baro_read,DR_STATUS,1);
  
  i2c_1.start();
  i2c_1.write(baro_write);
  i2c_1.write(DR_STATUS);
  i2c_1.start();
  i2c_1.write(baro_read);
  //buf = i2c.read(ACK);
  buf = i2c_1.read(NACK);
  i2c_1.stop();
    
  while((buf&DATA_READY_PRESSURE) == 0){ 
  i2c_1.start();
  i2c_1.write(baro_write);
  i2c_1.write(DR_STATUS);
  i2c_1.start();
  i2c_1.write(baro_read);
  //buf = i2c.read(ACK);
  buf = i2c_1.read(NACK);
  i2c_1.stop();
 //Serial.println(buf,BIN);
  }
   
   i2c_1.start();
   i2c_1.write(baro_write);
   i2c_1.write(OUT_P_MSB);
   i2c_1.start();
   i2c_1.write(baro_read);
   //i2c.read(ACK);
   Data[2] = i2c_1.read(ACK);
   Data[1] = i2c_1.read(ACK);
   Data[0] = i2c_1.read(NACK);
   i2c_1.stop();
  // buf = OUT_P_MSB;
  //i2c.transfer(baro_write,&buf,1,I2C_REP_START); //send the barometer address and the register address
  //i2c.transfer(baro_read,&Data,3,I2C_STOP); //read the value into the buffer

  //The bitmap of the value received from the barometer however places these at the 4 MSB positions of the 8-bit word received. 
  //The lower 4-bits are '0'. THus we rightshift to get rid of these.
  //CHANGED: we ignore the fraction now...
  //Serial.println(((MSB_Data<<10)|(CSB_Data<<2)|LSB_Data>>6),DEC);


  *raw = (unsigned long)((Data[2]<<10)|(Data[1]<<2)|Data[0]>>6);   //all output data put together. 
  if(raw != 0){
    return CP_OK;
  }
  else{
    return CP_ERROR;
  }

}




