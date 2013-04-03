// Dataacquisition and analysis
// by Raphael Nagel
// 10032284 - University of the West of England, Bristol
// Created 24 Januar 2013

// This software contacts the MPL3115A2 barometer via I2C and reads a single barometer value with 20bits
// accuracy. It calculates the averages over a 30 sample long period


#include <I2C.h>
#include <DigitalIO.h>
#include <math.h>

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
//0x02 -  1xOS: 90-154Hz
//0x0A -  2xOS: 70Hz
//0x12 -  4xOS: 45Hz
//0x1A -  8xOS: 26.47Hz-30Hz
//0x22 - 16xOS: 16.13Hz

//#define set_OST_OS 0x3A
#define set_PDEFE_DREM 0x3
#define DATA_READY_PRESSURE 0x4


#define s1 data_buffer[0]
#define s2 data_buffer[1]
#define s3 data_buffer[2]
#define s4 data_buffer[3]
#define s5 data_buffer[4]
#define s6 data_buffer[5]
#define s7 data_buffer[6]
#define s8 data_buffer[7]
#define s9 data_buffer[8]
#define s10 data_buffer[9]
#define s11 data_buffer[10]
#define s12 data_buffer[11]
#define s13 data_buffer[12]
#define Yes 1
#define No 0



//the raw input value in binary
struct triple{
  unsigned long s_0;
  unsigned long s_1;
  unsigned long s_2;
  unsigned long t;

} 
tri_raw;

unsigned long s0_log[13];
unsigned long s1_log[13];
unsigned long s2_log[13];


unsigned int last_raw = 0;
int derivative;

unsigned int old_time = 0;
unsigned int new_time = 0;

/*Derivative and threshold variables*/
long s0_dev, s1_dev,s2_dev;
long s0_old, s1_old, s2_old;
#define S0_THRES 60
#define S1_THRES 50
#define S2_THRES 50


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
  int x;
#ifdef DEBUG
  Serial.println("Hello World!");
#endif

  Soft_TripleBarometerRead(&tri_raw);

  /*Find the derivative and take the absolute value of that*/
  s0_dev = tri_raw.s_0-s0_old;
  s0_dev = abs(s0_dev);

  s1_dev = tri_raw.s_1-s1_old;  
  s1_dev = abs(s1_dev);

  s2_dev = tri_raw.s_2-s2_old;  
  s2_dev = abs(s2_dev);
  /********************************************************/

  s0_old = tri_raw.s_0;
  s1_old = tri_raw.s_1;
  s2_old = tri_raw.s_2;

  //store the last 13 measurements
  for(x = 12; x>0;x--){
    s0_log[x] = s0_log[x-1];
    s1_log[x] = s1_log[x-1];
    s2_log[x] = s2_log[x-1];

  }
  s0_log[0] = tri_raw.s_0;
  s1_log[0] = tri_raw.s_1;
  s2_log[0] = tri_raw.s_2;

  /******************* Send those timestamps ***************/
  if(s0_wave(s0_log)){
    Serial.print('0'); 
    Serial.print(' '); 
    Serial.print(tri_raw.t,DEC);
    Serial.println(""); 
  }

  if(s1_dev >= S1_THRES){
    Serial.print('1'); 
    Serial.print(' '); 
    Serial.print(tri_raw.t,DEC);
    Serial.println(""); 
  }

  if(s2_dev >= S2_THRES){
    Serial.print('2'); 
    Serial.print(' '); 
    Serial.print(tri_raw.t,DEC);
    Serial.println(""); 

  }

  /********************************************************/
  //  Serial.print(tri_raw.t,DEC);
  //  Serial.print(' ');
  //  Serial.print(tri_raw.s0,DEC);
  //  Serial.print(' ');
  //  Serial.print(tri_raw.s1,DEC);
  //  Serial.print(' ');
  //  Serial.print(tri_raw.s2,DEC);
  //  Serial.println(" ");
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

  tri_raw->s_0 = (unsigned long)((Data[2]<<10)|(Data[1]<<2)|(Data[0]>>6));   //all output data put together. 

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
  tri_raw->s_1 = (unsigned long)((Data[2]<<10)|(Data[1]<<2)|(Data[0]>>6));   //all output data put together. 

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

  tri_raw->s_2 = (unsigned long)((Data[2]<<10)|(Data[1]<<2)|(Data[0]>>6));   //all output data put together. 

}

int s0_wave(unsigned long data_buffer[13]){

  if((s8 >= 0.094754) && (s5 >= 0.114064) && (s7 >= 0.093751)){
    return Yes;
  };
  if((s12 >= 0.116425) && (s10 >= 0.090428) && (s12 >= 0.157013)){
    return Yes;
  }
  if((s1 >= 0.118435) && (s8 >= 0.07827) && (s7 >= 0.090501) && (s11 <= 0.049196)){
    return Yes;
  }
  if((s1 >= 0.107618) && (s5 >= 0.136535) && (s3 >= 0.039022) ){
    return Yes;
  }
  if((s12 >= 0.084135) && (s10 >= 0.041254) && (s9 <= 0.035335) && (s11 >= 0.081759) && (s9 >= 0.00658)){
    return Yes;
  }
  if((s12 >= 0.047582) && (s7 >= 0.086261) && (s10 >= 0.056804) && (s8 >= 0.097458) && (s11 >= 0.040183) && (s4 >= 0.043103)){
    return Yes;
  }
  if((s1 >= 0.094786) && (s10 >= 0.034625) && (s10 <= 0.081555) && (s7 >= 0.066904) && (s3 >= 0.075982)){
    return Yes;
  }
  if((s11 >= 0.047361) && (s5 >= 0.189817) && (s3 >= 0.084195)){
    return Yes;
  }
  if((s10 >= 0.032316) && (s1 >= 0.081805) && (s10 <= 0.081286) && (s11 >= 0.032578) && (s10 >= 0.058271) && (s3 >= 0.024199) && (s9 <= 0.04701)){
    return Yes;
  }
  if((s11 >= 0.071506) && (s5 <= 0.005717) && (s13 >= 0.072689) && (s6 >= 0.028328)){
    return Yes;
  }
  if((s12 >= 0.045506) && (s10 >= 0.032316) && (s6 <= 0.042945) && (s6 >= 0.02809) && (s4 <= 0.034184) && (s13 <= 0.052485) && (s1 >= 0.019355)){
    return Yes;
  }
  if((s1 >= 0.116402) && (s1 >= 0.16853) && (s5 >= 0.064349)){ 
    return Yes;
  }
  if((s11 >= 0.06454) && (s11 >= 0.243182)){ 
    return Yes;
  }
  if((s2 >= 0.086105) && (s5 <= 0.023498) && (s6 <= 0.043011) && (s6 >= 0.034493)){ 
    return Yes;
  }
  if((s8 >= 0.169368) && (s12 >= 0.125961)){ 
    return Yes;
  }
  if((s11 >= 0.06454) && (s7 >= 0.24207)){ 
    return Yes;
  }
  if((s1 >= 0.088711) && (s1 <= 0.090114)){ 
    return Yes;
  }
  if((s11 >= 0.065417) && (s4 <= 0.0403) && (s9 <= 0.02564) && (s9 >= 0.019506) && (s2 >= 0.065039)){ 
    return Yes;
  }
  if((s1 >= 0.12426) && (s10 >= 0.034625) && (s10 <= 0.036097)){ 
    return Yes;
  }
  if((s9 >= 0.168145) && (s4 >= 0.155626)){ 
    return Yes;
  }
  else{
    return No;
  }

}


int s1_wave(unsigned long data_buffer[13]){


  if((s8 >= 0.094754) && (s5 >= 0.114064) && (s7 >= 0.093751) && (s4 >= 0.038799) ){ 
    return Yes;
  } 
  if((s12 >= 0.116425) && (s10 >= 0.090428) && (s12 >= 0.157013) ){ 
    return Yes;
  }
  if((s1 >= 0.118435) && (s8 >= 0.07827) && (s7 >= 0.090501) && (s11 <= 0.049196) ){ 
    return Yes;
  } 
  if((s1 >= 0.107618) && (s5 >= 0.136535) && (s3 >= 0.039022) ){ 
    return Yes;
  }
  if((s12 >= 0.084135) && (s10 >= 0.041254) && (s9 <= 0.035335) && (s11 >= 0.081759) && (s9 >= 0.00658) ){ 
    return Yes;
  }
  if((s12 >= 0.047582) && (s7 >= 0.086261) && (s10 >= 0.056804) && (s8 >= 0.097458) && (s11 >= 0.040183) && (s4 >= 0.043103) ){ 
    return Yes;
  } 
  if((s1 >= 0.094786) && (s10 >= 0.034625) && (s10 <= 0.081555) && (s7 >= 0.066904) && (s3 >= 0.075982) ){
    return Yes;
  } 
  if((s11 >= 0.047361) && (s5 >= 0.189817) && (s3 >= 0.084195) ){ 
    return Yes;
  } 
  if((s10 >= 0.032316) && (s1 >= 0.081805) && (s10 <= 0.081286) && (s11 >= 0.032578) && (s10 >= 0.058271) && (s3 >= 0.024199) && (s9 <= 0.04701) ){ 
    return Yes;
  } 
  if((s11 >= 0.071506) && (s5 <= 0.005717) && (s13 >= 0.072689) && (s6 >= 0.028328) && (s9 <= 0.044201) ){ 
    return Yes;
  } 
  if((s12 >= 0.045506) && (s10 >= 0.032316) && (s6 <= 0.042945) && (s6 >= 0.02809) && (s4 <= 0.034184) && (s13 <= 0.052485) && (s1 >= 0.019355) ){ 
    return Yes;
  } 
  if((s1 >= 0.16853) && (s5 >= 0.064349) && (s7 >= 0.046448) ){ 
    return Yes;
  } 
  if((s11 >= 0.06454) && (s11 >= 0.243182) ){ 
    return Yes;
  } 
  if((s2 >= 0.086105) && (s5 <= 0.023498) && (s6 <= 0.043011) && (s6 >= 0.034493) ){ 
    return Yes;
  } 
  if((s8 >= 0.169368) && (s12 >= 0.125961) ){ 
    return Yes;
  } 
  if((s11 >= 0.06454) && (s7 >= 0.24207) ){ 
    return Yes;
  } 
  if((s1 >= 0.088711) && (s1 <= 0.090114) ){ 
    return Yes;
  }
  if((s11 >= 0.06454) && (s9 <= 0.02564) && (s9 >= 0.021842) && (s2 >= 0.065039) ){ 
    return Yes;
  } 
  if((s1 >= 0.12426) && (s10 >= 0.034625) && (s10 <= 0.036097) ){
    return Yes;
  } 
  if((s12 >= 0.068161) && (s10 >= 0.065693) && (s6 <= 0.038789) && (s6 >= 0.025867) && (s10 <= 0.080494) && (s9 <= 0.038529) ){ 
    return Yes;
  } 
  if((s9 >= 0.168145) && (s4 >= 0.155626) ){ 
    return Yes;
  } 
  if((s10 >= 0.054349) && (s10 <= 0.055448) ){ 
    return Yes;
  }
  else{
    return No;
  }

}

int s2_wave(unsigned long data_buffer[13]){

  if((s8 >= 0.094754) && (s5 >= 0.114064) && (s7 >= 0.093751) && (s4 >= 0.038799) ){ 
    return Yes;
  }

  if((s12 >= 0.116425) && (s10 >= 0.090428) && (s12 >= 0.157013) ){ 
    return Yes;
  }

  if((s1 >= 0.118435) && (s8 >= 0.07827) && (s7 >= 0.090501) && (s11 <= 0.049196) ){ 
    return Yes; 
  }

  if((s1 >= 0.107618) && (s5 >= 0.136535) && (s3 >= 0.039022) ){ 
    return Yes; 
  }

  if((s12 >= 0.084135) && (s10 >= 0.041254) && (s9 <= 0.035335) && (s11 >= 0.081759) && (s9 >= 0.00658) ){ 
    return Yes; 
  }

  if((s12 >= 0.047582) && (s7 >= 0.086261) && (s10 >= 0.056804) && (s8 >= 0.097458) && (s11 >= 0.040183) && (s4 >= 0.043103) ){ 
    return Yes; 
  }

  if((s1 >= 0.094786) && (s10 >= 0.034625) && (s10 <= 0.081555) && (s7 >= 0.066904) && (s3 >= 0.075982) ){ 
    return Yes; 
  }

  if((s11 >= 0.047361) && (s5 >= 0.189817) && (s3 >= 0.084195) ){ 
    return Yes; 
  }

  if((s10 >= 0.032316) && (s1 >= 0.081805) && (s10 <= 0.081286) && (s11 >= 0.032578) && (s10 >= 0.058271) && (s3 >= 0.024199) && (s9 <= 0.04701) ){ 
    return Yes; 
  }

  if((s11 >= 0.071506) && (s5 <= 0.005717) && (s13 >= 0.072689) && (s6 >= 0.028328) && (s9 <= 0.044201) ){ 
    return Yes; 
  }

  if((s12 >= 0.045506) && (s10 >= 0.032316) && (s6 <= 0.042945) && (s6 >= 0.02809) && (s4 <= 0.034184) && (s13 <= 0.052485) && (s1 >= 0.019355) ){ 
    return Yes; 
  }

  if((s1 >= 0.16853) && (s5 >= 0.064349) && (s7 >= 0.046448) ){ 
    return Yes; 
  }

  if((s11 >= 0.06454) && (s11 >= 0.243182) ){ 
    return Yes; 
  }

  if((s2 >= 0.086105) && (s5 <= 0.023498) && (s6 <= 0.043011) && (s6 >= 0.034493) ){ 
    return Yes; 
  }

  if((s8 >= 0.169368) && (s12 >= 0.125961) ){ 
    return Yes; 
  }

  if((s11 >= 0.06454) && (s7 >= 0.24207) ){ 
    return Yes; 
  }

  if((s1 >= 0.088711) && (s1 <= 0.090114) ){ 
    return Yes; 
  }

  if((s11 >= 0.06454) && (s9 <= 0.02564) && (s9 >= 0.021842) && (s2 >= 0.065039) ){ 
    return Yes; 
  }

  if((s1 >= 0.12426) && (s10 >= 0.034625) && (s10 <= 0.036097) ){ 
    return Yes; 
  }

  if((s12 >= 0.068161) && (s10 >= 0.065693) && (s6 <= 0.038789) && (s6 >= 0.025867) && (s10 <= 0.080494) && (s9 <= 0.038529) ){ 
    return Yes; 
  }

  if((s9 >= 0.168145) && (s4 >= 0.155626) ){ 
    return Yes; 
  }

  if((s10 >= 0.054349) && (s10 <= 0.055448) ){ 
    return Yes; 
  }
  else{
    return No;
  } 


}










