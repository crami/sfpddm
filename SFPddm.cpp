/*
  SFPddm.cpp - SFPddm library

Copyright 2013 Luka Mustafa - Musti, musti@wlan-si.net

This file is part of the SFPddm library for Arduino

The SFPddm library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your
option) any later version.

The SFPddm library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the SFPddm library. If not, see http://www.gnu.org/licenses/.

*/


// Includes

#include "Arduino.h"
#include "SFPddm.h"
#include "inttypes.h"
#include "I2C.h"

// Definitions

#define INFOADDR    0x50 // addr A0/1
#define DDMADDR     0x51 // addr A2/3

// Private variables

// error variable
uint8_t error;

// calibration data variables
struct _cal{
  uint16_t txc_slope;
  int16_t txc_off;
  uint16_t txp_slope;
  int16_t txp_off;
  uint16_t t_slope;
  int16_t t_off;
  uint16_t v_slope;
  int16_t v_off;
};
_cal cal_general = {1,0,1,0,1,0,1,0};

float cal_rxpower[5];
//raw measurement buffer
uint8_t raw_buffer[22];
//raw alarm and warnign buffer
uint8_t raw_alarmwarning[6];
//measurement values

struct _transchar {
  uint8_t identifier;
  uint8_t extidentifier;
  uint8_t connector;
  uint8_t transciever_res;
  uint8_t transciever_sonet1;
  uint8_t transciever_sonet2;
  uint8_t transciever_ethernet;
  uint8_t transciever_fc_ll;
  uint8_t transciever_fc_tt;
  uint8_t transciever_fc_tm;
  uint8_t transciever_fc_speed;
  uint8_t encoding;
  uint8_t bitrate;
  uint8_t RESERVED;
  uint8_t length9mkm;
  uint8_t length9m;
  uint8_t length50m;
  uint8_t length625m;
  uint8_t lengthcu;
  uint16_t wavelength;
}; // 19 bytes + 2 bytes
_transchar transchar={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

struct _meas {
  int16_t temperature; //reg A2/96-97
  uint16_t voltage; //reg A2/98-99
  uint16_t TXcurrent; //reg A2/100-101
  uint16_t TXpower; //reg A2/102-103
  uint16_t RXpower; //reg A2/104-105
  uint32_t RESERVED; //reg A2/106-109
  uint8_t RESERVED2; //reg A2/111 Intentional swapping due to eandiness adjustment in writing!
  uint8_t status; //reg A2/110 Intentional swapping due to eandiness adjustment in writing!
  uint16_t alarms; //reg A2/112-113
  uint16_t RESERVED3; //reg A2/114-115
  uint16_t warnings; //reg A2/116-117
};
_meas measdata={0,0,0,0,0,0,0,0,0,0,0};
//supported modes flags
uint8_t supported;
//contains register A0/92

//supported ddm modes flags
uint8_t ddmmodes;
//contains register A0/93

// Constructor /////////////////////////////////////////////////////////////////

SFPddm::SFPddm()
{
    //reset error code
    error=0x00;
}

// Public Methods //////////////////////////////////////////////////////////////

// The function initializes the communication, checks if the module is present, retrieves necessary information
uint8_t SFPddm::begin(void){
  I2c.begin();
  I2c.pullup(true);
  I2c.setSpeed(1); //400kHz
  //I2c.timeOut(500); //500ms timeout

  //reset error
  error=0x00;

  #ifdef HWSFP
  // configure pins
  pinMode(TX_DISABLE, OUTPUT);
  pinMode(TX_FAULT, INPUT);
  pinMode(RX_LOS, INPUT);
  pinMode(RATE_SELECT, OUTPUT);
  pinMode(MOD_DEF_0, INPUT);

  // test if the device is inserted
  if(digitalRead(MOD_DEF_0)!=HIGH){
    error=0xFF;
    return error;
  }
  #endif

  // test device communication and read modes
  error|=I2c.read(INFOADDR,92, 1,&supported);
  // stop if not present
  if(error){
    return error;
  }
  // if DDM mode is supported and externally callibrated
  if(supported&0x40){
    // check which DDM modes are supported
    error|=I2c.read(INFOADDR,93, 1,&ddmmodes);
    getCalibrationData();
  }

  return error;
}

// The function returns the 128 bytes of information at address 0xA0
void SFPddm::getRawInfo(uint8_t addr, uint8_t *data){
//implement when required
}

// The function returns an OR or all error codes detected, should be 0x00 if data is to be valid
// Can be used to check if the module is present
uint8_t SFPddm::getStatus(){
  // Do a test write to register pointer.
  error|=I2c.write(INFOADDR, 0x00);
  return error;
}

// This function can be used to get the supported information
uint8_t SFPddm::getSupported(){
  return supported;
}

// This function can be used to get the DDM supported control flags
uint8_t SFPddm::getDDMmodes(){
  return ddmmodes;
}

// The function acquires the measurements and returns an error code if sth went wrong, 0x00 is OK
uint8_t SFPddm::readMeasurements(){
  uint8_t i;
  //read diagnostic measurements registers 96-105 of 0xA2, store them in buffer
  error|=I2c.read(DDMADDR, 96, 22, (byte*)&raw_buffer);

  //copy raw measurements to results union
  uint8_t *p_meas = (uint8_t*)&measdata;
  for(i=0;i<22;i+=2){
    *p_meas++ =raw_buffer[i+1];
    *p_meas++ =raw_buffer[i];
  }

  //get raw values from hardware pins if enabled
  // txfaultstate (bit 2)
  // rxlosstate (bit 1)
  #ifdef HWSFP
  //mask
  measdata.status&=0xF9;
  //read
  if(digitalRead(TX_FAULT)==HIGH){
    measdata.status|=0x04;
  }
  //read
  if(digitalRead(RX_LOS)==HIGH){
    measdata.status|=0x02;
  }
  #endif

  //calibration if external data
  if(supported&0x10){
    measdata.temperature=calibrateTemperature(measdata.temperature, cal_general.t_slope, cal_general.t_off);
    measdata.voltage=calibrateMeasurement(measdata.voltage, cal_general.v_slope, cal_general.v_off);
    measdata.TXcurrent=calibrateMeasurement(measdata.TXcurrent, cal_general.txc_slope, cal_general.txc_off);
    measdata.TXpower=calibrateMeasurement(measdata.TXpower, cal_general.txp_slope, cal_general.txp_off);
    measdata.RXpower=calibrateRXpower(measdata.RXpower, &cal_rxpower[0]);
  }

  return error;
}

uint8_t SFPddm::getVendor(char * vendor){
  error|=I2c.read(INFOADDR, 20, 16, (byte*)vendor);
  return error;
}

uint8_t SFPddm::getPartNr(char * partnr){
  error|=I2c.read(INFOADDR, 40, 16, (byte*)partnr);
  return error;
}

uint8_t SFPddm::getSerial(char * serial){
  error|=I2c.read(INFOADDR, 68, 16, (byte*)serial);
  return error;
}

uint8_t SFPddm::readTransChar(){
  error|=I2c.read(INFOADDR, 0, 19, (byte*)&transchar);
  error|=I2c.read(INFOADDR, 60, 2, (byte*)&raw_buffer);

  uint8_t *p_wl = (uint8_t*)&transchar.wavelength;
  *p_wl++ = raw_buffer[1];
  *p_wl = raw_buffer[0];

  return error;
}

uint8_t SFPddm::getBR(){
  return transchar.bitrate;
}

uint8_t SFPddm::getLength9m(){
  return transchar.length9m;
}

uint8_t SFPddm::getLength9mkm(){
  return transchar.length9mkm;
}

uint8_t SFPddm::getLength50m(){
  return transchar.length50m;
}

uint8_t SFPddm::getLength625m(){
  return transchar.length625m;
}

uint8_t SFPddm::getLengthcu(){
  return transchar.lengthcu;
}

uint16_t SFPddm::getLength(){
  uint16_t length = 0;

  if (transchar.length9mkm > 0) {
    length = transchar.length9mkm * 256;
  } else if (transchar.length9m > 0) {
    length = transchar.length9m;
  } else if (transchar.length50m > 0) {
    length = transchar.length50m;
  } else if (transchar.length625m > 0) {
    length = transchar.length625m;
  } else if (transchar.lengthcu > 0) {
    length = transchar.lengthcu;
  }
  return length;
}

uint16_t SFPddm::getWavelength(){
  return transchar.wavelength;
}

// The function gets the value of the control register (0xA2 memory, register 110)
uint8_t SFPddm::getControl(){
  return measdata.status;
}
// The function sets the value of the control register (0xA2 memory, register 110)
void SFPddm::setControl(uint8_t data){
  // Software control
  #ifndef HWSFP
  //write the byte (not all bits are writable!)
  error|=I2c.write(DDMADDR,110, 0x40);
  #endif

  // Hardware control if enabled
  #ifdef HWSFP
  if((data&0x40)!=0x00){
    digitalWrite(TX_DISABLE, HIGH);
  }
  if((data&0x10)!=0x00){
    digitalWrite(TX_DISABLE, HIGH);
  }
  #endif
}

// The function returns the temperature , signed.
int16_t SFPddm::getTemperature(){
  return measdata.temperature;
}

// The function returns the supply voltage , unsigned
uint16_t SFPddm::getVoltage(){
  return measdata.voltage;
}

// The function returns the supply current , unsigned
uint16_t SFPddm::getTXcurrent(){
  return measdata.TXcurrent;
}

// The function returns the TX power , unsigned
uint16_t SFPddm::getTXpower(){
  return measdata.TXpower;
}

// The function returns the RX power , unsigned
uint16_t SFPddm::getRXpower(){
  return measdata.RXpower;
}

uint16_t SFPddm::getAlarms(){
  return measdata.alarms;
}

uint16_t SFPddm::getWarnings(){
  return measdata.warnings;
}

// Concert uW to dBm
double SFPddm::uWtodBm(int uw) {
  double dbm;
  dbm=10 * log10(uw/1000.0);
  return dbm;
}


// Private Methods /////////////////////////////////////////////////////////////

// The function retrieves callibration values
void SFPddm::getCalibrationData(){
  // buffer for data
  uint8_t calData[36];

  //read data
  error|=I2c.read(DDMADDR, 56, 36, &calData[0]);
  //loop variable
  uint8_t i;

  //writing binary to the float register using pointers
  uint8_t *pRXcal = (uint8_t*)&cal_rxpower;

  for(i=0;i<20;i++){
    //this goes from 0xA2 SFP bytes 56-75
    //write data to pointer location and decrement pointer
    //beware, low byte is LSB in Arduino
    *pRXcal =calData[19-i];
    pRXcal++;
  }

  //writing to uint16_t register using pointers
  //creating a pointer
  uint8_t *pCal = (uint8_t*)&cal_general;

  for(i=20;i<36;i+=2){
  //this goes from 0xA2 SFP bytes 76-91
  //write data to pointer location and increment pointer
  //beware od the endiness
    *pCal++ =calData[i+1];
    *pCal++ =calData[i];
  }
}

// This function calibrates all values except RX power and temperature
uint16_t SFPddm::calibrateMeasurement(uint16_t rawdata, uint16_t slope, int16_t offset)
{
  int32_t temporary = slope;
  temporary *= rawdata;
  //safe to use signed for all function, values do not overflow
  int16_t result = (temporary>>8 + offset);

  return result;
}

// This function calibrates temperature
int16_t SFPddm::calibrateTemperature(int16_t rawdata, uint16_t slope, int16_t offset)
{
  int32_t temporary = slope;
  temporary *= rawdata;
  //safe to use signed for all function, values do not overflow
  int16_t result = (temporary>>8 + offset);

  return result;
}

uint16_t SFPddm::calibrateRXpower(uint16_t rawdata, float *calibrationRX)
{
  uint16_t result;
  float temporary;

  temporary = calibrationRX[4] * rawdata;
  temporary += calibrationRX[3] * rawdata;
  temporary += calibrationRX[2] * rawdata;
  temporary += calibrationRX[1] * rawdata;
  temporary += calibrationRX[0]; //offset

  //Serial.print("RXcalibrated: ");
  //Serial.println(temporary);

  return (int16_t)temporary;
}
