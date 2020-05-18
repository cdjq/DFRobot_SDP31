/*!
 * @file DFRobot_SDP31.h
 * @brief Define the infrastructure and the implementation of the underlying method of the DFRobot_SDP31 class,
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2020-05-14
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_SDP31
 */

#include <DFRobot_SDP31.h>
#include"math.h"
DFRobot_SDP31::DFRobot_SDP31(TwoWire *pWire, uint8_t address)
{
  _pWire = pWire;
  _address = address;
  _mode = eOneShot;
}

int DFRobot_SDP31::begin()
{
  _pWire->begin();
  if(readSerialNumber() == 0){
    DBG("bus data access error");
    return ERR_DATA_BUS;
   }
  return ERR_OK;
}

uint32_t DFRobot_SDP31::readSerialNumber()
{
  uint32_t result = 0 ;
  uint8_t serialNumber1[3];
  uint8_t serialNumber2[3];
  uint8_t rawData[6];
  writeCommand(SDP31_READ_PRODUCT_IDENTIFIER,2);
  writeCommand(0xE102,2);
  delay(1);
  readData(rawData,6);
  memcpy(serialNumber1,rawData,3);
  memcpy(serialNumber2,rawData+3,3);
  if((checkCrc(serialNumber1) == serialNumber1[2]) && (checkCrc(serialNumber2) == serialNumber2[2])){
    result = serialNumber1[0];
    result = (result << 8) | serialNumber1[1];
    result = (result << 8) | serialNumber2[0];
    result = (result << 8) | serialNumber2[1];
  }
  else{
    return 0;
  }
  return result;
}
String DFRobot_SDP31::readUniqueSerialNumber()
{
  String serial;
  uint32_t serial1,serial2;
  uint32_t result = 0 ;
  uint8_t serialNumber1[3];
  uint8_t serialNumber2[3];
  uint8_t serialNumber4[3];
  uint8_t serialNumber5[3];
  uint8_t serialNumber6[3];
  uint8_t serialNumber7[3];
  uint8_t rawData[18];
  writeCommand(SDP31_READ_PRODUCT_IDENTIFIER,2);
  writeCommand(0xE102,2);
  delay(1);
  readData(rawData,18);
  memcpy(serialNumber1,rawData,3);
  memcpy(serialNumber2,rawData+3,3);

  if((checkCrc(serialNumber1) == serialNumber1[2]) && (checkCrc(serialNumber2) == serialNumber2[2])){
    serial1  = rawData[6];
    serial1  = serial1<<8 | rawData[7];
	serial1  = serial1<<8 | rawData[9];
	serial1  = serial1<<8 | rawData[10];

	serial2  = rawData[12];
	serial2  = serial2<<8 | rawData[13];
	serial2  = serial2<<8 | rawData[15];
	serial2  = serial2<<8 | rawData[16];
    serial = String(serial1) + String(serial2);
  }
  return serial;
}
void DFRobot_SDP31::softReset()
{
  uint8_t addr = _address;
  uint8_t  command[1] = {SDP31_SOFT_RESET};
  _address = 0x00;
  write(command,1);
  _address = addr;
  delay(20);
}


bool DFRobot_SDP31::startContinuousMode(eTempCompen_t tempcompen , eAveraging_t averaging){

  uint16_t command = 0;
  switch(tempcompen) {
    case MASSFLOW:
      _tempcompen = MASSFLOW;
      switch(averaging) {
        case AVGTILLREAD:
          command = SDP31_MASS_FLOW_AVG;
          break;
        case AVGNONE:
          command = SDP31_MASS_FLOW_NONE;
          break;
      }
      break;
    case DIFFPRESSURE:
      _tempcompen = DIFFPRESSURE;
      switch(averaging) {
        case AVGTILLREAD:
          command = SDP31_DIFFERENTIAL_PRESSURE_AVG;
          break;
        case AVGNONE:
          command = SDP31_DIFFERENTIAL_PRESSURE_NONE;
          break;
      }
      break;
  }
  writeCommand(command,2);
  delay(10);
  if(command != 0){
    _mode = ePeriodic;
    return true;
  } else {
    return false;
  }

}


DFRobot_SDP31::sDPAndTemp_t DFRobot_SDP31::readTempAndDP(eTempCompen_t tempcompen){
  uint8_t rawData[9];
  sDPAndTemp_t data;
  if(_mode == eOneShot){
    if(tempcompen == MASSFLOW)  writeCommand(SDP31_MASS_FLOW,2);
    else  writeCommand(SDP31_DIFFERENTIAL_PRESSURE,2);
    delay(100);
  }
  uint8_t retry = 10;
  while(retry){
  readData(rawData,9);
  int16_t diffPressureTicks = (rawData[0] << 8) | rawData[1];
  int16_t temperatureTicks = (rawData[3] << 8) | rawData[4];
  uint16_t scaleFactorDiffPressure = (rawData[6] << 8) | rawData[7];
  data.Temperature = (float)temperatureTicks/200;
  if(_mode == eOneShot){
    if(tempcompen == DIFFPRESSURE){
      data.DifferentialPressure = (float)diffPressureTicks/(float)scaleFactorDiffPressure;
    }
    else{
      data.MassFlow = (float)diffPressureTicks;
    }
  }
  else{
    if(_tempcompen == DIFFPRESSURE){
      data.DifferentialPressure = (float)diffPressureTicks/(float)scaleFactorDiffPressure;
    }
    else{
      data.MassFlow = (float)diffPressureTicks;
    }
  }
  if((checkCrc(rawData) == rawData[2]) && (checkCrc(rawData+3) == rawData[5])&& (checkCrc(rawData+6) == rawData[8]))
  {
    break;
  }
  }

  //Serial.println(data.DifferentialPressure);
  //Serial.println(data.Temperature);
  return data;
}
float DFRobot_SDP31::getTemperature(){
  sDPAndTemp_t data;
  data = readTempAndDP(DIFFPRESSURE);
  return data.Temperature;
}
int16_t DFRobot_SDP31::getDifferentialPressure(){
  sDPAndTemp_t data;
  data = readTempAndDP(DIFFPRESSURE);
  return data.DifferentialPressure;
}
int16_t DFRobot_SDP31::getMassFlow(){

  sDPAndTemp_t data;
  data = readTempAndDP(MASSFLOW);
  return data.MassFlow;
}
void DFRobot_SDP31::stopContinuousMode(){
  _mode = eOneShot;
  writeCommand(SDP31_STOP_CONTINUOUS_MEASUREMENT,2);
}

void DFRobot_SDP31::enterSleep(){
  writeCommand(SDP31_SLEEP_MODE,2);
}

uint8_t DFRobot_SDP31::checkCrc(uint8_t *data)
{
    uint8_t bit;
    uint8_t crc = 0xFF;

    for (uint8_t dataCounter = 0; dataCounter < 2; dataCounter++)
    {
        crc ^= (data[dataCounter]);
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

void DFRobot_SDP31::writeLimitData(uint16_t cmd,uint16_t limitData){
  uint8_t _pBuf[5];
  _pBuf[0] = cmd >>8;
  _pBuf[1] = cmd & 0xff;
  _pBuf[2] = limitData >> 8;
  _pBuf[3] = limitData & 0xff;
  uint8_t crc = checkCrc(_pBuf+2);
  _pBuf[4] = crc;
  write(_pBuf,5);
}

void DFRobot_SDP31::writeCommand(uint16_t cmd,size_t size)
{
  uint8_t _pBuf[2];
  _pBuf[0] = cmd >> 8;
  _pBuf[1] = cmd & 0xFF;
  delay(1);
  write(_pBuf,2);
}

void DFRobot_SDP31::write(const void* pBuf,size_t size)
{
  if (pBuf == NULL) {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_address);
  for (uint8_t i = 0; i < size; i++) {
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
}

uint8_t DFRobot_SDP31::readData(void *pBuf, size_t size) {
  if (pBuf == NULL) {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;

  _pWire->requestFrom(_address,size);
  uint8_t len = 0;
  for (uint8_t i = 0 ; i < size; i++) {
    _pBuf[i] = _pWire->read();
    len++;
  }
  _pWire->endTransmission();
  return len;
}
