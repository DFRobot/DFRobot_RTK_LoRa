/*!
 * @file DFRobot_RTK_LoRa.cpp
 * @brief Define the basic structure of the DFRobot_RTK_LoRa class, the implementation of the basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [ZhixinLiu](zhixin.liu@dfrobot.com)
 * @version V0.5.0
 * @date 2024-04-23
 * @url https://github.com/DFRobot/DFRobot_RTK_LoRa
 */
#include "DFRobot_RTK_LoRa.h"

DFRobot_RTK_LoRa::DFRobot_RTK_LoRa(){}
DFRobot_RTK_LoRa::~DFRobot_RTK_LoRa(){}

sTim_t DFRobot_RTK_LoRa::getDate(void)
{
  sTim_t data;
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_YEAR_H, _sendData, 4);
  data.year = ((uint16_t)_sendData[0] << 8) | _sendData[1];
  data.month = _sendData[2];
  data.date = _sendData[3];
  return data;
}

sTim_t DFRobot_RTK_LoRa::getUTC(void)
{
  sTim_t data;
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_HOUR, _sendData, 3);
  data.hour   = _sendData[0];
  data.minute = _sendData[1];
  data.second = _sendData[2];
  return data;
}

sLonLat_t DFRobot_RTK_LoRa::getLat(void)
{
  sLonLat_t data;
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_LAT_1, _sendData, 6);
  data.latDD  = _sendData[0];
  data.latMM = _sendData[1];
  data.latMMMMM = ((uint32_t)_sendData[2] << 16) | ((uint32_t)_sendData[3] << 8) | ((uint32_t)_sendData[4]);
  data.latitude = (double)data.latDD*100.0 + ((double)data.latMM) + ((double)data.latMMMMM / 100000.0);
  data.latitudeDegree = (double)data.latDD + (double)data.latMM/60.0 + (double)data.latMMMMM / 100000.0 / 60.0;
  data.latDirection = _sendData[5];
  return data;
}

sLonLat_t DFRobot_RTK_LoRa::getLon(void)
{
  sLonLat_t data;
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_LON_1, _sendData, 6);
  data.lonDDD  = _sendData[0];
  data.lonMM = _sendData[1];
  data.lonMMMMM = ((uint32_t)_sendData[2]<<16) | ((uint32_t)_sendData[3]<< 8) | ((uint32_t)_sendData[4]) ;
  data.lonitude = (double)data.lonDDD*100.0 + ((double)data.lonMM) + ((double)data.lonMMMMM / 100000.0);
  data.lonitudeDegree = (double)data.lonDDD + (double)data.lonMM/60.0 + (double)data.lonMMMMM / 100000.0/60.0;
  data.lonDirection = _sendData[5];
  return data;
}

uint8_t DFRobot_RTK_LoRa::getNumSatUsed(void)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_USE_STAR, _sendData, 1);
  return _sendData[0];
}

double DFRobot_RTK_LoRa::getAlt(void)
{
  double high;
  double sign = 1.0;
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_ALT_H, _sendData, 3);
  if(_sendData[0]&0x80){
    _sendData[0] &= 0x7F;
    sign = -1.0;
  }
  high = (double)((int16_t)(_sendData[0])<<8 | _sendData[1]) + (double)_sendData[2]/100.0;
  return high*sign;
}

double DFRobot_RTK_LoRa::getSep(void)
{
  double high;
  double sign = 1.0;
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_SEP_H, _sendData, 3);
  if(_sendData[0]&0x80){
    _sendData[0] &= 0x7F;
    sign = -1.0;
  }
  high = (double)((int16_t)(_sendData[0])<<8 | _sendData[1]) + (double)_sendData[2]/100.0;
  return high*sign;
}

double DFRobot_RTK_LoRa::getHdop(void)
{
  double hdop;
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_HDOP_Z, _sendData, 2);
  hdop = (double)_sendData[0] + (double)_sendData[1]/100.0;
  return hdop;
}

bool DFRobot_RTK_LoRa::getDataFlush(void)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_DATA_FLUSH, _sendData, 1);
  if(_sendData[0]){
    return true;
  }
  return false;
}

uint8_t DFRobot_RTK_LoRa::getQuality(void)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_GPS_STATE, _sendData, 1);
  return _sendData[0];
}

uint16_t DFRobot_RTK_LoRa::getSiteID(void)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_DIFID_H, _sendData, 2);
  return (uint16_t)_sendData[0] << 8 | _sendData[1];
}

double DFRobot_RTK_LoRa::getDifTime(void)
{
  double time;
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_DIF_Z, _sendData, 2);
  time = (double)_sendData[0] + (double)_sendData[1]/100.0;
  return time;
}

void DFRobot_RTK_LoRa::setModule(eModuleMode_t mode)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  if((uint8_t)mode == getModule()){
    return;
  }else{
    delay(50);
    _sendData[0] = mode;
    writeReg(REG_OPERATION, _sendData, 1);
    if(mode == module_4g){
      delay(3000);
    }else{
      delay(1000);
    }
  }
}

void DFRobot_RTK_LoRa::setModuleBaud(eModuleBaud_t baud)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  _sendData[0] = (uint8_t)baud;
  writeReg(REG_UNO_BAUD, _sendData, 1);
}


void DFRobot_RTK_LoRa::setLoraBaud(eModuleBaud_t baud)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  _sendData[0] = (uint8_t)baud;
  writeReg(REG_LORA_BAUD, _sendData, 1);
}

uint32_t DFRobot_RTK_LoRa::getModuleBaud(void)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_UNO_BAUD, _sendData, 1);
  return baudMatch((eModuleBaud_t)_sendData[0]);
}

uint32_t DFRobot_RTK_LoRa::getLoraBaud(void)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  readReg(REG_LORA_BAUD, _sendData, 1);
  return baudMatch((eModuleBaud_t)_sendData[0]);
}


uint32_t DFRobot_RTK_LoRa::baudMatch(eModuleBaud_t baud)
{
  if(baud == baud_2400){
    return 2400;
  }else if(baud == baud_4800){
    return 4800;
  }else if(baud == baud_9600){
    return 9600;
  }else if(baud == baud_14400){
    return 14400;
  }else if(baud == baud_19200){
    return 19200;
  }else if(baud == baud_38400){
    return 38400;
  }else if(baud == baud_56000){
    return 56000;
  }else if(baud == baud_57600){
    return 57600;
  }else if(baud == baud_115200){
    return 115200;
  }else if(baud == baud_256000){
    return 256000;
  }else if(baud == baud_512000){
    return 512000;
  }else if(baud == baud_921600){
    return 921600;
  }else{
    return 115200;
  }
}

char * DFRobot_RTK_LoRa::transmitAT(const char* cmd)
{
  uint8_t _sendData[255] = {0};
  uint8_t len = strlen(cmd);
  uint8_t templen[10] = {0};
  uint8_t oldlen = 0;
  if(strlen(cmd) > 255){
    return (char*)("AT cmd to long");
  }
  memcpy(_sendData ,cmd, len);
  while(len){
    if(len > 32){
      writeReg(REG_T_AT_LEN, templen, 1);
      writeReg(REG_TRANS_AT, _sendData, 32);
      len -= 32;
      templen[0] += 32;
    }else{
      writeReg(REG_T_AT_LEN, templen, 1);
      writeReg(REG_TRANS_AT, _sendData, len);
      len = 0;
    }
  }
  _sendData[0] = 0xaa;
  writeReg(REG_TRANMIT, _sendData, 1);
  for(uint8_t i = 0; i < 10; i++){
    delay(150);
    readReg(REG_R_AT_LEN, _sendData, 1);
    len = _sendData[0];
    if(len != 0){
      break;
    }
    if(i >= 9) return (char*)("timer out");
  }
  if(len > 120){
    return (char*)("return AT cmd to long");
  }
  while(len){
    if(len > 32){
      writeReg(REG_R_AT_LEN, &oldlen, 1);
      readReg(REG_RECV_AT, (uint8_t *)(__sourceData.cmd+oldlen), 32);
      len -= 32;
      oldlen += 32;
    }else{
      writeReg(REG_R_AT_LEN, &oldlen, 1);
      readReg(REG_RECV_AT, (uint8_t *)(__sourceData.cmd+oldlen), len);
      len = 0;
    }
  }
  return (char *)__sourceData.cmd;
}

eModuleMode_t DFRobot_RTK_LoRa::getModule(void)
{
  uint8_t _sendData[TEMP_LEN] = {0};
  delay(10);
  readReg(REG_OPERATION, _sendData, 1);
  return (eModuleMode_t)_sendData[0];
}


char * DFRobot_RTK_LoRa::getGnssMessage(eGnssData_t mode)
{
  uint8_t len = 0;
  uint8_t i = 0;
  memset(__sourceData.gga, 0, sizeof(__sourceData.gga));
  memset(__sourceData.rmc, 0, sizeof(__sourceData.rmc));
  memset(__sourceData.gll, 0, sizeof(__sourceData.gll));
  memset(__sourceData.vtg, 0, sizeof(__sourceData.vtg));

  if(mode == gnGGA){
    readReg(REG_GGA_LEN, &len, 1);
    if(len < sizeof(__sourceData.gga)){
      if(uartI2CFlag == UART_FLAG) {
        readReg(REG_GGA_ALL, (uint8_t *)(__sourceData.gga), len);
      }else{
        while(len){
          if(len > 32){
            writeReg(REG_GGA_LEN, &i, 1);
            readReg(REG_GGA_ALL, (uint8_t *)(__sourceData.gga+i), 32);
            len -= 32;
            i += 32;
          }else{
            writeReg(REG_GGA_LEN, &i, 1);
            readReg(REG_GGA_ALL, (uint8_t *)(__sourceData.gga+i), len);
            len = 0;
          }
        }
      }
    }
    return __sourceData.gga;
  }else if (mode == gnRMC){
    readReg(REG_RMC_LEN, &len, 1);
    if(len < sizeof(__sourceData.rmc)){
      if(uartI2CFlag == UART_FLAG) {
        readReg(REG_RMC_ALL, (uint8_t *)(__sourceData.rmc), len);
      }else{
        while(len){
          if(len > 32){
            writeReg(REG_RMC_LEN, &i, 1);
            readReg(REG_RMC_ALL, (uint8_t *)(__sourceData.rmc+i), 32);
            len -= 32;
            i += 32;
          }else{
            writeReg(REG_RMC_LEN, &i, 1);
            readReg(REG_RMC_ALL, (uint8_t *)(__sourceData.rmc+i), len);
            len = 0;
          }
        }
      }
    }
    return __sourceData.rmc;
  }else if (mode == gnGLL){
    readReg(REG_GLL_LEN, &len, 1);
    if(len < sizeof(__sourceData.gll)){
      if(uartI2CFlag == UART_FLAG) {
        readReg(REG_GLL_ALL, (uint8_t *)(__sourceData.gll), len);
      }else{
        while(len){
          if(len > 32){
            writeReg(REG_GLL_LEN, &i, 1);
            readReg(REG_GLL_ALL, (uint8_t *)(__sourceData.gll+i), 32);
            len -= 32;
            i += 32;
          }else{
            writeReg(REG_GLL_LEN, &i, 1);
            readReg(REG_GLL_ALL, (uint8_t *)(__sourceData.gll+i), len);
            len = 0;
          }
        }
      }
    }
    return __sourceData.gll;
  }else{    //(mode == gnVTG){
    readReg(REG_VTG_LEN, &len, 1);
    if(len < sizeof(__sourceData.vtg)){
      if(uartI2CFlag == UART_FLAG) {
        readReg(REG_VTG_ALL, (uint8_t *)(__sourceData.vtg), len);
      }else{
        while(len){
          if(len > 32){
            writeReg(REG_VTG_LEN, &i, 1);
            readReg(REG_VTG_ALL, (uint8_t *)(__sourceData.vtg+i), 32);
            len -= 32;
            i += 32;
          }else{
            writeReg(REG_VTG_LEN, &i, 1);
            readReg(REG_VTG_ALL, (uint8_t *)(__sourceData.vtg+i), len);
            len = 0;
          }
        }
      }
    }
    return __sourceData.vtg;
  }
}

uint16_t DFRobot_RTK_LoRa::getGnssLen(void)
{
  uint8_t count = 0;
  uint8_t _sendData[TEMP_LEN] = {0xAA};
  // enter all data mode
  writeReg(REG_ALL_MODE, _sendData, 1);
  while(1){
    readReg(REG_ALL_MODE, _sendData, 1);
    delay(100);
    if(_sendData[0] == 0x55){
      break;
    }
    if(count++ > 20){
      return 0;
    }
  }
  readReg(REG_ALL_LEN_H, _sendData, 2);
  return (uint16_t)_sendData[0] << 8 | _sendData[1];
}

void DFRobot_RTK_LoRa::getAllGnss(void)
{
  uint8_t _sendData[300] = {0};
  uint16_t len = getGnssLen();
  uint16_t old_len = len;
  
  uint16_t i = 0;
  if(uartI2CFlag == UART_FLAG) {
    uint8_t templen = len / 250;
    if(len % 250 != 0){
      templen += 1;
    }
    for(uint16_t i = 0; i < templen; i++){
      if(i == (uint16_t)(templen - 1)){
        readReg(REG_ALL, _sendData, len%250);
        if(old_len < MAX_LEN){
          if(callback){
            callback((char *)_sendData, (uint8_t)(len%250));
          }
        }
        return;
      }else{
        readReg(REG_ALL, _sendData, 250);
        if(old_len < MAX_LEN){
          if(callback){
            callback((char *)_sendData, (uint8_t)250);
          }
        }
      }
    }
  }else{
    while(len){
      if(len > 32){
        _sendData[0] = i>>8;
        _sendData[1] = i;
        writeReg(REG_ALL_LEN_H, _sendData, 2);
        readReg(REG_ALL, _sendData, (uint8_t)32);
        len -= 32;
        i += 32;
        if(old_len < MAX_LEN){
          if(callback){
            callback((char *)_sendData, (uint8_t)32);
          }
        }
      }else{
        _sendData[0] = i>>8;
        _sendData[1] = i;
        writeReg(REG_ALL_LEN_H, _sendData, 2);
        readReg(REG_ALL, _sendData, (uint8_t)len);
        if(old_len < MAX_LEN){
          if(callback){
            callback((char *)_sendData, (uint8_t)len);
          }
        }
        len = 0;
      }
    }
  }
  _sendData[0] = 3;
  writeReg(REG_ALL_MODE, _sendData, 1);
}

void DFRobot_RTK_LoRa::setCallback(void (*call)(char *, uint8_t))
{
  this->callback = call;
}

DFRobot_RTK_LoRa_I2C::DFRobot_RTK_LoRa_I2C(TwoWire *pWire, uint8_t addr)
{
  _pWire = pWire;
  this->_I2C_addr = addr;
  uartI2CFlag = I2C_FLAG;
}

bool DFRobot_RTK_LoRa_I2C::begin()
{
  _pWire->begin();
  _pWire->setClock(100000);
  _pWire->beginTransmission(_I2C_addr);
  if(_pWire->endTransmission() == 0){
    return true;
  }else{
    return false;
  }
}

void DFRobot_RTK_LoRa_I2C::writeReg(uint8_t reg, uint8_t *data, uint8_t len)
{
  _pWire->beginTransmission(this->_I2C_addr);
  _pWire->write(reg);
  for(uint8_t i = 0; i < len; i++){
    _pWire->write(data[i]);
  }
  _pWire->endTransmission();
}

int16_t DFRobot_RTK_LoRa_I2C::readReg(uint8_t reg, uint8_t *data, uint8_t len)
{
  int i = 0;
  uint8_t length = len;
  while(length)
  {
    if(length > 32){
      _pWire->beginTransmission(this->_I2C_addr);
      _pWire->write(reg);
      if(_pWire->endTransmission() != 0){
        return -1;
      }
      _pWire->requestFrom((uint8_t)this->_I2C_addr,(uint8_t)32);
      while (_pWire->available()){
        //if(i < len-1){
          data[i++]=_pWire->read();
        //}
      }
      length -= 32;
    }else{
      _pWire->beginTransmission(this->_I2C_addr);
      _pWire->write(reg);
      if(_pWire->endTransmission() != 0){
        return -1;
      }
      _pWire->requestFrom((uint8_t)this->_I2C_addr,(uint8_t)length);
      while (_pWire->available()){
        //if(i < len-1){
          data[i++]=_pWire->read();
        //}
      }
      length = 0;
    }
  }
  return 0;
}


#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  DFRobot_RTK_LoRa_UART::DFRobot_RTK_LoRa_UART(SoftwareSerial *sSerial, uint32_t Baud)
  {
    this->_serial = sSerial;
    this->_baud = Baud;
    uartI2CFlag = UART_FLAG;
    _serial->begin(this->_baud);
  }
#else
  DFRobot_RTK_LoRa_UART::DFRobot_RTK_LoRa_UART(HardwareSerial *hSerial, uint32_t Baud ,uint8_t txpin, uint8_t rxpin)
  {
    this->_serial = hSerial;
    this->_baud = Baud;
    uartI2CFlag = UART_FLAG;
    this->_txpin = txpin;
    this->_rxpin = rxpin;
  }
#endif

bool DFRobot_RTK_LoRa_UART::begin()
{
  uint8_t _sendData[TEMP_LEN] = {0};
  #ifdef ESP32
    _serial->begin(this->_baud, SERIAL_8N1, _txpin, _rxpin);
    delay(100);
  #elif defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    // nothing use software
  #else
    _serial->begin(this->_baud);  // M0 cannot create a begin in a construct
  #endif
  this->readReg (REG_I2C_ID, _sendData, 1);
  if(_sendData[0] == DEVICE_ADDR){
    return true;
  }else{
    return false;
  }
}

void DFRobot_RTK_LoRa_UART::writeReg(uint8_t reg, uint8_t *data, uint8_t len)
{
  _serial->write(0x55);
  _serial->write(reg | 0x80);
  for(uint8_t i = 0; i < len; i++){
    _serial->write(data[i]);
  }
  _serial->write(0xAA);
  delay(5);
}

int16_t DFRobot_RTK_LoRa_UART::readReg(uint8_t reg, uint8_t *data, uint8_t len)
{
  uint8_t i = 0;
  _serial->write(0x55);
  _serial->write((uint8_t)reg & 0x7F);
  _serial->write(len);
  _serial->write(0xAA);
  uint32_t nowtime = millis();
  while(millis() - nowtime < TIME_OUT){
    while(_serial->available() > 0){
      data[i++] = _serial->read();
      if(i == len) { _serial->flush(); return 0; }
    }
  }
  _serial->flush();
  return 0;
}
