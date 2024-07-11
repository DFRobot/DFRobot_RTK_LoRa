 /*!
  * @file  getGNSS.ino
  * @brief Get gnss simple data
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V0.5.0
  * @date 2023-04-23
  * @url https://github.com/DFRobot/DFRobot_RTK_LoRa
  */

#include "DFRobot_RTK_LoRa.h"

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
  DFRobot_RTK_LoRa_I2C rtk(&Wire ,DEVICE_ADDR);
#else
/* -----------------------------------------------------------------------------------------------------
 * |  Sensor  | Connect line | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 * |   VCC    |=============>|        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 * |   GND    |=============>|        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 * |   RX     |=============>|     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 * |   TX     |=============>|     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------*/
/* Baud rate cannot be changed */
  #if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    SoftwareSerial mySerial(4, 5);
    DFRobot_RTK_LoRa_UART rtk(&mySerial, 57600);
  #elif defined(ESP32)
    DFRobot_RTK_LoRa_UART rtk(&Serial1, 115200 ,/*rx*/D2 ,/*tx*/D3);
  #else
    DFRobot_RTK_LoRa_UART rtk(&Serial1, 115200);
  #endif
#endif

void setup()
{
  Serial.begin(115200);
  while(!rtk.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("Device connected !");
  rtk.setModule(module_lora);
  while(rtk.getModule() != module_lora){
    Serial.println("Module type is not lora!  please wait!");
    delay(1000);
  }
}

void loop()
{
  bool state = rtk.getDataFlush();
  if(state == true)
  {
    sTim_t utc = rtk.getUTC();
    sTim_t date = rtk.getDate();
    sLonLat_t lat = rtk.getLat();
    sLonLat_t lon = rtk.getLon();
    double high = rtk.getAlt();
    uint8_t starUserd = rtk.getNumSatUsed();
    double hdop = rtk.getHdop();
    double sep = rtk.getSep();
    uint8_t mode = rtk.getQuality();
    uint16_t siteID = rtk.getSiteID();
    double diftime = rtk.getDifTime();
    Serial.println("");
    Serial.print(date.year);
    Serial.print("/");
    Serial.print(date.month);
    Serial.print("/");
    Serial.print(date.date);
    Serial.print("/");
    Serial.print(utc.hour);
    Serial.print(":");
    Serial.print(utc.minute);
    Serial.print(":");
    Serial.print(utc.second);
    Serial.println();
    Serial.println((char)lat.latDirection);
    Serial.println((char)lon.lonDirection);
    // Serial.print("lat DDMM.MMMMM = ");
    // Serial.println(lat.latitude, 5);
    // Serial.print(" lon DDDMM.MMMMM = ");
    // Serial.println(lon.lonitude, 5);
    Serial.print("lat degree = ");
    Serial.println(lat.latitudeDegree,6);
    Serial.print("lon degree = ");
    Serial.println(lon.lonitudeDegree,6);
    Serial.print("star userd = ");
    Serial.println(starUserd);
    Serial.print("alt high = ");
    Serial.println(high);
    Serial.print("sep  = ");
    Serial.println(sep);
    Serial.print("hdop = ");
    Serial.println(hdop);
    Serial.print("message mode  = ");
    Serial.println(mode);
    Serial.print("siteID = ");
    Serial.println(siteID);
    Serial.print("diftime = ");
    Serial.println(diftime);
    Serial.println(rtk.getGnssMessage(gnGGA));
    Serial.println(rtk.getGnssMessage(gnRMC));
    Serial.println(rtk.getGnssMessage(gnGLL));
    Serial.println(rtk.getGnssMessage(gnVTG));
  }
  delay(300);
}
