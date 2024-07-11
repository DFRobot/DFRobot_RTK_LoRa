 /*!
  * @file  configParam.ino
  * @brief config moudle param
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V0.5.0
  * @date 2024-04-24
  * @url https://github.com/DFRobot/DFRobot_RTK_LoRa
  */

#include "DFRobot_RTK_LoRa.h"

// must use iic config parameter
DFRobot_RTK_LoRa_I2C rtk(&Wire ,DEVICE_ADDR);
void setup()
{
  Serial.begin(115200);
  while(!rtk.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("Device connected !");

  /**
   * | Support Baud | UNO/ESP8266 | Leonardo/Mega2560 | ESP32 |  M0 |
   * | baud_9600    |      √      |         √         |   √   |  √  |
   * | baud_14400   |      √      |         √         |   √   |  √  |
   * | baud_19200   |      √      |         √         |   √   |  √  |
   * | baud_38400   |      √      |         √         |   √   |  √  |
   * | baud_56000   |      √      |         √         |   √   |  √  |
   * | baud_57600   |      √      |         √         |   √   |  √  |
   * | baud_115200  |             |         √         |   √   |  √  |
   * | baud_256000  |             |                   |   √   |  √  |
   * | baud_512000  |             |                   |   √   |  √  |
   * | baud_921600  |             |                   |   √   |  √  |
   */
  rtk.setModuleBaud(baud_115200);

  Serial.print("module mode = ");
  Serial.println(rtk.getModule());

  Serial.print("moudle buad = ");
  Serial.println(rtk.getModuleBaud());
}

void loop()
{
  // Reserved interface, direct communication with gnss firmware, use with the original factory data manual
  // Serial.println(rtk.transmitAT("$PQTMVERNO*58\r\n"));
  delay(2000);
}