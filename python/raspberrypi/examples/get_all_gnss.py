# -*- coding: utf-8 -*
'''!
  @file get_gnss.py
  @brief Reading all gnss data
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     [ZhixinLiu](zhixin.liu@dfrobot.com)
  @version    V1.0
  @date       2024-08-12
  @url https://github.com/DFRobot/DFRobot_RTK_LoRa
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")
import time
from DFRobot_RTK_LoRa import *

'''
  Select to use i2c or UART
  I2C_MODE
  UART_MODE
'''
ctype = I2C_MODE

if ctype == I2C_MODE:
  I2C_1 = 0x01
  rtk = DFRobot_RTK_LoRa_I2C (I2C_1, DEVICE_ADDR)
elif ctype == UART_MODE:
  rtk = DFRobot_RTK_LoRa_UART(115200)

def setup():
  while (rtk.begin() == False):
    print("Sensor initialize failed!!")
    time.sleep(1)
  print("Sensor initialize success!!")
  rtk.set_module(MODULE_LORA)
def loop():
  rslt = rtk.get_all_gnss()
  if len(rslt) != 0:
    print(rslt)

if __name__ == "__main__":
  setup()
  while True:
    loop()
