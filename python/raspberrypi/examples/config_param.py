# -*- coding: utf-8 -*
'''!
  @file config_param.py
  @brief Reading all gnss data
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     [ZhixinLiu](zhixin.liu@dfrobot.com)
  @version    V1.0
  @date       2024-08-13
  @url https://github.com/DFRobot/DFRobot_RTK_LoRa
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")
import time
from DFRobot_RTK_LoRa import *

I2C_1 = 0x01
rtk = DFRobot_RTK_LoRa_I2C (I2C_1, DEVICE_ADDR)

def setup():
  while (rtk.begin() == False):
    print("Sensor initialize failed!!")
    time.sleep(1) 
  print("module baud = " + str(rtk.get_moudle_baud()))
  print("module mode = " + str(rtk.get_module()))
  rtk.set_module_baud(BAUD_115200)
  rtk.set_module(MODULE_LORA)
  
def loop():
  exit()

if __name__ == "__main__":
  setup()
  while True:
    loop()
