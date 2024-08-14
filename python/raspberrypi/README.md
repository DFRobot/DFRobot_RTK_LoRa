# DFRobot_RTK_LoRa
- [中文版](./README_CN.md)
This RTK high-precision positioning kit uses the combination of base station and mobile station to achieve centimeter-level positioning within a certain range of open areas. The calibration data obtained by the base station is transparently transmitted to the mobile station module in RTCM format through the Lora module, and the mobile station module realizes centimeter-level positioning output through calibration operations. The kit consists of two Lora modules, a mobile station module, a base station module, and two GNSS antennas supporting the L1+L5 band.
The mobile station adopts the remote LC29HDA module, which is a dual-band, multi-constellation GNSS module, supporting the simultaneous reception of GNSS, GPS, GLONASS, Galileo, BDS and QZSS satellites. Also tracking GPS L1 C/A, GLONASS L1, Galileo E1, BDS B1I, QZSS L1 C/A, GPS L5, GalileoE5a, BDS B2a and QZSS L5 bands, Can quickly obtain longitude and latitude, time, altitude and other data. The number of visible satellites is greatly increased compared with ordinary GPS, and combined with the base station module, after receiving differential data, centimeter-level positioning can be achieved in open areas.
The base station adopts the remote LC29HBS module, which is a dual-band, multi-constellation GNSS module, supporting the simultaneous reception of GNSSGPS, GLONASS, Galileo, BDS and QZSS satellites. Simultaneous tracking of GPS L1 C/A, GLONASS L1, Galileo E1, BDS B1I, QZSS L1 C/A, GPS L5, GalileoE5a, BDS B2a and QZSS L5 bands provides fast and accurate acquisition, Provide correction data.
The mobile end of the kit supports I2C/UART output, and is compatible with Arduino, ESP32, Raspberry PI and other main control devices. As an entry-level module, it is simple to use, without complex wiring and data link construction, and can be used for RTK entry learning, positioning tracking car, item tracking, high-precision control, etc.

![效果图](../../resources/images/xxx.jpg)

## Product Link(https://www.dfrobot.com)

    SKU：TEL0171-LoRa

## Table of Contents

* [Summary](#Summary)
* [Installation](#Installation)
* [Methods](#Methods)
* [Compatibility](#Compatibility)
* [History](#History)
* [Credits](#Credits)

## Summary

Provides an Arduino library that has the following features:
  Face recognition routine

## Installation
1. Download the library to Raspberry Pi first before use.

```
sudo git clone https://github.com/DFRobot/DFRobot_RTK_LoRa
```

1. Open and run the routine. To execute a routine demo_x.py, enter python demo_x.py in the command line. For example, to execute the routine get_gnss.py, you need to enter:

```
python config_param.py
or 
python2 get_all_gnss.py
or 
python3 get_gnss.py
```

## Methods

```python
  def begin(self):
    '''!
      @brief Init sensor 
      @return True or False
    '''
    
  def get_data_flush(self):
    '''!
      @brief get_data_flush 
      @return True or False
    '''
    
  def get_date(self):
    '''!
      @brief Get date information, year, month, day 
      @return struct_utc_tim type, represents the returned year, month, day
    '''
    
  def get_utc(self):
    '''!
      @brief Get time information, hour, minute second 
      @return struct_utc_tim type, represents the returned hour, minute, second 
    '''
    
  def get_lat(self):
    '''!
      @brief Get latitude 
      @return struct_lat_lon type, represents the returned latitude 
    '''

  def get_lon(self):
    '''!
      @brief Get longitude 
      @return struct_lat_lon type, represents the returned longitude 
    '''
    
  def get_num_sta_used(self):
    '''!
      @brief Get the number of the used satellite used
      @return uint8_t type, represents the number of the used satellite
    '''

  def get_alt(self):
    '''!
      @brief Altitude information
      @return double type, represents altitude 
    '''
    
  def get_sep(self):
    '''!
      @brief At the height of geoid
      @return Float data(unit: degree)
    '''
    
  def get_hdop(self):
    '''!
      @brief Indicates the horizontal accuracy of positioning
      @return hdop
    '''

  def get_quality(self):
    '''!
      @brief get message Quality 
      @return message Quality
    '''
    
  def get_site_id(self):
    '''!
      @brief get site id
      @return site id  
    '''
  
  def get_dif_time(self):
    '''!
      @brief The number of seconds in which a differential signal was last received
      @return differential time
    '''

  def get_module(self):
    '''!
      @brief Get the Module run mode
      @return mode
    '''
      
  def set_module(self, mode):
    '''!
      @brief Set the Module
      @param mode 4G or lora
    '''
      
  def set_module_baud(self, baud):
    '''!
      @brief Set the Module Baud rate
      @param baud rate
    '''
    
  def get_moudle_baud(self):
    '''!
      @brief Get the Module baud
      @return baud
    '''
    
  def get_gnss_message(self, mode):
    '''!
      @brief Get different types of gps data
      @param mode
      @return char* 
    '''
    

  def get_all_gnss(self):
    '''!
      @brief Get all GNSS data
      @return gnss all data
    '''
```

## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |     √     |            |          |         |
| RaspberryPi4 |           |            |    √     |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |


## History

- 2024/08/14 - Version 1.0.0 released.

## Credits

Written by ZhixinLiu(zhixin.liu@dfrobot.com), 2024. (Welcome to our website)
