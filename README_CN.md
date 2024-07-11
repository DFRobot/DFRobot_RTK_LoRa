# DFRobot_RTK_LoRa
- [English Version](./README.md)
这款RTK高精度定位套件采用基站、移动站搭配的方式，实现空旷地带一定范围内厘米级定位，通过将基站获取到的校准数据经过Lora模块以RTCM格式透明传输至移动站模块，移动站模块通过校准运算实现厘米级定位输出。套件由两个Lora模块，一个移动站模块，一个基站模块，两个支持L1+L5频段的GNSS天线组成。
移动站采用移远的LC29HDA模块，该模块是一款双频段、多星座的GNSS模块，支持同时接收GNSS，GPS, GLONASS, Galileo, BDS and QZSS卫星，同时跟踪GPS L1 C/A, GLONASS L1, Galileo E1, BDS B1I, QZSS L1 C/A, GPS L5, GalileoE5a, BDS B2a and QZSS L5频带，能快速获取经纬度、时间、高度等数据。可见的卫星数相比于普通的GPS大大增加，同时结合基站模块，接收差分数据后，在空旷地带能实现厘米级别定位。
基站采用移远的LC29HBS模块，该是一款双频段、多星座GNSS模块，支持同时接收GNSSGPS, GLONASS, Galileo, BDS and QZSS卫星，同时跟踪GPS L1 C/A, GLONASS L1, Galileo E1, BDS B1I, QZSS L1 C/A, GPS L5, GalileoE5a, BDS B2a and QZSS L5频带，提供了快速和准确的采集，提供校正数据。
套件移动端支持I2C/UART输出，可兼容Arduino、ESP32、树莓派等主控设备。作为入门级的模块，使用简单，没有复杂的接线和数据链的搭建，可用于RTK入门学习，定位循迹小车，物品追踪，高精度控制等。

![正反面svg效果图](/resources/images/xxx.jpg) 

## 产品链接(https://www.dfrobot.com.cn)

    SKU：TEL0171-LoRa

## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性y)
* [历史](#历史)
* [创作者](#创作者)

## 概述

提供一个Arduino库给RTK模块，此库具有以下功能：
  获取 gnss的数据
  获取 gnss的原始数据
  配置 模块

## 库安装
这里提供两种使用本库的方法：<br>
1.打开Arduino IDE,在状态栏中的Tools--->Manager Libraries 搜索"DFRobot_RTK_LoRa"并安装本库.<br>
2.首先下载库文件,将其粘贴到\Arduino\libraries目录中,然后打开examples文件夹并在该文件夹中运行演示.<br>

## 方法

```C++
/**
 * @fn getUTC
 * @brief 获取utc 标准时间
 * @return sTim_t 类型，表示返回的时分秒
 * @retval sTim_t.hour 时
 * @retval sTim_t.minute 分
 * @retval sTim_t.second 秒
 */
  sTim_t getUTC(void);

/**
 * @fn getDate
 * @brief 获取年月日等日期
 * @return sTim_t 类型，表示返回的年月日
 * @retval sTim_t.year 年
 * @retval sTim_t.month 月
 * @retval sTim_t.day 日
 */
  sTim_t getDate(void);

/**
 * @fn getLat
 * @brief 获取纬度
 * @return sLonLat_t 类型，表示返回的经纬度
 * @retval sLonLat_t.latDD   纬度 度（0-90）
 * @retval sLonLat_t.latMM   纬度 分后0-2位小数
 * @retval sLonLat_t.latMMMMM 纬度 分后2-7位小数
 * @retval sLonLat_t.latitude 包含7位小数的纬度值
 * @retval sLonLat_t.latDirection 纬度的方向
 */
  sLonLat_t getLat(void);

/**
 * @fn getLon
 * @brief 获取经度
 * @return sLonLat_t 类型，表示返回的经度
 * @retval sLonLat_t.lonDDD  经度 度（0-90）
 * @retval sLonLat_t.lonMM   经度 分后0-2位小数
 * @retval sLonLat_t.lonMMMMM 经度 分后2-7位小数
 * @retval sLonLat_t.lonitude 包含7位小数的经度值
 * @retval sLonLat_t.lonDirection 经度的方向
 */
  sLonLat_t getLon(void);

/**
 * @fn getDataFlush
 * @brief 获取数据是否刷新
 * @return bool
 * @retval True 数据刷新
 * @retval false 数据还未刷新
 */
  bool getDataFlush(void);

/**
 * @fn getNumSatUsed
 * @brief 获取使用的卫星数
 * @return uint8_t 类型，表示使用的卫星数
 */
  uint8_t getNumSatUsed(void);

/**
 * @fn getAlt
 * @brief 获取大地的高度
 * @return double 类型，表示大地的高度
 */
  double getAlt(void);

/**
 * @fn getSep
 * @brief 获取相对水平面的高度
 * @return double 
 */
  double getSep(void);

/**
 * @fn getHdop
 * @brief 获取水平精度因子，表示水平定位精度
 * @return double
 */
  double getHdop(void);

/**
 * @fn getQuality
 * @brief 获取消息的质量
 * @return uint8_t 
 */
  uint8_t getQuality(void);

/**
 * @fn getSiteID
 * @brief 差分gps数据的站点id，通常用于差分gps定位
 * @return uint16_t
 */
  uint16_t getSiteID(void);

/**
 * @fn getDifTime
 * @brief 最后一次接收差分信号的秒数
 * @return double 
 */
  double getDifTime(void);

/**
 * @fn disablePower
 * @brief 失能gnss的电源
 * @return null
 */
void disablePower(void);

/**
 * @fn setModule
 * @brief 设置模块运行的模式
 * @param mode 4G or lora
 */
  void setModule(eModuleMode_t mode);

/**
 * @fn getModule
 * @brief 获取模块运行的模式
 * @return eModuleMode_t 
 */
  eModuleMode_t getModule(void);

/**
 * @fn transmitAT
 * @brief 透传给gnss的命令接口
 * @return char * 返回gnss 返回的数据
 */
  char * transmitAT(const char* cmd);

/**
 * @fn getGnssMessage
 * @brief 获取不同类型的gnss数据
 * @param mode 想要的数据类型
 * @return char* 
 */
  char * getGnssMessage(eGnssData_t mode);

/**
 * @fn getAllGnss
 * @brief 获取所有的gnss数据，使用回调接收
 */
  void getAllGnss(void);

/**
 * @fn setModuleBaud
 * @brief 设置模块运行的波特率
 * @param baud eModuleBaud_t 枚举类型
 */
  void setModuleBaud(eModuleBaud_t baud);

/**
 * @fn setLoraBaud
 * @brief 设置接收lora模块的波特率
 * @param baud eModuleBaud_t 枚举类型
 */
  void setLoraBaud(eModuleBaud_t baud);

/**
 * @fn getModuleBaud
 * @brief 获取模块的波特率
 * @return uint32_t 模块通信的波特率
 */
  uint32_t getModuleBaud(void);

/**
 * @fn getModuleBaud
 * @brief 获取 接收lora的波特率
 * @return uint32_t lora通信的波特率
 */
  uint32_t getLoraBaud(void);

/**
 * @fn setCallback
 * @brief 设置回调函数类型
 * @param  * call 函数名
 * @return null
 */
  void setCallback(void (*call)(char *, uint8_t));
```

## 兼容性

MCU                | Work Well    |   Work Wrong    | Untested    | Remarks
------------------ | :----------: | :-------------: | :---------: | :----:
Arduino Uno        |      √       |                 |             |
Arduino MEGA2560   |      √       |                 |             |
Arduino Leonardo   |      √       |                 |             |
FireBeetle-ESP8266 |      √       |                 |             |
FireBeetle-ESP32   |      √       |                 |             |
FireBeetle-M0      |      √       |                 |             |
Micro:bit          |      √       | nonsupport uart |             |


## 历史
- 2023/3/27 - V0.1.0 版本

## 创作者

Written by ZhixinLiu(zhixin.liu@dfrobot.com), 2024. (Welcome to our website)