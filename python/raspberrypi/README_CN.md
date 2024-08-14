# DFRobot_RTK_LoRa
- [English Version](./README.md)

这款RTK高精度定位套件采用基站、移动站搭配的方式，实现空旷地带一定范围内厘米级定位，通过将基站获取到的校准数据经过Lora模块以RTCM格式透明传输至移动站模块，移动站模块通过校准运算实现厘米级定位输出。套件由两个Lora模块，一个移动站模块，一个基站模块，两个支持L1+L5频段的GNSS天线组成。
移动站采用移远的LC29HDA模块，该模块是一款双频段、多星座的GNSS模块，支持同时接收GNSS，GPS, GLONASS, Galileo, BDS and QZSS卫星，同时跟踪GPS L1 C/A, GLONASS L1, Galileo E1, BDS B1I, QZSS L1 C/A, GPS L5, GalileoE5a, BDS B2a and QZSS L5频带，能快速获取经纬度、时间、高度等数据。可见的卫星数相比于普通的GPS大大增加，同时结合基站模块，接收差分数据后，在空旷地带能实现厘米级别定位。
基站采用移远的LC29HBS模块，该是一款双频段、多星座GNSS模块，支持同时接收GNSSGPS, GLONASS, Galileo, BDS and QZSS卫星，同时跟踪GPS L1 C/A, GLONASS L1, Galileo E1, BDS B1I, QZSS L1 C/A, GPS L5, GalileoE5a, BDS B2a and QZSS L5频带，提供了快速和准确的采集，提供校正数据。
套件移动端支持I2C/UART输出，可兼容Arduino、ESP32、树莓派等主控设备。作为入门级的模块，使用简单，没有复杂的接线和数据链的搭建，可用于RTK入门学习，定位循迹小车，物品追踪，高精度控制等。

![正反面svg效果图](../../resources/images/xxx.jpg)


## 产品链接(https://www.dfrobot.com.cn)

    SKU：TEL0171-LoRa

## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)

## 概述

提供一个python库给模块，此库具有以下功能：
  获取 gnss 全部数据
  获取 gnss 解析后的数据

## 库安装
1. 下载库至树莓派，要使用这个库，首先要将库下载到Raspberry Pi，命令下载方法如下:

```
sudo git clone https://github.com/DFRobot/DFRobot_RTK_LoRa
```

1. 打开并运行例程，要执行一个例程demo_x.py，请在命令行中输入python demo_x.py。例如，要执行 face_matching.py例程，你需要输入:

```
python  config_param.py
或 
python2 get_all_gnss.py
或 
python3 get_gnss.py
```

## 方法

```python

  def begin(self):
    '''!
      @brief 初始化传感器 
      @return True or False
    '''
    
  def get_data_flush(self):
    '''!
      @brief 获取数据是否刷新
      @return True or False
    '''
    
  def get_date(self):
    '''!
      @brief 获取年月日等日期
      @return 表示返回的年月日
    '''

  def get_utc(self):
    '''!
      @brief 获取utc 标准时间
      @return 表示返回的时分秒
    '''
    
  def get_lat(self):
    '''!
      @brief 获取纬度 
      @return 表示返回的经纬度
    '''
    
  def get_lon(self):
    '''!
      @brief 获取经度 
      @return 表示返回的经度
    '''
    
  def get_num_sta_used(self):
    '''!
      @brief 获取使用的卫星数
      @return uint8_t 类型，表示使用的卫星数
    '''

  def get_alt(self):
    '''!
      @brief 获取大地的高度
      @return double type, 表示大地的高度
    '''
    
  def get_sep(self):
    '''!
      @brief 获取相对水平面的高度
      @return Float data(unit: degree)
    '''
    
  def get_hdop(self):
    '''!
      @brief 获取水平精度因子，表示水平定位精度
      @return 定位精度
    '''

  def get_quality(self):
    '''!
      @brief 获取消息的质量
      @return message Quality
    '''

  def get_site_id(self):
    '''!
      @brief  差分gps数据的站点id，通常用于差分gps定位
      @return site id
    '''
  
  def get_dif_time(self):
    '''!
      @brief 最后一次接收差分信号的秒数
      @return differential time
    '''

  def get_module(self):
    '''!
      @brief 获取模块运行的模式
      @return mode
    '''
      
  def set_module(self, mode):
    '''!
      @brief 设置模块运行的模式
      @param mode 4G or lora
    '''
      
  def set_module_baud(self, baud):
    '''!
      @brief 设置模块运行的波特率
      @param baud rate
    '''

  def get_moudle_baud(self):
    '''!
      @brief 获取模块的波特率
      @return baud
    '''
     
  def get_lora_baud(self):
    '''!
      @brief 获取 接收lora的波特率
      @return baud
    '''
    
  def get_gnss_message(self, mode):
    '''!
     @brief 获取不同类型的gnss数据
     @param 想要的数据类型 gga rcm ggl vtg
     @return char* 
    '''
    
  def get_all_gnss(self):
    '''!
      @brief 获取所有的gnss数据
      @return gnss all data
    '''
```

## 兼容性

* RaspberryPi Version

| Board        | 正常运行  | 运行失败   | 未测试    | 备注
| ------------ | :-------: | :--------: | :------: | :-----: |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python版本

| Python  | 正常运行  | 运行失败   | 未测试    | 备注
| ------- | :-------: | :--------: | :------: | :-----: |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |


## 历史

- 2024/08/14 - V1.0.0 版本

## 创作者

Written by ZhixinLiu(zhixin.liu@dfrobot.com), 2024. (Welcome to our website)