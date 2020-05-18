# DFRobot_SDP31
SDP31是一款具有出色的精度和长期稳定性的微差压压力传感器（测量两个压力之间的差值），采用著名厂商Sensirion的SDP31传感器，<br>
具有测量速度快，无零点漂移，自动校准和温度补偿等特点，I2C接口，压差范围为0-500pa。该传感器是呼吸机的核心传感器之一。<br>

![SVG Figure](https://github.com/ouki-wang/DFRobot_Sensor/raw/master/resources/images/SEN0245svg1.png)

## Product Link（链接到英文商城）
    SEN0334: DFRobot SDP31 Differential Pressure Sensor(+/- 500 Pa)
   
## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

   1.读取压差和温度<br>  
       Continuous mode:Best used where speed and accuracy are most important.<br>  
       Triggered mode:Best used where energy consumption is more important than speed.<br>  
   2.读取芯片信息
## Installation
To use this library, please download the library file first, and paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
  /*! 
   * @brief Construct the function
   * @param pWire IC bus pointer object and construction device, can both pass or not pass parameters, Wire in default.
   * @param address Chip IIC address, two optional addresses 0x22 and 0x21(0x22 in default).
   */
  DFRobot_SDP31(TwoWire *pWire = &Wire, uint8_t address = 0x22);
  
  /**
   * @brief Read the serial number of the chip
   * @return 32-digit serial number 
   */
   uint32_t readSerialNumber();
   
  /**
   * @brief Read the serial  unique number of the chip
   * @return 66-digit serial unique number 
   */
   String readUniqueSerialNumber();
  /** 
   * @brief Initialize the function 
   * @return Return 0 indicates a successful initialization, while other values indicates failure and return to error code.
   */
  int begin();
  
  /**
   * @brief Send command resets via iiC, enter the chip's default mode single-measure mode, 
   * turn off the heater, and clear the alert of the ALERT pin.
   * @return Read the status register to determine whether the command was executed successfully, and returning true indicates success
   */
  void softReset();  
  /*! 
   * @brief 进入连续测量模式
   * @param tempcompen 温度补偿方式
   * @param averaging 芯片内部处理数据的方法
   * @return true：成功进入连续测量模式，false：进入失败
   */
  bool startContinuousMode(eTempCompen_t tempcompen , eAveraging_t averaging);
  
  /*! 
   * @brief 退出连续测量模式
   */
  void stopContinuousMode();
  /*! 
   * @brief 芯片进入休眠模式，将停止数据的测量，达到省电的目的
   */
  void enterSleep();
  /*! 
   * @brief 读取温度，压差，质量流量的数据
   * @param tempcompen 温度补偿方式，选择不同则可以测量不同的数据
   * @n MASSFLOW 测量质量流量
   * @n DIFFPRESSURE 测量压差
   * @return 返回读到的数据
   */
  sDPAndTemp_t readTempAndDP(eTempCompen_t tempcompen = NOCOMPEN);
  /*! 
   * @brief 读取压差
   * @return 压差：(pa)
   */
  int16_t getDifferentialPressure();
  /*! 
   * @brief 获取质量流量
   * @return 返回质量流量的数据
   */
  int16_t getMassFlow();
  /*! 
   * @brief 获取温度
   * @return 返回温度数据：单位是摄氏度
   */
  float getTemperature();

```

## Compatibility

MCU           | Work Well    | Work Wrong   | Untested    | Remarks
--------------| :----------: | :----------: | :---------: | -----
Arduino Uno   |      √       |              |             | 
Mega2560      |      √       |              |             | 
Leonardo      |      √       |              |             | 
ESP32         |      √       |              |             | 
micro:bit     |      √       |              |             | 


## History

- data 2020-5-18
- version V0.1


## Credits

Written by fengli(li.feng@dfrobot.com), 2020.5.18 (Welcome to our [website](https://www.dfrobot.com/))





