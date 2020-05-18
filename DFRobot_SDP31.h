/*!
 * @file DFRobot_SDP31.h
 * @brief Define the infrastructure of the DFRobot_SDP31 class
 * @n This is a library used to drive sdp31 sensor
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2020-05-14
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_SDP31
 */
 
#ifndef DFROBOT_SDP31_H
#define DFROBOT_SDP31_H
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>
#include <string.h>

//#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

#define SDP31_UNDEFINED                          (0x0000)  //空命令
#define SDP31_MASS_FLOW_AVG                      (0x3603)  //Mass flow   Average till read 测平均流量
#define SDP31_MASS_FLOW_NONE                     (0x3608)  //Mass flow   None - Update rate 0.5ms   测实时流量
#define SDP31_DIFFERENTIAL_PRESSURE_AVG          (0x3615)  //Differential pressure   Average till read   测平均压差
#define SDP31_DIFFERENTIAL_PRESSURE_NONE         (0x361E)  //Differential pressure   None - Update rate 0.5ms   测实时压差
#define SDP31_STOP_CONTINUOUS_MEASUREMENT        (0x3FF9)  //停止周期测量模式
#define SDP31_MASS_FLOW                          (0x3624)  //单次气流
#define SDP31_DIFFERENTIAL_PRESSURE              (0x362F)  //单次测量气压
#define SDP31_SOFT_RESET                         (0x0006)  //软件复位
#define SDP31_SLEEP_MODE                         (0x3677)  //休眠
#define SDP31_READ_PRODUCT_IDENTIFIER            (0x367C)  // 读序列号


class DFRobot_SDP31
{
public:
  #define ERR_OK             0      //No error
  #define ERR_DATA_BUS      -1      //Data bus error
  #define ERR_IC_VERSION    -2      //Chip version does not match
  /**
    Two measurement modes for the chip
  */
  typedef enum {
    eOneShot,   /**<Single measurement mode>*/
    ePeriodic, /**<Cycle measurement mode>*/
  }eMeasureMode_t;
  /**
    温度补偿的类型
  */
  typedef enum{
    MASSFLOW,/**<测量质量流量的温度补偿>*/
    DIFFPRESSURE,/**<测量压差的温度补偿>*/
    NOCOMPEN,/**<没有温度补偿>*/
  }eTempCompen_t; 
  typedef enum{
    AVGTILLREAD,/**<取平均数的测量模式>*/
    AVGNONE,/**<不取平均数的测量模式>*/
  }eAveraging_t;
  typedef struct{
    float Temperature;/**<温度>*/
    int16_t DifferentialPressure;/**<压差>*/
    float MassFlow;/**<质量流量>*/
  }sDPAndTemp_t;

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
private:
  //计算校验码
  uint8_t checkCrc(uint8_t *data);
  void writeLimitData(uint16_t cmd,uint16_t limitData);
  void writeCommand(uint16_t cmd,size_t size);
  void write(const void* pBuf,size_t size);
  uint8_t readData(void *pBuf, size_t size);
  
private:
  TwoWire *_pWire;
  uint8_t _address;
  eTempCompen_t  _tempcompen;
  eMeasureMode_t _mode;
};   
#endif
