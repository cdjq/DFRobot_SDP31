/*!
 * @file continuousMeasurement.ino
 * @brief 芯片会连续不断地测量压差数据,在运行十秒后会退出连续测量模式
 * @n Continuous mode:Best used where speed and accuracy are most important.
 * @n Triggered mode:Best used where energy consumption is more important than speed.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2020-05-14
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_SDP31
 */
#include <DFRobot_SDP31.h>
/*! 
 * @brief Construct the function
 * @param pWire IC bus pointer object and construction device, can both pass or not pass parameters, Wire in default.
 * @param address Chip IIC address, two optional addresses 0x22 and 0x21(0x22 in default).
 */
//DFRobot_SDP31 SDP31(&Wire,0x22);
DFRobot_SDP31 SDP31;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //芯片初始化
  while (SDP31.begin() != 0) {
    Serial.println("Failed to initialize the chip, please confirm the chip connection");
    delay(1000);
  }
  //芯片软复位，需要通过软件将芯片复位到已知状态
  SDP31.softReset();
  /*! 
   * @brief 进入连续测量模式
   * @param tempcompen 温度补偿方式
   * @n  MASSFLOW 测量质量流量的温度补偿
   * @n  DIFFPRESSURE 测量压差的温度补偿
   * @param averaging 芯片内部处理数据的方法
   * @n AVGTILLREAD 取平均数的测量模式
   * @n AVGNONE 不取平均数的测量模式
   * @return true：成功进入连续测量模式，false：进入失败
   */
  SDP31.startContinuousMode(DFRobot_SDP31::DIFFPRESSURE,DFRobot_SDP31::AVGTILLREAD);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Temperature: ");
  //获取温度，单位为摄氏度
  Serial.print(SDP31.getTemperature());
  Serial.println(" C");
  Serial.print("Differential Pressure: ");
  //获取压差，单位为pa
  Serial.print(SDP31.getDifferentialPressure());
  Serial.println(" pa");
  delay(500);
  if(millis() > 10000 && millis() < 10500){
    /*! 
     * @brief 退出连续测量模式
     */
    SDP31.stopContinuousMode();
    Serial.println("Exited from the cycle measurement mode, enter the single measurement mode");
  }
 /*
  //以结构体的方式存储数据，然后再进行打印
  DFRobot_SDP31::sDPAndTemp_t data;
  data = SDP31.readTempAndDP();
  Serial.print("Temperature: ");
  Serial.print(data.Temperature);
  Serial.println(" C");
  Serial.print("Differential Pressure: ");
  Serial.print(data.DifferentialPressure);
  Serial.println(" pa");
  delay(500);
  */
  
}