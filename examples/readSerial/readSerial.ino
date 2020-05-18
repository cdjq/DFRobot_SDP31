/*!
 * @file readSerial.ino
 * @brief 读取芯片序列号
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

  Serial.begin(9600);
  //芯片初始化
  while (SDP31.begin() != 0) {
    Serial.println("Failed to initialize the chip, please confirm the chip connection");
    delay(1000);
  }
}

void loop() {
  Serial.print("芯片序列号：");
  Serial.println(SDP31.readSerialNumber());
  Serial.print("唯一序列号：");
  Serial.println(SDP31.readUniqueSerialNumber());
  Serial.println("------------------------");
  delay(2000);
}