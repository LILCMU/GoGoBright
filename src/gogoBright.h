#ifndef __GoGoBrightLib__
#define __GoGoBrightLib__

#include <Arduino.h>

#define GOGOBRIGHT_I2C_ADDRESS 0x42
#define GOGOBRIGHT_ID_1 0x60
#define GOGOBRIGHT_ID_2 0x61

//* ////////////////////////////////////
//* Register map
#define REG_GOGOBRIGHT_ID 0
#define REG_INPUT_PORT 2
#define REG_SERVO_ACTIVE 9
#define REG_SERVO_ANGLE 10
#define REG_MOTOR_ACTIVE 14
#define REG_MOTOR_STATUS 15
#define REG_MOTOR_DIR 16
#define REG_MOTOR_PWR 17

//* ///////////////////////////////////
//* Commands I2C
//* servo
#define CMD_SERVO_ACTIVE 91
#define CMD_SERVO_SETH 87
#define CMD_SERVO_CCW 88
#define CMD_SERVO_CW 89
//* motor
#define CMD_MOTOR_ACTIVE 90
#define CMD_MOTOR_ON 49
#define CMD_MOTOR_OFF 51
#define CMD_MOTOR_CCW 52
#define CMD_MOTOR_CW 53
#define CMD_MOTOR_RD 54
#define CMD_MOTOR_PWR 59
//* i2c
#define CMD_I2C_WRITE 107
#define CMD_I2C_READ 108

class GoGoBrightLib
{
private:
  int8_t _i2cAddr;

  //* Raw I2C Commands to control GoGoBright ref: SparkFun_APDS9960 Library
  bool wireWriteByte(uint8_t val);
  bool wireWriteDataByte(uint8_t reg, uint8_t val);
  bool wireWriteDataBlock(uint8_t reg, uint8_t *val, unsigned int len);
  bool wireReadDataByte(uint8_t reg, uint8_t &val);
  int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len);

  bool wireWriteDataByteToAddr(uint8_t cmd, uint8_t addr, uint8_t reg, uint8_t val);
  bool wireReadDataByteFromAddr(uint8_t cmd, uint8_t addr, uint8_t reg, uint8_t &val);

public:
  GoGoBrightLib(void);
  ~GoGoBrightLib(void);

  bool begin(int8_t i2cAddr = GOGOBRIGHT_I2C_ADDRESS);

  //* Input port functions
  //? get sensors value from input port 1-4
  int readInput(int port);

  //* Servo functions
  //? set servos to interact with ..
  bool talkToServo(String servo_port);
  //? set servos head to input head_angle
  bool setServoHead(int head_angle);
  //? turn servos clockwise by input angle
  bool turnServoCW(int cw_angle);
  //? turn servos counter-clockwise by input angle
  bool turnServoCCW(int ccw_angle);

  //* Motor functions
  //? set output to interact with ..
  bool talkToOutput(String output_port);
  //? set output power
  bool setOutputPower(int power);
  //? turn outputs on or off
  bool turnOutputON(void);
  bool turnOutputOFF(void);
  //? turn outputs direction
  bool turnOutputThisWay(void);
  bool turnOutputThatWay(void);
  bool toggleOutputWay(void);

  //* I2C onboard functions
  bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t value);
  uint8_t i2cRead(uint8_t addr, uint8_t reg);
};

#endif