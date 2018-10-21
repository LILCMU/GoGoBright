#include <Arduino.h>
#include <Wire.h>
#include "gogoBright.h"

gogobright_library::gogobright_library(void)
{
}
gogobright_library::~gogobright_library(void)
{
}

bool gogobright_library::begin(int8_t i2cAddr)
{
    uint8_t id;

    _i2cAddr = i2cAddr;
    Wire.begin();

    // read gogobright ID to confirm i2c connection
    if (!wireReadDataByte(REG_GOGOBRIGHT_ID, id))
    {
        return false;
    }
    if (!(id == GOGOBRIGHT_ID_1 || id == GOGOBRIGHT_ID_2))
    {
        return false;
    }

    return true;
}

int gogobright_library::readInput(int port)
{
    // if (port < 1 || port > 4)
    if (port < 1 || port > 3) //* temporary fix due to hw issues
        return 0;

    uint8_t val_byte;
    int val = 0;

    //* Read high byte register
    if (!wireReadDataByte((REG_INPUT_PORT * port) - 1, val_byte))
    {
        return 0;
    }
    val = (uint16_t)val_byte << 8;

    //* Read low byte register
    if (!wireReadDataByte((REG_INPUT_PORT * port), val_byte))
    {
        return 0;
    }

    return val + val_byte;
}

bool gogobright_library::talkToServo(String servo_port)
{
    uint8_t servoBits = 0;

    if (servo_port.length() < 1 || servo_port.length() > 4)
        return false;

    for (int i = 0; i < 4; i++)
    {
        if (servo_port[i] == '1')
        {
            servoBits |= 1;
        }
        else if (servo_port[i] == '2')
        {
            servoBits |= 2;
        }
        else if (servo_port[i] == '3')
        {
            servoBits |= 4;
        }
        else if (servo_port[i] == '4')
        {
            servoBits |= 8;
        }
        else
        {
            return false;
        }
    }

    if (!wireWriteDataByte(CMD_SERVO_ACTIVE, servoBits))
    {
        return false;
    }

    return true;
}
bool gogobright_library::setServoHead(int head_angle)
{
    if (head_angle < 0 || head_angle > 180)
        return false;

    if (!wireWriteDataByte(CMD_SERVO_SETH, head_angle))
    {
        return false;
    }

    return true;
}
bool gogobright_library::turnServoCW(int cw_angle)
{
    if (cw_angle < 0 || cw_angle > 180)
        return false;

    if (!wireWriteDataByte(CMD_SERVO_CW, cw_angle))
    {
        return false;
    }

    return true;
}
bool gogobright_library::turnServoCCW(int ccw_angle)
{
    if (ccw_angle < 0 || ccw_angle > 180)
        return false;

    if (!wireWriteDataByte(CMD_SERVO_CCW, ccw_angle))
    {
        return false;
    }

    return true;
}

bool gogobright_library::talkToOutput(String output_port)
{
    uint8_t motorBits = 0;
    output_port.toLowerCase();

    if (output_port.length() < 1 || output_port.length() > 4)
        return false;

    for (int i = 0; i < 4; i++)
    {
        if (output_port[i] == 'a')
        {
            motorBits |= 1;
        }
        else if (output_port[i] == 'b')
        {
            motorBits |= 2;
        }
        else if (output_port[i] == 'c')
        {
            motorBits |= 4;
        }
        else if (output_port[i] == 'd')
        {
            motorBits |= 8;
        }
    }

    if (!wireWriteDataByte(CMD_MOTOR_ACTIVE, motorBits))
    {
        return false;
    }

    return true;
}
bool gogobright_library::setOutputPower(int power)
{
    if (power < 0 || power > 100)
        return false;

    if (!wireWriteDataByte(CMD_MOTOR_PWR, power))
    {
        return false;
    }

    return true;
}
bool gogobright_library::turnOutputON(void)
{
    if (!wireWriteDataByte(CMD_MOTOR_ON, 1))
    {
        return false;
    }

    return true;
}
bool gogobright_library::turnOutputOFF(void)
{
    if (!wireWriteDataByte(CMD_MOTOR_OFF, 0))
    {
        return false;
    }

    return true;
}
bool gogobright_library::turnOutputThisWay(void)
{
    if (!wireWriteDataByte(CMD_MOTOR_CW, 1))
    {
        return false;
    }

    return true;
}
bool gogobright_library::turnOutputThatWay(void)
{
    if (!wireWriteDataByte(CMD_MOTOR_CCW, 0))
    {
        return false;
    }

    return true;
}
bool gogobright_library::toggleOutputWay(void)
{
    if (!wireWriteDataByte(CMD_MOTOR_RD, 1))
    {
        return false;
    }

    return true;
}

bool gogobright_library::i2cWrite(uint8_t addr, uint8_t reg, uint8_t value)
{
    if (!wireWriteDataByteToAddr(CMD_I2C_WRITE, addr, reg, value))
    {
        return false;
    }

    return true;
}
uint8_t gogobright_library::i2cRead(uint8_t addr, uint8_t reg)
{
    uint8_t val_byte = 0;
    if (!wireReadDataByteFromAddr(CMD_I2C_READ, addr, reg, val_byte))
    {
        return 0;
    }

    return val_byte;
}

//* Reference from SparkFun_APDS9960 Library
/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

/**
 * @brief Writes a single byte to the I2C device (no register)
 *
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool gogobright_library::wireWriteByte(uint8_t val)
{
    Wire.beginTransmission(_i2cAddr);
    Wire.write(val);
    if (Wire.endTransmission() != 0)
    {
        return false;
    }

    return true;
}

/**
 * @brief Writes a single byte to the I2C device and specified register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool gogobright_library::wireWriteDataByte(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    Wire.write(val);
    if (Wire.endTransmission() != 0)
    {
        return false;
    }

    return true;
}

bool gogobright_library::wireWriteDataByteToAddr(uint8_t cmd, uint8_t addr, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(_i2cAddr);
    Wire.write(cmd);
    Wire.write(addr);
    Wire.write(reg);
    Wire.write(val);
    if (Wire.endTransmission() != 0)
    {
        return false;
    }

    return true;
}

/**
 * @brief Writes a block (array) of bytes to the I2C device and register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val pointer to the beginning of the data byte array
 * @param[in] len the length (in bytes) of the data to write
 * @return True if successful write operation. False otherwise.
 */
bool gogobright_library::wireWriteDataBlock(uint8_t reg,
                                            uint8_t *val,
                                            unsigned int len)
{
    unsigned int i;

    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    for (i = 0; i < len; i++)
    {
        Wire.beginTransmission(val[i]);
    }
    if (Wire.endTransmission() != 0)
    {
        return false;
    }

    return true;
}

/**
 * @brief Reads a single byte from the I2C device and specified register
 *
 * @param[in] reg the register to read from
 * @param[out] the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
bool gogobright_library::wireReadDataByte(uint8_t reg, uint8_t &val)
{

    /* Indicate which register we want to read from */
    if (!wireWriteByte(reg))
    {
        return false;
    }

    /* Read from register */
    Wire.requestFrom(_i2cAddr, 1);
    while (Wire.available())
    {
        val = Wire.read();
    }

    return true;
}

bool gogobright_library::wireReadDataByteFromAddr(uint8_t cmd, uint8_t addr, uint8_t reg, uint8_t &val)
{
    Wire.beginTransmission(_i2cAddr);
    Wire.write(cmd);
    Wire.write(addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0)
    {
        return false;
    }

    /* Read from register */
    Wire.requestFrom(_i2cAddr, 1);
    while (Wire.available())
    {
        val = Wire.read();
    }

    return true;
}

/**
 * @brief Reads a block (array) of bytes from the I2C device and register
 *
 * @param[in] reg the register to read from
 * @param[out] val pointer to the beginning of the data
 * @param[in] len number of bytes to read
 * @return Number of bytes read. -1 on read error.
 */
int gogobright_library::wireReadDataBlock(uint8_t reg,
                                          uint8_t *val,
                                          unsigned int len)
{
    unsigned char i = 0;

    /* Indicate which register we want to read from */
    if (!wireWriteByte(reg))
    {
        return -1;
    }

    /* Read block data */
    Wire.requestFrom(_i2cAddr, len);
    while (Wire.available())
    {
        if (i >= len)
        {
            return -1;
        }
        val[i] = Wire.read();
        i++;
    }

    return i;
}
