#include <Arduino.h>
#include <Wire.h>
#include "GoGoBright.h"

GoGoBrightLib::GoGoBrightLib(void)
{
}
GoGoBrightLib::~GoGoBrightLib(void)
{
}

bool GoGoBrightLib::begin(int8_t i2cAddr)
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

int GoGoBrightLib::readInput(int port)
{
    if (port < 1 || port > 4)
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

bool GoGoBrightLib::talkToServo(String servo_port)
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
    }

    if (!wireWriteDataByte(CATEGORY_CMD, CMD_SERVO_ACTIVE, servoBits))
    {
        return false;
    }

    return true;
}
bool GoGoBrightLib::setServoHead(int head_angle)
{
    if (head_angle < 0 || head_angle > 180)
        return false;

    //! add this line for gogo5 report compatibility
    // head_angle = map(head_angle, 0, 180, 10, 40);

    uint8_t dataTmp[3] = {0, (head_angle >> 8), (head_angle & 0xFF)};
    if (!wireWriteDataBlock(CATEGORY_CMD, CMD_SERVO_SET_ANGLE, dataTmp, 3))
    {
        return false;
    }

    return true;
}
bool GoGoBrightLib::talkToOutput(String output_port)
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

    if (!wireWriteDataByte(CATEGORY_CMD, CMD_MOTOR_ACTIVE, motorBits))
    {
        return false;
    }

    return true;
}
bool GoGoBrightLib::setOutputPower(int power)
{
    if (power < 0 || power > 100)
        return false;

    uint8_t dataTmp[3] = {0, power >> 8, power & 0xFF};
    if (!wireWriteDataBlock(CATEGORY_CMD, CMD_MOTOR_PWR, dataTmp, 3))
    {
        return false;
    }

    return true;
}
bool GoGoBrightLib::turnOutputONOFF(int state)
{
    state &= 1;
    uint8_t dataTmp[2] = {0, state};
    if (!wireWriteDataBlock(CATEGORY_CMD, CMD_MOTOR_ONOFF, dataTmp, 2))
    {
        return false;
    }

    return true;
}
// bool GoGoBrightLib::turnOutputONOFF(String stateStr)
// {
//     int state = 0;
//     stateStr.toLowerCase();
//     if (stateStr.length() < 1 || stateStr.length() > 4)
//         return false;

//     if (stateStr == "on" | stateStr == "1")
//     {
//         state = 1;
//     }
//     else if (stateStr == "off" | stateStr == "0")
//     {
//         state = 0;
//     }
//     else
//     {
//         return false;
//     }
//     uint8_t dataTmp[2] = {0, state};
//     if (!wireWriteDataBlock(CATEGORY_CMD, CMD_MOTOR_ONOFF, dataTmp, 2))
//     {
//         return false;
//     }

//     return true;
// }
bool GoGoBrightLib::turnOutputON(void)
{
    return turnOutputONOFF(1);
}
bool GoGoBrightLib::turnOutputOFF(void)
{
    return turnOutputONOFF(0);
}
bool GoGoBrightLib::turnOutputDirection(int dir)
{
    dir &= 1; //* 1=CW, 0=CCW
    uint8_t dataTmp[2] = {0, dir};
    if (!wireWriteDataBlock(CATEGORY_CMD, CMD_MOTOR_DIR, dataTmp, 2))
    {
        return false;
    }

    return true;
}
// bool GoGoBrightLib::turnOutputDirection(String dirStr)
// {
//     int dir = 0;
//     dirStr.toLowerCase();
//     if (dirStr == "left" | dirStr == "l" | dirStr == "ccw" | dirStr == "counter-clockwise")
//     {
//         dir = 0;
//     }
//     else if (dirStr == "right" | dirStr == "r" | dirStr == "cw" | dirStr == "clockwise")
//     {
//         dir = 1;
//     }
//     else
//     {
//         return false;
//     }
//     uint8_t dataTmp[2] = {0, dir};
//     if (!wireWriteDataBlock(CATEGORY_CMD, CMD_MOTOR_DIR, dataTmp, 2))
//     {
//         return false;
//     }

//     return true;
// }
bool GoGoBrightLib::turnOutputThisWay(void)
{
    return turnOutputDirection(1);
}
bool GoGoBrightLib::turnOutputThatWay(void)
{
    return turnOutputDirection(0);
}
bool GoGoBrightLib::toggleOutputWay(void)
{
    if (!wireWriteDataByte(CATEGORY_CMD, CMD_MOTOR_RD, 0))
    {
        return false;
    }

    return true;
}

// bool GoGoBrightLib::i2cWrite(uint8_t addr, uint8_t reg, uint8_t value)
// {
//     if (!wireWriteDataByteToAddr(CMD_I2C_WRITE, addr, reg, value))
//     {
//         return false;
//     }

//     return true;
// }
// uint8_t GoGoBrightLib::i2cRead(uint8_t addr, uint8_t reg)
// {
//     uint8_t val_byte = 0;
//     if (!wireReadDataByteFromAddr(CMD_I2C_READ, addr, reg, val_byte))
//     {
//         return 0;
//     }

//     return val_byte;
// }

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
bool GoGoBrightLib::wireWriteByte(uint8_t val)
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
bool GoGoBrightLib::wireWriteDataByte(uint8_t category, uint8_t cmd, uint8_t val)
{
    Wire.beginTransmission(_i2cAddr);
    Wire.write(category);
    Wire.write(cmd);
    Wire.write(val);
    if (Wire.endTransmission() != 0)
    {
        return false;
    }

    return true;
}

bool GoGoBrightLib::wireWriteDataByteToAddr(uint8_t category, uint8_t cmd, uint8_t addr, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(_i2cAddr);
    Wire.write(category);
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
bool GoGoBrightLib::wireWriteDataBlock(uint8_t category, uint8_t cmd, uint8_t *val, unsigned int len)
{
    unsigned int i;

    Wire.beginTransmission(_i2cAddr);
    Wire.write(category);
    Wire.write(cmd);
    Wire.write(val, len);
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
bool GoGoBrightLib::wireReadDataByte(uint8_t reg, uint8_t &val)
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

bool GoGoBrightLib::wireReadDataByteFromAddr(uint8_t cmd, uint8_t addr, uint8_t reg, uint8_t &val)
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
int GoGoBrightLib::wireReadDataBlock(uint8_t reg,
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
