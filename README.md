# GoGoBright Library for KidBright Arduino

## Compatibility with the GoGoBright library

This library is fully compatible with the esp32 arduino based on KidBright board.

## GoGoBright library description
GoGoBright is an extension for the KidBright. It runs on specific Kidbright board with arduino programming.

## Usage

```sh
#include <GoGoBright.h>
GoGoBright gogoIO;
```

### List of commands
|                |Functions name                 |Parameter(s)         |Return value|
|----------------|-------------------------------|---------------------|------------|
|**Sensor**      |`readInput(`**param**`)`      |port number (1,2,3,4)|sensor value (0-1023)
||
|**Servo**       |`talkToServo(`**"param"**`)`  |port name (A,B,C,D)   |Boolean
|                |`setServoHead(`**param**`)`    |servo angle (0-180)  |Boolean
|                |`turnServoCW(`**param**`)`     |servo angle (0-180)  |Boolean
|                |`turnServoCCW(`**param**`)`    |servo angle (0-180)  |Boolean
||
|**Output**      |`talkToOutput(`**"param"**`)` |port name (1,2,3,4)  |Boolean
|                |`setOutputPower(`**param**`)`  |power value (0-100)  |Boolean
|                |`turnOutputON(void)`           |-                    |Boolean
|                |`turnOutputOFF(void)`          |-                    |Boolean
|                |`turnOutputThisWay(void)`      |-                    |Boolean
|                |`turnOutputThatWay(void)`      |-                    |Boolean
|                |`toggleOutputWay(void)`        |-                    |Boolean