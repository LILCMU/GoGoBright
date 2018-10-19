#include "gogoBright.h"

gogobright_library gogoIO;

void setup()
{
    Serial.begin(115200);
    gogoIO.begin();
    delay(100);
}

void loop()
{
    int sensor = gogoIO.readInput(1);
    Serial.print("Sensor value: ");
    Serial.println(sensor);

    if (sensor > 600)
    {
        gogoIO.talkToMotor("cd");
        gogoIO.talkToServo("1234");
        gogoIO.turnMotorOFF();
        gogoIO.setServoHead(40);
    }
    else if (sensor < 400)
    {
        gogoIO.talkToMotor("cd");
        gogoIO.talkToServo("1234");
        gogoIO.turnMotorON();
        gogoIO.setServoHead(140);
    }
    delay(100);
}