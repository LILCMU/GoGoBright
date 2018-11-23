#include "GoGoBright.h"

GoGoBright gogoIO;

void setup()
{
    Serial.begin(115200);
    gogoIO.begin();
    delay(100);

    gogoIO.talkToOutput("abcd");
    gogoIO.talkToServo("1234");
}

void loop()
{
    int sensor = gogoIO.readInput(1);
    Serial.print("Sensor value: ");
    Serial.println(sensor);

    if (sensor > 600)
    {
        gogoIO.turnOutputONOFF(0);
        gogoIO.setServoHead(40);
    }
    else if (sensor < 400)
    {
        gogoIO.turnOutputONOFF(1);
        gogoIO.setServoHead(140);
    }
    delay(100);
}