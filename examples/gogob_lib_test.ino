#include "gogoBright.h"

gogobright_library gogoIO;

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println(gogoIO.begin());
    // gogoIO.talk_to_motor("cd");
    // gogoIO.motor_turn_cw();
    // gogoIO.motor_on();
}

void loop()
{
    int sensor = gogoIO.readInput(1);
    Serial.print("Sensor value: ");
    Serial.println(sensor);

    if (sensor > 600)
    {
        gogoIO.talk_to_motor("cd");
        gogoIO.talk_to_servo("1234");
        gogoIO.motor_on();
        gogoIO.servo_set_head(40);
    }
    else if (sensor < 400)
    {
        gogoIO.talk_to_motor("cd");
        gogoIO.talk_to_servo("1234");
        gogoIO.motor_off();
        gogoIO.servo_set_head(140);
    }

    delay(100);
    // Serial.println(gogoIO.readInput(1));
    // delay(50);
    // gogoIO.talk_to_servo("1234");
    // delay(50);
    // gogoIO.motor_power(100);
    // delay(1000);
    // gogoIO.motor_power(75);
    // delay(1000);
    // gogoIO.motor_power(50);
    // delay(1000);
    // gogoIO.motor_power(25);
    // delay(1000);
    // gogoIO.motor_power(0);
    // delay(1000);
    // gogoIO.motor_toggle_direction();
    // delay(1000);
    // gogoIO.servo_set_head(30);
    // delay(1000);
    // gogoIO.servo_set_head(150);
    // delay(1000);
}