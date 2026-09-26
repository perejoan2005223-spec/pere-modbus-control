#include "Fan.h"

bool Fan::setSpeed(unsigned int speed)
{
    if (speed > 100)
    {
        speed = 100;
    }

    unsigned int speed_percent = speed * 10;

    // address of MIO12 = 1, address of AO1 = 0
    return this->modbus->writeSingleRegister(1,0,speed_percent);
}

Fan::Fan(Modbus* modbus)
{
    this->modbus = modbus;
}