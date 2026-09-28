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

// constructor
Fan::Fan(Modbus* modbus)
{
    this->modbus = modbus;
}

bool Fan::readPulses(uint16_t& pulses)
{
    // address of MIO12 = 1, AI1 pulse counter = 4, store the result in pulses
    return this->modbus->readRegisters(1,0x04,4,&pulses,1);
}