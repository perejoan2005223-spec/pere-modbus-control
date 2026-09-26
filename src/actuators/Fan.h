#pragma once
#include "modbus/Modbus.h"

class Fan
{
    public:
        bool setSpeed(unsigned int speed);
        Fan(Modbus* modbus);
    private:
        Modbus* modbus;
};