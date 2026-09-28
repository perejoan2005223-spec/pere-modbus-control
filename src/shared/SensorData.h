#pragma once
#include "FreeRTOS.h"
struct read_data
{
    float value = 0.0f;
    bool valid = false;
};

struct SensorData {
    read_data CO2; // in ppm
    read_data Humidity; // in %
    read_data Temperature; // in Celsius
    read_data Pressure; // in Pa
    TickType_t timestamp = 0;
};