#pragma once
#include "FreeRTOS.h"
struct read_data
{
    float value = 0.0f;
    bool valid = false;
    TickType_t timestamp = 0;
};

struct SensorData {
    read_data CO2; // in ppm
    read_data Humidity; // in %
    read_data Temperature; // in Celsius
};