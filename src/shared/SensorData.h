#pragma once

#include "FreeRTOS.h"

struct ReadData
{
    float value = 0.0f;
    bool valid = false;
};

struct SensorData {
    ReadData co2; // in ppm
    ReadData humidity; // in %
    ReadData temperature; // in Celsius
    ReadData pressure; // in Pa
    TickType_t timestamp = 0;
};