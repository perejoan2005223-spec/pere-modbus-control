#pragma once

#include "shared/SensorData.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "actuators/Valve.h"
#include "actuators/Fan.h"

struct FanValveTaskParams {
    QueueHandle_t sensor_queue;
    Valve* valve;
    float co2_threshold;
    TickType_t max_age;
    Fan* fan;
};

 void FanValveTask(void* params);