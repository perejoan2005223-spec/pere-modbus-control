#pragma once

#include "shared/SensorData.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "actuators/Valve.h"

struct FanValveTaskParams {
    QueueHandle_t sensor_queue;
    Valve* valve;
    float co2_threshold;
    TickType_t max_age;
};

 void FanValveTask(void* params);