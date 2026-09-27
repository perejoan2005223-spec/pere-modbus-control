#pragma once

#include "shared/SensorData.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "actuators/Valve.h"
#include "actuators/Fan.h"

struct FanValveTaskParams {
    QueueHandle_t sensor_queue;
    Valve* valve;
    QueueHandle_t config_queue; // latest user settings, shared with the UI
    TickType_t max_age;
    Fan* fan;
    QueueHandle_t status_queue = nullptr; // latest control status for other tasks
};

void FanValveTask(void* params);
