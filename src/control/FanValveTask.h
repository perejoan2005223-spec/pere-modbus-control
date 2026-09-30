#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "actuators/Valve.h"
#include "actuators/Fan.h"

struct FanValveTaskParams {
    QueueHandle_t sensorQueue;
    Valve* valve;
    QueueHandle_t configQueue; // latest user settings, shared with the UI
    TickType_t maxAge;
    Fan* fan;
    QueueHandle_t statusQueue = nullptr; // latest control status for other tasks
};

void FanValveTask(void* params);
