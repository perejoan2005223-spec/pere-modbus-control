#pragma once

#include "shared/SensorData.h"
#include "FreeRTOS.h"
#include "queue.h"

struct FanValveTaskParams {
    QueueHandle_t sensor_queue;
};