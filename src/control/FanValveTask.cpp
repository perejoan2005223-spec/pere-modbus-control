#include "FanValveTask.h"

#include "shared/SensorData.h"
#include "FreeRTOS.h"
#include "queue.h"

void FanValveTask(void* params)
{
    SensorData data;

    while (true)
    {
        int a = xQueueReceive(((FanValveTaskParams*)params)->sensor_queue, &data, portMAX_DELAY);
        if (a == pdTRUE)
        {
            if (data.CO2.valid)
            {
                // More CO2
            }
        }
    }
}
