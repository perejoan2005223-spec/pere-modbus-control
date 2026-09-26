#include "FanValveTask.h"

#include "shared/SensorData.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stdio.h"
#include "task.h"

void FanValveTask(void* params)
{
    auto* task_params = static_cast<FanValveTaskParams*>(params);
    SensorData data;

    bool valve_open = false;
    bool has_injected = false;
    TickType_t open_at = 0;
    TickType_t closed_at = 0;

    task_params->valve->close();

    while (true)
    {
        // Recive data from queue
        BaseType_t i = xQueueReceive(
            task_params->sensor_queue, &data, pdMS_TO_TICKS(50));

        TickType_t now = xTaskGetTickCount();

        // if valve is open and it's been more than 1 second then close it
        if (valve_open &&
            (TickType_t)(now - open_at) >= pdMS_TO_TICKS(1000))
        {
            task_params->valve->close();
            valve_open = false;
            closed_at = xTaskGetTickCount();
        }

        if (i == pdTRUE)
        {
            // get age of CO2 data
            TickType_t age = now - data.CO2.timestamp;

            if (data.CO2.valid)
            {
                // check if we have to open, more CO2 and not too old
                if (data.CO2.value < task_params->co2_threshold &&
                    age <= task_params->max_age)
                {
                    // if valve is closed and it's been more than 30 seconds since it was closed then open it
                    if (!valve_open && (!has_injected || (TickType_t)(xTaskGetTickCount() - closed_at) >= pdMS_TO_TICKS(30000)))
                    {
                        task_params->valve->open();
                        valve_open = true;
                        has_injected = true;
                        open_at = xTaskGetTickCount();
                    }
                }
                else
                {
                    // close and get the time
                    task_params->valve->close();

                    if (valve_open)
                    {
                        closed_at = xTaskGetTickCount();
                    }

                    valve_open = false;
                }
            }
            else
            {
                // close and get the time
                task_params->valve->close();

                if (valve_open)
                {
                    closed_at = xTaskGetTickCount();
                }

                valve_open = false;
            }
        }
    }
}