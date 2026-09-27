#include "FanValveTask.h"

#include "shared/SensorData.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "shared/ControlStatus.h"
#include "shared/ControlConfig.h"
#include <cmath>

// values to use in the control loop
namespace
{
constexpr TickType_t queue_wait = pdMS_TO_TICKS(50);
constexpr TickType_t valve_open_time = pdMS_TO_TICKS(1000);
constexpr TickType_t injection_wait = pdMS_TO_TICKS(30000);
constexpr TickType_t pulse_check_interval = pdMS_TO_TICKS(1000);
constexpr float ventilation_limit = 2000.0f;
constexpr float maximum_co2_target = 1500.0f; // maximum setting from the specification

// keep the values that we need to remember between loop iterations
struct ControlState
{
    bool valve_open = false;
    bool has_injected = false;
    TickType_t open_at = 0;
    TickType_t closed_at = 0;

    bool ventilating = false;
    int fan_speed = -1; // we still don't know the confirmed fan speed
    bool fan_fault = false; // no fan fault detected yet
    bool fan_comm_fault = true; // we still don't have a successful pulse read
    bool fan_write_fault = false; // no speed command has failed yet

    unsigned int zero_pulse_reads = 0;
    TickType_t last_pulse_read = 0;
};

bool readControlConfig(const FanValveTaskParams& params, ControlConfig& config)
{
    // peek copies the whole config safely and leaves it there for the other tasks
    // do not wait here, as we still need to check the valve timer
    if (params.config_queue == nullptr ||
        xQueuePeek(params.config_queue, &config, 0) != pdTRUE)
    {
        return false;
    }

    // a missing or invalid setting must not allow an injection
    return std::isfinite(config.co2_target) && config.co2_target >= 0.0f &&
           config.co2_target <= maximum_co2_target;
}

void publishStatus(const FanValveTaskParams& params, const ControlState& state)
{
    // other tasks may not be using the status queue yet
    if (params.status_queue == nullptr)
    {
        return;
    }

    // copy the current control state for the other tasks
    ControlStatus status{};
    status.valve_open = state.valve_open;
    status.ventilating = state.ventilating;
    status.fan_speed = state.fan_speed;
    status.fan_fault = state.fan_fault;
    status.fan_comm_fault = state.fan_comm_fault;
    status.fan_write_fault = state.fan_write_fault;
    // time when published
    status.timestamp = xTaskGetTickCount();

    // replace the previous status with the latest one
    xQueueOverwrite(params.status_queue, &status);
}

void closeValve(const FanValveTaskParams& params, ControlState& state)
{
    params.valve->close();

    // only start the waiting time if the valve was open
    if (state.valve_open)
    {
        state.closed_at = xTaskGetTickCount();
    }

    state.valve_open = false;
}

void checkValveTimeout(const FanValveTaskParams& params, ControlState& state,
                       TickType_t now)
{
    // close after 1 second, even if no sensor data arrives
    if (state.valve_open &&
        (TickType_t)(now - state.open_at) >= valve_open_time)
    {
        closeValve(params, state);
    }
}

void tryOpenValve(const FanValveTaskParams& params, ControlState& state)
{
    if (state.valve_open)
    {
        return;
    }

    // after the first injection, wait 30 seconds from when we closed the valve
    if (state.has_injected &&
        (TickType_t)(xTaskGetTickCount() - state.closed_at) < injection_wait)
    {
        return;
    }

    params.valve->open();
    state.valve_open = true;
    state.has_injected = true;
    state.open_at = xTaskGetTickCount();
}

void processSensorData(const FanValveTaskParams& params, ControlState& state,
                       const SensorData& data, TickType_t now, const ControlConfig* config)
{
    const TickType_t age = now - data.CO2.timestamp;

    // close if the CO2 reading failed, has a bad value or is too old
    if (!data.CO2.valid || !std::isfinite(data.CO2.value) ||
        data.CO2.value < 0.0f || age > params.max_age)
    {
        closeValve(params, state);
        return;
    }

    // between these two limits, keep the previous ventilation state
    if (data.CO2.value > ventilation_limit)
    {
        state.ventilating = true;
    }
    else if (config != nullptr && data.CO2.value <= config->co2_target)
    {
        state.ventilating = false;
    }

    // only inject if we need CO2 and no fan fault is detected
    const bool can_inject = config != nullptr && data.CO2.value < config->co2_target &&
                            !state.ventilating && state.fan_speed == 0 &&
                            !state.fan_fault && !state.fan_comm_fault &&
                            !state.fan_write_fault;

    if (!can_inject)
    {
        closeValve(params, state);
        return;
    }

    tryOpenValve(params, state);
}

void updateFanSpeed(const FanValveTaskParams& params, ControlState& state)
{
    // avoid waiting for Modbus while the valve is open
    if (state.valve_open)
    {
        return;
    }

    const int wanted_fan_speed = state.ventilating ? 100 : 0;

    // send a command only if the speed changed or the last command failed
    if (state.fan_speed == wanted_fan_speed && !state.fan_write_fault)
    {
        return;
    }

    const bool write_ok = params.fan->setSpeed(wanted_fan_speed);
    state.fan_write_fault = !write_ok;

    if (write_ok)
    {
        // save the speed only if the device accepted the command
        state.fan_speed = wanted_fan_speed;

        // start a new pulse check after changing the speed
        state.zero_pulse_reads = 0;
        state.last_pulse_read = xTaskGetTickCount();
    }
}

void updateFanFault(ControlState& state, uint16_t pulses)
{
    if (pulses == 0)
    {
        // stop counting at 2, that is enough
        if (state.zero_pulse_reads < 2)
        {
            state.zero_pulse_reads++;
        }
    }
    else
    {
        // we received pulses, the fan is turning
        state.zero_pulse_reads = 0;
    }

    // we asked the fan to run but two reads had no pulses
    if (state.fan_speed > 0 && state.zero_pulse_reads >= 2)
    {
        state.fan_fault = true;
    }
    else if (pulses > 0 || state.fan_speed == 0)
    {
        // the fan is turning, or we asked it to stop
        state.fan_fault = false;
    }
}

void checkFanPulses(const FanValveTaskParams& params, ControlState& state)
{
    // check about once per second, with the valve closed
    if (state.valve_open ||
        (TickType_t)(xTaskGetTickCount() - state.last_pulse_read) <
            pulse_check_interval)
    {
        return;
    }

    uint16_t pulses = 0;
    const bool read_ok = params.fan->readPulses(pulses);
    state.fan_comm_fault = !read_ok;
    state.last_pulse_read = xTaskGetTickCount();

    if (!read_ok)
    {
        // communication failed, start the count again but keep the fan fault
        state.zero_pulse_reads = 0;
        return;
    }

    updateFanFault(state, pulses);
}
} // these helpers are only used in this file

void FanValveTask(void* params)
{
    const auto* task_params = static_cast<FanValveTaskParams*>(params);
    ControlState state;
    SensorData data{};
    bool have_sensor_data = false;
    ControlConfig previous_config{};
    bool had_config = false;

    task_params->valve->close();
    state.last_pulse_read = xTaskGetTickCount();
    publishStatus(*task_params, state);

    while (true)
    {
        // receive data, but wake up regularly to check the valve timer
        const BaseType_t received = xQueueReceive(
            task_params->sensor_queue, &data, queue_wait);
        const TickType_t now = xTaskGetTickCount();

        checkValveTimeout(*task_params, state, now);

        if (received == pdTRUE)
        {
            have_sensor_data = true;
        }

        ControlConfig config{};
        const bool config_ok = readControlConfig(*task_params, config);
        const bool config_changed = config_ok != had_config ||
            (config_ok && config.co2_target != previous_config.co2_target);

        if (!config_ok)
        {
            closeValve(*task_params, state);
        }

        // also react to a new target without waiting for another sensor message
        // while open, keep checking that the saved reading is still recent
        if (have_sensor_data &&
            (received == pdTRUE || config_changed || state.valve_open))
        {
            processSensorData(*task_params, state, data, now, config_ok ? &config : nullptr);
        }

        if (config_ok)
        {
            previous_config = config;
        }
        had_config = config_ok;

        updateFanSpeed(*task_params, state);
        checkFanPulses(*task_params, state);
        publishStatus(*task_params, state);
    }
}
