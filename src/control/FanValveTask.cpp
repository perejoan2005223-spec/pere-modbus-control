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
constexpr TickType_t queueWait = pdMS_TO_TICKS(50);
constexpr TickType_t valveOpenTime = pdMS_TO_TICKS(1000);
constexpr TickType_t injectionWait = pdMS_TO_TICKS(30000);
constexpr TickType_t pulseCheckInterval = pdMS_TO_TICKS(1000);
constexpr float ventilationLimit = 2000.0f;
constexpr float maximumCo2Target = 1500.0f; // maximum setting from the specification

// keep the values that we need to remember between loop iterations
struct ControlState
{
    bool valveOpen = false;
    bool hasInjected = false;
    TickType_t openAt = 0;
    TickType_t closedAt = 0;

    bool ventilating = false;
    int fanSpeed = -1; // we still don't know the confirmed fan speed
    bool fanFault = false; // no fan fault detected yet
    bool fanCommFault = true; // we still don't have a successful pulse read
    bool fanWriteFault = false; // no speed command has failed yet

    unsigned int zeroPulseReads = 0;
    TickType_t lastPulseRead = 0;
};

bool readControlConfig(const FanValveTaskParams& params, ControlConfig& config);
void publishStatus(const FanValveTaskParams& params, const ControlState& state);
void closeValve(const FanValveTaskParams& params, ControlState& state);
void checkValveTimeout(const FanValveTaskParams& params, ControlState& state, TickType_t now);
void tryOpenValve(const FanValveTaskParams& params, ControlState& state);
void processSensorData(const FanValveTaskParams& params, ControlState& state, const SensorData& data, TickType_t now, const ControlConfig* config);
void updateFanSpeed(const FanValveTaskParams& params, ControlState& state);
void updateFanFault(ControlState& state, uint16_t pulses);
void checkFanPulses(const FanValveTaskParams& params, ControlState& state);
}

void FanValveTask(void* params)
{

    // parameters to be used
    const auto* taskParams = static_cast<FanValveTaskParams*>(params);
    ControlState state;
    SensorData data{};
    bool haveSensorData = false;
    ControlConfig previousConfig{};
    bool hadConfig = false;

    taskParams->valve->close();
    state.lastPulseRead = xTaskGetTickCount();
    publishStatus(*taskParams, state);

    while (true)
    {
        // receive data, but wake up regularly to check the valve timer
        const BaseType_t received = xQueueReceive(taskParams->sensorQueue, &data, queueWait);
        const TickType_t now = xTaskGetTickCount();

        checkValveTimeout(*taskParams, state, now);

        if (received == pdTRUE)
        {
            haveSensorData = true;
        }

        ControlConfig config{};
        const bool configOk = readControlConfig(*taskParams, config);
        const bool configChanged = configOk != hadConfig ||
            (configOk && config.co2Target != previousConfig.co2Target);

        if (!configOk)
        {
            closeValve(*taskParams, state);
        }

        // also react to a new target without waiting for another sensor message
        // while open, keep checking that the saved reading is still recent
        if (haveSensorData &&
            (received == pdTRUE || configChanged || state.valveOpen))
        {
            processSensorData(*taskParams, state, data, now, configOk ? &config : nullptr);
        }

        if (configOk)
        {
            previousConfig = config;
        }
        hadConfig = configOk;

        updateFanSpeed(*taskParams, state);
        checkFanPulses(*taskParams, state);
        publishStatus(*taskParams, state);
    }
}

namespace
{
bool readControlConfig(const FanValveTaskParams& params, ControlConfig& config)
{
    // peek copies the whole config safely and leaves it there for the other tasks
    // do not wait here, as we still need to check the valve timer
    if (params.configQueue == nullptr || xQueuePeek(params.configQueue, &config, 0) != pdTRUE)
    {
        return false;
    }

    // a missing or invalid setting must not allow an injection
    return std::isfinite(config.co2Target) && config.co2Target >= 0.0f &&
           config.co2Target <= maximumCo2Target;
}

void processSensorData(const FanValveTaskParams& params, ControlState& state,
                       const SensorData& data, TickType_t now, const ControlConfig* config)
{
    const TickType_t age = now - data.timestamp;

    // close if the CO2 reading failed, has a bad value or is too old
    if (!data.co2.valid || !std::isfinite(data.co2.value) ||
        data.co2.value < 0.0f || age > params.maxAge)
    {
        closeValve(params, state);
        return;
    }

    // between these two limits, keep the previous ventilation state
    if (data.co2.value > ventilationLimit)
    {
        state.ventilating = true;
    }
    else if (config != nullptr && data.co2.value <= config->co2Target)
    {
        state.ventilating = false;
    }

    // only inject if we need CO2 and no fan fault is detected
    const bool canInject = config != nullptr && data.co2.value < config->co2Target &&
                            !state.ventilating && state.fanSpeed == 0 &&
                            !state.fanFault && !state.fanCommFault &&
                            !state.fanWriteFault;

    if (!canInject)
    {
        closeValve(params, state);
        return;
    }

    tryOpenValve(params, state);
}

void updateFanSpeed(const FanValveTaskParams& params, ControlState& state)
{
    // avoid waiting for Modbus while the valve is open
    if (state.valveOpen)
    {
        return;
    }

    const int wantedFanSpeed = state.ventilating ? 100 : 0;

    // send a command only if the speed changed or the last command failed
    if (state.fanSpeed == wantedFanSpeed && !state.fanWriteFault)
    {
        return;
    }

    const bool writeOk = params.fan->setSpeed(wantedFanSpeed);
    state.fanWriteFault = !writeOk;

    if (writeOk)
    {
        // save the speed only if the device accepted the command
        state.fanSpeed = wantedFanSpeed;

        // start a new pulse check after changing the speed
        state.zeroPulseReads = 0;
        state.lastPulseRead = xTaskGetTickCount();
    }
}

void checkFanPulses(const FanValveTaskParams& params, ControlState& state)
{
    // check about once per second, with the valve closed
    if (state.valveOpen ||
        (TickType_t)(xTaskGetTickCount() - state.lastPulseRead) <
            pulseCheckInterval)
    {
        return;
    }

    uint16_t pulses = 0;
    const bool readOk = params.fan->readPulses(pulses);
    state.fanCommFault = !readOk;
    state.lastPulseRead = xTaskGetTickCount();

    if (!readOk)
    {
        // communication failed, start the count again but keep the fan fault
        state.zeroPulseReads = 0;
        return;
    }

    updateFanFault(state, pulses);
}

void publishStatus(const FanValveTaskParams& params, const ControlState& state)
{
    // other tasks may not be using the status queue yet
    if (params.statusQueue == nullptr)
    {
        return;
    }

    // copy the current control state for the other tasks
    ControlStatus status{};
    status.valveOpen = state.valveOpen;
    status.ventilating = state.ventilating;
    status.fanSpeed = state.fanSpeed;
    status.fanFault = state.fanFault;
    status.fanCommFault = state.fanCommFault;
    status.fanWriteFault = state.fanWriteFault;
    // time when published
    status.timestamp = xTaskGetTickCount();

    // replace the previous status with the latest one
    xQueueOverwrite(params.statusQueue, &status);
}

void checkValveTimeout(const FanValveTaskParams& params, ControlState& state,
                       TickType_t now)
{
    // close after 1 second, even if no sensor data arrives
    if (state.valveOpen &&
        (TickType_t)(now - state.openAt) >= valveOpenTime)
    {
        closeValve(params, state);
    }
}

void tryOpenValve(const FanValveTaskParams& params, ControlState& state)
{
    if (state.valveOpen)
    {
        return;
    }

    // after the first injection, wait 30 seconds from when we closed the valve
    if (state.hasInjected &&
        (TickType_t)(xTaskGetTickCount() - state.closedAt) < injectionWait)
    {
        return;
    }

    params.valve->open();
    state.valveOpen = true;
    state.hasInjected = true;
    state.openAt = xTaskGetTickCount();
}

void closeValve(const FanValveTaskParams& params, ControlState& state)
{
    params.valve->close();

    // only start the waiting time if the valve was open
    if (state.valveOpen)
    {
        state.closedAt = xTaskGetTickCount();
    }

    state.valveOpen = false;
}

void updateFanFault(ControlState& state, uint16_t pulses)
{
    if (pulses == 0)
    {
        // stop counting at 2, that is enough
        if (state.zeroPulseReads < 2)
        {
            state.zeroPulseReads++;
        }
    }
    else
    {
        // we received pulses, the fan is turning
        state.zeroPulseReads = 0;
    }

    // we asked the fan to run but two reads had no pulses
    if (state.fanSpeed > 0 && state.zeroPulseReads >= 2)
    {
        state.fanFault = true;
    }
    else if (pulses > 0 || state.fanSpeed == 0)
    {
        // the fan is turning, or we asked it to stop
        state.fanFault = false;
    }
}} // these helpers are only used in this file
