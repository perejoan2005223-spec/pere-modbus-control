#pragma once

#include "FreeRTOS.h"

struct ControlStatus
{
    bool valve_open = false;
    bool ventilating = false;

    int fan_speed = -1; // last confirmed command, -1 means unknown

    bool fan_fault = false; // no pulses when the fan should be turning
    bool fan_comm_fault = true; // no successful pulse read yet
    bool fan_write_fault = false; // the speed command failed

    TickType_t timestamp = 0; // when we updated this status
};