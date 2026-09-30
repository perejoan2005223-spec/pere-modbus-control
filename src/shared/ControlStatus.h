#pragma once

#include "FreeRTOS.h"

struct ControlStatus
{
    bool valveOpen = false;
    bool ventilating = false;

    int fanSpeed = -1; // last confirmed command, -1 means unknown

    bool fanFault = false; // no pulses when the fan should be turning
    bool fanCommFault = true; // no successful pulse read yet
    bool fanWriteFault = false; // the speed command failed

    TickType_t timestamp = 0; // when we updated this status
};