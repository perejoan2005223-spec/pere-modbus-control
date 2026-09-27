#pragma once

// keep one copy in config_queue, other tasks read it without removing it
struct ControlConfig
{
    float co2_target = 1000.0f; // initial value in ppm, the user can change it later
};
