#pragma once

struct Valve
{
    void open() const;
    void close() const;
    unsigned int pin; // GPIO pin number
    bool open_level;

    Valve(unsigned int pin, bool open_level);
};