#pragma once

class Valve
{
    public:
        void open() const;
        void close() const;
        Valve(unsigned int pin, bool open_level);
    private:
        unsigned int pin; // GPIO pin number
        bool open_level;

};