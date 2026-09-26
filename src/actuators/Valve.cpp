#include "Valve.h"
#include "hardware/gpio.h"

// Assign GPIO pin number to valve
Valve::Valve(unsigned int pin, bool open_level)
{
    this->pin = pin;
    gpio_init(this->pin);
    this->open_level = open_level; // which signal opens the valve
    close();
    gpio_set_dir(this->pin, GPIO_OUT);

}

void Valve::open() const
{
    gpio_put(this->pin, this->open_level);
}

void Valve::close() const
{
    gpio_put(this->pin, !this->open_level);
}