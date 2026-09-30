#include "Valve.h"
#include "hardware/gpio.h"

// constructor
// Assign GPIO pin number to valve
Valve::Valve(unsigned int pin, bool openLevel)
{
    this->pin = pin;
    gpio_init(this->pin);
    this->openLevel = openLevel; // which signal opens the valve
    close();
    gpio_set_dir(this->pin, GPIO_OUT);

}

void Valve::open() const
{
    gpio_put(this->pin, this->openLevel);
}

void Valve::close() const
{
    gpio_put(this->pin, !this->openLevel);
}