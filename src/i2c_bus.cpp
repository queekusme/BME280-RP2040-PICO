#include "i2c_bus.hpp"
#include "hardware/i2c.h"

i2c_bus::i2c_bus(i2c_inst_t *i2c, uint baudrate, uint sda, uint scl): i2c(i2c), sda_pin(sda), scl_pin(scl)
{
    // Constructor
    i2c_init(i2c, baudrate);

    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);

    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

i2c_bus::~i2c_bus()
{
    // Destructor
    gpio_disable_pulls(this->scl_pin);
    gpio_disable_pulls(this->sda_pin);

    gpio_set_function(this->scl_pin, GPIO_FUNC_NULL);
    gpio_set_function(this->sda_pin, GPIO_FUNC_NULL);

    i2c_deinit(this->i2c);
}

i2c_inst_t * i2c_bus::getI2CChannel()
{
    return this->i2c;
}