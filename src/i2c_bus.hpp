#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class i2c_bus
{
    public:
        i2c_bus(i2c_inst_t *i2c, uint baudrate, uint sda, uint scl);
        ~i2c_bus();

        i2c_inst_t * getI2CChannel();

    protected:
        i2c_inst_t *i2c;
        uint scl_pin;
        uint sda_pin;
};