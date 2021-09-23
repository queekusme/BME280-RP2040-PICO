#pragma once

#include <stdio.h>
#include "hardware/i2c.h"

#include "bme280/bme280.h"

#include "i2c_bus.hpp"

class bme280_device
{
    public:
        bme280_device(i2c_bus bus, int8_t address);
        ~bme280_device();

        int8_t read(bme280_data *comp_data);

    protected:
        i2c_bus bus;
        int8_t address;
        struct bme280_dev dev;

        static int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
        static int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
        static void user_delay_us(uint32_t period, void *intf_ptr);

        struct i2c_intf_ptr {
            i2c_inst_t *i2c;
            int8_t address;
        };

};