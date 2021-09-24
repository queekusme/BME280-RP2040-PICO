#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "math.h"

#include "bme280/bme280.h"
#include "i2c_bus.hpp"
#include "bme280_device.hpp"

void print_sensor_data(struct bme280_data *comp_data);

// Make the I2C pins available to picotool
bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

int main()
{
    // Initialize chosen serial port
    stdio_init_all();

    // Wait for us to connect to serial
    sleep_ms(10000);

    printf("Initialising...\n");

    i2c_bus bus(i2c0, 100 * 1000, PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);
    printf("* i2c initialised\n");
    bme280_device bme280(bus, BME280_I2C_ADDR_PRIM);
    printf("* bme280 initialised\n");

    printf("Initialisation complete!\n");

    while(true)
    {
        struct bme280_data comp_data;
        printf("pre-read\n");
        int8_t rslt = bme280.read(&comp_data);
        printf("post-read\n");

        if(BME280_OK != rslt)
        {
            printf("bme280_get_sensor_data failed with %02d\n", rslt);
            panic("bme280_device read failed to pull data...\n");
        }

        print_sensor_data(&comp_data);
    }

    return 0;
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        printf("%0.2f*C, %0.2fPa, %0.2f%%\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        printf("%ld*C, %ldPa, %ld%\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}
