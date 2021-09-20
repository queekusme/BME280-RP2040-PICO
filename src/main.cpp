#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "math.h"

#include "bme280/bme280.h"

// ========== TEMPORARY OVERRIDES UNTIL SOLDER IS COMPLETE ==========
#define PICO_DEFAULT_I2C_SDA_PIN 12
#define PICO_DEFAULT_I2C_SCL_PIN 13
// ========== END TEMPORARY OVERRIDES UNTIL SOLDER IS COMPLETE ==========

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);
int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev);
void print_sensor_data(struct bme280_data *comp_data);

struct i2c_intf_ptr {
    i2c_inst_t *i2c;
    int8_t address;
};

int main()
{
    // Initialize chosen serial port
    stdio_init_all();

    // Wait for us to connect to serial
    sleep_ms(10000);

    i2c_init(i2c_default, 100 * 1000);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    struct bme280_dev dev;
    int8_t rslt = BME280_OK;

    dev.intf_ptr = new i2c_intf_ptr{.i2c = i2c_default, .address = BME280_I2C_ADDR_PRIM};
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;

    rslt = bme280_init(&dev);

    if(BME280_OK != rslt)
    {
        panic("bme280_init failed with %d\n", rslt);
    }

    stream_sensor_data_normal_mode(&dev);

    return 0;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_intf_ptr* i2c = (i2c_intf_ptr *)intf_ptr;

    uint8_t buffer[length + 1];
    buffer[0] = reg_addr;
    for(int x = 0; x < length; x++) {
        buffer[x + 1] = reg_data[x];
    }

    int result = i2c_write_blocking(i2c->i2c, i2c->address, buffer, length + 1, false);

    return result == PICO_ERROR_GENERIC ? BME280_E_COMM_FAIL : BME280_OK;
};

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_intf_ptr* i2c = (i2c_intf_ptr *)intf_ptr;

    int result = i2c_write_blocking(i2c->i2c, i2c->address, &reg_addr, 1, true);
    result = i2c_read_blocking(i2c->i2c, i2c->address, reg_data, length, false);

    return result == PICO_ERROR_GENERIC ? BME280_E_COMM_FAIL : BME280_OK;
};

void user_delay_us(uint32_t period, void *intf_ptr)
{
    sleep_ms(period);
}

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

	printf("Temperature, Pressure, Humidity\r\n");
	while (1) {
		/* Delay while the sensor completes a measurement */
		dev->delay_us(70, dev->intf_ptr);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
		print_sensor_data(&comp_data);
	}

	return rslt;
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        printf("%0.2f*C, %0.2fPa, %0.2f%%\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        printf("%ld*C, %ldPa, %ld%\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}
