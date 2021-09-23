#include "bme280_device.hpp"

bme280_device::bme280_device(i2c_bus bus, int8_t address): bus(bus), address(address)
{
    // Constructor
    int8_t rslt = BME280_OK;

    this->dev.intf_ptr = new i2c_intf_ptr{.i2c = bus.getI2CChannel(), .address = address};
    this->dev.intf = BME280_I2C_INTF;
    this->dev.read = bme280_device::user_i2c_read;
    this->dev.write = bme280_device::user_i2c_write;
    this->dev.delay_us = bme280_device::user_delay_us;

    rslt = bme280_init(&(this->dev));

    if(BME280_OK != rslt)
    {
        printf("bme280_init failed with %02d\n", rslt);
        panic("bme280_device constructor failed to initialise...\n");
    }

	uint8_t settings_sel;

	/* Recommended mode of operation: Indoor navigation */
	this->dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	this->dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	this->dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	this->dev.settings.filter = BME280_FILTER_COEFF_16;
	this->dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, &(this->dev));
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &(this->dev));
}

bme280_device::~bme280_device()
{
    // Destructor
}

int8_t bme280_device::read(bme280_data *comp_data)
{
    int8_t rslt = BME280_OK;

    this->dev.delay_us(70, this->dev.intf_ptr);
	rslt = bme280_get_sensor_data(BME280_ALL, comp_data, &(this->dev));
    printf("[INTERNAL] %0.2f*C, %0.2fPa, %0.2f%%\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);

    return rslt;
}

BME280_INTF_RET_TYPE bme280_device::user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
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

BME280_INTF_RET_TYPE bme280_device::user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_intf_ptr* i2c = (i2c_intf_ptr *)intf_ptr;

    int result = i2c_write_blocking(i2c->i2c, i2c->address, &reg_addr, 1, true);
    result = i2c_read_blocking(i2c->i2c, i2c->address, reg_data, length, false);

    return result == PICO_ERROR_GENERIC ? BME280_E_COMM_FAIL : BME280_OK;
};

void bme280_device::user_delay_us(uint32_t period, void *intf_ptr)
{
    sleep_ms(period);
}