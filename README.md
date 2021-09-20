# BME280-RP2040-PICO
My implementation of the BME280 temperature, barometric pressure and humidity sensor for the Raspberry Pi PICO.

This app assumes that you have checked out the PICO SDK and the root of the SDK is setup under the "PICO_SDK_PATH" environment variable and have the required cmake/compilers required to build a C/C++ Pico Project

## Build
Run the following commands in the root of the checkout to build the app
```bash
mkdir build
cd build
cmake ..
make
```