# Instructions for using the BSEC Arduino Library in Arduino 1.8.19

## About BSEC

Bosch Sensortec Environmental Cluster (BSEC) Software v2.2.0.0 released on May 9th, 2022

The BSEC fusion library has been conceptualized to provide a higher-level signal processing and fusion for the BME688. The library receives compensated sensor values from the sensor API. It processes the BME688 signals to provide the requested sensor outputs.

Key features

- Selectivity to target gas classes
- Calculation of index for air quality (IAQ) level outside of the device
- Calculation of ambient air temperature outside of the device (e.g. phone)
- Calculation of ambient relative humidity outside of the device

Typical applications

- Indoor air quality
- Home automation and control
- Internet of things
- Weather forecast
- GPS enhancement (e.g. time-to-first-fix improvement, dead reckoning, slope detection)
- Indoor navigation (change of floor detection, elevator detection)
- Outdoor navigation, leisure and sports applications
- Vertical velocity indication (rise/sink speed)

Supported platforms

- BSEC library is supported on 32, 16 and 8 bit MCU platforms

Available binaries for download:

| Platform | Compiler | ROM (BSEC) | RAM  | TYPE |
|----------|----------|------------|------|------|
| Cortex-ARM | ARMCC | 24.22-25.72k | 4.38k | Cortex-M0, M0+, M3, M4, M4_FPU, M7 |
| Cortex-ARM | GCC | 27.22-49.45k | 4.38k | Cortex-M0, M0+, M3, M4, M4_FPU, M7 |
| Cortex-ARM | IAR | 27.05k-27.73 | 4.38k | Cortex-M0, M0+, M3, M4, M4_FPU, M7 |
| Cortex-A* | GCC | 28.20-33k | 4.38k | Cortex-A7, Cortex-A73 |
| AVR_8bit | AVR-GCC | 56.8-57.8k | 39.61k | MegaAVR, XMEGA |
| AVR_32bit | AVR-GCC | 31.99k | 4.5k | 32-bit AVR UC3 |
| ESP8266 | xtensa-lx106-elf-gcc | 36.24k | 4.39k | ESP8266 |
| ESP32 | xtensa-esp32-elf-gcc | 31.84k | 4.39k | ESP32 |
| MSP430 | msp430-elf-gcc | 45k | 3.97k | MSP430 |
| Android system-x86 | gcc | 46.93-44.72k | 4.48k | x86, x86_64 |
| Android system-arm | gcc | 51.9k | 4.43k | arm, arm64 |
| Raspberry PI 0 linux | arm-linux-gnueabihf-gcc | 75.36k | 4.39k | armv6-32bits |
| Raspberry PI3 linux | arm-linux-gnueabihf-gcc | 75.38k | 4.39k | armv8-a-64bits |

The library size information above doesn't include additional dependencies based on the embedded system project & platform.

For other platforms, please contact your local Bosch Sensortec representative

Advantages

- Easy to integrate
- Hardware and software co-design for optimal performance
- Complete software fusion solution out of one hand
- Eliminates need for own fusion software development
- Robust virtual sensor outputs optimized for the application

## Software license agreement

The BSEC software is only available for download or use after accepting the software license agreement. By using this library, you have agreed to the terms of the license agreement.

[BSEC license agreement](https://www.bosch-sensortec.com/media/boschsensortec/downloads/software/bme688_development_software/bosch-sensortec-clickthrough-license-bme688.pdf)

## Installation and getting started

### 1. Install the latest Arduino IDE

As of this publication, the latest Arduino IDE 1.8.19 can be downloaded from this [link](https://www.arduino.cc/en/software)

### 2. Install the BSEC2 library and BME68x Library

Download [Bosch_BSEC2_Library](https://github.com/BoschSensortec/Bosch-BSEC2-Library) and [Bosch_BME68x_Library](https://github.com/BoschSensortec/Bosch-BME68x-Library) (this library is a dependency to the BSEC2 library) as a zip and import it into the Arduino IDE. Refer to [this](https://www.arduino.cc/en/Guide/Libraries) guide on how to import libraries.

### 3. Install esp32 Board package in the Arduino IDE

- Open File->Preferences->Settings

- Insert the following link into the "Additional Boards Manager URLs":

	https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

	**Note:** Please ensure that the proxy settings are updated under File->Preferences->Network

- Go to Tools->Board->Boards Manager and search for "esp32"

- Install the esp32 package

### 4. Verify and upload the example code

Start or restart the Arduino IDE. Open any of the example codes found under  ```Bosch_BSEC2_Library>examples```.

Select your board and COM port. Upload the example. Open the Serial monitor. You should see an output on the terminal.

### More about example codes:

- basic.ino: This is an example for illustrating the basic BSEC virtual outputs.

- basic_config_state.ino: This is an example for illustrating the BSEC feature using desired configuration setting.

- bme68x_demo_sample.ino: This demonstrator application running on an x8 board has the feature of sensor data logging and BSEC algorithm illustrated. Please refer [BME688 Development Kit-Firmware-Quick-Start-Guide](examples/bme68x_demo_sample/Quick_Start_Guide.md) for installing dependent libraries and how to flash.

### 5. Tested board/core list

The current list of tested boards include,

| Core MCU | Tested boards | Arduino core version | Arduino core repository |
|----------|---------------|----------------------|-------------------------|
| Esp32 | Adafruit ESP32 Feather | v2.0.3 | https://github.com/espressif/arduino-esp32 |
| Esp8266 | Adafruit Feather HUZZAH ESP8266 | v3.0.2 | https://github.com/esp8266/Arduino |

## Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.