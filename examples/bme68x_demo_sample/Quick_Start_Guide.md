# BME688 Development Kit-Firmware-Quick-Start-Guide

## Pre-requisites

Follow the Installation and getting started in the [README.md](../../README.md).

## Instructions to build the **bme68x_demo_sample**

1. Install the following Arduino libraries under Sketch->Include Library->Manage Libraries

	- ArduinoJson (tested for 6.19.4 by Benoit Blanchon)

	- RTClib (tested for 2.0.3 by Adafruit)

    	- Click OK, when asked to install needed dependencies

	- SdFat (tested for 2.1.2 by Bill Greiman)

2. Open the 'bme68x_demo_sample.ino' file from the path - Bosch_BSEC2_Library/examples/bme68x_demo_sample

3. Connect your board via USB

	- Select "Adafruit ESP32 Feather" in the Arduino IDE under Tools->Board

	- Select your COM Port under Tools->Port

5. Upload the firmware

	- If you have a coin cell (CR1220) for the RTC, make sure it is inserted before flashing the board

	- When the board is flashed, don't unmount the coin cell, if not necessary

5. Wait for the board to be flashed and then disconnect it

	- Last line in the Arduino command line should be: "Hard resetting via RTS pin..."

6. To start the data logging, insert an SD-card with a '.bmeconfig file' in it (generated using BME AI studio) and connect power supply
  
7. A red LED blinking with 1Hz indicates a successful data logging, a faster blinking indicates an error during the initialization (RTC, SD or Bluetooth)

## Additional Note

The 'Bosch_BSEC2_Library' has a 'src' folder with the following files essential for running application-

- header files - bsec_datatypes.h and bsec_interface.h (under folder 'inc')

- esp32 library - static library (under 'esp32')

## More about Demo Application Code (Bme68x_demo_sample.ino Sketch):

This is an example code for datalogging and integration of BSEC2x library in BME688 development kit,
which has been designed to work with Adafruit ESP32 Feather Board.

The example code will primarily operate in two modes-

**a. DEMO_DATALOGGER_MODE**

**b. DEMO_BLE_STREAMING_MODE**

### Note:

- After successfully flashing the firmware, "DEMO_DATALOGGER_MODE" is the default mode set in the example.

- An initial check for the availability of sensor board configuration file (.bmeconfig) inside SD card
is performed.

- For DEMO_DATALOGGER_MODE, once .bmeconfig file has been detected, it will initialize and set the heater profiles for all the bme688 sensors with the provided configuration and create bme68x datalogger output file (with '.bmerawdata' extension).

- Until the user connects the board to the BME688 demo application through Bluetooth LE, it will continuously collect the data in DEMO_DATALOGGER_MODE.

- When it comes to 'DEMO_BLE_STREAMING_MODE' (connecting over bluetooth), initialization of sensor and BSEC library is undertaken.

- Furthermore, configuring with the BSEC config file (generated out of training through BME AI Studio or a default configuration from the BSEC website release package) and subscribing for the desired virtual outputs with the supported sample rate is complete, an output data file is created with the '.bsecdata' extension.

- While the bluetooth connection is active, it will continue to operate in the 'DEMO_BLE_STREAMING_MODE' and collect the sensor data from one of the sensors in the board for processing and stores the outputs in the output datalog file.

- Once bluetooth is disconnected, the device switches to the 'DEMO_DATALOGGER_MODE', where the data is logged into a new  output file (.bmerawdata).
