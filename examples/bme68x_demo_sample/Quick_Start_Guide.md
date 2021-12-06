# BME688 Development Kit-Firmware-Quick-Start-Guide

If not followed README.md, please refer Installation and getting started section under [README.md](../../README.md).

1. Install the following Arduino libraries under Sketch->Include Library->Manage Libraries
	
	- ArduinoJson (tested for 6.17.3 by Benoit Blanchon)

	- RTClib (tested for 1.12.5 by Adafruit)
	
    	- Click OK, when asked to install needed dependencies
    
	- SdFat (tested for 1.1.4 by Bill Greiman)
	
2. Download the Arduino library "Bosch_BME68x_Library" from [here](https://github.com/BoschSensortec/Bosch-BME68x-Library)
   
   - This library is a dependency for using the BSEC library
		  
3. Open the 'bme68x_demo_sample.ino' file from the path - Bosch_BSEC2_Library/examples/bme68x_demo_sample

4. Connect your board via USB
	
	- Select "Adafruit ESP32 Feather" in the Arduino IDE under Tools->Board
    
	- Select your COM Port under Tools-Port
	
5. Upload the firmware
	
	- If you have a coin cell(CR 1220) for the RTC, make sure it is inserted before flashing the board
	
	- When the board is flashed, don't unmount the coin cell, if not necessary
	
6. Wait for the board to be flashed and then disconnect it
	
	- Last line in the Arduino command line should be: "Hard resetting via RTS pin..."
	
7. To start the data logging, insert an SD-card with a '.bmeconfig file'(generated out of BME AI studio) in it and connect power supply
  
8. A red LED blinking with 1Hz indicates a successful data logging, a faster blinking indicates an error during the initialization (RTC, SD or Bluetooth)

## Additional Note:

'Bosch_BSEC2_Library' has 'src' folder with the following files essential for running application-

- header files - bsec_datatypes.h and bsec_interface.h (under folder 'inc')

- esp32 library - static library (under 'esp32')

### More about Demo Application Code (Bme68x_demo_sample.ino Sketch):

This is an example code for datalogging and integration of BSEC2x library in BME688 development kit,
which has been designed to work with Adafruit ESP32 Feather Board.

The example code will primarily operate in two modes-

**a. DEMO_DATALOGGER_MODE**

**b. DEMO_BLE_STREAMING_MODE**
 
- Once successfully flashing the firmware, "DEMO_DATALOGGER_MODE" is the default mode set in the example.
 
- An initial check for the availability of sensor board configuration file (.bmeconfig) inside SD card
is performed.
	  
- For DEMO_DATALOGGER_MODE, once .bmeconfig file availability has been confirmed, it will initialize 
  and set the heater profiles for all the bme688 sensors with the provided configuration and create 
  bme68x datalogger	output file (with '.bmerawdata' extension).

- Until the user connects the board to the BME688 demo application through ble, it will continuously collect
  the data in DEMO_DATALOGGER_MODE.

- When it comes to 'DEMO_BLE_STREAMING_MODE' (connecting over bluetooth), initialization of sensor and BSEC library is undertaken.

- Further, configuration with the BSEC config file (generated out of training through BME AI Studio or a default configuration
  from the BSEC website release package) and subscription for desired virtual outputs with the supported sample rate is done,
  Output data file is created with '.bsecdata' extension.

- Until bluetooth communication is disabled, it will continuously operate in 'DEMO_BLE_STREAMING_MODE' and collect the
  sensor data from one of the sensors in the board for processing and stores the outputs in the output datalog file.

- Once bluetooth communication stops, operating mode 'DEMO_DATALOGGER_MODE' is activated, where the data is logged into a new 
  output file (.bmerawdata).
