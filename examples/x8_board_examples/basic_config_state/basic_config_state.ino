/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

/* The new sensor needs to be conditioned before the example can work reliably. You may run this
 * example for 24hrs to let the sensor stabilize.
 */

/**
 * basic_config_state.ino sketch :
 * This is an example for integration of BSEC2x library in BME688 development kit,
 * which has been designed to work with Adafruit ESP32 Feather Board
 * For more information visit : 
 * https://www.bosch-sensortec.com/software-tools/software/bme688-software/
 * 
 * For quick integration test, example code can be used with configuration file under folder
 * Bosch_BSEC2_Library/src/config/FieldAir_HandSanitizer (Configuration file added as simple
 * code example for integration but not optimized on classification performance)
 * Config string for H2S and NonH2S target classes is also kept for reference (Suitable for
 * lab-based characterization of the sensor)
 */

#include <EEPROM.h>
#include <bsec2.h>
#include <commMux\commMux.h>
/* For two class classification use configuration under config/FieldAir_HandSanitizer */
#define CLASSIFICATION	1
#define REGRESSION		  2

#define COMPLETED       1

/* Note : 
          For the classification output from BSEC algorithm set OUTPUT_MODE macro to CLASSIFICATION.
          For the regression output from BSEC algorithm set OUTPUT_MODE macro to REGRESSION.
*/
#define OUTPUT_MODE   CLASSIFICATION

#if (OUTPUT_MODE == CLASSIFICATION)
  const uint8_t bsec_config[] = {
                                  #include "config/FieldAir_HandSanitizer/bsec_selectivity.txt"
                                };
#elif (OUTPUT_MODE == REGRESSION)
  const uint8_t bsec_config[] = {
                                  #include "config/bme688/bme688_reg_18v_300s_4d/bsec_selectivity.txt"
                                };
#endif

/* Macros used */
#define STATE_SAVE_PERIOD   UINT32_C(360 * 60 * 1000) /* 360 minutes - 4 times a day */
/* Number of sensors to operate*/
#define NUM_OF_SENS    8
#define PANIC_LED	LED_BUILTIN
#define ERROR_DUR   1000

/* Helper functions declarations */
/**
 * @brief : This function toggles the led continuously with one second delay
 */
void errLeds(void);

/**
 * @brief : This function checks the BSEC status, prints the respective error code. Halts in case of error
 * @param[in] bsec  : Bsec2 class object
 */
void checkBsecStatus(Bsec2 bsec);

/**
 * @brief : This function updates/saves BSEC state
 * @param[in] bsec  : Bsec2 class object
 */
void updateBsecState(Bsec2 bsec);

/**
 * @brief : This function is called by the BSEC library when a new output is available
 * @param[in] input     : BME68X sensor data before processing
 * @param[in] outputs   : Processed BSEC BSEC output data
 * @param[in] bsec      : Instance of BSEC2 calling the callback
 */
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

/**
 * @brief : This function retrieves the existing state
 * @param : Bsec2 class object
 */
bool loadState(Bsec2 bsec);

/**
 * @brief : This function writes the state into EEPROM
 * @param : Bsec2 class object
 */
bool saveState(Bsec2 bsec);

/* Create an object of the class Bsec2 */
Bsec2 envSensor[NUM_OF_SENS];
comm_mux commConfig[NUM_OF_SENS];
uint8_t bsecMemBlock[NUM_OF_SENS][BSEC_INSTANCE_SIZE];
uint8_t sensor = 0;
static uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
/* Gas estimate names will be according to the configuration classes used */
const String gasName[] = { "Field Air", "Hand sanitizer", "Undefined 3", "Undefined 4"};

/* Entry point for the example */
void setup(void)
{
	
#if (OUTPUT_MODE == CLASSIFICATION)
	/* Desired subscription list of BSEC2 Classification outputs */
    bsecSensor sensorList[] = {
            BSEC_OUTPUT_RAW_TEMPERATURE,
            BSEC_OUTPUT_RAW_PRESSURE,
            BSEC_OUTPUT_RAW_HUMIDITY,
            BSEC_OUTPUT_RAW_GAS,
            BSEC_OUTPUT_RAW_GAS_INDEX,
            BSEC_OUTPUT_GAS_ESTIMATE_1,
            BSEC_OUTPUT_GAS_ESTIMATE_2,
            BSEC_OUTPUT_GAS_ESTIMATE_3,
            BSEC_OUTPUT_GAS_ESTIMATE_4
    };
#elif (OUTPUT_MODE == REGRESSION)
	/* Desired subscription list of BSEC2 Regression outputs */
	bsecSensor sensorList[] = {
            BSEC_OUTPUT_RAW_TEMPERATURE,
            BSEC_OUTPUT_RAW_PRESSURE,
            BSEC_OUTPUT_RAW_HUMIDITY,
            BSEC_OUTPUT_RAW_GAS,
            BSEC_OUTPUT_RAW_GAS_INDEX,
            BSEC_OUTPUT_REGRESSION_ESTIMATE_1,
            BSEC_OUTPUT_REGRESSION_ESTIMATE_2,
            BSEC_OUTPUT_REGRESSION_ESTIMATE_3,
            BSEC_OUTPUT_REGRESSION_ESTIMATE_4
    };
#endif

    Serial.begin(115200);
    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);
    /* Initiate SPI communication */
    comm_mux_begin(Wire, SPI);
    pinMode(PANIC_LED, OUTPUT);
	  delay(100);
	  /* Valid for boards with USB-COM. Wait until the port is open */
    while (!Serial) delay(10);

    uint8_t state_write_otp = 0;
    
    for (uint8_t i = 0; i < NUM_OF_SENS; i++)
    {
        /* Sets the Communication interface for the given sensor */
        commConfig[i] = comm_mux_set_config(Wire, SPI, i, commConfig[i]);
    
         /* Assigning a chunk of memory block to the bsecInstance */
         envSensor[i].allocateMemory(bsecMemBlock[i]);
   
        /* Initialize the library and interfaces */
        if (!envSensor[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &commConfig[i]))
        {
            checkBsecStatus (envSensor[i]);
        }

        /* Load the configuration string that stores information on how to classify the detected gas */
        if (!envSensor[i].setConfig(bsec_config))
        {
        	checkBsecStatus (envSensor[i]);
        }

        /* Copy state from the EEPROM to the algorithm */
        if (state_write_otp == 0) 
        {
          if (!loadState(envSensor[i]))
          {
              checkBsecStatus (envSensor[i]);
          }
          state_write_otp = COMPLETED;
        }

        /* Subscribe for the desired BSEC2 outputs */
        if (!envSensor[i].updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_SCAN))
        {
            checkBsecStatus (envSensor[i]);
        }

        /* Whenever new data is available call the newDataCallback function */
        envSensor[i].attachCallback(newDataCallback);

        updateBsecState(envSensor[i]);
    }

    Serial.println("\nBSEC library version " + \
            String(envSensor[0].version.major) + "." \
            + String(envSensor[0].version.minor) + "." \
            + String(envSensor[0].version.major_bugfix) + "." \
            + String(envSensor[0].version.minor_bugfix));
}

/* Function that is looped forever */
void loop(void)
{
    /* Call the run function often so that the library can
     * check if it is time to read new data from the sensor
     * and process it.
     */
    for (sensor = 0; sensor < NUM_OF_SENS; sensor++)
    {
        if (!envSensor[sensor].run())
        {
         checkBsecStatus(envSensor[sensor]);
        }
    }
}

void errLeds(void)
{
    while(1)
    {
        digitalWrite(PANIC_LED, HIGH);
        delay(ERROR_DUR);
        digitalWrite(PANIC_LED, LOW);
        delay(ERROR_DUR);
    }
}

void updateBsecState(Bsec2 bsec)
{
    static uint16_t stateUpdateCounter = 0;
    bool update = false;

    if (!stateUpdateCounter || (stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
        /* Update every STATE_SAVE_PERIOD minutes */
        update = true;
        stateUpdateCounter++;
    }

    if (update && !saveState(bsec))
        checkBsecStatus(bsec);
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
        return;

    Serial.println("BSEC outputs:\n\tSensor num = " + String(sensor));
    Serial.println("\tTime stamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));

	  int index = 0;

    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output  = outputs.output[i];
        switch (output.sensor_id)
        {
            case BSEC_OUTPUT_RAW_TEMPERATURE:
                Serial.println("\tTemperature = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                Serial.println("\tPressure = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_HUMIDITY:
                Serial.println("\tHumidity = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_GAS:
                Serial.println("\tGas resistance = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_GAS_INDEX:
                Serial.println("\tGas index = " + String(output.signal));
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                Serial.println("\tCompensated temperature = " + String(output.signal));
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                Serial.println("\tCompensated humidity = " + String(output.signal));
                break;
            case BSEC_OUTPUT_GAS_ESTIMATE_1:
            case BSEC_OUTPUT_GAS_ESTIMATE_2:
            case BSEC_OUTPUT_GAS_ESTIMATE_3:
            case BSEC_OUTPUT_GAS_ESTIMATE_4:
                index = (output.sensor_id - BSEC_OUTPUT_GAS_ESTIMATE_1);
                if (index == 0) // The four classes are updated from BSEC with same accuracy, thus printing is done just once.
                {
                  Serial.println("\tAccuracy = " + String((int) output.accuracy));
                }
                Serial.println(("\tClass " + String(index + 1) + " probability = ") + String(output.signal * 100) + "%");
                break;
            case BSEC_OUTPUT_REGRESSION_ESTIMATE_1:
            case BSEC_OUTPUT_REGRESSION_ESTIMATE_2:
            case BSEC_OUTPUT_REGRESSION_ESTIMATE_3:
            case BSEC_OUTPUT_REGRESSION_ESTIMATE_4:                
                index = (output.sensor_id - BSEC_OUTPUT_REGRESSION_ESTIMATE_1);
                if (index == 0) // The four targets are updated from BSEC with same accuracy, thus printing is done just once.
                {
                  Serial.println("\tAccuracy = " + String(output.accuracy));
                }
                Serial.println("\tTarget " + String(index + 1) + " = " + String(output.signal * 100));
                break;
            default:
                break;
        }
    }

}

void checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK)
    {
        Serial.println("BSEC error code : " + String(bsec.status));
        errLeds(); /* Halt in case of failure */
    } else if (bsec.status > BSEC_OK)
    {
        Serial.println("BSEC warning code : " + String(bsec.status));
    }

    if (bsec.sensor.status < BME68X_OK)
    {
        Serial.println("BME68X error code : " + String(bsec.sensor.status));
        errLeds(); /* Halt in case of failure */
    } else if (bsec.sensor.status > BME68X_OK)
    {
        Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}

bool loadState(Bsec2 bsec)
{

    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
    {
        /* Existing state in EEPROM */
        Serial.println("Reading state from EEPROM");
        Serial.print("State file: ");
        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
        {
            bsecState[i] = EEPROM.read(i + 1);
            Serial.print(String(bsecState[i], HEX) + ", ");
        }
        Serial.println();

        if (!bsec.setState(bsecState))
            return false;
    } else
    {
        /* Erase the EEPROM with zeroes */
        Serial.println("Erasing EEPROM");

        for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
            EEPROM.write(i, 0);

        EEPROM.commit();
    }
    return true;
}

bool saveState(Bsec2 bsec)
{
    if (!bsec.getState(bsecState))
        return false;

    Serial.println("Writing state to EEPROM");
    Serial.print("State file: ");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
        EEPROM.write(i + 1, bsecState[i]);
        Serial.print(String(bsecState[i], HEX) + ", ");
    }
    Serial.println();

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();

    return true;
}
