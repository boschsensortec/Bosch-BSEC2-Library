/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/* If compiling this examples leads to an 'undefined reference error', refer to the README 
 * at https://github.com/BoschSensortec/Bosch-BSEC2-Library
 */
/* The new sensor needs to be conditioned before the example can work reliably. You may run this
 * example for 24hrs to let the sensor stabilize.
 */

/**
 * basic.ino sketch :
 * This is an example for illustrating the BSEC virtual outputs and
 * which has been designed to work with Adafruit ESP8266 Board
 */
 
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "I2Cbus.hpp"

#define BME688_ADD 0x77

static const char* TAG = "main";

#include <bsec2.h>

/* Macros used */
#define PANIC_LED   GPIO_NUM_5
#define ERROR_DUR   1000

#define SAMPLE_RATE		BSEC_SAMPLE_RATE_LP

/* Helper functions declarations */
/**
 * @brief : This function toggles the led when a fault was detected
 */
void errLeds(void);

/**
 * @brief : This function checks the BSEC status, prints the respective error code. Halts in case of error
 * @param[in] bsec  : Bsec2 class object
 */
void checkBsecStatus(Bsec2 bsec);

/**
 * @brief : This function is called by the BSEC library when a new output is available
 * @param[in] input     : BME68X sensor data before processing
 * @param[in] outputs   : Processed BSEC BSEC output data
 * @param[in] bsec      : Instance of BSEC2 calling the callback
 */
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

/* Create an object of the class Bsec2 */
Bsec2 envSensor;

/* Function wrappers */
int8_t read_bytes_wrapper(uint8_t a_register, uint8_t *data, uint32_t len, void *intfPtr) {
  return static_cast<I2C_t *>(intfPtr)->readBytes(BME688_ADD, a_register, len, data)==ESP_OK  ? 0 : -1;
}

int8_t write_bytes_wrapper(uint8_t a_register, const uint8_t *data, uint32_t len,
                                                    void *intfPtr) {
  return static_cast<I2C_t *>(intfPtr)->writeBytes(BME688_ADD, a_register, len, data)==ESP_OK ? 0 : -1;
}

uint32_t IRAM_ATTR millis() { return (uint32_t) (esp_timer_get_time() / 1000ULL); }
void IRAM_ATTR delay(uint32_t ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }
uint32_t IRAM_ATTR micros() { return (uint32_t) esp_timer_get_time(); }

void delay_microseconds_safe(uint32_t us) {  // avoids CPU locks that could trigger WDT or affect WiFi/BT stability
  uint32_t start = micros();

  const uint32_t lag = 5000;  // microseconds, specifies the maximum time for a CPU busy-loop.
                              // it must be larger than the worst-case duration of a delay(1) call (hardware tasks)
                              // 5ms is conservative, it could be reduced when exact BT/WiFi stack delays are known
  if (us > lag) {
    delay((us - lag) / 1000UL);  // note: in disabled-interrupt contexts delay() won't actually sleep
    while (micros() - start < us - lag)
      delay(1);  // in those cases, this loop allows to yield for BT/WiFi stack tasks
  }
  while (micros() - start < us)  // fine delay the remaining usecs
    ;
}

void delay_us(uint32_t period, void *intfPtr) {
  delay_microseconds_safe(period);
}

/* Entry point for the example */
void setup(void)
{
    /* Desired subscription list of BSEC2 outputs */
    bsecSensor sensorList[] = {
            BSEC_OUTPUT_IAQ,
            BSEC_OUTPUT_RAW_TEMPERATURE,
            BSEC_OUTPUT_RAW_PRESSURE,
            BSEC_OUTPUT_RAW_HUMIDITY,
            BSEC_OUTPUT_RAW_GAS,
            BSEC_OUTPUT_STABILIZATION_STATUS,
            BSEC_OUTPUT_RUN_IN_STATUS,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
            BSEC_OUTPUT_STATIC_IAQ,
            BSEC_OUTPUT_CO2_EQUIVALENT,
            BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
            BSEC_OUTPUT_GAS_PERCENTAGE,
            BSEC_OUTPUT_COMPENSATED_GAS
    };

    /* Initialize the communication interfaces */
    i2c0.begin(GPIO_NUM_3,GPIO_NUM_0);
    gpio_set_direction(PANIC_LED, GPIO_MODE_OUTPUT);


    /* Initialize the library and interfaces */
    if (!envSensor.begin(BME68X_I2C_INTF, read_bytes_wrapper, write_bytes_wrapper, delay_us, (void *) &i2c0, millis))
    {
        checkBsecStatus(envSensor);
    }
	
	/*
	 *	The default offset provided has been determined by testing the sensor in LP and ULP mode on application board 3.0
	 *	Please update the offset value after testing this on your product 
	 */
	if (SAMPLE_RATE == BSEC_SAMPLE_RATE_ULP)
	{
		envSensor.setTemperatureOffset(TEMP_OFFSET_ULP);
	}
	else if (SAMPLE_RATE == BSEC_SAMPLE_RATE_LP)
	{
		envSensor.setTemperatureOffset(TEMP_OFFSET_LP);
	}

    /* Subsribe to the desired BSEC2 outputs */
    if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), SAMPLE_RATE))
    {
        checkBsecStatus(envSensor);
    }

    /* Whenever new data is available call the newDataCallback function */
    envSensor.attachCallback(newDataCallback);

    ESP_LOGI(TAG, "BSEC library version %u.%u.%u.%u", envSensor.version.major, envSensor.version.minor, envSensor.version.major_bugfix, envSensor.version.minor_bugfix);
}

/* Function that is looped forever */
void loop(void)
{
    /* Call the run function often so that the library can 
     * check if it is time to read new data from the sensor  
     * and process it.
     */
    if (!envSensor.run())
    {
        checkBsecStatus(envSensor);
    }
}

void errLeds(void)
{
    while(1)
    {
        gpio_set_level(PANIC_LED, 1);
        vTaskDelay(ERROR_DUR / portTICK_PERIOD_MS);
        gpio_set_level(PANIC_LED, 0);
        vTaskDelay(ERROR_DUR / portTICK_PERIOD_MS);
    }
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
    {
        return;
    }

    ESP_LOGI(TAG, "BSEC outputs:\n\tTime stamp = %d", (int) (outputs.output[0].time_stamp / INT64_C(1000000)));
    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output  = outputs.output[i];
        switch (output.sensor_id)
        {
            case BSEC_OUTPUT_IAQ:
                ESP_LOGI(TAG, "\tIAQ = %f", output.signal);
                ESP_LOGI(TAG, "\tIAQ accuracy = %d", (int) output.accuracy);
                break;
            case BSEC_OUTPUT_RAW_TEMPERATURE:
                ESP_LOGI(TAG, "\tTemperature = %f", output.signal);
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                ESP_LOGI(TAG, "\tPressure = %f", output.signal);
                break;
            case BSEC_OUTPUT_RAW_HUMIDITY:
                ESP_LOGI(TAG, "\tHumidity = %f", output.signal);
                break;
            case BSEC_OUTPUT_RAW_GAS:
                ESP_LOGI(TAG, "\tGas resistance = %f", output.signal);
                break;
            case BSEC_OUTPUT_STABILIZATION_STATUS:
                ESP_LOGI(TAG, "\tStabilization status = %f", output.signal);
                break;
            case BSEC_OUTPUT_RUN_IN_STATUS:
                ESP_LOGI(TAG, "\tRun in status = %f", output.signal);
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                ESP_LOGI(TAG, "\tCompensated temperature = %f", output.signal);
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                ESP_LOGI(TAG, "\tCompensated humidity = %f", output.signal);
                break;
            case BSEC_OUTPUT_STATIC_IAQ:
                ESP_LOGI(TAG, "\tStatic IAQ = %f", output.signal);
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                ESP_LOGI(TAG, "\tCO2 Equivalent = %f", output.signal);
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                ESP_LOGI(TAG, "\tbVOC equivalent = %f", output.signal);
                break;
            case BSEC_OUTPUT_GAS_PERCENTAGE:
                ESP_LOGI(TAG, "\tGas percentage = %f", output.signal);
                break;
            case BSEC_OUTPUT_COMPENSATED_GAS:
                ESP_LOGI(TAG, "\tCompensated gas = %f", output.signal);
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
        ESP_LOGI(TAG, "BSEC error code : %d", bsec.status);
        errLeds(); /* Halt in case of failure */
    }
    else if (bsec.status > BSEC_OK)
    {
        ESP_LOGI(TAG, "BSEC warning code : %d", bsec.status);
    }

    if (bsec.sensor.status < BME68X_OK)
    {
        ESP_LOGI(TAG, "BME68X error code : %d", bsec.sensor.status);
        errLeds(); /* Halt in case of failure */
    }
    else if (bsec.sensor.status > BME68X_OK)
    {
        ESP_LOGI(TAG, "BME68X warning code : %d", bsec.sensor.status);
    }
}

void loop_task(void *pv_params) {
  setup();
  while (true) {
    loop();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

extern "C" void app_main()
{
	ESP_LOGI(TAG, "starting");
	xTaskCreate(loop_task, "loopTask", 8192, nullptr, 1, NULL);
}
