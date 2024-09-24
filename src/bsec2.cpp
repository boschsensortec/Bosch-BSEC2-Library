/**
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file	bsec2.cpp
 * @date	18 July 2024
 * @version	2.1.5
 *
 */

#include "bsec2.h"

static uint8_t workBuffer[BSEC_MAX_WORKBUFFER_SIZE];

/**
 * @brief Constructor of Bsec2 class
 */
Bsec2::Bsec2(void)
{
    ovfCounter = 0;
    lastMillis = 0;
    status = BSEC_OK;
	extTempOffset = 0.0f;
    opMode = BME68X_SLEEP_MODE;
    newDataCallback = nullptr;
    bsecInstance = nullptr;

    memset(&version, 0, sizeof(version));
    memset(&bmeConf, 0, sizeof(bmeConf));
    memset(&outputs, 0, sizeof(outputs));
}

/**
 * @brief Function to initialize the sensor based on custom callbacks
 */
bool Bsec2::begin(bme68xIntf intf, bme68x_read_fptr_t read, bme68x_write_fptr_t write,
        bme68x_delay_us_fptr_t idleTask, void *intfPtr)
{
    sensor.begin(intf, read, write, idleTask, intfPtr);

    if (sensor.checkStatus() == BME68X_ERROR)
        return false;

    return beginCommon();
}

/**
 * @brief Function to initialize the sensor based on the Wire library
 */ 
bool Bsec2::begin(uint8_t i2cAddr, TwoWire &i2c, bme68x_delay_us_fptr_t idleTask)
{
    sensor.begin(i2cAddr, i2c, idleTask);

    if (sensor.checkStatus() == BME68X_ERROR)
        return false;

    return beginCommon();
}

/**
 * @brief Function to initialize the sensor based on the SPI library
 */
bool Bsec2::begin(uint8_t chipSelect, SPIClass &spi, bme68x_delay_us_fptr_t idleTask)
{
    sensor.begin(chipSelect, spi, idleTask);

    if (sensor.checkStatus() == BME68X_ERROR)
        return false;

    return beginCommon();
}

/**
 * @brief Function to request/subscribe for desired virtual outputs with the supported sample rates
 */
bool Bsec2::updateSubscription(bsecSensor sensorList[], uint8_t nSensors, float sampleRate)
{
    bsec_sensor_configuration_t virtualSensors[BSEC_NUMBER_OUTPUTS], sensorSettings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t nSensorSettings = BSEC_MAX_PHYSICAL_SENSOR;

    for (uint8_t i = 0; i < nSensors; i++)
    {
        virtualSensors[i].sensor_id = sensorList[i];
        virtualSensors[i].sample_rate = sampleRate;
    }

    /* Subscribe to library virtual sensors outputs */
    status = bsec_update_subscription_m(bsecInstance, virtualSensors, nSensors, sensorSettings, &nSensorSettings);
    if (status != BSEC_OK)
        return false;

    return true;
}

/**
 * @brief Callback from the user to read data from the BME68X using parallel mode/forced mode, process and store outputs
 */
bool Bsec2::run(void)
{
    uint8_t nFieldsLeft = 0;
    bme68xData data;
    int64_t currTimeNs = getTimeMs() * INT64_C(1000000);
    opMode = bmeConf.op_mode;

    if (currTimeNs >= bmeConf.next_call)
    {
        /* Provides the information about the current sensor configuration that is
           necessary to fulfill the input requirements, eg: operation mode, timestamp
           at which the sensor data shall be fetched etc */
        status = bsec_sensor_control_m(bsecInstance ,currTimeNs, &bmeConf);
        if (status != BSEC_OK)
            return false;

        switch (bmeConf.op_mode)
        {
        case BME68X_FORCED_MODE:
            setBme68xConfigForced();
            break;
        case BME68X_PARALLEL_MODE:
            if (opMode != bmeConf.op_mode)
            {
                setBme68xConfigParallel();
            }
            break;

        case BME68X_SLEEP_MODE:
            if (opMode != bmeConf.op_mode)
            {
                sensor.setOpMode(BME68X_SLEEP_MODE);
                opMode = BME68X_SLEEP_MODE;
            }
            break;
        }

        if (sensor.checkStatus() == BME68X_ERROR)
            return false;

        if (bmeConf.trigger_measurement && bmeConf.op_mode != BME68X_SLEEP_MODE)
        {
            if (sensor.fetchData())
            {
                do
                {
                    nFieldsLeft = sensor.getData(data);
                    /* check for valid gas data */
                    if (data.status & BME68X_GASM_VALID_MSK)
                    {
						/* Convert sensor raw pressure unit from pascal to hecto pascal */
						data.pressure *= 0.01f;

                        if (!processData(currTimeNs, data))
                        {
                            return false;
                        }
                    }
                } while (nFieldsLeft);
            }

        }

    }
    return true;
}

/**
 * @brief Function to get the state of the algorithm to save to non-volatile memory
 */
bool Bsec2::getState(uint8_t *state)
{
    uint32_t n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;

    status = bsec_get_state_m(bsecInstance, 0, state, BSEC_MAX_STATE_BLOB_SIZE, workBuffer, BSEC_MAX_WORKBUFFER_SIZE,
            &n_serialized_state);
    if (status != BSEC_OK)
        return false;
    return true;
}

/**
 * @brief Function to set the state of the algorithm from non-volatile memory
 */
bool Bsec2::setState(uint8_t *state)
{
    status = bsec_set_state_m(bsecInstance, state, BSEC_MAX_STATE_BLOB_SIZE, workBuffer, BSEC_MAX_WORKBUFFER_SIZE);
    if (status != BSEC_OK)
        return false;

    memset(&bmeConf, 0, sizeof(bmeConf));

    return true;
}

/**
 * @brief Function to retrieve the current library configuration
 */
bool Bsec2::getConfig(uint8_t *config)
{
    uint32_t n_serialized_settings = 0;
    
    status = bsec_get_configuration_m(bsecInstance, 0, config, BSEC_MAX_PROPERTY_BLOB_SIZE, workBuffer, BSEC_MAX_WORKBUFFER_SIZE, &n_serialized_settings);
    if (status != BSEC_OK)
        return false;

    return true;
}

/**
 * @brief Function to set the configuration of the algorithm from memory
 */
bool Bsec2::setConfig(const uint8_t *config)
{
    status = bsec_set_configuration_m(bsecInstance, config, BSEC_MAX_PROPERTY_BLOB_SIZE, workBuffer, BSEC_MAX_WORKBUFFER_SIZE);
    if (status != BSEC_OK)
        return false;

    memset(&bmeConf, 0, sizeof(bmeConf));

    return true;
}

/**
 * @brief Function to calculate an int64_t timestamp in milliseconds
 */
int64_t Bsec2::getTimeMs(void)
{
    int64_t timeMs = millis();

    if (lastMillis > timeMs) /* An overflow occurred */
    { 
        ovfCounter++;
    }

    lastMillis = timeMs;

    return timeMs + (ovfCounter * INT64_C(0xFFFFFFFF));
}

/**
 * @brief Function to assign the memory block to the bsec instance
 */
void Bsec2::allocateMemory(uint8_t (&memBlock)[BSEC_INSTANCE_SIZE])
{
    /* allocating memory for the bsec instance */
    bsecInstance = memBlock;
}

/**
 * @brief Function to de-allocate the dynamically allocated memory
 */
void Bsec2::clearMemory(void)
{
    delete[] bsecInstance;
}

/* Private functions */

/**
 * @brief Reads data from the BME68X sensor and process it
 */
bool Bsec2::processData(int64_t currTimeNs, const bme68xData &data)
{
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temp, Pres, Hum & Gas */
    uint8_t nInputs = 0;
    /* Checks all the required sensor inputs, required for the BSEC library for the requested outputs */
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_HEATSOURCE))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[nInputs].signal = extTempOffset;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_TEMPERATURE))
    {
#ifdef BME68X_USE_FPU
        inputs[nInputs].signal = data.temperature;
#else
        inputs[nInputs].signal = data.temperature / 100.0f;
#endif
        inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_HUMIDITY))
    {
#ifdef BME68X_USE_FPU
        inputs[nInputs].signal = data.humidity;
#else
        inputs[nInputs].signal = data.humidity / 1000.0f;
#endif
        inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_PRESSURE))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[nInputs].signal = data.pressure;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_GASRESISTOR) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[nInputs].signal = data.gas_resistance;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_PROFILE_PART) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[nInputs].signal = (opMode == BME68X_FORCED_MODE) ? 0 : data.gas_index;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }

    if (nInputs > 0)
    {

        outputs.nOutputs = BSEC_NUMBER_OUTPUTS;
        memset(outputs.output, 0, sizeof(outputs.output));

        /* Processing of the input signals and returning of output samples is performed by bsec_do_steps() */
        status = bsec_do_steps_m(bsecInstance, inputs, nInputs, outputs.output, &outputs.nOutputs);

        if (status != BSEC_OK)
            return false;

        if(newDataCallback)
            newDataCallback(data, outputs, *this);
    }
    return true;
}

/**
 * @brief Common code for the begin function
 */
bool Bsec2::beginCommon()
{
    if (!bsecInstance)
    {
        /* allocate memory for the instance if not allocated */
        bsecInstance = new uint8_t[bsec_get_instance_size_m()];
    }

    if (BSEC_INSTANCE_SIZE < bsec_get_instance_size_m())
    {
        status = BSEC_E_INSUFFICIENT_INSTANCE_SIZE;
        return false;
    }
    status = bsec_init_m(bsecInstance);
    if (status != BSEC_OK)
        return false;

    status = bsec_get_version_m(bsecInstance, &version);
    if (status != BSEC_OK)
        return false;

    memset(&bmeConf, 0, sizeof(bmeConf));
    memset(&outputs, 0, sizeof(outputs));

    return true;
}

/**
 * @brief Set the BME68X sensor configuration to forced mode
 */
void Bsec2::setBme68xConfigForced(void)
{
    /* Set the filter, odr, temperature, pressure and humidity settings */
    sensor.setTPH(bmeConf.temperature_oversampling, bmeConf.pressure_oversampling, bmeConf.humidity_oversampling);

    if (sensor.checkStatus() == BME68X_ERROR)
        return;

    sensor.setHeaterProf(bmeConf.heater_temperature, bmeConf.heater_duration);

    if (sensor.checkStatus() == BME68X_ERROR)
        return;

    sensor.setOpMode(BME68X_FORCED_MODE);
    if (sensor.checkStatus() == BME68X_ERROR)
        return;

    opMode = BME68X_FORCED_MODE;
}

/**
 * @brief Set the BME68X sensor configuration to parallel mode
 */
void Bsec2::setBme68xConfigParallel(void)
{
    uint16_t sharedHeaterDur = 0;

    /* Set the filter, odr, temperature, pressure and humidity settings */
    sensor.setTPH(bmeConf.temperature_oversampling, bmeConf.pressure_oversampling, bmeConf.humidity_oversampling);

    if (sensor.checkStatus() == BME68X_ERROR)
        return;

    sharedHeaterDur = BSEC_TOTAL_HEAT_DUR - (sensor.getMeasDur(BME68X_PARALLEL_MODE) / INT64_C(1000));

    sensor.setHeaterProf(bmeConf.heater_temperature_profile, bmeConf.heater_duration_profile, sharedHeaterDur,
            bmeConf.heater_profile_len);

    if (sensor.checkStatus() == BME68X_ERROR)
        return;

    sensor.setOpMode(BME68X_PARALLEL_MODE);

    if (sensor.checkStatus() == BME68X_ERROR)
        return;

    opMode = BME68X_PARALLEL_MODE;
}
