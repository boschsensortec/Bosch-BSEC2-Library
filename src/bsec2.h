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
 * @file	bsec2.h
 * @date	18 July 2024
 * @version	2.1.5
 *
 */

#ifndef BSEC2_H_
#define BSEC2_H_

/* Includes */
#ifdef ARDUINO
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#endif

/* dependent library header */
#include "bme68xLibrary.h"
#include "inc/bsec_datatypes.h"
#include "inc/bsec_interface_multi.h"

#ifndef ARRAY_LEN
#define ARRAY_LEN(array)				(sizeof(array)/sizeof(array[0]))
#endif

#define BSEC_CHECK_INPUT(x, shift)		    (x & (1 << (shift-1)))
#define BSEC_TOTAL_HEAT_DUR                 UINT16_C(140)
#define BSEC_INSTANCE_SIZE                  3272
#define BSEC_E_INSUFFICIENT_INSTANCE_SIZE   (bsec_library_return_t)-105

/*
 *	The default offset provided has been determined by testing the sensor in LP and ULP mode on application board 3.0
 *	Please update the offset value after testing this on your product 
 */
#define TEMP_OFFSET_LP		(1.3255f)
#define TEMP_OFFSET_ULP		(0.466f)

typedef bsec_output_t bsecData;
typedef bsec_virtual_sensor_t bsecSensor;

typedef struct
{
    bsecData output[BSEC_NUMBER_OUTPUTS];
    uint8_t nOutputs;
} bsecOutputs;

class Bsec2;
typedef void (*bsecCallback)(const bme68xData data, const bsecOutputs outputs, const Bsec2 bsec);

/* BSEC2 class definition */
class Bsec2
{
public:
    Bme68x sensor;
    /* Stores the version of the BSEC algorithm */
    bsec_version_t version;
    bsec_library_return_t status;

    Bsec2(void);

    /**
     * @brief Function to initialize the sensor based on custom callbacks
     * @param intf     : BME68X_SPI_INTF or BME68X_I2C_INTF interface
     * @param read     : Read callback
     * @param write    : Write callback
     * @param idleTask : Delay or Idle function
     * @param millis : Function to get current time in milliseconds
     * @param intfPtr : Pointer to the interface descriptor
     * @return True if everything initialized correctly
     */
    bool begin(bme68xIntf intf, bme68x_read_fptr_t read, bme68x_write_fptr_t write,
            bme68x_delay_us_fptr_t idleTask, void *intfPtr, unsigned long (*millis)());

#ifdef ARDUINO
    /**
     * @brief Function to initialize the sensor based on custom callbacks
     * @param intf     : BME68X_SPI_INTF or BME68X_I2C_INTF interface
     * @param read     : Read callback
     * @param write    : Write callback
     * @param idleTask : Delay or Idle function
     * @param intfPtr : Pointer to the interface descriptor
     * @return True if everything initialized correctly
     */
    bool begin(bme68xIntf intf, bme68x_read_fptr_t read, bme68x_write_fptr_t write,
            bme68x_delay_us_fptr_t idleTask, void *intfPtr);

    /**
     * @brief Function to initialize the sensor based on the Wire library
     * @param i2cAddr  : The I2C address the sensor is at
     * @param i2c      : The TwoWire object
     * @param idleTask : Delay or Idle function
     * @return True if everything initialized correctly
     */
    bool begin(uint8_t i2cAddr, TwoWire &i2c, bme68x_delay_us_fptr_t idleTask = bme68xDelayUs);

    /**
     * @brief Function to initialize the sensor based on the SPI library
     * @param chipSelect : The chip select pin for SPI communication
     * @param spi        : The SPIClass object
     * @param idleTask   : Delay or Idle function
     * @return True if everything initialized correctly
     */
    bool begin(uint8_t chipSelect, SPIClass &spi, bme68x_delay_us_fptr_t idleTask = bme68xDelayUs);
#endif

    /**
     * @brief Function that sets the desired sensors and the sample rates
     * @param sensorList	: The list of output sensors
     * @param nSensors		: Number of outputs requested
     * @param sampleRate	: The sample rate of requested sensors
     * @return	true for success, false otherwise
     */
    bool updateSubscription(bsecSensor sensorList[], uint8_t nSensors, float sampleRate =
    BSEC_SAMPLE_RATE_ULP);

    /**
     * @brief Callback from the user to read data from the BME68x using parallel/forced mode, process and store outputs
     * @return	true for success, false otherwise
     */
    bool run(void);

    void attachCallback(bsecCallback callback)
    {
        newDataCallback = callback;
    }

    /**
     * @brief Function to get the BSEC outputs
     * @return	pointer to BSEC outputs if available else nullptr
     */
    const bsecOutputs* getOutputs(void)
    {
        if (outputs.nOutputs)
            return &outputs;
        return nullptr;
    }

    /**
     * @brief Function to get the BSEC output by sensor id
     * @return	pointer to BSEC output, nullptr otherwise
     */
    bsecData getData(bsecSensor id)
    {
        bsecData emp;
        memset(&emp, 0, sizeof(emp));
        for (uint8_t i = 0; i < outputs.nOutputs; i++)
            if (id == outputs.output[i].sensor_id)
                return outputs.output[i];
        return emp;
    }

    /**
     * @brief Function to get the state of the algorithm to save to non-volatile memory
     * @param state			: Pointer to a memory location, to hold the state
     * @return	true for success, false otherwise
     */
    bool getState(uint8_t *state);

    /**
     * @brief Function to set the state of the algorithm from non-volatile memory
     * @param state			: Pointer to a memory location that contains the state
     * @return	true for success, false otherwise
     */
    bool setState(uint8_t *state);

    /** 
     * @brief Function to retrieve the current library configuration
     * @param config    : Pointer to a memory location, to hold the serialized config blob
     * @return	true for success, false otherwise
     */
    bool getConfig(uint8_t *config);

    /**
     * @brief Function to set the configuration of the algorithm from memory
     * @param state			: Pointer to a memory location that contains the configuration
     * @return	true for success, false otherwise
     */
    bool setConfig(const uint8_t *config);

    /**
     * @brief Function to set the temperature offset
     * @param tempOffset	: Temperature offset in degree Celsius
     */
    void setTemperatureOffset(float tempOffset)
    {
        extTempOffset = tempOffset;
    }

    /**
     * @brief Function to calculate an int64_t timestamp in milliseconds
     */
    int64_t getTimeMs(void);

    /**
     * @brief Function to assign the memory block to the bsec instance
     * 
     * @param[in] memBlock : reference to the memory block
     */
    void allocateMemory(uint8_t (&memBlock)[BSEC_INSTANCE_SIZE]);

    /**
     * @brief Function to de-allocate the dynamically allocated memory
     */
    void clearMemory(void);

private:
    bsec_bme_settings_t bmeConf;

    bsecCallback newDataCallback;

    bsecOutputs outputs;
    /* operating mode of sensor */
    uint8_t opMode;

    float extTempOffset;
    /** Global variables to help create a millisecond timestamp that doesn't overflow every 51 days.
     * If it overflows, it will have a negative value. Something that should never happen.
     */
    uint32_t ovfCounter;
    
    unsigned long (*bsecMillis)();

    uint32_t lastMillis;
    /* Pointer to hold the address of the instance */
    uint8_t *bsecInstance;

    /**
     * @brief Reads the data from the BME68x sensor and process it
     * @param currTimeNs: Current time in ns
     * @return true if there are new outputs. false otherwise
     */
    bool processData(int64_t currTimeNs, const bme68xData &data);

    /**
     * @brief Common code for the begin function
     */
    bool beginCommon();

    /**
     * @brief Set the BME68x sensor configuration to forced mode
     */
    void setBme68xConfigForced(void);

    /**
     * @brief Set the BME68x sensor configuration to parallel mode
     */
    void setBme68xConfigParallel(void);
};

#endif /* BSEC2_CLASS_H */
