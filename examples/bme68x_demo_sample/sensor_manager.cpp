/*!
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
 * @file	    sensor_manager.cpp
 * @date	    22 June 2022
 * @version		1.5.5
 * 
 * @brief    	sensor manager
 *
 * 
 */

/* own header include */
#include "sensor_manager.h"

bme68xSensor 	sensorManager::_sensors[NUM_BME68X_UNITS];
commMux commSetup[NUM_BME68X_UNITS];

/*!
 * @brief The constructor of the sensorManager class
 */
sensorManager::sensorManager()
{}

/*!
 * @brief This function initializes the given BME688 sensor
 */
int8_t sensorManager::initializeSensor(uint8_t sensorNumber, uint32_t& sensorId)
{
    int8_t bme68xRslt;
	
	bme68xSensors[sensorNumber].begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &commSetup[sensorNumber]);
	bme68xRslt = bme68xSensors[sensorNumber].status;
	if (bme68xRslt != BME68X_OK)
	{
		return bme68xRslt;
	}
    sensorId = bme68xSensors[sensorNumber].getUniqueId();
	bme68xRslt = bme68xSensors[sensorNumber].status;
    return bme68xRslt;
}

/*!
 * @brief This function configures the heater settings of the sensor
 */
int8_t sensorManager::setHeaterProfile(const String& heaterProfileStr, const String& dutyCycleStr, bme68xHeaterProfile& heaterProfile, uint8_t sensorNumber)
{
    /* get heater profiles from parsed object */
    JsonArray heaterProfilesJson = _configDoc["configBody"]["heaterProfiles"].as<JsonArray>();
    /* iterate over all heater profiles */
    for (JsonVariant _heaterProfileJson: heaterProfilesJson)
    {
        /* compare with profile of given sensor */
        if (heaterProfileStr == _heaterProfileJson["id"].as<String>())
        {
            /* on match, save heater temperature and duration vectors */
            heaterProfile.length = _heaterProfileJson["temperatureTimeVectors"].size();
            for (int i = 0; i < heaterProfile.length; i++)
            {
                heaterProfile.temperature[i] = _heaterProfileJson["temperatureTimeVectors"][i][0].as<uint16_t>();
                heaterProfile.duration[i] = _heaterProfileJson["temperatureTimeVectors"][i][1].as<uint16_t>();
            }
            break;
        }
    }
	
	/* get duty cycle profiles from parsed object */
    JsonArray dutyCycleProfilesJson = _configDoc["configBody"]["dutyCycleProfiles"].as<JsonArray>();
    /* iterate over all duty cycle profiles */
    for (JsonVariant _dutyCycleProfileJson : dutyCycleProfilesJson)
    {
        /* compare with profile of the given sensor */
        if (dutyCycleStr == _dutyCycleProfileJson["id"].as<String>())
        {
            /* on match, save duty cycle information to the sensor profile */
            heaterProfile.nbRepetitions = _dutyCycleProfileJson["numberScanningCycles"].as<uint8_t>();
            heaterProfile.sleepDuration = _dutyCycleProfileJson["numberSleepingCycles"].as<uint8_t>();
			
			uint64_t sleepDuration = 0;
			for (uint16_t dur : heaterProfile.duration)
			{
				sleepDuration += (uint64_t)dur * HEATER_TIME_BASE;
			}
			heaterProfile.sleepDuration *= sleepDuration;
			
            break;
        }
    }
	return configureSensor(heaterProfile, sensorNumber);
}

/*!
 * @brief This function configures the bme688 sensor
 */
int8_t sensorManager::configureSensor(bme68xHeaterProfile& heaterProfile, uint8_t sensorNumber)
{
    bme68xSensors[sensorNumber].setTPH();
	int8_t bme68xRslt = bme68xSensors[sensorNumber].status;
	if (bme68xRslt != BME68X_OK)
	{
		return bme68xRslt;
	}

	/* getMeasDur() returns Measurement duration in micro sec. to convert to milli sec. '/ INT64_C(1000)' */
	uint32_t shared_heatr_dur = HEATER_TIME_BASE - (bme68xSensors[sensorNumber].getMeasDur(BME68X_PARALLEL_MODE) / INT64_C(1000));
	/* sets the heater configuration of the sensor */
	bme68xSensors[sensorNumber].setHeaterProf(heaterProfile.temperature, heaterProfile.duration, shared_heatr_dur, heaterProfile.length);
	return bme68xSensors[sensorNumber].status;
}

/*!
 * @brief This function initializes all bme688 sensors
 */
demoRetCode sensorManager::initializeAllSensors()
{
	int8_t bme68xRslt = BME68X_OK;
	commMuxBegin(Wire, SPI);
	
	for (uint8_t i = 0; i < NUM_BME68X_UNITS; i++)
	{
		bme68xSensor* sensor = getSensor(i);
		/* Communication interface set for all the 8 sensors */
		commSetup[i] = commMuxSetConfig(Wire, SPI, i, commSetup[i]);
		if (sensor != nullptr)
		{
			sensor->i2cMask = ((0x01 << i) ^ 0xFF);//TODO
			bme68xRslt = initializeSensor(i, sensor->id);
			if (bme68xRslt != BME68X_OK)
			{
				return EDK_BME68X_DRIVER_ERROR;
			}
		}
	}
	return EDK_OK;
}

/*!
 * @brief This function configures the sensor manager using the provided config file
 */
demoRetCode sensorManager::begin(const String& configName)
{
	int8_t bme68xRslt = BME68X_OK;
	
	commMuxBegin(Wire, SPI);
	/* Communication interface set for all the 8 sensors in the development kit */
	for (uint8_t i = 0; i < NUM_BME68X_UNITS; i++)
	{
		commSetup[i] = commMuxSetConfig(Wire, SPI, i, commSetup[i]);
	}

	/* open config file */
	File configFile;
    if (configFile.open(configName.c_str(), O_READ))
    {
        /* read in configuration and parse to JSON object */
        DeserializationError error = deserializeJson(_configDoc, configFile);
        /* close config file */
        configFile.close();
		if (error) 
        {
            Serial.println(error.c_str());
			return EDK_SENSOR_MANAGER_JSON_DESERIAL_ERROR;
        }
    }
    else
    {
		return EDK_SENSOR_MANAGER_CONFIG_FILE_ERROR;
    }
	
	memset(_sensors, 0, sizeof(_sensors));

    JsonArray devicefigurations = _configDoc["configBody"]["sensorConfigurations"].as<JsonArray>();
    for (JsonVariant devicefig : devicefigurations)
    {
        /* save config information to sensor profile */
        uint8_t sensorNumber = devicefig["sensorIndex"].as<uint8_t>();

		bme68xSensor* sensor = getSensor(sensorNumber);
		if(sensor == nullptr)
		{
			return EDK_SENSOR_MANAGER_SENSOR_INDEX_ERROR;
		}

        String heaterProfileStr = devicefig["heaterProfile"].as<String>();
        String dutyCycleStr = devicefig["dutyCycleProfile"].as<String>();
        
		sensor->isConfigured = false;
		sensor->wakeUpTime = 0;
		sensor->mode = BME68X_SLEEP_MODE;
		sensor->cyclePos = 0;
		sensor->nextGasIndex = 0;
        sensor->i2cMask = ((0x01 << sensorNumber) ^ 0xFF);
		
        /* initialize the sensor */
        bme68xRslt = initializeSensor(sensorNumber, sensor->id);
		if (bme68xRslt != BME68X_OK)
		{
			return EDK_BME68X_DRIVER_ERROR;
		}		

        /* set the heater profile */
        bme68xRslt = setHeaterProfile(heaterProfileStr, dutyCycleStr, sensor->heaterProfile, sensorNumber);
		if (bme68xRslt != BME68X_OK)
		{
			return EDK_BME68X_DRIVER_ERROR;
		}	
		
		sensor->isConfigured = true;
    }
	return EDK_OK;
}

/*!
 * @brief This function retrieves the selected sensor data
 */
demoRetCode sensorManager::collectData(uint8_t num, bme68x_data* data[3])
{
	demoRetCode retCode = EDK_OK;
	int8_t bme68xRslt = BME68X_OK;
	
	data[0] = data[1] = data[2] = nullptr;
	
	bme68xSensor* sensor = getSensor(num);
	if(sensor == nullptr)
	{
		return EDK_SENSOR_MANAGER_SENSOR_INDEX_ERROR;
	}
	
	uint64_t timeStamp = utils::getTickMs();
	if (sensor->isConfigured && (timeStamp >= sensor->wakeUpTime))
	{
		/* Wake up the sensor if necessary */
		if (sensor->mode == BME68X_SLEEP_MODE)
		{
			sensor->mode = BME68X_PARALLEL_MODE;
			bme68xSensors[num].setOpMode(BME68X_PARALLEL_MODE);
			bme68xRslt = bme68xSensors[num].status;
			sensor->nextGasIndex = 0;
			sensor->wakeUpTime = timeStamp + GAS_WAIT_SHARED;
		}
		else
		{
			uint8_t nFields, j = 0;
			
			nFields = bme68xSensors[num].fetchData();
			bme68x_data *sensorData = bme68xSensors[num].getAllData();
			for (int k = 0; k < 3; k++)
			{
				_fieldData[k] = sensorData[k];
			}
			for (uint8_t i = 0; i < nFields; i++)
			{
				if (_fieldData[i].status & BME68X_GASM_VALID_MSK)
				{
					uint8_t deltaIndex = _fieldData[i].gas_index - sensor->nextGasIndex;
					if (deltaIndex > sensor->heaterProfile.length)
					{
						continue;
					}
					else if (deltaIndex > 0)
					{
						retCode = EDK_SENSOR_MANAGER_DATA_MISS_WARNING;
					}

					data[j++] = &_fieldData[i];
					
					sensor->nextGasIndex = _fieldData[i].gas_index + 1;
					if (sensor->nextGasIndex == sensor->heaterProfile.length)
					{
						sensor->nextGasIndex = 0;
						if (++sensor->cyclePos >= sensor->heaterProfile.nbRepetitions)
						{
							sensor->cyclePos = 0; 
							sensor->mode = BME68X_SLEEP_MODE;
							sensor->wakeUpTime = utils::getTickMs() + sensor->heaterProfile.sleepDuration;
							bme68xSensors[num].setOpMode(BME68X_SLEEP_MODE);
							bme68xRslt = bme68xSensors[num].status;
							break;
						}
					}
					sensor->wakeUpTime = timeStamp + GAS_WAIT_SHARED;
				}
			}
			
			if (data[0] == nullptr)
			{
				sensor->wakeUpTime = timeStamp + GAS_WAIT_SHARED;
			}
		}
		
		if (bme68xRslt < BME68X_OK)
		{
			retCode = EDK_BME68X_DRIVER_ERROR;
		}
	}
	return retCode;
}

