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
 * @date		03 Jan 2024
 * @version		2.1.5
 * 
 * @brief    	sensor manager
 *
 * 
 */

/* own header include */
#include "sensor_manager.h"

bme68x_sensor 	sensorManager::_sensors[NUM_BME68X_UNITS];
comm_mux comm_setup[NUM_BME68X_UNITS];

/*!
 * @brief The constructor of the sensorManager class
 */
sensorManager::sensorManager()
{}

/*!
 * @brief This function initializes the given BME688 sensor
 */
int8_t sensorManager::initialize_sensor(uint8_t sensor_number, uint32_t& sensor_id)
{
	int8_t bme68x_rslt;
	
	bme68xSensors[sensor_number].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, 
	                                  &comm_setup[sensor_number]);
	bme68x_rslt = bme68xSensors[sensor_number].status;

	if (bme68x_rslt != BME68X_OK)
	{
		return bme68x_rslt;
	}
	sensor_id = bme68xSensors[sensor_number].getUniqueId();
	bme68x_rslt = bme68xSensors[sensor_number].status;
	return bme68x_rslt;
}

/*!
 * @brief This function configures the heater settings of the sensor
 */
int8_t sensorManager::set_heater_profile(const String& heater_profile_str, const String& duty_cycle_str,
                                       bme68x_heater_profile& heater_profile, uint8_t sensor_number)
{
	/* get heater profiles from parsed object */
	JsonArray heaterProfilesJson = _configDoc["configBody"]["heaterProfiles"].as<JsonArray>();

	/* iterate over all heater profiles */
	for (JsonVariant _heaterProfileJson: heaterProfilesJson)
	{

		/* compare with profile of given sensor */
		if (heater_profile_str == _heaterProfileJson["id"].as<String>())
		{
			/* on match, save heater temperature and duration vectors */
			heater_profile.length = _heaterProfileJson["temperatureTimeVectors"].size();

			for (uint32_t i = 0; i < heater_profile.length; i++)
			{
				heater_profile.temperature[i] = _heaterProfileJson["temperatureTimeVectors"][i][0].as<uint16_t>();
				heater_profile.duration[i] = _heaterProfileJson["temperatureTimeVectors"][i][1].as<uint16_t>();
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
		if (duty_cycle_str == _dutyCycleProfileJson["id"].as<String>())
		{
			/* on match, save duty cycle information to the sensor profile */
			heater_profile.nb_repetitions = _dutyCycleProfileJson["numberScanningCycles"].as<uint8_t>();
			heater_profile.sleep_duration = _dutyCycleProfileJson["numberSleepingCycles"].as<uint8_t>();

			uint64_t sleep_duration = 0;

			for (uint16_t dur : heater_profile.duration)
			{
				sleep_duration += (uint64_t)dur * HEATER_TIME_BASE;
			}
			heater_profile.sleep_duration *= sleep_duration;

			break;
		}
	}
	return configure_sensor(heater_profile, sensor_number);
}

/*!
 * @brief This function configures the bme688 sensor
 */
int8_t sensorManager::configure_sensor(bme68x_heater_profile& heater_profile, uint8_t sensor_number)
{
	bme68xSensors[sensor_number].setTPH();
	int8_t bme68x_rslt = bme68xSensors[sensor_number].status;

	if (bme68x_rslt != BME68X_OK)
	{
		return bme68x_rslt;
	}

	/* getMeasDur() returns Measurement duration in micro sec. to convert to milli sec. '/ INT64_C(1000)' */
	uint32_t shared_heatr_dur = HEATER_TIME_BASE - 
	                            (bme68xSensors[sensor_number].getMeasDur(BME68X_PARALLEL_MODE) / INT64_C(1000));
	/* sets the heater configuration of the sensor */
	bme68xSensors[sensor_number].setHeaterProf(heater_profile.temperature, heater_profile.duration,
	                                          shared_heatr_dur, heater_profile.length);
	return bme68xSensors[sensor_number].status;
}

/*!
 * @brief This function initializes all bme688 sensors
 */
demo_ret_code sensorManager::initialize_all_sensors()
{
	int8_t bme68x_rslt = BME68X_OK;
	comm_mux_begin(Wire, SPI);
	
	for (uint8_t i = 0; i < NUM_BME68X_UNITS; i++)
	{
		bme68x_sensor* sensor = get_sensor(i);
		/* Communication interface set for all the 8 sensors */
		comm_setup[i] = comm_mux_set_config(Wire, SPI, i, comm_setup[i]);

		if (sensor != nullptr)
		{
			sensor->i2c_mask = ((0x01 << i) ^ 0xFF);//TODO
			bme68x_rslt = initialize_sensor(i, sensor->id);

			if (bme68x_rslt != BME68X_OK)
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
demo_ret_code sensorManager::begin(const String& config_name)
{
	int8_t bme68xRslt = BME68X_OK;
	
	comm_mux_begin(Wire, SPI);

	/* Communication interface set for all the 8 sensors in the development kit */
	for (uint8_t i = 0; i < NUM_BME68X_UNITS; i++)
	{
		comm_setup[i] = comm_mux_set_config(Wire, SPI, i, comm_setup[i]);
	}

	/* open config file */
	File configFile;

	if (configFile.open(config_name.c_str(), O_READ))
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
		uint8_t sensor_number = devicefig["sensorIndex"].as<uint8_t>();
		bme68x_sensor* sensor = get_sensor(sensor_number);

		if (sensor == nullptr)
		{
			return EDK_SENSOR_MANAGER_SENSOR_INDEX_ERROR;
		}
	  
		String heater_profile_str = devicefig["heaterProfile"].as<String>();
		String duty_cycle_str = devicefig["dutyCycleProfile"].as<String>();
		
		sensor->is_configured = false;
		sensor->wake_up_time = 0;
		sensor->mode = BME68X_SLEEP_MODE;
		sensor->cycle_pos = 0;
		sensor->next_gas_index = 0;
		sensor->i2c_mask = ((0x01 << sensor_number) ^ 0xFF);
		sensor->scan_cycle_index = 1;
		
		/* initialize the sensor */
		bme68xRslt = initialize_sensor(sensor_number, sensor->id);

		if (bme68xRslt != BME68X_OK)
		{
			return EDK_BME68X_DRIVER_ERROR;
		}		
	  
		/* set the heater profile */
		bme68xRslt = set_heater_profile(heater_profile_str, duty_cycle_str, sensor->heater_profile, sensor_number);

		if (bme68xRslt != BME68X_OK)
		{
			return EDK_BME68X_DRIVER_ERROR;
		}	
		
		sensor->is_configured = true;
	}
	return EDK_OK;
}

/*!
 * @brief This function retrieves the selected sensor data
 */
demo_ret_code sensorManager::collect_data(uint8_t num, bme68x_data* data[3])
{
	demo_ret_code ret_code = EDK_OK;
	int8_t bme68x_rslt = BME68X_OK;
	
	data[0] = data[1] = data[2] = nullptr;
	bme68x_sensor* sensor = get_sensor(num);

	if (sensor == nullptr)
	{
		return EDK_SENSOR_MANAGER_SENSOR_INDEX_ERROR;
	}
	
	uint64_t time_stamp = utils::get_tick_ms();

	if (sensor->is_configured && (time_stamp >= sensor->wake_up_time))
	{

		/* Wake up the sensor if necessary */
		if (sensor->mode == BME68X_SLEEP_MODE)
		{
			sensor->mode = BME68X_PARALLEL_MODE;
			bme68xSensors[num].setOpMode(BME68X_PARALLEL_MODE);
			bme68x_rslt = bme68xSensors[num].status;
			sensor->next_gas_index = 0;
			sensor->wake_up_time = time_stamp + GAS_WAIT_SHARED;
		}
		else
		{
			uint8_t nFields, j = 0;
			
			nFields = bme68xSensors[num].fetchData();
			bme68x_data *sensor_data = bme68xSensors[num].getAllData();

			for (uint8_t k = 0; k < 3; k++)
			{
				_field_data[k] = sensor_data[k];
			}

			for (uint8_t i = 0; i < nFields; i++)
			{

				if (_field_data[i].status & BME68X_GASM_VALID_MSK)
				{
					uint8_t delta_index = _field_data[i].gas_index - sensor->next_gas_index;

					if (delta_index > sensor->heater_profile.length)
					{
						continue;
					}
					else if (delta_index > 0)
					{
						ret_code = EDK_SENSOR_MANAGER_DATA_MISS_WARNING;
					}

					data[j++] = &_field_data[i];
					
					sensor->next_gas_index = _field_data[i].gas_index + 1;

					if (sensor->next_gas_index == sensor->heater_profile.length)
					{
						sensor->next_gas_index = 0;

						if (++sensor->cycle_pos >= sensor->heater_profile.nb_repetitions)
						{
							sensor->cycle_pos = 0;
							sensor->mode = BME68X_SLEEP_MODE;
							sensor->wake_up_time = utils::get_tick_ms() + sensor->heater_profile.sleep_duration;
							bme68xSensors[num].setOpMode(BME68X_SLEEP_MODE);
							bme68x_rslt = bme68xSensors[num].status;
							break;
						}
					}
					sensor->wake_up_time = time_stamp + GAS_WAIT_SHARED;
				}
			}
			
			if (data[0] == nullptr)
			{
				sensor->wake_up_time = time_stamp + GAS_WAIT_SHARED;
			}
		}
		
		if (bme68x_rslt < BME68X_OK)
		{
			ret_code = EDK_BME68X_DRIVER_ERROR;
		}
	}
	return ret_code;
}
