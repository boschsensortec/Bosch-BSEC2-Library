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
 * @file	    bsec_datalogger.cpp
 * @date		03 Jan 2024
 * @version		2.1.5
 * 
 * @brief    	bsec_datalogger
 *
 * 
 */

/* own header include */
#include "bsec_datalogger.h"
#include <math.h>
#include <Esp.h>

StaticJsonDocument<80> 		filter1;
StaticJsonDocument<1024> 	doc;
StaticJsonDocument<70> 		filter2;
StaticJsonDocument<512> 	label_doc;
StaticJsonDocument<2048> 	config;

/*!
 * @brief The constructor of the bsec_datalogger class 
 */
bsecDataLogger::bsecDataLogger() : _bsec_file_counter(1)
{}

/*!
 * @brief This function configures the bsec datalogger using the provided bsec config string file
 */
demo_ret_code bsecDataLogger::begin(const String& config_name, const bsec_version_t& bsec_version, uint8_t sensor_num)
{
	demo_ret_code ret_code = utils::begin();
	
	_ai_config_name = config_name;
	_version = bsec_version;

	if (ret_code >= EDK_OK)
	{
		ret_code = create_bsec_file();

		if (ret_code >= EDK_OK)
		{
			ret_code = create_raw_data_file(sensor_num);

			if (ret_code >= EDK_OK)
			{
				ret_code = create_label_info_file();
			}
		}
	}
	return ret_code;
}
 
/*!
 * @brief This function creates a bsec output file
 */
demo_ret_code bsecDataLogger::create_bsec_file()
{
	demo_ret_code ret_code = EDK_OK;
	String mac_str = utils::get_mac_address();	
	String ai_file_base_name = "_Board_" + mac_str + "_PowerOnOff_1_";
	
	_ai_file_name = utils::get_date_time() + ai_file_base_name + utils::get_file_seed() + "_File_" + 
														String(_bsec_file_counter) + AI_DATA_FILE_EXT;
	
	File configFile, logFile;

	if (_ai_config_name.length() && !configFile.open(_ai_config_name.c_str(), O_RDWR))
	{
		ret_code = EDK_DATALOGGER_AI_CONFIG_FILE_ERROR;
	}
	else if (!logFile.open(_ai_file_name.c_str(), O_RDWR | O_CREAT))
	{
		ret_code = EDK_DATALOGGER_LOG_FILE_ERROR;
	}
	else 
	{

		if (_ai_config_name.length())
		{
			String line_buffer;

			/* read in each line from the config file and copy it to the log file */
			while (configFile.available())
			{
				line_buffer = configFile.readStringUntil('\n');

				/* skip the last closing curly bracket of the JSON document */
				if (line_buffer == "}")
				{
					logFile.println("\t,");
					break;
				}
				logFile.println(line_buffer);
			}
			configFile.close();
		}
		else
		{
			logFile.println("{");
		}
		logFile.println("\t\"aiPredictionsDataHeader\": {");
		logFile.println("\t\t\"counterPowerOnOff\": 1,");
		logFile.println("\t\t\"seedPowerOnOff\": \"" + utils::get_file_seed() + "\",");
		logFile.println("\t\t\"counterFileLimit\": " + String(_bsec_file_counter) + ",");
		logFile.println("\t\t\"dateCreated\": \"" + String(utils::get_rtc().now().unixtime()) + "\",");
		logFile.println("\t\t\"dateCreated_ISO\": \"" + utils::get_rtc().now().timestamp() + "+00:00\",");
		logFile.println("\t\t\"firmwareVersion\": \"" + String(FIRMWARE_VERSION) + "\",");
		logFile.println("\t\t\"bsecVersion\": \"" + String(_version.major) + "." + String(_version.minor) + \
							 "." + String(_version.major_bugfix) + "." + String(_version.minor_bugfix) + "\",");
		logFile.println("\t\t\"boardId\": \"" + mac_str + "\"");
		logFile.println("\t},");
		logFile.println("\t\"aiPredictionsDataBody\": {");
		logFile.println("\t\t\"dataColumns\": [");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Sensor Index\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"sensor_index\",");
		logFile.println("\t\t\t\t\"colId\": 1");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Sensor ID\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"sensor_id\",");
		logFile.println("\t\t\t\t\"colId\": 2");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Start Time Since PowerOn\",");
		logFile.println("\t\t\t\t\"unit\": \"Milliseconds\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"start_time_since_poweron\",");
		logFile.println("\t\t\t\t\"colId\": 3");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"End Time Since PowerOn\",");
		logFile.println("\t\t\t\t\"unit\": \"Milliseconds\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"end_time_since_poweron\",");
		logFile.println("\t\t\t\t\"colId\": 4");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Class/Target 1 prediction\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"classtarget_1_prediction\",");
		logFile.println("\t\t\t\t\"colId\": 5");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Class/Target 2 prediction\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"classtarget_2_prediction\",");
		logFile.println("\t\t\t\t\"colId\": 6");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Class/Target 3 prediction\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"classtarget_3_prediction\",");
		logFile.println("\t\t\t\t\"colId\": 7");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Class/Target 4 prediction\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"classtarget_4_prediction\",");
		logFile.println("\t\t\t\t\"colId\": 8");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Prediction accuracy\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"prediction_accuracy\",");
		logFile.println("\t\t\t\t\"colId\": 9");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"IAQ\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"iaq\",");
		logFile.println("\t\t\t\t\"colId\": 10");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"IAQ accuracy\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"iaq_accuracy\",");
		logFile.println("\t\t\t\t\"colId\": 11");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Error code\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"error_code\",");
		logFile.println("\t\t\t\t\"colId\": 12");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Ground truth\",");
		logFile.println("\t\t\t\t\"unit\": \"classId\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"ground_truth\",");
		logFile.println("\t\t\t\t\"colId\": 13");
		logFile.println("\t\t\t}");
		logFile.println("\t\t],");
		
		/* data block */
		logFile.println("\t\t\"dataBlock\": [");
		/* save position in file, where to write the first data set */
		_ai_data_pos = logFile.position();
		logFile.println("\t\t]");
		logFile.println("\t}");
		logFile.println("}");
		
		/* close log file */
		logFile.close();
		
		_first_line = true;
	}
	return ret_code;
}

/*!
 * @brief This function creates a bme68x datalogger output file
 */
demo_ret_code bsecDataLogger::create_raw_data_file(uint8_t sensor_num)
{
	demo_ret_code ret_code = EDK_OK;
	String mac_str = utils::get_mac_address();	
	String bme_file_base_name = "_Board_" + mac_str + "_PowerOnOff_1_";
	
	_bme_file_name = utils::get_date_time() + bme_file_base_name + utils::get_file_seed() + "_File_" + 
														String(_bsec_file_counter) + BME68X_RAWDATA_FILE_EXT;
	
	File logFile;

	if (!logFile.open(_bme_file_name.c_str(), O_RDWR | O_CREAT))
	{
		ret_code = EDK_DATALOGGER_LOG_FILE_ERROR;
	}
	else 
	{
		ret_code = prepare_config_content(sensor_num);

		if (ret_code != EDK_OK)
		{
			return ret_code;
		}

		/* Writes the config header and body to the logfile */
		logFile.print(config_string);
		/* write data header / skeleton */
		/* raw data header */
		logFile.println("\t,\n\t\"rawDataHeader\": {");
		logFile.println("\t\t\"counterPowerOnOff\": 1,");
		logFile.println("\t\t\"seedPowerOnOff\": \"" + utils::get_file_seed() + "\",");
		logFile.println("\t\t\"counterFileLimit\": " + String(_bsec_file_counter) + ",");
		logFile.println("\t\t\"dateCreated\": \"" + String(utils::get_rtc().now().unixtime()) + "\",");
		logFile.println("\t\t\"dateCreated_ISO\": \"" + utils::get_rtc().now().timestamp() + "+00:00\",");
		logFile.println("\t\t\"firmwareVersion\": \"" + String(FIRMWARE_VERSION) + "\",");
		logFile.println("\t\t\"boardId\": \"" + mac_str + "\"");
		logFile.println("\t},");
		logFile.println("\t\"rawDataBody\": {");
		logFile.println("\t\t\"dataColumns\": [");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Sensor Index\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"sensor_index\",");
		logFile.println("\t\t\t\t\"colId\": 1");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Sensor ID\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"sensor_id\",");
		logFile.println("\t\t\t\t\"colId\": 2");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Time Since PowerOn\",");
		logFile.println("\t\t\t\t\"unit\": \"Milliseconds\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"timestamp_since_poweron\",");
		logFile.println("\t\t\t\t\"colId\": 3");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Real time clock\",");
		logFile.println("\t\t\t\t\"unit\": \"Unix Timestamp: seconds since Jan 01 1970. (UTC); 0 = missing\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"real_time_clock\",");
		logFile.println("\t\t\t\t\"colId\": 4");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Temperature\",");
		logFile.println("\t\t\t\t\"unit\": \"DegreesCelcius\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"temperature\",");
		logFile.println("\t\t\t\t\"colId\": 5");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Pressure\",");
		logFile.println("\t\t\t\t\"unit\": \"Hectopascals\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"pressure\",");
		logFile.println("\t\t\t\t\"colId\": 6");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Relative Humidity\",");
		logFile.println("\t\t\t\t\"unit\": \"Percent\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"relative_humidity\",");
		logFile.println("\t\t\t\t\"colId\": 7");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Resistance Gassensor\",");
		logFile.println("\t\t\t\t\"unit\": \"Ohms\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"resistance_gassensor\",");
		logFile.println("\t\t\t\t\"colId\": 8");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Heater Profile Step Index\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"heater_profile_step_index\",");
		logFile.println("\t\t\t\t\"colId\": 9");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Scanning Mode Enabled\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"boolean\",");
		logFile.println("\t\t\t\t\"key\": \"scanning_enabled\",");
		logFile.println("\t\t\t\t\"colId\": 10");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Scanning Cycle Index\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"scanning_cycle_index\",");
		logFile.println("\t\t\t\t\"colId\": 11");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Label Tag\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"label_tag\",");
		logFile.println("\t\t\t\t\"colId\": 12");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Error Code\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"error_code\",");
		logFile.println("\t\t\t\t\"colId\": 13");
		logFile.println("\t\t\t}");
		logFile.println("\t\t],");
		
		/* data block */
		logFile.println("\t\t\"dataBlock\": [");
		/* save position in file, where to write the first data set */
		_bme_data_pos = logFile.position();
		logFile.println("\t\t]");
		logFile.println("\t}");
		logFile.println("}");

		_end_of_line = false;
		/* close log file */
		logFile.close();
	}
	return ret_code;
}

demo_ret_code bsecDataLogger::prepare_config_content(uint8_t sensor_num)
{
	demo_ret_code ret_code = EDK_OK;
	filter1.clear();
	doc.clear();
	config.clear();
	config_string = "\0";

	File configFile;

	if (_ai_config_name.length() && !configFile.open(_ai_config_name.c_str(), O_RDWR))
	{
		return	EDK_DATALOGGER_AI_CONFIG_FILE_ERROR;
	}

	filter1["aiConfigHeader"]["appVersion"] = true;
	JsonObject aiConfigBody = filter1.createNestedObject("aiConfigBody");
	aiConfigBody["heaterProfile"] = true;
	aiConfigBody["dutyCycleProfile"] = true;

	DeserializationError error = deserializeJson(doc, configFile, DeserializationOption::Filter(filter1));

	if (error)
	{
		Serial.println(error.c_str());
		return EDK_SENSOR_MANAGER_JSON_DESERIAL_ERROR;
	}

	/* parcing configuration details from aiConfig file in to local buffers */
	const char* app_version = doc["aiConfigHeader"]["appVersion"];

	JsonObject heaterProfile = doc["aiConfigBody"]["heaterProfile"];
	const char* heater_profile_id = heaterProfile["id"];
	int32_t heater_profile_time_base = heaterProfile["timeBase"];

	JsonArray tempTimeVector = heaterProfile["temperatureTimeVectors"];

	int32_t temp_time_vector_0_0 = tempTimeVector[0][0];
	int32_t temp_time_vector_0_1 = tempTimeVector[0][1];

	int32_t temp_time_vector_1_0 = tempTimeVector[1][0];
	int32_t temp_time_vector_1_1 = tempTimeVector[1][1];

	int32_t temp_time_vector_2_0 = tempTimeVector[2][0];
	int32_t temp_time_vector_2_1 = tempTimeVector[2][1];

	int32_t temp_time_vector_3_0 = tempTimeVector[3][0];
	int32_t temp_time_vector_3_1 = tempTimeVector[3][1];

	int32_t temp_time_vector_4_0 = tempTimeVector[4][0];
	int32_t temp_time_vector_4_1 = tempTimeVector[4][1];

	int32_t temp_time_vector_5_0 = tempTimeVector[5][0];
	int32_t temp_time_vector_5_1 = tempTimeVector[5][1];

	int32_t temp_time_vector_6_0 = tempTimeVector[6][0];
	int32_t temp_time_vector_6_1 = tempTimeVector[6][1];

	int32_t temp_time_vector_7_0 = tempTimeVector[7][0];
	int32_t temp_time_vector_7_1 = tempTimeVector[7][1];

	int32_t temp_time_vector_8_0 = tempTimeVector[8][0];
	int32_t temp_time_vector_8_1 = tempTimeVector[8][1];

	int32_t temp_time_vector_9_0 = tempTimeVector[9][0];
	int32_t temp_time_vector_9_1 = tempTimeVector[9][1];

	JsonObject dutyCycleProfile = doc["aiConfigBody"]["dutyCycleProfile"];
	const char* duty_cycle_profile_id = dutyCycleProfile["id"];
	int32_t number_scanning_cycles = dutyCycleProfile["numberScanningCycles"];
	int32_t number_sleeping_cycles = dutyCycleProfile["numberSleepingCycles"];

	/* creating configuration header and body */
	JsonObject configHeader = config.createNestedObject("configHeader");
	configHeader["dateCreated_ISO"] = utils::get_rtc().now().timestamp() + "+00:00";
	configHeader["appVersion"] = app_version;
	configHeader["boardType"] = "board_8";
	configHeader["boardMode"] = "live_test_algorithm";
	configHeader["boardLayout"] = "custom";

	JsonObject configBody = config.createNestedObject("configBody");

	JsonObject heaterProfiles = configBody["heaterProfiles"].createNestedObject();
	heaterProfiles["id"] = heater_profile_id;
	heaterProfiles["timeBase"] = heater_profile_time_base;

	JsonArray tempTimeVectors = heaterProfiles.createNestedArray("temperatureTimeVectors");

	JsonArray tempTimeVectors_0 = tempTimeVectors.createNestedArray();
	tempTimeVectors_0.add(temp_time_vector_0_0);
	tempTimeVectors_0.add(temp_time_vector_0_1);

	JsonArray tempTimeVectors_1 = tempTimeVectors.createNestedArray();
	tempTimeVectors_1.add(temp_time_vector_1_0);
	tempTimeVectors_1.add(temp_time_vector_1_1);

	JsonArray tempTimeVectors_2 = tempTimeVectors.createNestedArray();
	tempTimeVectors_2.add(temp_time_vector_2_0);
	tempTimeVectors_2.add(temp_time_vector_2_1);

	JsonArray tempTimeVectors_3 = tempTimeVectors.createNestedArray();
	tempTimeVectors_3.add(temp_time_vector_3_0);
	tempTimeVectors_3.add(temp_time_vector_3_1);

	JsonArray tempTimeVectors_4 = tempTimeVectors.createNestedArray();
	tempTimeVectors_4.add(temp_time_vector_4_0);
	tempTimeVectors_4.add(temp_time_vector_4_1);

	JsonArray tempTimeVectors_5 = tempTimeVectors.createNestedArray();
	tempTimeVectors_5.add(temp_time_vector_5_0);
	tempTimeVectors_5.add(temp_time_vector_5_1);

	JsonArray tempTimeVectors_6 = tempTimeVectors.createNestedArray();
	tempTimeVectors_6.add(temp_time_vector_6_0);
	tempTimeVectors_6.add(temp_time_vector_6_1);

	JsonArray tempTimeVectors_7 = tempTimeVectors.createNestedArray();
	tempTimeVectors_7.add(temp_time_vector_7_0);
	tempTimeVectors_7.add(temp_time_vector_7_1);

	JsonArray tempTimeVectors_8 = tempTimeVectors.createNestedArray();
	tempTimeVectors_8.add(temp_time_vector_8_0);
	tempTimeVectors_8.add(temp_time_vector_8_1);

	JsonArray tempTimeVectors_9 = tempTimeVectors.createNestedArray();
	tempTimeVectors_9.add(temp_time_vector_9_0);
	tempTimeVectors_9.add(temp_time_vector_9_1);

	JsonArray configBody_dutyCycleProfiles = configBody.createNestedArray("dutyCycleProfiles");

	JsonObject dutyCycleProfiles = configBody_dutyCycleProfiles.createNestedObject();
	dutyCycleProfiles["id"] = duty_cycle_profile_id;
	dutyCycleProfiles["numberScanningCycles"] = number_scanning_cycles;
	dutyCycleProfiles["numberSleepingCycles"] = number_sleeping_cycles;

	JsonArray sensorConfigurations = configBody.createNestedArray("sensorConfigurations");

	JsonObject sensorConfiguration_0 = sensorConfigurations.createNestedObject();
	sensorConfiguration_0["sensorIndex"] = 0;

	if ( (sensorConfiguration_0["sensorIndex"] == sensor_num) || (sensor_num == NUM_OF_SENS) )
	{
		sensorConfiguration_0["active"] = true;
		sensorConfiguration_0["heaterProfile"] = heater_profile_id;
		sensorConfiguration_0["dutyCycleProfile"] = duty_cycle_profile_id;
	}
	else
	{
		sensorConfiguration_0["active"] = false;
		sensorConfiguration_0["heaterProfile"] = (char*)0;
		sensorConfiguration_0["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_1 = sensorConfigurations.createNestedObject();
	sensorConfiguration_1["sensorIndex"] = 1;

	if ( (sensorConfiguration_1["sensorIndex"] == sensor_num) || (sensor_num == NUM_OF_SENS) )
	{
		sensorConfiguration_1["active"] = true;
		sensorConfiguration_1["heaterProfile"] = heater_profile_id;
		sensorConfiguration_1["dutyCycleProfile"] = duty_cycle_profile_id;
	}
	else
	{
		sensorConfiguration_1["active"] = false;
		sensorConfiguration_1["heaterProfile"] = (char*)0;
		sensorConfiguration_1["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_2 = sensorConfigurations.createNestedObject();
	sensorConfiguration_2["sensorIndex"] = 2;

	if ( (sensorConfiguration_2["sensorIndex"] == sensor_num) || (sensor_num == NUM_OF_SENS) )
	{
		sensorConfiguration_2["active"] = true;
		sensorConfiguration_2["heaterProfile"] = heater_profile_id;
		sensorConfiguration_2["dutyCycleProfile"] = duty_cycle_profile_id;
	}
	else
	{
		sensorConfiguration_2["active"] = false;
		sensorConfiguration_2["heaterProfile"] = (char*)0;
		sensorConfiguration_2["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_3 = sensorConfigurations.createNestedObject();
	sensorConfiguration_3["sensorIndex"] = 3;

	if ( (sensorConfiguration_3["sensorIndex"] == sensor_num) || (sensor_num == NUM_OF_SENS) )
	{
		sensorConfiguration_3["active"] = true;
		sensorConfiguration_3["heaterProfile"] = heater_profile_id;
		sensorConfiguration_3["dutyCycleProfile"] = duty_cycle_profile_id;
	}
	else
	{
		sensorConfiguration_3["active"] = false;
		sensorConfiguration_3["heaterProfile"] = (char*)0;
		sensorConfiguration_3["dutyCycleProfile"] = (char*)0;
	}
	
	/* Sensor number 4 in start command is used to test sensors 0 - 3 in multi instance mode*/
	JsonObject sensorConfiguration_4 = sensorConfigurations.createNestedObject();
	sensorConfiguration_4["sensorIndex"] = 4;

	if ( (sensorConfiguration_4["sensorIndex"] == sensor_num) && (sensor_num != NUM_OF_SENS) )
	{
		sensorConfiguration_4["active"] = true;
		sensorConfiguration_4["heaterProfile"] = heater_profile_id;
		sensorConfiguration_4["dutyCycleProfile"] = duty_cycle_profile_id;
	}
	else
	{
		sensorConfiguration_4["active"] = false;
		sensorConfiguration_4["heaterProfile"] = (char*)0;
		sensorConfiguration_4["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_5 = sensorConfigurations.createNestedObject();
	sensorConfiguration_5["sensorIndex"] = 5;

	if (sensorConfiguration_5["sensorIndex"] == sensor_num)
	{
		sensorConfiguration_5["active"] = true;
		sensorConfiguration_5["heaterProfile"] = heater_profile_id;
		sensorConfiguration_5["dutyCycleProfile"] = duty_cycle_profile_id;
	}
	else
	{
		sensorConfiguration_5["active"] = false;
		sensorConfiguration_5["heaterProfile"] = (char*)0;
		sensorConfiguration_5["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_6 = sensorConfigurations.createNestedObject();
	sensorConfiguration_6["sensorIndex"] = 6;

	if (sensorConfiguration_6["sensorIndex"] == sensor_num)
	{
		sensorConfiguration_6["active"] = true;
		sensorConfiguration_6["heaterProfile"] = heater_profile_id;
		sensorConfiguration_6["dutyCycleProfile"] = duty_cycle_profile_id;
	}
	else
	{
		sensorConfiguration_6["active"] = false;
		sensorConfiguration_6["heaterProfile"] = (char*)0;
		sensorConfiguration_6["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_7 = sensorConfigurations.createNestedObject();
	sensorConfiguration_7["sensorIndex"] = 7;

	if (sensorConfiguration_7["sensorIndex"] == sensor_num)
	{
		sensorConfiguration_7["active"] = true;
		sensorConfiguration_7["heaterProfile"] = heater_profile_id;
		sensorConfiguration_7["dutyCycleProfile"] = duty_cycle_profile_id;
	}
	else
	{	
		sensorConfiguration_7["active"] = false;
		sensorConfiguration_7["heaterProfile"] = (char*)0;
		sensorConfiguration_7["dutyCycleProfile"] = (char*)0;
	}
	serializeJsonPretty(config, config_string);
	config_string = config_string.substring(0,config_string.length()-1);

	return EDK_OK;
}

/*!
 * @brief Function to create a bme68x label information file with .bmelabelinfo extension
 */
demo_ret_code bsecDataLogger::create_label_info_file()
{
	demo_ret_code ret_code = EDK_OK;
	String mac_str = utils::get_mac_address();
	String label_file_base_name = "_Board_" + mac_str + "_PowerOnOff_1_";
	uint16_t label_tag = 1001; //since labelTag starts with 1001
	
	_label_file_name = utils::get_date_time() + label_file_base_name + utils::get_file_seed() + 
                   "_File_" + String(_bsec_file_counter) + BME68X_LABEL_INFO_FILE_EXT;
	File logFile;

	if (!logFile.open(_label_file_name.c_str(), O_RDWR | O_CREAT))
	{
		ret_code = EDK_DATALOGGER_LABEL_INFO_FILE_ERROR;
	}
	else
	{
		filter2.clear();
		label_doc.clear();
		
		/* write labelinfo header / skeleton */
		logFile.println("{");
		logFile.println("\t\"labelInfoHeader\": {");
		logFile.println("\t\t\"counterPowerOnOff\": 1,");
		logFile.println("\t\t\"seedPowerOnOff\": \"" + utils::get_file_seed() + "\",");
		logFile.println("\t\t\"dateCreated\": \"" + String(utils::get_rtc().now().unixtime()) + "\",");
		logFile.println("\t\t\"dateCreated_ISO\": \"" + utils::get_rtc().now().timestamp() + "+00:00\",");
		logFile.println("\t\t\"firmwareVersion\": \"" + String(FIRMWARE_VERSION) + "\",");
		logFile.println("\t\t\"boardId\": \"" + mac_str + "\"");
		logFile.println("\t},");
		logFile.println("\t\"labelInformation\": [");
		logFile.println("\t\t{");
		logFile.println("\t\t\t\"labelTag\": 0,");
		logFile.println("\t\t\t\"labelName\": \"Initial\",");
		logFile.println("\t\t\t\"labelDescription\": \"Standard label for no label has been set\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t\t\"labelTag\": 1,");
		logFile.println("\t\t\t\"labelName\": \"Button 1\",");
		logFile.println("\t\t\t\"labelDescription\": \"Standard label for hardware button 1 pressed\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t\t\"labelTag\": 2,");
		logFile.println("\t\t\t\"labelName\": \"Button 2\",");
		logFile.println("\t\t\t\"labelDescription\": \"Standard label for hardware button 2 pressed\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t\t\"labelTag\": 3,");
		logFile.println("\t\t\t\"labelName\": \"Button 1+2\",");
		logFile.println("\t\t\t\"labelDescription\": \"Standard label for hardware button 1 and button 2 pressed\"");
		logFile.print("\t\t}");

		filter2["aiConfigBody"]["type"] = true;
		filter2["aiConfigBody"]["classes"] = true;
		filter2["aiConfigBody"]["targets"] = true;

		File configFile;
		configFile.open(_ai_config_name.c_str(), O_RDWR);
		
		DeserializationError error = deserializeJson(label_doc, configFile, DeserializationOption::Filter(filter2));
		configFile.close();

		if (error)
		{
			Serial.println(error.c_str());
			return EDK_SENSOR_MANAGER_JSON_DESERIAL_ERROR;
		}
		
		// reading configuration type
		const char* type = label_doc["aiConfigBody"]["type"];

		strcpy(ai_config_type, type);

		if (strcmp(type, "classification") == 0)
		{

			for (JsonObject classes : label_doc["aiConfigBody"]["classes"].as<JsonArray>())
			{
				const char* class_name = classes["name"];

				logFile.println(",\n\t\t{");
				logFile.println("\t\t\t\"labelTag\": " + String(label_tag++) + ",");
				logFile.println("\t\t\t\"labelName\": \"" + String(class_name) + "\",");
				logFile.println("\t\t\t\"labelDescription\": \"Predefined by algorithm classes (class name used as labelName)\"");
				logFile.print("\t\t}");
			}
		}
		else if (strcmp(type, "regression") == 0)
		{

			for (JsonObject target_name : label_doc["aiConfigBody"]["targets"].as<JsonArray>())
			{
				const char* targets = target_name["name"];

				logFile.println(",\n\t\t{");
				logFile.println("\t\t\t\"labelTag\": " + String(label_tag++) + ",");
				logFile.println("\t\t\t\"labelName\": \"" + String(targets) + "\",");
				logFile.println("\t\t\t\"labelDescription\": \"Predefined by algorithm Targets (target name used as labelName)\"");
				logFile.print("\t\t}");
			}
		}
		logFile.println("\n\t]");
		logFile.print("}");

		/* close log file */
		logFile.close();
		
	}
	return ret_code;
}

/*!
 * @brief Function which flushes the buffered sensor data to the current log file
 */
demo_ret_code bsecDataLogger::flush_sensor_data(uint8_t sensor_num)
{
	demo_ret_code ret_code = EDK_OK;
	File logFile;
	std::string txt;
	
	if (_bs.rdbuf()->in_avail())
	{
		txt = _bs.str();
		_bs.str(std::string());
		
		if (_bme_file_counter && logFile.open(_bme_file_name.c_str(), O_RDWR | O_AT_END))
		{
			logFile.seek(_bme_data_pos);
			logFile.print(txt.c_str());
			_bme_data_pos = logFile.position();
			logFile.print("\n\t\t]\n\t}\n}");
			
			if (logFile.size() >= FILE_SIZE_LIMIT)
			{
				logFile.close();
				++_bme_file_counter;
				ret_code = create_raw_data_file(sensor_num);

				if (ret_code >= EDK_OK)
				{
					ret_code = create_label_info_file();
				}
			}
			else
			{
				logFile.close();
			}
		}
		else
		{
			ret_code = EDK_DATALOGGER_LOG_FILE_ERROR;
		}
	}
	return ret_code;
}

/*!
 * @brief This function writes the bsec output to the current log file
 */
demo_ret_code bsecDataLogger::write_bsec_output(sensor_io_data& buff_data)
{
	demo_ret_code ret_code = EDK_OK;	    
	File logFile;
	
	if (_bsec_file_counter && logFile.open(_ai_file_name.c_str(), O_RDWR | O_AT_END))
	{
		/* set writing position to end of data block */
		logFile.seek(_ai_data_pos);
		
		float gas_signal[4] = {NAN, NAN, NAN, NAN}, iaq_signal = NAN; 
		uint8_t iaq_accuracy = 0xFF; //, gasAccuracy = 0xFF; 
		uint8_t gas_accuracy[4] = {0xFF, 0xFF, 0xFF, 0xFF};
		uint8_t is_reg_class_subscribe = 0;
		uint8_t Gas_accuracy = 0;
		uint8_t index = 0;
		
		for (uint8_t i = 0; ((buff_data.outputs).output != nullptr) && (i < (buff_data.outputs).nOutputs); i++) 
		{
			const bsec_output_t& output = (buff_data.outputs).output[i];

			if (output.sensor_id >= BSEC_OUTPUT_GAS_ESTIMATE_1 && output.sensor_id <= BSEC_OUTPUT_GAS_ESTIMATE_4)
			{
				index = output.sensor_id - BSEC_OUTPUT_GAS_ESTIMATE_1;
				gas_signal[index] = output.signal;
				gas_accuracy[index] = (output.accuracy > gas_accuracy[index]) ? gas_accuracy[index] : output.accuracy;
				Gas_accuracy |= ((gas_accuracy[index] & 0x03) << (index * 2));
				is_reg_class_subscribe = 1;
			}
			else if (output.sensor_id >= BSEC_OUTPUT_REGRESSION_ESTIMATE_1 && output.sensor_id <= BSEC_OUTPUT_REGRESSION_ESTIMATE_4)
			{
				index = output.sensor_id -  BSEC_OUTPUT_REGRESSION_ESTIMATE_1;
				gas_signal[index] = output.signal;
				gas_accuracy[index] = (output.accuracy > gas_accuracy[index]) ? gas_accuracy[index] : output.accuracy;
				Gas_accuracy |= ((gas_accuracy[index] & 0x03) << (index * 2));
				is_reg_class_subscribe = 1;
			}
			else if(output.sensor_id == BSEC_OUTPUT_IAQ)
			{
				iaq_signal = output.signal;
				iaq_accuracy = output.accuracy;
			}
		}

		if (!_first_line)
		{
			logFile.println(",");
		}		
		logFile.print("\t\t\t[\n\t\t\t\t");
		logFile.print(buff_data.sensor_num);
		logFile.print(",\n\t\t\t\t");
		logFile.print(buff_data.sensor_id);
		logFile.print(",\n\t\t\t\t");
		logFile.print(buff_data.start_time_since_power_on);
		logFile.print(",\n\t\t\t\t");
		logFile.print(buff_data.end_time_since_power_on);
		logFile.print(",\n\t\t\t\t");
		(!isnan(gas_signal[0])) ? logFile.print(gas_signal[0]) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(!isnan(gas_signal[1])) ? logFile.print(gas_signal[1]) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(!isnan(gas_signal[2])) ? logFile.print(gas_signal[2]) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(!isnan(gas_signal[3])) ? logFile.print(gas_signal[3]) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(is_reg_class_subscribe == 1) ? logFile.print(Gas_accuracy) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(!isnan(iaq_signal)) ? logFile.print(iaq_signal) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(iaq_accuracy != 0xFF) ? logFile.print(iaq_accuracy) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		logFile.print(buff_data.code);
		logFile.print(",\n\t\t\t\t");
		logFile.println(buff_data.ground_truth);
		logFile.print("\t\t\t]");
		
		_ai_data_pos = logFile.position();
		_first_line = false;
		logFile.println("\n\t\t]\n\t}\n}");
		
		logFile.close();
	}
	else 
	{
		ret_code = EDK_DATALOGGER_LOG_FILE_ERROR;
	}
	return ret_code;
}

/*!
 * @brief This function writes the sensor data to the current log file.
 */ 
demo_ret_code bsecDataLogger::write_sensor_data(const uint8_t* num, const uint32_t* sensor_id,
                                            const bme68x_data* bme68xData, const uint32_t* scan_cycle_index,
                                            uint32_t ground_truth, demo_ret_code code, sensor_io_data& buff_data)
{
	demo_ret_code ret_code = EDK_OK;
	uint32_t rtc_tsp = utils::get_rtc().now().unixtime();
	uint32_t time_since_power_on = millis();

	//Copying time for sync with aiprediction file
	if (bme68xData != nullptr)
	{

		if (bme68xData->gas_index == 0)
		{
			buff_data.start_time_since_power_on = time_since_power_on;
		}
		else
		{
			buff_data.end_time_since_power_on = time_since_power_on;
		}
	}

	if (_end_of_line)
	{
		_bs << ",\n";
	}
	_bs << "\t\t\t[\n\t\t\t\t";
	(num != nullptr) ? (_bs << (int32_t)*num) : (_bs << "null");
	_bs << ",\n\t\t\t\t";
	(sensor_id != nullptr) ? (_bs << (int32_t)*sensor_id) : (_bs << "null");
	_bs << ",\n\t\t\t\t";
	_bs << time_since_power_on;
	_bs << ",\n\t\t\t\t";
	_bs << rtc_tsp;
	_bs << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_bs << bme68xData->temperature) : (_bs << "null");
	_bs << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_bs << bme68xData->pressure) : (_bs << "null");
	_bs << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_bs << bme68xData->humidity) : (_bs << "null");
	_bs << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_bs << bme68xData->gas_resistance) : (_bs << "null");
	_bs << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_bs << (int32_t)bme68xData->gas_index) : (_bs << "null");
	_bs << ",\n\t\t\t\t";
	_bs << (bool)(BME68X_PARALLEL_MODE);
	_bs << ",\n\t\t\t\t";
	(scan_cycle_index != nullptr) ? (_bs << (int32_t)(*scan_cycle_index)) : (_bs << "null");
	_bs << ",\n\t\t\t\t";
	_bs << ground_truth;
	_bs << ",\n\t\t\t\t";
	_bs << (int32_t)code;
	_bs << "\n\t\t\t]";

	_end_of_line = true;
   
  return ret_code;
}
