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
 * @file	bme68x_datalogger.cpp
 * @date	03 Jan 2024
 * @version	2.1.5
 * 
 * @brief    	bme68x_datalogger
 *
 * 
 */

/* own header include */
#include "bme68x_datalogger.h"
#include <Esp.h>

/*!
 * @brief The constructor of the bme68xDataLogger class
 */
bme68xDataLogger::bme68xDataLogger() : _file_counter(1)
{}

/*!
 * @brief Function to configure the datalogger using the provided sensor config file
 */
demo_ret_code bme68xDataLogger::begin(const String& config_name)
{
	demo_ret_code ret_code = utils::begin();
	
	_config_name = config_name;

	if (ret_code >= EDK_OK)
	{
		/* Resets the file counter when seed file is generated */
		_file_counter = 1;
		ret_code = create_log_file();

		if (ret_code >= EDK_OK)
		{
			ret_code = create_label_info_file();
		}
		_ss.setf(std::ios::fixed, std::ios::floatfield);
	}
	return ret_code;
}

/*!
 * @brief Function which flushes the buffered sensor data to the current log file
 */
demo_ret_code bme68xDataLogger::flush()
{
	demo_ret_code ret_code = EDK_OK;
	File logFile;
	std::string txt;
	
  if (_ss.rdbuf()->in_avail())
	{
		txt = _ss.str();
		_ss.str(std::string());
		
		if (_file_counter && logFile.open(_log_file_name.c_str(), O_RDWR | O_AT_END))
		{
			logFile.seek(_sensor_data_pos);
			logFile.print(txt.c_str());
			_sensor_data_pos = logFile.position();
			logFile.println("\n\t    ]\n\t}\n}");
			
			if (logFile.size() >= FILE_SIZE_LIMIT)
			{
				logFile.close();
				++_file_counter;
				ret_code = create_log_file();

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
 * @brief Function writes the sensor data to the current log file
 */
demo_ret_code bme68xDataLogger::write_sensor_data(const uint8_t* num, const uint32_t* sensor_id, const uint8_t* sensorMode,
                                              const bme68x_data* bme68xData, const uint32_t* scan_cycle_index, 
                                              gas_label label, demo_ret_code code)
{
	demo_ret_code ret_code = EDK_OK;
	uint32_t rtc_tsp = utils::get_rtc().now().unixtime();
	uint32_t time_since_power_on = millis();

	if (_end_of_line)
	{
		_ss << ",\n";
	}
	_ss << "\t\t\t[\n\t\t\t\t";
	(num != nullptr) ? (_ss << (uint32_t)*num) : (_ss << "null");
	_ss << ",\n\t\t\t\t";
	(sensor_id != nullptr) ? (_ss << (uint32_t)*sensor_id) : (_ss << "null");
	_ss << ",\n\t\t\t\t";
	_ss << time_since_power_on;
	_ss << ",\n\t\t\t\t";
	_ss << rtc_tsp;
	_ss << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_ss << bme68xData->temperature) : (_ss << "null");
	_ss << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_ss << (bme68xData->pressure * .01f)) : (_ss << "null");
	_ss << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_ss << bme68xData->humidity) : (_ss << "null");
	_ss << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_ss << bme68xData->gas_resistance) : (_ss << "null");
	_ss << ",\n\t\t\t\t";
	(bme68xData != nullptr) ? (_ss << (uint32_t)bme68xData->gas_index) : (_ss << "null");
	_ss << ",\n\t\t\t\t";
	_ss << (bool)(BME68X_PARALLEL_MODE);
	_ss << ",\n\t\t\t\t";
	(scan_cycle_index != nullptr) ? (_ss << (uint32_t)(*scan_cycle_index)) : (_ss << "null");
	_ss << ",\n\t\t\t\t";
	_ss << (uint32_t)label;
	_ss << ",\n\t\t\t\t";
	_ss << (uint32_t)code;
	_ss << "\n\t\t\t]";

	_end_of_line = true;
	return ret_code;
}

/*!
 * @brief Function stores the labelTag, labelName and labelDescription to the .bmelabelinfo file
 */
demo_ret_code bme68xDataLogger::set_label_info(int32_t label_tag, const String& label_name, const String&  label_desc)
{
	demo_ret_code ret_code = EDK_OK;
		
	File logFile;

	if (logFile.open(_label_file_name.c_str(), O_READ))
	{
		DeserializationError error = deserializeJson(label_doc, logFile);
		logFile.close();

		if (error)
		{
			Serial.println(error.c_str());
			return EDK_SENSOR_MANAGER_JSON_DESERIAL_ERROR;
		}
		JsonArray lblInfo = label_doc["labelInformation"].as<JsonArray>();
		
		JsonObject obj = lblInfo.createNestedObject();
		obj["labelTag"] = label_tag;
		obj["labelName"] = label_name;
		obj["labelDescription"] = label_desc;

		if (logFile.open(_label_file_name.c_str(), O_RDWR | O_TRUNC))
		{
			serializeJsonPretty(label_doc, logFile);
			logFile.close();
		}
		else
		{
			ret_code = EDK_DATALOGGER_LABEL_INFO_FILE_ERROR;
		}
	}
	else
	{
		ret_code = EDK_DATALOGGER_LABEL_INFO_FILE_ERROR;
	}
	return ret_code;
}

/*!
 * @brief Function to create a bme68x datalogger output file with .bmerawdata extension
 */
demo_ret_code bme68xDataLogger::create_log_file()
{
	demo_ret_code ret_code = EDK_OK;
	String mac_str = utils::get_mac_address();
	String log_file_base_name = "_Board_" + mac_str + "_PowerOnOff_1_";
	
	_log_file_name = utils::get_date_time() + log_file_base_name + utils::get_file_seed() +
                 "_File_" + String(_file_counter) + BME68X_RAWDATA_FILE_EXT;             

	File configFile, logFile;

	if (_config_name.length() && !configFile.open(_config_name.c_str(), O_RDWR))
	{
		ret_code = EDK_DATALOGGER_SENSOR_CONFIG_FILE_ERROR;
	}
	else if (!logFile.open(_log_file_name.c_str(), O_RDWR | O_CREAT))
	{
		ret_code = EDK_DATALOGGER_LOG_FILE_ERROR;
	}
	else
	{
		if (_config_name.length())
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

		/* write data header / skeleton */
		/* raw data header */
		logFile.println("\t\"rawDataHeader\": {");
		logFile.println("\t\t\"counterPowerOnOff\": 1,");
		logFile.println("\t\t\"seedPowerOnOff\": \"" + utils::get_file_seed() + "\",");
		logFile.println("\t\t\"counterFileLimit\": " + String(_file_counter) + ",");
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
		_sensor_data_pos = logFile.position();
		logFile.println("\t\t]");
		logFile.println("\t}");
		logFile.println("}");
		
		/* close log file */
		logFile.close();
		
		_end_of_line = false;
	}
	return ret_code;
}

/*!
 * @brief Function to create a bme68x label information file with .bmelabelinfo extension
 */
demo_ret_code bme68xDataLogger::create_label_info_file()
{
	demo_ret_code ret_code = EDK_OK;
	String mac_str = utils::get_mac_address();
	String label_file_base_name = "_Board_" + mac_str + "_PowerOnOff_1_";
	
	_label_file_name = utils::get_date_time() + label_file_base_name + utils::get_file_seed() +
                   "_File_" + String(_file_counter) + BME68X_LABEL_INFO_FILE_EXT;
	File logFile;

	if (!logFile.open(_label_file_name.c_str(), O_RDWR | O_CREAT))
	{
		ret_code = EDK_DATALOGGER_LABEL_INFO_FILE_ERROR;
	}
	else
	{
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
		logFile.println("\n\t]");
		logFile.print("}");
		
		/* close log file */
		logFile.close();
	}
	return ret_code;
}
