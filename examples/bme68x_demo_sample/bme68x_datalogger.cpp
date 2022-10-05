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
 * @file	    bme68x_datalogger.cpp
 * @date	    22 June 2022
 * @version		1.5.5
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
bme68xDataLogger::bme68xDataLogger() : _fileCounter(0)
{}

/*!
 * @brief Function to configure the datalogger using the provided sensor config file
 */
demoRetCode bme68xDataLogger::begin(const String& configName)
{
	demoRetCode retCode = utils::begin();
	
	_configName = configName;
	if (retCode >= EDK_OK)
	{
		retCode = createLogFile();
		
		_ss.setf(std::ios::fixed, std::ios::floatfield);
	}
	return retCode;
}

/*!
 * @brief Function which flushes the buffered sensor data to the current log file
 */
demoRetCode bme68xDataLogger::flush()
{
	demoRetCode retCode = EDK_OK;
	File logFile;
	std::string txt;
	
    if (_ss.rdbuf()->in_avail())
	{
		txt = _ss.str();
		_ss.str(std::string());
		
		if (_fileCounter && logFile.open(_logFileName.c_str(), O_RDWR | O_AT_END))
		{
			logFile.seek(_sensorDataPos);
			logFile.print(txt.c_str());
			_sensorDataPos = logFile.position();
			logFile.println("\n\t    ]\n\t}\n}");
			
			logFile.close();
			
			if (logFile.size() >= FILE_SIZE_LIMIT)
			{
				retCode = createLogFile();
			}
		}
		else
		{
			retCode = EDK_DATALOGGER_LOG_FILE_ERROR;
		}
	}
	return retCode;
}

/*!
 * @brief Function writes the sensor data to the current log file
 */
demoRetCode bme68xDataLogger::writeSensorData(const uint8_t* num, const uint32_t* sensorId, const uint8_t* sensorMode, const bme68x_data* bme68xData, gasLabel label, demoRetCode code)
{
	demoRetCode retCode = EDK_OK;
    uint32_t rtcTsp = utils::getRtc().now().unixtime();
    uint32_t timeSincePowerOn = millis();

	if (_endOfLine)
	{
		_ss << ",\n";
	}
	_ss << "\t\t[";
	(num != nullptr) ? (_ss << (int)*num) : (_ss << "null");
	_ss << ",";
	(sensorId != nullptr) ? (_ss << (int)*sensorId) : (_ss << "null");
	_ss << ",";
	_ss << timeSincePowerOn;
	_ss << ",";
	_ss << rtcTsp;
	_ss << ",";
	(bme68xData != nullptr) ? (_ss << bme68xData->temperature) : (_ss << "null");
	_ss << ",";
	(bme68xData != nullptr) ? (_ss << (bme68xData->pressure * .01f)) : (_ss << "null");
	_ss << ",";
	(bme68xData != nullptr) ? (_ss << bme68xData->humidity) : (_ss << "null");
	_ss << ",";
	(bme68xData != nullptr) ? (_ss << bme68xData->gas_resistance) : (_ss << "null");
	_ss << ",";
	(bme68xData != nullptr) ? (_ss << (int)bme68xData->gas_index) : (_ss << "null");
	_ss << ",";
	(sensorMode != nullptr) ? (_ss << (int)(*sensorMode == BME68X_PARALLEL_MODE)) : (_ss << "null");
	_ss << ",";
	_ss << (int)label;
	_ss << ",";
	_ss << (int)code;
	_ss << "]";

	_endOfLine = true;
    return retCode;
}

/*!
 * @brief function to create a bme68x datalogger output file with .bmerawdata extension
 */
demoRetCode bme68xDataLogger::createLogFile()
{
	demoRetCode retCode = EDK_OK;
	String macStr = utils::getMacAddress();
    String logFileBaseName = "_Board_" + macStr + "_PowerOnOff_1_";
	
    _logFileName = utils::getDateTime() + logFileBaseName + utils::getFileSeed() + "_File_" + String(_fileCounter) + BME68X_RAWDATA_FILE_EXT;             

	File configFile, logFile;
    if (_configName.length() && !configFile.open(_configName.c_str(), O_RDWR))
	{
		retCode = EDK_DATALOGGER_SENSOR_CONFIG_FILE_ERROR;
	}
	else if (!logFile.open(_logFileName.c_str(), O_RDWR | O_CREAT))
	{
		retCode = EDK_DATALOGGER_LOG_FILE_ERROR;
	}
	else
	{
		if (_configName.length())
		{
			String lineBuffer;
			/* read in each line from the config file and copy it to the log file */
			while(configFile.available())
			{
				lineBuffer = configFile.readStringUntil('\n');
				/* skip the last closing curly bracket of the JSON document */
				if (lineBuffer == "}")
				{
					logFile.println("\t,");
					break;
				}
				logFile.println(lineBuffer);
			}
			configFile.close();
		}
		else
		{
			logFile.println("{");
		}

		/* write data header / skeleton */
		/* raw data header */
		logFile.println("    \"rawDataHeader\":");		
		logFile.println("\t{");
		logFile.println("\t    \"counterPowerOnOff\": 1,");
		logFile.println("\t    \"seedPowerOnOff\": \"" + utils::getFileSeed() + "\",");
		logFile.println("\t    \"counterFileLimit\": " + String(_fileCounter) + ",");
		logFile.println("\t    \"dateCreated\": \"" + String(utils::getRtc().now().unixtime()) + "\",");
		logFile.println("\t    \"dateCreated_ISO\": \"" + utils::getRtc().now().timestamp() + "+00:00\",");
		logFile.println("\t    \"firmwareVersion\": \"" + String(FIRMWARE_VERSION) + "\",");
		logFile.println("\t    \"boardId\": \"" + macStr + "\"");
		logFile.println("\t},");
		logFile.println("    \"rawDataBody\":");
		logFile.println("\t{");
		logFile.println("\t    \"dataColumns\": [");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Sensor Index\",");
		logFile.println("\t\t    \"unit\": \"\",");
		logFile.println("\t\t    \"format\": \"integer\",");
		logFile.println("\t\t    \"key\": \"sensor_index\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Sensor ID\",");
		logFile.println("\t\t    \"unit\": \"\",");
		logFile.println("\t\t    \"format\": \"integer\",");
		logFile.println("\t\t    \"key\": \"sensorId\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Time Since PowerOn\",");
		logFile.println("\t\t    \"unit\": \"Milliseconds\",");
		logFile.println("\t\t    \"format\": \"integer\",");
		logFile.println("\t\t    \"key\": \"timestamp_since_poweron\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Real time clock\",");
		logFile.println("\t\t    \"unit\": \"Unix Timestamp: seconds since Jan 01 1970. (UTC); 0 = missing\",");
		logFile.println("\t\t    \"format\": \"integer\",");
		logFile.println("\t\t    \"key\": \"real_time_clock\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Temperature\",");
		logFile.println("\t\t    \"unit\": \"DegreesClecius\",");
		logFile.println("\t\t    \"format\": \"float\",");
		logFile.println("\t\t    \"key\": \"temperature\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Pressure\",");
		logFile.println("\t\t    \"unit\": \"Hectopascals\",");
		logFile.println("\t\t    \"format\": \"float\",");
		logFile.println("\t\t    \"key\": \"pressure\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Relative Humidity\",");
		logFile.println("\t\t    \"unit\": \"Percent\",");
		logFile.println("\t\t    \"format\": \"float\",");
		logFile.println("\t\t    \"key\": \"relative_humidity\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Resistance Gassensor\",");
		logFile.println("\t\t    \"unit\": \"Ohms\",");
		logFile.println("\t\t    \"format\": \"float\",");
		logFile.println("\t\t    \"key\": \"resistance_gassensor\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Heater Profile Step Index\",");
		logFile.println("\t\t    \"unit\": \"\",");
		logFile.println("\t\t    \"format\": \"integer\",");
		logFile.println("\t\t    \"key\": \"heater_profile_step_index\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Scanning enabled\",");
		logFile.println("\t\t    \"unit\": \"\",");
		logFile.println("\t\t    \"format\": \"integer\",");
		logFile.println("\t\t    \"key\": \"scanning_enabled\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Label Tag\",");
		logFile.println("\t\t    \"unit\": \"\",");
		logFile.println("\t\t    \"format\": \"integer\",");
		logFile.println("\t\t    \"key\": \"label_tag\"");
		logFile.println("\t\t},");
		logFile.println("\t\t{");
		logFile.println("\t\t    \"name\": \"Error Code\",");
		logFile.println("\t\t    \"unit\": \"\",");
		logFile.println("\t\t    \"format\": \"integer\",");
		logFile.println("\t\t    \"key\": \"error_code\"");
		logFile.println("\t\t}");
		logFile.println("\t    ],");
		
		/* data block */
		logFile.println("\t    \"dataBlock\": [");
		/* save position in file, where to write the first data set */
		_sensorDataPos = logFile.position();
		logFile.println("\t    ]");
		logFile.println("\t}");
		logFile.println("}");
		
		/* close log file */
		logFile.close();
		
		_endOfLine = false;
		++_fileCounter;
	}
    return retCode;
}
