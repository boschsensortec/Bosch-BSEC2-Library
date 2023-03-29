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
 * @date		17 January 2023
 * @version		2.0.6
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
StaticJsonDocument<32> 		filter2;
StaticJsonDocument<512> 	labelDoc;
StaticJsonDocument<2048> 	config;

/*!
 * @brief The constructor of the bsec_datalogger class 
 */
bsecDataLogger::bsecDataLogger() : _bsecFileCounter(1)
{}

/*!
 * @brief This function configures the bsec datalogger using the provided bsec config string file
 */
demoRetCode bsecDataLogger::begin(const String& configName, const bsec_version_t& bsecVersion, uint8_t sensorNum)
{
	demoRetCode retCode = utils::begin();
	
	_aiConfigName = configName;
	_version = bsecVersion;
	if (retCode >= EDK_OK)
	{
		retCode = createBsecFile();
		if (retCode >= EDK_OK)
		{
			retCode = createRawDataFile(sensorNum);
			if (retCode >= EDK_OK)
			{
				retCode = createLabelInfoFile();
			}
		}
	}
	return retCode;
}
 
/*!
 * @brief This function creates a bsec output file
 */
demoRetCode bsecDataLogger::createBsecFile()
{
	demoRetCode retCode = EDK_OK;
	String macStr = utils::getMacAddress();	
    String aiFileBaseName = "_Board_" + macStr + "_PowerOnOff_1_";
	
    _aiFileName = utils::getDateTime() + aiFileBaseName + utils::getFileSeed() + "_File_" + 
														String(_bsecFileCounter) + AI_DATA_FILE_EXT;
	
	File configFile, logFile;
	if (_aiConfigName.length() && !configFile.open(_aiConfigName.c_str(), O_RDWR))
	{
		retCode = EDK_DATALOGGER_AI_CONFIG_FILE_ERROR;
	}
	else if (!logFile.open(_aiFileName.c_str(), O_RDWR | O_CREAT))
	{
		retCode = EDK_DATALOGGER_LOG_FILE_ERROR;
	}
	else 
	{
		if (_aiConfigName.length())
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
		logFile.println("\t\"aiPredictionsDataHeader\": {");
		logFile.println("\t\t\"counterPowerOnOff\": 1,");
		logFile.println("\t\t\"seedPowerOnOff\": \"" + utils::getFileSeed() + "\",");
		logFile.println("\t\t\"counterFileLimit\": " + String(_bsecFileCounter) + ",");
		logFile.println("\t\t\"dateCreated\": \"" + String(utils::getRtc().now().unixtime()) + "\",");
		logFile.println("\t\t\"dateCreated_ISO\": \"" + utils::getRtc().now().timestamp() + "+00:00\",");
		logFile.println("\t\t\"firmwareVersion\": \"" + String(FIRMWARE_VERSION) + "\",");
		logFile.println("\t\t\"bsecVersion\": \"" + String(_version.major) + "." + String(_version.minor) + \
							 "." + String(_version.major_bugfix) + "." + String(_version.minor_bugfix) + "\",");
		logFile.println("\t\t\"boardId\": \"" + macStr + "\"");
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
		logFile.println("\t\t\t\t\"name\": \"Class 1 prediction\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"class_1_prediction\",");
		logFile.println("\t\t\t\t\"colId\": 5");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Class 2 prediction\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"class_2_prediction\",");
		logFile.println("\t\t\t\t\"colId\": 6");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Class 3 prediction\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"class_3_prediction\",");
		logFile.println("\t\t\t\t\"colId\": 7");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Class 4 prediction\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"float\",");
		logFile.println("\t\t\t\t\"key\": \"class_4_prediction\",");
		logFile.println("\t\t\t\t\"colId\": 8");
		logFile.println("\t\t\t},");
		logFile.println("\t\t\t{");
		logFile.println("\t\t\t\t\"name\": \"Class prediction accuracy\",");
		logFile.println("\t\t\t\t\"unit\": \"\",");
		logFile.println("\t\t\t\t\"format\": \"integer\",");
		logFile.println("\t\t\t\t\"key\": \"class_prediction_accuracy\",");
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
		_aiDataPos = logFile.position();
		logFile.println("\t\t]");
		logFile.println("\t}");
		logFile.println("}");
		
		/* close log file */
		logFile.close();
		
		_firstLine = true;
	}
    return retCode;
}

/*!
 * @brief This function creates a bme68x datalogger output file
 */
demoRetCode bsecDataLogger::createRawDataFile(uint8_t sensorNum)
{
	demoRetCode retCode = EDK_OK;
	String macStr = utils::getMacAddress();	
    String bmeFileBaseName = "_Board_" + macStr + "_PowerOnOff_1_";
	
    _bmeFileName = utils::getDateTime() + bmeFileBaseName + utils::getFileSeed() + "_File_" + 
														String(_bsecFileCounter) + BME68X_RAWDATA_FILE_EXT;
	
	File logFile;
	if (!logFile.open(_bmeFileName.c_str(), O_RDWR | O_CREAT))
	{
		retCode = EDK_DATALOGGER_LOG_FILE_ERROR;
	}
	else 
	{
		retCode = prepareConfigContent(sensorNum);
		if (retCode != EDK_OK)
		{
			return retCode;
		}

		/* Writes the config header and body to the logfile */
		logFile.print(configString);
		/* write data header / skeleton */
		/* raw data header */
		logFile.println("\t,\n\t\"rawDataHeader\": {");
		logFile.println("\t\t\"counterPowerOnOff\": 1,");
		logFile.println("\t\t\"seedPowerOnOff\": \"" + utils::getFileSeed() + "\",");
		logFile.println("\t\t\"counterFileLimit\": " + String(_bsecFileCounter) + ",");
		logFile.println("\t\t\"dateCreated\": \"" + String(utils::getRtc().now().unixtime()) + "\",");
		logFile.println("\t\t\"dateCreated_ISO\": \"" + utils::getRtc().now().timestamp() + "+00:00\",");
		logFile.println("\t\t\"firmwareVersion\": \"" + String(FIRMWARE_VERSION) + "\",");
		logFile.println("\t\t\"boardId\": \"" + macStr + "\"");
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
		_bmeDataPos = logFile.position();
		logFile.println("\t\t]");
		logFile.println("\t}");
		logFile.println("}");

		_endOfLine = false;
		/* close log file */
		logFile.close();
	}
    return retCode;
}

demoRetCode bsecDataLogger::prepareConfigContent(uint8_t sensorNum)
{
	demoRetCode retCode = EDK_OK;
	filter1.clear();
	doc.clear();
	config.clear();
	configString = "\0";

	File configFile;
	if (_aiConfigName.length() && !configFile.open(_aiConfigName.c_str(), O_RDWR))
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
	const char* appVersion = doc["aiConfigHeader"]["appVersion"];

	JsonObject heaterProfile = doc["aiConfigBody"]["heaterProfile"];
	const char* heaterProfileId = heaterProfile["id"];
	int heaterProfileTimeBase = heaterProfile["timeBase"];

	JsonArray tempTimeVector = heaterProfile["temperatureTimeVectors"];

	int tempTimeVector_0_0 = tempTimeVector[0][0];
	int tempTimeVector_0_1 = tempTimeVector[0][1];

	int tempTimeVector_1_0 = tempTimeVector[1][0];
	int tempTimeVector_1_1 = tempTimeVector[1][1];

	int tempTimeVector_2_0 = tempTimeVector[2][0];
	int tempTimeVector_2_1 = tempTimeVector[2][1];

	int tempTimeVector_3_0 = tempTimeVector[3][0];
	int tempTimeVector_3_1 = tempTimeVector[3][1];

	int tempTimeVector_4_0 = tempTimeVector[4][0];
	int tempTimeVector_4_1 = tempTimeVector[4][1];

	int tempTimeVector_5_0 = tempTimeVector[5][0];
	int tempTimeVector_5_1 = tempTimeVector[5][1];

	int tempTimeVector_6_0 = tempTimeVector[6][0];
	int tempTimeVector_6_1 = tempTimeVector[6][1];

	int tempTimeVector_7_0 = tempTimeVector[7][0];
	int tempTimeVector_7_1 = tempTimeVector[7][1];

	int tempTimeVector_8_0 = tempTimeVector[8][0];
	int tempTimeVector_8_1 = tempTimeVector[8][1];

	int tempTimeVector_9_0 = tempTimeVector[9][0];
	int tempTimeVector_9_1 = tempTimeVector[9][1];

	JsonObject dutyCycleProfile = doc["aiConfigBody"]["dutyCycleProfile"];
	const char* dutyCycleProfileId = dutyCycleProfile["id"];
	int numberScanningCycles = dutyCycleProfile["numberScanningCycles"];
	int numberSleepingCycles = dutyCycleProfile["numberSleepingCycles"];

	/* creating configuration header and body */
	JsonObject configHeader = config.createNestedObject("configHeader");
	configHeader["dateCreated"] = utils::getRtc().now().timestamp() + "+00:00";
	configHeader["appVersion"] = appVersion;
	configHeader["boardType"] = "board_8";
	configHeader["boardMode"] = "live_test_algorithm";
	configHeader["boardLayout"] = "custom";

	JsonObject configBody = config.createNestedObject("configBody");

	JsonObject heaterProfiles = configBody["heaterProfiles"].createNestedObject();
	heaterProfiles["id"] = heaterProfileId;
	heaterProfiles["timeBase"] = heaterProfileTimeBase;

	JsonArray tempTimeVectors = heaterProfiles.createNestedArray("temperatureTimeVectors");

	JsonArray tempTimeVectors_0 = tempTimeVectors.createNestedArray();
	tempTimeVectors_0.add(tempTimeVector_0_0);
	tempTimeVectors_0.add(tempTimeVector_0_1);

	JsonArray tempTimeVectors_1 = tempTimeVectors.createNestedArray();
	tempTimeVectors_1.add(tempTimeVector_1_0);
	tempTimeVectors_1.add(tempTimeVector_1_1);

	JsonArray tempTimeVectors_2 = tempTimeVectors.createNestedArray();
	tempTimeVectors_2.add(tempTimeVector_2_0);
	tempTimeVectors_2.add(tempTimeVector_2_1);

	JsonArray tempTimeVectors_3 = tempTimeVectors.createNestedArray();
	tempTimeVectors_3.add(tempTimeVector_3_0);
	tempTimeVectors_3.add(tempTimeVector_3_1);

	JsonArray tempTimeVectors_4 = tempTimeVectors.createNestedArray();
	tempTimeVectors_4.add(tempTimeVector_4_0);
	tempTimeVectors_4.add(tempTimeVector_4_1);

	JsonArray tempTimeVectors_5 = tempTimeVectors.createNestedArray();
	tempTimeVectors_5.add(tempTimeVector_5_0);
	tempTimeVectors_5.add(tempTimeVector_5_1);

	JsonArray tempTimeVectors_6 = tempTimeVectors.createNestedArray();
	tempTimeVectors_6.add(tempTimeVector_6_0);
	tempTimeVectors_6.add(tempTimeVector_6_1);

	JsonArray tempTimeVectors_7 = tempTimeVectors.createNestedArray();
	tempTimeVectors_7.add(tempTimeVector_7_0);
	tempTimeVectors_7.add(tempTimeVector_7_1);

	JsonArray tempTimeVectors_8 = tempTimeVectors.createNestedArray();
	tempTimeVectors_8.add(tempTimeVector_8_0);
	tempTimeVectors_8.add(tempTimeVector_8_1);

	JsonArray tempTimeVectors_9 = tempTimeVectors.createNestedArray();
	tempTimeVectors_9.add(tempTimeVector_9_0);
	tempTimeVectors_9.add(tempTimeVector_9_1);

	JsonArray configBody_dutyCycleProfiles = configBody.createNestedArray("dutyCycleProfiles");

	JsonObject dutyCycleProfiles = configBody_dutyCycleProfiles.createNestedObject();
	dutyCycleProfiles["id"] = dutyCycleProfileId;
	dutyCycleProfiles["numberScanningCycles"] = numberScanningCycles;
	dutyCycleProfiles["numberSleepingCycles"] = numberSleepingCycles;

	JsonArray sensorConfigurations = configBody.createNestedArray("sensorConfigurations");

	JsonObject sensorConfiguration_0 = sensorConfigurations.createNestedObject();
	sensorConfiguration_0["sensorIndex"] = 0;
	if (sensorConfiguration_0["sensorIndex"] == sensorNum)
	{
		sensorConfiguration_0["active"] = true;
		sensorConfiguration_0["heaterProfile"] = heaterProfileId;
		sensorConfiguration_0["dutyCycleProfile"] = dutyCycleProfileId;
	}
	else
	{
		sensorConfiguration_0["active"] = false;
		sensorConfiguration_0["heaterProfile"] = (char*)0;
		sensorConfiguration_0["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_1 = sensorConfigurations.createNestedObject();
	sensorConfiguration_1["sensorIndex"] = 1;
	if (sensorConfiguration_1["sensorIndex"] == sensorNum)
	{
		sensorConfiguration_1["active"] = true;
		sensorConfiguration_1["heaterProfile"] = heaterProfileId;
		sensorConfiguration_1["dutyCycleProfile"] = dutyCycleProfileId;
	}
	else
	{
		sensorConfiguration_1["active"] = false;
		sensorConfiguration_1["heaterProfile"] = (char*)0;
		sensorConfiguration_1["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_2 = sensorConfigurations.createNestedObject();
	sensorConfiguration_2["sensorIndex"] = 2;
	if (sensorConfiguration_2["sensorIndex"] == sensorNum)
	{
		sensorConfiguration_2["active"] = true;
		sensorConfiguration_2["heaterProfile"] = heaterProfileId;
		sensorConfiguration_2["dutyCycleProfile"] = dutyCycleProfileId;
	}
	else
	{
		sensorConfiguration_2["active"] = false;
		sensorConfiguration_2["heaterProfile"] = (char*)0;
		sensorConfiguration_2["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_3 = sensorConfigurations.createNestedObject();
	sensorConfiguration_3["sensorIndex"] = 3;
	if (sensorConfiguration_3["sensorIndex"] == sensorNum)
	{
		sensorConfiguration_3["active"] = true;
		sensorConfiguration_3["heaterProfile"] = heaterProfileId;
		sensorConfiguration_3["dutyCycleProfile"] = dutyCycleProfileId;
	}
	else
	{
		sensorConfiguration_3["active"] = false;
		sensorConfiguration_3["heaterProfile"] = (char*)0;
		sensorConfiguration_3["dutyCycleProfile"] = (char*)0;
	}
	
	JsonObject sensorConfiguration_4 = sensorConfigurations.createNestedObject();
	sensorConfiguration_4["sensorIndex"] = 4;
	if (sensorConfiguration_4["sensorIndex"] == sensorNum)
	{
		sensorConfiguration_4["active"] = true;
		sensorConfiguration_4["heaterProfile"] = heaterProfileId;
		sensorConfiguration_4["dutyCycleProfile"] = dutyCycleProfileId;
	}
	else
	{
		sensorConfiguration_4["active"] = false;
		sensorConfiguration_4["heaterProfile"] = (char*)0;
		sensorConfiguration_4["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_5 = sensorConfigurations.createNestedObject();
	sensorConfiguration_5["sensorIndex"] = 5;
	if (sensorConfiguration_5["sensorIndex"] == sensorNum)
	{
		sensorConfiguration_5["active"] = true;
		sensorConfiguration_5["heaterProfile"] = heaterProfileId;
		sensorConfiguration_5["dutyCycleProfile"] = dutyCycleProfileId;
	}
	else
	{
		sensorConfiguration_5["active"] = false;
		sensorConfiguration_5["heaterProfile"] = (char*)0;
		sensorConfiguration_5["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_6 = sensorConfigurations.createNestedObject();
	sensorConfiguration_6["sensorIndex"] = 6;
	if (sensorConfiguration_6["sensorIndex"] == sensorNum)
	{
		sensorConfiguration_6["active"] = true;
		sensorConfiguration_6["heaterProfile"] = heaterProfileId;
		sensorConfiguration_6["dutyCycleProfile"] = dutyCycleProfileId;
	}
	else
	{
		sensorConfiguration_6["active"] = false;
		sensorConfiguration_6["heaterProfile"] = (char*)0;
		sensorConfiguration_6["dutyCycleProfile"] = (char*)0;
	}

	JsonObject sensorConfiguration_7 = sensorConfigurations.createNestedObject();
	sensorConfiguration_7["sensorIndex"] = 7;
	if (sensorConfiguration_7["sensorIndex"] == sensorNum)
	{
		sensorConfiguration_7["active"] = true;
		sensorConfiguration_7["heaterProfile"] = heaterProfileId;
		sensorConfiguration_7["dutyCycleProfile"] = dutyCycleProfileId;
	}
	else
	{	
		sensorConfiguration_7["active"] = false;
		sensorConfiguration_7["heaterProfile"] = (char*)0;
		sensorConfiguration_7["dutyCycleProfile"] = (char*)0;
	}
	serializeJsonPretty(config, configString);
	configString = configString.substring(0,configString.length()-1);

	return EDK_OK;
}

/*!
 * @brief Function to create a bme68x label information file with .bmelabelinfo extension
 */
demoRetCode bsecDataLogger::createLabelInfoFile()
{
	demoRetCode retCode = EDK_OK;
	String macStr = utils::getMacAddress();
    String labelFileBaseName = "_Board_" + macStr + "_PowerOnOff_1_";
	uint16_t labelTag = 1001; //since labelTag starts with 1001
	
    _labelFileName = utils::getDateTime() + labelFileBaseName + utils::getFileSeed() + "_File_" + String(_bsecFileCounter) + BME68X_LABEL_INFO_FILE_EXT;
	File logFile;
    if (!logFile.open(_labelFileName.c_str(), O_RDWR | O_CREAT))
	{
		retCode = EDK_DATALOGGER_LABEL_INFO_FILE_ERROR;
	}
	else
	{
		filter2.clear();
		labelDoc.clear();
		
		/* write labelinfo header / skeleton */
		logFile.println("{");
		logFile.println("\t\"labelInfoHeader\": {");
		logFile.println("\t\t\"counterPowerOnOff\": 1,");
		logFile.println("\t\t\"seedPowerOnOff\": \"" + utils::getFileSeed() + "\",");
		logFile.println("\t\t\"dateCreated\": \"" + String(utils::getRtc().now().unixtime()) + "\",");
		logFile.println("\t\t\"dateCreated_ISO\": \"" + utils::getRtc().now().timestamp() + "+00:00\",");
		logFile.println("\t\t\"firmwareVersion\": \"" + String(FIRMWARE_VERSION) + "\",");
		logFile.println("\t\t\"boardId\": \"" + macStr + "\"");
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

		filter2["aiConfigBody"]["classes"] = true;

		File configFile;
		configFile.open(_aiConfigName.c_str(), O_RDWR);
		
		DeserializationError error = deserializeJson(labelDoc, configFile, DeserializationOption::Filter(filter2));
		configFile.close();
		if (error)
		{
			Serial.println(error.c_str());
			return EDK_SENSOR_MANAGER_JSON_DESERIAL_ERROR;
		}
		
		for (JsonObject classes : labelDoc["aiConfigBody"]["classes"].as<JsonArray>())
		{
			const char* className = classes["className"];
			
			logFile.println(",\n\t\t{");
			logFile.println("\t\t\t\"labelTag\": " + String(labelTag++) + ",");
			logFile.println("\t\t\t\"labelName\": \"" + String(className) + "\",");
			logFile.println("\t\t\t\"labelDescription\": \"Predefined by algorithm classes (className used as labelName)\"");
			logFile.print("\t\t}");
		}
		logFile.println("\n\t]");
		logFile.print("}");

		/* close log file */
		logFile.close();
		
	}
	return retCode;
}

/*!
 * @brief This function writes the bsec output to the current log file
 */
demoRetCode bsecDataLogger::writeBsecOutput(SensorIoData& buffData)
{
	demoRetCode retCode = EDK_OK;	    
	File logFile;
	
	if (_bsecFileCounter && logFile.open(_aiFileName.c_str(), O_RDWR | O_AT_END))
	{
		/* set writing position to end of data block */
		logFile.seek(_aiDataPos);
		
		float gasSignal[4] = {NAN, NAN, NAN, NAN}, iaqSignal = NAN; 
		uint8_t iaqAccuracy = 0xFF, gasAccuracy = 0xFF; 
		
		for (uint8_t i = 0; ((buffData.outputs).output != nullptr) && (i < (buffData.outputs).nOutputs); i++) 
		{
			const bsec_output_t& output = (buffData.outputs).output[i];
			switch (output.sensor_id)
			{
				case BSEC_OUTPUT_GAS_ESTIMATE_1:
				case BSEC_OUTPUT_GAS_ESTIMATE_2:
				case BSEC_OUTPUT_GAS_ESTIMATE_3:
				case BSEC_OUTPUT_GAS_ESTIMATE_4:
					gasSignal[output.sensor_id - BSEC_OUTPUT_GAS_ESTIMATE_1] = output.signal;
					gasAccuracy = (output.accuracy > gasAccuracy) ? gasAccuracy : output.accuracy;
				break;
				case BSEC_OUTPUT_IAQ:
					iaqSignal = output.signal;
					iaqAccuracy = output.accuracy;
				break;
				default:
				break;
			}
		}
		
		if (!_firstLine)
		{
			logFile.println(",");
		}		
		logFile.print("\t\t\t[\n\t\t\t\t");
		logFile.print(buffData.sensorNum);
		logFile.print(",\n\t\t\t\t");
		logFile.print(buffData.sensorId);
		logFile.print(",\n\t\t\t\t");
		logFile.print(buffData.startTimeSincePowerOn);
		logFile.print(",\n\t\t\t\t");
		logFile.print(buffData.endTimeSincePowerOn);
		logFile.print(",\n\t\t\t\t");
		(!isnan(gasSignal[0])) ? logFile.print(gasSignal[0]) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(!isnan(gasSignal[1])) ? logFile.print(gasSignal[1]) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(!isnan(gasSignal[2])) ? logFile.print(gasSignal[2]) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(!isnan(gasSignal[3])) ? logFile.print(gasSignal[3]) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(gasAccuracy != 0xFF) ? logFile.print(gasAccuracy) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(!isnan(iaqSignal)) ? logFile.print(iaqSignal) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		(iaqAccuracy != 0xFF) ? logFile.print(iaqAccuracy) : logFile.print("null");
		logFile.print(",\n\t\t\t\t");
		logFile.print(buffData.code);
		logFile.print(",\n\t\t\t\t");
		logFile.println(buffData.groundTruth);
		logFile.print("\t\t\t]");
		
		_aiDataPos = logFile.position();
		_firstLine = false;
		logFile.println("\n\t\t]\n\t}\n}");
		
		logFile.close();
	}
	else 
	{
		retCode = EDK_DATALOGGER_LOG_FILE_ERROR;
	}
    return retCode;	
}

/*!
 * @brief This function writes the sensor data to the current log file.
 */ 
demoRetCode bsecDataLogger::writeSensorData(const uint8_t* num, const uint32_t* sensorId, const bme68x_data* bme68xData, const uint32_t* scanCycleIndex, uint32_t groundTruth, demoRetCode code, SensorIoData& buffData)
{
	demoRetCode retCode = EDK_OK;
    uint32_t rtcTsp = utils::getRtc().now().unixtime();
    uint32_t timeSincePowerOn = millis();

	//Copying time for sync with aiprediction file
	if(bme68xData != nullptr)
	{
		if(bme68xData->gas_index == 0)
		{
			buffData.startTimeSincePowerOn = timeSincePowerOn;
		}
		else
		{
			buffData.endTimeSincePowerOn = timeSincePowerOn;
		}
	}

	File logFile;
	logFile.open(_bmeFileName.c_str(), FILE_WRITE);
	logFile.seek(_bmeDataPos);

	if (_endOfLine)
	{
		logFile.print(",\n");
	}
	logFile.print("\t\t\t[\n\t\t\t\t");
	(num != nullptr) ? logFile.print((int)*num) : logFile.print("null");
	logFile.print(",\n\t\t\t\t");
	(sensorId != nullptr) ? logFile.print((int)*sensorId) : logFile.print("null");
	logFile.print(",\n\t\t\t\t");
	logFile.print(timeSincePowerOn);
	logFile.print(",\n\t\t\t\t");
	logFile.print(rtcTsp);
	logFile.print(",\n\t\t\t\t");
	(bme68xData != nullptr) ? logFile.print(bme68xData->temperature) : logFile.print("null");
	logFile.print(",\n\t\t\t\t");
	(bme68xData != nullptr) ? logFile.print(bme68xData->pressure * .01f) : logFile.print("null");
	logFile.print(",\n\t\t\t\t");
	(bme68xData != nullptr) ? logFile.print(bme68xData->humidity) : logFile.print("null");
	logFile.print(",\n\t\t\t\t");
	(bme68xData != nullptr) ? logFile.print(bme68xData->gas_resistance) : logFile.print("null");
	logFile.print(",\n\t\t\t\t");
	(bme68xData != nullptr) ? logFile.print((int)bme68xData->gas_index) : logFile.print("null");
	logFile.print(",\n\t\t\t\t");
	logFile.print((bool)(BME68X_PARALLEL_MODE));
	logFile.print(",\n\t\t\t\t");
	(scanCycleIndex != nullptr) ? logFile.print((int)(*scanCycleIndex)) : logFile.print("null");
	logFile.print(",\n\t\t\t\t");
	logFile.print(groundTruth);
	logFile.print(",\n\t\t\t\t");
	logFile.print((int)code);
	logFile.print("\n\t\t\t]");

	_bmeDataPos = logFile.position();
	
	logFile.print("\n\t\t]\n\t}\n}");
	
	/* close log file */
	logFile.close();
	_endOfLine = true;
   
    return retCode;
}
