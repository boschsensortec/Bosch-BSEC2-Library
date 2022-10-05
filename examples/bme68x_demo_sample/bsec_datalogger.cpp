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
 * @date	    22 June 2022
 * @version		1.5.5
 * 
 * @brief    	bsec_datalogger
 *
 * 
 */

/* own header include */
#include "bsec_datalogger.h"
#include <math.h>
#include <Esp.h>

/*!
 * @brief The constructor of the bsec_datalogger class 
 */
bsecDataLogger::bsecDataLogger() : _fileCounter(0)
{}

/*!
 * @brief This function configures the bsec datalogger using the provided bsec config string file
 */
demoRetCode bsecDataLogger::begin(const String& configName)
{
	demoRetCode retCode = utils::begin();
	
	_bsecConfigName = configName;
	if (retCode >= EDK_OK)
	{
		retCode = createBsecFile();
	}
	return retCode;
}
 
/*!
 * @brief This function creates a bsec output file
 */
demoRetCode bsecDataLogger::createBsecFile()
{
	String macStr = utils::getMacAddress();
	uint8_t configStr[BSEC_MAX_PROPERTY_BLOB_SIZE];
	
	demoRetCode retCode = utils::getBsecConfig(_bsecConfigName, configStr);
    String bsecFileBaseName = "_Board_" + macStr + "_PowerOnOff_1_";
	
    _bsecFileName = utils::getDateTime() + bsecFileBaseName + utils::getFileSeed() + "_File_" + 
														String(_fileCounter) + BSEC_DATA_FILE_EXT;
	
	if (retCode == EDK_OK)
	{
		File logFile;
		if (!logFile.open(_bsecFileName.c_str(), O_RDWR | O_CREAT))
		{
			retCode = EDK_DATALOGGER_LOG_FILE_ERROR;
		}
		else 
		{
			String base64ConfigStr = base64::encode(configStr, BSEC_MAX_PROPERTY_BLOB_SIZE);
			
			logFile.println("{");
			logFile.println("    \"bsecBase64ConfigString\": \"" + base64ConfigStr + "\",");
			logFile.println("    \"bsecDataHeader\":");
			logFile.println("\t{");
			logFile.println("\t    \"counterPowerOnOff\": 1,");
			logFile.println("\t    \"seedPowerOnOff\": \"" + utils::getFileSeed() + "\",");
			logFile.println("\t    \"counterFileLimit\": " + String(_fileCounter) + ",");
			logFile.println("\t    \"dateCreated\": \"" + String(utils::getRtc().now().unixtime()) + "\",");
			logFile.println("\t    \"dateCreated_ISO\": \"" + utils::getRtc().now().timestamp() + "+00:00\",");
			logFile.println("\t    \"firmwareVersion\": \"" + String(FIRMWARE_VERSION) + "\",");
			logFile.println("\t    \"boardId\": \"" + macStr + "\"");
			logFile.println("\t},");
			logFile.println("    \"bsecDataBody\":");
			logFile.println("\t{");
			logFile.println("\t    \"dataColumns\": [");
			logFile.println("\t\t{");
			logFile.println("\t\t    \"name\": \"Sensor Index\",");
			logFile.println("\t\t    \"unit\": \"\",");
			logFile.println("\t\t    \"format\": \"integer\",");
			logFile.println("\t\t    \"key\": \"sensorIndex\"");
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
			logFile.println("\t\t    \"key\": \"timeSincePowerOn\"");
			logFile.println("\t\t},");
			logFile.println("\t\t{");
			logFile.println("\t\t    \"name\": \"Real time clock\",");
			logFile.println("\t\t    \"unit\": \"Unix Timestamp: seconds since Jan 01 1970. (UTC); 0 = missing\",");
			logFile.println("\t\t    \"format\": \"integer\",");
			logFile.println("\t\t    \"key\": \"realTimeClock\"");
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
			logFile.println("\t\t    \"key\": \"relativeHumidity\"");
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
			logFile.println("\t\t},");
			logFile.println("\t\t{");
			logFile.println("\t\t    \"name\": \"Gas estimate 1\",");
			logFile.println("\t\t    \"unit\": \"\",");
			logFile.println("\t\t    \"format\": \"float\",");
			logFile.println("\t\t    \"key\": \"gas_estimate_1\"");
			logFile.println("\t\t},");
			logFile.println("\t\t{");
			logFile.println("\t\t    \"name\": \"Gas estimate 2\",");
			logFile.println("\t\t    \"unit\": \"\",");
			logFile.println("\t\t    \"format\": \"float\",");
			logFile.println("\t\t    \"key\": \"gas_estimate_2\"");
			logFile.println("\t\t},");
			logFile.println("\t\t{");
			logFile.println("\t\t    \"name\": \"Gas estimate 3\",");
			logFile.println("\t\t    \"unit\": \"\",");
			logFile.println("\t\t    \"format\": \"float\",");
			logFile.println("\t\t    \"key\": \"gas_estimate_3\"");
			logFile.println("\t\t},");
			logFile.println("\t\t{");
			logFile.println("\t\t    \"name\": \"Gas estimate 4\",");
			logFile.println("\t\t    \"unit\": \"\",");
			logFile.println("\t\t    \"format\": \"float\",");
			logFile.println("\t\t    \"key\": \"gas_estimate_4\"");
			logFile.println("\t\t},");
			logFile.println("\t\t{");
			logFile.println("\t\t    \"name\": \"Gas estimate accuracy\",");
			logFile.println("\t\t    \"unit\": \"\",");
			logFile.println("\t\t    \"format\": \"integer\",");
			logFile.println("\t\t    \"key\": \"gas_estimate_accuracy\"");
			logFile.println("\t\t},");
			logFile.println("\t\t{");
			logFile.println("\t\t    \"name\": \"IAQ\",");
			logFile.println("\t\t    \"unit\": \"\",");
			logFile.println("\t\t    \"format\": \"float\",");
			logFile.println("\t\t    \"key\": \"iaq\"");
			logFile.println("\t\t},");
			logFile.println("\t\t{");
			logFile.println("\t\t    \"name\": \"IAQ accuracy\",");
			logFile.println("\t\t    \"unit\": \"\",");
			logFile.println("\t\t    \"format\": \"integer\",");
			logFile.println("\t\t    \"key\": \"iaqAccuracy\"");
			logFile.println("\t\t}");
			logFile.println("\t    ],");
			
			/* data block */
			logFile.println("\t    \"dataBlock\": [");
			/* save position in file, where to write the first data set */
			_bsecDataPos = logFile.position();
			logFile.println("\t    ]");
			logFile.println("\t}");
			logFile.println("}");
			
			/* close log file */
			logFile.close();
			
			_firstLine = true;
			++_fileCounter;
		}
	}
    return retCode;
}

/*!
 * @brief This function writes the block of bsec output to the current log file
 */
demoRetCode bsecDataLogger::writeBsecOutput(SensorIoData buffData[], uint8_t buffSize)
{
	demoRetCode retCode = EDK_OK;
		    
	File logFile;
	
	if (_fileCounter && logFile.open(_bsecFileName.c_str(), O_RDWR | O_AT_END))
	{
		if (buffData != NULL) 
		{
			/* set writing position to end of data block */
			logFile.seek(_bsecDataPos);
			
			for (uint8_t j = 0; j < buffSize; j++)
			{
				float gasSignal[4] = {NAN, NAN, NAN, NAN}, iaqSignal = NAN; 
				uint8_t iaqAccuracy = 0xFF, gasAccuracy = 0xFF; 
				
				for (uint8_t i = 0; ((buffData[j].outputs).output != nullptr) && (i < (buffData[j].outputs).nOutputs); i++) 
				{
					const bsec_output_t& output = (buffData[j].outputs).output[i];
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
				logFile.print("\t\t[");
				logFile.print(buffData[j].sensorNum);
				logFile.print(",");
				logFile.print(buffData[j].sensorId);
				logFile.print(",");
				logFile.print(buffData[j].timeSincePowerOn);
				logFile.print(",");
				logFile.print(buffData[j].rtcTsp);
				logFile.print(",");
				logFile.print((buffData[j].inputData).temperature);
				logFile.print(",");
				logFile.print((buffData[j].inputData).pressure * .01f);
				logFile.print(",");
				logFile.print((buffData[j].inputData).humidity);
				logFile.print(",");
				logFile.print((buffData[j].inputData).gas_resistance);
				logFile.print(",");
				logFile.print((buffData[j].inputData).gas_index);
				logFile.print(",");
				logFile.print(buffData[j].sensorMode == BME68X_PARALLEL_MODE);
				logFile.print(",");
				logFile.print(buffData[j].label);
				logFile.print(",");
				logFile.print(buffData[j].code);
				logFile.print(",");
				(!isnan(gasSignal[0])) ? logFile.print(gasSignal[0]) : logFile.print("null");
				logFile.print(",");
				(!isnan(gasSignal[1])) ? logFile.print(gasSignal[1]) : logFile.print("null");
				logFile.print(",");
				(!isnan(gasSignal[2])) ? logFile.print(gasSignal[2]) : logFile.print("null");
				logFile.print(",");
				(!isnan(gasSignal[3])) ? logFile.print(gasSignal[3]) : logFile.print("null");
				logFile.print(",");
				(gasAccuracy != 0xFF) ? logFile.print(gasAccuracy) : logFile.print("null");
				logFile.print(",");
				(!isnan(iaqSignal)) ? logFile.print(iaqSignal) : logFile.print("null");
				logFile.print(",");
				(iaqAccuracy != 0xFF) ? logFile.print(iaqAccuracy) : logFile.print("null");
				logFile.print("]");
				
				_bsecDataPos = logFile.position();
				_firstLine = false;
			}
			logFile.println("\n\t    ]\n\t}\n}");
		}
		else
		{
			retCode = EDK_BUFFER_DATA_ERROR;
		}
		logFile.close();
	}
	else 
	{
		retCode = EDK_DATALOGGER_LOG_FILE_ERROR;
	}
    return retCode;	
}
