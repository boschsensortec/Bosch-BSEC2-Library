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
 * @file	bsec_datalogger.h
 * @date	17 January 2023
 * @version	2.0.6
 * 
 * @brief	Header file for the bsec datalogger
 * 
 * 
 */

#ifndef BSEC_DATALOGGER_H
#define BSEC_DATALOGGER_H

/* Include of Arduino Core */
#include "Arduino.h"
#include <SdFat.h>
#include <RTClib.h>
#include <base64.h>
#include <ArduinoJson.h>
#include "utils.h"
#include "demo_app.h"
#include "label_provider.h"

#define COUNT_MAX_SIZE 2

/*!
 * @brief Class library that holds functionality of the bsec datalogger
 */
class bsecDataLogger
{
private:
	String _aiConfigName, _aiFileName, _bmeFileName, _labelFileName;
	unsigned long _aiDataPos = 0, _bmeDataPos = 0;
    int _bsecFileCounter = 1;
	bool _firstLine = false;
	bsec_version_t _version;
	bool _endOfLine = false;
	String configString;
	
	/*!
	 * @brief : This function creates a bsec output file
	 * 
     * @return  bosch error code
	 */
	demoRetCode createBsecFile();

	/*!
	 * @brief	:	This function creates a bme68x datalogger output file
	 * 
	 * @param[in] sensorNum	:	Selected sensor number
	 *
	 * @return  bosch error code
	 */
	demoRetCode createRawDataFile(uint8_t sensorNum);

	/*!
	 * @brief : Fuction that parses the config data from aiConfig file and creates config header and body
	 *
	 * @param[in] sensorNum	:	Selected sensor number
	 * 
	 * @return  bosch error code
	 */
	demoRetCode prepareConfigContent(uint8_t sensorNum);

public:
	/*!
	* @brief structure which comprises sensor input and output data to write into SD card
	*/
    typedef struct
    {
		/*! sensor number */
    	uint8_t sensorNum;
    
    	/*! sensor Index */
		uint32_t sensorId;
	    
    	/*! bsec output structure*/
    	bsecOutputs outputs;
	
    	/*! return code */
    	demoRetCode code;
		
		/*! start time since power on */
    	uint32_t startTimeSincePowerOn;
		
		/*! end time since power on */
    	uint32_t endTimeSincePowerOn;

		/*! ground truth */
		uint16_t groundTruth;
		
	} SensorIoData;
	
	uint8_t scanCycles = 0;
	
	/*!
     * @brief :The constructor of the bsec_datalogger class
     *         Creates an instance of the class
     */
    bsecDataLogger();
	
	/*!
	 * @brief : This function configures the bsec datalogger using the provided bsec config string file.
	 * 
	 * @param[in] configName 	: sensor configuration file
	 * @param[in] bsecVersion	: reference to bsec version
	 * @param[in] sensorNum 	: selected sensor number
	 * 
     * @return  bosch error code
	 */
    demoRetCode begin(const String& configName, const bsec_version_t& bsecVersion, uint8_t sensorNum);

	/*!
	 * @brief : This function writes the bsec output to the current log file.
	 * 
	 * @param[in] buffData : reference to the buffered data
     * 
	 * @return bosch error code
	 */
	
    demoRetCode writeBsecOutput(SensorIoData& buffData);

	/*!
	 * @brief : This function writes the sensor data to the current log file.
	 * 
	 * @param[in] num 				: sensor number
	 * @param[in] sensorId 			: pointer to sensor id, if NULL a null json object is inserted
	 * @param[in] sensorMode		: pointer to sensor operation mode, if NULL a null json object is inserted
	 * @param[in] bme68xData		: pointer to bme68x data, if NULL a null json object is inserted
	 * @param[in] scanCycleIndex	: pointer to sensor scanning cycle index
	 * @param[in] label 			: class label
	 * @param[in] code 				: application return code
     * 
     * @return  bosch error code
	 */
    demoRetCode writeSensorData(const uint8_t* num, const uint32_t* sensorId, const bme68x_data* bme68xData, const uint32_t* scanCycleIndex, uint32_t groundTruth, demoRetCode code, SensorIoData& buffData);

	/*!
	 * @brief : This function creates a bme68x label information file with .bmelabelinfo extension
	 * 
     * @return  bosch error code
	 */
	demoRetCode createLabelInfoFile();
};

#endif
