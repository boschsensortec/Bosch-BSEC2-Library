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
 * @file	bme68x_datalogger.h
 * @date	22 June 2022
 * @version	1.5.5
 * 
 * @brief	Header file for the bme68x datalogger
 * 
 * 
 */

#ifndef BME68X_DATALOGGER_H
#define BME68X_DATALOGGER_H

/* Include of Arduino Core */
#include "Arduino.h"
#include <SdFat.h>
#include <RTClib.h>
#include <base64.h>
#include "utils.h"
#include "demo_app.h"
#include "label_provider.h"
#include <sstream>

/*!
 * @brief : Class library that holds functionality of the bme68x datalogger
 */
class bme68xDataLogger
{
private:
	String _configName, _logFileName;
	std::stringstream _ss;
	unsigned long _sensorDataPos = 0;
    int _fileCounter = 0;
    bool _endOfLine = false;
		
	/*!
	 * @brief : This function creates a bme68x datalogger output file with .bmerawdata extension
	 * 
     * @return  bosch error code
	 */
	demoRetCode createLogFile();
public:
    /*!
     * @brief : The constructor of the bme68xDataLogger class
     *        	Creates an instance of the class
     */
    bme68xDataLogger();
	
	/*!
	 * @brief : This function configures the datalogger using the provided sensor config file
	 * 
	 * @param[in] configName : sensor configuration file
     * 
     * @return  bosch error code
	 */
    demoRetCode begin(const String& configName = "");
	
	/*!
	 * @brief : This function flushes the buffered sensor data to the current log file
	 * 
     * @return  bosch error code
	 */
	demoRetCode flush();
	
	/*!
	 * @brief : This function writes the sensor data to the current log file.
	 * 
	 * @param[in] num 		: sensor number
	 * @param[in] sensorId 	: pointer to sensor id, if NULL a null json object is inserted
	 * @param[in] sensorMode: pointer to sensor operation mode, if NULL a null json object is inserted
	 * @param[in] bme68xData: pointer to bbme68x data, if NULL a null json object is inserted
	 * @param[in] label 	: class label
	 * @param[in] code 		: application return code
     * 
     * @return  bosch error code
	 */
    demoRetCode writeSensorData(const uint8_t* num, const uint32_t* sensorId, const uint8_t* sensorMode, 
												const bme68x_data* bme68xData, gasLabel label, demoRetCode code);
};

#endif