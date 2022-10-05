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
 * @date	22 June 2022
 * @version	1.5.5
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
#include "utils.h"
#include "demo_app.h"
#include "label_provider.h"

/*!
 * @brief Class library that holds functionality of the bsec datalogger
 */
class bsecDataLogger
{
private:
	String _bsecConfigName, _bsecFileName;
	unsigned long _bsecDataPos = 0;
    int _fileCounter = 0;
	bool _firstLine = false;
	
	/*!
	 * @brief : This function creates a bsec output file
	 * 
     * @return  bosch error code
	 */
	demoRetCode createBsecFile();

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
    
  		/*! sensor mode */
		uint8_t sensorMode;
    
    	/*! bme68x data registers data */
		bme68x_data inputData;
	    
    	/*! bsec output structure*/
    	bsecOutputs outputs;
    
		/*! gas label */
    	gasLabel label;
	
    	/*! return code */
    	demoRetCode code;
		
		/*! time since power on */
    	uint32_t timeSincePowerOn;
		
		/*! rtc time */
		uint32_t rtcTsp; 
		
	} SensorIoData;
	
	/*!
     * @brief :The constructor of the bsec_datalogger class
     *         Creates an instance of the class
     */
    bsecDataLogger();
	
	/*!
	 * @brief : This function configures the bsec datalogger using the provided bsec config string file.
	 * 
	 * @param[in] configName : sensor configuration file
     * 
     * @return  bosch error code
	 */
    demoRetCode begin(const String& configName);

	/*!
	 * @brief : This function writes the block of bsec output to the current log file.
	 * 
	 * @param[in] buffData : array of structure SensorIoData
     * @param[in] buffSize : size of a buffer
     * 
	 * @return bosch error code
	 */
	
    demoRetCode writeBsecOutput(SensorIoData buffData[], uint8_t buffSize);
};

#endif
