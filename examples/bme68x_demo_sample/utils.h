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
 * @file	utils.h
 * @date	22 June 2022
 * @version	1.5.5
 * 
 * @brief	Header file for the datalogger utils
 * 
 * 
 */

#ifndef DATALOGGER_UTILS_H
#define DATALOGGER_UTILS_H

#include "Arduino.h"
#include <SdFat.h>
#include <RTClib.h>
#include "demo_app.h"

#define BME68X_RAWDATA_FILE_EXT 		".bmerawdata"
#define BME68X_CONFIG_FILE_EXT 			".bmeconfig"
#define BSEC_DATA_FILE_EXT 				".bsecdata"
#define BSEC_CONFIG_FILE_EXT 			".config"
#define FILE_SIZE_LIMIT 				311427072
#define TIMEZONE						2.0
#define DATA_LOG_FILE_SEED_SIZE 		17
#define PIN_SD_CS 						33

class utils
{
private:
	static uint64_t 	_tickMs;
	static uint64_t 	_tickOverFlowCnt;
	static SdFat		_sd;
	static RTC_PCF8523 	_rtc;
	static char 		_fileSeed[DATA_LOG_FILE_SEED_SIZE];
	
	/*!
	 * @brief : This function creates the random alphanumeric file seed for the log file
	 */
	static void createFileSeed();
public:
	/*!
	 * @brief : This function is a callback function to set the correct date for modified SD-card files
	 */
	static void dateTime(uint16_t* date, uint16_t* time);
	
	/*!
	 * @brief : This function initializes the module
	 *
	 * @return a bosch return code
	 */
	static demoRetCode begin();
	
	/*!
	 * @brief : This function retrieves the rtc handle
	 *
	 * @return a reference to the rtc handle
	 */
	static RTC_PCF8523&	getRtc();
	
	/*!
	 * @brief : This function retrieves the created file seed
	 *
	 * @return the created file seed (16 random alphanumerical characters)
	 */
	static String getFileSeed();
	
	/*!
	 * @brief : This function creates a mac address string
	 *
	 * @return the mac address string
	 */
	static String getMacAddress();
	
	/*!
	 * @brief : This function creates a date string
	 *
	 * @return the date string
	 */
	static String getDateTime();
	
	/*!
	 * @brief : This function retrieves the first file with provided file extension
	 * 
	 * @param[out] fName	: the filename found
	 * @param[in] extension	: the file extension
	 *
	 * @return the date string
	 */
	static bool getFileWithExtension(String& fName, const String& extension);
	
	/*!
	 * @brief : This function retrives the bsec configuration string from the provided file
	 * 
	 * @param[in] fileName	: the bsec configuration filename
	 * @param[out] configStr: the configuration string
	 *
	 * @return a bosch return code
	 */
	static demoRetCode getBsecConfig(const String& fileName, uint8_t configStr[BSEC_MAX_PROPERTY_BLOB_SIZE]);
	
	/*!
	 * @brief : This function returns the tick value (ms)
	 *
	 * @return tick value in milliseconds
	 */
	static uint64_t getTickMs(void);
};

#endif