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
 * @date	03 Jan 2024
 * @version	2.1.5
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
#define BME68X_LABEL_INFO_FILE_EXT 		".bmelabelinfo"
#define BME68X_CONFIG_FILE_EXT 			".bmeconfig"
#define BSEC_CONFIG_FILE_EXT 			".config"
#define AI_CONFIG_FILE_EXT 				".aiconfig"
#define AI_DATA_FILE_EXT 				".aipredictions"
#define FILE_SIZE_LIMIT 				UINT32_C(311427072)
#define TIMEZONE						2.0
#define DATA_LOG_FILE_SEED_SIZE 		UINT8_C(17)
#define PIN_SD_CS 						UINT8_C(33)


class utils
{
private:
	static uint64_t 	_tick_ms;
	static uint64_t 	_tick_over_flow_cnt;
	static SdFat		_sd;
	static RTC_PCF8523 	_rtc;
	static char 		_file_seed[DATA_LOG_FILE_SEED_SIZE];
	static uint32_t 	_file_data_pos;
	static bool			_is_conf_available;


	
	/*!
	 * @brief : This function creates the random alphanumeric file seed for the log file
	 */
	static void create_file_seed();
public:
	/*!
	 * @brief : This function is a callback function to set the correct date for modified SD-card files
	 */
	static void date_time(uint16_t* date, uint16_t* time);
	
	/*!
	 * @brief : This function initializes the module
	 *
	 * @return a bosch return code
	 */
	static demo_ret_code begin();
	
	/*!
	 * @brief : This function retrieves the rtc handle
	 *
	 * @return a reference to the rtc handle
	 */
	static RTC_PCF8523&	get_rtc();
	
	/*!
	 * @brief : This function retrieves the created file seed
	 *
	 * @return the created file seed (16 random alphanumerical characters)
	 */
	static String get_file_seed();
	
	/*!
	 * @brief : This function creates a mac address string
	 *
	 * @return the mac address string
	 */
	static String get_mac_address();
	
	/*!
	 * @brief : This function creates a date string
	 *
	 * @return the date string
	 */
	static String get_date_time();
	
	/*!
	 * @brief : This function retrieves the first file with provided file extension
	 * 
	 * @param[out] fName	: the filename found
	 * @param[in] extension	: the file extension
	 *
	 * @return the date string
	 */
	static bool get_file_with_extension(String& fName, const String& extension);
	
	/*!
	 * @brief : This function retrieves the latest file with provided file extension
	 * 
	 * @param[out] fName	: the filename found
	 * @param[in] extension	: the file extension
	 *
	 * @return the date string
	 */
	static bool get_latest_file_with_extension(String& fName, const String& extension);

	/*!
	 * @brief : This function retrives the bsec configuration string from the provided file
	 * 
	 * @param[in] fileName	: the bsec configuration filename
	 * @param[out] configStr: the configuration string
	 *
	 * @return a bosch return code
	 */
	static demo_ret_code get_bsec_config(const String& file_name, uint8_t config_str[BSEC_MAX_PROPERTY_BLOB_SIZE]);
	
	/*!
	 * @brief : This function returns the tick value (ms)
	 *
	 * @return tick value in milliseconds
	 */
	static uint64_t get_tick_ms(void);

	/*!
	 * @brief : This function reads size of bytes from the file of given fileExtension
	 *
	 * @param[in] fileExtension	: file extension to search for a file in SD card
	 * @param[in] size			: number of bytes to read
	 * @param[in] fileData		: pointer to store the read data
	 *
	 * @return	a bosch return code
	 */
	static demo_ret_code read_file(const String& file_extension, size_t size, char *file_data);
};
#endif
