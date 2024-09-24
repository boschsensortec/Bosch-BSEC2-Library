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
 * @file	    utils.cpp
 * @date		03 Jan 2024
 * @version		2.1.5
 * 
 * @brief    	utils
 *
 * 
 */

#define _GNU_SOURCE

/* own header include */
#include "utils.h"
#include <ArduinoJson.h>
#include <math.h>

uint64_t 	utils::_tick_ms;
uint64_t 	utils::_tick_over_flow_cnt;
SdFat		utils::_sd;
RTC_PCF8523 utils::_rtc;
char 		utils::_file_seed[DATA_LOG_FILE_SEED_SIZE];
uint32_t 	utils::_file_data_pos = 0;
bool		utils::_is_conf_available;

/*!
 * @brief This function creates the random alphanumeric file seed for the log file
 */
void utils::create_file_seed()
{
	/* setup an alphanumeric characters buffer to choose from */
	const char *letters = "abcdefghijklmnopqrstuvwxyz0123456789";

	/* for each character of the file seed, randomly select one from the buffer */
	for (uint32_t seed_char = 0; seed_char < 16; seed_char++)
	{
		uint32_t rand_c = random(0, 36);
		_file_seed[seed_char] = letters[rand_c];
	}
}

/*!
 * @brief This function is a callback function to set the correct date for modified SD-card files
 */
void utils::date_time(uint16_t* date, uint16_t* time)
{
	DateTime now = _rtc.now();
	/* return date using FAT_DATE macro to format fields */
	*date = FAT_DATE(now.year(), now.month(), now.day());
	/* return time using FAT_TIME macro to format fields */
	*time = FAT_TIME(now.hour(), now.minute(), now.second());
}

/*!
 * @brief This function initializes the module
 */	
demo_ret_code utils::begin()
{
	demo_ret_code ret_code = EDK_OK;
	
	if (!_sd.begin(PIN_SD_CS, SPI_EIGHTH_SPEED))
	{
		ret_code = EDK_SD_CARD_INIT_ERROR;
	}
	else if (!_rtc.begin())
	{
		ret_code = EDK_DATALOGGER_RTC_BEGIN_WARNING;
	}
	else if (!_rtc.initialized() || _rtc.lostPower())
	{
		_rtc.adjust(DateTime(F(__DATE__), F(__TIME__)) - TimeSpan((int32_t)(TIMEZONE * 3600)));
		
		ret_code = EDK_DATALOGGER_RTC_ADJUST_WARNING;
	}

	randomSeed(analogRead(0));
	
	create_file_seed();
	
	SdFile::dateTimeCallback(date_time);

	return ret_code;
}

/*!
 * @brief This function retrieves the rtc handle
 */	
RTC_PCF8523& utils::get_rtc()
{
	return _rtc;
}

/*!
 * @brief This function retrieves the created file seed
 */	
String utils::get_file_seed()
{
	return String(_file_seed);
}

/*!
 * @brief This function creates a mac address string
 */
String utils::get_mac_address()
{
	uint64_t mac = ESP.getEfuseMac();
	char *mac_ptr = (char*)&mac;
	char mac_str[13];
	
	sprintf(mac_str, "%02X%02X%02X%02X%02X%02X", mac_ptr[0], mac_ptr[1], mac_ptr[2], mac_ptr[3], mac_ptr[4], mac_ptr[5]);
	
	return String(mac_str);
}

/*!
 * @brief This function creates a date string
 */
String utils::get_date_time()
{
	char time_buffer[20];
	DateTime date = _rtc.now();
	
	sprintf(time_buffer, "%d_%02d_%02d_%02d_%02d", date.year(), date.month(), date.day(), date.hour(), date.minute());
	
	return String(time_buffer);
}

/*!
 * @brief This function retrieves the first file with provided file extension
 */
bool utils::get_file_with_extension(String& fName, const String& extension)
{
	File root;
	File file;
	char file_name[90];
	
	if (root.open("/"))
	{

		while (file.openNext(&root, O_READ))
		{

			if (file.isFile())
			{
				file.getName(file_name, sizeof(file_name));

				if (String(file_name).endsWith(extension))
				{
					file.close();
					fName = String(file_name);
					return true;
				}
			}
			file.close();
		}
	}
	return false;
}

/*!
 * @brief This function retrieves the latest file with provided file extension
 */
bool utils::get_latest_file_with_extension(String& fName, const String& extension)
{
	File root;
	File file;
	char file_name[90];
	bool flag = false;

	if (root.open("/"))
	{

		while (file.openNext(&root, O_READ))
		{

			if (file.isFile())
			{
				file.getName(file_name, sizeof(file_name));

				if (String(file_name).endsWith(extension))
				{
					file.close();
					fName = String(file_name);
					flag = true;
				}
			}
			file.close();
		}
	}

	if (flag)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*!
 * @brief This function retrives the bsec configuration string from the provided file
 */
demo_ret_code utils::get_bsec_config(const String& file_name, uint8_t config_str[BSEC_MAX_PROPERTY_BLOB_SIZE])
{
	demo_ret_code ret_code = EDK_OK;
	uint32_t config_str_len;
	
	File configFile;
	if (!configFile.open(file_name.c_str(), O_RDWR))
	{
		ret_code = EDK_BSEC_CONFIG_STR_FILE_ERROR;
	}
	else if (configFile.read(&config_str_len, sizeof(uint32_t)) != sizeof(uint32_t))
	{
		ret_code = EDK_BSEC_CONFIG_STR_READ_ERROR;
	}
	else if (config_str_len != BSEC_MAX_PROPERTY_BLOB_SIZE)
	{
		ret_code = EDK_BSEC_CONFIG_STR_SIZE_ERROR;
	}
	else if (configFile.read(config_str, config_str_len) != config_str_len)
	{
		ret_code = EDK_BSEC_CONFIG_STR_READ_ERROR;
	}
	return ret_code;
}

/*!
 * @brief This function returns the tick value (ms)
 */
uint64_t utils::get_tick_ms(void)
{
	uint64_t time_ms = millis();

	if (_tick_ms > time_ms) /* An overflow occurred */
	{ 
		_tick_over_flow_cnt++;
	}
	_tick_ms = time_ms;
	return time_ms + (_tick_over_flow_cnt * INT64_C(0xFFFFFFFF));
}

/*!
 * @brief : This function reads the size of bytes from the file of given fileExtension
 */
demo_ret_code utils::read_file(const String& file_extension, size_t size, char *file_data)
{
	File file;
	demo_ret_code ret_code;
	static String file_name;
	static bool first_time = true;
	memset(file_data, 0, size); /* Clears the previous data if any */
	ret_code = utils::begin(); /* Initializes the SD card module */

	size = size - 1;
	
	if (ret_code >= EDK_OK)
	{

		if (first_time)
		{

			if (file_extension == BME68X_LABEL_INFO_FILE_EXT)
			{
				/* check for a file with given extension */
				_is_conf_available = utils::get_latest_file_with_extension(file_name, file_extension);
			}
			else
			{
				/* check for a file with given extension */
				_is_conf_available = utils::get_file_with_extension(file_name, file_extension);
			}
		}

		if (_is_conf_available)
		{

			if (!file.open(file_name.c_str(), O_READ)) /* open the given file in read mode */
			{
				return EDK_FILE_OPEN_ERROR;
			}
			first_time = false;
			file.seek(_file_data_pos); /* sets the position of the file to a particular byte */

			if (file.available())
			{
				file.read(file_data, size); /* reads the given number of bytes of data */
				_file_data_pos = file.position(); /* save the file position indicator for next read */
				file.close(); /* closes the file */
				return EDK_OK;
			}
			file.close();
			_file_data_pos = 0; /* resets the file position indicator */
			first_time = true;
			return EDK_END_OF_FILE;
		}
		return EDK_EXTENSION_NOT_AVAILABLE;
	}
	return ret_code;
}
