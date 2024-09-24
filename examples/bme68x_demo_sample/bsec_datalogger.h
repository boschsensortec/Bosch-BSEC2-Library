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
 * @date	03 Jan 2024
 * @version	2.1.5
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
#include <sstream>
#include <iostream>

/* Number of sensors to operate*/
#define NUM_OF_SENS    		UINT8_C(4)
#define COUNT_MAX_SIZE 		UINT8_C(2)

/*!
 * @brief Class library that holds functionality of the bsec datalogger
 */
class bsecDataLogger
{
private:
	String _ai_config_name, _ai_file_name, _bme_file_name, _label_file_name;
	std::stringstream _bs, _ba;
	uint32_t _ai_data_pos = 0, _bme_data_pos = 0;
	uint32_t _bme_file_counter = 1;
	uint32_t _bsec_file_counter = 1;
	bool _first_line = false;
	bsec_version_t _version;
	bool _end_of_line = false;
	String config_string;
	
	/*!
	 * @brief : This function creates a bsec output file
	 * 
     * @return  bosch error code
	 */
	demo_ret_code create_bsec_file();

	/*!
	 * @brief	:	This function creates a bme68x datalogger output file
	 * 
	 * @param[in] sensorNum	:	Selected sensor number
	 *
	 * @return  bosch error code
	 */
	demo_ret_code create_raw_data_file(uint8_t sensor_num);

	/*!
	 * @brief : Fuction that parses the config data from aiConfig file and creates config header and body
	 *
	 * @param[in] sensorNum	:	Selected sensor number
	 * 
	 * @return  bosch error code
	 */
	demo_ret_code prepare_config_content(uint8_t sensor_num);

public:
	/*!
	* @brief structure which comprises sensor input and output data to write into SD card
	*/
  typedef struct
  {
		/*! sensor number */
  	uint8_t sensor_num;
    
  	/*! sensor Index */
		uint32_t sensor_id;
	    
  	/*! bsec output structure*/
  	bsecOutputs outputs;
	
  	/*! return code */
  	demo_ret_code code;
		
		/*! start time since power on */
  	uint32_t start_time_since_power_on;
		
		/*! end time since power on */
  	uint32_t end_time_since_power_on;

		/*! ground truth */
		uint16_t ground_truth;
		
	} sensor_io_data;
	
	uint8_t scanCycles = 0;

	/* Holds the aiconfig type (clasification / regression) */
	char ai_config_type[15] = {};
	
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
	demo_ret_code begin(const String& config_name, const bsec_version_t& bsec_version, uint8_t sensor_num);
	
	/*!
	 * @brief : This function flushes the buffered sensor data to the current log file
	 * 
     * @return  bosch error code
	 */
	demo_ret_code flush_sensor_data(uint8_t sensor_num);

	/*!
	 * @brief : This function writes the bsec output to the current log file.
	 * 
	 * @param[in] buffData : reference to the buffered data
     * 
	 * @return bosch error code
	 */
	
	demo_ret_code write_bsec_output(sensor_io_data& buff_data);

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
	demo_ret_code write_sensor_data(const uint8_t* num, const uint32_t* sensor_id, const bme68x_data* bme68xData, 
                              const uint32_t* scan_cycle_index, uint32_t ground_truth, demo_ret_code code, 
                              sensor_io_data& buff_data);

	/*!
	 * @brief : This function creates a bme68x label information file with .bmelabelinfo extension
	 * 
     * @return  bosch error code
	 */
	demo_ret_code create_label_info_file();
};

#endif
