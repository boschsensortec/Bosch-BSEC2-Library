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
 * @file	demo_app.h
 * @date	22 June 2022
 * @version	1.5.5
 * 
 * @brief	Header file for the bosch application definitions
 * 
 * 
 */

#ifndef DEMO_APPLICATION_H
#define DEMO_APPLICATION_H

#include "bsec2.h"

#define FIRMWARE_VERSION 				"1.5.5"

/*!
 * @brief Enumeration for demo app mode
 */
enum demoAppMode
{
	DEMO_IDLE_MODE,
	DEMO_DATALOGGER_MODE,
	DEMO_BLE_STREAMING_MODE
};

/*!
 * @brief Enumeration for the demo app return code
 */
enum demoRetCode
{
	EDK_BLE_CONTROLLER_FULL_QUEUE = -20,
	EDK_BLE_CONTROLLER_INVALID_CMD = -19,
	EDK_BLE_CONTROLLER_OUT_OF_RANGE = -18,
	EDK_BLE_CONTROLLER_OUT_OF_BOUNDS = -17,
	
	EDK_BSEC_INIT_ERROR = -16,
	EDK_BSEC_SET_CONFIG_ERROR = -15,
	EDK_BSEC_UPDATE_SUBSCRIPTION_ERROR = -14,
	EDK_BSEC_RUN_ERROR = -13,
	
	EDK_DATALOGGER_LOG_FILE_ERROR = -12,
	EDK_DATALOGGER_SENSOR_CONFIG_FILE_ERROR = -11,
	
	EDK_BME68X_DRIVER_ERROR = -10,
	
	EDK_SENSOR_MANAGER_CONFIG_FILE_ERROR = -9,
	EDK_SENSOR_MANAGER_SENSOR_INDEX_ERROR = -8,
	EDK_SENSOR_MANAGER_JSON_DESERIAL_ERROR = -7,
	EDK_SENSOR_MANAGER_JSON_FORMAT_ERROR = -6,
	
	EDK_BSEC_CONFIG_STR_FILE_ERROR = -5,
	EDK_BSEC_CONFIG_STR_READ_ERROR = -4,
	EDK_BSEC_CONFIG_STR_SIZE_ERROR = -3,
	
	EDK_SD_CARD_INIT_ERROR = -2,
	EDK_SENSOR_CONFIG_FILE_ERROR = -1,
	
	EDK_OK = 0,
	
	EDK_SENSOR_MANAGER_DATA_MISS_WARNING = 1,
	
	EDK_DATALOGGER_RTC_BEGIN_WARNING = 2,
	EDK_DATALOGGER_RTC_ADJUST_WARNING = 3,
	
	EDK_BUFFER_DATA_ERROR = -21
};

/*!
 * @brief Structure to hold heater profile data
 */
struct bme68xHeaterProfile 
{
	uint64_t sleepDuration;
	uint16_t temperature[10];
	uint16_t duration[10];
	uint8_t nbRepetitions;
	uint8_t length;
};

/*!
 * @brief Structure to hold sensor state information
 */
struct bme68xSensor
{
	bme68xDev device;
	bme68xHeaterProfile heaterProfile;
	
	uint64_t wakeUpTime;
	uint32_t id;
	bool isConfigured;
	uint8_t mode;
	uint8_t cyclePos;
	uint8_t nextGasIndex;
	int8_t i2cMask;
};

#endif
