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
 * @file	ble_controller.h
 * @date	    03 Jan 2024
 * @version		2.1.5
 * 
 * @brief	Header file for the ble controller
 * 
 * 
 */

#ifndef BLE_CONTROLLER_H
#define BLE_CONTROLLER_H

/* Include Arduino Core */
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <sstream>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BLECharacteristic.h"
#include <ArduinoJson.h>
#include "label_provider.h"
#include "demo_app.h"

/* Bluetooth UART UUID's */
#define SERVICE_UUID           		"6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX 		"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX 		"6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_MSG_QUEUE_LEN			UINT8_C(3)
#define BLE_JSON_DOC_SIZE			UINT16_C(2048)
#define BLE_CONTROLLER_NOTIF_SIZE	UINT16_C(600)
#define BLE_MAX_MTU_SIZE			UINT16_C(512)

static bool device_connected = false;
static bool old_device_connected = false;

/*!
 * @brief Class library for the ble controller
 */
class bleController: public BLECharacteristicCallbacks
{
public:
	/*!
	 * @brief ble communication status
	 */
	enum cmd_status
	{
		CMD_VALID,
		CMD_INVALID,
		CONTROLLER_QUEUE_FULL,
		LABEL_INVALID,
		BSEC_SELECTED_SENSOR_INVALID,
		BSEC_CONFIG_FILE_ERROR,
		BSEC_INIT_ERROR,
		BSEC_SET_CONFIG_ERROR,
		BSEC_UPDATE_SUBSCRIPTION_ERROR,
		BSEC_RUN_ERROR,
		BSEC_OUTPUT_EXCESS_ERROR,
		SENSOR_CONFIG_MISSING,
		SENSOR_INITIALIZATION_FAILED,
		SD_CARD_INIT_ERROR,
		CONFIG_FILE_ERROR,
		APP_ALREADY_IN_STREAMING_MODE,
		SENSOR_READ_ERROR,
		BSEC_CONFIG_FILE_MISSING,
		AI_CONFIG_FILE_MISSING,
		LABEL_INFO_FILE_MISSING,
		INVALID_APP_MODE,
		LABEL_FILE_OPEN_FAILED,
		MAX_LABEL_NAME_REACHED,
		MAX_LABEL_DESCRIPTION_REACHED,
		FILE_OPEN_ERROR,
		DESERIALIZATION_FAILED,
		LABEL_NOT_FOUND,
		AI_CONFIG_AND_SUBSCRIPTION_MISSMATCH
	};
	
	/*!
	 * @brief bsec sample rate enumeration
	 */
	enum bsec_sample_rate
	{
		ULP,
		LP,
		HP
	};

	/*!
	 * @brief config file type enumeration
	 */
	enum config_file
	{
		BMECONFIG,
		AICONFIG
	};

	/*!
	 * @brief bluetooth message id enumeration
	 */
	enum ble_msg_id
	{
		GET_LABEL_INFO,
		SET_LABEL_INFO,
		SET_LABEL,
		GET_RTC_TIME,
		SET_RTC_TIME,
		START_STREAMING,
		STOP_STREAMING,
		READ_CONFIG,
		SET_APPMODE,
		GET_APPMODE,
		SET_GROUNDTRUTH,
		GET_FW_VERSION
	};
	
	/*!
	 * @brief bluetooth bsec message
	 */
	struct ble_bsec_msg
	{
		uint8_t	 	selected_sensor;
		uint8_t		sample_rate;
		uint8_t		len;
		uint8_t	 	output_id[BSEC_NUMBER_OUTPUTS];
	};
	
	/*!
	 * @brief label information
	 */
	struct ble_label_info
	{
		uint32_t	 	label;
		char label_name[LABEL_NAME_SIZE + 1];
		char label_desc[LABEL_DESC_SIZE + 1];
	};
	
	
	/*!
	 * @brief bluetooth message structure
	 */
	struct ble_msg
	{
		const char *name;
		ble_msg_id id;
		union
		{
			ble_bsec_msg	bsec;
			uint32_t	 	label;
			uint32_t		rtc_time;
			config_file		file_type;
			uint8_t			mode;
			ble_label_info 	label_info;
			uint32_t		ground_truth;
		};
	};
	
	/*!
	 * @brief bluetooth command structure
	 */
	struct ble_cmd
	{
		const char *name;
		cmd_status (*parse)(std::stringstream& ss, ble_msg& msg);
		ble_msg_id id;
	};
	
	typedef void (*bleCallBack)(const ble_msg &msg, JsonDocument& jsonDoc);
	
    /*!
     * @brief : The constructor of the bleController class creates an instance of the class
	 * 
	 * @param[in] callBack : ble callBack called when a message is dequeued.
     */
	bleController(bleCallBack callBack);
	
	/*!
	 * @brief :  This function initializes the ble controller.
     * 
     * @return bosch error code
	 */
	demo_ret_code begin();
	
	/*!
	 * @brief : This function dequeues the last received ble message. The ble callBack
	 *		   is called if a new message is available.
     * 
     * @return true if a new message is available else false
	 */
	bool dequeue_ble_msg(void);
	
	/*!
	 * @brief :  This function send a json formatted notification.
	 *
	 * @param[out] jsonDoc : json formatted message
	 */
	void send_notification(JsonDocument& jsonDoc);
	
	/*!
	 * @brief :  This function gets called when data is received from a bluetooth device.
	 *		   	 It will read in the sent bluetooth command.
	 */
	void onWrite(BLECharacteristic *pCharacteristic);

	/*!
	 * @brief : This function checks the ble connection status, restarts advertising if disconnected
	 */
	void check_ble_connection_sts();
	
private:
	bleCallBack						_callBack;
	
	static QueueHandle_t 			msg_queue;
	static ble_cmd					    cmd_list[];
	static BLECharacteristic	*ble_char_tx, *ble_char_rx;
	static BLEServer 				  *pServer;

	/*!
	 * @brief : This function fetches the RTC time which is requested through ble command
	 */
	static cmd_status parse_cmd_get_rtc_time(std::stringstream& ss, ble_msg& msg);

	/*!
	 * @brief : This function parses the RTC time received from the ble device and updates 
	 *        	the RTC time to the ble structure
	 */
	static cmd_status parse_cmd_set_rtc_time(std::stringstream& ss, ble_msg& msg);

	/*!
	 * @brief : This function fetches the current label information from the .bmelabelinfo file
	 */
	static cmd_status parse_cmd_get_label_info(std::stringstream& ss, ble_msg& msg);

	/*!
	 * @brief : This function updates the received label information to the ble structure
	 */
	static cmd_status parse_cmd_set_label_info(std::stringstream& ss, ble_msg& msg);

	/*!
	 * @brief : This function updates the received label to the ble structure
	 */
	static cmd_status parse_cmd_set_label(std::stringstream& ss, ble_msg& msg);
	
	/*!
	 * @brief : This function launches sensor data or sensor data and BSEC output streaming through ble
	 *			based on the app mode
	 */
	static cmd_status parse_cmd_start_streaming(std::stringstream& ss, ble_msg& msg);
	
	/*!
	 * @brief : This function stops ble streaming
	 */
	static cmd_status parse_cmd_stop_streaming(std::stringstream& ss, ble_msg& msg);
	
	/*!
	 * @brief : This function launches the config file data through ble
	 */
	static cmd_status parse_cmd_read_config(std::stringstream& ss, ble_msg& msg);

	/*!
	 * @brief : This function updates the current Appmode
	 */
	static cmd_status parse_cmd_set_appmode(std::stringstream& ss, ble_msg& msg);

	/*!
	 * @brief : This function retrieves the current Appmode through ble
	 */
	static cmd_status parse_cmd_get_appmode(std::stringstream& ss, ble_msg& msg);

	/*!
	 * @brief : This function updates the Groundtruth through ble
	 */
	static cmd_status parse_cmd_set_groundtruth(std::stringstream& ss, ble_msg& msg);

	/*!
	 * @brief : This function retrieves the current firmware version through ble
	 */
	static cmd_status parse_cmd_get_fw_version(std::stringstream& ss, ble_msg& msg);
};

class serverCallbacks: public BLEServerCallbacks
{
	void onConnect(BLEServer* pServer)
	{
		device_connected = true;
	}

	void onDisconnect(BLEServer* pServer)
	{
		device_connected = false;
	}
};

#endif
