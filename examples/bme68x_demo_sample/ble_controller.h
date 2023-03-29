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
 * @date	    17 January 2023
 * @version		2.0.6
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
#define BLE_MSG_QUEUE_LEN			3
#define BLE_JSON_DOC_SIZE			2048
#define BLE_CONTROLLER_NOTIF_SIZE	600
#define BLE_MAX_MTU_SIZE			512

static bool deviceConnected = false;
static bool oldDeviceConnected = false;

/*!
 * @brief Class library for the ble controller
 */
class bleController: public BLECharacteristicCallbacks
{
public:
	/*!
	 * @brief ble communication status
	 */
	enum cmdStatus
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
		LABEL_NOT_FOUND
	};
	
	/*!
	 * @brief bsec sample rate enumeration
	 */
	enum bsecSampleRate
	{
		ULP,
		LP,
		HP
	};

	/*!
	 * @brief config file type enumeration
	 */
	enum configFile
	{
		BMECONFIG,
		AICONFIG
	};

	/*!
	 * @brief bluetooth message id enumeration
	 */
	enum bleMsgId
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
	struct bleBsecMsg
	{
		uint8_t	 	selectedSensor;
		uint8_t		sampleRate;
		uint8_t		len;
		uint8_t	 	outputId[BSEC_NUMBER_OUTPUTS];
	};
	
	/*!
	 * @brief label information
	 */
	struct bleLabelInfo
	{
		uint32_t	 	label;
		char labelName[LABEL_NAME_SIZE + 1];
		char labelDesc[LABEL_DESC_SIZE + 1];
	};
	
	
	/*!
	 * @brief bluetooth message structure
	 */
	struct bleMsg
	{
		const char *name;
		bleMsgId id;
		union
		{
			bleBsecMsg		bsec;
			uint32_t	 	label;
			uint32_t		rtcTime;
			configFile		fileType;
			uint8_t			mode;
			bleLabelInfo    labelInfo;
			uint32_t		groundTruth;
		};
	};
	
	/*!
	 * @brief bluetooth command structure
	 */
	struct bleCmd
	{
		const char *name;
		cmdStatus (*parse)(std::stringstream& ss, bleMsg& msg);
		bleMsgId id;
	};
	
	typedef void (*bleCallBack)(const bleMsg &msg, JsonDocument& jsonDoc);
	
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
    demoRetCode begin();
	
	/*!
	 * @brief : This function dequeues the last received ble message. The ble callBack
	 *		   is called if a new message is available.
     * 
     * @return true if a new message is available else false
	 */
    bool dequeueBleMsg(void);
	
	/*!
	 * @brief :  This function send a json formatted notification.
	 *
	 * @param[out] jsonDoc : json formatted message
	 */
    void sendNotification(JsonDocument& jsonDoc);
	
	/*!
	 * @brief :  This function gets called when data is received from a bluetooth device.
	 *		   	 It will read in the sent bluetooth command.
	 */
	void onWrite(BLECharacteristic *pCharacteristic);

	/*!
	 * @brief : This function checks the ble connection status, restarts advertising if disconnected
	 */
	void checkBleConnectionSts();
	
private:
	bleCallBack						_callBack;
	
	static QueueHandle_t 			msgQueue;
	static bleCmd					cmdList[];
	static BLECharacteristic		*bleCharTx, *bleCharRx;
	static BLEServer 				*pServer;

	/*!
	 * @brief : This function fetches the RTC time which is requested through ble command
	 */
	static cmdStatus parseCmdGetRtcTime(std::stringstream& ss, bleMsg& msg);

	/*!
	 * @brief : This function parses the RTC time received from the ble device and updates 
	 *        	the RTC time to the ble structure
	 */
	static cmdStatus parseCmdSetRtcTime(std::stringstream& ss, bleMsg& msg);

	/*!
	 * @brief : This function fetches the current label information from the .bmelabelinfo file
	 */
	static cmdStatus parseCmdGetLabelInfo(std::stringstream& ss, bleMsg& msg);

	/*!
	 * @brief : This function updates the received label information to the ble structure
	 */
	static cmdStatus parseCmdSetLabelInfo(std::stringstream& ss, bleMsg& msg);

	/*!
	 * @brief : This function updates the received label to the ble structure
	 */
	static cmdStatus parseCmdSetLabel(std::stringstream& ss, bleMsg& msg);
	
	/*!
	 * @brief : This function launches sensor data or sensor data and BSEC output streaming through ble
	 *			based on the app mode
	 */
	static cmdStatus parseCmdStartStreaming(std::stringstream& ss, bleMsg& msg);
	
	/*!
	 * @brief : This function stops ble streaming
	 */
	static cmdStatus parseCmdStopStreaming(std::stringstream& ss, bleMsg& msg);
	
	/*!
	 * @brief : This function launches the config file data through ble
	 */
	static cmdStatus parseCmdReadConfig(std::stringstream& ss, bleMsg& msg);

	/*!
	 * @brief : This function updates the current Appmode
	 */
	static cmdStatus parseCmdSetAppmode(std::stringstream& ss, bleMsg& msg);

	/*!
	 * @brief : This function retrieves the current Appmode through ble
	 */
	static cmdStatus parseCmdGetAppmode(std::stringstream& ss, bleMsg& msg);

	/*!
	 * @brief : This function updates the Groundtruth through ble
	 */
	static cmdStatus parseCmdSetGroundtruth(std::stringstream& ss, bleMsg& msg);

	/*!
	 * @brief : This function retrieves the current firmware version through ble
	 */
	static cmdStatus parseCmdGetFwVersion(std::stringstream& ss, bleMsg& msg);
};

class serverCallbacks: public BLEServerCallbacks
{
	void onConnect(BLEServer* pServer)
	{
		deviceConnected = true;
	}

	void onDisconnect(BLEServer* pServer)
	{
		deviceConnected = false;
	}
};

#endif
