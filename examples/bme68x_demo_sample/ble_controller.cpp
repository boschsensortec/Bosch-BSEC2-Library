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
 * @file	    ble_controller.cpp
 * @date	    03 Jan 2024
 * @version		2.1.5
 * 
 * @brief    	ble_controller
 *
 * 
 */

/* own header include */
#include "ble_controller.h"

bleController::ble_cmd		bleController::cmd_list[] = {
    {"setlabel", &bleController::parse_cmd_set_label, bleController::SET_LABEL},
  	{"setlabelinfo", &bleController::parse_cmd_set_label_info, bleController::SET_LABEL_INFO},
  	{"getlabelinfo", &bleController::parse_cmd_get_label_info, bleController::GET_LABEL_INFO},
  	{"setrtctime", &bleController::parse_cmd_set_rtc_time, bleController::SET_RTC_TIME},
  	{"getrtctime", &bleController::parse_cmd_get_rtc_time, bleController::GET_RTC_TIME},
  	{"start", &bleController::parse_cmd_start_streaming, bleController::START_STREAMING},
  	{"stop", &bleController::parse_cmd_stop_streaming, bleController::STOP_STREAMING},
  	{"readconfig", &bleController::parse_cmd_read_config, bleController::READ_CONFIG},
  	{"setappmode", &bleController::parse_cmd_set_appmode, bleController::SET_APPMODE},
  	{"getappmode", &bleController::parse_cmd_get_appmode, bleController::GET_APPMODE},
  	{"setgroundtruth", &bleController::parse_cmd_set_groundtruth, bleController::SET_GROUNDTRUTH},
  	{"getfwversion", &bleController::parse_cmd_get_fw_version, bleController::GET_FW_VERSION},
	};

QueueHandle_t 			bleController::msg_queue = nullptr; 
BLECharacteristic		*bleController::ble_char_tx = nullptr, *bleController::ble_char_rx = nullptr;
BLEServer 				*bleController::pServer = nullptr;

/*!
 * @brief bleController class Constructor
 */
bleController::bleController(bleCallBack callBack) : _callBack(callBack)
{}

/*!
 * @brief function to initialize the ble controller
 */
demo_ret_code bleController::begin()
{
	demo_ret_code ret_code = EDK_OK;
	
	msg_queue = xQueueCreate(BLE_MSG_QUEUE_LEN, sizeof(ble_msg));

	/* Initialize BLE with Device name */
	BLEDevice::init("BME688 Development Kit");
	/* Create Server */
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new serverCallbacks());
    
	/* Create UART Service */
	BLEService *pService = pServer->createService(SERVICE_UUID);
	/* add characteristics for transmitting and receiving */
	ble_char_tx = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );         
	ble_char_tx->addDescriptor(new BLE2902());

	ble_char_rx = pService->createCharacteristic(
						 CHARACTERISTIC_UUID_RX,
						 BLECharacteristic::PROPERTY_WRITE
					   );
	/* set callback functions */
	ble_char_rx->setCallbacks(this);
	/* start advertising */
	pService->start();
	pServer->getAdvertising()->start();
	
	return ret_code;
}

/*!
 * @brief : This function fetches the RTC time which is requested through ble command
 */
bleController::cmd_status bleController::parse_cmd_get_rtc_time(std::stringstream& ss, ble_msg& msg)
{
	return CMD_VALID;
}

/*!
 * @brief : This function parses the RTC time received from the ble device and updates
 *        	the RTC time to the ble structure
 */
bleController::cmd_status bleController::parse_cmd_set_rtc_time(std::stringstream& ss, ble_msg& msg)
{
	uint32_t rtc;

	if (ss >> rtc)
	{
		msg.rtc_time = rtc;
		return CMD_VALID;
	}
	return CMD_INVALID;
}

/*!
 * @brief : This function updates the received label to the ble structure
 */
bleController::cmd_status bleController::parse_cmd_set_label(std::stringstream& ss, ble_msg& msg)
{
	uint32_t label;

	if (ss >> label)
	{
		msg.label = label;
		return CMD_VALID;
	}
	return CMD_INVALID;
}

/*!
 * @brief : This function fetches the current label information
 */
bleController::cmd_status bleController::parse_cmd_get_label_info(std::stringstream& ss, ble_msg& msg)
{
	return CMD_VALID;
}

/*!
 * @brief : This function updates the received label information to the ble structure
 */
bleController::cmd_status bleController::parse_cmd_set_label_info(std::stringstream& ss, ble_msg& msg)
{
	uint32_t label;
	std::string lbl_name, lbl_desc;

	if (ss >> label)
	{
		msg.label_info.label = label;

		/* read label name until comma */
		if (std::getline(ss, lbl_name, ','))
		{
			lbl_name.erase(lbl_name.begin());

			if (lbl_name.length() > LABEL_NAME_SIZE)
			{
				return MAX_LABEL_NAME_REACHED;
			}
			memset(msg.label_info.label_name, 0, (LABEL_NAME_SIZE + 1));
			strncpy(msg.label_info.label_name, lbl_name.c_str(), (LABEL_NAME_SIZE - 1));

			/* read label description until dot */
			if(std::getline(ss, lbl_desc, '.'))
			{

				if (lbl_desc.length() > LABEL_DESC_SIZE)
				{
					return MAX_LABEL_DESCRIPTION_REACHED;
				}
				memset(msg.label_info.label_desc, 0, (LABEL_DESC_SIZE + 1));
				strncpy(msg.label_info.label_desc, lbl_desc.c_str(), (LABEL_DESC_SIZE - 1));
				return CMD_VALID;
			}
		}
	}
	return CMD_INVALID;
}

/*!
 * @brief : This function launches sensor data or sensor data and BSEC output streaming through ble
 *			based on the app mode
 */
bleController::cmd_status bleController::parse_cmd_start_streaming(std::stringstream& ss, ble_msg& msg)
{
	int32_t sensor_num, sample_rate, output_id;

	if (ss >> sensor_num)
	{
		msg.bsec.selected_sensor = static_cast<uint8_t>(sensor_num);

		if (ss >> sample_rate)
		{
			msg.bsec.sample_rate = static_cast<uint8_t>(sample_rate);
			msg.bsec.len = 0;
			
			while ((msg.bsec.len < BSEC_NUMBER_OUTPUTS) && (ss >> output_id))
			{
				msg.bsec.output_id[msg.bsec.len++] = static_cast<uint8_t>(output_id);
			}
			
			if (ss >> output_id)
			{
				return BSEC_OUTPUT_EXCESS_ERROR;
			}	
			return CMD_VALID;
		}
	}
  return CMD_INVALID;
}

/*!
 * @brief : This function stops ble streaming
 */
bleController::cmd_status bleController::parse_cmd_stop_streaming(std::stringstream& ss, ble_msg& msg)
{
	return CMD_VALID;
}

/*!
 * @brief : This function launches the config file data through ble
 */
bleController::cmd_status bleController::parse_cmd_read_config(std::stringstream& ss, ble_msg& msg)
{
	int32_t file_type;

	if (ss >> file_type)
	{
		msg.file_type = static_cast<config_file>(file_type);
		return CMD_VALID;
	}
	return CMD_INVALID;
}

/*!
 * @brief : This function updates the current Appmode
 */
bleController::cmd_status bleController::parse_cmd_set_appmode(std::stringstream& ss, ble_msg& msg)
{
	int32_t mode;

	if (ss >> mode)
	{
		msg.mode = static_cast<uint8_t>(mode);
		return CMD_VALID;
	}
	return CMD_INVALID;
}

/*!
 * @brief : This function retrieves the current Appmode through ble
 */
bleController::cmd_status bleController::parse_cmd_get_appmode(std::stringstream& ss, ble_msg& msg)
{
	return CMD_VALID;
}


/*!
 * @brief : This function updates the Groundtruth
 */
bleController::cmd_status bleController::parse_cmd_set_groundtruth(std::stringstream& ss, ble_msg& msg)
{
	int32_t	ground_truth;

	if (ss >> ground_truth)
	{
		msg.ground_truth = static_cast<uint32_t>(ground_truth);
		return CMD_VALID;
	}
	return CMD_INVALID;
}

/*!
* @brief : This function retrieves the current firmware version through ble
*/
bleController::cmd_status bleController::parse_cmd_get_fw_version(std::stringstream& ss, ble_msg& msg)
{
	return CMD_VALID;
}

/*!
 * @brief function gets called when data is received from a bluetooth device.
 * 		  It will read in the sent bluetooth command
 */
void bleController::onWrite(BLECharacteristic *pCharacteristic)
{
	std::string rx_value = pCharacteristic->getValue(), cmd_name;
	std::stringstream ss(rx_value);
	cmd_status status = CMD_INVALID;
	ble_msg msg;
	
	if (ss >> cmd_name)
	{
		StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
		
		for (auto& cmd : cmd_list)
		{

			if (cmd_name == cmd.name)
			{
				status = cmd.parse(ss, msg);

				if (status == CMD_VALID)
				{
					msg.name = cmd.name;
					msg.id = cmd.id;
					
					if (xQueueSendFromISR(msg_queue, (const void*)&msg, 0) == pdPASS)
					{
						return;
					}
					else
					{
						status = CONTROLLER_QUEUE_FULL;
					}
				}
				break;
			}
		}
		
		jsonDoc[cmd_name.c_str()] = status;
		send_notification(jsonDoc);
	}
}

/*!
 * @brief : This function checks the ble connection status, restarts advertising if disconnected
 */
void bleController::check_ble_connection_sts()
{
	/* disconnecting */
  if (!device_connected && old_device_connected)
	{
    pServer->startAdvertising(); /* restart advertising, when ble is disconnected */
    old_device_connected = device_connected;
  }
  /* connecting */
  if (device_connected && !old_device_connected)
	{
    old_device_connected = device_connected;
	}
}

/**
 * @brief function dequeues the last received ble message. The ble callBack is
 * 		  called if a new message is available.
 */
bool bleController::dequeue_ble_msg(void)
{
	ble_msg msg;

	if (xQueueReceive(msg_queue, &msg, 0) == pdPASS)
	{
		StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;

		if (_callBack != nullptr)
		{
			_callBack(msg, jsonDoc);
		
			send_notification(jsonDoc);
		}		
		return true;
	}
	return false;
}

/*!
 * @brief function to send a json formatted notification
 */
void bleController::send_notification(JsonDocument& jsonDoc)
{
	String notif, msg;
	
	serializeJson(jsonDoc, notif);
	
	size_t not_len = notif.length(), begin_msg = 0, end_msg = BLE_CONTROLLER_NOTIF_SIZE;

	while (begin_msg < not_len)
	{

		if (end_msg > not_len)
		{
			end_msg = not_len;
		}
		msg = notif.substring(begin_msg, end_msg);
		
		begin_msg += BLE_CONTROLLER_NOTIF_SIZE;
		end_msg += BLE_CONTROLLER_NOTIF_SIZE;
		
		ble_char_tx->setValue((const char*)msg.c_str());
		ble_char_tx->notify();
	}
}
