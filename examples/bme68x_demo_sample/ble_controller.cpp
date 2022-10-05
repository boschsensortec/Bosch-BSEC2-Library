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
 * @date	    22 June 2022
 * @version		1.5.5
 * 
 * @brief    	ble_controller
 *
 * 
 */

/* own header include */
#include "ble_controller.h"

bleController::bleCmd		bleController::cmdList[] = {
    {"setlabel", &bleController::parseCmdSetLabel, bleController::SET_LABEL},
	{"getlabel", &bleController::parseCmdGetLabel, bleController::GET_LABEL},
	{"setrtctime", &bleController::parseCmdSetRtcTime, bleController::SET_RTC_TIME},
	{"getrtctime", &bleController::parseCmdGetRtcTime, bleController::GET_RTC_TIME},
	{"start", &bleController::parseCmdStartStreaming, bleController::START_STREAMING},
	{"stop", &bleController::parseCmdStopStreaming, bleController::STOP_STREAMING},
};

QueueHandle_t 			bleController::msgQueue = nullptr; 
BLECharacteristic		*bleController::bleCharTx = nullptr, *bleController::bleCharRx = nullptr;

/*!
 * @brief bleController class Constructor
 */
bleController::bleController(bleCallBack callBack) : _callBack(callBack)
{}

/*!
 * @brief function to initialize the ble controller
 */
demoRetCode bleController::begin()
{
	demoRetCode retCode = EDK_OK;
	
	msgQueue = xQueueCreate(BLE_MSG_QUEUE_LEN, sizeof(bleMsg));
	
	/* Initialize BLE with Device name */
    BLEDevice::init("BME688 Development Kit");
    /* Create Server */
    BLEServer *pServer = BLEDevice::createServer();
    /* Create UART Service */
    BLEService *pService = pServer->createService(SERVICE_UUID);
    /* add characteristics for transmitting and receiving */
    bleCharTx = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );         
    bleCharTx->addDescriptor(new BLE2902());
	
    bleCharRx = pService->createCharacteristic(
						 CHARACTERISTIC_UUID_RX,
						 BLECharacteristic::PROPERTY_WRITE
					   );
    /* set callback functions */
    bleCharRx->setCallbacks(this);
    /* start advertising */
    pService->start();
    pServer->getAdvertising()->start();
	
	return retCode;
}

bleController::cmdStatus bleController::parseCmdGetRtcTime(std::stringstream& ss, bleMsg& msg)
{
	return CMD_VALID;
}

bleController::cmdStatus bleController::parseCmdSetRtcTime(std::stringstream& ss, bleMsg& msg)
{
	uint32_t rtc;
	if (ss >> rtc)
	{
		msg.rtcTime = rtc;
		return CMD_VALID;
	}
    return CMD_INVALID;
}

bleController::cmdStatus bleController::parseCmdGetLabel(std::stringstream& ss, bleMsg& msg)
{
	return CMD_VALID;
}

bleController::cmdStatus bleController::parseCmdSetLabel(std::stringstream& ss, bleMsg& msg)
{
	int label;
	if (ss >> label)
	{
		msg.label = static_cast<uint8_t>(label);
		return CMD_VALID;
	}
    return CMD_INVALID;
}

bleController::cmdStatus bleController::parseCmdStartStreaming(std::stringstream& ss, bleMsg& msg)
{
	int sensorNum, sampleRate, outputId;
	if (ss >> sensorNum)
	{
		msg.bsec.selectedSensor = static_cast<uint8_t>(sensorNum);
		if (ss >> sampleRate)
		{
			msg.bsec.sampleRate = static_cast<uint8_t>(sampleRate);
			msg.bsec.len = 0;
			
			while ((msg.bsec.len < BSEC_NUMBER_OUTPUTS) && (ss >> outputId))
			{
				msg.bsec.outputId[msg.bsec.len++] = static_cast<uint8_t>(outputId);
			}
			
			if (ss >> outputId)
			{
				return BSEC_OUTPUT_EXCESS_ERROR;
			}	
			return CMD_VALID;
		}
	}
    return CMD_INVALID;
}

bleController::cmdStatus bleController::parseCmdStopStreaming(std::stringstream& ss, bleMsg& msg)
{
	return CMD_VALID;
}

/*!
 * @brief function gets called when data is received from a bluetooth device.
 * 		  It will read in the sent bluetooth command
 */
void bleController::onWrite(BLECharacteristic *pCharacteristic)
{
    std::string rxValue = pCharacteristic->getValue(), cmdName;
	std::stringstream ss(rxValue);
	cmdStatus status = CMD_INVALID;
	bleMsg msg;
	
	if (ss >> cmdName)
	{
		StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
		
		for (auto& cmd : cmdList)
		{
			if (cmdName == cmd.name)
			{
				status = cmd.parse(ss, msg);
				if (status == CMD_VALID)
				{
					msg.name = cmd.name;
					msg.id = cmd.id;
					
					if (xQueueSendFromISR(msgQueue, (const void*)&msg, 0) == pdPASS)
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
		
		jsonDoc[cmdName.c_str()] = status;
		sendNotification(jsonDoc);
	}
}

/**
 * @brief function dequeues the last received ble message. The ble callBack is
 * 		  called if a new message is available.
 */
bool bleController::dequeueBleMsg(void)
{
	bleMsg msg;
	if (xQueueReceive(msgQueue, &msg, 0) == pdPASS)
	{
		StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
		if (_callBack != nullptr)
		{
			_callBack(msg, jsonDoc);
		
			sendNotification(jsonDoc);
		}		
		return true;
	}
	return false;
}

/*!
 * @brief function to send a json formatted notification
 */
void bleController::sendNotification(JsonDocument& jsonDoc)
{
	String notif, msg;
	
	serializeJson(jsonDoc, notif);
	
	size_t notLen = notif.length(), beginMsg = 0, endMsg = BLE_CONTROLLER_NOTIF_SIZE;
	while (beginMsg < notLen)
	{
		if (endMsg > notLen)
		{
			endMsg = notLen;
		}
		msg = notif.substring(beginMsg, endMsg);
		
		beginMsg += BLE_CONTROLLER_NOTIF_SIZE;
		endMsg += BLE_CONTROLLER_NOTIF_SIZE;
		
		bleCharTx->setValue((const char*)msg.c_str());
		bleCharTx->notify();
	}
}
