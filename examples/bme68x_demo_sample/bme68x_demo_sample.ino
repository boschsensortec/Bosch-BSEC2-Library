/**
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
 * @file	bme68x_demo_sample.ino
 * @date	22 June 2022
 * @version	1.5.5
 * 
 * 
 */

/* The new sensor needs to be conditioned before the example can work reliably. You may 
 * run this example for 24hrs to let the sensor stabilize.
 */

/**
 * bme68x_demo_sample.ino :
 * This is an example code for datalogging and integration of BSEC2x library in BME688 development kit
 * which has been designed to work with Adafruit ESP32 Feather Board
 * For more information visit : 
 * https://www.bosch-sensortec.com/software-tools/software/bme688-software/
 */

#include "bme68x_datalogger.h"
#include "bsec_datalogger.h"
#include "label_provider.h"
#include "led_controller.h"
#include "sensor_manager.h"
#include "ble_controller.h"
#include <bsec2.h>
#include "utils.h"

/*! BUFF_SIZE determines the size of the buffer */
#define BUFF_SIZE 10

/*!
 * @brief : This function is called by the BSEC library when a new output is available
 *
 * @param[in] input 	: BME68X data
 * @param[in] outputs	: BSEC output data
 */
void bsecCallBack(const bme68x_data input, const bsecOutputs outputs);

/*!
 * @brief : This function handles BLE message reception 
 *
 * @param[in] msg 			: reference to the new BLE message
 * @param[inout] jsonDoc	: reference to the json formatted BLE response
 */
void bleMessageReceived(const bleController::bleMsg& msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles getlabel BLE command reception (get class label)
 *
 * @param[in] msg			: reference to the new BLE message
 * @param[inout] jsonDoc 	: reference to the json formatted BLE response
 */
void bleNotifyGetLabel(const bleController::bleMsg& msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles setlabel BLE command reception (set class label)
 *
 * @param[in] msg 		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifySetLabel(const bleController::bleMsg& msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles getrtctime BLE command reception (get RTC timestamp)
 *
 * @param[in] msg 		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifyGetRtcTime(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles setrtctime BLE command reception (set RTC timestamp)
 *
 * @param[in] msg 		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifySetRtcTime(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles start BLE command reception (start BLE streaming of BSEC data)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifyStartStreaming(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles stop BLE command reception (stop BLE streaming of BSEC data)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifyStopStreaming(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles BSEC output notification
 *
 * @param[in] outputs : reference to the new BSEC output data
 */
void bleNotifyBsecOutput(const bsecOutputs& outputs);

/*!
 * @brief : This function handles BME68X data notification
 *
 * @param[in] input : reference to the new BME68X data
 */
void bleNotifyBme68xData(const bme68x_data& input);

/*!
 * @brief : This function handles sensor manager and BME68X datalogger configuration
 *
 * @param[in] bmeExtension : reference to the bmeconfig file extension
 *
 * @return  Application return code
 */
demoRetCode configureSensorLogging(const String& bmeExtension);

/*!
 * @brief : This function handles BSEC datalogger configuration
 *
 * @param[in] bsecExtension		 : reference to the BSEC configuration string file extension
 * @param[inout] bsecConfigStr	 : pointer to the BSEC configuration string
 *
 * @return  Application return code
 */
demoRetCode configureBsecLogging(const String& bsecExtension, uint8_t bsecConfigStr[BSEC_MAX_PROPERTY_BLOB_SIZE]);

uint8_t 				bsecConfig[BSEC_MAX_PROPERTY_BLOB_SIZE];
Bsec2 					bsec2;
bleController  			bleCtlr(bleMessageReceived);
labelProvider 			labelPvr;
ledController			ledCtlr;
sensorManager 			sensorMgr;
bme68xDataLogger		bme68xDlog;
bsecDataLogger 			bsecDlog;
demoRetCode				retCode;
uint8_t					bsecSelectedSensor;
String 					bme68xConfigFile, bsecConfigFile;
demoAppMode				appMode;
gasLabel 				label;
bool 					isBme68xConfAvailable, isBsecConfAvailable;
commMux					comm;

static volatile uint8_t buffCount = 0;
static bsecDataLogger::SensorIoData buff[BUFF_SIZE];

void setup()
{
	Serial.begin(115200);
	/* Datalogger Mode is set by default */
	appMode = DEMO_DATALOGGER_MODE;
	label = BSEC_NO_CLASS;
	bsecSelectedSensor = 0;
	
	/* Initializes the label provider module */
	labelPvr.begin();
	/* Initializes the led controller module */
    ledCtlr.begin();    
	/* Initializes the SD and RTC module */
	retCode = utils::begin();
  
	if (retCode >= EDK_OK)
	{
        /* checks the availability of BME board configuration and BSEC configuration files */
		isBme68xConfAvailable = utils::getFileWithExtension(bme68xConfigFile, BME68X_CONFIG_FILE_EXT);
		isBsecConfAvailable = utils::getFileWithExtension(bsecConfigFile, BSEC_CONFIG_FILE_EXT);
		
		if (isBme68xConfAvailable)
		{
			retCode = configureSensorLogging(bme68xConfigFile);
		}
		else
		{
			retCode = EDK_SENSOR_CONFIG_FILE_ERROR;
		}
	}
  
	if (retCode >= EDK_OK)
	{
		/* initialize the ble controller */
		retCode = bleCtlr.begin();
	}
  
	if (retCode < EDK_OK)
	{
		if (retCode != EDK_SD_CARD_INIT_ERROR)
		{
			/* creates log file and updates the error codes */
			if (bme68xDlog.begin(bme68xConfigFile) != EDK_SD_CARD_INIT_ERROR)
			/* Writes the sensor data to the current log file */
			(void) bme68xDlog.writeSensorData(nullptr, nullptr, nullptr, nullptr, label, retCode);
			/* Flushes the buffered sensor data to the current log file */
			(void) bme68xDlog.flush();
		}
		appMode = DEMO_IDLE_MODE;
	}
	bsec2.attachCallback(bsecCallBack);
}

void loop() 
{
	/* Updates the led controller status */
	ledCtlr.update(retCode);
	if (retCode >= EDK_OK)
	{
		while (bleCtlr.dequeueBleMsg());
		
		/* Retrieves the current label */
		(void) labelPvr.getLabel(label);
		
		switch (appMode)
		{
			/*  Logs the bme688 sensors raw data from all 8 sensors */
			case DEMO_DATALOGGER_MODE:
			{
				uint8_t i;
              
                /* Schedules the next readable sensor */
				while (sensorMgr.scheduleSensor(i))
				{
					bme68x_data* sensorData[3];
					/* Returns the selected sensor address */
                    bme68xSensor* sensor = sensorMgr.getSensor(i);

					/* Retrieves the selected sensor data */
					retCode = sensorMgr.collectData(i, sensorData);
					if (retCode < EDK_OK)
					{
						/* Writes the sensor data to the current log file */
						retCode = bme68xDlog.writeSensorData(&i, &sensor->id, &sensor->mode, nullptr, label, retCode);
					}
					else
					{
						for (const auto data : sensorData)
						{
							if (data != nullptr)
							{
								retCode = bme68xDlog.writeSensorData(&i, &sensor->id, &sensor->mode, data, label, retCode);
							}
						}
					}
				}
				
				retCode = bme68xDlog.flush();
			}
			break;
			/* Example of BSEC library integration: gets the data from one out of 8 sensors
			   (this can be selected through application) and calls BSEC library,
			   get the outputs in app and logs the data */
			case DEMO_BLE_STREAMING_MODE:
			{
				/* Callback from the user to read data from the BME688 sensors using parallel mode/forced mode,
				   process and store outputs */
				(void) bsec2.run();
			}
			break;
			default:
			break;
		}
	}
	else
	{
		Serial.println("Error code = " + String((int) retCode));
		while(1)
		{
			/* Updates the led controller status */
			ledCtlr.update(retCode);
		}
	}
}

void bsecCallBack(const bme68x_data input, const bsecOutputs outputs, Bsec2 bsec)
{ 
	bleNotifyBme68xData(input);	
	if (outputs.nOutputs)
	{
		bleNotifyBsecOutput(outputs);
	}

	bme68xSensor *sensor = sensorMgr.getSensor(bsecSelectedSensor); /* returns the selected sensor address */
	
	if (sensor != nullptr)
	{
		buff[buffCount].sensorNum = bsecSelectedSensor;
		buff[buffCount].sensorId = sensor->id;
		buff[buffCount].sensorMode = sensor->mode;
		buff[buffCount].inputData = input;
		buff[buffCount].outputs = outputs;
		buff[buffCount].label = label;
		buff[buffCount].code = retCode;
		buff[buffCount].timeSincePowerOn = millis();
		buff[buffCount].rtcTsp = utils::getRtc().now().unixtime();
		buffCount ++;  
		
		if (buffCount == BUFF_SIZE)
		{
			retCode = bsecDlog.writeBsecOutput(buff, BUFF_SIZE);
			buffCount = 0;
		}
	}
}

void bleMessageReceived(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	switch (msg.id)
	{
		case bleController::GET_LABEL:
			bleNotifyGetLabel(msg, jsonDoc);
		break;
		case bleController::SET_LABEL:
			bleNotifySetLabel(msg, jsonDoc);
		break;
		case bleController::GET_RTC_TIME:
			bleNotifyGetRtcTime(msg, jsonDoc);
		break;
		case bleController::SET_RTC_TIME:
			bleNotifySetRtcTime(msg, jsonDoc);
		break;
		case bleController::START_STREAMING:
			bleNotifyStartStreaming(msg, jsonDoc);
		break;
		case bleController::STOP_STREAMING:
			bleNotifyStopStreaming(msg, jsonDoc);
		break;
		default:
		break;
	}
}

void bleNotifyGetLabel(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	jsonDoc["value"] = (int)label;
}

void bleNotifySetLabel(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	if (msg.label >= BSEC_CLASS_1 && msg.label <= BSEC_CLASS_4)
	{
		label = static_cast<gasLabel>(msg.label);
		jsonDoc[msg.name] = bleController::CMD_VALID;
	}
	else
	{
		jsonDoc[msg.name] = bleController::LABEL_INVALID;
	}
}

void bleNotifyGetRtcTime(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	jsonDoc["value"] = utils::getRtc().now().unixtime();
}

void bleNotifySetRtcTime(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	utils::getRtc().adjust(DateTime(msg.rtcTime));
}

void bleNotifyStartStreaming(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	bsec_virtual_sensor_t bsecOutputList[BSEC_NUMBER_OUTPUTS];
	bleController::cmdStatus bleRetCode = bleController::CMD_VALID;
	const bleController::bleBsecMsg& bsecMsg = msg.bsec;
	float sampleRate = -1;
	
	commMuxBegin(Wire, SPI);
	comm = commMuxSetConfig(Wire, SPI, bsecMsg.selectedSensor, comm);
	
	for (uint8_t i = 0; i < bsecMsg.len; i++)
	{
		bsecOutputList[i] = static_cast<bsec_virtual_sensor_t>(bsecMsg.outputId[i]);
	}
	
	switch (bsecMsg.sampleRate)
	{
		case bleController::ULP:
			sampleRate = BSEC_SAMPLE_RATE_ULP;
		break;
		case bleController::LP:
			sampleRate = BSEC_SAMPLE_RATE_LP;
		break;
		case bleController::HP:
			sampleRate = BSEC_SAMPLE_RATE_SCAN;
		break;
		default:
		break;
	}
	
	appMode = DEMO_IDLE_MODE;
	bme68xSensor *sensor = sensorMgr.getSensor(bsecMsg.selectedSensor);
	if (sensor == nullptr)
	{
		bleRetCode = bleController::BSEC_SELECTED_SENSOR_INVALID;
	}
	else if (!bsec2.begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &comm))
	{
		bleRetCode = bleController::BSEC_INIT_ERROR;
	}
	else if (isBsecConfAvailable)
	{
		if (configureBsecLogging(bsecConfigFile, bsecConfig) < EDK_OK)
		{
			bleRetCode = bleController::BSEC_CONFIG_FILE_ERROR;
		}
		else if (!bsec2.setConfig(bsecConfig))
		{
			bleRetCode = bleController::BSEC_SET_CONFIG_ERROR;
		}
		else if (!bsec2.updateSubscription(bsecOutputList, bsecMsg.len, sampleRate))
		{
			bleRetCode = bleController::BSEC_UPDATE_SUBSCRIPTION_ERROR;
		}
		else if (!bsec2.run()) 
		{
			bleRetCode = bleController::BSEC_RUN_ERROR;
		}
		else
		{
			const bme68x_heatr_conf& heaterConf = bsec2.sensor.getHeaterConfiguration();
			if (heaterConf.profile_len == 0)
			{
				jsonDoc["temperature"] = heaterConf.heatr_temp;
				jsonDoc["duration"] = ((int)heaterConf.heatr_dur) * GAS_WAIT_SHARED;
				jsonDoc["mode"] = "forced";
				
				sensor->heaterProfile.length = heaterConf.profile_len;
				sensor->heaterProfile.duration[0] = heaterConf.heatr_dur;
				sensor->heaterProfile.temperature[0] = heaterConf.heatr_temp;
				sensor->mode = BME68X_FORCED_MODE;
			}
			else if ((heaterConf.heatr_dur_prof != nullptr) && (heaterConf.heatr_temp_prof != nullptr))
			{
				JsonArray heaterDurationArray = jsonDoc.createNestedArray("duration");
				JsonArray heaterTemperatureArray = jsonDoc.createNestedArray("temperature");
				for (uint8_t i = 0; i < heaterConf.profile_len; i++) 
				{
					heaterDurationArray.add(((int)heaterConf.heatr_dur_prof[i]) * GAS_WAIT_SHARED);
					heaterTemperatureArray.add(heaterConf.heatr_temp_prof[i]);
					
					sensor->heaterProfile.duration[i] = heaterConf.heatr_dur_prof[i];
					sensor->heaterProfile.temperature[i] = heaterConf.heatr_temp_prof[i];
				}
				jsonDoc["mode"] = "parallel";
				
				sensor->heaterProfile.length = heaterConf.profile_len;
				sensor->mode = BME68X_PARALLEL_MODE;
			}
			
			bsecSelectedSensor = bsecMsg.selectedSensor;
			appMode = DEMO_BLE_STREAMING_MODE;
		}
	}
	else
	{
		bleRetCode = bleController::BSEC_CONFIG_FILE_ERROR;
	}
	jsonDoc[msg.name] = bleRetCode;
}

void bleNotifyStopStreaming(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	bsecSelectedSensor = 0;
	
	appMode = DEMO_DATALOGGER_MODE;
	
	retCode = configureSensorLogging(bme68xConfigFile);
	if (retCode < EDK_OK)
	{
		(void) bme68xDlog.writeSensorData(nullptr, nullptr, nullptr, nullptr, label, retCode);
		(void) bme68xDlog.flush();
		
		appMode = DEMO_IDLE_MODE;
	}
}

void bleNotifyBsecOutput(const bsecOutputs& outputs)
{
	StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
	JsonArray bsecOutputArray = jsonDoc.createNestedArray("bsec");
		
	for (uint8_t i = 0; i < outputs.nOutputs; i++) 
	{
		const bsec_output_t& output = outputs.output[i];
		JsonObject bsecOutputObj = bsecOutputArray.createNestedObject();
		
		bsecOutputObj["id"] = output.sensor_id;
		bsecOutputObj["signal"] = output.signal;
		bsecOutputObj["accuracy"] = output.accuracy;
	}
	
	bleCtlr.sendNotification(jsonDoc);
}

void bleNotifyBme68xData(const bme68x_data& data)
{
	StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
	JsonObject bme68xObj = jsonDoc.createNestedObject("bme68x");
	
	bme68xObj["pressure"] = data.pressure;
	bme68xObj["humidity"] = data.humidity;
	bme68xObj["temperature"] = data.temperature;
	bme68xObj["gas_resistance"] = data.gas_resistance;
	bme68xObj["gas_index"] = data.gas_index;

	bleCtlr.sendNotification(jsonDoc);
}

demoRetCode configureSensorLogging(const String& bmeConfigFile)
{
	demoRetCode ret = sensorMgr.begin(bmeConfigFile);
	if (ret >= EDK_OK)
	{
		ret = bme68xDlog.begin(bmeConfigFile);
	}
	return ret;
}

demoRetCode configureBsecLogging(const String& bsecConfigFile, uint8_t bsecConfigStr[BSEC_MAX_PROPERTY_BLOB_SIZE])
{
	memset(bsecConfigStr, 0, BSEC_MAX_PROPERTY_BLOB_SIZE);
	demoRetCode ret = bsecDlog.begin(bsecConfigFile);
	if (ret >= EDK_OK)
	{
		ret = utils::getBsecConfig(bsecConfigFile, bsecConfigStr);
	}
	return ret;
}
