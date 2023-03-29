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
 * @date	17 January 2023
 * @version	2.0.6
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

/* Macros used */
/* Number of sensors to operate*/
#define NUM_OF_SENS    4

/*! FILE_DATA_READ_SIZE determines the size of the data to be read from the file */
#define FILE_DATA_READ_SIZE	400

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
 * @brief : This function handles getlabelinfo BLE command reception (get label information)
 *
 * @param[in] msg			: reference to the new BLE message
 * @param[inout] jsonDoc 	: reference to the json formatted BLE response
 */
void bleNotifyGetLabelInfo(const bleController::bleMsg& msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles setlabelinfo BLE command reception (set label information)
 *
 * @param[in] msg 		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifySetLabelInfo(const bleController::bleMsg& msg, JsonDocument& jsonDoc);

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
 * @brief : This function handles start BLE command reception (start BLE streaming of BSEC and BME raw data)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifyStartStreaming(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles stop BLE command reception (stop BLE streaming of BSEC and BME raw data)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifyStopStreaming(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles readconfig BLE command reception (start BLE Streaming of config file data)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifyReadConfig(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles BSEC output notification
 *
 * @param[in] outputs : reference to the new BSEC output data
 */
void bleNotifyBsecOutput(const bsecOutputs& outputs, const uint8_t sensNum);

/*!
 * @brief : This function handles BME68X data notification
 *
 * @param[in] input : reference to the new BME68X data
 * @param[in] sensNum: sensor number
 */
void bleNotifyBme68xData(const bme68x_data& input, const uint8_t sensNum);

/*!
 * @brief : This function handles setappmode BLE command reception (Setting the current App mode)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifySetAppmode(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles getappmode BLE command reception (Returns the current App mode)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifyGetAppmode(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles setgroundtruth BLE command reception(Setting the Groundtruth)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifySetGroundtruth(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles getfwversion BLE command reception (Returns the current firmware version)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void bleNotifyGetFwVersion(const bleController::bleMsg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles sensor manager and BME68X datalogger configuration
 *
 * @param[in] bmeExtension : reference to the bmeconfig file extension
 *
 * @return  Application return code
 */
demoRetCode configureSensorLogging(const String& bmeExtension);

/*!
 * @brief : This function handles the read configuration File
 *
 * @param[in] configFile	: reference to the config File Name
 *
 * @return  Application return code
 */
demoRetCode configFileRead(const String& configFile);

/*!
 * @brief : This function handles BSEC datalogger configuration
 * 
 * @param[in] aiConfigFile		: reference to the AI configuration file extension
 * 
 * @param[in] bsecConfigFile	: reference to the BSEC configuration string file extension
 *
 * @param[inout] bsecConfigStr	: pointer to the BSEC configuration string
 * 
 * @param[in] sensorNum			: selected sensor number
 *
 * @return  Application return code
 */
demoRetCode configureBsecLogging(const String& aiConfigFile, const String& bsecConfigFile, uint8_t bsecConfigStr[BSEC_MAX_PROPERTY_BLOB_SIZE], uint8_t sensorNum);

/*!
 * @brief : This function handles collecting, sending sensor raw data via ble and data logging
 *
 * @param[in] sensNum :	sensor number to collect and log the data
 *
 * @return  a Bosch return code
 */
demoRetCode logSensorData(uint8_t sensNum);

uint8_t 				bsecConfig[BSEC_MAX_PROPERTY_BLOB_SIZE];
char 					configFileData[FILE_DATA_READ_SIZE];
Bsec2 					bsec2[NUM_OF_SENS];
bleController  			bleCtlr(bleMessageReceived);
labelProvider 			labelPvr;
ledController			ledCtlr;
sensorManager 			sensorMgr;
bme68xDataLogger		bme68xDlog;
bsecDataLogger 			bsecDlog;
demoRetCode				retCode;
uint8_t					selectedSensor;
String 					bme68xConfFileName, bsecConfFileName, aiConfFileName, configFileName, labelFileName;
demoAppMode				appMode, currentAppMode;
gasLabel 				label;
bool 					isBme68xConfAvailable, isBsecConfAvailable, isAIConfAvailable, isLabelInfoAvailable;
commMux					comm[NUM_OF_SENS];
uint8_t					bsecMemBlock[NUM_OF_SENS][BSEC_INSTANCE_SIZE];
uint8_t 				sensor = 0;
uint8_t 				sensor_num;
uint32_t				groundTruth;
static uint8_t 			buffCount = 0;
static bsecDataLogger::SensorIoData buff[NUM_OF_SENS];

void setup()
{
	Serial.begin(115200);
	/* Setting default mode as idle */
	appMode = DEMO_RECORDING_MODE;
	currentAppMode = DEMO_RECORDING_MODE;

	label = BSEC_NO_CLASS;

	/* Setting default sensor as sensor 0 to collect data */
	selectedSensor = NUM_BME68X_UNITS;
	
	/* Initializes the label provider module */
	labelPvr.begin();

	/* Initializes the led controller module */
    ledCtlr.begin();

	/* initialize the ble controller */
	bleCtlr.begin();
  
  	/* Initializes the SD and RTC module */
	retCode = utils::begin();
	if (retCode >= EDK_OK)
	{
		isBme68xConfAvailable = utils::getFileWithExtension(bme68xConfFileName, BME68X_CONFIG_FILE_EXT);
		/* checks the availability of BME board configuration file */
		if (isBme68xConfAvailable)
		{
			if (configureSensorLogging(bme68xConfFileName) < EDK_OK)
			{
				retCode = EDK_SENSOR_INITIALIZATION_FAILED;
			}
		}
		else
		{
			retCode = EDK_SENSOR_CONFIG_MISSING_ERROR;
		}
	}
	
	if (retCode < EDK_OK)
	{
		currentAppMode = DEMO_IDLE_MODE;
		appMode = DEMO_IDLE_MODE;
	}
}

void loop()
{
	/* Updates the led controller status */
	ledCtlr.update(retCode);
	while (bleCtlr.dequeueBleMsg());

	/*checks the ble connection status, restarts advertising if disconnected */
	bleCtlr.checkBleConnectionSts();

	if (retCode >= EDK_OK)
	{
		/* Retrieves the current label */
		(void) labelPvr.getLabel(label);
		
		switch (currentAppMode)
		{
			/*  Gets the bme688 sensors raw data in app and logs the same based on the selected sensors */
			case DEMO_RECORDING_MODE:
			{
				uint8_t i;
                
				/* Flushes the buffered sensor data to the current log file */
				retCode = bme68xDlog.flush();
				
				if (retCode >= EDK_OK)
				{
					if (selectedSensor == NUM_BME68X_UNITS)
					{
						/* Schedules the next readable sensor */
						while (sensorMgr.scheduleSensor(i))
						{
							/* Gets the given sensor raw data and sends via ble and log into the SD card */
							retCode = logSensorData(i);
						}
					}
					else
					{
						retCode = logSensorData(selectedSensor);
					}
				}
			}
			break;
			/* Example of BSEC library integration: gets the data from one out of 8 sensors
			   (this can be selected through application) and calls BSEC library,
			   get the outputs in app and logs the data */
			case DEMO_TEST_ALGORITHM_MODE:
			{
				if (selectedSensor == NUM_OF_SENS)
				{
					for (sensor_num = 0; sensor_num < NUM_OF_SENS; sensor_num++)
					{
						/* Callback from the user to read data from the BME688 sensors using parallel mode/forced mode,
						process and store outputs */
						(void) bsec2[sensor_num].run();
					}
				}
				else if(selectedSensor < NUM_OF_SENS)
				{
					(void) bsec2[selectedSensor].run();
					sensor_num = selectedSensor;
				}
			}
			break;
			default:
			break;
		}
	}
}

demoRetCode logSensorData(uint8_t sensNum)
{
	demoRetCode ret;
	bme68x_data* sensorData[3];

	/* Returns the selected sensor address */
	bme68xSensor* sensor = sensorMgr.getSensor(sensNum);

	/* Retrieves the selected sensor data */
	ret = sensorMgr.collectData(sensNum, sensorData);
	if (ret < EDK_OK)
	{
		StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
		JsonObject errorObj = jsonDoc.createNestedObject("errorCode");
		
		errorObj["sensor_number"] = sensNum;
		errorObj["error_code"] = bleController::SENSOR_READ_ERROR;

		bleCtlr.sendNotification(jsonDoc);
	}
	else
	{
		for (const auto data : sensorData)
		{
			if (data != nullptr)
			{
				bme68x_data rawData;
				rawData.temperature = data->temperature;
				rawData.pressure = data->pressure;
				rawData.humidity = data->humidity;
				rawData.gas_resistance = data->gas_resistance;
				rawData.gas_index = data->gas_index;

				/* sends the sensor raw data via ble */
				bleNotifyBme68xData(rawData, sensNum);
				/* Writes the sensor data to the current log file */
				bme68xDlog.writeSensorData(&sensNum, &sensor->id, &sensor->mode, data, &sensor->scanCycleIndex, label, retCode);
				/* Increments scanCycleIndex by one, if cycle completes */
				if (rawData.gas_index == sensor->heaterProfile.length - 1)
				{
					sensor->scanCycleIndex += 1;
					if (sensor->scanCycleIndex > sensor->heaterProfile.nbRepetitions)
					{
						sensor->scanCycleIndex = 1;
					}
				}
			}
		}
	}
	return ret;
}

void bsecCallBack(const bme68x_data input, const bsecOutputs outputs, Bsec2 bsec)
{
	/* Sending bme raw data via ble */
	bleNotifyBme68xData(input, sensor_num);
	
	/* Returns the selected sensor address */
	bme68xSensor *sensor = sensorMgr.getSensor(sensor_num);
	if(sensor != nullptr)
	{
		bsecDlog.writeSensorData(&sensor_num, &sensor->id, &input, &sensor->scanCycleIndex, groundTruth, retCode, buff[sensor_num]);

		if (input.gas_index == sensor->heaterProfile.length-1)
		{
			sensor->scanCycleIndex++;
			if (sensor->scanCycleIndex > bsecDlog.scanCycles)
			{
				sensor->scanCycleIndex = 1;
			}
		}
		
		if (outputs.nOutputs)
		{
			/* Sending BSEC output via ble */
			bleNotifyBsecOutput(outputs, sensor_num);
			if (sensor != nullptr)
			{
				buff[sensor_num].sensorNum = sensor_num;
				buff[sensor_num].sensorId = sensor->id;
				buff[sensor_num].outputs = outputs;
				buff[sensor_num].code = retCode;
				buff[sensor_num].groundTruth = groundTruth;
				/* Writing BSEC output into SD card */
				retCode = bsecDlog.writeBsecOutput(buff[sensor_num]);
			}
		}
	}
}

void bleMessageReceived(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	switch (msg.id)
	{
		case bleController::GET_LABEL_INFO:
			bleNotifyGetLabelInfo(msg, jsonDoc);
		break;
		case bleController::SET_LABEL_INFO:
			bleNotifySetLabelInfo(msg, jsonDoc);
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
		case bleController::READ_CONFIG:
			bleNotifyReadConfig(msg, jsonDoc);
		break;
		case bleController::SET_APPMODE:
			bleNotifySetAppmode(msg, jsonDoc);
		break;
		case bleController::GET_APPMODE:
			bleNotifyGetAppmode(msg, jsonDoc);
		break;
		case bleController::SET_GROUNDTRUTH:
			bleNotifySetGroundtruth(msg, jsonDoc);
		break;
		case bleController::GET_FW_VERSION:
			bleNotifyGetFwVersion(msg, jsonDoc);
		break;
		default:
		break;
	}
}

void bleNotifyGetLabelInfo(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	demoRetCode ret;
	if(currentAppMode != DEMO_IDLE_MODE)
	{
		jsonDoc[msg.name] = bleController::APP_ALREADY_IN_STREAMING_MODE;
		return;
	}
	ret = configFileRead(BME68X_LABEL_INFO_FILE_EXT);
	if(ret == EDK_END_OF_FILE)
	{
		jsonDoc.clear();
		jsonDoc.add(EOF);
	}
	else if (ret == EDK_SD_CARD_INIT_ERROR)
	{
		jsonDoc[msg.name] = bleController::SD_CARD_INIT_ERROR;
	}
	else if (ret == EDK_EXTENSION_NOT_AVAILABLE)
	{
		jsonDoc[msg.name] = bleController::LABEL_INFO_FILE_MISSING;
	}
	else
	{
		jsonDoc[msg.name] = bleController::FILE_OPEN_ERROR;
	}
}

void bleNotifySetLabelInfo(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	demoRetCode ret;
	
	if (currentAppMode == DEMO_RECORDING_MODE)
	{
		if (msg.labelInfo.label >= LABEL_TAG_MIN_RANGE && msg.labelInfo.label <= LABEL_TAG_MAX_RANGE)
		{
			ret = bme68xDlog.setLabelInfo(msg.labelInfo.label, String(msg.labelInfo.labelName), String(msg.labelInfo.labelDesc));
			if (ret == EDK_DATALOGGER_LABEL_INFO_FILE_ERROR)
			{
				jsonDoc[msg.name] = bleController::LABEL_FILE_OPEN_FAILED;
			}
			else if (ret == EDK_SENSOR_MANAGER_JSON_DESERIAL_ERROR)
			{
				jsonDoc[msg.name] = bleController::DESERIALIZATION_FAILED;
			}
			else
			{
				jsonDoc[msg.name] = bleController::CMD_VALID;
			}
		}
		else
		{
			jsonDoc[msg.name] = bleController::LABEL_INVALID;
		}
	}
	else
	{
		jsonDoc[msg.name] = bleController::INVALID_APP_MODE;
	}
}

void bleNotifySetLabel(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	if (msg.label >= LABEL_TAG_MIN_RANGE && msg.label <= LABEL_TAG_MAX_RANGE)
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

	/* Initializes the SD and RTC module */
	retCode = utils::begin();

	if (retCode >= EDK_OK)
	{
		if (appMode == DEMO_TEST_ALGORITHM_MODE)
		{
			isBsecConfAvailable = utils::getFileWithExtension(bsecConfFileName, BSEC_CONFIG_FILE_EXT);
			isAIConfAvailable = utils::getFileWithExtension(aiConfFileName, AI_CONFIG_FILE_EXT);

			/* checks the availability of BSEC configuration file */
			if (isBsecConfAvailable)
			{
				if(isAIConfAvailable)
				{
					bsec_virtual_sensor_t bsecOutputList[BSEC_NUMBER_OUTPUTS];
					float sampleRate = -1;
					
					/* Check added to ensure support for multi instance in test algorithm mode */
					if(bsecMsg.selectedSensor > NUM_OF_SENS) 
					{
						bleRetCode = bleController::BSEC_SELECTED_SENSOR_INVALID;
						jsonDoc[msg.name] = bleRetCode;
						return;
					}
					
					commMuxBegin(Wire, SPI);
					
					for (uint8_t i = 0; i < bsecMsg.len; i++)
					{
						bsecOutputList[i] = static_cast<bsec_virtual_sensor_t>(bsecMsg.outputId[i]);
					}
					
					for(uint8_t i = 0; i < NUM_OF_SENS; i++)
					{
						bme68xSensor *sensor = sensorMgr.getSensor(i);
						if (sensor == nullptr)
						{
							bleRetCode = bleController::BSEC_SELECTED_SENSOR_INVALID;
							jsonDoc[msg.name] = bleRetCode;
							return;
						}
						
						/* Sets the Communication interface for the sensors */
						comm[i] = commMuxSetConfig(Wire, SPI, i, comm[i]);
						sensor->scanCycleIndex = 1;
						
						/* Assigning a chunk of memory block to the bsecInstance */
						bsec2[i].allocateMemory(bsecMemBlock[i]);
						
						/* Whenever new data is available call the newDataCallback function */
						bsec2[i].attachCallback(bsecCallBack);
						
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
						
						if (!bsec2[i].begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &comm[i]))
						{
							bleRetCode = bleController::BSEC_INIT_ERROR;
						}
						if(i == 0)
						{
							if (configureBsecLogging(aiConfFileName, bsecConfFileName, bsecConfig, bsecMsg.selectedSensor) < EDK_OK)
							{
								bleRetCode = bleController::BSEC_CONFIG_FILE_ERROR;
								jsonDoc[msg.name] = bleRetCode;
								return;
							}
						}
						if (!bsec2[i].setConfig(bsecConfig))
						{
							bleRetCode = bleController::BSEC_SET_CONFIG_ERROR;
						}
						else if (!bsec2[i].updateSubscription(bsecOutputList, bsecMsg.len, sampleRate))
						{
							bleRetCode = bleController::BSEC_UPDATE_SUBSCRIPTION_ERROR;
						}
						else if (!bsec2[i].run())
						{
							bleRetCode = bleController::BSEC_RUN_ERROR;
						}
						else
						{
							const bme68x_heatr_conf& heaterConf = bsec2[i].sensor.getHeaterConfiguration();
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
							
							selectedSensor = bsecMsg.selectedSensor;
							currentAppMode = DEMO_TEST_ALGORITHM_MODE;
						}
					}
				}
				else
				{
					bleRetCode = bleController::AI_CONFIG_FILE_MISSING;
				}
			}
			else
			{
				bleRetCode = bleController::BSEC_CONFIG_FILE_MISSING;
			}
		}
		else
		{
			isBme68xConfAvailable = utils::getFileWithExtension(bme68xConfFileName, BME68X_CONFIG_FILE_EXT);
			/* checks the availability of BME board configuration file */
			if (isBme68xConfAvailable)
			{
				if (configureSensorLogging(bme68xConfFileName) < EDK_OK)
				{
					bleRetCode = bleController::SENSOR_INITIALIZATION_FAILED;
					retCode = EDK_SENSOR_INITIALIZATION_FAILED;
				}
				else
				{
					selectedSensor = bsecMsg.selectedSensor;
					currentAppMode = DEMO_RECORDING_MODE;
				}
			}
			else
			{
				bleRetCode = bleController::SENSOR_CONFIG_MISSING;
				retCode = EDK_SENSOR_CONFIG_MISSING_ERROR;
			}
		}
		jsonDoc[msg.name] = bleRetCode;
	}
	else
	{
		jsonDoc[msg.name] = bleController::SD_CARD_INIT_ERROR;
		retCode = EDK_SD_CARD_INIT_ERROR;
		currentAppMode = DEMO_IDLE_MODE;
	}
}

void bleNotifyStopStreaming(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	selectedSensor = 0;
	
	currentAppMode = DEMO_IDLE_MODE;
	appMode = DEMO_IDLE_MODE;
}

void bleNotifyReadConfig(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	demoRetCode ret;
	if(currentAppMode != DEMO_IDLE_MODE)
	{
		jsonDoc[msg.name] = bleController::APP_ALREADY_IN_STREAMING_MODE;
	}

	else if (msg.fileType >= bleController::BMECONFIG && msg.fileType <= bleController::AICONFIG)
	{
		if(msg.fileType == bleController::BMECONFIG)
		{
			configFileName = BME68X_CONFIG_FILE_EXT;
		}
		else
		{
			configFileName = AI_CONFIG_FILE_EXT;
		}
		ret = configFileRead(configFileName);
		if(ret == EDK_END_OF_FILE)
		{
			jsonDoc.clear();
			jsonDoc.add(EOF);
		}
		else if (ret == EDK_SD_CARD_INIT_ERROR)
		{
			jsonDoc[msg.name] = bleController::SD_CARD_INIT_ERROR;
		}
		else if (ret == EDK_EXTENSION_NOT_AVAILABLE)
		{
			if(configFileName == BME68X_CONFIG_FILE_EXT)
			{
				jsonDoc[msg.name] = bleController::SENSOR_CONFIG_MISSING;
			}
			else
			{
				jsonDoc[msg.name] = bleController::AI_CONFIG_FILE_MISSING;
			}
		}
		else
		{
			jsonDoc[msg.name] = bleController::CONFIG_FILE_ERROR;
		}
	}
	else
	{
		jsonDoc[msg.name] = bleController::CMD_INVALID;
	}
}

void bleNotifySetAppmode(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	if (msg.mode >= DEMO_RECORDING_MODE && msg.mode <= DEMO_TEST_ALGORITHM_MODE)
	{
		if(currentAppMode == DEMO_IDLE_MODE)
		{
			appMode = (demoAppMode)msg.mode;
			jsonDoc[msg.name] = bleController::CMD_VALID;
		}
		else
		{
			jsonDoc[msg.name] = bleController::APP_ALREADY_IN_STREAMING_MODE;
		}
	}
	else
	{
		jsonDoc[msg.name] = bleController::CMD_INVALID;
	}
}

void bleNotifyGetAppmode(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	jsonDoc["appMode"] = (int)currentAppMode;
}

void bleNotifySetGroundtruth(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	groundTruth = msg.groundTruth;
	jsonDoc[msg.name] = bleController::CMD_VALID;
}

void bleNotifyBsecOutput(const bsecOutputs& outputs, const uint8_t sensNum)
{
	StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
	JsonArray bsecOutputArray = jsonDoc.createNestedArray("bsec");
		
	for (uint8_t i = 0; i < outputs.nOutputs; i++) 
	{
		const bsec_output_t& output = outputs.output[i];
		JsonObject bsecOutputObj = bsecOutputArray.createNestedObject();
		
		bsecOutputObj["sensor_num"] = sensNum;
		bsecOutputObj["id"] = output.sensor_id;
		bsecOutputObj["signal"] = output.signal;
		bsecOutputObj["accuracy"] = output.accuracy;
	}
	
	bleCtlr.sendNotification(jsonDoc);
}

void bleNotifyBme68xData(const bme68x_data& data, const uint8_t sensNum)
{
	StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
	JsonObject bme68xObj = jsonDoc.createNestedObject("bme68x");
	
	bme68xObj["sensor_number"] = sensNum;
	bme68xObj["temperature"] = data.temperature;
	bme68xObj["pressure"] = data.pressure;
	bme68xObj["humidity"] = data.humidity;
	bme68xObj["gas_resistance"] = data.gas_resistance;
	bme68xObj["gas_index"] = data.gas_index;

	bleCtlr.sendNotification(jsonDoc);
}

void bleNotifyGetFwVersion(const bleController::bleMsg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	jsonDoc["FirmwareVersion"] = FIRMWARE_VERSION;
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

demoRetCode configureBsecLogging(const String& aiConfigFile, const String& bsecConfigFile, uint8_t bsecConfigStr[BSEC_MAX_PROPERTY_BLOB_SIZE], uint8_t sensorNum)
{
	memset(bsecConfigStr, 0, BSEC_MAX_PROPERTY_BLOB_SIZE);
	demoRetCode ret = bsecDlog.begin(aiConfigFile, bsec2[0].version, sensorNum);
	if (ret >= EDK_OK)
	{
		ret = utils::getBsecConfig(bsecConfigFile, bsecConfigStr);
	}
	return ret;
}

demoRetCode configFileRead(const String& configFile)
{
	demoRetCode ret;
	static bool firstTime = true;
	StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;

	while ((ret = utils::readFile(configFile, FILE_DATA_READ_SIZE, configFileData)) == EDK_OK)
	{
		if(firstTime)
		{
			if (configFile == BME68X_LABEL_INFO_FILE_EXT)
			{
				jsonDoc["getlabelinfo"] = bleController::CMD_VALID;
			}
			else
			{
				jsonDoc["readconfig"] = bleController::CMD_VALID;
			}
			bleCtlr.sendNotification(jsonDoc);
			firstTime = false;
		}
		jsonDoc.clear();
		jsonDoc.add(configFileData);
		bleCtlr.sendNotification(jsonDoc);
	}
	firstTime = true;
	return ret;
}
