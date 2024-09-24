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
 * @date	03 Jan 2024
 * @version	2.1.5
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
/*! FILE_DATA_READ_SIZE determines the size of the data to be read from the file */
#define FILE_DATA_READ_SIZE	    UINT16_C(400)

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
void bleMessageReceived(const bleController::ble_msg& msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles getlabelinfo BLE command reception (get label information)
 *
 * @param[in] msg			: reference to the new BLE message
 * @param[inout] jsonDoc 	: reference to the json formatted BLE response
 */
void ble_notify_get_label_info(const bleController::ble_msg& msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles setlabelinfo BLE command reception (set label information)
 *
 * @param[in] msg 		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_set_label_info(const bleController::ble_msg& msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles setlabel BLE command reception (set class label)
 *
 * @param[in] msg 		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_set_label(const bleController::ble_msg& msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles getrtctime BLE command reception (get RTC timestamp)
 *
 * @param[in] msg 		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_get_rtc_time(const bleController::ble_msg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles setrtctime BLE command reception (set RTC timestamp)
 *
 * @param[in] msg 		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_set_rtc_time(const bleController::ble_msg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles start BLE command reception (start BLE streaming of BSEC and BME raw data)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_start_streaming(const bleController::ble_msg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles stop BLE command reception (stop BLE streaming of BSEC and BME raw data)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_stop_streaming(const bleController::ble_msg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles readconfig BLE command reception (start BLE Streaming of config file data)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_read_config(const bleController::ble_msg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles BSEC output notification
 *
 * @param[in] outputs : reference to the new BSEC output data
 */
void ble_notify_bsec_output(const bsecOutputs& outputs, const uint8_t sens_num);

/*!
 * @brief : This function handles BME68X data notification
 *
 * @param[in] input : reference to the new BME68X data
 * @param[in] sensNum: sensor number
 */
void ble_notify_bme68x_data(const bme68x_data& input, const uint8_t sens_num);

/*!
 * @brief : This function handles setappmode BLE command reception (Setting the current App mode)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_set_appmode(const bleController::ble_msg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles getappmode BLE command reception (Returns the current App mode)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_get_appmode(const bleController::ble_msg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles setgroundtruth BLE command reception(Setting the Groundtruth)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_set_groundtruth(const bleController::ble_msg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles getfwversion BLE command reception (Returns the current firmware version)
 *
 * @param[in] msg		 : reference to the new BLE message
 * @param[inout] jsonDoc : reference to the json formatted BLE response
 */
void ble_notify_get_fw_version(const bleController::ble_msg &msg, JsonDocument& jsonDoc);

/*!
 * @brief : This function handles sensor manager and BME68X datalogger configuration
 *
 * @param[in] bmeExtension : reference to the bmeconfig file extension
 *
 * @return  Application return code
 */
demo_ret_code configure_sensor_logging(const String& bme_extension);

/*!
 * @brief : This function handles the read configuration File
 *
 * @param[in] configFile	: reference to the config File Name
 *
 * @return  Application return code
 */
demo_ret_code config_file_read(const String& config_file);

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
demo_ret_code configure_bsec_logging(const String& ai_config_file, const String& bsec_config_file,
                                 uint8_t bsec_config_str[BSEC_MAX_PROPERTY_BLOB_SIZE], uint8_t sensor_num);

/*!
 * @brief : This function handles collecting, sending sensor raw data via ble and data logging
 *
 * @param[in] sensNum :	sensor number to collect and log the data
 *
 * @return  a Bosch return code
 */
demo_ret_code log_sensor_data(uint8_t sens_num);

/*!
 * @brief : This function receive the system time (in unix epoch time format) from serial port and update into RTC time
 *
 * @return  void
 */
void read_sys_time();

uint8_t 				bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE];
char 					config_file_data[FILE_DATA_READ_SIZE];
Bsec2 					bsec2[NUM_OF_SENS];
bleController  			bleCtlr(bleMessageReceived);
labelProvider 			labelPvr;
ledController			ledCtlr;
sensorManager 			sensorMgr;
bme68xDataLogger		bme68xDlog;
bsecDataLogger 			bsecDlog;
demo_ret_code			ret_code;
uint8_t					selected_sensor;
String 					bme68x_conf_file_name, bsec_conf_file_name, ai_conf_file_name, config_file_name, label_file_name;
demo_app_mode			app_mode, current_app_mode;
gas_label 				label;
bool 					is_bme68x_conf_available, is_bsec_conf_available, is_ai_conf_available, is_label_info_available;
comm_mux				comm[NUM_OF_SENS];
uint8_t					bsec_mem_block[NUM_OF_SENS][BSEC_INSTANCE_SIZE];
uint8_t 				sensor = 0;
uint8_t 				sensor_num;
uint32_t				ground_truth;
static uint8_t 			buff_count = 0;
static bsecDataLogger::sensor_io_data buff[NUM_OF_SENS];

uint32_t received_sys_time = 0;

void setup()
{
	Serial.begin(115200);
	/* Setting default mode as idle */
	app_mode = DEMO_RECORDING_MODE;
	current_app_mode = DEMO_RECORDING_MODE;

	label = BSEC_NO_CLASS;

	/* Setting default sensor as sensor 0 to collect data */
	selected_sensor = NUM_BME68X_UNITS;
	
	/* Initializes the label provider module */
	labelPvr.begin();

	/* Initializes the led controller module */
	ledCtlr.begin();

	/* initialize the ble controller */
	bleCtlr.begin();
  
  	/* Initializes the SD and RTC module */
	ret_code = utils::begin();

	if (ret_code >= EDK_OK)
	{
		is_bme68x_conf_available = utils::get_file_with_extension(bme68x_conf_file_name, BME68X_CONFIG_FILE_EXT);

		/* checks the availability of BME board configuration file */
		if (is_bme68x_conf_available)
		{

			if (configure_sensor_logging(bme68x_conf_file_name) < EDK_OK)
			{
				ret_code = EDK_SENSOR_INITIALIZATION_FAILED;
			}
		}
		else
		{
			ret_code = EDK_SENSOR_CONFIG_MISSING_ERROR;
		}
	}
	
	if (ret_code < EDK_OK)
	{
		current_app_mode = DEMO_IDLE_MODE;
		app_mode = DEMO_IDLE_MODE;
	}
}

void loop()
{
	/* Updates the led controller status */
	ledCtlr.update(ret_code);

  read_sys_time();

	while (bleCtlr.dequeue_ble_msg());

	/*checks the ble connection status, restarts advertising if disconnected */
	bleCtlr.check_ble_connection_sts();

	if (ret_code >= EDK_OK)
	{
		/* Retrieves the current label */
		(void) labelPvr.get_label(label);
		
		switch (current_app_mode)
		{
			/*  Gets the bme688 sensors raw data in app and logs the same based on the selected sensors */
			case DEMO_RECORDING_MODE:
			{
				uint8_t i;
                
				/* Flushes the buffered sensor data to the current log file */
				ret_code = bme68xDlog.flush();
				
				if (ret_code >= EDK_OK)
				{

					if (selected_sensor == NUM_BME68X_UNITS)
					{

						/* Schedules the next readable sensor */
						while (sensorMgr.schedule_sensor(i))
						{
							/* Gets the given sensor raw data and sends via ble and log into the SD card */
							ret_code = log_sensor_data(i);
						}
					}
					else
					{
						ret_code = log_sensor_data(selected_sensor);
					}
				}
			}
			break;
			/* Example of BSEC library integration: gets the data from one out of 8 sensors
			   (this can be selected through application) and calls BSEC library,
			   get the outputs in app and logs the data */
			case DEMO_TEST_ALGORITHM_MODE:
			{
				/* Flushes the buffered sensor data to the current log file */
				ret_code = bsecDlog.flush_sensor_data(selected_sensor);

				if (selected_sensor == NUM_OF_SENS)
				{

					for (sensor_num = 0; sensor_num < NUM_OF_SENS; sensor_num++)
					{
						/* Callback from the user to read data from the BME688 sensors using parallel mode/forced mode,
						process and store outputs */
						(void) bsec2[sensor_num].run();
					}
				}
				else if (selected_sensor < NUM_OF_SENS)
				{
					(void) bsec2[selected_sensor].run();
					sensor_num = selected_sensor;
				}
			}
			break;
			default:
			break;
		}
	}
}

demo_ret_code log_sensor_data(uint8_t sens_num)
{
	demo_ret_code ret;
	bme68x_data* sensor_data[3];

	/* Returns the selected sensor address */
	bme68x_sensor* sensor = sensorMgr.get_sensor(sens_num);

	/* Retrieves the selected sensor data */
	ret = sensorMgr.collect_data(sens_num, sensor_data);

	if (ret < EDK_OK)
	{
		StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
		JsonObject errorObj = jsonDoc.createNestedObject("errorCode");
		
		errorObj["sensor_number"] = sens_num;
		errorObj["error_code"] = bleController::SENSOR_READ_ERROR;

		bleCtlr.send_notification(jsonDoc);
	}
	else
	{

		for (const auto data : sensor_data)
		{

			if (data != nullptr)
			{
				bme68x_data raw_data;
				raw_data.temperature = data->temperature;
				raw_data.pressure = (data->pressure * 0.01f);
				raw_data.humidity = data->humidity;
				raw_data.gas_resistance = data->gas_resistance;
				raw_data.gas_index = data->gas_index;

				/* sends the sensor raw data via ble */
				ble_notify_bme68x_data(raw_data, sens_num);
				/* Writes the sensor data to the current log file */
				ret = bme68xDlog.write_sensor_data(&sens_num, &sensor->id, &sensor->mode, data,
				                                 &sensor->scan_cycle_index, label, ret);

				/* Increments scanCycleIndex by one, if cycle completes */
				if (raw_data.gas_index == sensor->heater_profile.length - 1)
				{
					sensor->scan_cycle_index += 1;

					if (sensor->scan_cycle_index > sensor->heater_profile.nb_repetitions)
					{
						sensor->scan_cycle_index = 1;
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
	ble_notify_bme68x_data(input, sensor_num);
	
	/* Returns the selected sensor address */
	bme68x_sensor *sensor = sensorMgr.get_sensor(sensor_num);

	if (sensor != nullptr)
	{
		bsecDlog.write_sensor_data(&sensor_num, &sensor->id, &input, &sensor->scan_cycle_index,
		                          ground_truth, ret_code, buff[sensor_num]);

		if (input.gas_index == sensor->heater_profile.length - 1)
		{
			sensor->scan_cycle_index++;

			if (sensor->scan_cycle_index > bsecDlog.scanCycles)
			{
				sensor->scan_cycle_index = 1;
			}
		}

		if (outputs.nOutputs)
		{
			/* Sending BSEC output via ble */
			ble_notify_bsec_output(outputs, sensor_num);

			if (sensor != nullptr)
			{
				buff[sensor_num].sensor_num = sensor_num;
				buff[sensor_num].sensor_id = sensor->id;
				buff[sensor_num].outputs = outputs;
				buff[sensor_num].code = ret_code;
				buff[sensor_num].ground_truth = ground_truth;
				/* Writing BSEC output into SD card */
				ret_code = bsecDlog.write_bsec_output(buff[sensor_num]);
			}
		}
	}
}

void bleMessageReceived(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{

	switch (msg.id)
	{
		case bleController::GET_LABEL_INFO:
			ble_notify_get_label_info(msg, jsonDoc);
		break;
		case bleController::SET_LABEL_INFO:
			ble_notify_set_label_info(msg, jsonDoc);
		break;
		case bleController::SET_LABEL:
			ble_notify_set_label(msg, jsonDoc);
		break;
		case bleController::GET_RTC_TIME:
			ble_notify_get_rtc_time(msg, jsonDoc);
		break;
		case bleController::SET_RTC_TIME:
			ble_notify_set_rtc_time(msg, jsonDoc);
		break;
		case bleController::START_STREAMING:
			ble_notify_start_streaming(msg, jsonDoc);
		break;
		case bleController::STOP_STREAMING:
			ble_notify_stop_streaming(msg, jsonDoc);
		break;
		case bleController::READ_CONFIG:
			ble_notify_read_config(msg, jsonDoc);
		break;
		case bleController::SET_APPMODE:
			ble_notify_set_appmode(msg, jsonDoc);
		break;
		case bleController::GET_APPMODE:
			ble_notify_get_appmode(msg, jsonDoc);
		break;
		case bleController::SET_GROUNDTRUTH:
			ble_notify_set_groundtruth(msg, jsonDoc);
		break;
		case bleController::GET_FW_VERSION:
			ble_notify_get_fw_version(msg, jsonDoc);
		break;
		default:
		break;
	}
}

void ble_notify_get_label_info(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	demo_ret_code ret;

	if (current_app_mode != DEMO_IDLE_MODE)
	{
		jsonDoc[msg.name] = bleController::APP_ALREADY_IN_STREAMING_MODE;
		return;
	}
	ret = config_file_read(BME68X_LABEL_INFO_FILE_EXT);

	if (ret == EDK_END_OF_FILE)
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

void ble_notify_set_label_info(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	demo_ret_code ret;
	
	if (current_app_mode == DEMO_RECORDING_MODE)
	{

		if (msg.label_info.label >= LABEL_TAG_MIN_RANGE && msg.label_info.label <= LABEL_TAG_MAX_RANGE)
		{
			ret = bme68xDlog.set_label_info(msg.label_info.label, String(msg.label_info.label_name),
			                              String(msg.label_info.label_desc));

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

void ble_notify_set_label(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{

	if (msg.label >= LABEL_TAG_MIN_RANGE && msg.label <= LABEL_TAG_MAX_RANGE)
	{
		label = static_cast<gas_label>(msg.label);
		jsonDoc[msg.name] = bleController::CMD_VALID;
	}
	else
	{
		jsonDoc[msg.name] = bleController::LABEL_INVALID;
	}
}

void ble_notify_get_rtc_time(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	jsonDoc["value"] = utils::get_rtc().now().unixtime();
}

void ble_notify_set_rtc_time(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	utils::get_rtc().adjust(DateTime(msg.rtc_time));
}

void ble_notify_start_streaming(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	bsec_virtual_sensor_t bsec_output_list[BSEC_NUMBER_OUTPUTS];
	bleController::cmd_status ble_ret_code = bleController::CMD_VALID;
	const bleController::ble_bsec_msg& bsecMsg = msg.bsec;

	/* Initializes the SD and RTC module */
	ret_code = utils::begin();

	if (ret_code >= EDK_OK)
	{

		//Flushing the contents of the buffer to the rawdata file generated during the previous run
		if (current_app_mode == DEMO_TEST_ALGORITHM_MODE)
		{
			bsecDlog.flush_sensor_data(selected_sensor);
		}
		else if (current_app_mode == DEMO_RECORDING_MODE)
		{
			bme68xDlog.flush();
		}
		
		if (app_mode == DEMO_TEST_ALGORITHM_MODE)
		{
			is_bsec_conf_available = utils::get_file_with_extension(bsec_conf_file_name, BSEC_CONFIG_FILE_EXT);
			is_ai_conf_available = utils::get_file_with_extension(ai_conf_file_name, AI_CONFIG_FILE_EXT);

			/* checks the availability of BSEC configuration file */
			if (is_bsec_conf_available)
			{

				if (is_ai_conf_available)
				{
					bsec_virtual_sensor_t bsec_output_list[BSEC_NUMBER_OUTPUTS];
					float sample_rate = -1;
					
					/* Check added to ensure support for multi instance in test algorithm mode */
					if (bsecMsg.selected_sensor > NUM_OF_SENS) 
					{
						ble_ret_code = bleController::BSEC_SELECTED_SENSOR_INVALID;
						jsonDoc[msg.name] = ble_ret_code;
						return;
					}
					
					comm_mux_begin(Wire, SPI);
					
					for (uint8_t i = 0; i < bsecMsg.len; i++)
					{
						bsec_output_list[i] = static_cast<bsec_virtual_sensor_t>(bsecMsg.output_id[i]);
					}
					
					for (uint8_t i = 0; i < NUM_OF_SENS; i++)
					{
						bme68x_sensor *sensor = sensorMgr.get_sensor(i);

						if (sensor == nullptr)
						{
							ble_ret_code = bleController::BSEC_SELECTED_SENSOR_INVALID;
							jsonDoc[msg.name] = ble_ret_code;
							return;
						}
						
						/* Sets the Communication interface for the sensors */
						comm[i] = comm_mux_set_config(Wire, SPI, i, comm[i]);
						sensor->scan_cycle_index = 1;
						
						/* Assigning a chunk of memory block to the bsecInstance */
						bsec2[i].allocateMemory(bsec_mem_block[i]);
						
						/* Whenever new data is available call the newDataCallback function */
						bsec2[i].attachCallback(bsecCallBack);
						
						switch (bsecMsg.sample_rate)
						{
							case bleController::ULP:
								sample_rate = BSEC_SAMPLE_RATE_ULP;
							break;
							case bleController::LP:
								sample_rate = BSEC_SAMPLE_RATE_LP;
							break;
							case bleController::HP:
								sample_rate = BSEC_SAMPLE_RATE_SCAN;
							break;
							default:
							break;
						}
						
						if (!bsec2[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &comm[i]))
						{
							ble_ret_code = bleController::BSEC_INIT_ERROR;
						}

						if (i == 0)
						{

							if (configure_bsec_logging(ai_conf_file_name, bsec_conf_file_name, bsec_config, bsecMsg.selected_sensor) < EDK_OK)
							{
								ble_ret_code = bleController::BSEC_CONFIG_FILE_ERROR;
								jsonDoc[msg.name] = ble_ret_code;
								return;
							}
						}

						/* Checking the aiconfig type and subscription is same (classification/regression) */
						if (strcmp(bsecDlog.ai_config_type, "classification") == 0)
						{
							
							for (int i = 0; i < bsecMsg.len; i++)
							{
								
								switch (bsec_output_list[i])
								{
									case BSEC_OUTPUT_REGRESSION_ESTIMATE_1:
									case BSEC_OUTPUT_REGRESSION_ESTIMATE_2:
									case BSEC_OUTPUT_REGRESSION_ESTIMATE_3:
									case BSEC_OUTPUT_REGRESSION_ESTIMATE_4:
										ble_ret_code = bleController::AI_CONFIG_AND_SUBSCRIPTION_MISSMATCH;
										jsonDoc[msg.name] = ble_ret_code;
										return;
									default: break;
								}
							}
						}
						else if (strcmp(bsecDlog.ai_config_type, "regression") == 0)
						{
							
							for (int i = 0; i < bsecMsg.len; i++)
							{
								
								switch (bsec_output_list[i])
								{
									case BSEC_OUTPUT_GAS_ESTIMATE_1:
									case BSEC_OUTPUT_GAS_ESTIMATE_2:
									case BSEC_OUTPUT_GAS_ESTIMATE_3:
									case BSEC_OUTPUT_GAS_ESTIMATE_4:
										ble_ret_code = bleController::AI_CONFIG_AND_SUBSCRIPTION_MISSMATCH;
										jsonDoc[msg.name] = ble_ret_code;
										return;
									default: break;
								}
							}
						}

						/* Sets the sensor ID when .bmeconfig file is not initilized */
						if (sensor->id == 0)
						{
							sensor->id = bsec2[i].sensor.getUniqueId();
						}

						if (!bsec2[i].setConfig(bsec_config))
						{
							ble_ret_code = bleController::BSEC_SET_CONFIG_ERROR;
						}
						else if (!bsec2[i].updateSubscription(bsec_output_list, bsecMsg.len, sample_rate))
						{
							ble_ret_code = bleController::BSEC_UPDATE_SUBSCRIPTION_ERROR;
						}
						else if (!bsec2[i].run())
						{
							ble_ret_code = bleController::BSEC_RUN_ERROR;
						}
						else
						{
							const bme68x_heatr_conf& heater_conf = bsec2[i].sensor.getHeaterConfiguration();

							if (heater_conf.profile_len == 0)
							{
								jsonDoc["temperature"] = heater_conf.heatr_temp;
								jsonDoc["duration"] = ((int32_t)heater_conf.heatr_dur) * GAS_WAIT_SHARED;
								jsonDoc["mode"] = "forced";
								
								sensor->heater_profile.length = heater_conf.profile_len;
								sensor->heater_profile.duration[0] = heater_conf.heatr_dur;
								sensor->heater_profile.temperature[0] = heater_conf.heatr_temp;
								sensor->mode = BME68X_FORCED_MODE;
							}
							else if ((heater_conf.heatr_dur_prof != nullptr) && (heater_conf.heatr_temp_prof != nullptr))
							{
								JsonArray heaterDurationArray = jsonDoc.createNestedArray("duration");
								JsonArray heaterTemperatureArray = jsonDoc.createNestedArray("temperature");

								for (uint8_t i = 0; i < heater_conf.profile_len; i++)
								{
									heaterDurationArray.add(((int32_t)heater_conf.heatr_dur_prof[i]) * GAS_WAIT_SHARED);
									heaterTemperatureArray.add(heater_conf.heatr_temp_prof[i]);
									sensor->heater_profile.duration[i] = heater_conf.heatr_dur_prof[i];
									sensor->heater_profile.temperature[i] = heater_conf.heatr_temp_prof[i];
								}
								jsonDoc["mode"] = "parallel";

								sensor->heater_profile.length = heater_conf.profile_len;
								sensor->mode = BME68X_PARALLEL_MODE;
							}
							
							selected_sensor = bsecMsg.selected_sensor;
							current_app_mode = DEMO_TEST_ALGORITHM_MODE;
						}
					}
				}
				else
				{
					ble_ret_code = bleController::AI_CONFIG_FILE_MISSING;
				}
			}
			else
			{
				ble_ret_code = bleController::BSEC_CONFIG_FILE_MISSING;
			}
		}
		else
		{
			is_bme68x_conf_available = utils::get_file_with_extension(bme68x_conf_file_name, BME68X_CONFIG_FILE_EXT);

			/* checks the availability of BME board configuration file */
			if (is_bme68x_conf_available)
			{

				if (configure_sensor_logging(bme68x_conf_file_name) < EDK_OK)
				{
					ble_ret_code = bleController::SENSOR_INITIALIZATION_FAILED;
					ret_code = EDK_SENSOR_INITIALIZATION_FAILED;
				}
				else
				{
					selected_sensor = bsecMsg.selected_sensor;
					current_app_mode = DEMO_RECORDING_MODE;
				}
			}
			else
			{
				ble_ret_code = bleController::SENSOR_CONFIG_MISSING;
				ret_code = EDK_SENSOR_CONFIG_MISSING_ERROR;
			}
		}
		jsonDoc[msg.name] = ble_ret_code;
	}
	else
	{
		jsonDoc[msg.name] = bleController::SD_CARD_INIT_ERROR;
		ret_code = EDK_SD_CARD_INIT_ERROR;
		current_app_mode = DEMO_IDLE_MODE;
	}
}

void ble_notify_stop_streaming(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	
	//Flushing the contents of the buffer to the rawdata file generated during the previous run
	if (current_app_mode == DEMO_TEST_ALGORITHM_MODE)
	{
		bsecDlog.flush_sensor_data(selected_sensor);
	}
	else
	{
		bme68xDlog.flush();
	}
	selected_sensor = 0;
	
	current_app_mode = DEMO_IDLE_MODE;
	app_mode = DEMO_IDLE_MODE;
}

void ble_notify_read_config(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	demo_ret_code ret;

	if (current_app_mode != DEMO_IDLE_MODE)
	{
		jsonDoc[msg.name] = bleController::APP_ALREADY_IN_STREAMING_MODE;
	}
	else if (msg.file_type >= bleController::BMECONFIG && msg.file_type <= bleController::AICONFIG)
	{

		if (msg.file_type == bleController::BMECONFIG)
		{
			config_file_name = BME68X_CONFIG_FILE_EXT;
		}
		else
		{
			config_file_name = AI_CONFIG_FILE_EXT;
		}
		ret = config_file_read(config_file_name);

		if (ret == EDK_END_OF_FILE)
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

			if (config_file_name == BME68X_CONFIG_FILE_EXT)
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

void ble_notify_set_appmode(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{

	if (msg.mode >= DEMO_RECORDING_MODE && msg.mode <= DEMO_TEST_ALGORITHM_MODE)
	{

		if (current_app_mode == DEMO_IDLE_MODE)
		{
			app_mode = (demo_app_mode)msg.mode;
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

void ble_notify_get_appmode(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	jsonDoc["appMode"] = (int32_t)current_app_mode;
}

void ble_notify_set_groundtruth(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	ground_truth = msg.ground_truth;
	jsonDoc[msg.name] = bleController::CMD_VALID;
}

void ble_notify_bsec_output(const bsecOutputs& outputs, const uint8_t sens_num)
{
	StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
	JsonArray bsecOutputArray = jsonDoc.createNestedArray("bsec");
		
	for (uint8_t i = 0; i < outputs.nOutputs; i++) 
	{
		const bsec_output_t& output = outputs.output[i];
		JsonObject bsecOutputObj = bsecOutputArray.createNestedObject();
		
		bsecOutputObj["sensor_num"] = sens_num;
		bsecOutputObj["id"] = output.sensor_id;
		bsecOutputObj["signal"] = output.signal;
		bsecOutputObj["accuracy"] = output.accuracy;
	}
	
	bleCtlr.send_notification(jsonDoc);
}

void ble_notify_bme68x_data(const bme68x_data& data, const uint8_t sens_num)
{
	StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;
	JsonObject bme68xObj = jsonDoc.createNestedObject("bme68x");
	
	bme68xObj["sensor_number"] = sens_num;
	bme68xObj["temperature"] = data.temperature;
	bme68xObj["pressure"] = data.pressure;
	bme68xObj["humidity"] = data.humidity;
	bme68xObj["gas_resistance"] = data.gas_resistance;
	bme68xObj["gas_index"] = data.gas_index;

	bleCtlr.send_notification(jsonDoc);
}

void ble_notify_get_fw_version(const bleController::ble_msg &msg, JsonDocument& jsonDoc)
{
	jsonDoc[msg.name] = bleController::CMD_VALID;
	jsonDoc["FirmwareVersion"] = FIRMWARE_VERSION;
}

demo_ret_code configure_sensor_logging(const String& bme_config_file)
{
	demo_ret_code ret = sensorMgr.begin(bme_config_file);

	if (ret >= EDK_OK)
	{
		ret = bme68xDlog.begin(bme_config_file);
	}
	return ret;
}

demo_ret_code configure_bsec_logging(const String& ai_config_file, const String& bsec_config_file,
                                 uint8_t bsec_config_str[BSEC_MAX_PROPERTY_BLOB_SIZE], uint8_t sensor_num)
{
	memset(bsec_config_str, 0, BSEC_MAX_PROPERTY_BLOB_SIZE);
	demo_ret_code ret = bsecDlog.begin(ai_config_file, bsec2[0].version, sensor_num);

	if (ret >= EDK_OK)
	{
		ret = utils::get_bsec_config(bsec_config_file, bsec_config_str);
	}
	return ret;
}

demo_ret_code config_file_read(const String& config_file)
{
	demo_ret_code ret;
	static bool first_time = true;
	StaticJsonDocument<BLE_JSON_DOC_SIZE> jsonDoc;

	while ((ret = utils::read_file(config_file, FILE_DATA_READ_SIZE, config_file_data)) == EDK_OK)
	{

		if (first_time)
		{

			if (config_file == BME68X_LABEL_INFO_FILE_EXT)
			{
				jsonDoc["getlabelinfo"] = bleController::CMD_VALID;
			}
			else
			{
				jsonDoc["readconfig"] = bleController::CMD_VALID;
			}
			bleCtlr.send_notification(jsonDoc);
			first_time = false;
		}
		jsonDoc.clear();
		jsonDoc.add(config_file_data);
		bleCtlr.send_notification(jsonDoc);
	}
	first_time = true;
	return ret;
}

void read_sys_time()
{
  while (Serial.available()) {
    String rx_msg;

    rx_msg = Serial.readStringUntil('\n');
    received_sys_time = rx_msg.toInt();

    utils::get_rtc().adjust(DateTime(received_sys_time));
  }
}
