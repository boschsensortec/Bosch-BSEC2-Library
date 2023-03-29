/**
 * Copyright (C) Bosch Sensortec GmbH. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet. Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchaser's own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 * @file     bsec_interface_multi.h  
 *
 * @brief
 * Contains the multi-instance API for BSEC
 *
 */

#ifndef __BSEC_INTERFACE_MULTI_H__
#define __BSEC_INTERFACE_MULTI_H__

#include "bsec_datatypes.h"

#ifdef __cplusplus
 extern "C" {
#endif

 /*! @addtogroup bsec_lib_interface BSEC Multi-instance Interfaces
 *  @brief The multi-instance interface of BSEC signal processing library is used for interfacing multiple sensors with BSEC library.
 *
 * # Multi-instance interface usage
 * 
 * The following provides a short overview on the typical operation sequence for BSEC.
 * 
 * - Initialization of the library
 * 
 * | Steps                                                               | Function                 |
 * |---------------------------------------------------------------------|--------------------------|
 * | Initialization of library                                           | bsec_init_m()              |
 * | Update configuration settings (optional)                            | bsec_set_configuration_m() |
 * | Restore the state of the library (optional)                         | bsec_set_state_m()         |
 *
 * 
 * - The following function is called to enable output signals and define their sampling rate / operation mode.
 * 
 * | Steps                                       |  Function                  |
 * |---------------------------------------------|----------------------------|
 * | Enable library outputs with specified mode  | bsec_update_subscription_m() |
 *
 * 
 * - This table describes the main processing loop.
 * 
 * | Steps                                     | Function                         |
 * |-------------------------------------------|----------------------------------|
 * | Retrieve sensor settings to be used       | bsec_sensor_control_m()            |
 * | Configure sensor and trigger measurement  | See BME688 API and example codes |
 * | Read results from sensor                  | See BME688 API and example codes |
 * | Perform signal processing                 | bsec_do_steps_m()                  |
 *
 * 
 * - Before shutting down the system, the current state of BSEC can be retrieved and can then be used during 
 *   re-initialization to continue processing.
 *   
 * | Steps                                       | Function          |
 * |---------------------------------------------|-------------------|
 * | Retrieve the current library state          |  bsec_get_state_m() |
 * | Retrieve the current library configuration  |  bsec_get_configuration_m() |
 * 
 * 
 * ### Configuration and state                       
 * 
 * Values of variables belonging to a BSEC instance are divided into two groups:
 *  - Values **not updated by processing** of signals belong to the **configuration group**. If available, BSEC can be 
 *    configured before use with a customer specific configuration string.
 *  - Values **updated during processing** are member of the **state group**. Saving and restoring of the state of BSEC 
 *    is necessary to maintain previously estimated sensor models and baseline information which is important for best 
 *    performance of the gas sensor outputs.
 * 
 * @note BSEC library consists of adaptive algorithms which models the gas sensor which improves its performance over 
 *       the time. These will be lost if library is initialized due to system reset. In order to avoid this situation 
 *       library state shall be stored in non volatile memory so that it can be loaded after system reset.
 *
 * 
 *   @{
 */

/********************************************************/
/* function prototype declarations */

/*!
 * @brief Function that provides the size of the internal instance in bytes. 
 * To be used for allocating memory for struct BSEC_STRUCT_NAME
 * @return Size of the internal instance in bytes
 */
size_t bsec_get_instance_size_m(void);

/*!
 * @brief Return the version information of BSEC library instance
 * @param[in,out]   inst    Reference to the pointer containing the instance
 * @param [out]     bsec_version_p      pointer to struct which is to be populated with the version information
 *
 * @return Zero if successful, otherwise an error code
 *
 * See also: bsec_version_t
 *
*/
bsec_library_return_t bsec_get_version_m(void *inst, bsec_version_t *bsec_version_p);

/*!
 * @brief Initialize the library instance
 *
 * Initialization and reset of BSEC library instance is performed by calling bsec_init_m() as done with bsec_init() for standard interface.
 *
 * @param[in,out]   inst    Reference to the pointer containing the instance
 *
 * @return Zero if successful, otherwise an error code
 *
*/
bsec_library_return_t bsec_init_m(void *inst);

/*!
 * @brief Subscribe to library virtual sensors outputs
 *
 * Like bsec_update_subscription(), bsec_update_subscription_m() is used to instruct BSEC which of the processed output signals 
 * of the library instance are requested at which sample rates.
 *
 * @param[in,out]   inst                            Reference to the pointer containing the instance
 * @param[in]       requested_virtual_sensors       Pointer to array of requested virtual sensor (output) configurations for the library
 * @param[in]       n_requested_virtual_sensors     Number of virtual sensor structs pointed by requested_virtual_sensors
 * @param[out]      required_sensor_settings        Pointer to array of required physical sensor configurations for the library
 * @param[in,out]   n_required_sensor_settings      [in] Size of allocated required_sensor_settings array, [out] number of sensor configurations returned
 *
 * @return Zero when successful, otherwise an error code
 *
 * @sa bsec_sensor_configuration_t
 * @sa bsec_physical_sensor_t
 * @sa bsec_virtual_sensor_t
 *
 */
bsec_library_return_t bsec_update_subscription_m(void *inst, const bsec_sensor_configuration_t *const requested_virtual_sensors,
													 const uint8_t n_requested_virtual_sensors, bsec_sensor_configuration_t *required_sensor_settings,
													 uint8_t *n_required_sensor_settings);

/*!
 * @brief Main signal processing function of BSEC library instance
 *
 *
 * Processing of the input signals and returning of output samples for each instances of BSEC library is performed by bsec_do_steps_m(). 
 * bsec_do_steps_m() processes multiple instaces of BSEC library simillar to how bsec_do_steps() handles single instance.
 *
 * @param[in,out]   inst            Reference to the pointer containing the instance
 * @param[in]       inputs          Array of input data samples. Each array element represents a sample of a different physical sensor.
 * @param[in]       n_inputs        Number of passed input data structs.
 * @param[out]      outputs         Array of output data samples. Each array element represents a sample of a different virtual sensor.
 * @param[in,out]   n_outputs       [in] Number of allocated output structs, [out] number of outputs returned
 *
 * @return Zero when successful, otherwise an error code
 *
 */
bsec_library_return_t bsec_do_steps_m(void *inst, const bsec_input_t *const inputs, const uint8_t n_inputs, bsec_output_t *outputs, uint8_t *n_outputs);

/*!
 * @brief Reset a particular virtual sensor output of the library instance
 *
 * This function allows specific virtual sensor outputs of each library instance to be reset.
 * It processes in same way as bsec_reset_output().
 *
 * @param[in,out]   inst                Reference to the pointer containing the instance
 * @param[in]       sensor_id           Virtual sensor to be reset
 *
 * @return Zero when successful, otherwise an error code
 *
 */
bsec_library_return_t bsec_reset_output_m(void *inst, uint8_t sensor_id);

/*!
 * @brief Update algorithm configuration parameters of the library instance
 *
 * As done with bsec_set_configuration(), the initial configuration of BSEC libray instance can be customized 
 * by bsec_set_configuration_m(). This is an optional step.
 * 
 * Please use #BSEC_MAX_PROPERTY_BLOB_SIZE for allotting the required size.
 *
 * @param[in,out]   inst                    Reference to the pointer containing the instance
 * @param[in]       serialized_settings     Settings serialized to a binary blob
 * @param[in]       n_serialized_settings   Size of the settings blob
 * @param[in,out]   work_buffer             Work buffer used to parse the blob
 * @param[in]       n_work_buffer_size      Length of the work buffer available for parsing the blob
 *
 * @return Zero when successful, otherwise an error code
 *
 */
bsec_library_return_t bsec_set_configuration_m(void *inst, const uint8_t *const serialized_settings,
												   const uint32_t n_serialized_settings, uint8_t *work_buffer,
												   const uint32_t n_work_buffer_size);
/*!
 * @brief Restore the internal state of the library instance
 *
 * BSEC uses a default state for all signal processing modules and the BSEC module for each instance. To ensure optimal performance, 
 * especially of the gas sensor functionality, it is recommended to retrieve the state using bsec_get_state_m()
 * before unloading the library, storing it in some form of non-volatile memory, and setting it using bsec_set_state_m() 
 * before resuming further operation of the library.
 *
 * Please use #BSEC_MAX_STATE_BLOB_SIZE for allotting the required size.
 *
 * @param[in,out]   inst                    Reference to the pointer containing the instance
 * @param[in]       serialized_state        States serialized to a binary blob
 * @param[in]       n_serialized_state      Size of the state blob
 * @param[in,out]   work_buffer             Work buffer used to parse the blob
 * @param[in]       n_work_buffer_size      Length of the work buffer available for parsing the blob
 *
 * @return Zero when successful, otherwise an error code
 *
*/
bsec_library_return_t bsec_set_state_m(void *inst, const uint8_t *const serialized_state, const uint32_t n_serialized_state,
										   uint8_t *work_buffer, const uint32_t n_work_buffer_size);

/*!
 * @brief Retrieve the current library instance configuration
 *
 * BSEC allows to retrieve the current configuration of the library instance using bsec_get_configuration_m(). 
 * In the same way as bsec_get_configuration(), this API returns a binary blob encoding 
 * the current configuration parameters of the library in a format compatible with bsec_set_configuration_m().
 *
 * Please use #BSEC_MAX_PROPERTY_BLOB_SIZE for allotting the required size.
 *
 * @param[in,out]   inst                        Reference to the pointer containing the instance 
 * @param[in]       config_id                   Identifier for a specific set of configuration settings to be returned;
 *                                              shall be zero to retrieve all configuration settings.
 * @param[out]      serialized_settings         Buffer to hold the serialized config blob
 * @param[in]       n_serialized_settings_max   Maximum available size for the serialized settings
 * @param[in,out]   work_buffer                 Work buffer used to parse the binary blob
 * @param[in]       n_work_buffer               Length of the work buffer available for parsing the blob
 * @param[out]      n_serialized_settings       Actual size of the returned serialized configuration blob
 *
 * @return Zero when successful, otherwise an error code
 *
 */
bsec_library_return_t bsec_get_configuration_m(void *inst, const uint8_t config_id, uint8_t *serialized_settings, const uint32_t n_serialized_settings_max,
												   uint8_t *work_buffer, const uint32_t n_work_buffer, uint32_t *n_serialized_settings);

/*!
 *@brief Retrieve the current internal library instance state
 *
 * BSEC allows to retrieve the current states of all signal processing modules and the BSEC module of the library instance using 
 * bsec_get_state_m(). As done by bsec_get_state(), this allows a restart of the processing after a reboot of the system by calling bsec_set_state_m().
 * 
 * Please use #BSEC_MAX_STATE_BLOB_SIZE for allotting the required size.
 *
 * @param[in,out]   inst                        Reference to the pointer containing the instance 
 * @param[in]       state_set_id                Identifier for a specific set of states to be returned; shall be
 *                                              zero to retrieve all states.
 * @param[out]      serialized_state            Buffer to hold the serialized config blob
 * @param[in]       n_serialized_state_max      Maximum available size for the serialized states
 * @param[in,out]   work_buffer                 Work buffer used to parse the blob
 * @param[in]       n_work_buffer               Length of the work buffer available for parsing the blob
 * @param[out]      n_serialized_state          Actual size of the returned serialized blob
 *
 * @return Zero when successful, otherwise an error code
 *
 */
bsec_library_return_t bsec_get_state_m(void *inst, const uint8_t state_set_id, uint8_t *serialized_state,
										   const uint32_t n_serialized_state_max, uint8_t *work_buffer, const uint32_t n_work_buffer,
										   uint32_t *n_serialized_state);

/*!
 * @brief Retrieve BMExxx sensor instructions for the library instance
 *
 * The bsec_sensor_control_m() allows an easy way for the signal processing library to control the operation of the 
 * BME sensor which uses the correspodning BSEC library instance. Operation of bsec_sensor_control_m() is simillar to bsec_sensor_control()
 * except that former API supports multiples library instances.
 *
 * @param[in,out]   inst                      Reference to the pointer containing the instance
 * @param [in]      time_stamp                Current timestamp in [ns]
 * @param[out]      sensor_settings           Settings to be passed to API to operate sensor at this time instance
 *
 * @return Zero when successful, otherwise an error code
 */
bsec_library_return_t bsec_sensor_control_m(void *inst, const int64_t time_stamp, bsec_bme_settings_t *sensor_settings);

/*@}*/ //BSEC Interface

#ifdef __cplusplus
}
#endif

#endif /* __BSEC_INTERFACE_MULTI_H__ */
