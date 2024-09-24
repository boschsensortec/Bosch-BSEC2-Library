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
 * @file	label_provider.h
 * @date	03 Jan 2024
 * @version	2.1.5
 * 
 * @brief	Header file for the label provider
 * 
 * 
 */

#ifndef LABEL_PROVIDER_H
#define LABEL_PROVIDER_H

/* Include of Arduino Core */
#include "Arduino.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/* Pins connected to interrupt buttons */
#define PIN_BUTTON_1			UINT8_C(32)
#define PIN_BUTTON_2 			UINT8_C(14)
#define LABEL_NAME_SIZE			UINT8_C(50)
#define LABEL_DESC_SIZE      	UINT8_C(200)
#define LABEL_TAG_MAX_RANGE   	UINT32_C(2147483647)
#define LABEL_TAG_MIN_RANGE   	UINT16_C(1001)

enum gas_label
{
	BSEC_NO_CLASS,
	BSEC_CLASS_1,
	BSEC_CLASS_2,
	BSEC_CLASS_3,
	BSEC_CLASS_4
};

/*!
 * @brief : Class library that holds functionality of the label provider
 */
class labelProvider
{
private:
	/* variable for temporarily holding the new set user label */
	static volatile gas_label _label;
	/* variables to store information about the current buttons states */
	static volatile bool _but1_pressed, _but2_pressed;
	
	static QueueHandle_t _queue;

    /*!
	 * @brief : This function is the interrupt handler of the first button
	 */
	static void isr_button1();

    /*!
	 * @brief :  This function is the interrupt handler of the second button
	 */
	static void isr_button2();

public:

    /*!
     * @brief : The constructor of the label_provider class
     *          Creates an instance of the class
     */
	labelProvider();
	
	/*!
     * @brief : This function initializes the label provider module
     */
	void begin();

	/*!
	 * @brief : This function retrieves the current label. 
	 * 
     * @param[out] label : reference to the label
     * 
     * @return  true if a new label is available else false
	 */
	bool get_label(gas_label &label);
};

#endif
