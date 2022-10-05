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
 * @file	led_controller.h
 * @date	22 June 2022
 * @version	1.5.5
 * 
 * @brief	Header file for the led_controller
 * 
 * 
 */


#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

/* Include of Arduino Core */
#include <Arduino.h>

#include "demo_app.h"

#define PIN_LED 			13
#define LED_ERROR_PERIOD	100
#define LED_OK_PERIOD		1000

/*!
 * @brief : Class library that holds functionality of the led controller
 */
class ledController
{
private:
	bool _ledOn = false;
	uint32_t _timeStamp = 0;
	
	/*!
	 * @brief : This function updates the led blinking pattern according to
	 *			the provided period.
	 * 
	 * @param[in] period : blinking period
	 */
	void switchLed(uint32_t period);

public:
    /*!
     * @brief : The constructor of the led_controller class
     *        	Creates an instance of the class
     */
    ledController();
	
	/*!
     * @brief : This function initializes the led controller module
     */
	void begin();

	/*!
	 * @brief : This function updates the led controller status. It should be
	 *		   called at least 10 times per second to ensure that the correct
	 *		   blinking pattern is generated.
	 *
	 * @param[in] retCode : error code
	 */
    void update(demoRetCode retCode);
};

#endif