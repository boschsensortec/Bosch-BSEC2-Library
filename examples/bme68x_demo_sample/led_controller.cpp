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
 * @file	    led_controller.cpp
 * @date	    22 June 2022
 * @version		1.5.5
 * 
 * @brief    	led controller
 *
 * 
 */

/* own header include */
#include "led_controller.h"

/*!
 * @brief The constructor of the led_controller class
 */
ledController::ledController()
{}

/*!
 * @brief This function initializes the led controller module
 */
void ledController::begin()
{
	/* LED pin initialization for runtime monitoring and error indication */
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

	_ledOn = false;
}

/*!
 * @brief This function updates the led blinking pattern according to the
 * 		  provided period
 */
void ledController::switchLed(uint32_t period)
{
	uint32_t timeStamp = millis();
	if ((timeStamp - _timeStamp) >= period)
	{
		if (_ledOn)
		{
			digitalWrite(PIN_LED, LOW);
			_ledOn = false;
		}
		else
		{
			digitalWrite(PIN_LED, HIGH);
			_ledOn = true;
		}
		_timeStamp = timeStamp;
	}
}

/*!
 * @brief This function updates the led controller status
 */
void ledController::update(demoRetCode retcode)
{
	if (retcode >= EDK_OK)
	{
		switchLed(LED_OK_PERIOD);
	}
	else
	{
		switchLed(LED_ERROR_PERIOD);
	}
}

