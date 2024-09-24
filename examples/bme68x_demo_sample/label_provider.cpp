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
 * @file	    label_provider.cpp
 * @date		03 Jan 2024
 * @version		2.1.5
 * 
 * @brief       label provider
 *
 * 
 */

/* own header include */
#include "label_provider.h"

volatile gas_label labelProvider::_label;
volatile bool labelProvider::_but1_pressed, labelProvider::_but2_pressed;

QueueHandle_t labelProvider::_queue = nullptr; 

/*!
 * @brief The constructor of the label_provider class
 */
labelProvider::labelProvider()
{}

/*!
 * @brief This function initializes the label provider module
 */
void labelProvider::begin()
{
	_but1_pressed = false;
	_but2_pressed = false;
	
	_queue = xQueueCreate(2, sizeof(gas_label));
	
	/* Button interrupts setup and attachment */
	pinMode(PIN_BUTTON_1, INPUT_PULLUP);
	pinMode(PIN_BUTTON_2, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_1), isr_button1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_2), isr_button2, CHANGE);
}

/*!
 * @brief This function is the interrupt function, that handles the button press of the first button
 */
void labelProvider::isr_button1()
{
    /* check if button is pressed or idle */
	if (_but1_pressed == false)
	{
		/* determine if only this button or both are pressed and set helper button label */
		_but1_pressed = true;
		
		if (_but2_pressed)
		{
			_label = BSEC_CLASS_3;
		}
		else
		{
			_label = BSEC_CLASS_1;
		}
	}
	else
	{
		/* if both buttons are released, user label according to helper button label */
		_but1_pressed = false;
		
		if (!_but2_pressed)
		{
			xQueueSendFromISR(_queue, (const void*)&_label, 0);
		}
	}
}

/*!
 * @brief This function is the interrupt function, that handles the button press of the second button
 */
void labelProvider::isr_button2()
{
	/* check if button is pressed or idle */
	if (_but2_pressed == false)
	{
		/* determine if only this button or both are pressed and set helper button label */
		_but2_pressed = true;
		
		if (_but1_pressed)
		{
			_label = BSEC_CLASS_3;
		}
		else
		{
			_label = BSEC_CLASS_2;
		}
	}
	else
	{
		/* if both buttons are released, user label according to helper button label */
		_but2_pressed = false;
		
		if (!_but1_pressed)
		{
			xQueueSendFromISR(_queue, (const void*)&_label, 0);
		}
	}
}

/*!
 * @brief This function retrieves the current label
 */
bool labelProvider::get_label(gas_label &label)
{
	return xQueueReceive(_queue, &label, 0);
}
