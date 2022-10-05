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
 * @date	    22 June 2022
 * @version	    1.5.5
 * 
 * @brief       label provider
 *
 * 
 */

/* own header include */
#include "label_provider.h"

volatile gasLabel labelProvider::_label;
volatile bool labelProvider::_but1Pressed, labelProvider::_but2Pressed;

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
    _but1Pressed = false;
	_but2Pressed = false;
	
	_queue = xQueueCreate(2, sizeof(gasLabel));
	
	/* Button interrupts setup and attachment */
    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);
	
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_1), isrButton1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_2), isrButton2, CHANGE);
}

/*!
 * @brief This function is the interrupt function, that handles the button press of the first button
 */
void labelProvider::isrButton1()
{
    /* check if button is pressed or idle */
    if (_but1Pressed == false)
    {
        /* determine if only this button or both are pressed and set helper button label */
        _but1Pressed = true;
        if (_but2Pressed)
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
        _but1Pressed = false;
        if (!_but2Pressed)
        {
			xQueueSendFromISR(_queue, (const void*)&_label, 0);
        }
    }
}

/*!
 * @brief This function is the interrupt function, that handles the button press of the second button
 */
void labelProvider::isrButton2()
{
    /* check if button is pressed or idle */
    if (_but2Pressed == false)
    {
        /* determine if only this button or both are pressed and set helper button label */
        _but2Pressed = true;
        if (_but1Pressed)
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
        _but2Pressed = false;
        if (!_but1Pressed)
        {
			xQueueSendFromISR(_queue, (const void*)&_label, 0);
        }
    }
}

/*!
 * @brief This function retrieves the current label
 */
bool labelProvider::getLabel(gasLabel &label)
{
	return xQueueReceive(_queue, &label, 0);
}

