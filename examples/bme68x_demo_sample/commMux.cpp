/**
 Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.

 BSD-3-Clause

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its
 contributors may be used to endorse or promote products derived from
 this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 @file    commMux.cpp
 @date	17 January 2023
 @version	2.0.6

 */

#include "commMux.h"

#define CLOCK_FREQUENCY 400000
#define COMM_SPEED 8000000

const uint8_t I2C_EXPANDER_ADDR = 0x20;
const uint8_t I2C_EXPANDER_OUTPUT_REG_ADDR = 0x01;
const uint8_t I2C_EXPANDER_OUTPUT_DESELECT = 0xFF;
const uint8_t I2C_EXPANDER_CONFIG_REG_ADDR = 0x03;
const uint8_t I2C_EXPANDER_CONFIG_REG_MASK = 0x00;

/**
 * @brief Function to configure the communication across sensors
 */
commMux commMuxSetConfig(TwoWire &wireobj, SPIClass &spiobj, uint8_t idx, commMux &comm)
{
	comm.select = ((0x01 << idx) ^ 0xFF);
	comm.spiobj = &spiobj;
	comm.wireobj = &wireobj;

	return comm;
}

/**
 * @brief Function to trigger the communication
 */
void commMuxBegin(TwoWire &wireobj, SPIClass &spiobj)
{
	wireobj.begin();
	wireobj.setClock(CLOCK_FREQUENCY);
	wireobj.beginTransmission(I2C_EXPANDER_ADDR);
	wireobj.write(I2C_EXPANDER_CONFIG_REG_ADDR);
	wireobj.write(I2C_EXPANDER_CONFIG_REG_MASK);
	wireobj.endTransmission();

	spiobj.begin();
}

/** 
 * @brief Function to set the ship select pin of the SPI
 */
static void setChipSelect(TwoWire *wireobj, uint8_t mask)
{
	// send I2C-Expander device address
	wireobj->beginTransmission(I2C_EXPANDER_ADDR);
	// send I2C-Expander output register address
	wireobj->write(I2C_EXPANDER_OUTPUT_REG_ADDR);
	// send mask to set output level of GPIO pins
	wireobj->write(mask);
	// end communication
	wireobj->endTransmission();
}

/**
 * @brief Function to write the sensor data to the register
 */
int8_t commMuxWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	commMux *comm = (commMux*) intf_ptr;
	uint32_t i;

	if (comm)
	{
		setChipSelect(comm->wireobj, comm->select);

		comm->spiobj->beginTransaction(SPISettings(COMM_SPEED, MSBFIRST, SPI_MODE0));
		comm->spiobj->transfer(reg_addr);
		for (i = 0; i < length; i++)
		{
			comm->spiobj->transfer(reg_data[i]);
		}
		comm->spiobj->endTransaction();

		setChipSelect(comm->wireobj, I2C_EXPANDER_OUTPUT_DESELECT);

		return 0;
	}

	return 1;
}

/**
 * @brief Function to read the sensor data from the register
 */
int8_t commMuxRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	commMux *comm = (commMux*) intf_ptr;
	uint32_t i;

	if (comm)
	{
		setChipSelect(comm->wireobj, comm->select);

		comm->spiobj->beginTransaction(SPISettings(COMM_SPEED, MSBFIRST, SPI_MODE0));
		comm->spiobj->transfer(reg_addr);
		for (i = 0; i < length; i++)
		{
			reg_data[i] = comm->spiobj->transfer(0xFF);
		}
		comm->spiobj->endTransaction();

		setChipSelect(comm->wireobj, I2C_EXPANDER_OUTPUT_DESELECT);

		return 0;
	}

	return 1;
}

/**
 * @brief Function to maintain a delay between communication
 */
void commMuxDelay(uint32_t period_us, void *intf_ptr)
{
	(void) intf_ptr;
	delayMicroseconds(period_us);
}
