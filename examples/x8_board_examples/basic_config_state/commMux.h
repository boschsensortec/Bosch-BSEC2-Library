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

   @file    commMux.h
   @date	17 January 2023
   @version	2.0.6

*/
#ifndef COMM_MUX_H
#define COMM_MUX_H

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

/**
 * Datatype working as an interface descriptor
 */
typedef struct {
   TwoWire *wireobj;
   SPIClass *spiobj;
   uint8_t select;
} commMux;

/**
 * @brief Function to configure the communication across sensors
 * @param wireobj : The TwoWire object
 * @param spiobj  : The SPIClass object
 * @param idx     : Selected sensor for communication interface
 * @param comm    : Structure for selected sensor
 * @return        : Structure holding the communication setup
 */
commMux commMuxSetConfig(TwoWire &wireobj, SPIClass &spiobj, uint8_t idx, commMux &comm);

/**
 * @brief Function to trigger the communication
 * @param wireobj : The TwoWire object
 * @param spiobj  : The SPIClass object
 */
void commMuxBegin(TwoWire &wireobj, SPIClass &spiobj);

/**
 * @brief Function to write the sensor data to the register
 * @param reg_addr : Address of the register
 * @param reg_data : Pointer to the data to be written
 * @param length   : length of the register data
 * @param intf_ptr : Pointer to the interface descriptor
 * @return 0 if successful, non-zero otherwise
 */
int8_t commMuxWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
 * @brief Function to read the sensor data from the register
 * @param reg_addr : Address of the register
 * @param reg_data : Pointer to the data to be read from the sensor
 * @param length   : length of the register data
 * @param intf_ptr : Pointer to the interface descriptor
 * @return 0 if successful, non-zero otherwise
 */
int8_t commMuxRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
 * @brief Function to maintain a delay between communication
 * @param period_us   : Time delay in micro secs
 * @param intf_ptr    : Pointer to the interface descriptor
 */
void commMuxDelay(uint32_t period_us, void *intf_ptr);

#endif /* COMM_MUX_H */
