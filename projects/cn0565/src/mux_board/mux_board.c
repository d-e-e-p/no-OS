/***************************************************************************//**
 *   @file   mux_board.c
 *   @brief  Cross-point switch handling code.
 *   @author Kister Jimenez (kister.jimenez@analog.com)
 *   @author Darius Berghe (darius.berghe@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <stdbool.h>
#include <stdio.h>
#include "no_os_delay.h"
#include "mux_board.h"
#include "app.h"

struct adg2128_pinmap {
	uint8_t chip_addr;
	uint8_t selector;
};

/* Mapping of ADG2128 Xm Pins to Yn Pins
   Where:
   Xm - is one of the multiplexed electrode pins
   Yn - is one of the Bio Impedance Electrodes (F+, F-, S+, S-)

   Y0 -> F+
   Y1 -> S+
   Y2 -> S-
   Y3 -> F-
*/
struct adg2128_pinmap board_map[ADG2128_MUX_SIZE] = {
	
	{0x70, 0x80}, //Electrode 0, I2C addr = 0x70, Mux Config (X0 to Yn)
	{0x70, 0x88}, //Electrode 1, I2C addr = 0x70, Mux Config (X1 to Yn)
	{0x70, 0x90}, //Electrode 2, I2C addr = 0x70, Mux Config (X2 to Yn)
	{0x70, 0x98}, //Electrode 3, I2C addr = 0x70, Mux Config (X3 to Yn)
	{0x70, 0xA0}, //Electrode 4, I2C addr = 0x70, Mux Config (X4 to Yn)
	{0x70, 0xA8}, //Electrode 5, I2C addr = 0x70, Mux Config (X5 to Yn)
	{0x70, 0xC0}, //Electrode 6, I2C addr = 0x70, Mux Config (X6 to Yn)
	{0x70, 0xC8}, //Electrode 7, I2C addr = 0x70, Mux Config (X7 to Yn)
	{0x70, 0xD0}, //Electrode 8, I2C addr = 0x70, Mux Config (X8 to Yn)
	{0x70, 0xD8}, //Electrode 9, I2C addr = 0x70, Mux Config (X9 to Yn)
	{0x70, 0xE0}, //Electrode 10, I2C addr = 0x70, Mux Config (X10 to Yn)
	{0x70, 0xE8}, //Electrode 11, I2C addr = 0x70, Mux Config (X11 to Yn)
                  
	{0x71, 0x80}, //Electrode 12, I2C addr = 0x71, Mux Config (X0 to Yn)
	{0x71, 0x88}, //Electrode 13, I2C addr = 0x71, Mux Config (X1 to Yn)
	{0x71, 0x90}, //Electrode 14, I2C addr = 0x71, Mux Config (X2 to Yn)
	{0x71, 0x98}, //Electrode 15, I2C addr = 0x71, Mux Config (X3 to Yn)
	{0x71, 0xA0}, //Electrode 16, I2C addr = 0x71, Mux Config (X4 to Yn)
	{0x71, 0xA8}, //Electrode 17, I2C addr = 0x71, Mux Config (X5 to Yn)
	{0x71, 0xC0}, //Electrode 18, I2C addr = 0x71, Mux Config (X6 to Yn)
	{0x71, 0xC8}, //Electrode 19, I2C addr = 0x71, Mux Config (X7 to Yn)
	{0x71, 0xD0}, //Electrode 20, I2C addr = 0x71, Mux Config (X8 to Yn)
	{0x71, 0xD8}, //Electrode 21, I2C addr = 0x71, Mux Config (X9 to Yn)
	{0x71, 0xE0}, //Electrode 22, I2C addr = 0x71, Mux Config (X10 to Yn)
	{0x71, 0xE8}, //Electrode 23, I2C addr = 0x71, Mux Config (X11 to Yn)
                  
};

void setMuxSwitch(struct no_os_i2c_desc *i2c, struct ad5940_dev *ad5940,
                  struct electrode_combo sw)
{
    uint8_t i2c_addr;
    uint8_t muxData[2] = {0, 0x01};
    uint16_t *Y = (uint16_t *)&sw; // Points to Y0..Y3 in struct
    uint16_t curr_el;
    uint8_t i;

    size_t total_electrodes = sizeof(board_map) / sizeof(board_map[0]);

    //printf("[DEBUG] setMuxSwitch: total_electrodes=%u\r\n", total_electrodes);

    ADG2128_SwRst(ad5940);

    for (i = 0; i < 4; i++) { // Up to Y0–Y3 connections
        if ((*(Y + i)) < total_electrodes) {
            curr_el = *(Y + i);  // No scaling since logical == physical
            i2c_addr = board_map[curr_el].chip_addr;
            muxData[0] = board_map[curr_el].selector + i;

            // printf("[DEBUG] Y%u -> electrode %u (curr_el=%u) i2c_addr=0x%02X selector=0x%02X\r\n",
            //       i, *(Y + i), curr_el, i2c_addr, muxData[0]);

            i2c->slave_address = i2c_addr;
            if (no_os_i2c_write(i2c, muxData, 2, true) != 0) {
                printf("[ERROR] I2C write failed for addr 0x%02X\r\n", i2c_addr);
            }
        } else {
            printf("[WARN] Y%u electrode index %u out of range (max=%zu)\r\n",
                   i, *(Y + i), total_electrodes - 1);
        }
    }
    no_os_udelay(1);
}


//Up to 8 ADG2128 can be addressed or 4 AD2128 boards
void OldsetMuxSwitch(struct no_os_i2c_desc *i2c, struct ad5940_dev *ad5940,
		  struct electrode_combo sw, uint16_t nElCount)
{
	uint8_t i2c_addr;
	uint8_t muxData[2] = {0, 0x01};
	uint16_t *Y = (uint16_t *)&sw; //
	uint16_t curr_el;
	uint8_t i;
	// This enables using the 16-electrode board as 8-electrode system
	uint16_t el_factor = (uint16_t)ADG2128_MUX_SIZE / nElCount;

	// Just make sure nElCount is a power of 2 factor of ADG2128_MUX_SIZE
	if (el_factor != 0 && ((el_factor & (el_factor - 1)) == 0)) {
		ADG2128_SwRst(ad5940);
		for (i = 0; i < 4; i++) { //Y0 to Y3
			if ((*(Y + i)) < ADG2128_MUX_SIZE) {
				curr_el = *(Y + i) * el_factor;
				i2c_addr = board_map[curr_el].chip_addr;
				muxData[0] = board_map[curr_el].selector + i;
				// muxData[1] = 0x01; //Latch
				i2c->slave_address = i2c_addr;
				no_os_i2c_write(i2c, muxData, 2, true);
			}
		}
		no_os_udelay(1);
	}
}
