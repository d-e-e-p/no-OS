/***************************************************************************//**
 *   @file   interactive.c
 *   @brief  Interactive mode for EIT application.
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
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "no_os_delay.h"
#include "no_os_uart.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "bia_measurement.h"
#include "mux_board.h"
#include "app.h"
#include "parameters.h"
#include "interactive.h"

extern uint32_t AppBuff[];
extern struct electrode_combo swComboSeq[];
extern unsigned int runningCmd;

void ParseResultMode(struct measurement_config *pMeasCfg)
{
	char *cmd_ptr;
	char cTmp;
	uint8_t cmd_ok = 0;
	char hex_string_byte_param[10];
	cmd_ptr = strtok(NULL, ",");
	pMeasCfg->bImpedanceReadMode = false; //default
	if (cmd_ptr) { // If last parameter exists read it
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%c", &cTmp);
		if (cmd_ok) {
			if (cTmp == 'Z')
				pMeasCfg->bImpedanceReadMode = true;
		}
	}

	cmd_ptr = strtok(NULL, ",");
	pMeasCfg->bMagnitudeMode = false;
	if (cmd_ptr) { // If parameter exists read it
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%c", &cTmp);
		if (cmd_ok) {
			if (cTmp == 'M')
				pMeasCfg->bMagnitudeMode = true;
		}
	}
}

int32_t ParseConfig(char  *pStr,
		    struct eit_config *pEitCfg,
		    struct measurement_config *pMeasCfg)
{
	char *cmd_ptr;
	uint8_t cmd_ok = 0;
	char hex_string_byte_param[10];
	*pStr = 0;
	cmd_ptr = strtok(pStr + 1, ",");
	strcpy(hex_string_byte_param, cmd_ptr);
	cmd_ok = sscanf(hex_string_byte_param, "%hu", &pMeasCfg->nFrequency);
	if (cmd_ok) {
		cmd_ptr = strtok(NULL, ",");
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%hu",
				&pEitCfg->nElectrodeCnt);
	}
	if (cmd_ok) {
		cmd_ptr = strtok(NULL, ",");
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%hu", &pEitCfg->nForceDist);
	}
	if (cmd_ok) {
		cmd_ptr = strtok(NULL, ",");
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%hu", &pEitCfg->nSenseDist);
	}
	if (cmd_ok) {
		cmd_ptr = strtok(NULL, ",");
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%hu", &pEitCfg->nRefEl);
	}
	if (cmd_ok)
		ParseResultMode(pMeasCfg);

	if (!cmd_ok) //Error Parsing
		return 1;
	else
		return 0;
}

int32_t ParseQuery(char *pStr,
		   struct electrode_combo *pElCmb,
		   struct measurement_config *pMeasCfg)
{
	char *cmd_ptr;
	uint8_t cmd_ok = 0;
	char hex_string_byte_param[10];
	*pStr = 0;
	cmd_ptr = strtok(pStr + 1, ",");
	strcpy(hex_string_byte_param, cmd_ptr);
	cmd_ok = sscanf(hex_string_byte_param, "%hu", &pMeasCfg->nFrequency);
	if (cmd_ok) {
		cmd_ptr = strtok(NULL, ",");
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%hu", &pElCmb->F_plus);
	}
	if (cmd_ok) {
		cmd_ptr = strtok(NULL, ",");
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%hu", &pElCmb->F_minus);
	}
	if (cmd_ok) {
		cmd_ptr = strtok(NULL, ",");
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%hu", &pElCmb->S_plus);
	}
	if (cmd_ok) {
		cmd_ptr = strtok(NULL, ",");
		strcpy(hex_string_byte_param, cmd_ptr);
		cmd_ok = sscanf(hex_string_byte_param, "%hu", &pElCmb->S_minus);
	}
	if (cmd_ok)
		ParseResultMode(pMeasCfg);

	if (!cmd_ok) //Error Parsing
		return 1;
	else
		return 0;
}

void MuxSupportedElectrodeCounts()
{
	//uint8_t outBuff[20]={0};
	printf("%s", "08,10,20");
}

void interactive_mode(struct no_os_i2c_desc *i2c,
		      struct ad5940_dev *ad5940,
		      struct no_os_uart_desc *uart,
		      struct eit_config oldEitCfg,
		      struct measurement_config oldMeasCfg,
		      struct electrode_combo oldElCfg,
		      uint16_t switchSeqCnt)
{
	int ret;
	static uint32_t IntCount;
	struct eit_config newEitCfg;
	struct electrode_combo newElCfg;
	struct measurement_config newMeasCfg;

	char *buffStr = 0;
	uint32_t temp = APPBUFF_SIZE;
	uint16_t switchSeqNum = 0;

	int32_t cmd_err = 0;
	uint8_t lastConfig = 'C';

	uint8_t cmd[32];
	uint8_t cmdi = 0;

	while (1) {
		ret = no_os_uart_read(uart, &cmd[cmdi], 1);
		if (ret == 1)
			cmdi++;
		else if (!ret)
			continue;

		if (cmd[cmdi - 1] == '\n') {
			if (cmd[0] == 'O') { // STOP
				ADG2128_SwRst(ad5940);
				AppBiaCtrl(ad5940, BIACTRL_STOPNOW, 0);
				printf("%s", "\n!O ");
				runningCmd = 0;
				fflush(stdin);
				printf("%s", "\n");
			}
			if (cmd[0] == 'R') { // RESET
				ADG2128_SwRst(ad5940);
				printf("%s", "!R \n");
				runningCmd = 0;
				fflush(stdin);
			}

			if (!runningCmd) {
				if (cmd[0] == 'B') { //
					ADG2128_SwRst(ad5940);
					printf("%s", "!B ");
					runningCmd = 0;
					fflush(stdin);
					MuxSupportedElectrodeCounts();
					putchar('\n');
				}

				if (cmd[0] == 'Q') { //Run Specific combination
					AppBiaCtrl(ad5940, BIACTRL_STOPNOW, 0);
					cmd[0] = 0;
					newElCfg = oldElCfg;
					newMeasCfg = oldMeasCfg;
					buffStr = (char *)(cmd + 1);
					cmd_err = ParseQuery(buffStr,
							     &newElCfg,
							     &newMeasCfg);
					fflush(stdin);
					// No errors during parsing.
					if (!cmd_err) {
						runningCmd = 'Q';
						printf("%s", "!CMD Q OK\n");
						configMeasurement(&oldMeasCfg, newMeasCfg);
						AppBiaInit(ad5940, AppBuff, APPBUFF_SIZE);
						no_os_udelay(10);
						printf("%s", "!Q ");
						setMuxSwitch(i2c, ad5940, newElCfg, MUXBOARD_SIZE);
						no_os_udelay(3);
						AppBiaCtrl(ad5940, BIACTRL_START, 0);
						lastConfig = 'Q';
					} else {
						printf("%s", "!CMD Q ERROR\n");
					}
				}

				if (cmd[0] == 'C') { //Configure
					AppBiaCtrl(ad5940, BIACTRL_STOPNOW, 0);
					cmd[0] = 0;
					newEitCfg = oldEitCfg;
					newMeasCfg = oldMeasCfg;
					buffStr = (char *)(cmd + 1);
					cmd_err = ParseConfig(buffStr,
							      &newEitCfg,
							      &newMeasCfg);
					fflush(stdin);
					//all command params are valid, execute command
					if (!cmd_err) {
						runningCmd = 0;
						switchSeqNum = 0;
						printf("%s", "!CMD C OK\n");
						switchSeqCnt = generateSwitchCombination(newEitCfg,
								swComboSeq);
						configMeasurement(&oldMeasCfg, newMeasCfg);
						AppBiaInit(ad5940, AppBuff, APPBUFF_SIZE);
						no_os_udelay(10);
						printf("%s", "!C OK\n");
						lastConfig = 'C';
					} else {
						printf("%s", "!CMD C ERROR\n");
					}
				}

				if (cmd[0] == 'V') { //Start boundary voltage query sequence
					fflush(stdin);
					if (lastConfig == 'C') {
						printf("%s", "!CMD V OK\n");
						runningCmd = 'V';
						switchSeqNum = 0;
						setMuxSwitch(i2c, ad5940, swComboSeq[switchSeqNum++], newEitCfg.nElectrodeCnt);
						AppBiaInit(ad5940, AppBuff, APPBUFF_SIZE);
						no_os_udelay(10);
						AppBiaCtrl(ad5940, BIACTRL_START, 0);
						printf("%s", "!V ");
					} else
						printf("%s", "!Send C Command first to configure!\n");
				}
			}

			memset(cmd, 0, sizeof(cmd));
			cmdi = 0;
		}

		/* Check if interrupt flag which will be set when interrupt occured. */
		if (GetMCUIntFlag()) {
			IntCount++;
			ClrMCUIntFlag(); /* Clear this flag */
			temp = APPBUFF_SIZE;
			AppBiaISR(ad5940, AppBuff,
				  &temp); /* Deal with it and provide a buffer to store data we got */
			AppBiaCtrl(ad5940, BIACTRL_STOPNOW, 0);
			if (runningCmd == 'V' || runningCmd == 'Q') {
				//If Q command is being ran return result
				if (runningCmd == 'Q') {
					SendResult(AppBuff, temp, newMeasCfg.bImpedanceReadMode,
						   newMeasCfg.bMagnitudeMode);
					putchar('\n');
					runningCmd = 0;
				}
				//If V or Z command is being ran and this is the last set of ADC, send a terminator character
				if ((runningCmd == 'V') && switchSeqNum >= switchSeqCnt) {
					SendResult(AppBuff, temp, newMeasCfg.bImpedanceReadMode,
						   newMeasCfg.bMagnitudeMode);
					putchar('\n');;
					runningCmd = 0;
				}

				//if V is still running and switch combinations are not exhausted, restart AFE Seq with new switch combo
				if ((runningCmd == 'V') && switchSeqNum < switchSeqCnt) {
					SendResult(AppBuff, temp, newMeasCfg.bImpedanceReadMode,
						   newMeasCfg.bMagnitudeMode);
					putchar(',');
					setMuxSwitch(i2c, ad5940, swComboSeq[switchSeqNum++], newEitCfg.nElectrodeCnt);
					no_os_udelay(3);
					AppBiaCtrl(ad5940, BIACTRL_START, 0);
				}
			}
		}
	}
}