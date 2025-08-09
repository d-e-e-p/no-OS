/***************************************************************************//**
 *   @file   app.c
 *   @brief  EIT application source.
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

extern struct no_os_uart_desc *uart;

uint32_t AppBuff[APPBUFF_SIZE];
struct electrode_combo swComboSeq[256]; // TODO review when nElCount is 32

float SinFreqVal = 0.0;
unsigned int SinFreqValUINT = 0;
unsigned int runningCmd = 0;

volatile uint32_t ucInterrupted = 0; /* Flag to indicate interrupt occurred */
extern volatile unsigned char szInSring[32];

/* Since the reset pin is mapped to GPIO of AD5940, Access it through
 * AD5940 GPIO Register. */
int ADG2128_SwRst(struct ad5940_dev *dev)
{
	int ret = ad5940_WriteReg(dev, REG_AGPIO_GP0OUT, 0);
	if (ret < 0)
		return ret;
	no_os_udelay(1);
	return ad5940_WriteReg(dev, REG_AGPIO_GP0OUT, AGPIO_Pin1);
}

uint32_t GetMCUIntFlag(void)
{
	return ucInterrupted;
}

uint32_t ClrMCUIntFlag(void)
{
	ucInterrupted = 0;
	return 1;
}

void configMeasurement(struct measurement_config *oldCfg,
		       struct measurement_config newCfg)
{
	AppBiaCfg_Type *pBiaCfg;
	AppBiaGetCfg(&pBiaCfg);
	if (oldCfg->nFrequency != newCfg.nFrequency) {
		pBiaCfg->bParamsChanged = true;
		oldCfg->nFrequency = newCfg.nFrequency;
		pBiaCfg->SinFreq = ((float)(newCfg.nFrequency)) * 1000.0;
	}

	if (oldCfg->nAmplitudePP != newCfg.nAmplitudePP) {
		pBiaCfg->bParamsChanged = true;
		oldCfg->nAmplitudePP = newCfg.nAmplitudePP;
		pBiaCfg->DacVoltPP = newCfg.nAmplitudePP * 1.0;
	}
	pBiaCfg->bImpedanceReadMode = newCfg.bImpedanceReadMode;
	pBiaCfg->SweepCfg.SweepEn = newCfg.bSweepEn;
	if (newCfg.bImpedanceReadMode) { // Impedance mode
		pBiaCfg->FifoThresh = 4;
	} else {
		pBiaCfg->FifoThresh = 2;
	}
	// Set newCfg as old for next command execution;
	*oldCfg = newCfg;
}

void SendResultUint32(uint32_t *pData, uint32_t nDataCount)
{
	uint32_t i;

	for (i = 0; i < nDataCount - 1; i++) {
		printf("%lx,", pData[i]);
	}

	printf("%lx", pData[i]);
}

void SendResultIeee754(float *data, uint32_t DataCount)
{

	uint32_t *pVal = (uint32_t *)((void *)data);
	uint32_t i;

	for (i = 0; i < DataCount - 1; i++) {
		printf("%lx,", pVal[i]);
	}

	printf("%lx", pVal[i]);
}

void SendResult(uint32_t *pData, uint16_t len,
		bool bImpedanceReadMode, bool bMagnitudeMode)
{
	float fMagVal = 0;
	fImpCar_Type fCarZval;
	iImpCar_Type iCarVval;
	signExtend18To32(pData, len);
	if (bImpedanceReadMode && (len == 4)) { // Send Impedance
		fCarZval = computeImpedance(pData);
		if (bMagnitudeMode) { // Complex to Magnitude
			fMagVal = sqrt(fCarZval.Real * fCarZval.Real +
					       fCarZval.Image * fCarZval.Image);
			SendResultIeee754(&fMagVal, 1); // Impedance Magnitude only. Float
		} else { // Complex Impedance in IEE754 uint32 hex string.
			SendResultIeee754((float *)&fCarZval, 2);
		}
	} else if ((!bImpedanceReadMode) && (len == 2)) { // Send Voltage
		if (bMagnitudeMode) { // Complex to Magnitude
			iCarVval = *((iImpCar_Type *)pData);
			fMagVal = sqrt((iCarVval.Real * 1.0) * (iCarVval.Real * 1.0) +
					       (iCarVval.Image * 1.0) * (iCarVval.Image * 1.0));
			SendResultIeee754(&fMagVal, 1); // Voltage Magnitude only. Float
		} else { // Complex Voltage in uint32 hex string.
			SendResultUint32(pData, 2);
		}
	}
}

// switch comb for independent meas of each up-down connecter pair
uint16_t generateSwitchCombination(struct eit_config eitCfg,
				   struct electrode_combo *swSeq)
{
	uint16_t i = 0;
	//uint16_t j = 0;
	uint8_t F_plus;
	uint8_t F_minus;
	uint8_t S_plus;
	uint8_t S_minus;
	uint16_t seqCtr = 0;

    for (i = 0; i < eitCfg.nElectrodeCnt; i += 2) {

        F_plus = i;
        F_minus = (i + eitCfg.nForceDist) % eitCfg.nElectrodeCnt;
        S_plus = F_plus;
        S_minus = F_minus;

		swSeq[seqCtr].F_plus = F_plus;
		swSeq[seqCtr].F_minus = F_minus;
		swSeq[seqCtr].S_plus = S_plus;
		swSeq[seqCtr++].S_minus = S_minus;
        printf("seqCtr %d: S_plus=%d F_plus=%d S_minus=%d F_minus=%d \r\n",seqCtr, S_plus, F_plus, S_minus, F_minus);

      }
	return seqCtr;
}


/* !!Change the application parameters here if you want to change it to
 * none-default value */
void AD5940BiaStructInit(void)
{
	AppBiaCfg_Type *pBiaCfg;

	AppBiaGetCfg(&pBiaCfg);

	pBiaCfg->SeqStartAddr = 0;
	pBiaCfg->MaxSeqLen = 512; /** @todo add checker in function */

	pBiaCfg->RcalVal = 1000.0;
	pBiaCfg->DftNum = DFTNUM_8192;
	//pBiaCfg->DftNum = DFTNUM_2048;
	pBiaCfg->NumOfData = -1; /* Never stop until you stop it manually by
							  * AppBiaCtrl(dev, ) function */
	pBiaCfg->BiaODR = 20;	 /* ODR(Sample Rate) 20Hz */
	pBiaCfg->FifoThresh = 2; /* 4 */
	pBiaCfg->ADCSinc3Osr = ADCSINC3OSR_2;

	pBiaCfg->DacVoltPP = 300.0; //800.0
	pBiaCfg->SinFreq = 1000.0; /* 1000Hz */
	pBiaCfg->SweepCfg.SweepEn = true;
	pBiaCfg->SweepCfg.SweepStart = 1000;
	pBiaCfg->SweepCfg.SweepStop = 10000;
	pBiaCfg->SweepCfg.SweepPoints = 10;
	pBiaCfg->SweepCfg.SweepLog = false;
	pBiaCfg->SweepCfg.SweepIndex = 0;
}

int app_main(struct no_os_i2c_desc *i2c, struct ad5940_init_param *ad5940_ip)
{
	int ret;
	struct eit_config oldEitCfg;
	struct electrode_combo oldElCfg;
	struct measurement_config oldMeasCfg;

	struct measurement_config newMeasCfg;

	uint32_t temp = APPBUFF_SIZE;
	uint16_t switchSeqCnt = 0;
	uint16_t switchSeqNum = 0;

	struct ad5940_dev *ad5940;
	ret = ad5940_init(&ad5940, ad5940_ip);
	if (ret)
		return ret;

	AD5940BiaStructInit(); /* Configure your parameters in this function */

	oldMeasCfg.bImpedanceReadMode = true;
	oldMeasCfg.bMagnitudeMode = false;
	oldMeasCfg.nFrequency = 1;	// default 1 Khz Excitation
	oldMeasCfg.nAmplitudePP = 300; // default 300mV peak to peak excitation
	oldMeasCfg.bSweepEn = true;

	oldElCfg.F_plus = 0;
	oldElCfg.F_minus = 3;
	oldElCfg.S_plus = 1;
	oldElCfg.S_minus = 2;

	oldEitCfg.nElectrodeCnt = 16;
	oldEitCfg.nForceDist = 1;
	oldEitCfg.nSenseDist = 1;
	oldEitCfg.nRefEl = 0;

	switchSeqNum = 0;
	switchSeqCnt = generateSwitchCombination(oldEitCfg, swComboSeq);

    // run initial meas even before going into interactive mode
    // step1 : setup sequence
	runningCmd = 'Q';
	printf("%s", "!CMD Q OK\r\n");
    newMeasCfg = oldMeasCfg;
    configMeasurement(&oldMeasCfg, newMeasCfg);

	AppBiaInit(ad5940, AppBuff, APPBUFF_SIZE);
	printf("start init seq \r\n");
	no_os_udelay(10);

    for (switchSeqNum = 0; switchSeqNum < switchSeqCnt; switchSeqNum++) {
		printf("running seq %d: \r\n", switchSeqNum);
		setMuxSwitch(i2c, ad5940, swComboSeq[switchSeqNum], oldEitCfg.nElectrodeCnt);
	    no_os_udelay(3);
	    AppBiaCtrl(ad5940, BIACTRL_START, 0);
	    no_os_udelay(10);
        SendResult(AppBuff, temp, newMeasCfg.bImpedanceReadMode, newMeasCfg.bMagnitudeMode);
    }
	printf("complete init seq \r\n");


    // OK, now go into interactive mode
	interactive_mode(i2c, ad5940, uart, oldEitCfg, oldMeasCfg, oldElCfg, switchSeqCnt);

	return 0;
}
