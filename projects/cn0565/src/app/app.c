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
#include "impedance2LCR.h"

extern struct no_os_uart_desc *uart;

uint32_t AppBuff[APPBUFF_SIZE];
struct electrode_combo swComboSeq[32]; // TODO review when nElCount is 32

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

fImpCar_Type GetImpedanceFromPdata(uint32_t *pData)
{
    signExtend18To32(pData, 4); // ??
    return computeImpedance(pData);
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

// switch combination setup that allows independent measurement of each up-down connector pair 
// in Left to right order, instead of default 0x70, 0x71, swapped order
uint16_t generateSwitchCombination(struct eit_config eitCfg, struct electrode_combo *swSeq)
{
    uint8_t plus;
    uint8_t minus;

    uint16_t seqCtr = 0;
    uint8_t electrode;

    // see projects/cn0565/src/mux_board/mux_board.c for sequence, was originally weird
    //uint8_t seq[] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22};
    uint8_t seq[] = {18, 20, 22};
    for (seqCtr = 0; seqCtr < sizeof(seq)/sizeof(seq[0]); seqCtr++) {
        electrode = seq[seqCtr];

        plus = electrode;
        minus = electrode + 1;

        swSeq[seqCtr].F_plus = plus;
        swSeq[seqCtr].F_minus = minus;
        swSeq[seqCtr].S_plus = plus;
        swSeq[seqCtr].S_minus = minus;
        //printf("seqCtr %d: plus=%d minus=%d \r\n",seqCtr, plus, minus);

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

    //pBiaCfg->RcalVal = 1000.0; //Note: RCAL value should be similar to RTIA so the accuracy is best.
    pBiaCfg->RcalVal = 10.0; //Note: RCAL value should be similar to RTIA so the accuracy is best.
    pBiaCfg->HstiaRtiaSel = HSTIARTIA_200; // +- 200mV with 200 ohm dut
    //pBiaCfg->HstiaRtiaSel = HSTIARTIA_1K; 
                              
                               
    pBiaCfg->DftNum = DFTNUM_8192;
    //pBiaCfg->DftNum = DFTNUM_2048;
    pBiaCfg->NumOfData = -1; /* Never stop until you stop it manually by
                              * AppBiaCtrl(dev, ) function */
    pBiaCfg->BiaODR = 20;    /* ODR(Sample Rate) 20Hz */
    pBiaCfg->FifoThresh = 2; /* 4 */
    pBiaCfg->ADCSinc3Osr = ADCSINC3OSR_2;

    pBiaCfg->DacVoltPP = 800.0; // 800.0
    pBiaCfg->SinFreq = 1000.0; /* Hz */
    pBiaCfg->SweepCfg.SweepEn = false;
    pBiaCfg->SweepCfg.SweepStart = 0;
    pBiaCfg->SweepCfg.SweepStop = 1000;
    pBiaCfg->SweepCfg.SweepPoints = 20;
    pBiaCfg->SweepCfg.SweepLog = false;
    pBiaCfg->SweepCfg.SweepIndex = 0;

    pBiaCfg->bImpedanceReadMode = true;

    if (pBiaCfg->bImpedanceReadMode) { // Impedance mode
        pBiaCfg->FifoThresh = 4;
    } else {
        pBiaCfg->FifoThresh = 2;
    }
}

int app_main(struct no_os_i2c_desc *i2c, struct ad5940_init_param *ad5940_ip)
{
    int ret;
    struct eit_config oldEitCfg;
    struct electrode_combo oldElCfg;

    AppBiaCfg_Type *pBiaCfg;
    AppBiaGetCfg(&pBiaCfg);


    uint16_t switchSeqCnt;
    uint16_t switchSeqNum = 0;

    struct ad5940_dev *ad5940;
    ret = ad5940_init(&ad5940, ad5940_ip);
    if (ret)
        return ret;

    AD5940BiaStructInit(); /* Configure run parameters */

    uint32_t fifocnt = 0;
    uint32_t readbuf[APPBUFF_SIZE]; // Ensure APPBUFF_SIZE >= seq_expected_samples
    SWMatrixCfg_Type sw_cfg;

    sw_cfg.Dswitch = SWD_RCAL0; // S+
    sw_cfg.Pswitch = SWP_RCAL0; // F+
    sw_cfg.Nswitch = SWN_RCAL1; // F-
    sw_cfg.Tswitch = SWT_RCAL1 | SWT_TRTIA; // S-
                                          
    ret = ad5940_SWMatrixCfgS(ad5940, &sw_cfg);
    if (ret < 0)
        return ret;


    switchSeqCnt = generateSwitchCombination(oldEitCfg, swComboSeq);
    setMuxSwitch(i2c, ad5940, swComboSeq[-1]);

    // run initial meas even before going into interactive mode
    // step1 : setup sequence
    printf("%s: OK\r\n", __FUNCTION__);

    AppBiaInit(ad5940, AppBuff, APPBUFF_SIZE);
    no_os_udelay(10);

    if (! pBiaCfg->SweepCfg.SweepEn) {
        pBiaCfg->SweepCfg.SweepPoints = 1; 
    }
    uint32_t seq_expected_samples = pBiaCfg->SweepCfg.SweepPoints * pBiaCfg->FifoThresh;
    printf("expected_samples %lu = SweepPoints %lu * FifoThresh %lu \r\n", 
        seq_expected_samples, pBiaCfg->SweepCfg.SweepPoints, pBiaCfg->FifoThresh);

    LCR_Result res[switchSeqCnt]; // VLA in C99
    for (switchSeqNum = 0; switchSeqNum < switchSeqCnt; switchSeqNum++) {
         printf("running seq %d: at freq: %.0f \r\n", switchSeqNum, pBiaCfg->FreqofData);
         setMuxSwitch(i2c, ad5940, swComboSeq[switchSeqNum]);
         no_os_udelay(3);

          ad5940_FIFOGetCnt(ad5940, &fifocnt);
          if (fifocnt) {
             ad5940_FIFORd(ad5940, readbuf, fifocnt);
             printf("Drained %lu samples from FIFO \r\n", fifocnt);
          }

         // Start measurement
         AppBiaCtrl(ad5940, BIACTRL_START, 0);
         //no_os_udelay(100000000);

         fifocnt = 0;
         // Wait until enough samples for this sequence are ready
         while (fifocnt < seq_expected_samples) {
             ad5940_FIFOGetCnt(ad5940, &fifocnt);
             no_os_udelay(10);
             // printf("waiting for fifo count %lu/%lu\r\n", fifocnt, seq_expected_samples);
         }

         // Stop measurement
         AppBiaCtrl(ad5940, BIACTRL_STOPNOW, 0);
         // Read out all samples for this switch sequence
         AppBiaISR(ad5940, readbuf, &fifocnt);

         // Send results frequency point by frequency point
         uint32_t idx;
         uint32_t len = pBiaCfg->FifoThresh;

         uint32_t sweepPoints = pBiaCfg->SweepCfg.SweepPoints;
         ImpedanceDataPoint points[sweepPoints]; // VLA in C99

         for (uint32_t freqPoint = 0; freqPoint < pBiaCfg->SweepCfg.SweepPoints; freqPoint++) {
             idx = freqPoint * len;
             memcpy(AppBuff, &readbuf[idx], len * sizeof(uint32_t));
    
             if (pBiaCfg->SweepCfg.SweepEn) {
                pBiaCfg->RtiaCurrValue[0] = pBiaCfg->RtiaCalTable[freqPoint][0];
                pBiaCfg->RtiaCurrValue[1] = pBiaCfg->RtiaCalTable[freqPoint][1];
             }

             printf("idx = %ld/%ld freqSeq=%ld/%ld\r\n", 
                 idx, seq_expected_samples, freqPoint, pBiaCfg->SweepCfg.SweepPoints);
             fImpCar_Type Zdut = GetImpedanceFromPdata(AppBuff);
             points[freqPoint].frequency = pBiaCfg->FreqofData;
             points[freqPoint].impedance = Zdut;
         }

         LCR_Result result = lcr_from_impedance(points, sweepPoints);
         res[switchSeqNum] = result;

     }

    printf("complete init seq \r\n");
    AppBiaCtrl(ad5940, BIACTRL_STOPNOW, 0);

    for (switchSeqNum = 0; switchSeqNum < switchSeqCnt; switchSeqNum++) {
         LCR_Result result = res[switchSeqNum];
         printf("\r\n--- LCR Fitting Results ---\r\n");
         if (!isnan(result.L)) {
             printf("seq: %d L = %.1gmH C = %.1gµF R=%eΩ err=%.1g \r\n", 
                     switchSeqNum, result.L * 1e3, result.C * 1e6, result.R, result.fit_error);
         } else {
             printf("seq: %d LCR Fitting failed.\r\n", switchSeqNum);
         }
    }

    /*
    printf("╔══════════════════════════════════════════════════════════════╗\r\n");
    printf("║                    LCR Fitting Results                       ║\r\n");
    printf("╠═════╤══════════════╤══════════════╤═══════════════╤══════════╣\r\n");
    printf("║ Seq │      L       │      C       │       R       │  Error   ║\r\n");
    printf("║     │    (mH)      │     (µF)     │      (Ω)      │          ║\r\n");
    printf("╞═════╪══════════════╪══════════════╪═══════════════╪══════════╡\r\n");

    for (switchSeqNum = 0; switchSeqNum < switchSeqCnt; switchSeqNum++) {
        LCR_Result result = res[switchSeqNum];
        
        if (!isnan(result.L)) {
            printf("║ %3d │ %10.1g │ %10.1g │ %11.2e │ %8.1g ║\r\n",
                    switchSeqNum, result.L * 1e3, result.C * 1e6, result.R, result.fit_error);
        } else {
            printf("║ %3d │    FITTING FAILED           │              │          ║\r\n", switchSeqNum);
        }
        
        if (switchSeqNum < switchSeqCnt - 1) {
            printf("╟─────┼──────────────┼──────────────┼───────────────┼──────────╢\r\n");
        }
    }

    printf("╚═════╧══════════════╧══════════════╧═══════════════╧══════════╝\r\n");
    */


    // OK, now go into interactive mode
    struct measurement_config oldMeasCfg;
    interactive_mode(i2c, ad5940, uart, oldEitCfg, oldMeasCfg, oldElCfg, switchSeqCnt);

    return 0;
}
