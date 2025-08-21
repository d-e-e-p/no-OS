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
#include "measure.h"
#include "impedance2LCR.h"

extern struct no_os_uart_desc *uart;

uint32_t AppBuff[APPBUFF_SIZE];

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

void print_int_array(const char *name, uint8_t *arr, size_t n) {
    printf("%s = { ", name);
    for (size_t i = 0; i < n; i++) {
        printf("%d", arr[i]);
        if (i != n - 1) printf(", ");
    }
    printf(" }\r\n");
}



/**
 * Generate switch combinations from a given sequence array.
 * see projects/cn0565/src/mux_board/mux_board.c for sequence, was originally weird
 * switch combination setup that allows independent measurement of each up-down connector pair 
 * in Left to right order, instead of default 0x70, 0x71, swapped order
 *
 * @param swSeq   Output array of electrode_combo structs
 * @param seq     Input sequence array (e.g. {1,10,11,9})
 * @param num_seq  Length of seq[]
 * @return        Number of generated combos (== num_seq)
 * eg:
 *  uint8_t seq[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
 */
void generateSwitchCombination(struct electrode_combo *swSeq,
                                 size_t num_seq,
                                 uint8_t seq[num_seq])
{
    print_int_array("seq_list", seq, num_seq);
    for (size_t seqCtr = 0; seqCtr < num_seq; seqCtr++) {

        // each electrode has 2 pins, so jump by 2
        uint8_t electrode = seq[seqCtr] * 2;

        uint8_t plus  = electrode;
        uint8_t minus = electrode + 1;

        swSeq[seqCtr].F_plus  = plus;
        swSeq[seqCtr].F_minus = minus;
        swSeq[seqCtr].S_plus  = plus;
        swSeq[seqCtr].S_minus = minus;

        //printf("seqCtr %zu: plus=%u minus=%u\r\n", seqCtr, plus, minus);
    }

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
    //pBiaCfg->HstiaRtiaSel = HSTIARTIA_200; // +- 200mV with 200 ohm dut
    pBiaCfg->HstiaRtiaSel = HSTIARTIA_1K; 
                              
                               
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

void print_float_array(const char *name, float *arr, size_t n) {
    printf("%s = { ", name);
    for (size_t i = 0; i < n; i++) {
        printf("%.1f", arr[i]);
        if (i != n - 1) printf(", ");
    }
    printf(" }\r\n");
}


int app_main(struct no_os_i2c_desc *i2c, struct ad5940_init_param *ad5940_ip)
{
    int ret;
    //struct eit_config oldEitCfg;
    //struct electrode_combo oldElCfg;
    struct electrode_combo swComboSeq[MUXBOARD_SIZE]; // TODO review when nElCount is 32
    uint16_t switchSeqNum = 0;

    AppBiaCfg_Type *pBiaCfg;
    AppBiaGetCfg(&pBiaCfg);


    struct ad5940_dev *ad5940;
    ret = ad5940_init(&ad5940, ad5940_ip);
    if (ret)
        return ret;

    printf("%s: OK\r\n", __FUNCTION__);
    AD5940BiaStructInit(); /* Configure run parameters */

    //uint32_t fifocnt = 0;
    //uint32_t readbuf[APPBUFF_SIZE]; // Ensure APPBUFF_SIZE >= seq_expected_samples
    
    /*
    SWMatrixCfg_Type sw_cfg;
    sw_cfg.Dswitch = SWD_RCAL0; // S+
    sw_cfg.Pswitch = SWP_RCAL0; // F+
    sw_cfg.Nswitch = SWN_RCAL1; // F-
    sw_cfg.Tswitch = SWT_RCAL1 | SWT_TRTIA; // S-
    ret = ad5940_SWMatrixCfgS(ad5940, &sw_cfg);
    if (ret < 0)
        return ret;
    */

    uint8_t seq[] = {10, 11, 9};
    size_t num_seq = sizeof(seq)/sizeof(seq[0]);
    generateSwitchCombination(swComboSeq, num_seq, seq);
    setMuxSwitch(i2c, ad5940, swComboSeq[0]);
    print_int_array("seq_list", seq, num_seq);


    // run initial meas before going into interactive mode
    // step1 : setup sequence
    float freq_list[] = {200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000,};
    //float freq_list[] = {200, };
    size_t num_freq = sizeof(freq_list) / sizeof(freq_list[0]);
    print_float_array("freq_list", freq_list, num_freq);

    //float desired_vpp[] = {10, 20, 50, 100, 200, 500, 1000, 2000 };
    //float desired_vpp[] = {10, 15, 20, 25, 30, 35, 40};
    float desired_vpp[] = {10, 15, 20};
    size_t num_volt = sizeof(desired_vpp)/sizeof(desired_vpp[0]);
    print_float_array("desired_vpp", desired_vpp, num_volt);

    ImpedanceDataPoint resZ[num_seq][num_volt][num_freq];   // hold Z results 
    LCR_Result resLCR[num_seq][num_volt];   // hold LCR results 
                                                 //
    fImpCar_Type Ztia[num_volt][num_freq];
    printf("%s : init \r\n", __FUNCTION__);
                                                        
    for (size_t i = 0; i < num_volt; i++) {
        pBiaCfg->DesiredVoltage = desired_vpp[i];
        printf("%s: desired_vpp voltage[%d] = %f\r\n", __FUNCTION__, i, desired_vpp[i]);
        for (size_t j = 0; j < num_freq; j++) {
            pBiaCfg->SinFreq = freq_list[j];
            pBiaCfg->bParamsChanged = true;
            AppBiaInit(ad5940, AppBuff, APPBUFF_SIZE);
            Ztia[i][j] = pBiaCfg->ZtiaCalCurrValue;
            for (switchSeqNum = 0; switchSeqNum < num_seq; switchSeqNum++) {
                printf("%s:═══════════════════════ experiment %d probe  %.0f mV %.0f Hz \r\n",
                        __FUNCTION__, switchSeqNum, desired_vpp[i], freq_list[j] );
                setMuxSwitch(i2c, ad5940, swComboSeq[switchSeqNum]);
                AppBiaRdutRun(ad5940, &resZ[switchSeqNum][i][j]);
            }
        }
        printf("complete init seq \r\n");
        AppBiaCtrl(ad5940, BIACTRL_STOPNOW, 0);

        printf("\r\n--- LCR Fitting Results ---\r\n");

        for (switchSeqNum = 0; switchSeqNum < num_seq; switchSeqNum++) {
            LCR_Result result = lcr_from_impedance(resZ[switchSeqNum][i], num_freq);
            resLCR[switchSeqNum][i] = result;
            if (!isnan(result.L)) {
                printf("seq: %d volt=%.0f mV  L = %.3g mH  C = %.3g fF  R=%.3g   Ω err=%.2g\r\n", 
                        switchSeqNum, desired_vpp[i], 
                        result.L * 1e3, result.C * 1e15, result.R, result.fit_error * 1e3);
            } else {
                printf("seq: %d LCR Fitting failed.\r\n", switchSeqNum);
            }
        }
    }


    fImpCar_Type ZtiaAve[num_volt];
    dump_ztia_csv(num_seq, num_volt, num_freq, desired_vpp, freq_list, Ztia, ZtiaAve);
    dump_zdut_csv(num_seq, num_volt, num_freq, desired_vpp, freq_list, resZ);
    dump_raw_lcr_csv(num_seq, num_volt, ZtiaAve, desired_vpp, resLCR);
    dump_fit_lcr_csv(num_seq, num_volt, ZtiaAve, desired_vpp, resLCR);
        
    dump_lcr_box(num_seq, num_volt, ZtiaAve, desired_vpp, resLCR);


    // OK, now go into interactive mode
    //struct measurement_config oldMeasCfg;
    //interactive_mode(i2c, ad5940, uart, oldEitCfg, oldMeasCfg, oldElCfg, num_seq);

    return 0;
}
