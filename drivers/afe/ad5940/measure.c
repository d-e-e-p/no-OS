#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include "no_os_delay.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_alloc.h"
#include "ad5940.h"
#include "calibrate.h"


#include <stdio.h>
#include <stdint.h>

float decode_fifo_word(uint32_t w)
{
    /* Extract sequence ID */
    uint8_t seqid = (w >> 23) & 0x03;

    /* Extract channel ID bits [22:16] */
    uint8_t chid  = (w >> 16) & 0x7F;   // 7 bits
    uint8_t chid5 = chid & 0x1F;        // lower 5 bits (real channel ID)
    uint8_t subid = chid & 0x03;        // lower 2 bits (extra info when chid=11111)

    /* Is it DFT? */
    bool is_dft = (chid5 == 0x1F);

    /* Extract 18-bit signed payload */
    uint32_t raw18 = w & 0x3FFFF;
    int32_t val_q2 = (int32_t)(raw18 << 14) >> 14;  // sign extend 18→32

    /* Convert to float, divide by 4 to remove 2 fractional bits */
    float fval = (float)val_q2 / 4.0f;

    /* Diagnostics */
    printf("%s: seq=%u channel=0x%02X type=%X is_dft=%s fval=%.2f\r\n", 
            __FUNCTION__, seqid, chid5, subid, is_dft ? "true" : "false", fval);

    return fval;
}


fImpCar_Type computeImpedanceFromFifo(AppBiaCfg_Type *pBiaCfg, uint32_t *const pData)
{

    // Step0: interpret raw DFT outputs
    fImpCar_Type DftVdut, DftVtia;
    DftVdut.Real  = decode_fifo_word(pData[0]);
    DftVdut.Image = decode_fifo_word(pData[1]);
    DftVtia.Real  = decode_fifo_word(pData[2]);
    DftVtia.Image = decode_fifo_word(pData[3]);

    printf("DftVdut=%f + %f j\r\n", DftVdut.Real, DftVdut.Image);
    printf("DftVtia=%f + %f j\r\n", DftVtia.Real, DftVtia.Image);

    // Fix AD5940 sign conventions
    DftVdut.Image = -DftVdut.Image;
    DftVtia.Image = -DftVtia.Image;

    // Step1: fetch Ztia 
    fImpCar_Type Ztia = {
        .Real  = pBiaCfg->RtiaCurrValue[0],
        .Image = pBiaCfg->RtiaCurrValue[1]
    };

    // Step2: compute current through DUT
    fImpCar_Type Idut = ad5940_ComplexDivFloat(&DftVtia, &Ztia);

    // Step3: compute impedance
    fImpCar_Type Zdut = ad5940_ComplexDivFloat(&DftVdut, &Idut);

    printf(
        "%s: D_Idut = D_Vrtia (%.0f + %.0f j) / Ztia (%.0f + %.0f j)\r\n",
        __FUNCTION__,
        DftVtia.Real, DftVtia.Image,
        Ztia.Real, Ztia.Image
    );

    printf(
        "%s: Zdut (%.0f + %.0f j) = D_Vdut (%.0f + %.0f j) / D_Idut (%.0f + %.0f j)\r\n",
        __FUNCTION__,
        Zdut.Real, Zdut.Image,
        DftVdut.Real, DftVdut.Image,
        Idut.Real, Idut.Image
    );


    return Zdut;
}



int ad5940_MeasureDUT(struct ad5940_dev *dev, HSRTIACal_Type *pCalCfg,
		     void *pResult)
{
	int ret;
	AFERefCfg_Type aferef_cfg;
	HSLoopCfg_Type hs_loop;
	DSPCfg_Type dsp_cfg;
	bool bADCClk32MHzMode = false;
	uint32_t ExcitBuffGain = EXCITBUFGAIN_2;
	uint32_t HsDacGain = HSDACGAIN_1;

	float ExcitVolt; /* Excitation voltage, unit is mV */
	uint32_t RtiaVal;
	static uint32_t const HpRtiaTable[] = {200, 1000, 5000, 10000, 20000, 40000, 80000, 160000, 0};
	uint32_t WgAmpWord;
    float RcalVal = pCalCfg->fRcal;
    //fImpCar_Type ZcalVal = { RcalVal, 0.0f };

	fImpCar_Type DftVdut, DftVtia;
    uint32_t regVal;

	if (pCalCfg == NULL) return -EINVAL;
	if (pCalCfg->fRcal == 0)
		return -EINVAL;
	if (pCalCfg->HsTiaCfg.HstiaRtiaSel > HSTIARTIA_160K)
		return -EINVAL;
	if (pCalCfg->HsTiaCfg.HstiaRtiaSel == HSTIARTIA_OPEN)
		return -EINVAL; /* Do not support calibrating DE0-RTIA */
	if (pResult == NULL)
		return -EINVAL;

	if (pCalCfg->AdcClkFreq > (32000000 * 0.8))
		bADCClk32MHzMode = true;

	/* Calculate the excitation voltage we should use based on RCAL/Rtia */
	RtiaVal = HpRtiaTable[pCalCfg->HsTiaCfg.HstiaRtiaSel];
	ExcitBuffGain = EXCITBUFGAIN_2;
	HsDacGain = HSDACGAIN_1;
	/* Excitation buffer voltage full range is 800mVpp*2=1600mVpp */
	ExcitVolt = 1800 * 0.8;
	WgAmpWord = ((uint32_t)(ExcitVolt / 1600 * 2047 * 2) + 1)
			    >> 1; /* Assign value with rounding (0.5 LSB error) */
	if (WgAmpWord > 0x7ff)
		WgAmpWord = 0x7ff;

    ExcitVolt = 2300 * 0.8 * RcalVal / RtiaVal; 
    ExcitBuffGain = EXCITBUFGAIN_0P25;
    HsDacGain = HSDACGAIN_0P2;
    WgAmpWord = ((uint32_t)(ExcitVolt / 40 * 2047 * 2) + 1)
                >> 1; 

    printf("%s: using RcalVal=%.0f RtiaVal=%lu ExcitVolt=%f WgAmpWord=%lu\r\n", 
            __FUNCTION__, RcalVal, RtiaVal, ExcitVolt, WgAmpWord);

	ret = ad5940_AFECtrlS(dev, AFECTRL_ALL, false);  /* Init all to disable state */
	if (ret < 0)
		return ret;

	/* Configure reference system */
	aferef_cfg.HpBandgapEn = true;
	aferef_cfg.Hp1V1BuffEn = true;
	aferef_cfg.Hp1V8BuffEn = true;
	aferef_cfg.Disc1V1Cap = false;
	aferef_cfg.Disc1V8Cap = false;
	aferef_cfg.Hp1V8ThemBuff = false;
	aferef_cfg.Hp1V8Ilimit = false;
	aferef_cfg.Lp1V1BuffEn = false;
	aferef_cfg.Lp1V8BuffEn = false;
	aferef_cfg.LpBandgapEn = false;
	aferef_cfg.LpRefBufEn = false;
	aferef_cfg.LpRefBoostEn = false;
	ret = ad5940_REFCfgS(dev, &aferef_cfg);
	if (ret < 0)
		return ret;

	/* Configure HP Loop */
	hs_loop.HsDacCfg.ExcitBufGain = ExcitBuffGain;
	hs_loop.HsDacCfg.HsDacGain = HsDacGain;
	hs_loop.HsDacCfg.HsDacUpdateRate = 7; /* Set it to highest update rate */
	memcpy(&hs_loop.HsTiaCfg, &pCalCfg->HsTiaCfg, sizeof(pCalCfg->HsTiaCfg));
    /* 
    // orig rcal measure
	hs_loop.SWMatCfg.Dswitch = SWD_RCAL0;
	hs_loop.SWMatCfg.Pswitch = SWP_RCAL0;
	hs_loop.SWMatCfg.Nswitch = SWN_RCAL1;
	hs_loop.SWMatCfg.Tswitch = SWT_RCAL1 | SWT_TRTIA;
    */

    // measure DUT
    hs_loop.SWMatCfg.Dswitch = SWD_CE0; // S+
    hs_loop.SWMatCfg.Pswitch = SWP_CE0; // F+
    // RLOAD02 and RLOAD04 are fixed 100 Ω for SE0 and AFE3.
    hs_loop.SWMatCfg.Nswitch = SWN_SE0LOAD; // F-
    hs_loop.SWMatCfg.Tswitch = SWT_SE0LOAD | SWT_TRTIA;  // S-
                                                        
	hs_loop.WgCfg.WgType = WGTYPE_SIN;
	hs_loop.WgCfg.GainCalEn =
		false;      /* @todo. If we have factory calibration data, enable it */
	hs_loop.WgCfg.OffsetCalEn = false;
	hs_loop.WgCfg.SinCfg.SinFreqWord = ad5940_WGFreqWordCal(pCalCfg->fFreq,
					   pCalCfg->SysClkFreq);
	hs_loop.WgCfg.SinCfg.SinAmplitudeWord = WgAmpWord;
	hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
	hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
	ret = ad5940_HSLoopCfgS(dev, &hs_loop);
	if (ret < 0)
		return ret;

	/* Configure DSP */
	dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_N_NODE;
	dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_P_NODE;
	dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_1P5;
	AD5940_StructInit(&dsp_cfg.ADCDigCompCfg, sizeof(dsp_cfg.ADCDigCompCfg));
	dsp_cfg.ADCFilterCfg.ADCAvgNum =
		ADCAVGNUM_16;  /* Don't care because it's disabled */
	dsp_cfg.ADCFilterCfg.ADCRate = bADCClk32MHzMode ? ADCRATE_1P6MHZ :
				       ADCRATE_800KHZ;
	dsp_cfg.ADCFilterCfg.ADCSinc2Osr = pCalCfg->ADCSinc2Osr;
	dsp_cfg.ADCFilterCfg.ADCSinc3Osr = pCalCfg->ADCSinc3Osr;
	dsp_cfg.ADCFilterCfg.BpNotch = true;
	dsp_cfg.ADCFilterCfg.BpSinc3 = false;
	dsp_cfg.ADCFilterCfg.DFTClkEnable = true;
	dsp_cfg.ADCFilterCfg.Sinc2NotchClkEnable = true;
	dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = true;
	dsp_cfg.ADCFilterCfg.Sinc3ClkEnable = true;
	dsp_cfg.ADCFilterCfg.WGClkEnable = true;

	memcpy(&dsp_cfg.DftCfg, &pCalCfg->DftCfg, sizeof(pCalCfg->DftCfg));
	memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg));
	ret = ad5940_DSPCfgS(dev, &dsp_cfg);
	if (ret < 0)
		return ret;

	/* Enable all of them. They are automatically turned off during hibernate mode to save power */
	ret = ad5940_AFECtrlS(dev,
			      AFECTRL_HSTIAPWR | AFECTRL_INAMPPWR | AFECTRL_EXTBUFPWR | \
			      /*AFECTRL_WG|*/AFECTRL_DACREFPWR | AFECTRL_HSDACPWR | \
			      AFECTRL_SINC2NOTCH, true);
	if (ret < 0)
		return ret;

	ret = ad5940_AFECtrlS(dev, AFECTRL_WG | AFECTRL_ADCPWR,
			      true);  /* Enable Waveform generator, ADC power */
	if (ret < 0)
		return ret;

	ret = ad5940_AFECtrlS(dev, AFECTRL_ADCCNV | AFECTRL_DFT,
			      true);  /* Start ADC convert and DFT */
	if (ret < 0)
		return ret;

	/* Wait until DFT ready */
	while (ad5940_INTCTestFlag(dev, AFEINTC_1, AFEINTSRC_DFTRDY) == false);

	ret = ad5940_AFECtrlS(dev,
			      AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR,
			      false);  /* Stop ADC convert and DFT */
	if (ret < 0)
		return ret;

	ret = ad5940_INTCClrFlag(dev, AFEINTSRC_DFTRDY);
	if (ret < 0)
		return ret;


    /* --- DftVdut --- */
    //dump_afe_registers(dev, "Zcal");
    ad5940_ReadAfeResult(dev, AFERESULT_DFTREAL, &regVal);
    DftVdut.Real = decode_afe_result(regVal);

    ad5940_ReadAfeResult(dev, AFERESULT_DFTIMAGE, &regVal);
    DftVdut.Image = decode_afe_result(regVal);

	ret = ad5940_ADCMuxCfgS(dev, ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);
	if (ret < 0)
		return ret;

	ret = ad5940_AFECtrlS(dev, AFECTRL_WG | AFECTRL_ADCPWR,
			      true);  /* Enable Waveform generator, ADC power */
	if (ret < 0)
		return ret;

	ret = ad5940_AFECtrlS(dev, AFECTRL_ADCCNV | AFECTRL_DFT,
			      true);  /* Start ADC convert and DFT */
	if (ret < 0)
		return ret;

	/* Wait until DFT ready */
	while (ad5940_INTCTestFlag(dev, AFEINTC_1, AFEINTSRC_DFTRDY) == false);
	ret = ad5940_AFECtrlS(dev,
			      AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR,
			      false);  /* Stop ADC convert and DFT */
	if (ret < 0)
		return ret;

	ret = ad5940_INTCClrFlag(dev, AFEINTSRC_DFTRDY);
	if (ret < 0)
		return ret;

    /* --- DftVtia --- */
    //dump_afe_registers(dev, "Ztia");
    ad5940_ReadAfeResult(dev, AFERESULT_DFTREAL, &regVal);
    DftVtia.Real = decode_afe_result(regVal);

    ad5940_ReadAfeResult(dev, AFERESULT_DFTIMAGE, &regVal);
    DftVtia.Image = decode_afe_result(regVal);

    /* Current flow & DFT convention corrections */
    DftVtia.Real  = -DftVtia.Real;
    DftVtia.Image = -DftVtia.Image;
    DftVdut.Real  = -DftVdut.Real;
    DftVdut.Image =  DftVdut.Image;

    fImpCar_Type Ztia = {
        .Real  = 220,
        .Image = 0,
    };

    printf("%s: dtia = %.2f + %.2f j\r\n", __FUNCTION__, DftVtia.Real, DftVtia.Image);
    printf("%s: ddut = %.2f + %.2f j\r\n", __FUNCTION__, DftVdut.Real, DftVdut.Image);

    // Step2: compute current through DUT
    fImpCar_Type DftIdut = ad5940_ComplexDivFloat(&DftVtia, &Ztia);

    // Step3: compute impedance
    fImpCar_Type Zdut = ad5940_ComplexDivFloat(&DftVdut, &DftIdut);

    printf(
        "%s: D_Idut = D_Vrtia (%.0f + %.0f j) / Ztia (%.0f + %.0f j)\r\n",
        __FUNCTION__,
        DftVtia.Real, DftVtia.Image,
        Ztia.Real, Ztia.Image
    );

    printf(
        "%s: Zdut (%.0f + %.0f j) = D_Vdut (%.0f + %.0f j) / D_Idut (%.0f + %.0f j)\r\n",
        __FUNCTION__,
        Zdut.Real, Zdut.Image,
        DftVdut.Real, DftVdut.Image,
        DftIdut.Real, DftIdut.Image
    );


    return 0;
}



