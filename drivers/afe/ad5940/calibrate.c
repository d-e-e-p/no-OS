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
#include "measure.h"


#include <stdio.h>
#include <stdint.h>

void dump_afe_registers(void *dev, const char *tag)
{
    uint32_t rd;

    ad5940_ReadReg(dev, REG_AFE_ADCDAT, &rd);
    printf("%s: REG_AFE_ADCDAT     = 0x%08ld\r\n", tag, (unsigned long)rd);

    ad5940_ReadReg(dev, REG_AFE_SINC2DAT, &rd);
    printf("%s: REG_AFE_SINC2DAT   = 0x%08ld\r\n", tag, (unsigned long)rd);

    ad5940_ReadReg(dev, REG_AFE_TEMPSENSDAT, &rd);
    printf("%s: REG_AFE_TEMPSENSDAT= 0x%08ld\r\n", tag, (unsigned long)rd);

    ad5940_ReadReg(dev, REG_AFE_DFTREAL, &rd);
    printf("%s: REG_AFE_DFTREAL    = 0x%08ld\r\n", tag, (unsigned long)rd);

    ad5940_ReadReg(dev, REG_AFE_DFTIMAG, &rd);
    printf("%s: REG_AFE_DFTIMAG    = 0x%08ld\r\n", tag, (unsigned long)rd);

    ad5940_ReadReg(dev, REG_AFE_STATSMEAN, &rd);
    printf("%s: REG_AFE_STATSMEAN  = 0x%08ld\r\n", tag, (unsigned long)rd);

    ad5940_ReadReg(dev, REG_AFE_STATSVAR, &rd);
    printf("%s: REG_AFE_STATSVAR   = 0x%08ld\r\n", tag, (unsigned long)rd);
}

/* 
 * Decode an AFE result register value into float. 
 * Works for AFERESULT_DFTREAL / AFERESULT_DFTIMAG, which are 18-bit signed (Q2 format).
 */
float decode_afe_result(uint32_t regVal)
{
    /* Sign extend 18-bit two's complement (bits [17:0]) into 32-bit */
    int32_t val_q2 = (int32_t)(regVal << 14) >> 14;

    /* Convert Q2 fixed-point to float (divide by 4) */
    return (float)val_q2 / 4.0f;
}

/**
   @brief void AD5940_HSTIACfgS(HSTIACfg_Type *pHsTiaCfg)
          ====== Initialize High speed TIA amplifier
   @param pWGInit : {0 - 0xffffffff}
          - Pointer to configuration structure

   @return return 0 in case of success, negative error code otherwise.
 */

int ad5940_HSRtiaCal(struct ad5940_dev *dev, HSRTIACal_Type *pCalCfg, AppBiaCfg_Type *AppBiaCfg,
		     void *pResult)
{
	int ret;
	AFERefCfg_Type aferef_cfg;
	HSLoopCfg_Type hs_loop;
	DSPCfg_Type dsp_cfg;
	bool bADCClk32MHzMode = false;

    AppBiaCfg_Type *pBiaCfg;
    AppBiaGetCfg(&pBiaCfg);

	//uint32_t RtiaVal;
	//static uint32_t const HpRtiaTable[] = {200, 1000, 5000, 10000, 20000, 40000, 80000, 160000, 0};
    //printf("%s: Rtia = %ld\r\n", __func__, HpRtiaTable[AppBiaCfg->HstiaRtiaSel]);
    float RcalVal = pCalCfg->fRcal;
    fImpCar_Type ZcalVal = { RcalVal, 0.0f };

	fImpCar_Type Dcal, Dtia;
    uint32_t regVal;

	if (pCalCfg == NULL) return -EINVAL;
	if (pCalCfg->fRcal == 0)
		return -EINVAL;
    /*
	if (pCalCfg->HsTiaCfg.HstiaRtiaSel > HSTIARTIA_160K)
		return -EINVAL;
	if (pCalCfg->HsTiaCfg.HstiaRtiaSel == HSTIARTIA_OPEN)
		return -EINVAL; // Do not support calibrating DE0-RTIA 
    */
	if (pResult == NULL)
		return -EINVAL;

	if (pCalCfg->AdcClkFreq > (32000000 * 0.8))
		bADCClk32MHzMode = true;

    /* Calculate the excitation voltage we should use based on setting */
    ExcitConfig excit_config = compute_excit_config(AppBiaCfg->DesiredVoltage);
    //printf("%s: desired_vpp=%f , expected_vpp=%f\r\n",
    //        __func__, excit_config.requested_vpp, excit_config.actual_vpp);

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
    //printf("%s: line: %d\r\n", __func__, __LINE__);

	/* Configure HP Loop */
	hs_loop.HsDacCfg.ExcitBufGain = excit_config.ExcitBuffGain;
	hs_loop.HsDacCfg.HsDacGain = excit_config.HsDacGain;
	hs_loop.HsDacCfg.HsDacUpdateRate = 7; /* Set it to highest update rate */


    // RTIA or RTIA_DE0 ?
    if(pBiaCfg->HstiaDeRtia == HSTIADERTIA_OPEN) {
        // regular RTIA
        hs_loop.SWMatCfg.Dswitch = SWD_RCAL0;
        hs_loop.SWMatCfg.Pswitch = SWP_RCAL0;
        hs_loop.SWMatCfg.Nswitch = SWN_RCAL1;
        hs_loop.SWMatCfg.Tswitch = SWT_RCAL1 | SWT_TRTIA;
    } else {
        //RTIA_DE0
        hs_loop.SWMatCfg.Dswitch = SWD_RCAL0;
        hs_loop.SWMatCfg.Pswitch = SWP_RCAL0;
        hs_loop.SWMatCfg.Nswitch = SWN_RCAL1 | SWN_DE0LOAD; // NR1 + N6
        hs_loop.SWMatCfg.Tswitch = SWT_DE0LOAD; //  T10 
    }

    // is the RTIA coming from RTIA or RTIA_DE0 ?
	memcpy(&hs_loop.HsTiaCfg, &pCalCfg->HsTiaCfg, sizeof(pCalCfg->HsTiaCfg));
    if(pBiaCfg->HstiaDeRtia == HSTIADERTIA_OPEN) {
        // regular RTIA
        hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERTIA_TODE; // short HSTIA output to DE0 pin
        hs_loop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
        hs_loop.HsTiaCfg.HstiaRtiaSel = pBiaCfg->HstiaRtiaSel;
    } else {
        hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN; // just for calib
        hs_loop.HsTiaCfg.HstiaDeRtia = pBiaCfg->HstiaDeRtia;
        hs_loop.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_OPEN;
    }

	hs_loop.WgCfg.WgType = WGTYPE_SIN;
	hs_loop.WgCfg.GainCalEn =
		false;      /* @todo. If we have factory calibration data, enable it */
	hs_loop.WgCfg.OffsetCalEn = false;
	hs_loop.WgCfg.SinCfg.SinFreqWord = ad5940_WGFreqWordCal(pCalCfg->fFreq,
					   pCalCfg->SysClkFreq);
	hs_loop.WgCfg.SinCfg.SinAmplitudeWord = excit_config.WgAmpWord;
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

    /* --- Dcal --- */
    //dump_afe_registers(dev, "Zcal");
    ad5940_ReadAfeResult(dev, AFERESULT_DFTREAL, &regVal);
    Dcal.Real = decode_afe_result(regVal);

    ad5940_ReadAfeResult(dev, AFERESULT_DFTIMAGE, &regVal);
    Dcal.Image = decode_afe_result(regVal);

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

    /* --- Dtia --- */
    //dump_afe_registers(dev, "Ztia");
    ad5940_ReadAfeResult(dev, AFERESULT_DFTREAL, &regVal);
    Dtia.Real = decode_afe_result(regVal);

    ad5940_ReadAfeResult(dev, AFERESULT_DFTIMAGE, &regVal);
    Dtia.Image = decode_afe_result(regVal);

    /* Current flow & DFT convention corrections */
    Dtia.Real  =  Dtia.Real;
    Dtia.Image = -Dtia.Image;
    Dcal.Real  = -Dcal.Real;
    Dcal.Image =  Dcal.Image;

    /*
     * ADC MUX is set to HSTIA_P and HSTIA_N.
     * Current flows through RCAL into RTIA. When measuring across RCAL 
     * using MUXSELP_P_NODE and MUXSELN_N_NODE, the current direction is 
     * HSTIA_N -> HSTIA_P.
     * => So we flip the sign of both real and imag once here.
     */
    //Dtia.Real  = -Dtia.Real;
    //Dtia.Image = -Dtia.Image;

    /* The AD5940 DFT engine also inverts the imaginary part internally. */
    //Dcal.Image = -Dcal.Image;

    /* --- Do the complex ratio and scaling --- */
    fImpCar_Type Zratio   = ad5940_ComplexDivFloat(&Dtia, &Dcal);
    fImpCar_Type ZtiaVal  = ad5940_ComplexMulFloat(&Zratio, &ZcalVal);

    /* --- Debug prints: show signed integers for raw values --- */
    //printf("dtia = %.2f + %.2f j\r\n", Dtia.Real, Dtia.Image);
    //printf("dcal = %.2f + %.2f j\r\n", Dcal.Real, Dcal.Image);

    printf("  DEBUG Zratio (%.0f + %.0f j) = Dtia (%.0f + %.0f j) / Dcal (%.0f + %.0f j)\r\n",
           Zratio.Real, Zratio.Image,
           Dtia.Real, Dtia.Image,
           Dcal.Real, Dcal.Image);

    printf("  DEBUG ZtiaVal (%.0f + %.0f j) = Zratio (%.0f + %.0f j) * ZcalVal (%.0f + %.0f j)\r\n",
           ZtiaVal.Real, ZtiaVal.Image,
           Zratio.Real, Zratio.Image,
           ZcalVal.Real, ZcalVal.Image);

    if (pCalCfg->bPolarResult == false) {
        *((fImpCar_Type*)pResult) = ZtiaVal;
        AppBiaCfg->ZtiaCalCurrValue = ZtiaVal;

    } else {
        float ZtiaValMag   = ad5940_ComplexMag(&ZtiaVal);
        float ZtiaValPhase = ad5940_ComplexPhase(&ZtiaVal);

        ((fImpPol_Type*)pResult)->Magnitude = ZtiaValMag;
        ((fImpPol_Type*)pResult)->Phase     = ZtiaValPhase;
    }
    return 0;
}



