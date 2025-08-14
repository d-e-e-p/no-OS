#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include "no_os_delay.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_alloc.h"
#include "ad5940.h"
#include "calibrate.h"

/**
   @brief void AD5940_HSTIACfgS(HSTIACfg_Type *pHsTiaCfg)
          ====== Initialize High speed TIA amplifier
   @param pWGInit : {0 - 0xffffffff}
          - Pointer to configuration structure

   @return return 0 in case of success, negative error code otherwise.
 */

int ad5940_HSRtiaCal(struct ad5940_dev *dev, HSRTIACal_Type *pCalCfg,
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

	iImpCar_Type DftRcal, DftRtia;

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
	/*
	DAC output voltage calculation
	Note: RCAL value should be similar to RTIA so the accuracy is best.
	HSTIA output voltage should be limited to 0.2V to AVDD-0.2V, with 1.1V bias. We use 80% of this range for safe.
	Because the bias voltage is fixed to 1.1V, so for AC signal maximum amplitude is 1.1V-0.2V = 0.9Vp. That's 1.8Vpp.
	Formula is:    ExcitVolt(in mVpp) = (1800mVpp*80% / RTIA) * RCAL
	ADC input range is +-1.5V which is enough for calibration.

	 */
	ExcitVolt = 1800 * 0.8 * pCalCfg->fRcal / RtiaVal;

	if (ExcitVolt <=
	    800 * 0.05) { /* Voltage is so small that we can enable the attenuator of DAC(1/5) and Excitation buffer(1/4). 800mVpp is the DAC output voltage */
		ExcitBuffGain = EXCITBUFGAIN_0P25;
		HsDacGain = HSDACGAIN_0P2;
		/* Excitation buffer voltage full range is 800mVpp*0.05 = 40mVpp */
		WgAmpWord = ((uint32_t)(ExcitVolt / 40 * 2047 * 2) + 1)
			    >> 1; /* Assign value with rounding (0.5 LSB error) */
	} else if (ExcitVolt <= 800 * 0.25) { /* Enable Excitation buffer attenuator */
		ExcitBuffGain = EXCITBUFGAIN_0P25;
		HsDacGain = HSDACGAIN_1;
		/* Excitation buffer voltage full range is 800mVpp*0.25 = 200mVpp */
		WgAmpWord = ((uint32_t)(ExcitVolt / 200 * 2047 * 2) + 1)
			    >> 1; /* Assign value with rounding (0.5 LSB error) */
	} else if (ExcitVolt <= 800 * 0.4) { /* Enable DAC attenuator */
		ExcitBuffGain = EXCITBUFGAIN_2;
		HsDacGain = HSDACGAIN_0P2;
		/* Excitation buffer voltage full range is 800mVpp*0.4 = 320mV */
		WgAmpWord = ((uint32_t)(ExcitVolt / 320 * 2047 * 2) + 1)
			    >> 1; /* Assign value with rounding (0.5 LSB error) */
	} else { /* No attenuator is needed. This is the best condition which means RTIA is close to RCAL */
		ExcitBuffGain = EXCITBUFGAIN_2;
		HsDacGain = HSDACGAIN_1;
		/* Excitation buffer voltage full range is 800mVpp*2=1600mVpp */
		WgAmpWord = ((uint32_t)(ExcitVolt / 1600 * 2047 * 2) + 1)
			    >> 1; /* Assign value with rounding (0.5 LSB error) */
	}
    printf("ad5940_HSRtiaCal: using RtiaVal=%lu ExcitVolt=%f\r\n", RtiaVal, ExcitVolt);


	if (WgAmpWord > 0x7ff)
		WgAmpWord = 0x7ff;

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
	hs_loop.SWMatCfg.Dswitch = SWD_RCAL0;
	hs_loop.SWMatCfg.Pswitch = SWP_RCAL0;
	hs_loop.SWMatCfg.Nswitch = SWN_RCAL1;
	hs_loop.SWMatCfg.Tswitch = SWT_RCAL1 | SWT_TRTIA;
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
	dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_1; /* @todo Change the gain? */
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

	ret = ad5940_ReadAfeResult(dev, AFERESULT_DFTREAL, (uint32_t *)&DftRcal.Real);
	if (ret < 0)
		return ret;

	ret = ad5940_ReadAfeResult(dev, AFERESULT_DFTIMAGE, (uint32_t *)&DftRcal.Image);
	if (ret < 0)
		return ret;

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

	ret = ad5940_ReadAfeResult(dev, AFERESULT_DFTREAL, (uint32_t *)&DftRtia.Real);
	if (ret < 0)
		return ret;

	ret = ad5940_ReadAfeResult(dev, AFERESULT_DFTIMAGE, (uint32_t *)&DftRtia.Image);
	if (ret < 0)
		return ret;

	if (DftRcal.Real & (1 << 17))
		DftRcal.Real |= 0xfffc0000;
	if (DftRcal.Image & (1 << 17))
		DftRcal.Image |= 0xfffc0000;
	if (DftRtia.Real & (1 << 17))
		DftRtia.Real |= 0xfffc0000;
	if (DftRtia.Image & (1 << 17))
		DftRtia.Image |= 0xfffc0000;
	/*
	ADC MUX is set to HSTIA_P and HSTIA_N.
	While the current flow through RCAL and then into RTIA, the current direction should be from HSTIA_N to HSTIA_P if we
	measure the voltage across RCAL by MUXSELP_P_NODE and MUXSELN_N_NODE.
	So here, we add a negative sign to results
	 */
	DftRtia.Image = -DftRtia.Image;
	DftRtia.Real =
		-DftRtia.Real; /* Current is measured by MUX HSTIA_P-HSTIA_N. It should be  */
	/*
	The impedance engine inside of AD594x give us Real part and Imaginary part of DFT. Due to technology used, the Imaginary
	part in register is the opposite number. So we add a negative sign on the Imaginary part of results.
	 */
	DftRtia.Image = -DftRtia.Image;
	DftRcal.Image = -DftRcal.Image;

	if (pCalCfg->bPolarResult == false) {
		float temp = (float)DftRcal.Real * DftRcal.Real + (float)
			     DftRcal.Image * DftRcal.Image;

		/* RTIA = (DftRtia.Real, DftRtia.Image)/(DftRcal.Real, DftRcal.Image)*fRcal */
		((fImpCar_Type*)pResult)->Real = ((float)DftRtia.Real * DftRcal.Real +
						  (float)DftRtia.Image * DftRcal.Image) / temp * pCalCfg->fRcal; /* Real Part */
		((fImpCar_Type*)pResult)->Image = ((float)DftRtia.Image * DftRcal.Real -
						   (float)DftRtia.Real * DftRcal.Image) / temp *
						  pCalCfg->fRcal; /* Imaginary Part */
	} else {
		float RcalMag, RtiaMag, RtiaPhase;
		RcalMag = sqrt((float)DftRcal.Real * DftRcal.Real + (float)
			       DftRcal.Image * DftRcal.Image);
		RtiaMag = sqrt((float)DftRtia.Real * DftRtia.Real + (float)
			       DftRtia.Image * DftRtia.Image);
		RtiaMag = (RtiaMag / RcalMag) * pCalCfg->fRcal;
		RtiaPhase = atan2(DftRtia.Image, DftRtia.Real) - atan2(DftRcal.Image,
				DftRcal.Real);

		((fImpPol_Type*)pResult)->Magnitude = RtiaMag;
		((fImpPol_Type*)pResult)->Phase = RtiaPhase;
		printf("RTIA mag:%f,",RtiaMag);
		printf("phase:%f\r\n",RtiaPhase*180/MATH_PI);
	}
	return 0;
}


