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
int ad5940_HSTIACfgS(struct ad5940_dev *dev, HSTIACfg_Type *pHsTiaCfg)
{
	int ret;
	uint32_t tempreg;

	if (pHsTiaCfg == NULL) return -EINVAL;
	/* Available parameter is 1k, 5k,...,160k, short, OPEN */
	if (pHsTiaCfg->HstiaDeRtia < HSTIADERTIA_1K)
		return -EINVAL;
	if (pHsTiaCfg->HstiaDeRtia > HSTIADERTIA_OPEN)
		return -EINVAL;  /* Parameter is invalid */

	if (pHsTiaCfg->HstiaDeRload > HSTIADERLOAD_OPEN)
		return -EINVAL;  /* Available parameter is OPEN, 0R,..., 100R */

	tempreg = 0;
	tempreg |= pHsTiaCfg->HstiaBias;
	ret = ad5940_WriteReg(dev, REG_AFE_HSTIACON, tempreg);
	if (ret < 0)
		return ret;

	/* HSRTIACON */
	/* Calculate CTIA value */
	tempreg = pHsTiaCfg->HstiaCtia << BITP_AFE_HSRTIACON_CTIACON;
	tempreg |= pHsTiaCfg->HstiaRtiaSel;
	if (pHsTiaCfg->DiodeClose == true)
		tempreg |= BITM_AFE_HSRTIACON_TIASW6CON; /* Close switch 6 */
	ret = ad5940_WriteReg(dev, REG_AFE_HSRTIACON, tempreg);
	if (ret < 0)
		return ret;

	/* HSTIARES03CON */
	tempreg = 0;
	/* deal with HSTIA DE RTIA */
	if (pHsTiaCfg->HstiaDeRtia >= HSTIADERTIA_OPEN)
		tempreg = 0x1f << 3;  /* bit field HPTIRES03CON[7:3] */
	else if (pHsTiaCfg->HstiaDeRtia >= HSTIADERTIA_1K) {
		tempreg = (pHsTiaCfg->HstiaDeRtia - 3 + 11) << 3;
	} else { /* DERTIA 50/100/200Ohm */
		const uint8_t DeRtiaTable[3][5] = {
			/*Rload  0      10    30    50    100 */
			{ 0x00, 0x01, 0x02, 0x03, 0x06,}, /* RTIA 50Ohm */
			{ 0x03, 0x04, 0x05, 0x06, 0x07,},  /* RTIA 100Ohm */
			{ 0x07, 0x07, 0x09, 0x09, 0x0a,} /* RTIA 200Ohm */
		};
		if (pHsTiaCfg->HstiaDeRload < HSTIADERLOAD_OPEN)
			tempreg = (uint32_t)(
					  DeRtiaTable[pHsTiaCfg->HstiaDeRtia][pHsTiaCfg->HstiaDeRload]) << 3;
		else
			tempreg = (0x1f) << 3; /* Set it to HSTIADERTIA_OPEN. This setting is illegal */
	}
	/* deal with HSTIA Rload */
	tempreg |= pHsTiaCfg->HstiaDeRload;

	return ad5940_WriteReg(dev, REG_AFE_DE0RESCON, tempreg);
}


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

/**
  * @brief Calibrate ADC PGA. It will measure the offset and gain of each PGA gain.
  * @note This function will do the calibration for all PGA gains.
  * @param pADCPGACal: Pointer to configuration structure.
  * @return return 0 in case of success, negative error code otherwise.
*/
int ad5940_ADCPGACal(struct ad5940_dev *dev, ADCPGACal_Type *pADCPGACal)
{
  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type hs_loop_cfg;
  DSPCfg_Type dsp_cfg;
  int32_t CalResult;
  uint32_t count;
  fImpCar_Type DftRcal;

  if(pADCPGACal == NULL)
    return AD5940ERR_NULLP;
  if(pADCPGACal->VRef1p82 == 0)
    return AD5940ERR_PARA;
  if(pADCPGACal->VRef1p11 == 0)
    return AD5940ERR_PARA;

  printf("Start ADC PGA Calibration");

  /* Use HSDAC as voltage source */
  ad5940_AFECtrlS(dev, AFECTRL_ALL, false);  /* Init all to disable state */

  aferef_cfg.HpBandgapEn = true;
  aferef_cfg.Hp1V1BuffEn = true;
  aferef_cfg.Hp1V8BuffEn = true;
  aferef_cfg.Disc1V1Cap = false;
  aferef_cfg.Disc1V8Cap = false;
  aferef_cfg.Hp1V8ThemBuff = false;
  aferef_cfg.Hp1V8Ilimit = false;
  aferef_cfg.Lp1V1BuffEn = true;
  aferef_cfg.Lp1V8BuffEn = true;
  aferef_cfg.LpBandgapEn = true;
  aferef_cfg.LpRefBufEn = true;
  aferef_cfg.LpRefBoostEn = false;
  ad5940_REFCfgS(dev, &aferef_cfg);

  hs_loop_cfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
  hs_loop_cfg.HsDacCfg.HsDacGain = HSDACGAIN_1;
  hs_loop_cfg.HsDacCfg.HsDacUpdateRate = 7;
  hs_loop_cfg.HsTiaCfg.DiodeClose = false;
  hs_loop_cfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hs_loop_cfg.HsTiaCfg.HstiaCtia = 31;
  hs_loop_cfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop_cfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop_cfg.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_OPEN;
  hs_loop_cfg.SWMatCfg.Dswitch = SWD_OPEN;
  hs_loop_cfg.SWMatCfg.Pswitch = SWP_OPEN;
  hs_loop_cfg.SWMatCfg.Nswitch = SWN_OPEN;
  hs_loop_cfg.SWMatCfg.Tswitch = SWT_OPEN;
  hs_loop_cfg.WgCfg.WgType = WGTYPE_SIN;
  hs_loop_cfg.WgCfg.GainCalEn = false;
  hs_loop_cfg.WgCfg.OffsetCalEn = false;
  hs_loop_cfg.WgCfg.SinCfg.SinFreqWord = ad5940_WGFreqWordCal(pADCPGACal->fFreq, pADCPGACal->SysClkFreq);
  hs_loop_cfg.WgCfg.SinCfg.SinAmplitudeWord = 500;
  hs_loop_cfg.WgCfg.SinCfg.SinOffsetWord = 0;
  hs_loop_cfg.WgCfg.SinCfg.SinPhaseWord = 0;
  ad5940_HSLoopCfgS(&hs_loop_cfg);

  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_VSET1P1;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_VSET1P1;
  dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_1;
  AD5940_StructInit(&dsp_cfg.ADCDigCompCfg, sizeof(dsp_cfg.ADCDigCompCfg));
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = pADCPGACal->ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = pADCPGACal->ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpNotch = true;
  dsp_cfg.ADCFilterCfg.BpSinc3 = false;
  dsp_cfg.ADCFilterCfg.DFTClkEnable = true;
  dsp_cfg.ADCFilterCfg.Sinc2NotchClkEnable = true;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = true;
  dsp_cfg.ADCFilterCfg.Sinc3ClkEnable = true;
  dsp_cfg.ADCFilterCfg.WGClkEnable = true;
  memcpy(&dsp_cfg.DftCfg, &pADCPGACal->DftCfg, sizeof(pADCPGACal->DftCfg));
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg));
  ad5940_DSPCfgS(&dsp_cfg);

  /* Calibrate ADC offset for each PGA gain setting */
  printf("Calibrating ADC offset for each PGA gain setting\n");
  for(count=0; count<5; count++)
  {
    uint32_t RegAddr;
    ad5940_ADCPGACal(count, pADCPGACal->ADCPower);
    ad5940_AFECtrlS(dev, AFECTRL_ADCPWR, true);
    ad5940_AFECtrlS(dev, AFECTRL_ADCCNV, true);
    ad5940_INTCClrFlag(AFEINTSRC_ADCDONE);
    while(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_ADCDONE) == false);
    ad5940_AFECtrlS(dev, AFECTRL_ADCCNV|AFECTRL_ADCPWR, false);
    CalResult = ad5940_ReadAfeResult(AFERESULT_SINC3);
    CalResult -= 32768; /* Ideal result is 0x8000. */
    CalResult /= 4; /* LSB of ADCOFFSET is 0.25LSB of ADC result */
    RegAddr = REG_AFE_ADCOFFSETGN0 + count*4;
    ad5940_WriteReg(RegAddr, CalResult);
    printf("PGA Gain %u, Offset: %ld\n", count, CalResult);
  }

  /* Calibrate ADC gain for PGA gain of 1, 1.5 and 2 */
  printf("Calibrating ADC gain for PGA gain of 1, 1.5 and 2\n");
  ad5940_ADCMuxCfgS(ADCMUXP_VREF1P8, ADCMUXN_VSET1P1);
  for(count=0; count<3; count++)
  {
    uint32_t RegAddr;
    float IdealAdcCode;
    ad5940_ADCPGACal(count, pADCPGACal->ADCPower);
    ad5940_AFECtrlS(dev, AFECTRL_ADCPWR, true);
    ad5940_AFECtrlS(dev, AFECTRL_ADCCNV, true);
    ad5940_INTCClrFlag(AFEINTSRC_ADCDONE);
    while(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_ADCDONE) == false);
    ad5940_AFECtrlS(dev, AFECTRL_ADCCNV|AFECTRL_ADCPWR, false);
    CalResult = ad5940_ReadAfeResult(AFERESULT_SINC3);
    IdealAdcCode = (pADCPGACal->VRef1p82 - pADCPGACal->VRef1p11)*AD5940_ADCPgaGain[count]/1.82*32768+32768;
    CalResult = (int32_t)((float)CalResult*32768/IdealAdcCode);
    RegAddr = REG_AFE_ADCGAINGN0 + count*4;
    ad5940_WriteReg(RegAddr, CalResult);
    printf("PGA Gain %u, Gain: %ld\n", count, CalResult);
  }

  /* Calibrate ADC gain for PGA gain of 4 and 9 */
  printf("Calibrating ADC gain for PGA gain of 4 and 9\n");
  ad5940_AFECtrlS(dev, AFECTRL_HSDACPWR|AFECTRL_DACREFPWR|AFECTRL_EXTBUFPWR|AFECTRL_INAMPPWR|AFECTRL_HSTIAPWR, true);
  ad5940_ADCMuxCfgS(ADCMUXP_P_NODE, ADCMUXN_N_NODE);
  AD5940_HSLoopCfgS(&hs_loop_cfg);
  ad5940_AFECtrlS(dev, AFECTRL_WG | AFECTRL_ADCPWR, true);
  ad5940_AFECtrlS(dev, AFECTRL_ADCCNV | AFECTRL_DFT, true);
  ad5940_INTCClrFlag(AFEINTSRC_DFTRDY);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DFTRDY) == false);
  ad5940_AFECtrlS(dev, AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR, false);
  DftRcal.Real = ad5940_ReadAfeResult(AFERESULT_DFTREAL);
  DftRcal.Image = ad5940_ReadAfeResult(AFERESULT_DFTIMAGE);
  if(DftRcal.Real&(1<<17)) DftRcal.Real |= 0xfffc0000;
  if(DftRcal.Image&(1<<17)) DftRcal.Image |= 0xfffc0000;

  for(count=3; count<5; count++)
  {
    uint32_t RegAddr;
    float IdealAdcMag;
    iImpCar_Type DftAdc;
    float AdcMag;

    ad5940_ADCPGACal(count, pADCPGACal->ADCPower);
    ad5940_AFECtrlS(dev, AFECTRL_WG | AFECTRL_ADCPWR, true);
    ad5940_AFECtrlS(dev, AFECTRL_ADCCNV | AFECTRL_DFT, true);
    ad5940_INTCClrFlag(AFEINTSRC_DFTRDY);
    while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DFTRDY) == false);
    ad5940_AFECtrlS(dev, AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR, false);
    DftAdc.Real = ad5940_ReadAfeResult(AFERESULT_DFTREAL);
    DftAdc.Image = ad5940_ReadAfeResult(AFERESULT_DFTIMAGE);
    if(DftAdc.Real&(1<<17)) DftAdc.Real |= 0xfffc0000;
    if(DftAdc.Image&(1<<17)) DftAdc.Image |= 0xfffc0000;

    AdcMag = sqrt((float)DftAdc.Real*DftAdc.Real + (float)DftAdc.Image*DftAdc.Image);
    IdealAdcMag = sqrt((float)DftRcal.Real*DftRcal.Real + (float)DftRcal.Image*DftRcal.Image);
    CalResult = (int32_t)(AdcMag*32768/IdealAdcMag);
    RegAddr = REG_AFE_ADCGAINGN0 + count*4;
    ad5940_WriteReg(RegAddr, CalResult);
    printf("PGA Gain %u, Gain: %ld\n", count, CalResult);
  }

  ad5940_AFECtrlS(dev, AFECTRL_ALL, false);
  printf("ADC PGA Calibration finished\n");
  return AD5940ERR_OK;
}

/**
  * @brief Calibrate HSDAC. It will measure the offset of HSDAC.
  * @note This function will do the calibration for all HSDAC gain settings.
  * @param pCalCfg: Pointer to configuration structure.
  * @return return 0 in case of success, negative error code otherwise.
*/
int ad5940_HSDACCal(struct ad5940_dev *dev, HSDACCal_Type *pCalCfg)
{
  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type hs_loop_cfg;
  DSPCfg_Type dsp_cfg;
  int32_t CalResult;
  uint32_t count;

  if(pCalCfg == NULL)
    return AD5940ERR_NULLP;

  printf("Start HSDAC Calibration\n");

  ad5940_AFECtrlS(dev, AFECTRL_ALL, false);

  aferef_cfg.HpBandgapEn = true;
  aferef_cfg.Hp1V1BuffEn = true;
  aferef_cfg.Hp1V8BuffEn = true;
  aferef_cfg.Disc1V1Cap = false;
  aferef_cfg.Disc1V8Cap = false;
  aferef_cfg.Hp1V8ThemBuff = false;
  aferef_cfg.Hp1V8Ilimit = false;
  aferef_cfg.Lp1V1BuffEn = true;
  aferef_cfg.Lp1V8BuffEn = true;
  aferef_cfg.LpBandgapEn = true;
  aferef_cfg.LpRefBufEn = true;
  aferef_cfg.LpRefBoostEn = false;
  ad5940_REFCfgS(dev, &aferef_cfg);

  hs_loop_cfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
  hs_loop_cfg.HsDacCfg.HsDacGain = HSDACGAIN_1;
  hs_loop_cfg.HsDacCfg.HsDacUpdateRate = 7;
  hs_loop_cfg.HsTiaCfg.DiodeClose = false;
  hs_loop_cfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hs_loop_cfg.HsTiaCfg.HstiaCtia = 31;
  hs_loop_cfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop_cfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop_cfg.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_OPEN;
  hs_loop_cfg.SWMatCfg.Dswitch = SWD_RCAL0;
  hs_loop_cfg.SWMatCfg.Pswitch = SWP_RCAL0;
  hs_loop_cfg.SWMatCfg.Nswitch = SWN_RCAL1;
  hs_loop_cfg.SWMatCfg.Tswitch = SWT_RCAL1;
  hs_loop_cfg.WgCfg.WgType = WGTYPE_DC;
  hs_loop_cfg.WgCfg.GainCalEn = false;
  hs_loop_cfg.WgCfg.OffsetCalEn = false;
  hs_loop_cfg.WgCfg.DcCfg.DcOffsetWord = 0x800;
  AD5940_HSLoopCfgS(&hs_loop_cfg);

  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_N_NODE;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_P_NODE;
  dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_1;
  AD5940_StructInit(&dsp_cfg.ADCDigCompCfg, sizeof(dsp_cfg.ADCDigCompCfg));
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = pCalCfg->ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = pCalCfg->ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpNotch = true;
  dsp_cfg.ADCFilterCfg.BpSinc3 = false;
  dsp_cfg.ADCFilterCfg.DFTClkEnable = false;
  dsp_cfg.ADCFilterCfg.Sinc2NotchClkEnable = true;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = true;
  dsp_cfg.ADCFilterCfg.Sinc3ClkEnable = true;
  dsp_cfg.ADCFilterCfg.WGClkEnable = true;
  memset(&dsp_cfg.DftCfg, 0, sizeof(dsp_cfg.DftCfg));
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg));
  ad5940_DSPCfgS(&dsp_cfg);

  ad5940_AFECtrlS(dev, AFECTRL_HSDACPWR|AFECTRL_DACREFPWR|AFECTRL_EXTBUFPWR|AFECTRL_INAMPPWR|AFECTRL_HSTIAPWR, true);
  ad5940_AFECtrlS(dev, AFECTRL_WG | AFECTRL_ADCPWR, true);

  /* Calibrate HSDAC offset for all gain settings */
  printf("Calibrating HSDAC offset for all gain settings\n");
  for(count=0; count<4; count++)
  {
    uint32_t RegAddr;
    AD5940_HSDacPwr(pCalCfg->HSDacPwr);
    AD5940_HSDacGain(count);
    ad5940_AFECtrlS(dev, AFECTRL_ADCCNV, true);
    ad5940_INTCClrFlag(AFEINTSRC_ADCDONE);
    while(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_ADCDONE) == false);
    ad5940_AFECtrlS(dev, AFECTRL_ADCCNV, false);
    CalResult = ad5940_ReadAfeResult(AFERESULT_SINC3);
    CalResult -= 32768;
    if(pCalCfg->HSDacPwr == HSDACPWR_LOW)
      RegAddr = REG_AFE_DACOFFSETATTEN + count*4;
    else
      RegAddr = REG_AFE_DACOFFSETHP + count*4;
    ad5940_WriteReg(RegAddr, CalResult);
    printf("HSDAC Gain %u, Offset: %ld\n", count, CalResult);
  }

  ad5940_AFECtrlS(dev, AFECTRL_ALL, false);
  printf("HSDAC Calibration finished\n");
  return AD5940ERR_OK;
}


