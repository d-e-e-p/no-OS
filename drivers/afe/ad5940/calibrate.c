/**
 * AD5940 RCAL Calibration Example
 * 
 * The goal is to measure the differential voltage across RCAL and adjust
 * the DAC offset until the voltage is ~0V. This calibrates the system
 * for accurate impedance measurements.
 */

#include "ad5940.h"
#include "calibrate.h"

/**
 * @brief Helper function to perform a DFT measurement and read the raw result.
 * @param dev: AD5940 device structure.
 * @param pDftResult: Pointer to store the DFT result (real and imaginary parts).
 * @return 0 on success, negative error code on failure.
 */
static int AD5940_MeasureRcalDft(struct ad5940_dev *dev, iImpCar_Type *pDftResult)
{
    int ret;

    /* Start excitation and measurement */
    ret = ad5940_AFECtrlS(dev, AFECTRL_WG | AFECTRL_ADCPWR, true);
    if (ret < 0) return ret;
    
    ret = ad5940_AFECtrlS(dev, AFECTRL_ADCCNV | AFECTRL_DFT, true);
    if (ret < 0) return ret;
    
    /* Wait for DFT completion */
    while (ad5940_INTCTestFlag(dev, AFEINTC_1, AFEINTSRC_DFTRDY) == false);
    
    /* Stop measurement */
    ret = ad5940_AFECtrlS(dev, 
                  AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR, 
                  false);
    if (ret < 0) return ret;
    
    /* Clear interrupt flag */
    ret = ad5940_INTCClrFlag(dev, AFEINTSRC_DFTRDY);
    if (ret < 0) return ret;
    
    /* Read DFT results */
    ret = ad5940_ReadAfeResult(dev, AFERESULT_DFTREAL, (uint32_t *)&pDftResult->Real);
    if (ret < 0) return ret;
    
    ret = ad5940_ReadAfeResult(dev, AFERESULT_DFTIMAGE, (uint32_t *)&pDftResult->Image);
    if (ret < 0) return ret;

    return 0;
}

/**
 * @brief Perform RCAL calibration accounting for RTIA
 * @param dev: AD5940 device structure
 * @param pCalCfg: Calibration configuration
 * @return 0 on success, negative error code on failure
 * 
 * Uses AppBiaCfg.RcalVal and AppBiaCfg.RtiaCurrValue for calibration values
 */
int AD5940_RcalCalibration(struct ad5940_dev *dev, 
                          CalCfg_Type *pCalCfg)
{
    int ret;
    DSPCfg_Type dsp_cfg;
    DFTCfg_Type dft_cfg;
    iImpCar_Type DftRcal;
    uint32_t DACCode = 0x800;  // Start with mid-scale DAC code
    uint32_t iteration = 0;
    const uint32_t MAX_ITERATIONS = 20;
    const float VOLTAGE_THRESHOLD = 1.0e-6f;  // 1µV threshold
    float MeasuredVoltage;
    float ExpectedCurrent;
    float SystemGain;
    float VoltageError;
    
    // Get calibration values from global config
    float RcalVal = AppBiaCfg.RcalVal;
    fImpCar_Type Zrtia = { 
        .Real  = AppBiaCfg.RtiaCurrValue[0], 
        .Image = AppBiaCfg.RtiaCurrValue[1] 
    };
    
    printf("=== RCAL Calibration Start ===\r\n");
    printf("RcalVal: %.0f ohm\r\n", RcalVal);
    printf("Zrtia: (%.0f + %.0f j) ohm\r\n", Zrtia.Real, Zrtia.Image);
    
    // Calculate expected system impedance (assume RTIA magnitude for DC analysis)
    float RTIAMag = sqrtf(Zrtia.Real * Zrtia.Real + Zrtia.Image * Zrtia.Image);
    float SystemZ = RcalVal + RTIAMag;  // Total impedance magnitude in series
    
    printf("RTIA Magnitude: %.0f ohm\r\n", RTIAMag);
    printf("Expected System Z: %.0f ohm\r\n", SystemZ);
    
    // Initialize DSP configuration
    AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
    
    /* Configure ADC MUX to measure across RCAL */
    dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_P_NODE;   // Positive terminal of RCAL
    dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_N_NODE;   // Negative terminal of RCAL
    dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_1;          // Unity gain
    
    /* Configure ADC filter settings */
    dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;
    dsp_cfg.ADCFilterCfg.ADCSinc2Osr = pCalCfg->ADCSinc2Osr;
    dsp_cfg.ADCFilterCfg.ADCSinc3Osr = pCalCfg->ADCSinc3Osr;
    dsp_cfg.ADCFilterCfg.BpSinc3 = false;
    dsp_cfg.ADCFilterCfg.BpNotch = true;
    dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = true;
    dsp_cfg.ADCFilterCfg.DFTClkEnable = true;
    dsp_cfg.ADCFilterCfg.WGClkEnable = true;
    
    /* Configure DFT for single frequency measurement */
    memcpy(&dsp_cfg.DftCfg, &pCalCfg->DftCfg, sizeof(pCalCfg->DftCfg));
    memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg));
    
    // Apply DSP configuration
    ret = ad5940_DSPCfgS(dev, &dsp_cfg);
    if (ret < 0) return ret;
    
    /* Configure excitation DAC to apply known current through RCAL */
    WGCfg_Type wg_cfg;
    AD5940_StructInit(&wg_cfg, sizeof(wg_cfg));
    wg_cfg.WgType = WGTYPE_SIN;
    wg_cfg.GainCalEn = false;
    wg_cfg.OffsetCalEn = false;
    wg_cfg.SinCfg.SinFreqWord = pCalCfg->DftCfg.DftSrc;  // Same frequency as DFT
    wg_cfg.SinCfg.SinAmplitudeWord = 0x1FF;              // Full scale amplitude
    wg_cfg.SinCfg.SinOffsetWord = DACCode;               // Starting DAC offset
    wg_cfg.SinCfg.SinPhaseWord = 0;
    
    ret = ad5940_WGCfgS(dev, &wg_cfg);
    if (ret < 0) return ret;
    
    // Iterative calibration loop
    printf("\n--- Starting Calibration Iterations ---\r\n");
    while (iteration < MAX_ITERATIONS) {
        printf("Iteration %d: DACCode = 0x%03X\r\n", iteration + 1, DACCode);
        /* Enable AFE components for measurement */
        ret = ad5940_AFECtrlS(dev, 
                      AFECTRL_HSTIAPWR | AFECTRL_INAMPPWR | AFECTRL_EXTBUFPWR |
                      AFECTRL_DACREFPWR | AFECTRL_HSDACPWR | AFECTRL_SINC2NOTCH, 
                      true);
        if (ret < 0) return ret;
        
        /* Perform measurement */
        ret = AD5940_MeasureRcalDft(dev, &DftRcal);
        if (ret < 0) return ret;
        
        printf("  Raw DFT: Real=%d, Image=%d\r\n", DftRcal.Real, DftRcal.Image);
        
        // Apply sign corrections similar to computeImpedance function
        // AD594x's DFT hardware outputs the imaginary part with an opposite sign
        DftRcal.Image = -DftRcal.Image;
        printf("  Corrected DFT: Real=%d, Image=%d\r\n", DftRcal.Real, DftRcal.Image);
        
        /* Calculate measured voltage magnitude */
        float RealVolt = DftRcal.Real * pCalCfg->AdcPgaGain * pCalCfg->ADCRefVolt / 32768.0f;
        float ImagVolt = DftRcal.Image * pCalCfg->AdcPgaGain * pCalCfg->ADCRefVolt / 32768.0f;
        MeasuredVoltage = sqrtf(RealVolt * RealVolt + ImagVolt * ImagVolt);
        
        printf("  Voltage: Real=%.6f V, Imag=%.6f V, Mag=%.6f V\r\n", 
               RealVolt, ImagVolt, MeasuredVoltage);
        
        /* Calculate expected voltage based on known system impedance */
        // Expected current from DAC excitation
        float DACVoltage = (DACCode - 0x800) * pCalCfg->DACRefVolt / 2048.0f;  // Convert DAC code to voltage
        ExpectedCurrent = DACVoltage / SystemZ;  // I = V / (RCAL + RTIA)
        float ExpectedVoltageAcrossRCAL = ExpectedCurrent * RcalVal;  // V = I * RCAL only
        
        printf("  DAC Voltage: %.6f V\r\n", DACVoltage);
        printf("  Expected Current: %.9f A\r\n", ExpectedCurrent);
        printf("  Expected RCAL Voltage: %.6f V\r\n", ExpectedVoltageAcrossRCAL);
        
        /* For proper calibration, we want the voltage across RCAL to be what we expect
           based on the known current and RCAL value */
        VoltageError = MeasuredVoltage - ExpectedVoltageAcrossRCAL;
        
        printf("  Voltage Error: %.6f V (%.3f%%)\r\n", 
               VoltageError, 
               ExpectedVoltageAcrossRCAL != 0 ? (VoltageError / ExpectedVoltageAcrossRCAL) * 100.0f : 0.0f);
        
        /* Check if voltage error is within threshold */
        if (fabsf(VoltageError) < VOLTAGE_THRESHOLD) {
            // Calibration successful - store system gain for future use
            SystemGain = MeasuredVoltage / ExpectedVoltageAcrossRCAL;
            pCalCfg->SysGainFactor = SystemGain;  // Store for impedance calculations
            printf("  *** CALIBRATION CONVERGED ***\r\n");
            printf("  System Gain Factor: %.6f\r\n", SystemGain);
            break;
        }
        
        /* Adjust DAC offset based on voltage error */
        uint32_t DACCodeOld = DACCode;
        if (VoltageError > 0) {
            DACCode -= (uint32_t)(fabsf(VoltageError) * 1000);  // Proportional adjustment
        } else {
            DACCode += (uint32_t)(fabsf(VoltageError) * 1000);
        }
        
        printf("  DAC Adjustment: 0x%03X -> 0x%03X (delta=%d)\r\n", 
               DACCodeOld, DACCode, (int32_t)DACCode - (int32_t)DACCodeOld);
        
        /* Clamp DAC code to valid range */
        if (DACCode > 0xFFF) {
            DACCode = 0xFFF;
            printf("  DAC code clamped to max: 0x%03X\r\n", DACCode);
        }
        if (DACCode < 0x001) {
            DACCode = 0x001;
            printf("  DAC code clamped to min: 0x%03X\r\n", DACCode);
        }
        
        /* Update waveform generator with new offset */
        wg_cfg.SinCfg.SinOffsetWord = DACCode;
        ret = ad5940_WGCfgS(dev, &wg_cfg);
        if (ret < 0) return ret;
        
        iteration++;
        printf("\r\n");  // Add spacing between iterations
    }
    
    if (iteration >= MAX_ITERATIONS) {
        printf("*** CALIBRATION FAILED - Max iterations reached ***\r\n");
        printf("Final voltage error: %.6f V\r\n", VoltageError);
        return -1;  // Calibration failed to converge
    }
    
    /* Store calibrated offset in appropriate register */
    printf("--- Storing Calibration ---\r\n");
    if (pCalCfg->HsDacCon & (1 << 12)) {
        // High range mode - use DACOFFSETHS register
        printf("Storing DAC offset 0x%03X to DACOFFSETHS (high range mode)\r\n", DACCode);
        ret = ad5940_WriteReg(dev, REG_AFE_DACOFFSETHS, DACCode);
    } else {
        // Low range mode - use DACOFFSET register  
        printf("Storing DAC offset 0x%03X to DACOFFSET (low range mode)\r\n", DACCode);
        ret = ad5940_WriteReg(dev, REG_AFE_DACOFFSET, DACCode);
    }
    
    printf("=== RCAL Calibration Complete ===\r\n");
    printf("Final DAC Code: 0x%03X\r\n", DACCode);
    printf("System Gain Factor: %.6f\r\n", pCalCfg->SysGainFactor);
    printf("Iterations: %d\r\n", iteration);
    printf("\r\n");
    
    return ret;
}