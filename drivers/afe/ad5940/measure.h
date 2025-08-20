
#ifndef MEASURE_H_
#define MEASURE_H_

#include "ad5940.h"
#include "calibrate.h"
#include "bia_measurement.h"

// WgAmpWord config
typedef struct {
    float requested_vpp;    // what you asked for
    float actual_vpp;       // what the hardware will generate
    uint16_t WgAmpWord;     // DAC word (0–2047)
    uint8_t ExcitBuffGain;  // excitation buffer gain
    uint8_t HsDacGain;      // DAC atten
} ExcitConfig;

fImpCar_Type computeImpedanceFromFifo(AppBiaCfg_Type *pBiaCfg, uint32_t *const pData);
int ad5940_MeasureDUT(struct ad5940_dev *dev, HSRTIACal_Type *pCalCfg, AppBiaCfg_Type *AppBiaCfg,
        ImpedanceDataPoint *res);
ExcitConfig compute_excit_config(float desired_vpp);

#endif /* MEASURE_H_ */
