
#ifndef MEASURE_H_
#define MEASURE_H_

#include "ad5940.h"
#include "calibrate.h"
#include "bia_measurement.h"

fImpCar_Type computeImpedanceFromFifo(AppBiaCfg_Type *pBiaCfg, uint32_t *const pData);
int ad5940_MeasureDUT(struct ad5940_dev *dev, HSRTIACal_Type *pCalCfg, AppBiaCfg_Type *AppBiaCfg,
        ImpedanceDataPoint *res);

#endif /* MEASURE_H_ */
