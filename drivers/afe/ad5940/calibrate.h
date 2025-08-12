
#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#include "bia_measurement.h"

int AD5940_RcalCalibration(struct ad5940_dev *dev, 
                          CalCfg_Type *pCalCfg);

#endif /* CALIBRATE_H_ */
