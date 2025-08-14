
#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#include "ad5940.h"
#include "bia_measurement.h"

/**
 * ADC PGA calibration type
*/
typedef struct
{
  float SysClkFreq;           /**< The real frequency of system clock */
  float AdcClkFreq;           /**< The real frequency of ADC clock */
  float VRef1p82;             /**< The real voltage of 1.82 reference. Unit is volt. */
  float VRef1p11;             /**< The real voltage of 1.1 reference. Unit is volt. */
  uint32_t ADCSinc3Osr;       /**< SINC3OSR_5, SINC3OSR_4 or SINC3OSR_2 */
  uint32_t ADCSinc2Osr;       /**< SINC3OSR_5, SINC3OSR_4 or SINC3OSR_2 */
  uint32_t ADCPga;            /**< Which PGA gain we are going to calibrate? */
  uint32_t PGACalType;        /**< Calibrate gain of offset or gain+offset? */
  int32_t TimeOut10us;        /**< Timeout in 10us. -1 means no time-out*/
}ADCPGACal_Type;

/**
 * HSDAC calibration structure.
*/
typedef struct
{
  float fRcal;                /**< Rcal resistor value in Ohm*/
  float SysClkFreq;           /**< The real frequency of system clock */  
  float AdcClkFreq;           /**< The real frequency of ADC clock */ 

  uint32_t AfePwrMode;        /**< Calibrate DAC in High power mode */
  uint32_t ExcitBufGain;      /**< Select from  EXCITBUFGAIN_2, EXCITBUFGAIN_0P25 */     
  uint32_t HsDacGain;         /**< Select from  HSDACGAIN_1, HSDACGAIN_0P2 */

  uint32_t ADCSinc3Osr;       /**< SINC3OSR_5, SINC3OSR_4 or SINC3OSR_2 */
  uint32_t ADCSinc2Osr;       /**< SINC3OSR_5, SINC3OSR_4 or SINC3OSR_2 */ 
}HSDACCal_Type;

/**
 * @defgroup AD5940ERR_Const
 * @brief AD5940 error code used by library and example codes.
 * @{
*/
#define AD5940ERR_OK               0  /**< No error */
#define AD5940ERR_ERROR           -1  /**< General error message */
#define AD5940ERR_PARA            -2  /**< Parameter is illegal */
#define AD5940ERR_NULLP           -3  /**< Null pointer */
#define AD5940ERR_BUFF            -4  /**< Buffer limited. */
#define AD5940ERR_ADDROR          -5  /**< Out of Range. Register address is out of range. */
#define AD5940ERR_SEQGEN          -6  /**< Sequence generator error */
#define AD5940ERR_SEQREG          -7  /**< Register info is not found */
#define AD5940ERR_SEQLEN          -8  /**< Sequence length is too long. */
#define AD5940ERR_WAKEUP          -9  /**< Unable to wakeup AFE in specified time */
#define AD5940ERR_TIMEOUT         -10 /**< Time out error. */
#define AD5940ERR_CALOR           -11 /**< calibration out of range. */
#define AD5940ERR_APPERROR        -100  /**< Used in example code to indicated the application has not been initialized. */
/** @} */

int ad5940_HSRtiaCal(struct ad5940_dev *dev, HSRTIACal_Type *pCalCfg,
             void *pResult);
int ad5940_ADCPGACal(struct ad5940_dev *dev, ADCPGACal_Type *pADCPGACal);
int ad5940_HSDACCal(struct ad5940_dev *dev, HSDACCal_Type *pCalCfg);

#endif /* CALIBRATE_H_ */
