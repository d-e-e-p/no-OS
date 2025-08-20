
#ifndef IMPEDANCE2LCR_H_
#define IMPEDANCE2LCR_H_

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ad5940.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief A single impedance measurement data point.
 */
typedef struct
{
    float freq;
    fImpCar_Type Z;
} ImpedanceDataPoint;


/**
 * @brief LCR fitting results.
 */
typedef struct
{
    double L;      // Inductance in Henrys
    double C;      // Capacitance in Farads
    double R;      // Resistance in Ohms
    double fit_error; // Final root mean square error of the fit
} LCR_Result;


/**
 * @brief Calculates L, C, and R from impedance measurements at different frequencies.
 *
 * The function fits the impedance data to a model of a parallel combination
 * of a capacitor (C) and a series resistor-inductor (RL) branch.
 * It uses a gradient descent algorithm to find the optimal L, C, and R values.
 *
 * @param data A pointer to an array of ImpedanceDataPoint structures.
 * @param num_points The number of data points in the array.
 * @return An LCR_Result structure containing the calculated L, C, R and fit error.
 *         If fitting fails, L, C, and R will be NAN.
 */
LCR_Result lcr_from_impedance(ImpedanceDataPoint data[], int num_points);


#endif /* IMPEDANCE2LCR_H_ */
