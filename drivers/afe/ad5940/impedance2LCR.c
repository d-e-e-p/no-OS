
#include "impedance2LCR.h"
#include <stdlib.h> // For qsort
#include <complex.h>

#define MAX_ITERATIONS 5000
#define LEARNING_RATE 1e-10
#define GRADIENT_STEP 1e-9

// --- Forward Declarations for Static Functions ---
static void calculate_admittance_model(double L, double C, double R, double omega, double *G_calc, double *B_calc);
static double cost_function(double L, double C, double R, const double *omegas, const fImpCar_Type *admittances, int n);
static void calculate_gradient(double L, double C, double R, const double *omegas, const fImpCar_Type *admittances, int n, double *grad_L, double *grad_C, double *grad_R);
static int compare_freq(const void *a, const void *b);

LCR_Result fit_result(LCR_Result res)
{
    LCR_Result rfit = res;
    rfit.L = res.L;
    rfit.C = res.C;
    rfit.R = res.R;
    return rfit;
} 

// zero out negative LCR
LCR_Result zero_result(LCR_Result res)
{
    LCR_Result rfit = res;

    if (rfit.L < 0)  rfit.L = 0; 
    if (rfit.C < 0)  rfit.C = 0; 
    if (rfit.R < 0)  rfit.R = 0; 

    return rfit;
}
    

/**
 * @brief Comparison function for qsort to sort data by frequency.
 */
static int compare_freq(const void *a, const void *b)
{
    ImpedanceDataPoint *p1 = (ImpedanceDataPoint *)a;
    ImpedanceDataPoint *p2 = (ImpedanceDataPoint *)b;
    if (p1->freq < p2->freq)
        return -1;
    if (p1->freq > p2->freq)
        return 1;
    return 0;
}

/**
 * @brief Calculates the modeled conductance (G) and susceptance (B).
 */
static void calculate_admittance_model(double L, double C, double R, double omega, double *G_calc, double *B_calc)
{
    double denominator = R * R + (omega * L) * (omega * L);
    if (denominator == 0)
    {
        *G_calc = 1e12; // Large number to indicate error
        *B_calc = 1e12;
        return;
    }
    *G_calc = R / denominator;
    *B_calc = omega * C - (omega * L) / denominator;
}

/**
 * @brief Calculates the cost (mean squared error) between measured and modeled admittance.
 */
static double cost_function(double L, double C, double R, const double *omegas, const fImpCar_Type *admittances, int n)
{
    double total_error = 0;
    for (int i = 0; i < n; i++)
    {
        double G_calc, B_calc;
        calculate_admittance_model(L, C, R, omegas[i], &G_calc, &B_calc);
        double error_G = G_calc - admittances[i].Real;
        double error_B = B_calc - admittances[i].Image;
        total_error += error_G * error_G + error_B * error_B;
    }
    return total_error / n;
}

/**
 * @brief Calculates the gradient of the cost function with respect to L, C, and R.
 */
static void calculate_gradient(double L, double C, double R, const double *omegas, const fImpCar_Type *admittances, int n, double *grad_L, double *grad_C, double *grad_R)
{
    double cost_center = cost_function(L, C, R, omegas, admittances, n);

    double cost_L = cost_function(L + GRADIENT_STEP, C, R, omegas, admittances, n);
    double cost_C = cost_function(L, C + GRADIENT_STEP, R, omegas, admittances, n);
    double cost_R = cost_function(L, C, R + GRADIENT_STEP, omegas, admittances, n);

    *grad_L = (cost_L - cost_center) / GRADIENT_STEP;
    *grad_C = (cost_C - cost_center) / GRADIENT_STEP;
    *grad_R = (cost_R - cost_center) / GRADIENT_STEP;
}

LCR_Result orig_lcr_from_impedance(ImpedanceDataPoint data[], int num_points)
{
    LCR_Result result = {NAN, NAN, NAN, NAN};
    if (num_points < 3)
    {
        return result; // Need at least 3 points to fit 3 parameters
    }

    // Sort data by frequency
    qsort(data, num_points, sizeof(ImpedanceDataPoint), compare_freq);

    double omegas[num_points];
    fImpCar_Type admittances[num_points];

    for (int i = 0; i < num_points; i++)
    {
        omegas[i] = 2 * M_PI * data[i].freq;
        fImpCar_Type one = {1.0, 0.0};
        admittances[i] = ad5940_ComplexDivFloat(&one, &data[i].Z);
    }

    // --- Initial Guess Calculation ---
    fImpCar_Type low_freq_z = data[0].Z;
    double low_omega = omegas[0];
    fImpCar_Type high_freq_y = admittances[num_points - 1];
    double high_omega = omegas[num_points - 1];

    double R_guess = fabs(low_freq_z.Real);
    if (R_guess < 1e-3) R_guess = 1e-3;

    double L_guess = fabs(low_freq_z.Image / low_omega);
    if (L_guess < 1e-9) L_guess = 1e-9; // 1 nH

    double C_guess = fabs(high_freq_y.Image / high_omega);
    if (C_guess < 1e-15) C_guess = 1e-15; // 1 fF

    double L = L_guess, C = C_guess, R = R_guess;

    // --- Gradient Descent Optimization ---
    for (int i = 0; i < MAX_ITERATIONS; i++)
    {
        double grad_L, grad_C, grad_R;
        calculate_gradient(L, C, R, omegas, admittances, num_points, &grad_L, &grad_C, &grad_R);

        L -= LEARNING_RATE * grad_L;
        C -= LEARNING_RATE * grad_C;
        R -= LEARNING_RATE * grad_R;

        // Ensure parameters are physically meaningful
        if (L <= 0) L = 1e-12;
        if (C <= 0) C = 1e-18;
        if (R <= 0) R = 1e-6;
    }

    result.L = L;
    result.C = C;
    result.R = R;
    result.fit_error = sqrt(cost_function(L, C, R, omegas, admittances, num_points));

    return result;
}

LCR_Result lcr_from_impedance(ImpedanceDataPoint data[], int num_points)
{
    LCR_Result result = {0};
    double sumR = 0.0;

    // For regression on imag part
    double Sww = 0, Swy = 0, S11 = 0, S1y = 0, Sw1 = 0;
    double y;  // imag(Z)
    double w;  // angular freq = 2*pi*f

    for (int i = 0; i < num_points; i++) {
        double Re = data[i].Z.Real;
        double Im = data[i].Z.Image;
        w = 2.0 * M_PI * data[i].freq;

        sumR += Re;

        // Linear regression: Im = w*A + (-1/w)*B
        y = Im;
        double X1 = w;
        double X2 = -1.0 / w;

        // Accumulate normal equations
        Sww += X1*X1;       // Σ w^2
        Sw1 += X1*X2;       // Σ (w * -1/w)
        S11 += X2*X2;       // Σ (1/w^2)
        Swy += X1*y;        // Σ w*y
        S1y += X2*y;        // Σ (1/w)*y
    }

    // Average R
    result.R = sumR / num_points;

    // Solve 2x2 linear system:
    // [Sww  Sw1] [A] = [Swy]
    // [Sw1  S11] [B]   [S1y]

    double det = Sww*S11 - Sw1*Sw1;
    if (fabs(det) < 1e-12) {
        result.L = result.C = NAN;
        return result;
    }

    double A = (Swy*S11 - S1y*Sw1) / det; // L
    double B = (Sww*S1y - Swy*Sw1) / det; // 1/C

    result.L = A;
    result.C = (B != 0.0) ? 1.0/B : NAN;

    // Compute RMS fit error
    double err2 = 0.0;
    for (int i = 0; i < num_points; i++) {
        w = 2.0 * M_PI * data[i].freq;
        double predIm = w*result.L - 1.0/(w*result.C);
        double diff = data[i].Z.Image - predIm;
        err2 += diff*diff;
    }
    result.fit_error = sqrt(err2 / num_points);

    return result;
}


void dump_ztia_csv(size_t switchSeqCnt,
                       size_t num_volt,
                       size_t num_freq,
                       const float desired_vpp[num_volt],
                       const float freq_list[num_freq],
                       const fImpCar_Type Ztia[num_volt][num_freq],
                       fImpCar_Type ZtiaAve[num_volt])
{
    printf("════════════   Ztia ═════════════════════\r\n");
    printf("desired_vpp, freq, Ztia.R, Ztia.I\r\n");
    for (size_t i = 0; i < num_volt; i++) {
        ZtiaAve[i].Real = 0.0f;
        ZtiaAve[i].Image = 0.0f;
        for (size_t j = 0; j < num_freq; j++) {
            ZtiaAve[i].Real += Ztia[i][j].Real;
            ZtiaAve[i].Image += Ztia[i][j].Image;
            printf("%.0f, %.0f, %0.f, %.0f\r\n",
                    desired_vpp[i], freq_list[j], Ztia[i][j].Real, Ztia[i][j].Image);
        }
        ZtiaAve[i].Real = ZtiaAve[i].Real / (float) num_freq;
        ZtiaAve[i].Image = ZtiaAve[i].Image / (float) num_freq;
    }
}

void dump_zdut_csv(size_t switchSeqCnt,
                       size_t num_volt,
                       size_t num_freq,
                       const float desired_vpp[num_volt],
                       const float freq_list[num_freq],
                       const ImpedanceDataPoint resZ[switchSeqCnt][num_volt][num_freq])
{
    printf("════════════   Zdut   ═══════════════════\r\n");
    printf("seq, desired_vpp, freq, Zdut.R, Zdut.I\r\n");

    for (size_t seq = 0; seq < switchSeqCnt; seq++) {
        for (size_t i = 0; i < num_volt; i++) {
            for (size_t j = 0; j < num_freq; j++) {
                float zr = resZ[seq][i][j].Z.Real;
                float zi = resZ[seq][i][j].Z.Image;
                printf("%-3d, %-4.0f, %-5.0f, %-10.2f, %-10.2f\r\n",
                    seq, desired_vpp[i], freq_list[j], zr, zi);
            }
        }
    }
}

void dump_lcr_box(size_t switchSeqCnt,
                       size_t num_volt,
                       const fImpCar_Type ZtiaAve[num_volt],
                       const float desired_vpp[num_volt],
                       LCR_Result resLCR[switchSeqCnt][num_volt])
{
    printf("╔═════════════════════════════════════════╗\r\n");
    printf("║            LCR Fitting Results          ║\r\n");
    printf("╠══╤═════════╤═════════╤══════════╤═══════╣\r\n");
    printf("║S │     L   │    C    │    R     │ dutV  ║\r\n");
    printf("║  │   (mH)  │   (µF)  │   (Ω)    │ (mV)  ║\r\n");
    printf("╠══╪═════════╪═════════╪══════════╪═══════╣\r\n");

    for (size_t seq = 0; seq < switchSeqCnt; seq++) {
      for (size_t i = 0; i < num_volt; i++) {
            LCR_Result result = resLCR[seq][i];
            LCR_Result res = zero_result(fit_result(result));

            if (!isnan(result.L)) {
                printf("║%2d│%8.2g │%8.2g │%9.3g │%7.0f║\r\n",
                       seq, res.L * 1e3, res.C * 1e6, res.R , desired_vpp[i]);
            } else {
                printf("║%2d│ %7.2g │ %7.2g │%9.3g │%7.0f║\r\n",
                       seq, result.L, result.C, result.R, desired_vpp[i]);
            }

            if (i == num_volt - 1) {
                printf("╟──┼─────────┼─────────┼──────────┼───────╢\r\n");
            }
        }
    }

    printf("╚══╧═════════╧═════════╧══════════╧═══════╝\r\n");
}

void dump_raw_lcr_csv(size_t switchSeqCnt,
                  size_t num_volt,
                  const fImpCar_Type ZtiaAve[num_volt],
                  const float desired_vpp[num_volt],
                  LCR_Result resLCR[switchSeqCnt][num_volt])
{
    // Header row
    printf("╔═════════════════════════════════════════╗\r\n");
    printf("║              CSV Raw                    ║\r\n");
    printf("╠══╤═════════╤═════════╤═══════════╤══════╣\r\n");
    printf("Seq,Vpp (mV), Ztia(Ohm),L(mH), C(pF),R(Ohm),FitError\r\n");

    for (size_t seq = 0; seq < switchSeqCnt; seq++) {
      for (size_t i = 0; i < num_volt; i++) {

            LCR_Result res = resLCR[seq][i];
            printf("%d,%.0f,%.0f,%.2g,%.2g,%.2f,%.2f\r\n",
                   seq,
                   desired_vpp[i],
                   ZtiaAve[i].Real,  // Ohm
                   res.L * 1e3,   // mH
                   res.C * 1e12,  // pF
                   res.R,         // Ohm
                   res.fit_error * 1e3);
        }
    }
}

void dump_fit_lcr_csv(size_t switchSeqCnt,
                  size_t num_volt,
                  const fImpCar_Type ZtiaAve[num_volt],
                  const float desired_vpp[num_volt],
                  LCR_Result resLCR[switchSeqCnt][num_volt])
{
    // Header row
    printf("╔═════════════════════════════════════════╗\r\n");
    printf("║               CSV Fit                   ║\r\n");
    printf("╠══╤═════════╤═════════╤═══════════╤══════╣\r\n");
    printf("Seq,Vpp (mV), Ztia(Ohm),L(mH), C(pF),R(Ohm),FitError\r\n");

    for (size_t seq = 0; seq < switchSeqCnt; seq++) {
      for (size_t i = 0; i < num_volt; i++) {
            LCR_Result result = resLCR[seq][i];
            LCR_Result res = fit_result(result);

            printf("%d,%.0f,%.0f,%.2g,%.2g,%.2f,%.2f\r\n",
                   seq,
                   desired_vpp[i],
                   ZtiaAve[i].Real,  // Ohm
                   res.L * 1e3,   // mH
                   res.C * 1e12,  // pF
                   res.R, // ohm
                   result.fit_error * 1e3);
        }
    }
}
// Define a main routine for testing
int test()
{
    printf("Running LCR Extraction Test...\n");

    // Test data based on a known LCR circuit
    // L=1mH, C=1uF, R=10 Ohm
    // Z = 1 / (1/(R + jwL) + jwC)
    ImpedanceDataPoint test_data[] = {
        {1000, {10.0003, 6.279}},
        {2000, {10.002, 12.54}},
        {3000, {10.009, 18.75}},
        {4000, {10.02, 24.88}},
        {5000, {10.04, 30.88}},
        {6000, {10.07, 36.69}},
        {7000, {10.11, 42.24}},
        {8000, {10.16, 47.45}},
        {9000, {10.23, 52.25}},
        {10000, {10.3, 56.55}}
    };
    int num_points = sizeof(test_data) / sizeof(test_data[0]);

    LCR_Result result = lcr_from_impedance(test_data, num_points);

    printf("\n--- LCR Fitting Results ---\n");
    if (!isnan(result.L))
    {
        printf("L = %e H\n", result.L);
        printf("C = %e F\n", result.C);
        printf("R = %e Ohm\n", result.R);
        printf("Fit Error (RMSE) = %e\n", result.fit_error);
    }
    else
    {
        printf("Fitting failed.\n");
    }
    printf("---------------------------\n");

    // Expected values for this test case are approx:
    // L = 1e-3 H, C = 1e-6 F, R = 10 Ohm
    // Note: The simple gradient descent may not be perfectly accurate.
    // The learning rate and number of iterations may need tuning for different component ranges.

    return 0;
}

//
// Compute model impedance
//
static inline fImpCar_Type Z_model(double f, double R, double L, double Cs, double Cp) {
    double w = 2*M_PI*f;

    // Series branch: R + jωL + 1/(jωCs)
    double complex Zs = R + I*w*L + 1.0/(I*w*Cs);

    // Cp branch: 1/(jωCp)
    double complex Zcp = 1.0/(I*w*Cp);

    // Parallel combination
    double complex Zt = (Zs * Zcp) / (Zs + Zcp);

    fImpCar_Type res = {creal(Zt), cimag(Zt)};
    return res;
}

//
// Compute RMS error for given params
//
static double error_function(ImpedanceDataPoint *data, int N,
                             double R, double L, double Cs, double Cp) {
    double err = 0.0;
    for (int i = 0; i < N; i++) {
        fImpCar_Type Zm = Z_model(data[i].freq, R, L, Cs, Cp);
        double dr = Zm.Real - data[i].Z.Real;
        double di = Zm.Imag - data[i].Z.Imag;
        err += dr*dr + di*di;
    }
    return sqrt(err/N);
}

//
// Gradient-based optimization (very simple LM-like update)
//
LCR_Result fit_rlc_cp(ImpedanceDataPoint *data, int N) {
    // Initial guesses
    double R = 300.0, L = 1e-6, Cs = 1e-9, Cp = 1e-9;
    double lr = 0.1; // learning rate
    double eps = 1e-6;

    for (int iter=0; iter<500; iter++) {
        double base_err = error_function(data,N,R,L,Cs,Cp);

        // Numerical gradients
        double dR  = (error_function(data,N,R+eps,L,Cs,Cp) - base_err)/eps;
        double dL  = (error_function(data,N,R,L+eps,Cs,Cp) - base_err)/eps;
        double dCs = (error_function(data,N,R,L,Cs+eps,Cp) - base_err)/eps;
        double dCp = (error_function(data,N,R,L,Cs,Cp+eps) - base_err)/eps;

        // Gradient descent update
        R  -= lr*dR;
        L  -= lr*dL;
        Cs -= lr*dCs;
        Cp -= lr*dCp;

        if (iter % 50 == 0) {
            printf("Iter %d: Err=%.6g R=%.3f L=%.3e Cs=%.3e Cp=%.3e\n",
                   iter, base_err, R, L, Cs, Cp);
        }

        if (base_err < 1e-3) break; // converged
    }

    LCR_Result best;
    best.R = R; best.L = L; best.Cs = Cs; best.Cp = Cp;
    best.fit_error = error_function(data,N,R,L,Cs,Cp);
    return best;
}

