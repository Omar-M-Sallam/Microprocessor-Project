#include "kalman_soc.h"

KalmanSOC::KalmanSOC(float initSOC, float initP, float Q_process, float R_meas, float batteryCapacity) 
{
    soc = initSOC;
    P   = initP;

    Q = Q_process;
    R = R_meas;

    capacityAh = batteryCapacity;
}

// -------------------------
// PREDICTION STEP ONLY
// -------------------------
void KalmanSOC::predict(float currentA, float dt)
{
    // Convert dt from milliseconds → hours
    float dt_hours = dt / 3600000.0f;

    float deltaSOC = -(currentA * dt_hours) / capacityAh;

    soc += deltaSOC;

    // Limit SOC between 0 and 1
    if (soc > 1.0f) soc = 1.0f;
    if (soc < 0.0f) soc = 0.0f;

    // Increase uncertainty
    P += Q;
}

// --------------------------------------
// CORRECTION STEP (OPTIONAL - if you add voltage sensor)
// --------------------------------------
void KalmanSOC::correct(float measuredSOC)
{
    float K = P / (P + R);        // Kalman Gain
    soc = soc + K * (measuredSOC - soc);
    P = (1 - K) * P;
}
