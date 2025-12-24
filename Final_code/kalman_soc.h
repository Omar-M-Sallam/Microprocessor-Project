#ifndef KALMAN_SOC_H
#define KALMAN_SOC_H

class KalmanSOC {
public:
    // State
    float soc;      // Estimated State of Charge
    float P;        // Estimation Uncertainty

    // Tunable Noise Parameters
    float Q;        // Process noise (uncertainty in model)
    float R;        // Measurement noise (if voltage sensor added later)

    // Battery parameters
    float capacityAh;   // Battery capacity in Ah

    KalmanSOC(float initSOC, float initP, float Q_process, float R_meas, float batteryCapacity);

    void predict(float currentA, float dt);
    void correct(float measuredSOC);   // Only if voltage sensor added
};

#endif
