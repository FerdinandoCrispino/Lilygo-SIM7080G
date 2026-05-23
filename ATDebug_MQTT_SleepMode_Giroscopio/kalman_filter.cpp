#include "kalman_filter.h"

void kalmanInit(KalmanFilter *k) {
    k->Q_angle = 0.001f;
    k->Q_bias  = 0.003f;
    k->R       = 0.03f;

    k->angle = 0.0f;
    k->bias  = 0.0f;
    k->rate  = 0.0f;

    k->P[0][0] = 0; k->P[0][1] = 0;
    k->P[1][0] = 0; k->P[1][1] = 0;

    k->initialized = false;
}

void kalmanReset(KalmanFilter *k, float angle) {
    k->angle = angle;
    k->bias = 0;

    k->P[0][0] = 0; k->P[0][1] = 0;
    k->P[1][0] = 0; k->P[1][1] = 0;

    k->initialized = true;
}

float kalmanUpdate(KalmanFilter *k, float newAngle, float newRate, float dt) {

    if (dt <= 0 || dt > 0.1f) return k->angle;

    if (!k->initialized) {
        kalmanReset(k, newAngle);
        return newAngle;
    }

    // Predição
    k->rate = newRate - k->bias;
    k->angle += dt * k->rate;

    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    // Correção
    float S = k->P[0][0] + k->R;

    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    float y = newAngle - k->angle;

    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00 = k->P[0][0];
    float P01 = k->P[0][1];

    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    return k->angle;
}
