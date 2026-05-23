#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

struct KalmanFilter {
    float Q_angle;
    float Q_bias;
    float R;

    float angle;
    float bias;
    float rate;

    float P[2][2];

    bool initialized;
};

void kalmanInit(KalmanFilter *k);
void kalmanReset(KalmanFilter *k, float angle);
float kalmanUpdate(KalmanFilter *k, float newAngle, float newRate, float dt);

#endif
