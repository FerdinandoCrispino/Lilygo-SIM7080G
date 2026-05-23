#ifndef my_sensor_dist
#define my_sensor_dist
#include "Arduino.h"

struct kalmanDadosDist {
  float Q = 0.05;      // (maior mais rapido e mais ruidoso. variancia do processo (quanto se confia na medida)
  float R = 0.03;       //(maior mais lento. variancia da medida (quanto ruido esperamos)
  float x_est = 0.0;    // estado atual
  float P = 0.1;        // estimativa inicial de erro
  float input;
  float x_pred;
  float P_pred;
  float k;
};

void kalmanUpdateDist(struct kalmanDadosDist *paramDist);
long getValue(String dataSensor, String separator);
float filtroMediana(float leituras[5]);


#endif
