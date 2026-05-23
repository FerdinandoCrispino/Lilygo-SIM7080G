#include "Arduino.h"
#include "my_sensor_dist.h"
// =======================  Sensor de distancia ==============================================


void kalmanUpdateDist(struct kalmanDadosDist *paramDist) {

  paramDist->x_pred = paramDist->x_est;

  paramDist->P_pred = paramDist->P + paramDist->Q;
  paramDist->k = paramDist->P_pred / (paramDist->P_pred + paramDist->R);                       // calcula o ganho de Kalman

  paramDist->x_est = paramDist->x_pred + paramDist->k * (paramDist->input - paramDist->x_pred);    // atualiza a estimativa com a medicao
  paramDist->P = (1 - paramDist->k) * paramDist->P_pred;                                   // atualiza o erro de estimativa
}

struct kalmanDadosDist kalmanDist;

long getValue(String dataSensor, String separator) {
    String leitura = "-2";
    int textIndex_ini = dataSensor.indexOf(separator) + separator.length() + 1;
    if (textIndex_ini != -1) {
        String contentType = dataSensor.substring(textIndex_ini);
        //Serial.println(contentType);
        int textIndex_next = contentType.indexOf("&") ;
        if (textIndex_next != -1) {
            leitura = contentType.substring(0, textIndex_next-1);
        }
    }
    return leitura.toInt();
}

float filtroMediana(float leituras[5]) {
  const int JANELA = 5; // Numero de amostras (deve ser Impar)


  // Cria uma copia para ordenar
  float ordenado[JANELA];
  for (int i = 0; i < JANELA; i++) ordenado[i] = leituras[i];

  // Ordenaçao simples (Bubble Sort)
  for (int i = 0; i < JANELA - 1; i++) {
    for (int j = i + 1; j < JANELA; j++) {
      if (ordenado[i] > ordenado[j]) {
        float temp = ordenado[i];
        ordenado[i] = ordenado[j];
        ordenado[j] = temp;
      }
    }
  }

  // Retorna o valor do meio
  return ordenado[JANELA / 2];
}

//void read_sensor_dist(kalmanDadosDist *kalDist ){
