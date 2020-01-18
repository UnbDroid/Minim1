#include "MPU9250.h"
#include <NewPing.h>

#define MD 4
#define MDINA 30
#define MDINB 31
#define MDEN 22
#define ENCODER_D 2

#define ME 5
#define MEINA 46
#define MEINB 47
#define MEEN 52
#define ENCODER_E 3

#define LED_dir 50
#define LED_esq 51
#define LDR_dir A2
#define LDR_esq A6

#define POT_MIN_MOTOR 45
#define POT_MAX_MOTOR 50
#define POT_MED_MOTOR_ESQ 45
#define POT_MED_MOTOR_DIR 48

#define BRANCO 0
#define PRETO 6
#define min_esq 117
#define max_esq 210
#define min_dir 100
#define max_dir 200
#define MED_ESQ (min_esq+max_esq)/2
#define MED_DIR (min_dir+max_dir)/2

#define TRIGGER_PIN  34
#define US_FRONT     37
#define US_SIDE      36
#define MAX_DISTANCE 200
#define TIME_STEP 20
#define Turn_Tension 3
#define DIST_OBJ 15
#define DIST_OBJ_LATERAL 20

#define BUTTON 24

#define kp 0.00005
#define ki 0
#define kd 0

#define direita 0
#define esquerda 1
#define frente 2
#define tras 3
#define ultra_f 4
#define ultra_l 5

NewPing usFront(TRIGGER_PIN, US_FRONT, MAX_DISTANCE);
NewPing usSide(TRIGGER_PIN, US_SIDE, MAX_DISTANCE);

// PID Motores
int p = 0;
int i = 0;
int d = 0;
int erro_atual = 0;
int erro_total = 0;
int erro_ant = 0;
int pot_motor_esq = 45;
int pot_motor_dir = 55;
int encoder_d = 0;
int encoder_e = 0;

// Leitura LDR
int leitura_max = 0;
int leitura_min = 1000;
int contador = 0;
int iteracoes = 5;
int vetor_leituras_esquerda[5];
int vetor_leituras_direita[5];
int vetor_leituras_us_front[5];
int vetor_leituras_us_lat[5];
int validacao = 0;
int leitura_dir = 0;
int leitura_esq = 0;
int leitura_front = 0;
int leitura_lat = 0;

    int media_vetor(int direcao){
  int valida = 1;
  int i=0, pos_min, pos_max;
  int minimo = 1000;
  int maximo = 0;
  int total = 0;

  if(direcao == 0){
    for(i=0; i<iteracoes; i++){
      if(vetor_leituras_direita[i] < minimo){
        minimo = vetor_leituras_direita[i];
        pos_max = i;
      }
      if(vetor_leituras_direita[i] > maximo){
        maximo = vetor_leituras_direita[i];
        pos_min = i;
      }
    }

    if(pos_min == pos_max){
      pos_max++;
    }

    vetor_leituras_direita[pos_min] = 0;
    vetor_leituras_direita[pos_max] = 0;

    for(i=0; i<iteracoes; i++){
      total += vetor_leituras_direita[i];
    }
  }else if(direcao == 1){
    for(i=0; i<iteracoes; i++){
      if(vetor_leituras_esquerda[i] < minimo){
        minimo = vetor_leituras_esquerda[i];
        pos_max = i;
      }
      if(vetor_leituras_esquerda[i] > maximo){
        maximo = vetor_leituras_esquerda[i];
        pos_min = i;
      }
    }

    if(pos_min == pos_max){
      pos_max++;
    }

    vetor_leituras_esquerda[pos_min] = 0;
    vetor_leituras_esquerda[pos_max] = 0;

    for(i=0; i<iteracoes; i++){
      total += vetor_leituras_esquerda[i];
    }
  }else if(direcao == 4){
    for(i=0; i<iteracoes; i++){
      if(vetor_leituras_us_front[i] < minimo){
        minimo = vetor_leituras_us_front[i];
        pos_max = i;
      }
      if(vetor_leituras_us_front[i] > maximo){
        maximo = vetor_leituras_us_front[i];
        pos_min = i;
      }
    }

    if(pos_min == pos_max){
      pos_max++;
    }

    vetor_leituras_us_front[pos_min] = 0;
    vetor_leituras_us_front[pos_max] = 0;

    for(i=0; i<iteracoes; i++){
      if(vetor_leituras_us_front[i] != 0){
        valida = 0;
      }
    }

    if(valida == 1){
      vetor_leituras_us_front[2] = 300;
      vetor_leituras_us_front[3] = 300;
      vetor_leituras_us_front[4] = 300;
    }

    for(i=0; i<iteracoes; i++){
      total += vetor_leituras_us_front[i];
    }
  }else if(direcao == 5){
    for(i=0; i<iteracoes; i++){
      if(vetor_leituras_us_lat[i] < minimo){
        minimo = vetor_leituras_us_lat[i];
        pos_max = i;
      }
      if(vetor_leituras_us_lat[i] > maximo){
        maximo = vetor_leituras_us_lat[i];
        pos_min = i;
      }
    }

    if(pos_min == pos_max){
      pos_max++;
    }

    vetor_leituras_us_lat[pos_min] = 0;
    vetor_leituras_us_lat[pos_max] = 0;

    for(i=0; i<iteracoes; i++){
      if(vetor_leituras_us_lat[i] != 0){
        valida = 0;
      }
    }

    if(valida == 1){
      vetor_leituras_us_lat[2] = 300;
      vetor_leituras_us_lat[3] = 300;
      vetor_leituras_us_lat[4] = 300;
    }

    for(i=0; i<iteracoes; i++){
      total += vetor_leituras_us_lat[i];
    }
  }

  return total/(iteracoes-2);
}

    void le_ultra(){
  int f, l;

  while(contador < 5){
    f = usFront.ping_cm();
    l = usSide.ping_cm();
    vetor_leituras_us_lat[contador] = l;
    vetor_leituras_us_front[contador] = f;
    contador++;
  }
  contador = 0;

}

    void setup() {
      Serial.begin(9600);
    }

    void loop() {
      delay(100);
      le_ultra();
      leitura_front = media_vetor(4);
      leitura_lat = media_vetor(5);
      Serial.print("Frente: ");
      Serial.print(leitura_front);
      Serial.println("cm");
      Serial.print("Lado: ");
      Serial.print(leitura_lat);
      Serial.println("cm");
    }
