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

#define POT_MIN_MOTOR 40
#define POT_MAX_MOTOR 60
#define POT_MED_MOTOR_ESQ 40
#define POT_MED_MOTOR_DIR 43

#define BRANCO 0
#define PRETO 6
#define min_esq 122
#define max_esq 240
#define min_dir 128
#define max_dir 250
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

void acresce_encoder_d(){
  encoder_d++;
}

void acresce_encoder_e(){
  encoder_e++;
}

void movimento(int direcao){
  switch(direcao){
    case direita:
      digitalWrite(MEINA, LOW);
      digitalWrite(MEINB, HIGH);
      digitalWrite(MDINA, LOW);
      digitalWrite(MDINB, HIGH);
    break;
    case esquerda:
      digitalWrite(MEINA, HIGH);
      digitalWrite(MEINB, LOW);
      digitalWrite(MDINA, HIGH);
      digitalWrite(MDINB, LOW);
    break;
    case frente:
      digitalWrite(MDINA, HIGH);
      digitalWrite(MDINB, LOW);
      digitalWrite(MEINA, LOW);
      digitalWrite(MEINB, HIGH);
    break;
    case tras:
      digitalWrite(MDINA, LOW);
      digitalWrite(MDINB, HIGH);
      digitalWrite(MEINA, HIGH);
      digitalWrite(MEINB, LOW);
    break;
  }
}

void trava_motores(int trava){
  if (trava){
    digitalWrite(MEINA, HIGH);
    digitalWrite(MEINB, HIGH);

    digitalWrite(MDINA, HIGH);
    digitalWrite(MDINB, HIGH);
  }else{
    digitalWrite(MEINA, LOW);
    digitalWrite(MEINB, LOW);

    digitalWrite(MDINA, LOW);
    digitalWrite(MDINB, LOW);
  }
}

void setup(){
  pinMode(MD , INPUT);
  pinMode(MDINA, INPUT);
  pinMode(MDINB, INPUT);
  pinMode(MDEN , INPUT);

  pinMode(ME , INPUT);
  pinMode(MEINA, INPUT);
  pinMode(MEINB, INPUT);
  pinMode(MEEN , INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_E), acresce_encoder_e, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D), acresce_encoder_d, CHANGE);

  delay(500);
  Serial.begin(9600);
}

void loop(){
  Serial.print("Encoder esq: ");
  Serial.print(encoder_e);
  Serial.print(" || Encoder dir: ");
  Serial.println(encoder_d);

  delay(1500);
  Serial.println("Os dois indo pra frente!");
  movimento(frente);
  analogWrite(ME, 55);
  analogWrite(MD, 55);
  delay(2000);
  trava_motores(1);
  delay(200);
  Serial.print("Encoder esq: ");
  Serial.print(encoder_e);
  Serial.print(" || Encoder dir: ");
  Serial.println(encoder_d);

  delay(500);
  Serial.println("Esquerda indo pra frente e direita indo pra tras!");
  movimento(direita);
  analogWrite(ME, 55);
  analogWrite(MD, 55);
  delay(2000);
  trava_motores(1);
  delay(200);
  Serial.print("Encoder esq: ");
  Serial.print(encoder_e);
  Serial.print(" || Encoder dir: ");
  Serial.println(encoder_d);
  delay(500);

  Serial.println("Direita indo pra frente e esquerda indo pra tras!");
  movimento(esquerda);
  analogWrite(ME, 55);
  analogWrite(MD, 55);
  delay(2000);
  trava_motores(1);
  delay(200);
  Serial.print("Encoder esq: ");
  Serial.print(encoder_e);
  Serial.print(" || Encoder dir: ");
  Serial.println(encoder_d);
  delay(500);

  Serial.println("Os dois indo pra tras!");
  movimento(tras);
  analogWrite(ME, 55);
  analogWrite(MD, 55);
  delay(2000);
  trava_motores(1);
  delay(200);
  Serial.print("Encoder esq: ");
  Serial.print(encoder_e);
  Serial.print(" || Encoder dir: ");
  Serial.println(encoder_d);
  delay(1000);
  encoder_d = 0;
  encoder_e = 0;
  Serial.println("Encoders zerados");
  delay(500);
}
