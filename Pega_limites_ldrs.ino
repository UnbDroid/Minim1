#include "MPU9250.h"
#include <NewPing.h>

#define MD 5
#define MDINA 34
#define MDINB 36
#define MDEN 38
#define ENCODER_D 2

#define ME 4
#define MEINA 42
#define MEINB 44
#define MEEN 40
#define ENCODER_E 3

#define LED_dir 48
#define LED_esq 46
#define LED_dir_T 52
#define LED_esq_T 50
#define LDR_dir A5
#define LDR_esq A6
#define LDR_dir_T A3
#define LDR_esq_T A4

#define POT_MIN_MOTOR 40
#define POT_MAX_MOTOR 45
#define POT_MED_MOTOR_ESQ 45
#define POT_MED_MOTOR_DIR 48

#define BRANCO 0
#define PRETO 6
#define min_esq 112
#define max_esq 170
#define min_dir 104
#define max_dir 150
#define MED_ESQ (min_esq+max_esq)/2
#define MED_DIR (min_dir+max_dir)/2

#define TRIGGER_PIN_Front  45
#define TRIGGER_PIN_Dir  53
#define TRIGGER_PIN_Esq  49
#define US_FRONT     43
#define US_ESQ      47
#define US_DIR      51
#define MAX_DISTANCE 200
#define TIME_STEP 20
#define Turn_Tension 3
#define DIST_OBJ 15
#define DIST_OBJ_LATERAL 20

#define S0 33
#define S1 35
#define S2 37
#define S3 39
#define Out 41

#define BUTTON 23

#define kp 0.00005
#define ki 0
#define kd 0

#define direita 0
#define esquerda 1
#define frente 2
#define tras 3
#define ultra_f 4
#define ultra_l 5

NewPing usFront(TRIGGER_PIN_Front, US_FRONT, MAX_DISTANCE);
NewPing usDir(TRIGGER_PIN_Dir, US_DIR, MAX_DISTANCE);
NewPing usEsq(TRIGGER_PIN_Esq, US_ESQ, MAX_DISTANCE);

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

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 gyro(Wire,0x68);
int status;
float degreeZ = 0;

void setup(){

  Serial.begin(9600);
  pinMode(LDR_esq, INPUT);
  pinMode(LDR_dir, INPUT);
  pinMode(LED_dir, OUTPUT);
  pinMode(LED_esq, OUTPUT);

  digitalWrite(LED_esq, HIGH);
  digitalWrite(LED_dir, HIGH);
  delay(500);
}

int max_dir = 0;
int max_esq = 0;
int min_esq = 1110;
int min_dir = 1110;

void loop(){
  int ldr_esq, ldr_dir;
  ldr_esq = analogRead(LDR_esq);
  ldr_dir = analogRead(LDR_dir);

  if (ldr_esq > max_esq){
    max_esq = ldr_esq;
    Serial.print("Novo maximo esquerdo: ");
    Serial.println(max_esq);
  }else if (ldr_esq < min_esq){
    min_esq = ldr_esq;
    Serial.print("Novo minim1 esquerdo: ");
    Serial.println(min_esq);
  }

  if (ldr_dir > max_dir){
    max_dir = ldr_dir;
    Serial.print("Novo maximo direito: ");
    Serial.println(max_dir);
  }else if (ldr_dir < min_dir){
    min_dir = ldr_dir;
    Serial.print("Novo minim1 direito: ");
    Serial.println(min_dir);
  }
  delay(500);

}
