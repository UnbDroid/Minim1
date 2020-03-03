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

// Código pra atribuir os valores das frquencias maximas e minimas de verde, vermelho e azul
// Vermelho: 217/31
// Verde: 240/23
// Azul: 255/48

int freq = 0;
int green_freqMax = 0;
int red_freqMax = 0;
int blue_freqMax = 0;

int green_freqMin = 10000;
int red_freqMin = 10000;
int blue_freqMin = 10000;
void setup() {

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(Out, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  // Setting frequency scaling to 20%

  Serial.begin(9600);
  delay(300);
}

void loop() {
    // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);

  freq = pulseIn(Out, LOW);
  if (freq > red_freqMax){
      red_freqMax = freq;
      Serial.print("Novo maior vermelho: ");
      Serial.println(red_freqMax);
  }else if (freq < red_freqMin){
      red_freqMin = freq;
      Serial.print("Novo menor vermelho: ");
      Serial.println(red_freqMin);
  }

   // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);

  freq = pulseIn(Out, LOW);
  if (freq > green_freqMax){
      green_freqMax = freq;
      Serial.print("Novo maior verde: ");
      Serial.println(green_freqMax);
  }else if (freq < green_freqMin){
      green_freqMin = freq;
      Serial.print("Novo menor verde: ");
      Serial.println(green_freqMin);
  }

  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);

  freq = pulseIn(Out, LOW);
  if (freq > blue_freqMax){
      blue_freqMax = freq;
      Serial.print("Novo maior azul: ");
      Serial.println(blue_freqMax);
  }else if (freq < blue_freqMin){
      blue_freqMin = freq;
      Serial.print("Novo menor azul: ");
      Serial.println(blue_freqMin);
  }

}
