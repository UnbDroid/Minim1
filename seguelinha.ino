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

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 gyro(Wire,0x68);
int status;
float degreeZ = 0;

int media_vetor(int direcao){
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
  }else{
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
  }

  return total/(iteracoes-2);
}

void le_ldr(){

  vetor_leituras_esquerda[contador] = analogRead(LDR_esq);
  vetor_leituras_direita[contador] = analogRead(LDR_dir);

  contador++;

  while(contador<5){
    vetor_leituras_esquerda[contador] = analogRead(LDR_esq);
    vetor_leituras_direita[contador] = analogRead(LDR_dir);
    contador++;
  }


  if(contador == 5){
    contador = 0;
  }
}

void atualiza_ldr(){
    le_ldr();

  leitura_esq = media_vetor(1);
  leitura_dir = media_vetor(0);

  // Media do esquerdo: 147
  // Media do direito: 180

  if(leitura_esq > MED_ESQ){
    leitura_esq = PRETO;
  }else{
    leitura_esq = BRANCO;
  }

  if(leitura_dir > MED_DIR){
    leitura_dir = PRETO;
  }else{
    leitura_dir = BRANCO;
  }
}

void alinhar(){
    digitalWrite(MEINA, HIGH);
    digitalWrite(MEINB, LOW);
    digitalWrite(MDINA, HIGH);
    digitalWrite(MDINB, LOW);
    atualiza_ldr();
    while(leitura_esq == PRETO)
    {
        analogWrite(MD, 0);
        analogWrite(ME, POT_MIN_MOTOR);
        atualiza_ldr();
    }
    digitalWrite(MDINA, LOW);
    digitalWrite(MDINB, HIGH);
    digitalWrite(MEINA, LOW);
    digitalWrite(MEINB, HIGH);
    while(leitura_dir == PRETO)
    {
        analogWrite(MD, POT_MIN_MOTOR);
        analogWrite(ME, 0);
        atualiza_ldr();
    }
    analogWrite(MD, 0);
    analogWrite(ME, 0);

}

void UpdateGyro() {
  gyro.readSensor();
  degreeZ += (gyro.getGyroZ_rads() * TIME_STEP * 180) / (1000 * PI);
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

// Gyro setup function
void StartGyro() {
  status = gyro.begin();
  if (status < 0)
    Serial.println("IMU initialization unsuccessful");
}

void segue_linha(){

  le_ldr();

  leitura_esq = media_vetor(1);
  leitura_dir = media_vetor(0);

  if(leitura_esq > MED_ESQ){
    leitura_esq = PRETO;
  }else{
    leitura_esq = BRANCO;
  }

  if(leitura_dir > MED_DIR){
    leitura_dir = PRETO;
  }else{
    leitura_dir = BRANCO;
  }
  movimento(frente);

  while(leitura_esq == leitura_dir){
    analogWrite(MD, pot_motor_dir);
    analogWrite(ME, pot_motor_esq);
  }

}

void anda_reto(){

  analogWrite(MD, pot_motor_dir);
  analogWrite(ME, pot_motor_esq);

  erro_atual = (encoder_d - encoder_e);
  erro_total += erro_atual;

  p = erro_atual;
  i = erro_total;
  d = erro_atual - erro_ant;

  pot_motor_dir = pot_motor_dir - ((kp*p) + (ki*i) + (kd*d));
  pot_motor_esq = pot_motor_esq + ((kp*p) + (ki*i) + (kd*d));

  if(pot_motor_dir > 130){
    pot_motor_dir = 130;
  }else if(pot_motor_dir < 30){
    pot_motor_dir = 30;
  }

  if(pot_motor_esq > 130){
    pot_motor_esq = 130;
  }else if(pot_motor_esq < 30){
    pot_motor_esq = 30;
  }

  erro_ant = erro_atual;

  // Serial.print("Direita: ");
  // Serial.println(encoder_d);
  //
  // Serial.print("Esquerda: ");
  // Serial.println(encoder_e);
  //
  // Serial.print("P: ");
  // Serial.println(p);
  // Serial.print("Erro atual: ");
  // Serial.println(erro_atual);
  //
  // Serial.print("Motor direita: ");
  // Serial.println(pot_motor_dir);
  // Serial.print("Motor esquerda: ");
  // Serial.println(pot_motor_esq);
  //
  // Serial.println("----------------------------------------------------------------------------------------------------");

}

void acresce_encoder_d(){
  encoder_d++;
}

void acresce_encoder_e(){
  encoder_e++;
}

void setup() {
  delay(5000);
  pinMode(LDR_dir, INPUT);
  pinMode(LDR_esq, INPUT);
  pinMode(LED_dir, OUTPUT);
  pinMode(LED_esq, OUTPUT);

  pinMode(MD , INPUT);
  pinMode(MDINA, INPUT);
  pinMode(MDINB, INPUT);
  pinMode(MDEN , INPUT);

  pinMode(ME , INPUT);
  pinMode(MEINA, INPUT);
  pinMode(MEINB, INPUT);
  pinMode(MEEN , INPUT);

  digitalWrite(LED_dir, HIGH);
  digitalWrite(LED_esq, HIGH);


  digitalWrite(MDEN, HIGH);
  digitalWrite(MEEN, HIGH);

  digitalWrite(MDINA, HIGH);
  digitalWrite(MDINB, LOW);
  digitalWrite(MEINA, LOW);
  digitalWrite(MEINB, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCODER_E), acresce_encoder_e, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D), acresce_encoder_d, CHANGE);

  Serial.begin(9600);
  while(!Serial);
  StartGyro();
}

void conserta(){

  trava_motores(1);

  le_ldr();

  leitura_esq = media_vetor(1);
  leitura_dir = media_vetor(0);

  if(leitura_esq > MED_ESQ){
    leitura_esq = PRETO;
  }else{
    leitura_esq = BRANCO;
  }

  if(leitura_dir > MED_DIR){
    leitura_dir = PRETO;
  }else{
    leitura_dir = BRANCO;
  }

  while(leitura_esq > leitura_dir){
    movimento(esquerda);
    analogWrite(MD, 45);
    analogWrite(ME, 45);
  }

  while(leitura_dir > leitura_esq){
    movimento(direita);
    analogWrite(MD, 45);
    analogWrite(ME, 45);
  }

  trava_motores(1);
  delay(200);

}

void loop() {
  segue_linha();
  conserta();
}
