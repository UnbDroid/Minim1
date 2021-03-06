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



void StartGyro() {
  status = gyro.begin();
  if (status < 0)
    Serial.println("IMU initialization unsuccessful");
}



void UpdateGyro() {
  gyro.readSensor();
  degreeZ += (gyro.getGyroZ_rads() * TIME_STEP * 180) / (1000 * PI);
}



void gira(int angulo){
  unsigned long now,last_update = 0;
  int angulo_atual;

  if(angulo > 0){
    movimento(esquerda);

  }else{
    movimento(direita);
  }

  angulo_atual = degreeZ;

  while(abs(degreeZ - angulo_atual) < abs(angulo)){
    now = millis();
    if(now - last_update > 20){
      UpdateGyro();
      last_update = now;
      analogWrite(MD, 45);
      analogWrite(ME, 45);
    }
  }

  trava_motores(1);
}



void acresce_encoder_d(){
  encoder_d++;
}

void acresce_encoder_e(){
  encoder_e++;
}



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
      total += vetor_leituras_us_lat[i];
    }
  }

  return total/(iteracoes-2);
}



void le_ultra(){
  int f, l;
  if((f = usFront.ping_cm()) > 0 && (l = usSide.ping_cm()) > 0){
    vetor_leituras_us_front[contador] = f;
    vetor_leituras_us_lat[contador] = l;
    contador++;
  }


  while(contador<5){
    if((f = usFront.ping_cm()) > 0 && (l = usSide.ping_cm()) > 0){
      vetor_leituras_us_front[contador] = f;
      vetor_leituras_us_lat[contador] = l;
      contador++;
    }
  }

  if(contador == 5){
    contador = 0;
  }
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




void anda_reto(){

  movimento(frente);

  analogWrite(MD, pot_motor_dir);
  analogWrite(ME, pot_motor_esq);

  // detachInterrupt(digitalPinToInterrupt(ENCODER_E));
  // detachInterrupt(digitalPinToInterrupt(ENCODER_D));

  erro_atual = (encoder_d - encoder_e);
  erro_total += erro_atual;


  p = erro_atual;
  i = erro_total;
  d = erro_atual - erro_ant;

  pot_motor_dir = pot_motor_dir - ((kp*p) + (ki*i) + (kd*d));
  pot_motor_esq = pot_motor_esq + ((kp*p) + (ki*i) + (kd*d));

  if(pot_motor_dir > 130){
    pot_motor_dir = 130;
  }else if(pot_motor_dir < 45){
    pot_motor_dir = 45;
  }

  if(pot_motor_esq > 130){
    pot_motor_esq = 130;
  }else if(pot_motor_esq < 45){
    pot_motor_esq = 45;
  }

  erro_ant = erro_atual;
}




void desvia(){
  gira(90);
  Serial.println("Gira gira gira gira girooou");

  trava_motores(1);
  Serial.println("Putz travei");
  delay(200);

  attachInterrupt(digitalPinToInterrupt(ENCODER_E), acresce_encoder_e, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D), acresce_encoder_d, CHANGE);

  le_ultra();

  leitura_front = media_vetor(4);
  leitura_lat = media_vetor(5);

  movimento(frente);

  pot_motor_dir = POT_MED_MOTOR_DIR;
  pot_motor_esq = POT_MED_MOTOR_ESQ;
  Serial.println("Segue reto");

  Serial.println("Procurando objeto na direita");
  while(leitura_lat > DIST_OBJ_LATERAL){
    anda_reto();
    le_ultra();
    leitura_front = media_vetor(4);
    leitura_lat = media_vetor(5);
  }

  Serial.println("Achei o objeto");
  while(leitura_lat < DIST_OBJ_LATERAL){
    anda_reto();
    le_ultra();
    leitura_front = media_vetor(4);
    leitura_lat = media_vetor(5);
  }

  trava_motores(1);
  Serial.println("Putz travei");
  delay(200);

  gira(-80);
  Serial.println("Gira gira gira gira giroooou");

  trava_motores(1);
  Serial.println("Putz travei");
  delay(200);

  movimento(frente);
  Serial.println("Segue reto");

  encoder_e = 0;
  encoder_d = 0;

  Serial.println("Procurando de novo");

  le_ultra();

  leitura_front = media_vetor(4);
  leitura_lat = media_vetor(5);

  while(leitura_lat > DIST_OBJ_LATERAL){
    anda_reto();
    le_ultra();
    leitura_front = media_vetor(4);
    leitura_lat = media_vetor(5);
  }

  Serial.println("Achei caraio, boa porra, é isso!!!! Vamo que agora vai");
  while(leitura_lat < DIST_OBJ_LATERAL){
    anda_reto();
    le_ultra();
    leitura_front = media_vetor(4);
    leitura_lat = media_vetor(5);
  }

  trava_motores(1);
  Serial.println("Putz travei");
  delay(200);

  gira(-80);
  Serial.println("Gira gira gira gira giroooou");

  trava_motores(1);
  Serial.println("Putz travei");
  delay(200);

  encoder_e = 0;
  encoder_d = 0;

  le_ldr();
  leitura_dir = media_vetor(direita);
  leitura_esq = media_vetor(esquerda);

  Serial.println("Cade a linha preta na direita?");

  while(leitura_dir < MED_DIR){
    anda_reto();
    le_ldr();
    leitura_dir = media_vetor(direita);
    leitura_esq = media_vetor(esquerda);
  }

  Serial.println("Achei a linha");

  trava_motores(1);
  Serial.println("Putz travei");
  delay(200);

  gira(50);
  Serial.println("Gira gira gira gira girooou");

  trava_motores(1);
  Serial.println("Putz travei");
  delay(200);

  movimento(frente);
  analogWrite(MD, 45);
  analogWrite(ME, 45);
  delay(200);

  trava_motores(1);
  Serial.println("Putz travei");
  delay(200);

  gira(10);
  Serial.println("Gira gira gira gira girooou");
  Serial.println("Voltei a seguir linha");
}



void setup(){
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

  pinMode(BUTTON, INPUT);

  while(digitalRead(BUTTON) == 0);
  Serial.begin(9600);
  delay(2000);

  digitalWrite(LED_dir, HIGH);
  digitalWrite(LED_esq, HIGH);

  digitalWrite(MDEN, HIGH);
  digitalWrite(MEEN, HIGH);

  digitalWrite(MDINA, HIGH);
  digitalWrite(MDINB, LOW);
  digitalWrite(MEINA, LOW);
  digitalWrite(MEINB, HIGH);

  while(!Serial);
  StartGyro();

  analogWrite(ME, 45);
  analogWrite(MD, 50);
  delay(100);
}

void loop(){
  le_ultra();
  leitura_front = media_vetor(4);
  leitura_lat = media_vetor(5);
  if(leitura_front < DIST_OBJ){
    Serial.println("ACHEI");
    desvia();
  }
}
