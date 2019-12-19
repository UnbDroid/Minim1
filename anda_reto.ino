// #include "MPU9250.h"
// #include <NewPing.h>

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
#define LDR_esq A1
#define LDR_dir A0

#define POT_MIN_MOTOR 65
#define POT_MAX_MOTOR 55
#define POT_MED_MOTOR 50

#define BRANCO 0
#define PRETO 6
#define min_esq  134
#define min_dir 117
#define max_esq 258
#define max_dir 215
#define MED_ESQ (min_esq+max_esq)/2
#define MED_DIR (min_dir+max_dir)/2

#define TRIGGER_PIN  34
#define US_FRONT     37
#define US_SIDE      36
#define MAX_DISTANCE 200
#define TIME_STEP 20
#define Turn_Tension 3
#define DIST_OBJ 15

#define kp 0.001
#define ki 0
#define kd 0

#define direita 0
#define esquerda 1
#define frente 2
#define tras 3

// NewPing usFront(TRIGGER_PIN, US_FRONT, MAX_DISTANCE);
// NewPing usSide(TRIGGER_PIN, US_SIDE, MAX_DISTANCE);

// PID Motores
int p = 0;
int i = 0;
int d = 0;
int erro_atual = 0;
int erro_total = 0;
int erro_ant = 0;
int pot_motor_esq = 80;
int pot_motor_dir = 80;
int encoder_d = 0;
int encoder_e = 0;

// Leitura LDR
int leitura_max = 0;
int leitura_min = 1000;
int contador = 0;
int iteracoes = 5;
int vetor_leituras_esquerda[5];
int vetor_leituras_direita[5];
int validacao = 0;
int leitura_dir = 0;
int leitura_esq = 0;

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
// MPU9250 gyro(Wire,0x68);
// int status;
// float degreeZ = 0;

void movimento(int direcao);
void trava_motores(int trava);
void anda_reto();
void acresce_encoder_e();
void acresce_encoder_d();

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

void anda_reto(){

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
  }else if(pot_motor_dir < 30){
    pot_motor_dir = 30;
  }

  if(pot_motor_esq > 130){
    pot_motor_esq = 130;
  }else if(pot_motor_esq < 30){
    pot_motor_esq = 30;
  }

  erro_ant = erro_atual;

  Serial.print("Direita: ");
  Serial.println(encoder_d);
  //
  Serial.print("Esquerda: ");
  Serial.println(encoder_e);

  Serial.print("P: ");
  Serial.println(p);
  Serial.print("Erro atual: ");
  Serial.println(erro_atual);

  Serial.print("Motor direita: ");
  Serial.println(pot_motor_dir);
  Serial.print("Motor esquerda: ");
  Serial.println(pot_motor_esq);

  Serial.println("----------------------------------------------------------------------------------------------------");


 // attachInterrupt(digitalPinToInterrupt(ENCODER_E), acresce_encoder_e, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(ENCODER_D), acresce_encoder_d, CHANGE);

}

void acresce_encoder_d(){
  encoder_d++;
}

void acresce_encoder_e(){
  encoder_e++;
}

void setup() {
  delay(5000);

  pinMode(MD , INPUT);
  pinMode(MDINA, INPUT);
  pinMode(MDINB, INPUT);
  pinMode(MDEN , INPUT);

  pinMode(ME , INPUT);
  pinMode(MEINA, INPUT);
  pinMode(MEINB, INPUT);
  pinMode(MEEN , INPUT);

  digitalWrite(MDEN, HIGH);
  digitalWrite(MEEN, HIGH);

  movimento(frente);

  attachInterrupt(digitalPinToInterrupt(ENCODER_E), acresce_encoder_e, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D), acresce_encoder_d, CHANGE);

  Serial.begin(9600);
}

void loop() {
  anda_reto();
  delay(400);
}
