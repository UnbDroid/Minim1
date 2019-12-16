#include "MPU9250.h"

#define MD 3
#define MDINA 30
#define MDINB 31
#define MDEN 22

#define ME 4
#define MEINA 46
#define MEINB 47
#define MEEN 52

#define TIME_STEP 20
#define Turn_Tension 3

int pot_motor_esq = 60;
int pot_motor_dir = 60;

MPU9250 gyro(Wire,0x68);
int status;
float degreeZ = 0;

// Atualiza a posição do giro
void UpdateGyro() {
  gyro.readSensor();
  degreeZ += (gyro.getGyroZ_rads() * TIME_STEP * 180) / (1000 * PI);
}

// Inicializa o giroscopio
void StartGyro() {
  status = gyro.begin();
  if (status < 0)
    Serial.println("IMU initialization unsuccessful");
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

// se o argumento for 1, trava os motores e se for 0, libera os motores
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

// se o angulo for positivo, gira no sentido anti-horário
void gira_graus(int angulo){
  unsigned long now,last_update = 0;
  int angulo_atual;

  if(angulo > 0){
    movimento(esquerda);

  }else{
    digitalWrite(MEINA, LOW);
    digitalWrite(MEINB, HIGH);

    digitalWrite(MDINA, LOW);
    digitalWrite(MDINB, HIGH);
  }

  angulo_atual = degreeZ;

  while(abs(degreeZ - angulo_atual) < abs(angulo)){
    now = millis();
    if(now - last_update > 20){
      UpdateGyro();
//      Serial.print("Z = ");
//      Serial.println(degreeZ);
      last_update = now;
      analogWrite(MD, pot_motor_dir);
      analogWrite(ME, pot_motor_esq);
    }
  }

  trava_motores(1);
}

void setup() {
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

  delay(5000);
  Serial.begin(9600);
  while(!Serial);
  StartGyro();
  gira_graus(90);
  gira_graus(-90);
  gira_graus(-90);

}

void loop() {
}
