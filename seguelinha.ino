#define M1PWM 3
#define M1INA 30
#define M1INB 31
#define M1EN 22

#define M2PWM 4
#define M2INA 46
#define M2INB 47
#define M2EN 52

#define kp = 0;
#define ki = 0;
#define kd = 0;

#define esq = A1;
#define dir = A0;

// PID Motores
int erro_total = 0;
int erro_ant = 0;
int pot_motor_esq = 150;
int pot_motor_dir = 150;

// Leitura LDR
int leitura_max = 0;
int leitura_min = 1000;
int contador = 0;
int iteracoes = 5;
int vetor_leituras_esquerda[5];
int vetor_leituras_direita[5];
int validacao = 0;

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

  vetor_leituras_esquerda[contador] = analogRead(esq);
  vetor_leituras_direita[contador] = analogRead(dir);
  contador++;

  while(validacao == 0 && contador<5){
    vetor_leituras_esquerda[contador] = analogRead(esq);
    vetor_leituras_direita[contador] = analogRead(dir);
    contador++;
  }
  validacao = 1;

  if(contador == 5){
    contador = 0;
  }
}


void segue_linha(){
  int p, i, d;
  int erro_atual = 0;

  le_ldr();

  leitura_esq = media_vetor(1);
  leitura_dir = media_vetor(0);

  erro_atual = leitura_esq - leitura_dir;
  erro_total += erro_atual;

  p = erro_atual;
  i = erro_total;
  d = erro_atual - erro_ant;

  erro_ant = erro_atual;

  pot_motor_esq -=  (kp*p + ki*i + kd*d);
  pot_motor_dir +=  (kp*p + ki*i + kd*d);

  if(pot_motor_esq > 220){
    pot_motor_esq = 220;
  }

  if(pot_motor_dir > 220){
    pot_motor_dir = 220;
  }
  if(pot_motor_esq < 30){
    pot_motor_esq = 30;
  }

  if(pot_motor_dir < 30){
    pot_motor_dir = 30;
  }

  analogWrite(M1PWM, pot_motor_dir);
  analogWrite(M2PWM, pot_motor_esq);

   // branco < preto
}





void setup(){

  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(50, OUTPUT);
  pinMode(51, OUTPUT);

  pinMode(M1PWM , INPUT);
  pinMode(M1INA, INPUT);
  pinMode(M1INB, INPUT);
  pinMode(M1EN , INPUT);

  pinMode(M2PWM , INPUT);
  pinMode(M2INA, INPUT);
  pinMode(M2INB, INPUT);
  pinMode(M2EN , INPUT);

  digitalWrite(M1EN, HIGH);
  digitalWrite(M2EN, HIGH);

  digitalWrite(M1INA, HIGH);
  digitalWrite(M1INB, LOW);
  digitalWrite(M2INA, HIGH);
  digitalWrite(M2INB, LOW);
  // put your setup code here, to run once:


}

void loop() {


 digitalWrite(M1EN, HIGH);
 digitalWrite(M2EN, HIGH);


 digitalWrite(M1INA, HIGH);
 digitalWrite(M1INB, LOW);
 analogWrite(M1PWM, pot_motor_dir);
 digitalWrite(M2INA, HIGH);
 digitalWrite(M2INB, LOW);
 analogWrite(M2PWM, pot_motor_esq);
 delay(1000);

 // Para motores

 digitalWrite(M1INA, HIGH);
 digitalWrite(M1INB, HIGH);
 digitalWrite(M2INA, HIGH);
 digitalWrite(M2INB, HIGH);
 delay(1000);

 // Gira outro lado

 digitalWrite(M1INA, LOW);
 digitalWrite(M1INB, HIGH);
 analogWrite(M1PWM, 150);
 digitalWrite(M2INA, LOW);
 digitalWrite(M2INB, HIGH);
 analogWrite(M2PWM, 150);
 delay(1000);
 // put your main code here, to run repeatedly:

}
