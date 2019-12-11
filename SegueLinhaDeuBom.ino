#define MD 3
#define MDINA 30
#define MDINB 31
#define MDEN 22


#define ME 4
#define MEINA 46
#define MEINB 47
#define MEEN 52

#define POT_MIN_MOTOR 40
#define POT_MAX_MOTOR 60
#define POT_MED_MOTOR 50

#define LED_dir 50
#define LED_esq 51

#define kp 0.35
#define ki 0
#define kd 0

#define LDR_esq A1
#define LDR_dir A0

#define BRANCO 0
#define PRETO 6
#define min_esq  134
#define min_dir 117
#define max_esq 258
#define max_dir 215
#define MED_ESQ (min_esq+max_esq)/2
#define MED_DIR (min_dir+max_dir)/2

// PID Motores
int erro_total = 0;
int erro_ant = 0;
int pot_motor_esq = 30;
int pot_motor_dir = 30;

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


void segue_linha(){
  int p, i, d;
  int erro_atual = 0;
  int leitura_dir = 0, leitura_esq = 0;

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

  if(leitura_dir == leitura_esq)
  {
    digitalWrite(MDINA, HIGH);
    digitalWrite(MDINB, LOW);
    digitalWrite(MEINA, LOW);
    digitalWrite(MEINB, HIGH);
      pot_motor_dir = POT_MED_MOTOR;
      pot_motor_esq = POT_MED_MOTOR;
  } else if(leitura_dir > leitura_esq)
  {
        digitalWrite(MDINA, LOW);
        digitalWrite(MDINB, HIGH);
      digitalWrite(MEINA, LOW);
    digitalWrite(MEINB, HIGH);  
      pot_motor_esq = POT_MAX_MOTOR;
      pot_motor_dir = POT_MIN_MOTOR;
  } else if(leitura_esq > leitura_dir)
  {
      digitalWrite(MEINA, HIGH);
      digitalWrite(MEINB, LOW);
      digitalWrite(MDINA, HIGH);
    digitalWrite(MDINB, LOW);
      pot_motor_dir = POT_MAX_MOTOR;
      pot_motor_esq = POT_MIN_MOTOR;
  }

  analogWrite(MD, pot_motor_dir);
  analogWrite(ME, pot_motor_esq);

   // branco < preto
}





void setup(){

  Serial.begin(9600);
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
  delay(5000);
  // put your setup code here, to run once:


}

void loop() {


  segue_linha();
 // put your main code here, to run repeatedly:

}
