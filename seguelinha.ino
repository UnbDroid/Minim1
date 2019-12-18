#define MD 4
#define MDINA 30
#define MDINB 31
#define MDEN 22


#define ME 5
#define MEINA 46
#define MEINB 47
#define MEEN 52

#define POT_MIN_MOTOR 30

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

  erro_atual = -leitura_esq + leitura_dir;
  erro_total += erro_atual;

  p = erro_atual;
  i = erro_total;
  d = erro_atual - erro_ant;

  erro_ant = erro_atual;

  pot_motor_esq -=  (kp*p + ki*i + kd*d)/4;
  pot_motor_dir +=  (kp*p + ki*i + kd*d)/4;
  Serial.print("Motor esq: ");
  Serial.println(pot_motor_esq);
  Serial.print("Motor dir: ");
  Serial.println(pot_motor_dir);

  if(pot_motor_esq > 150){
    pot_motor_esq = 150;
  }

  if(pot_motor_dir > 150){
    pot_motor_dir = 150;
  }
  if(pot_motor_esq < POT_MIN_MOTOR){
    pot_motor_esq = POT_MIN_MOTOR;
  }

  if(pot_motor_dir < POT_MIN_MOTOR){
    pot_motor_dir = POT_MIN_MOTOR;
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
