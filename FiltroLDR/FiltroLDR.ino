// Sensor de luz antigo
// Min: 78; Max: 186; Media: 132

// Sensor de luz novo
// Min: ; Max: ; Media: 

int leitura_max = 0;
int leitura_min = 1000;
int contador = 0;
int iteracoes = 5;
int vetor_leituras[5];

int media_vetor(){
  int i=0, pos_min, pos_max;
  int minimo = 1000;
  int maximo = 0;
  int total = 0;

  for(i=0; i<iteracoes; i++){
    if(vetor_leituras[i] < minimo){
      minimo = vetor_leituras[i];
      pos_max = i;
    }
    if(vetor_leituras[i] > maximo){
      maximo = vetor_leituras[i];
      pos_min = i;
    }
  }

  if(pos_min == pos_max){
    pos_max++;
  }
  
  vetor_leituras[pos_min] = 0;
  vetor_leituras[pos_max] = 0;

  for(i=0; i<iteracoes; i++){
    total += vetor_leituras[i];
  }
  
  return total/(iteracoes-2);
}

void setup(){
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(51, OUTPUT);
}

void loop(){
  int ldr, media;
  digitalWrite(51, HIGH);
  ldr = analogRead(0);
  if(contador<5){
    vetor_leituras[contador] = ldr;
    contador++;
  //  Serial.print("Valor lido do ldr: ");
  //  Serial.println(ldr);
  }else{
    Serial.print("Valor da media dos 5: ");
    media = media_vetor();
    Serial.println(media);
    contador = 0;
  }
}
