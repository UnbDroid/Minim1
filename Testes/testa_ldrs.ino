#define LED_dir 33
#define LED_esq 34
#define LDR_dir A3
#define LDR_esq A5

// ORDEM:
//Vcc (vermelho), Digital (azul), analogico (amarelo), ground (preto)
void setup(){
  Serial.begin(9600);
  pinMode(LDR_esq, INPUT);
  pinMode(LDR_dir, INPUT);
  pinMode(LED_dir, OUTPUT);
  pinMode(LED_esq, OUTPUT);



  digitalWrite(LED_esq, HIGH);
  digitalWrite(LED_dir, HIGH);

  analogRead(LDR_esq);
  analogRead(LDR_dir);

  analogRead(LDR_esq);
  analogRead(LDR_dir);

  analogRead(LDR_esq);
  analogRead(LDR_dir);
}

void loop(){
  int ldr_esq, ldr_dir;
  ldr_esq = analogRead(LDR_esq);
  ldr_dir = analogRead(LDR_dir);

  Serial.print("LDR ESQUERDA : ");
  Serial.print(ldr_esq);
  Serial.print(" || LDR DIREITA : ");
  Serial.println(ldr_dir);
  delay(500);

}
