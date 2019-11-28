#define LED_esq 50
#define LED_dir 51
#define LDR_esq A0
#define LDR_dir A1

void setup(){
  Serial.begin(9600);
  pinMode(LDR_esq, INPUT);
  pinMode(LDR_dir, INPUT);
  pinMode(LED_dir, OUTPUT);
  pinMode(LED_esq, OUTPUT);

  analogRead(LDR_esq);
  analogRead(LDR_dir);

  analogRead(LDR_esq);
  analogRead(LDR_dir);
}

int max_dir = 0;
int max_esq = 0;
int min_esq = 1110;
int min_dir = 1110;

void loop(){
  int ldr_esq, ldr_dir;
  digitalWrite(LED_esq, HIGH);
  digitalWrite(LED_dir, HIGH);
  ldr_esq = analogRead(LDR_esq);
  ldr_dir = analogRead(LDR_dir);

  if (ldr_esq > max_esq){
    max_esq = ldr_esq;
    Serial.print("Novo maximo esquerdo: ");
    Serial.println(max_esq);
  }else if (ldr_esq < min_esq){
    min_esq = ldr_esq;
    Serial.print("Novo minim1 esquerdo: ");
    Serial.println(min_esq);
  }

  if (ldr_dir > max_dir){
    max_dir = ldr_dir;
    Serial.print("Novo maximo direito: ");
    Serial.println(max_dir);
  }else if (ldr_dir < min_dir){
    min_dir = ldr_dir;
    Serial.print("Novo minim1 direito: ");
    Serial.println(min_dir);
  }
  delay(500);

}
