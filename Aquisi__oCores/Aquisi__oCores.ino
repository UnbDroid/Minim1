#define S0 38
#define S1 39
#define S2 40
#define S3 41
#define ColorOut 42

// Código pra atribuir os valores das frquencias maximas e minimas de verde, vermelho e azul
// Vermelho: 217/29
// Verde: 285/29
// Azul: 245/24

int freq = 0;
int green_freqMax = 0;
int red_freqMax = 0;
int blue_freqMax = 0;

int green_freqMin = 10000;
int red_freqMin = 10000;
int blue_freqMin = 10000;
void setup() {
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(ColorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  // Setting frequency scaling to 20%

  Serial.begin(9600);
  delay(300);
}

void loop() {
    // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  freq = pulseIn(ColorOut, LOW);
  if (freq > red_freqMax){
      red_freqMax = freq;
      Serial.print("Novo maior vermelho: ");
      Serial.println(red_freqMax);
  }else if (freq < red_freqMin){
      red_freqMin = freq;
      Serial.print("Novo menor vermelho: ");
      Serial.println(red_freqMin);
  }

   // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);

  freq = pulseIn(ColorOut, LOW);
  if (freq > green_freqMax){
      green_freqMax = freq;
      Serial.print("Novo maior verde: ");
      Serial.println(green_freqMax);
  }else if (freq < green_freqMin){
      green_freqMin = freq;
      Serial.print("Novo menor verde: ");
      Serial.println(green_freqMin);
  }

  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  freq = pulseIn(ColorOut, LOW);
  if (freq > blue_freqMax){
      blue_freqMax = freq;
      Serial.print("Novo maior azul: ");
      Serial.println(blue_freqMax);
  }else if (freq < blue_freqMin){
      blue_freqMin = freq;
      Serial.print("Novo menor azul: ");
      Serial.println(blue_freqMin);
  }

}
