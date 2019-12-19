#define S0 38
#define S1 39
#define S2 40
#define S3 41
#define ColorOut 42

int green_freq = 0;
int red_freq = 0;
int blue_freq = 0;

int red = 0;
int green = 0;
int blue = 0;

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
  
  red_freq = pulseIn(ColorOut, LOW);
  red = map(red_freq, 29, 217, 255, 0);

   // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);

  green_freq = pulseIn(ColorOut, LOW);
  green = map(green_freq, 29, 285, 255, 0);

  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  blue_freq = pulseIn(ColorOut, LOW);
  blue = map(blue_freq, 24, 245, 255, 0);

    if ((red > green) && (red > blue)){
        Serial.println("VERMELHO");
    }else if ((green > red) && (green > blue)){
        Serial.println("VERDE");
    }else if ((blue > red) && (blue > green)){
        Serial.println("AZUL");
    }else{
        Serial.println("ZeCava");
    }
    Serial.print("Vermelho: ");
    Serial.print(red);
    Serial.print(" Verde: ");
    Serial.print(green);
    Serial.print(" Azul: ");
    Serial.print(blue);
    Serial.println("");
  delay(200);
}
