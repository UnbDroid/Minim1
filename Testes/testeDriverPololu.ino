
#define M1PWM 3
#define M1INA 30
#define M1INB 31
#define M1EN 22

#define M2PWM 4
#define M2INA 46
#define M2INB 47
#define M2EN 52



void setup(){
    pinMode(M1PWM , INPUT);
    pinMode(M1INA, INPUT);
    pinMode(M1INB, INPUT);
    pinMode(M1EN , INPUT);

    pinMode(M2PWM , INPUT);
    pinMode(M2INA, INPUT);
    pinMode(M2INB, INPUT);
    pinMode(M2EN , INPUT);
}

void loop(){
    digitalWrite(M1EN, HIGH);
    digitalWrite(M2EN, HIGH);


    digitalWrite(M1INA, HIGH);
    digitalWrite(M1INB, LOW);
    analogWrite(M1PWM, 150);
    digitalWrite(M2INA, HIGH);
    digitalWrite(M2INB, LOW);
    analogWrite(M2PWM, 150);
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

}