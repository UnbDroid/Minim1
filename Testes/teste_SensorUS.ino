    #include <NewPing.h>
     
    #define TRIGGER_PIN  34
    #define US_FRONT     37
    #define US_SIDE      36
    #define MAX_DISTANCE 200
     
    NewPing usFront(TRIGGER_PIN, US_FRONT, MAX_DISTANCE);
    NewPing usSide(TRIGGER_PIN, US_SIDE, MAX_DISTANCE);
     
    void setup() {
      Serial.begin(9600);
    }
     
    void loop() {
      delay(500);
      Serial.print("Frente: ");
      Serial.print(usFront.convert_cm(usFront.ping_median(5)));
      Serial.println("cm");
      Serial.print("Lado: ");
      Serial.print(usSide.convert_cm(usSide.ping_median(5)));
      Serial.println("cm");
    }
