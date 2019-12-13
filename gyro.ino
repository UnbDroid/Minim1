#define MD 3
#define MDINA 30
#define MDINB 31
#define MDEN 22

#define ME 4
#define MEINA 46
#define MEINB 47
#define MEEN 52

int pot_motor_esq = 60;
int pot_motor_dir = 60;

/*------------------------------------------ Gyroscope Functions ---------------------------------------------*/

// Comments:
// Here we have the Turn function and the angular position calculation.

// Usage:
// To read the current angular position just call UpdateGyro, beware that this function shouldn't be called
// more than once in 20 milli seconds since it time step is 20.
// The Turn function receives the angle disired. Positive turns right and negative turns left.

// Gyro lib
#include "MPU9250.h"

// Iteration step in milli seconds
#define TIME_STEP 20

// Set the turning speed
#define Turn_Tension 3

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 gyro(Wire,0x68);
int status;
float degreeZ = 0;

// Gets gyro's raw readings (rad/s) and integrates them into angles (rad).
// Angles are also converted from rad to degrees.
void UpdateGyro() {
  gyro.readSensor();
  degreeZ += (gyro.getGyroZ_rads() * TIME_STEP * 180) / (1000 * PI);
}


// Gyro setup function
void StartGyro() {
  status = gyro.begin();
  if (status < 0)
    Serial.println("IMU initialization unsuccessful");
}

void setup() {
  delay(5000);
  Serial.begin(9600);
  while(!Serial);
  StartGyro();
}

void loop() {

  unsigned long now,last_update = 0;

  digitalWrite(MEINA, HIGH);
  digitalWrite(MEINB, LOW);

  digitalWrite(MDINA, HIGH);
  digitalWrite(MDINB, LOW);

  while(degreeZ < 90){
    now = millis();
    if(now - last_update > 20){
      UpdateGyro();
//      Serial.print("Z = ");
//      Serial.println(degreeZ);
      last_update = now;
      analogWrite(MD, pot_motor_dir);
      analogWrite(ME, pot_motor_esq);
    }

  }

  digitalWrite(MEINA, LOW);
  digitalWrite(MEINB, HIGH);

  digitalWrite(MDINA, LOW);
  digitalWrite(MDINB, HIGH);

  while(degreeZ >= 0){
    now = millis();
    if(now - last_update > 20){
      UpdateGyro();
//      Serial.print("Z = ");
//      Serial.println(degreeZ);
      last_update = now;
      analogWrite(MD, pot_motor_dir);
      analogWrite(ME, pot_motor_esq);
    }
  }

}
