#include <ESP32Servo.h>

Servo servo;

const int SERVO_PIN = 25;

// Calibrazione DS3235
const int PWM_0   = 500;   // 0°
const int PWM_180 = 1800;  // 180°

void setup() {
  Serial.begin(115200);

  servo.attach(SERVO_PIN, 500, 2500);

  servo.writeMicroseconds(PWM_0);

  Serial.println("DS3235 CONTROLLO");
  Serial.println("Inserisci un angolo da 0 a 180:");
}

void loop() {

  if (Serial.available()) {

    int angolo = Serial.parseInt();

    if (angolo >= 0 && angolo <= 180) {

      int microsecondi = map(
        angolo,
        0, 180,
        PWM_0, PWM_180
      );

      servo.writeMicroseconds(microsecondi);

      Serial.print("Angolo: ");
      Serial.print(angolo);
      Serial.print("°  ->  PWM: ");
      Serial.print(microsecondi);
      Serial.println(" us");

    } else {
      Serial.println("Inserisci un valore tra 0 e 180.");
    }

    // Pulisce il buffer seriale
    while (Serial.available()) {
      Serial.read();
    }
  }
}