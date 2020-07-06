#include <Wire.h>
#include <SoftwareSerial.h>

const int ledPin = 13;
const int JOYSTICK_X_PIN = 7;
const int JOYSTICK_Y_PIN = 6;
const int JOYSTICK_BUTTON_PIN = 5;

SoftwareSerial BTSerial(2, 3); // RX | TX

long previousMillis = 0;
const long TRANSMIT_INTERVAL = 140;

void setup() {
  Serial.begin(115200);
  BTSerial.begin(38400);
}

void loop() {
  long currentMillis = millis();
  if (currentMillis - previousMillis < TRANSMIT_INTERVAL) {
    return;
  }

  int xValue = analogRead(JOYSTICK_X_PIN);
  int yValue = analogRead(JOYSTICK_Y_PIN);

  if (yValue < 256) {
    BTSerial.write("F");
    Serial.println("F");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  } else if (yValue > 768) {
    BTSerial.write("B");
    Serial.println("B");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  } else if (xValue < 256) {
    BTSerial.write("L");
    Serial.println("L");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  } else if (xValue > 768) {
    BTSerial.write("R");
    Serial.println("R");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  /*
    Serial.print("x:");
    Serial.print(xValue);
    Serial.print(", y:");
    Serial.println(yValue);
  */

  previousMillis = millis();
  //delay(100);
}
