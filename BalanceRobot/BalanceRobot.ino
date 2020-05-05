#include <PID_v1.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int IN1 = 7;
const int IN2 = 8;
const int ENA = 6;
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 11;

Adafruit_MPU6050 mpu;

double setPoint = -1.2;
double accelerationZ;
double output;
PID pid(&accelerationZ, &output, &setPoint, 150, 0.0, 0, DIRECT);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting serial");
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("Starting MPU");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("Initialised MPU");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
  pid.SetMode(AUTOMATIC);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  sensors_event_t a, g, temp;
  //int measurementTime = millis();
  mpu.getEvent(&a, &g, &temp);

  accelerationZ = a.acceleration.z;
  pid.Compute();

  Serial.print("ax:");
  Serial.print(a.acceleration.x);
  Serial.print(", ay:");
  Serial.print(a.acceleration.y);
  Serial.print(", az:");
  Serial.print(a.acceleration.z);
  Serial.print(", output:");
  Serial.print(output);
  Serial.println();
  delay(5);
  //control speed, deadzone A:50, B:40
  analogWrite(ENA, map(abs(output),0,255,50,255));
  analogWrite(ENB, map(abs(output),0,255,40,255));
  //control direction 
  digitalWrite(IN1, output < 0 ? LOW : HIGH);
  digitalWrite(IN2, output > 0 ? LOW : HIGH);
  //direction reversed for second motor (wiring)
  digitalWrite(IN3, output < 0 ? HIGH : LOW);
  digitalWrite(IN4, output > 0 ? HIGH : LOW);
}
