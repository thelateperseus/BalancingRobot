#include <PID_v1.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

const int IN1 = 7;
const int IN2 = 8;
const int ENA = 6;
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 11;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double setPoint = 0;
double pitchAngle;
double output;
PID pid(&pitchAngle, &output, &setPoint, 30, 100, 0.5, DIRECT);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting serial");
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  //           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
  //OFFSETS    -3276,   -2046,    1929,     137,     -15,       8
  mpu.setXGyroOffset(137);
  mpu.setYGyroOffset(-15);
  mpu.setZGyroOffset(8);
  mpu.setXAccelOffset(-3276);
  mpu.setYAccelOffset(-2046);
  mpu.setZAccelOffset(1929);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pid.SetSampleTime(5);
  pid.SetOutputLimits(-255, 255);
  pid.SetMode(AUTOMATIC);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  //int measurementTime = millis();
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.resetFIFO();

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("yaw:");
    //Serial.print(ypr[0] * 180/M_PI);
    Serial.print(", pitch:");
    Serial.print(ypr[1] * 180/M_PI);
    //Serial.print(", roll:");
    //Serial.print(ypr[2] * 180/M_PI);
    pitchAngle = ypr[1] * 180/M_PI;

    pid.Compute();

    //Serial.print(", output:");
    //Serial.print(output);
    Serial.println();
    //delay(5);

    //control speed, deadzone A:50, B:40
    double outputA = map(abs(output),0,255,54,255);
    double outputB = map(abs(output),0,255,40,255);
    if (pitchAngle > 45 || pitchAngle < -45) {
      outputA = 0;
      outputB = 0;
    }
    analogWrite(ENA, outputA);
    analogWrite(ENB, outputB);
    //control direction 
    digitalWrite(IN1, output < 0 ? HIGH : LOW);
    digitalWrite(IN2, output > 0 ? HIGH : LOW);
    //direction reversed for second motor (wiring)
    digitalWrite(IN3, output < 0 ? LOW : HIGH);
    digitalWrite(IN4, output > 0 ? LOW : HIGH);
  }
}
