#include <Wire.h>
#include <PID_v1.h>

const int IN1 = 7;
const int IN2 = 8;
const int ENA = 6;
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 11;

const int GYRO_ADDRESS = 0x68;
const int ACCELEROMETER_Z_CALIBRATION = 1000;
const int LOOP_MICROS = 4000;

boolean started = false;

long gyroXCalibration, gyroYCalibration;

double pitchBalanceSetPoint = 0;
double pitchDriveSetPoint = 0;

double pitchSetPoint = 0;
double pitchReading;
double pitchOutput;
//PID pid(&pitchAngle, &output, &setPoint, 45, 200, 1.2, DIRECT);
//PID pid(&pitchAngle, &output, &setPoint, 45, 300, 1, DIRECT);
//PID pitchPid(&pitchReading, &pitchOutput, &pitchSetPoint, 45, 350, 0.5, DIRECT);
PID pitchPid(&pitchReading, &pitchOutput, &pitchSetPoint, 30, 100, 0.5, DIRECT);
/*
double yawSetPoint = 0;
double yawReading;
double yawOutput;
PID yawPid(&yawReading, &yawOutput, &yawSetPoint, 1, 0, 0, DIRECT);
*/
unsigned long driveTimer = 0;

unsigned long loopEndTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing MPU-6050...");

  // Wake up the MPU-6050
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00);
  Wire.endTransmission();
  // Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  // Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x08);
  Wire.endTransmission();
  // Enable Digital Low Pass Filter to improve the raw data.
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x03); // ~43Hz
  Wire.endTransmission();

  Serial.println("Initializing L298N...");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pitchPid.SetSampleTime(4);
  pitchPid.SetOutputLimits(-255, 255);
  /*yawPid.SetSampleTime(5);
  yawPid.SetOutputLimits(-10, 10);
  yawPid.SetMode(AUTOMATIC);*/

  Serial.println("Measuring gyro calibration values...");
  loopEndTime = micros() + LOOP_MICROS;
  for (int i = 0; i < 500; i++) {
    if (i % 15 == 0) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));        //Change the state of the LED every 15 loops to make the LED blink fast
    }
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x43); // GYRO_XOUT[15:8]
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDRESS, 4); // GYRO_XOUT[15:8], GYRO_XOUT[7:0], GYRO_YOUT[15:8], GYRO_YOUT[7:0]
    gyroXCalibration += Wire.read()<<8 | Wire.read();
    gyroYCalibration += Wire.read()<<8 | Wire.read();
    delayMicroseconds(loopEndTime - micros()); // Simulate the main program loop time
    loopEndTime = micros() + LOOP_MICROS;
  }
  // Get the average from the 500 readings
  gyroXCalibration /= 500;
  gyroYCalibration /= 500;

  Serial.println("Ready to roll!");
  digitalWrite(LED_BUILTIN, HIGH);
  loopEndTime = micros() + LOOP_MICROS;
}

void loop() {
  // Remote control
  // --------------
  int command = -1;
  while (Serial.available()) {
    command = Serial.read(); //reads serial input
  }
  /*if (command == 'l' || command == 'L') {
    yawSetPoint -= 15;
    Serial.print("yawSetPoint: ");
    Serial.println(yawSetPoint);
  } else if (command == 'r' || command == 'R') {
    yawSetPoint += 15;
    Serial.print("yawSetPoint: ");
    Serial.println(yawSetPoint);
  } else if (command == 'f' || command == 'F') {
    pitchSetPoint -= 0.75;
    driveTimer = millis();
    Serial.print("pitchSetPoint: ");
    Serial.println(pitchSetPoint);
  } else if (command == 'b' || command == 'B') {
    pitchSetPoint += 0.75;
    driveTimer = millis();
    Serial.print("pitchSetPoint: ");
    Serial.println(pitchSetPoint);
  }

  unsigned long driveTimeMillis = millis() - driveTimer;
  if (driveTimer > 0 && driveTimeMillis > 2000) {
    Serial.print("driveTimer: ");
    Serial.println(driveTimer);
    driveTimer = 0;
    pitchSetPoint = PITCH_UPRIGHT;
    Serial.print("pitchSetPoint: ");
    Serial.println(pitchSetPoint);
  }*/

  /* TODO handle yaw wrap-around somehow
  while (yawSetPoint < -180) {
    yawSetPoint += 360;
  }
  while (yawSetPoint > 180) {
    yawSetPoint -= 360;
  }*/

  // Angle calculations
  // ------------------
  /* http://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/
   * Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)
   * where 
   *     α = τ/(τ + Δt)   
   *     (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
   *     Δt = sampling rate (4ms)
   *     τ = time constant greater than timescale of typical accelerometer noise (1s)
   */

  // Read z accelerometer value
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x3F); // ACCEL_ZOUT[15:8]
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 2); // ACCEL_ZOUT[15:8], ACCEL_ZOUT[7:0]
  long accelerometerZ = Wire.read()<<8 | Wire.read();
  accelerometerZ += ACCELEROMETER_Z_CALIBRATION;
  accelerometerZ = constrain(accelerometerZ, -8200, 8200);

  // Calculate the current pitch angle in degrees according to the accelerometer
  float pitchAccelerometer = asin((float)accelerometerZ/8200.0)* 57.296;

  // Start balancing when angle is close to zero
  if(!started && pitchAccelerometer > -0.5&& pitchAccelerometer < 0.5) {
    pitchReading = pitchAccelerometer;
    started = true;
    pitchBalanceSetPoint = 0;
    pitchDriveSetPoint = 0;
    pitchPid.SetMode(AUTOMATIC);
  }

  // Read X and Y gyro values
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x43); // GYRO_XOUT[15:8]
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 4); // GYRO_XOUT[15:8], GYRO_XOUT[7:0], GYRO_YOUT[15:8], GYRO_YOUT[7:0]
  long gyroX = Wire.read()<<8 | Wire.read();
  long gyroY = Wire.read()<<8 | Wire.read();
  
  gyroX -= gyroXCalibration;
  gyroY -= gyroYCalibration;

  // Calculate the angle in degrees traveled during this loop angle
  // (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
  pitchReading += gyroY * 0.000031;                             

  // Complementary filter to combine the gyro and accelerometer angle
  // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)
  pitchReading = pitchReading * 0.9996 + pitchAccelerometer * 0.0004;

  // TODO compute accumulated Yaw

  if (started) {
    pitchSetPoint = pitchBalanceSetPoint + pitchDriveSetPoint;

    //Serial.print("yaw:");
    //Serial.print(yawReading);
    Serial.print("setpoint:");
    Serial.print(pitchSetPoint);
    Serial.print(", reading:");
    Serial.print(pitchReading);
    Serial.println();

    pitchPid.Compute();
    //yawPid.Compute();

    // Deadband to prevent oscillations
    if (pitchOutput > 0 && pitchOutput < 2) {
      pitchOutput = 0;
    }
    if (pitchOutput < 0 && pitchOutput > -2) {
      pitchOutput = 0;
    }

    // The self balancing point is adjusted when the remote control isn't trying to move the robot.
    // This should stop the robot from wandering.
    if (pitchDriveSetPoint == 0) {
      // Increase the self balancing setpoint if the robot is still moving forwards
      if (pitchOutput < 0) {
        pitchBalanceSetPoint -= 0.0015;
      }
      // Decrease the self balancing setpoint if the robot is still moving backwards
      if (pitchOutput > 0) {
        pitchBalanceSetPoint += 0.0015;
      }
    }

    double speedA = pitchOutput;// + yawOutput;
    double speedB = pitchOutput;// - yawOutput;
    if (pitchReading > 45 || pitchReading < -45) {
      speedA = 0;
      speedB = 0;
      // reset balancing point
    }

    speedA = constrain(speedA, -255, 255);
    speedB = constrain(speedB, -255, 255);

    // control speed, deadzone A:50, B:40
    double pwmA = map(abs(speedA),1,255,54,255);
    double pwmB = map(abs(speedB),1,255,40,255);
    analogWrite(ENA, pwmA);
    analogWrite(ENB, pwmB);
    //control direction 
    digitalWrite(IN1, speedA < 0 ? LOW : HIGH);
    digitalWrite(IN2, speedA > 0 ? LOW : HIGH);
    //direction reversed for second motor (wiring)
    digitalWrite(IN3, speedB < 0 ? HIGH : LOW);
    digitalWrite(IN4, speedB > 0 ? HIGH : LOW);
  }

  // The angle calculations are tuned for a loop time of 4 milliseconds
  delayMicroseconds(loopEndTime - micros()); // Simulate the main program loop time
  loopEndTime = micros() + LOOP_MICROS;
}
