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

const double PITCH_BALANCE_SP_STEP = 0.0015;
const double PITCH_DRIVE_SP_STEP = 0.0005;
const double DRIVE_SPEED_MIN = 40;
const double DRIVE_SPEED_MAX = 50;

boolean started = false;

long gyroXCalibration = 0, gyroYCalibration = 0;

double pitchBalanceSetPoint = 0;
double pitchDriveSetPoint = 0;

double pitchSetPoint = 0;
double pitchReading = 0;
double pitchOutput = 0;
PID pitchPid(&pitchReading, &pitchOutput, &pitchSetPoint, 30, 100, 1, REVERSE);
/*
double yawSetPoint = 0;
double yawReading = 0;
double yawOutput = 0;
PID yawPid(&yawReading, &yawOutput, &yawSetPoint, 1, 0, 0, DIRECT);
*/

int activeCommand = -1;
unsigned long commandTimer = 0;

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
  } else*/ if (command == 'f' || command == 'F') {
    activeCommand = command;
    commandTimer = millis();
    Serial.print("command: ");
    Serial.println((char)command);
    pitchDriveSetPoint += 0.75;
  } else if (command == 'b' || command == 'B') {
    activeCommand = command;
    commandTimer = millis();
    Serial.print("command: ");
    Serial.println((char)command);
    pitchDriveSetPoint -= 0.75;
  }

  unsigned long commandTimeMillis = millis() - commandTimer;
  if (commandTimer > 0 && commandTimeMillis > 5000) {
    activeCommand = -1;
    commandTimer = 0;
    pitchDriveSetPoint = 0;
    Serial.println("cleared command");
  }

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
  if(!started && pitchAccelerometer > -0.5 && pitchAccelerometer < 0.5) {
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
  float pitchGyro = pitchReading + gyroY * 0.000031;                             

  // Complementary filter to combine the gyro and accelerometer angle
  // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)
  pitchReading = pitchGyro * 0.9996 + pitchAccelerometer * 0.0004;

  // TODO compute accumulated Yaw

  if (started) {
    pitchSetPoint = pitchBalanceSetPoint + pitchDriveSetPoint;

    pitchPid.Compute();
    //yawPid.Compute();

    //Serial.print("yaw:");
    //Serial.print(yawReading);
    if (activeCommand != -1) {
      /*Serial.print("bsp:");
      Serial.print(pitchBalanceSetPoint);*/
      Serial.print(", dsp:");
      Serial.print(pitchDriveSetPoint);
      /*Serial.print(", sp:");
      Serial.print(pitchSetPoint);*/
      Serial.print(", p:");
      Serial.print(pitchReading);
      Serial.print(", o:");
      Serial.print(pitchOutput);
      Serial.println();
    }

    // Deadband to prevent oscillations
    if (pitchOutput > 0 && pitchOutput < 2) {
      pitchOutput = 0;
    }
    if (pitchOutput < 0 && pitchOutput > -2) {
      pitchOutput = 0;
    }

    if (activeCommand == 'f' || activeCommand == 'F') {
      if (commandTimeMillis > 1000) {
        if (pitchOutput < DRIVE_SPEED_MIN) {
          pitchDriveSetPoint += PITCH_DRIVE_SP_STEP;
        }
        if (pitchOutput > DRIVE_SPEED_MAX) {
          pitchDriveSetPoint -= PITCH_DRIVE_SP_STEP * 20;
        }
      }
    } else if (activeCommand == 'b' || activeCommand == 'B') {
      if (commandTimeMillis > 1000) {
        if (pitchOutput > -DRIVE_SPEED_MIN) {
          pitchDriveSetPoint -= PITCH_DRIVE_SP_STEP;
        }
        if (pitchOutput < -DRIVE_SPEED_MAX) {
          pitchDriveSetPoint += PITCH_DRIVE_SP_STEP * 20;
        }
      }
    } else if (pitchDriveSetPoint == 0) {
      // The self balancing point is adjusted when the remote control isn't trying to move the robot.
      // This should stop the robot from wandering.
      if (pitchOutput < 0) {
        pitchBalanceSetPoint += PITCH_BALANCE_SP_STEP;
      }
      if (pitchOutput > 0) {
        pitchBalanceSetPoint -= PITCH_BALANCE_SP_STEP;
      }
    }
    // Prevent runaway
    pitchDriveSetPoint = constrain(pitchDriveSetPoint, -1.2, 1.2);

    double speedA = pitchOutput;// + yawOutput;
    double speedB = pitchOutput;// - yawOutput;
    if (pitchReading > 45 || pitchReading < -45) {
      speedA = 0;
      speedB = 0;
      pitchDriveSetPoint = 0;
      activeCommand = -1;
    }

    speedA = constrain(speedA, -255, 255);
    speedB = constrain(speedB, -255, 255);

    // control speed, deadzone A:50, B:40
    double pwmA = map(abs(speedA),1,255,54,255);
    double pwmB = map(abs(speedB),1,255,40,255);
    analogWrite(ENA, pwmA);
    analogWrite(ENB, pwmB);
    //control direction 
    digitalWrite(IN1, speedA < 0 ? HIGH : LOW);
    digitalWrite(IN2, speedA > 0 ? HIGH : LOW);
    //direction reversed for second motor (wiring)
    digitalWrite(IN3, speedB < 0 ? LOW : HIGH);
    digitalWrite(IN4, speedB > 0 ? LOW : HIGH);
  }

  // The angle calculations are tuned for a loop time of 4 milliseconds
  delayMicroseconds(loopEndTime - micros()); // Simulate the main program loop time
  loopEndTime = micros() + LOOP_MICROS;
}
