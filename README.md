Balancing Robot Project
=======================

An Arduino-based, two-wheeled balancing robot.

Components:
* Arduino Nano
* MPU-6050 6 DoF sensor
* L298N motor driver
* Machifit 25GA370 DC 12V 130rpm Gear Reduction Motor with Mounting Bracket and Wheel
* 2x 4-AA battery holders with switch
* HC-06 bluetooth module for remote control (via Android app or hand-held remote control)

Control Algorithm
-----------------

The main control alorithm changes the motor speed based on the measured angle. If the robot tips forward or backward, the motors drive forward or backward to prevent the robot from falling over. Two PID controllers are used to achieve this. The first measures the angle from the MPU-6050 and changes the motor speed PWM signal. The second uses the motor speed PWM signal and changes the angle set point. The remote control sets the speed set point, and the PID controller adjusts the angle set point accordingly.

```
              Speed             Speed                   Angle             Angle                  Speed              Angle
-----------  setpoint  -------  error  --------------  setpoint  -------  error  --------------  output  ---------  reading
| Remote  | ---------> | Sum | ------> | Speed PID  | ---------> | Sum | ------> | Angle PID  | ---+---> | Robot | ---
| Control |          + -------         | Controller |          + -------         | Controller |    |     ---------   |
-----------               ^  -         --------------               ^  -         --------------    |                 |
                          |                                         |                              |                 |
                          |                                         -------------------------------+------------------
                          |                                                                        |
                          --------------------------------------------------------------------------
```

The motors do not have encoders, so the speed PWM signal is used as the actual speed of the robot. If the motors had encoders, we would read the actual speed from the encoders instead.
