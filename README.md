# ESP32 Self-Balancing Robot

A two-wheel self-balancing robot developed with an ESP32, an MPU6050 inertial measurement unit, and an L298N dual H-bridge motor driver. The robot estimates its tilt angle using a complementary filter and uses a PID controller to calculate the motor correction.

![Self-balancing robot prototype](images/robot.png)

## Project team

- **Developers:** Mohamed Nairi and Soliemen Essid
- **Organization:** Club CPU – ISIMM
- **Supervisor:** Rami Zaag — Robotics Training and Projects Manager

## Overview

The objective of this project is to build a two-wheel robot that maintains an upright position through continuous feedback:

1. The MPU6050 measures acceleration and angular velocity.
2. The ESP32 reads the sensor data over I²C.
3. A complementary filter combines accelerometer and gyroscope measurements to estimate the robot's tilt angle.
4. A PID controller calculates a correction from the difference between the target angle and the estimated angle.
5. The ESP32 drives both DC motors through the L298N motor driver using direction signals and PWM.

## Hardware

| Component | Role |
|---|---|
| ESP32 development board | Main controller and PID computation |
| MPU6050 | Accelerometer and gyroscope for tilt estimation |
| L298N | Dual DC motor driver |
| 2 DC geared motors | Robot movement and balance correction |
| 2 wheels and chassis | Mechanical platform |
| Battery / power supply | Motor and electronics power |

Check the voltage and current requirements of each component before wiring. The ESP32, MPU6050 and L298N logic must share a common ground. Do not power the motors from the ESP32 3.3 V pin.

## Wiring

The following table documents the GPIO assignments used in the supplied sketch.

### MPU6050 — I²C

| MPU6050 | ESP32 |
|---|---:|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| AD0 | GND (I²C address 0x68) |
| GND | GND |
| VCC | Appropriate module supply (check your board) |

### L298N — motors

| L298N signal | ESP32 |
|---|---:|
| ENA | GPIO 25 (PWM) |
| IN1 | GPIO 26 |
| IN2 | GPIO 27 |
| ENB | GPIO 13 (PWM) |
| IN3 | GPIO 14 |
| IN4 | GPIO 12 |

Connect the left motor to the L298N output pair A and the right motor to output pair B, then verify the actual motor direction with the wheels lifted off the ground. GPIO 12 is a boot-strapping pin on some ESP32 boards; if boot problems occur, move IN4 to a safer available GPIO and update the sketch.

## Control algorithm

### 1. Tilt estimation

The accelerometer estimates the angle relative to gravity. The gyroscope measures angular velocity, which is integrated over the elapsed time.

The complementary filter combines the two:

```text
angle = alpha * (previous_angle + gyro_rate * dt)
      + (1 - alpha) * accelerometer_angle
```

The sketch uses `ALPHA = 0.99` as an initial value. The correct angle axis and sign depend on the physical orientation of the MPU6050. Confirm the sensor mounting and test the angle response before enabling the motors.

### 2. PID controller

The controller uses the angle error:

```text
error = target_angle - measured_angle

P = Kp * error
I = Ki * integral(error * dt)
D = Kd * derivative(error)

PID_output = P + I + D
```

The output is limited to the configured PWM range. The integral is clamped to reduce integral windup.

Initial values in the supplied sketch:

| Parameter | Initial value |
|---|---:|
| Kp | 60.0 |
| Ki | 5.0 |
| Kd | 4.0 |
| PWM frequency | 20 kHz |
| PWM resolution | 8 bits |
| PWM maximum | 230 |
| PWM minimum start compensation | 60 |
| Complementary filter alpha | 0.99 |
| Tilt safety threshold | 35° |

These are starting values, not universal tuning values. The best gains depend on the motors, gear ratio, wheel size, battery voltage, chassis centre of mass, sensor orientation and loop timing.

## Software

- Arduino IDE
- ESP32 board support package
- Arduino `Wire` library
- ESP32 LEDC PWM API

The sketch uses the Arduino-ESP32 LEDC pin API (`ledcAttach()` and `ledcWrite(pin, duty)`), so use a compatible Arduino-ESP32 core version.

## Getting started

1. Assemble the chassis and mount the MPU6050 firmly.
2. Wire the MPU6050 and L298N according to the tables above.
3. Install ESP32 board support in Arduino IDE.
4. Open the project sketch from `src/`.
5. Select the correct ESP32 board and serial port.
6. Upload the program.
7. Keep the robot stationary during gyroscope calibration at startup.
8. With the wheels lifted, verify the measured angle, motor direction and correction direction.
9. Tune the PID gains gradually before testing on the floor.

## Safety and testing

- Start with the wheels lifted and keep hands clear of moving parts.
- Secure the robot and provide an accessible way to disconnect motor power.
- Verify that the motors correct the tilt in the direction that moves the wheels underneath the falling robot. If correction is reversed, stop and correct the motor direction or control sign before continuing.
- The sketch stops the motors when the estimated tilt exceeds the configured safety angle. This software threshold is not a substitute for a physical emergency stop.
- Calibrate the gyroscope while the robot is completely still.
- Confirm that the sensor axis used by the sketch matches the actual mounting orientation.
- Check motor-driver temperature and battery limits during tests.

## Repository structure

```text
self-balancing-robot/
├── README.md
├── images/
│   └── robot.png
└── src/
    └── self_balancing_robot.ino
```

Place the complete Arduino sketch in `src/self_balancing_robot.ino`.

## Future improvements

- Add encoder feedback for wheel speed and position.
- Improve sensor calibration and angle filtering.
- Add battery-voltage monitoring.
- Implement configurable PID gains and data logging.
- Add a dedicated physical emergency-stop circuit.
- Document repeatable PID tuning experiments.

---

**Club CPU – ISIMM**  
Educational robotics project.
# self-balancing-robot
