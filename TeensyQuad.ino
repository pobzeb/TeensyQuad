#include <Wire.h>
#include <L3G.h>
#include <ADXL345.h>
#include <HMC5883L.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include "Config.h"

// Servo for each motor and motor value
Servo m1, m2, m3, m4;
int m1_val, m2_val, m3_val, m4_val;
byte motorMode = 0b00001111;
int maxMotorDelta = 0;

// Receiver channels
byte last_heartbeat, last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_arm;
unsigned long heartbeatTimer, timer_1, timer_2, timer_3, timer_4, current_time;
int heartbeatPulse;

// PID controllers for all three axis
float rollErrSum,   rollKp,   rollKi,   rollKd,   lastRollError;
float pitchErrSum,  pitchKp,  pitchKi,  pitchKd,  lastPitchError;
float yawErrSum,    yawKp,    yawKi,    yawKd,    lastYawError;
int rollOutput = 0, pitchOutput = 0, yawOutput = 0;
int rollSetPoint = 0, pitchSetPoint = 0, yawSetPoint = 0;

// Hold sensor values and calibration
L3G gyro;
ADXL345 accel;
HMC5883L compass;
Adafruit_BMP085 barometer;
int calIdx = 0;
float rollCalibration, pitchCalibration, yawCalibration;
float gyroRollRaw, gyroPitchRaw, gyroYawRaw;
float accelRollRaw, accelPitchRaw, accelYawRaw;
float gyroRoll, gyroPitch, gyroYaw;
float accelRoll, accelPitch, accelYaw;
float heading;
float sensorRoll, sensorPitch, sensorYaw;
float altitude, temperature;
int pressure;

// Current flight state and loop timer
int flightState = STATE_WAITING_FOR_INIT;
unsigned long loopTimer;

// Hold input Throttle, roll, pitch and yaw
float current_throttle_value = 0.0f;
int throttle_input, roll_input, pitch_input, yaw_input;

void setup() {
  Serial.begin(9600);

  Serial.println(F("Beginning Setup"));
  if (SENSORS == true) {
    // Initialize the sensors
    flashLED(2);
    delay(200);
    Serial.println(F("Sensor Setup"));
    Sensors_init();
  }

  if (MOTORS == true) {
    // Initialize the motors
    flashLED(2);
    delay(200);
    Serial.println(F("Motor Setup"));
    Motors_init();
  }

  // Interrupts for receiver channels
  pinMode(HEARTBEAT_PIN, INPUT);
  pinMode(THROTTLE_CHANNEL_PIN, INPUT);
  pinMode(YAW_CHANNEL_PIN, INPUT);
  pinMode(PITCH_CHANNEL_PIN, INPUT);
  pinMode(ROLL_CHANNEL_PIN, INPUT);
  pinMode(ARM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HEARTBEAT_PIN), isrCheck, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_CHANNEL_PIN), isrCheck, CHANGE);
  attachInterrupt(digitalPinToInterrupt(YAW_CHANNEL_PIN), isrCheck, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PITCH_CHANNEL_PIN), isrCheck, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROLL_CHANNEL_PIN), isrCheck, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ARM_PIN), isrCheck, CHANGE);

  // Now LED off
  delay(200);
  flashLED(6);
  digitalWriteFast(LED, LOW);
  Serial.println(F("Setup Complete"));
  delay(200);

  // Set the timer.
  loopTimer = millis();
}

// Function to quickly flash led
void flashLED(int times) {
  for (int i = 1; i <= times; i++) {
    digitalWriteFast(LED, HIGH);
    delay(90);
    digitalWriteFast(LED, LOW);
    delay(90);
  }
}

void Sensors_init() {
  // Start the sensors
  Wire.begin();
  delay(3000);

  Serial.println(F("Finding Gyro..."));
  if (!gyro.init(gyro.device_4200D, gyro.sa0_high)) {
    Serial.println(F("Failed to autodetect gyro type!"));
    while(1);
  }
  gyro.enableDefault();
  Wire.beginTransmission(105);
  Wire.write(0x20);
  Wire.write(0x0F);
  Wire.endTransmission();

  Wire.beginTransmission(105);
  Wire.write(0x23);
  Wire.write(0x90);
  Wire.endTransmission();
  Serial.println(F("Gyro initialized"));
  delay(250);

  Serial.println(F("Finding Accel..."));
  if (!accel.begin()) {
    Serial.println(F("Failed to detect accelerometer!"));
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16G);
  Serial.println(F("Accel initialized"));
  delay(250);

  Serial.println(F("Finding Compass..."));
  compass = HMC5883L();
  int error;
  error = compass.SetScale(1.3);
  if(error != 0) {
    Serial.println(compass.GetErrorText(error));
  }
  error = compass.SetMeasurementMode(Measurement_Continuous);
  if(error != 0) {
    Serial.println(compass.GetErrorText(error));
  }
  Serial.println(F("Compass initialized"));
  delay(250);

  Serial.println(F("Finding Barometer..."));
  if (!barometer.begin()) {
    Serial.println(F("Failed to detect barometer!"));
    while(1);
  }
  Serial.println(F("Barometer initialized"));
  delay(250);

  // Set the PID values
  rollKp = ROLL_PID_KP;
  rollKi = ROLL_PID_KI;
  rollKd = ROLL_PID_KD;
  pitchKp = PITCH_PID_KP;
  pitchKi = PITCH_PID_KI;
  pitchKd = PITCH_PID_KD;
  yawKp = YAW_PID_KP;
  yawKi = YAW_PID_KI;
  yawKd = YAW_PID_KD;

  doCalibration();
}

void doCalibration() {
  Serial.println(F("Running calibration..."));

  // Reset PID calculations.
  rollErrSum = 0;
  lastRollError = 0;
  pitchErrSum = 0;
  lastPitchError = 0;
  yawErrSum = 0;
  lastYawError = 0;

  // Zero out the calibrated values
  rollCalibration   = 0;
  pitchCalibration  = 0;
  yawCalibration    = 0;
  gyroRoll   = 0;
  gyroPitch  = 0;
  gyroYaw    = 0;
  accelRoll  = 0;
  accelPitch = 0;
  accelYaw   = 0;
  digitalWriteFast(LED, LOW);

  // Calibrate and get a baseline setpoint for all axis
  unsigned long calibrationTimer = millis();
  int calibrationSteps = CALIBRATION_SAMPLE_SIZE;
  Serial.print(F("0\%"));
  for (calIdx = 0; calIdx < calibrationSteps; calIdx++) {
    // Blink the LED
    if (calIdx % 15 == 0) digitalWriteFast(LED, !digitalRead(LED));
    if (calIdx > 0 && calIdx % (calibrationSteps / 10) == 0) {
      Serial.printf("...%d%%", ((calIdx * 100) / calibrationSteps));
    }

    // Read the sensor values
    readSensors();
    if (calIdx % 4 == 0) readAltitude();

    // Accumulate
    rollCalibration  += sensorRoll;
    pitchCalibration += sensorPitch;
    yawCalibration   += sensorYaw;

    while (millis() - calibrationTimer < 4000);
    calibrationTimer = millis();
  }

  // Get an average
  rollCalibration  /= calibrationSteps;
  pitchCalibration /= calibrationSteps;
  yawCalibration   /= calibrationSteps;

  Serial.print(F("...Done\n"));
  Serial.println(F("Calibration complete."));
  digitalWriteFast(LED, LOW);
}

void readSensors() {
  // Read gyro and accel.
  readGyro();
  readAccel();
  readCompass();

  // Complementary filter to combine gyro and accelerometer.
  gyroRoll  = GYRO_TRUST * (gyroRoll  + gyroRollRaw)  + (accelRoll  * (1.0 - GYRO_TRUST));
  gyroPitch = GYRO_TRUST * (gyroPitch + gyroPitchRaw) + (accelPitch * (1.0 - GYRO_TRUST));
  gyroYaw   = GYRO_TRUST * (gyroYaw   + gyroYawRaw)   + (heading    * (1.0 - GYRO_TRUST));

  sensorRoll  = gyroRoll;
  sensorPitch = gyroPitch;
  sensorYaw   = gyroYaw;

  // Add in calibration offsets.
  if (calIdx == CALIBRATION_SAMPLE_SIZE) {
    sensorRoll  -= rollCalibration;
    sensorPitch -= pitchCalibration;
    sensorYaw   -= yawCalibration;
  }
}

void readGyro() {
  gyro.read();

  gyroRollRaw = gyro.g.y;
  gyroPitchRaw = gyro.g.x * -1;
  gyroYawRaw = gyro.g.z * -1;

  // Convert to dps and calculate time delta.
  gyroRollRaw  = (gyroRollRaw  / 57.14286) * LOOP_DT;
  gyroPitchRaw = (gyroPitchRaw / 57.14286) * LOOP_DT;
  gyroYawRaw   = (gyroYawRaw   / 57.14286) * LOOP_DT;

  // Final values are angle change in degrees for the time delta.
}

void readAccel() {
  Vector norm = accel.readNormalize();
  Vector filtered = accel.lowPassFilter(norm, 0.5);
  accelRollRaw  = filtered.XAxis;
  accelPitchRaw = filtered.YAxis;
  accelYawRaw   = filtered.ZAxis;

  // Convert accel data to roll, pitch and yaw.
  accelRoll  = -(atan2(accelRollRaw, sqrt(accelPitchRaw * accelPitchRaw + accelYawRaw * accelYawRaw)) * 180.0) / M_PI;
  accelPitch =  (atan2(-accelPitchRaw, accelYawRaw) * 180.0) / M_PI;
  accelYaw   =  0;

  // Final values are angle in degrees.
}

void readCompass() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  heading = atan2(scaled.YAxis, scaled.XAxis);

  // Correct for when signs are reversed.
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;

  heading *= RAD_TO_DEG;
}

void readAltitude() {
  // Get altitude in meters.
  altitude = (altitude * 0.98f) + (barometer.readAltitude() * 0.02f);
  // Get temperature in deg F.
//  temperature = (barometer.readTemperature() * 1.8f) + 32.0f;
  // Get pressure in Pa.
//  pressure = (pressure * 0.98f) + (barometer.readPressure() * 0.02f);
}

void Motors_init() {
  // Attach the motors
  m1.attach(M1_PIN);
  m2.attach(M2_PIN);
  m3.attach(M3_PIN);
  m4.attach(M4_PIN);

  m1.writeMicroseconds(MIN_MOTOR_VALUE);
  m2.writeMicroseconds(MIN_MOTOR_VALUE);
  m3.writeMicroseconds(MIN_MOTOR_VALUE);
  m4.writeMicroseconds(MIN_MOTOR_VALUE);

  // Motors initialized
  flightState = STATE_MOTORS_INITIALIZED;

  // Short pause
  delay(2000);
}

void loop() {
  // Read the gyro and calculate the new values
  if (SENSORS == true) {
    readSensors();
  }

  // Calculate the pid motor values
  calculatePID();

  // Check the heartbeat pulse is greater than the timeout
  heartbeatPulse = micros() - heartbeatTimer;
  if (heartbeatPulse > HEARTBEAT_TIMEOUT) {
    // No heartbeat received so kill the motors
    flightState = STATE_WAITING_FOR_INIT;
  }

  // Calculate the pid motor values
  calculatePID();

  if (flightState == STATE_RUNNING) {
    // Leave padding for pid values.
    if (throttle_input > 1800) throttle_input = 1800;

    // Mix in the deltas and combine with throttle
    m1_val = throttle_input - pitchOutput - rollOutput - yawOutput;
    m2_val = throttle_input + pitchOutput - rollOutput + yawOutput;
    m3_val = throttle_input + pitchOutput + rollOutput - yawOutput;
    m4_val = throttle_input - pitchOutput + rollOutput + yawOutput;

    // Find the motor with the most power over the MAX_MOTOR_VALUE and
    // reduce all motors by that amount.
    maxMotorDelta = 0;
    maxMotorDelta = max(0, (m1_val - MAX_MOTOR_VALUE));
    maxMotorDelta = max(maxMotorDelta, (m2_val - MAX_MOTOR_VALUE));
    maxMotorDelta = max(maxMotorDelta, (m3_val - MAX_MOTOR_VALUE));
    maxMotorDelta = max(maxMotorDelta, (m4_val - MAX_MOTOR_VALUE));
    m1_val -= maxMotorDelta;
    m2_val -= maxMotorDelta;
    m3_val -= maxMotorDelta;
    m4_val -= maxMotorDelta;

    // Make sure the motors don't drop below min + padding
    if (throttle_input > 1050) {
      m1_val = max(m1_val, 1200);
      m2_val = max(m2_val, 1200);
      m3_val = max(m3_val, 1200);
      m4_val = max(m4_val, 1200);
    }
    else {
      m1_val = max(m1_val, MIN_MOTOR_VALUE);
      m2_val = max(m2_val, MIN_MOTOR_VALUE);
      m3_val = max(m3_val, MIN_MOTOR_VALUE);
      m4_val = max(m4_val, MIN_MOTOR_VALUE);
    }

    // Stop motors that are not active
    if (!(motorMode & 0b00000001)) m1_val = MIN_MOTOR_VALUE;
    if (!(motorMode & 0b00000010)) m2_val = MIN_MOTOR_VALUE;
    if (!(motorMode & 0b00000100)) m3_val = MIN_MOTOR_VALUE;
    if (!(motorMode & 0b00001000)) m4_val = MIN_MOTOR_VALUE;
  }
  else {
    // Set min values
    m1_val = MIN_MOTOR_VALUE;
    m2_val = MIN_MOTOR_VALUE;
    m3_val = MIN_MOTOR_VALUE;
    m4_val = MIN_MOTOR_VALUE;
  }

  // Wait at least 4ms.
  while (micros() - loopTimer < 4000);
  loopTimer = micros();

  if (DEBUG == true) {
    // Print the results.
    // printResults();
  }

  if (MOTORS == true) {
    // Write the current speed to each motor
    m1.writeMicroseconds(m1_val);
    m2.writeMicroseconds(m2_val);
    m3.writeMicroseconds(m3_val);
    m4.writeMicroseconds(m4_val);
  }
}

void isrCheck() {
  current_time = micros();

  // Check the heartbeat
  if (digitalReadFast(HEARTBEAT_PIN)) {
    if (last_heartbeat == 0) {
      last_heartbeat = 1;
      heartbeatTimer = current_time;
    }
  }
  else if (last_heartbeat == 1) {
    last_heartbeat = 0;
  }

  // Channel 1 - Throttle
  if (digitalReadFast(THROTTLE_CHANNEL_PIN)) {
    if (last_channel_1 == 0) {
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  }
  else if (last_channel_1 == 1) {
    last_channel_1 = 0;
    throttle_input = current_time - timer_1;
  }

  // Channel 2 - Yaw
  if (digitalReadFast(YAW_CHANNEL_PIN)) {
    if (last_channel_2 == 0) {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  }
  else if (last_channel_2 == 1) {
    last_channel_2 = 0;
    yaw_input = current_time - timer_2;
  }

  // Channel 3 - Pitch
  if (digitalReadFast(PITCH_CHANNEL_PIN)) {
    if (last_channel_3 == 0) {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  }
  else if (last_channel_3 == 1) {
    last_channel_3 = 0;
    pitch_input = current_time - timer_3;
  }

  // Channel 4 - Roll
  if (digitalReadFast(ROLL_CHANNEL_PIN)) {
    if (last_channel_4 == 0) {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  }
  else if (last_channel_4 == 1) {
    last_channel_4 = 0;
    roll_input = current_time - timer_4;
  }

  // Arm button
  if (digitalReadFast(ARM_PIN)) {
    if (last_arm == 0) {
      last_arm = 1;
    }
  }
  else if (last_arm == 1) {
    last_arm = 0;
    if (flightState != STATE_RUNNING) {
      flightState = STATE_RUNNING;

      // Reset PID calculations.
      rollErrSum = 0;
      lastRollError = 0;
      pitchErrSum = 0;
      lastPitchError = 0;
      yawErrSum = 0;
      lastYawError = 0;
    }
    else {
      flightState = STATE_MOTORS_INITIALIZED;
    }
  }
}

void printResults() {
  Serial.printf(F("Throttle: %4d"), throttle_input);
  // if (currentPIDAxis == 'r') {
  //   Serial.printf(F(", Roll PID - Kp: %4.6f, Ki: %4.6f, Kd: %4.6f"), rollKp, rollKi, rollKd);
  // }
  //   Serial.printf(F(", Pitch PID - Kp: %4.6f, Ki: %4.6f, Kd: %4.6f"), pitchKp, pitchKi, pitchKd);
  // }
  // else if (currentPIDAxis == 'y') {
  //   Serial.printf(F(", Yaw PID - Kp: %4.6f, Ki: %4.6f, Kd: %4.6f"), yawKp, yawKi, yawKd);
  // }
  // Serial.printf(F("    R: %s%4.4f    P: %s%4.4f    Y: %s%4.4f"), rollOutput < 0 ? "-" : "+", abs(rollOutput), pitchOutput < 0 ? "-" : "+", abs(pitchOutput), yawOutput < 0 ? "-" : "+", abs(yawOutput));
  // Serial.printf(F("    R: %s%4.4f    P: %s%4.4f    Y: %s%4.4f"), rollOutput < 0 ? "-" : "+", abs(rollOutput), pitchOutput < 0 ? "-" : "+", abs(pitchOutput), yawOutput < 0 ? "-" : "+", abs(yawOutput));
  // Serial.printf(F("    gRollOffset: %s%4.4f    gPitchOffset: %s%4.4f    gYawOffset: %s%4.4f"), gyroRollCalibration < 0 ? "-" : "+", abs(gyroRollCalibration), gyroPitchCalibration < 0 ? "-" : "+", abs(gyroPitchCalibration), gyroYawCalibration < 0 ? "-" : "+", abs(gyroYawCalibration));
  // Serial.printf(F("    gRoll: %s%4.4f    gPitch: %s%4.4f    gYaw: %s%4.4f"), gyroRoll < 0 ? "-" : "+", abs(gyroRoll), gyroPitch < 0 ? "-" : "+", abs(gyroPitch), gyroYaw < 0 ? "-" : "+", abs(gyroYaw));
  // Serial.printf(F("    aRollOffset: %s%4.4f    aPitchOffset: %s%4.4f    aYawOffset: %s%4.4f"), accelRollCalibration < 0 ? "-" : "+", abs(accelRollCalibration), accelPitchCalibration < 0 ? "-" : "+", abs(accelPitchCalibration), accelYawCalibration < 0 ? "-" : "+", abs(accelYawCalibration));
  // Serial.printf(F("    aRoll: %s%4.4f    aPitch: %s%4.4f"), accelRoll < 0 ? "-" : "+", abs(accelRoll), accelPitch < 0 ? "-" : "+", abs(accelPitch));
  // Serial.printf(F("    sRoll: %s%4.4f    sPitch: %s%4.4f    sYaw: %s%4.4f"), (sensorRoll < 0 ? "-" : "+"), abs(sensorRoll), (sensorPitch < 0 ? "-" : "+"), abs(sensorPitch), (sensorYaw < 0 ? "-" : "+"), abs(sensorYaw));
  Serial.printf(F("    M1: %4d    M2: %4d    M3: %4d    M4: %4d    MaxDelta: %d"), m1_val, m2_val, m3_val, m4_val, maxMotorDelta);
  // Serial.printf(F("    Roll: %s%4d    Pitch: %s%4d    Yaw: %s%4d"), rollSetPoint < 0 ? "-" : "+", abs(rollSetPoint), pitchSetPoint < 0 ? "-" : "+", abs(pitchSetPoint), yawSetPoint < 0 ? "-" : "+", abs(yawSetPoint));
  Serial.printf(F("    Motors Armed: %s"), flightState == STATE_RUNNING ? "True" : "False");
  Serial.printf(F("\n"));
}

void calculatePID() {
  // Roll PID
  float rollError = sensorRoll - rollSetPoint;
  rollErrSum += rollKi * rollError;
  if (rollErrSum > ROLL_PID_MAX) rollErrSum = ROLL_PID_MAX;
  if (rollErrSum < -ROLL_PID_MAX) rollErrSum = -ROLL_PID_MAX;
  rollOutput = (rollKp * rollError + rollErrSum + rollKd * (rollError - lastRollError));
  if (rollOutput > 90.0) rollOutput = 90.0;
  if (rollOutput < -90.0) rollOutput = -90.0;
  rollOutput = map(rollOutput, -90.0, 90.0, -ROLL_PID_MAX, ROLL_PID_MAX);
  if (rollOutput > ROLL_PID_MAX) rollOutput = ROLL_PID_MAX;
  if (rollOutput < -ROLL_PID_MAX) rollOutput = -ROLL_PID_MAX;
  lastRollError = rollError;

  // Pitch PID
  float pitchError = sensorPitch - pitchSetPoint;
  pitchErrSum += pitchKi * pitchError;
  if (pitchErrSum > PITCH_PID_MAX) pitchErrSum = PITCH_PID_MAX;
  if (pitchErrSum < -PITCH_PID_MAX) pitchErrSum = -PITCH_PID_MAX;
  pitchOutput = (pitchKp * pitchError + pitchErrSum + pitchKd * (pitchError - lastPitchError));
  if (pitchOutput > 90.0) pitchOutput = 90.0;
  if (pitchOutput < -90.0) pitchOutput = -90.0;
  pitchOutput = map(pitchOutput, -90.0, 90.0, -PITCH_PID_MAX, PITCH_PID_MAX);
  if (pitchOutput > PITCH_PID_MAX) pitchOutput = PITCH_PID_MAX;
  if (pitchOutput < -PITCH_PID_MAX) pitchOutput = -PITCH_PID_MAX;
  lastPitchError = pitchError;

  // Yaw PID
  float yawError = sensorYaw - yawSetPoint;
  yawErrSum += yawKi * yawError;
  if (yawErrSum > YAW_PID_MAX) yawErrSum = YAW_PID_MAX;
  if (yawErrSum < -YAW_PID_MAX) yawErrSum = -YAW_PID_MAX;
  yawOutput = (yawKp * yawError + yawErrSum + yawKd * (yawError - lastYawError));
  if (yawOutput > 360.0) yawOutput = 360.0;
  if (yawOutput < -360.0) yawOutput = -360.0;
  yawOutput = map(yawOutput, -360.0, 360.0, -YAW_PID_MAX, YAW_PID_MAX);
  if (yawOutput > YAW_PID_MAX) yawOutput = YAW_PID_MAX;
  if (yawOutput < -YAW_PID_MAX) yawOutput = -YAW_PID_MAX;
  lastYawError = yawError;
  // Serial.printf(F("R: %4.2f    RSP: %d    RE: %4.4f    RO: %d        P: %4.2f    PSP: %d    PE: %4.4f    PO: %d\n"), sensorRoll, rollSetPoint, rollError, rollOutput, sensorPitch, pitchSetPoint, pitchError, pitchOutput);
}

// Map values from a source domain to a destination domain
float map(int value, int s_low, int s_high, float d_low, float d_high) {
  return d_low + (d_high - d_low) * ((float)value - (float)s_low) / ((float)s_high - (float)s_low);
}
int map(int value, int s_low, int s_high, int d_low, int d_high) {
  return d_low + (d_high - d_low) * (value - s_low) / (s_high - s_low);
}

