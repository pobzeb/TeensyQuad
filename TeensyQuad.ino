#include <Wire.h>
#include <L3G.h>
#include <ADXL345.h>
#include <HMC5883L.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include "Config.h"

// Radio input values.
HardwareSerial *radio;
int bytesAvailable;
unsigned long heartbeatPulse, heartbeatTimer, current_time;

// Servo for each motor and motor value
Servo m1, m2, m3, m4;
int m1_val, m2_val, m3_val, m4_val;

// Receiver channels
struct RC_Channel {
  int channel[CHANNELS];
};
struct RC_Channel rc;

// PID controllers for all three axis
float rollErrSum,   rollKp,   rollKi,   rollKd,   lastRollError;
float pitchErrSum,  pitchKp,  pitchKi,  pitchKd,  lastPitchError;
float yawErrSum,    yawKp,    yawKi,    yawKd,    lastYawError;
float rollSetPoint = 0, pitchSetPoint = 0, yawSetPoint = 0;
float throttle = 0, rollOutput = 0, pitchOutput = 0, yawOutput = 0;

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
unsigned long loopTimer, loopTime = 0;

void setup() {
  radio = &Serial1;
  Serial.begin(9600);
  radio->begin(RADIO_SERIAL_BAUD_RATE);

  Serial.println(F("Beginning Setup"));
  pinMode(LED, OUTPUT);
  while(radio->read() != -1);

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

  // Now LED off
  delay(200);
  flashLED(6);
  digitalWrite(LED, LOW);
  Serial.println(F("Setup Complete"));
  delay(200);

  // Set the timer.
  loopTimer = millis();
}

// Function to quickly flash led
void flashLED(int times) {
  for (int i = 1; i <= times; i++) {
    digitalWrite(LED, HIGH);
    delay(90);
    digitalWrite(LED, LOW);
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
  resetPID();

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
  digitalWrite(LED, LOW);

  // Calibrate and get a baseline setpoint for all axis
  unsigned long calibrationTimer = millis();
  int calibrationSteps = CALIBRATION_SAMPLE_SIZE;
  Serial.print(F("0\%"));
  for (calIdx = 0; calIdx < calibrationSteps; calIdx++) {
    // Blink the LED
    if (calIdx % 15 == 0) digitalWrite(LED, !digitalRead(LED));
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

    while (micros() - calibrationTimer < 4000);
    calibrationTimer = micros();
  }

  // Get an average
  rollCalibration  /= calibrationSteps;
  pitchCalibration /= calibrationSteps;
  yawCalibration   /= calibrationSteps;

  Serial.print(F("...Done\n"));
  Serial.println(F("Calibration complete."));
  digitalWrite(LED, LOW);
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

  // Read the radio input.
  readRadio();

  // Check to see if the motors should be armed or not.
  if (rc.channel[ARM_CHANNEL] == 2000 && flightState != STATE_RUNNING) {
    flightState = STATE_RUNNING;
    resetPID();
  }
  else if (rc.channel[ARM_CHANNEL] == 1000) {
    flightState = STATE_MOTORS_INITIALIZED;
  }

  // Roll Setpoint for PID.
  rollSetPoint = 0;
  if (rc.channel[ROLL_CHANNEL] > 1508) rollSetPoint = (rc.channel[ROLL_CHANNEL] - 1508) / 3.0;
  else if (rc.channel[ROLL_CHANNEL] < 1492) rollSetPoint = (rc.channel[ROLL_CHANNEL] - 1492) / 3.0;

  // Pitch Setpoint for PID.
  pitchSetPoint = 0;
  if (rc.channel[PITCH_CHANNEL] > 1508) pitchSetPoint = (rc.channel[PITCH_CHANNEL] - 1508) / 3.0;
  else if (rc.channel[PITCH_CHANNEL] < 1492) pitchSetPoint = (rc.channel[PITCH_CHANNEL] - 1492) / 3.0;

  // Yaw Setpoint for PID.
  yawSetPoint = 0;
  if (rc.channel[THROTTLE_CHANNEL] > 1050) {
    if (rc.channel[YAW_CHANNEL] > 1508) yawSetPoint = (rc.channel[YAW_CHANNEL] - 1508) / 3.0;
    else if (rc.channel[YAW_CHANNEL] < 1492) yawSetPoint = (rc.channel[YAW_CHANNEL] - 1492) / 3.0;
  }

  // Check the heartbeat pulse is greater than the timeout
  if (heartbeatPulse > HEARTBEAT_TIMEOUT) {
    // No heartbeat received so kill the motors
    flightState = STATE_WAITING_FOR_INIT;
  }

  // Set the throttle.
  throttle = rc.channel[THROTTLE_CHANNEL];

  if (flightState == STATE_RUNNING) {
    // Leave padding for pid values.
    if (throttle > 1800) throttle = 1800;

    // Don't apply PID values until throttle is running.
    if (throttle > 1200) {
      // Calculate the pid motor values
      calculatePID();
    }
    else {
      resetPID();
      if (throttle < 1050) {
        pitchOutput = rollOutput = yawOutput = 0;
      }
    }

    // Mix in the deltas and combine with throttle
    m1_val = throttle - pitchOutput - rollOutput - yawOutput;
    m2_val = throttle + pitchOutput - rollOutput + yawOutput;
    m3_val = throttle + pitchOutput + rollOutput - yawOutput;
    m4_val = throttle - pitchOutput + rollOutput + yawOutput;

    // Don't drop too low.
    int mMin = MIN_MOTOR_VALUE;
    if (throttle > 1200) mMin = 1200;
    if (m1_val < mMin) m1_val = mMin;
    if (m2_val < mMin) m2_val = mMin;
    if (m3_val < mMin) m3_val = mMin;
    if (m4_val < mMin) m4_val = mMin;

    // Don't excede max.
    if (m1_val > MAX_MOTOR_VALUE) m1_val = MAX_MOTOR_VALUE;
    if (m2_val > MAX_MOTOR_VALUE) m2_val = MAX_MOTOR_VALUE;
    if (m3_val > MAX_MOTOR_VALUE) m3_val = MAX_MOTOR_VALUE;
    if (m4_val > MAX_MOTOR_VALUE) m4_val = MAX_MOTOR_VALUE;
  }
  else {
    // Set min values
    m1_val = MIN_MOTOR_VALUE;
    m2_val = MIN_MOTOR_VALUE;
    m3_val = MIN_MOTOR_VALUE;
    m4_val = MIN_MOTOR_VALUE;
  }

  // Wait at least 4ms.
  while ((loopTime = micros() - loopTimer) < 4000);
  loopTimer = micros();

  if (DEBUG == true) {
    // Print the results.
    printResults();
  }

  if (MOTORS == true) {
    // Write the current speed to each motor
    m1.writeMicroseconds(m1_val);
    m2.writeMicroseconds(m2_val);
    m3.writeMicroseconds(m3_val);
    m4.writeMicroseconds(m4_val);
  }
}

void resetPID() {
  // Reset PID calculations.
  rollErrSum = 0;
  lastRollError = 0;
  rollSetPoint = 0;
  pitchErrSum = 0;
  lastPitchError = 0;
  pitchSetPoint = 0;
  yawErrSum = 0;
  lastYawError = 0;
  yawSetPoint = 0;
}

byte blockingRead(int len) {
  while (radio->available() < len);
  return radio->read();
}

void readRadio() {
  heartbeatPulse = millis() - heartbeatTimer;
  bytesAvailable = radio->available();
  while(bytesAvailable > 0) {
    // Check for the start of a data frame.
    if (radio->read() == 0x00) {
      // Check for the second start frame.
      if (blockingRead(1) == 0x00) {
        // Get the number of channels to read.
        int count = blockingRead(1);

        // Make sure the count matches the number of channels.
        if (count == CHANNELS) {
          // Read the channel data.
          for (int i = 0; i < count; i++) {
            int value = blockingRead(2);
            value |= radio->read() << 8;
            rc.channel[i] = value;
          }

          // Set the heartbeat pulse.
          heartbeatTimer = millis();
        }
      }
    }

    bytesAvailable = radio->available();
  }
}

void printResults() {
  Serial.printf(F("Loop: %3d"), loopTime);
  Serial.printf(F("    Throttle: %4d"), throttle);
//  Serial.printf(F("    R: %s%4.4f    P: %s%4.4f    Y: %s%4.4f"), rollOutput < 0 ? "-" : "+", abs(rollOutput), pitchOutput < 0 ? "-" : "+", abs(pitchOutput), yawOutput < 0 ? "-" : "+", abs(yawOutput));
//  Serial.printf(F("    gRoll: %s%4.4f    gPitch: %s%4.4f    gYaw: %s%4.4f"), gyroRoll < 0 ? "-" : "+", abs(gyroRoll), gyroPitch < 0 ? "-" : "+", abs(gyroPitch), gyroYaw < 0 ? "-" : "+", abs(gyroYaw));
  // Serial.printf(F("    aRollOffset: %s%4.4f    aPitchOffset: %s%4.4f    aYawOffset: %s%4.4f"), accelRollCalibration < 0 ? "-" : "+", abs(accelRollCalibration), accelPitchCalibration < 0 ? "-" : "+", abs(accelPitchCalibration), accelYawCalibration < 0 ? "-" : "+", abs(accelYawCalibration));
  // Serial.printf(F("    aRoll: %s%4.4f    aPitch: %s%4.4f"), accelRoll < 0 ? "-" : "+", abs(accelRoll), accelPitch < 0 ? "-" : "+", abs(accelPitch));
//  Serial.printf(F("    sRoll: %s%4.4f    sPitch: %s%4.4f    sYaw: %s%4.4f"), (sensorRoll < 0 ? "-" : "+"), abs(sensorRoll), (sensorPitch < 0 ? "-" : "+"), abs(sensorPitch), (sensorYaw < 0 ? "-" : "+"), abs(sensorYaw));
  Serial.printf(F("    M1: %4d    M2: %4d    M3: %4d    M4: %4d"), m1_val, m2_val, m3_val, m4_val);
//  Serial.printf(F("    Roll: %s%4d    Pitch: %s%4d    Yaw: %s%4d"), rc.channel[ROLL_CHANNEL] < 0 ? "-" : "+", abs(rc.channel[ROLL_CHANNEL]), rc.channel[PITCH_CHANNEL] < 0 ? "-" : "+", abs(rc.channel[PITCH_CHANNEL]), rc.channel[YAW_CHANNEL] < 0 ? "-" : "+", abs(rc.channel[YAW_CHANNEL]));
  Serial.printf(F("    RollSp: %s%4.2f    PitchSp: %s%4.2f    YawSp: %s%4.2f"), rollSetPoint < 0 ? "-" : "+", abs(rollSetPoint), pitchSetPoint < 0 ? "-" : "+", abs(pitchSetPoint), yawSetPoint < 0 ? "-" : "+", abs(yawSetPoint));
  Serial.printf(F("    Motors Armed: %s"), flightState == STATE_RUNNING ? "True" : "False");
//  Serial.printf(F("    HeartbeatPulse: %d"), heartbeatPulse);
  Serial.printf(F("\n"));
}

void calculatePID() {
  // Roll PID
  float rollError = sensorRoll - rollSetPoint;
  rollErrSum += rollKi * rollError;
  if (rollErrSum > ROLL_PID_MAX) rollErrSum = ROLL_PID_MAX;
  if (rollErrSum < ROLL_PID_MAX * -1) rollErrSum = ROLL_PID_MAX * -1;

  rollOutput = (rollKp * rollError + rollErrSum + rollKd * (rollError - lastRollError));
  if (rollOutput > ROLL_PID_MAX) rollOutput = ROLL_PID_MAX;
  else if (rollOutput < ROLL_PID_MAX * -1) rollOutput = ROLL_PID_MAX * -1;
  lastRollError = rollError;

  // Pitch PID
  float pitchError = sensorPitch - pitchSetPoint;
  pitchErrSum += pitchKi * pitchError;
  if (pitchErrSum > PITCH_PID_MAX) pitchErrSum = PITCH_PID_MAX;
  if (pitchErrSum < PITCH_PID_MAX * -1) pitchErrSum = PITCH_PID_MAX * -1;

  pitchOutput = (pitchKp * pitchError + pitchErrSum + pitchKd * (pitchError - lastPitchError));
  if (pitchOutput > PITCH_PID_MAX) pitchOutput = PITCH_PID_MAX;
  else if (pitchOutput < PITCH_PID_MAX * -1) pitchOutput = PITCH_PID_MAX * -1;
  lastPitchError = pitchError;

  // Yaw PID
  float yawError = sensorYaw - yawSetPoint;
  yawErrSum += yawKi * yawError;
  if (yawErrSum > YAW_PID_MAX) yawErrSum = YAW_PID_MAX;
  if (yawErrSum < YAW_PID_MAX * -1) yawErrSum = YAW_PID_MAX * -1;

  yawOutput = (yawKp * yawError + yawErrSum + yawKd * (yawError - lastYawError));
  if (yawOutput > YAW_PID_MAX) yawOutput = YAW_PID_MAX;
  else if (yawOutput < YAW_PID_MAX * -1) yawOutput = YAW_PID_MAX * -1;
  lastYawError = yawError;
}

