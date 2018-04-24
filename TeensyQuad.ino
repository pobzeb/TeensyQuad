#include <Wire.h>
//#include <EEPROM.h>

#include "Config.h"

#include <L3G.h>
#include <ADXL345.h>
#include <HMC5883L.h>
//#include <Adafruit_BMP085.h>
#include <Servo.h>

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
int rollSetPoint = 0, pitchSetPoint = 0, yawSetPoint = 0;
int throttle = 0, rollOutput = 0, pitchOutput = 0, yawOutput = 0;
float pidVal = 0.0;

// Hold sensor values and calibration
int calIdx = 0;
boolean calibrated = false;
float rollCalibration, pitchCalibration, yawCalibration;

L3G gyro;
double fXg, fYg, fZg;
double gyroRoll, gyroPitch, gyroYaw;

ADXL345 accel;
float accelX, accelY, accelZ;
float accelRoll, accelPitch;

float angleRoll, anglePitch;
float rollLevelAdjust, pitchLevelAdjust;
float sensorRoll, sensorPitch, sensorYaw;

//HMC5883L compass;
//Adafruit_BMP085 barometer;
//float heading, altitude, temperature;
//int pressure;

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

  // Initialize the sensors
  flashLED(2);
  delay(200);
  Serial.println(F("Sensor Setup"));
  Sensors_init();

  // Initialize the motors
  flashLED(2);
  delay(200);
  Serial.println(F("Motor Setup"));
  Motors_init();

  // Now LED off
  delay(200);
  flashLED(6);
  digitalWrite(LED, LOW);
  Serial.println(F("Setup Complete"));
  delay(200);

  // Set the timer.
  loopTimer = micros();
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
  // Set:
  //  10 - 400Hz Data Rate (00 - 100Hz, 01 - 200Hz, 10 - 400Hz, 11 - 800Hz)
  //  10 -  50Hz Bandwidth
  //   1 - Normal Mode (not power down mode)
  // 111 - Z, Y and X axis enabled
  gyro.writeReg(L3G::CTRL_REG1, 0b10101111); // 0b10101111
  // Using +/-  250 dps full scale = 1000
  // Using +/-  500 dps full scale = 1001
  // Using +/- 2000 dps full scale = 1010
  gyro.writeReg(L3G::CTRL_REG4, 0b10010000); // 0b10010000
  Serial.println(F("Gyro initialized"));
  delay(250);

  Serial.println(F("Finding Accel..."));
  if (!accel.begin()) {
    Serial.println(F("Failed to detect accelerometer!"));
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16G);
  accel.setDataRate(ADXL345_DATARATE_400HZ);
  Serial.println(F("Accel initialized"));
  delay(250);

  /*
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
  */

  /*
  Serial.println(F("Finding Barometer..."));
  if (!barometer.begin()) {
    Serial.println(F("Failed to detect barometer!"));
    while(1);
  }
  Serial.println(F("Barometer initialized"));
  delay(250);
  */

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
  anglePitch = 0;
  angleRoll  = 0;
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
    readGyro();

    // Accumulate
    rollCalibration  += gyroRoll;
    pitchCalibration += gyroPitch;
    yawCalibration   += gyroYaw;

    delay(3);
  }

  // Get an average
  rollCalibration  /= calibrationSteps;
  pitchCalibration /= calibrationSteps;
  yawCalibration   /= calibrationSteps;

  Serial.print(F("...Done\n"));
  Serial.println(F("Calibration complete."));
  calibrated = true;
  digitalWrite(LED, LOW);
}

void readSensors() {
  // Read gyro and accel.
  readGyro();
  readAccel();

  // Integrate the angle
  sensorRoll  += gyroRoll  / LOOP_RATE;
  sensorPitch += gyroPitch / LOOP_RATE;

  // Compensate for tilt on both axis.
  sensorRoll  -= sensorPitch * sin((gyroYaw / LOOP_RATE) * (M_PI / 180.0));
  sensorPitch += sensorRoll  * sin((gyroYaw / LOOP_RATE) * (M_PI / 180.0));

  // Convert accel data to roll and pitch.
  accelRoll  = (atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0) / M_PI;
  accelPitch = (atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0) / M_PI;

  // Complementary filter to combine gyro and accelerometer.
  sensorRoll  = sensorRoll  * ALPHA + accelRoll  * (1.0 - ALPHA);
  sensorPitch = sensorPitch * ALPHA + accelPitch * (1.0 - ALPHA);
  sensorYaw   = gyroYaw;

  rollLevelAdjust = sensorRoll * 15;
  pitchLevelAdjust = sensorPitch * 15;
}

void readGyro() {
  // Get the gyro values and convert to dps.
  gyro.read();
  float fGX = gyro.g.x * -1.0;
  float fGY = gyro.g.y * -1.0;
  float fGZ = gyro.g.z * -1.0;

  // Adjust by calibration offsets.
  if (calIdx == CALIBRATION_SAMPLE_SIZE) {
    fGX -= rollCalibration;
    fGY -= pitchCalibration;
    fGZ -= yawCalibration;

    // Filter the values.
    gyroRoll  = (gyroRoll  * GYRO_TRUST) + (fGX * GYRO_SENSITIVITY * (1.0 - GYRO_TRUST));
    gyroPitch = (gyroPitch * GYRO_TRUST) + (fGY * GYRO_SENSITIVITY * (1.0 - GYRO_TRUST));
    gyroYaw   = (gyroYaw   * GYRO_TRUST) + (fGZ * GYRO_SENSITIVITY * (1.0 - GYRO_TRUST));
  }
  else {
    gyroRoll  = fGX;
    gyroPitch = fGY;
    gyroYaw   = fGZ;
  }
}

void readAccel() {
  // Get the accel values and scale them.
  Vector norm = accel.readNormalize(ADXL345_GRAVITY_EARTH);
  float fAX = norm.XAxis * ACCEL_SENSITIVITY * -1.0;
  float fAY = norm.YAxis * ACCEL_SENSITIVITY * -1.0;
  float fAZ = norm.ZAxis * ACCEL_SENSITIVITY;

  // Filter the values.
  accelX = (accelX * ACCEL_TRUST) + (fAX * (1.0 - ACCEL_TRUST));
  accelY = (accelY * ACCEL_TRUST) + (fAY * (1.0 - ACCEL_TRUST));
  accelZ = (accelZ * ACCEL_TRUST) + (fAZ * (1.0 - ACCEL_TRUST));
}

/*
void readCompass() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  heading = atan2(scaled.YAxis, scaled.XAxis);

  // Correct for when signs are reversed.
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;

  heading *= RAD_TO_DEG;
}
*/

/*
void readAltitude() {
  // Get altitude in meters.
  altitude = (altitude * 0.98f) + (barometer.readAltitude() * 0.02f);
  // Get temperature in deg F.
  // temperature = (barometer.readTemperature() * 1.8f) + 32.0f;
  // Get pressure in Pa.
  // pressure = (pressure * 0.98f) + (barometer.readPressure() * 0.02f);
}
*/

void Motors_init() {
  // Attach the motors
  m1.attach(M1_PIN, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE);
  m2.attach(M2_PIN, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE);
  m3.attach(M3_PIN, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE);
  m4.attach(M4_PIN, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE);

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
  readSensors();

  // Read the radio input.
  readRadio();
  pidVal = rc.channel[TRIM_CHANNEL] * 0.2;  // P (Take dial value and reduce by 50%)
  //pidVal = rc.channel[TRIM_CHANNEL] * 0.01; // I (Take dial value and reduce by 50%)
  //pidVal = rc.channel[TRIM_CHANNEL] * 0.5;  // D (Take dial value and reduce by 50%)

  // Set the throttle.
  throttle = rc.channel[THROTTLE_CHANNEL];

  // Check to see if the motors should be armed or not.
  if (rc.channel[ARM_CHANNEL] == MAX_MOTOR_VALUE && flightState != STATE_RUNNING) {
    flightState = STATE_RUNNING;
    sensorPitch = accelPitch;
    sensorRoll  = accelRoll;
    resetPID();
  }
  else if (rc.channel[ARM_CHANNEL] == MIN_MOTOR_VALUE) {
    flightState = STATE_MOTORS_INITIALIZED;
    if (!calibrated) doCalibration();
  }

  // Roll Setpoint for PID.
  rollSetPoint = 0;
  if (rc.channel[ROLL_CHANNEL] > 1508) rollSetPoint = (rc.channel[ROLL_CHANNEL] - 1508);
  else if (rc.channel[ROLL_CHANNEL] < 1492) rollSetPoint = (rc.channel[ROLL_CHANNEL] - 1492);
  rollSetPoint -= rollLevelAdjust;
  rollSetPoint /= CONTROLER_SENSITIVITY;

  // Pitch Setpoint for PID.
  pitchSetPoint = 0;
  if (rc.channel[PITCH_CHANNEL] > 1508) pitchSetPoint = (rc.channel[PITCH_CHANNEL] - 1508);
  else if (rc.channel[PITCH_CHANNEL] < 1492) pitchSetPoint = (rc.channel[PITCH_CHANNEL] - 1492);
  pitchSetPoint -= pitchLevelAdjust;
  pitchSetPoint /= CONTROLER_SENSITIVITY;

  // Yaw Setpoint for PID.
  yawSetPoint = 0;
  if (throttle > 1050) {
    if (rc.channel[YAW_CHANNEL] > 1508) yawSetPoint = (rc.channel[YAW_CHANNEL] - 1508) / CONTROLER_SENSITIVITY;
    else if (rc.channel[YAW_CHANNEL] < 1492) yawSetPoint = (rc.channel[YAW_CHANNEL] - 1492) / CONTROLER_SENSITIVITY;
  }

  // Check the heartbeat pulse is greater than the timeout
  if (heartbeatPulse > HEARTBEAT_TIMEOUT) {
    // No heartbeat received so kill the motors
    flightState = STATE_WAITING_FOR_INIT;
    heartbeatPulse = HEARTBEAT_TIMEOUT + 1;
  }

  if (flightState == STATE_RUNNING) {
    // Leave padding for pid values.
    if (throttle > 1800) throttle = 1800;

    // Don't apply PID values until throttle is running.
    if (throttle > 1080) {
      // Calculate the pid motor values
      calculatePID();
      calibrated = false;
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
  while ((loopTime = micros() - loopTimer) < LOOP_DT);
  loopTimer = micros();

  // Print the results.
  printResults();

  // Write the current speed to each motor
  m1.writeMicroseconds(m1_val);
  m2.writeMicroseconds(m2_val);
  m3.writeMicroseconds(m3_val);
  m4.writeMicroseconds(m4_val);
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
  // Serial.printf(F("    Kp: %4.4f    Ki: %4.4f    Kd: %4.4f"), rollKp , rollKi, rollKd);
  // Serial.printf(F("    R: %s%4.4f    P: %s%4.4f    Y: %s%4.4f"), rollOutput < 0 ? "-" : "+", abs(rollOutput), pitchOutput < 0 ? "-" : "+", abs(pitchOutput), yawOutput < 0 ? "-" : "+", abs(yawOutput));
  // Serial.printf(F("    aRollOffset: %s%4.4f    aPitchOffset: %s%4.4f    aYawOffset: %s%4.4f"), accelRollCalibration < 0 ? "-" : "+", abs(accelRollCalibration), accelPitchCalibration < 0 ? "-" : "+", abs(accelPitchCalibration), accelYawCalibration < 0 ? "-" : "+", abs(accelYawCalibration));
  //Serial.printf(F("    aRoll: %s%4.4f    aPitch: %s%4.4f"), accelRoll < 0 ? "-" : "+", abs(accelRoll), accelPitch < 0 ? "-" : "+", abs(accelPitch));
  //Serial.printf(F("    gRoll: %s%4.4f    gPitch: %s%4.4f    gYaw: %s%4.4f"), (gyroRoll < 0 ? "-" : "+"), abs(gyroRoll), (gyroPitch < 0 ? "-" : "+"), abs(gyroPitch), (gyroYaw < 0 ? "-" : "+"), abs(gyroYaw));
  // Serial.printf(F("    aglRoll: %s%4.4f    aglPitch: %s%4.4f"), (angleRoll < 0 ? "-" : "+"), abs(angleRoll), (anglePitch < 0 ? "-" : "+"), abs(anglePitch));
  Serial.printf(F("    Roll: %s%4.4f    Pitch: %s%4.4f    Yaw: %s%4.4f"), (sensorRoll < 0 ? "-" : "+"), abs(sensorRoll), (sensorPitch < 0 ? "-" : "+"), abs(sensorPitch), (sensorYaw < 0 ? "-" : "+"), abs(sensorYaw));
  // Serial.printf(F("    ROffset: %s%4.4f    POffset: %s%4.4f    YOffset: %s%4.4f"), (rollCalibration < 0 ? "-" : "+"), abs(rollCalibration), (pitchCalibration < 0 ? "-" : "+"), abs(pitchCalibration), (yawCalibration < 0 ? "-" : "+"), abs(yawCalibration));
  Serial.printf(F("    M1: %4d    M2: %4d    M3: %4d    M4: %4d"), m1_val, m2_val, m3_val, m4_val);
  // Serial.printf(F("    Roll: %s%4d    Pitch: %s%4d    Yaw: %s%4d"), rc.channel[ROLL_CHANNEL] < 0 ? "-" : "+", abs(rc.channel[ROLL_CHANNEL]), rc.channel[PITCH_CHANNEL] < 0 ? "-" : "+", abs(rc.channel[PITCH_CHANNEL]), rc.channel[YAW_CHANNEL] < 0 ? "-" : "+", abs(rc.channel[YAW_CHANNEL]));
  // Serial.printf(F("    RollSp: %s%4d    PitchSp: %s%4d    YawSp: %s%4d"), rollSetPoint < 0 ? "-" : "+", abs(rollSetPoint), pitchSetPoint < 0 ? "-" : "+", abs(pitchSetPoint), yawSetPoint < 0 ? "-" : "+", abs(yawSetPoint));
  // Serial.printf(F("    RollErr: %s%4.2f    PitchErr: %s%4.2f    YawErr: %s%4.2f"), lastRollError < 0 ? "-" : "+", abs(lastRollError), lastPitchError < 0 ? "-" : "+", abs(lastPitchError), lastYawError < 0 ? "-" : "+", abs(lastYawError));
  Serial.printf(F("    Motors Armed: %s"), flightState == STATE_RUNNING ? "True" : "False");
  // Serial.printf(F("    HeartbeatPulse: %d"), heartbeatPulse);
  // Serial.printf(F("    Loop Rate: %4.2f"), LOOP_RATE);
  // Serial.printf(F("    Loop DT: %4.2f"), LOOP_DT);
  Serial.printf(F("\n"));
}

void calculatePID() {
  // Roll PID
  rollKp = pitchKp = pidVal;
  //rollKi = pitchKi = pidVal;
  //rollKd = pitchKd = pidVal;
  float rollError = gyroRoll - rollSetPoint;
  rollErrSum += rollKi * rollError;
  if (rollErrSum > ROLL_PID_MAX) rollErrSum = ROLL_PID_MAX;
  if (rollErrSum < ROLL_PID_MAX * -1) rollErrSum = ROLL_PID_MAX * -1;

  rollOutput = (rollKp * rollError + rollErrSum + rollKd * (rollError - lastRollError));
  if (rollOutput > ROLL_PID_MAX) rollOutput = ROLL_PID_MAX;
  else if (rollOutput < ROLL_PID_MAX * -1) rollOutput = ROLL_PID_MAX * -1;
  lastRollError = rollError;

  // Pitch PID
  float pitchError = gyroPitch - pitchSetPoint;
  pitchErrSum += pitchKi * pitchError;
  if (pitchErrSum > PITCH_PID_MAX) pitchErrSum = PITCH_PID_MAX;
  if (pitchErrSum < PITCH_PID_MAX * -1) pitchErrSum = PITCH_PID_MAX * -1;

  pitchOutput = (pitchKp * pitchError + pitchErrSum + pitchKd * (pitchError - lastPitchError));
  if (pitchOutput > PITCH_PID_MAX) pitchOutput = PITCH_PID_MAX;
  else if (pitchOutput < PITCH_PID_MAX * -1) pitchOutput = PITCH_PID_MAX * -1;
  lastPitchError = pitchError;

  // Yaw PID
  float yawError = gyroYaw - yawSetPoint;
  yawErrSum += yawKi * yawError;
  if (yawErrSum > YAW_PID_MAX) yawErrSum = YAW_PID_MAX;
  if (yawErrSum < YAW_PID_MAX * -1) yawErrSum = YAW_PID_MAX * -1;

  yawOutput = (yawKp * yawError + yawErrSum + yawKd * (yawError - lastYawError));
  if (yawOutput > YAW_PID_MAX) yawOutput = YAW_PID_MAX;
  else if (yawOutput < YAW_PID_MAX * -1) yawOutput = YAW_PID_MAX * -1;
  lastYawError = yawError;
}
