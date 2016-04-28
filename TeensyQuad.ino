#include <Wire.h>
#include <L3G.h>
#include <ADXL345.h>
#include <HMC5883L.h>
#include <Servo.h>
#include "Config.h"

// Servo for each motor and motor value
Servo m1, m2, m3, m4;
int m1_val, m2_val, m3_val, m4_val;
byte motorMode = 0b00001111;
int maxMotorDelta = 0;

// Radio input values.
HardwareSerial *radio;
int bytesAvailable;
int rInputs[7];
bool messageReady = false;
bool displayResults = false;
char rb = ' ';
int b = 0;
char currentPIDAxis = 'r';
char action = ' ';
float kValue = 0.0;
unsigned char* kValuePtr = (unsigned char*)&kValue;

// PID controllers for all three axis
float rollErrSum,   rollKp,   rollKi,   rollKd,   lastRollError;
float pitchErrSum,  pitchKp,  pitchKi,  pitchKd,  lastPitchError;
float yawErrSum,    yawKp,    yawKi,    yawKd,    lastYawError;
int rollOutput, pitchOutput, yawOutput;
int rollSetPoint = 0, pitchSetPoint = 0, yawSetPoint = 0;

// Hold sensor values and calibration
L3G gyro;
ADXL345 accel;
HMC5883L compass;
int calIdx = 0;
float rollCalibration, pitchCalibration, yawCalibration;
float gyroRollRaw, gyroPitchRaw, gyroYawRaw;
float accelRollRaw, accelPitchRaw, accelYawRaw;
float gyroRoll, gyroPitch, gyroYaw;
float accelRoll, accelPitch, accelYaw;
float heading;
float sensorRoll, sensorPitch, sensorYaw;

// Current flight state and loop timer
int state = 0;
int flightState = STATE_WAITING_FOR_INIT;
unsigned long loopTimer;
unsigned long statusTxTimer;
unsigned long heartbeatTimer;

// Hold input Throttle, roll, pitch and yaw
int throttle_input, roll_input, pitch_input, yaw_input;

void setup() {
  radio = &Serial1;
  Serial.begin(9600);
  radio->begin(RADIO_SERIAL_BAUD_RATE);

  // Zero out the radio buffer
  Serial.println(F("Beginning Setup"));
  pinMode(LED, OUTPUT);
  while(radio->read() != -1);

#ifdef SENSORS
  // Initialize the sensors
  flashLED(2);
  delay(200);
  Serial.println(F("Sensor Setup"));
  Sensors_init();
#endif

#ifdef MOTORS
  // Initialize the motors
  flashLED(2);
  delay(200);
  Serial.println(F("Motor Setup"));
  Motors_init();
#endif

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
  if(error != 0) Serial.println(compass.GetErrorText(error));
  error = compass.SetMeasurementMode(Measurement_Continuous);
  if(error != 0) Serial.println(compass.GetErrorText(error));
  Serial.println(F("Compass initialized"));
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
  digitalWrite(LED, LOW);

  // Calibrate and get a baseline setpoint for all axis
  unsigned long calibrationTimer = micros();
  int calibrationSteps = CALIBRATION_SAMPLE_SIZE;
  for (calIdx = 0; calIdx < calibrationSteps; calIdx++) {
    // Blink the LED
    if (calIdx % 15 == 0) digitalWrite(LED, !digitalRead(LED));

    // Read the sensor values
    readSensors();

    // Accumulate
    rollCalibration  += sensorRoll;
    pitchCalibration += sensorPitch;
    yawCalibration   += sensorYaw;

    while (micros() - calibrationTimer < (LOOP_DT * 1000));
    calibrationTimer = micros();
  }

  // Get an average
  rollCalibration  /= calibrationSteps;
  pitchCalibration /= calibrationSteps;
  yawCalibration   /= calibrationSteps;

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

  // Zero out the sensor values if they are under 0.5.
  if (abs(sensorRoll) < 0.5) sensorRoll = 0.0;
  if (abs(sensorPitch) < 0.5) sensorPitch = 0.0;
  if (abs(sensorYaw) < 0.5) sensorYaw = 0.0;
}

void readGyro() {
  gyro.read();

  gyroRollRaw = gyro.g.y;
  gyroPitchRaw = gyro.g.x * -1;
  gyroYawRaw = gyro.g.z * -1;

  // Convert to dps and calculate time delta.
  gyroRollRaw  = (gyroRollRaw  / 57.14286) * (LOOP_DT / 1000);
  gyroPitchRaw = (gyroPitchRaw / 57.14286) * (LOOP_DT / 1000);
  gyroYawRaw   = (gyroYawRaw   / 57.14286) * (LOOP_DT / 1000);

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
#ifdef SENSORS
  // Read the gyro and calculate the new values
  readSensors();
#endif

  // Check to see if there are new values
  handleRadioInput();

  // Check the heartbeat
  if (micros() - heartbeatTimer > HEARTBEAT_TIMEOUT) {
    // No heartbeat received so kill the motors
    flightState = STATE_WAITING_FOR_INIT;
  }

  // Get the setpoints.
//  rollSetPoint = 0;
//  if (roll_input > 1508) rollSetPoint = -1.0 * map((roll_input - 1508) / STICK_SENSITIVITY, 1500, 2000, 0, 90);
//  else if (roll_input < 1492) rollSetPoint = map((roll_input - 1492) / STICK_SENSITIVITY, 1000, 1500, 0, 90);
//  pitchSetPoint = 0;
//  if (pitch_input > 1508) pitchSetPoint = (pitch_input - 1508) / STICK_SENSITIVITY;
//  else if (pitch_input < 1492) pitchSetPoint = (pitch_input - 1492) / STICK_SENSITIVITY;
//  yawSetPoint = 0;
//  if (throttle_input > 1050) {
//    if (yaw_input > 1508) yawSetPoint = (yaw_input - 1508) / STICK_SENSITIVITY;
//    else if (yaw_input < 1492) yawSetPoint = (yaw_input - 1492) / STICK_SENSITIVITY;
//  }

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

  // Wait LOOP_DT millis per loop.
  while (micros() - loopTimer < (LOOP_DT * 1000));
  loopTimer = micros();

#ifdef DEBUG
  // Print the results.
//  printResults();
#endif

  // Send status over radio
  if (micros() - statusTxTimer > STATUS_TX_TIMEOUT) {
    // Print status to radio.
    radio->printf(F("T %4d r %s%4.4f p %s%4.4f y %s%4.4f armed: %s  PID - Kp: %4.6f, Ki: %4.6f, Kd: %4.6f\n"), throttle_input, (sensorRoll < 0 ? "-" : "+"), abs(sensorRoll), (sensorPitch < 0 ? "-" : "+"), abs(sensorPitch), (sensorYaw < 0 ? "-" : "+"), abs(sensorYaw), flightState == STATE_RUNNING ? "True" : "False", rollKp, rollKi, rollKd);
    statusTxTimer = micros();
  }

#ifdef MOTORS
  // Write the current speed to each motor
  m1.writeMicroseconds(m1_val);
  m2.writeMicroseconds(m2_val);
  m3.writeMicroseconds(m3_val);
  m4.writeMicroseconds(m4_val);
#endif
}

void handleRadioInput() {
  // Get the number of bytes available
  bytesAvailable = radio->available();

  // If there is data to read, read it
  if (bytesAvailable > 0) {
    digitalWrite(LED, HIGH);
    while (bytesAvailable > 0) {
      // Look for start of message character.
      if (state == 0) {
        b = 0;
        messageReady = false;
        displayResults = false;
        rb = (char)radio->read();

        // Check to see if this is the beginning of the message
        if (rb == 'm') {
          // Move to next state
          state = 1;
        }
        else if (rb == 'k') {
          // Move to keyboard input state
          state = 2;
        }
        else if (rb == 'h') {
          // Move to heartbeat state
          state = 3;
        }
        else {
          // Reset state
          state = 0;
        }
        continue;
      }
      // Read message bytes.
      else if (state == 1) {
        // Read the byte.
        rInputs[b] = radio->read();

        // Check to see if we found the start of another message instead.
        if ((char)rInputs[b] == 'm') {
          messageReady = false;
          b = 0;
          continue;
        }

        // Increment byte counter.
        b++;

        // Check to see if that was the end of the message.
        if (b == 7) messageReady = true;
      }
      else if (state == 2) {
        // Read the command and value
        if (b == 0) {
          action = (char) radio->read();
        }
        else {
          kValuePtr[b - 1] = radio->read();
        }

        // Increment byte counter.
        b++;

        if (b == 5) {
          Serial.printf(F("Command: %c, Value: %4.6f\n"), action, kValue);

          switch (action) {
            case 't': /* Throttle */ {
              throttle_input = (int)kValue;
              break;
            }
            case 'q': /* Roll Kp PID Setting */  {
              rollKp = kValue;
              currentPIDAxis = 'r';
              break;
            }
            case 'a': /* Roll Ki PID Setting */  {
              rollKi = kValue;
              currentPIDAxis = 'r';
              break;
            }
            case 'z': /* Roll Kd PID Setting */  {
              rollKd = kValue;
              currentPIDAxis = 'r';
              break;
            }
            case 'w': /* Pitch Kp PID Setting */  {
              pitchKp = kValue;
              currentPIDAxis = 'p';
              break;
            }
            case 's': /* Pitch Ki PID Setting */  {
              pitchKi = kValue;
              currentPIDAxis = 'p';
              break;
            }
            case 'x': /* Pitch Kd PID Setting */  {
              pitchKd = kValue;
              currentPIDAxis = 'p';
              break;
            }
            case 'e': /* Yaw Kp PID Setting */  {
              yawKp = kValue;
              currentPIDAxis = 'y';
              break;
            }
            case 'd': /* Yaw Ki PID Setting */  {
              yawKi = kValue;
              currentPIDAxis = 'y';
              break;
            }
            case 'c': /* Yaw Kd PID Setting */  {
              yawKd = kValue;
              currentPIDAxis = 'y';
              break;
            }
          }

          // Print the results.
//          printResults();

          // Reset for the next command
          state = 0;
          messageReady = false;
          b = 0;
        }
      }
      else if (state == 3) {
          heartbeatTimer = micros();
          radio->read();
          Serial.println(F("HEARTBEAT"));
//          printResults();

          // Reset for the next command
          state = 0;
          messageReady = false;
          b = 0;
      }

      // If the message is ready, parse the bytes.
      if (messageReady == true) {
        for (b = 0; b < 7; b++) {
          switch (b) {
            case 0: /* Throttle */ {
              throttle_input = (int)map(rInputs[b], 0, 255, 1000, 2000);
              break;
            }
            case 1: /* Yaw */ {
              yawSetPoint = (int)map(rInputs[b], 0, 255, -45, 45);
              break;
            }
            case 2: /* Pitch */ {
              pitchSetPoint = (int)map(rInputs[b], 0, 255, -45, 45);
              break;
            }
            case 3: /* Roll */ {
              rollSetPoint = (int)map(rInputs[b], 0, 255, -45, 45);
              break;
            }
            case 4: /* L_Trigger */ {
//              float value = map(rInputs[b], 0, 255, -1.0f, 1.0f);
              break;
            }
            case 5: /* R_Trigger */ {
//              float value = map(rInputs[b], 0, 255, -1.0f, 1.0f);
              break;
            }
            case 6: /* Buttons */ {
              if (rInputs[b] == 1) {
                // Run sensor calibration
                doCalibration();
              }
              if (rInputs[b] == 2) {
                // Arm the motors
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
                else flightState = STATE_MOTORS_INITIALIZED;
              }
              if (rInputs[b] == 4) {
                if (flightState != STATE_RUNNING) {
                  // Clear sensor values
                  sensorRoll = 0.0;
                  sensorPitch = 0.0;
                  sensorYaw = 0.0;
                  rollCalibration = 0.0;
                  pitchCalibration = 0.0;
                  yawCalibration = 0.0;
                }
              }
              break;
            }
          }
        }

        // Print the results.
//        printResults();

        // Reset state for next message
        state = 0;
      }

      bytesAvailable = radio->available();
    }
    digitalWrite(LED, LOW);
  }
}

void printResults() {
  Serial.printf(F("Throttle: %4d"), throttle_input);
/*
  if (currentPIDAxis == 'r') {
    Serial.printf(F(", Roll PID - Kp: %4.6f, Ki: %4.6f, Kd: %4.6f"), rollKp, rollKi, rollKd);
  }
    Serial.printf(F(", Pitch PID - Kp: %4.6f, Ki: %4.6f, Kd: %4.6f"), pitchKp, pitchKi, pitchKd);
  }
  else if (currentPIDAxis == 'y') {
    Serial.printf(F(", Yaw PID - Kp: %4.6f, Ki: %4.6f, Kd: %4.6f"), yawKp, yawKi, yawKd);
  }
*/
//  Serial.printf(F("    R: %s%4.4f    P: %s%4.4f    Y: %s%4.4f"), rollOutput < 0 ? "-" : "+", abs(rollOutput), pitchOutput < 0 ? "-" : "+", abs(pitchOutput), yawOutput < 0 ? "-" : "+", abs(yawOutput));
//  Serial.printf(F("    R: %s%4.4f    P: %s%4.4f    Y: %s%4.4f"), rollOutput < 0 ? "-" : "+", abs(rollOutput), pitchOutput < 0 ? "-" : "+", abs(pitchOutput), yawOutput < 0 ? "-" : "+", abs(yawOutput));
//  Serial.printf(F("    gRollOffset: %s%4.4f    gPitchOffset: %s%4.4f    gYawOffset: %s%4.4f"), gyroRollCalibration < 0 ? "-" : "+", abs(gyroRollCalibration), gyroPitchCalibration < 0 ? "-" : "+", abs(gyroPitchCalibration), gyroYawCalibration < 0 ? "-" : "+", abs(gyroYawCalibration));
//  Serial.printf(F("    gRoll: %s%4.4f    gPitch: %s%4.4f    gYaw: %s%4.4f"), gyroRoll < 0 ? "-" : "+", abs(gyroRoll), gyroPitch < 0 ? "-" : "+", abs(gyroPitch), gyroYaw < 0 ? "-" : "+", abs(gyroYaw));
//  Serial.printf(F("    aRollOffset: %s%4.4f    aPitchOffset: %s%4.4f    aYawOffset: %s%4.4f"), accelRollCalibration < 0 ? "-" : "+", abs(accelRollCalibration), accelPitchCalibration < 0 ? "-" : "+", abs(accelPitchCalibration), accelYawCalibration < 0 ? "-" : "+", abs(accelYawCalibration));
//  Serial.printf(F("    aRoll: %s%4.4f    aPitch: %s%4.4f"), accelRoll < 0 ? "-" : "+", abs(accelRoll), accelPitch < 0 ? "-" : "+", abs(accelPitch));
//  Serial.printf(F("    sRoll: %s%4.4f    sPitch: %s%4.4f    sYaw: %s%4.4f"), (sensorRoll < 0 ? "-" : "+"), abs(sensorRoll), (sensorPitch < 0 ? "-" : "+"), abs(sensorPitch), (sensorYaw < 0 ? "-" : "+"), abs(sensorYaw));
  Serial.printf(F("    M1: %4d    M2: %4d    M3: %4d    M4: %4d    MaxDelta: %d"), m1_val, m2_val, m3_val, m4_val, maxMotorDelta);
//  Serial.printf(F("    Roll: %s%4d    Pitch: %s%4d    Yaw: %s%4d"), rollSetPoint < 0 ? "-" : "+", abs(rollSetPoint), pitchSetPoint < 0 ? "-" : "+", abs(pitchSetPoint), yawSetPoint < 0 ? "-" : "+", abs(yawSetPoint));
  Serial.printf(F("    Motors Armed: %s"), flightState == STATE_RUNNING ? "True" : "False");
  Serial.printf(F("\n"));

/*
  unsigned char* resp;
  resp = (unsigned char*)calloc(7, sizeof(unsigned char));
  resp[0] = 'm';
  resp[1] = (int)map(throttle_input, 1000, 2000, 0, 255);
  resp[2] = (int)map(m1_val, 1000, 2000, 0, 255);
  resp[3] = (int)map(m2_val, 1000, 2000, 0, 255);
  resp[4] = (int)map(m3_val, 1000, 2000, 0, 255);
  resp[5] = (int)map(m4_val, 1000, 2000, 0, 255);
  resp[6] = flightState == STATE_RUNNING ? 1 : 0;
  radio->write(resp, 7);
*/
}

void calculatePID() {
  // Roll PID
  float rollError = sensorRoll - rollSetPoint;
  if (abs(rollError) < 0.5) rollError = 0.0;
  rollErrSum += rollKi * rollError;
  rollOutput = (rollKp * rollError + rollErrSum + rollKd * (rollError - lastRollError));
  if (rollOutput > 180.0) rollOutput = 180.0;
  if (rollOutput < -180.0) rollOutput = -180.0;
  rollOutput = map(rollOutput, -180.0, 180.0, -ROLL_PID_MAX, ROLL_PID_MAX);
  if (rollOutput > ROLL_PID_MAX) rollOutput = ROLL_PID_MAX;
  if (rollOutput < -ROLL_PID_MAX) rollOutput = -ROLL_PID_MAX;
  lastRollError = rollError;

  // Pitch PID
  float pitchError = sensorPitch - pitchSetPoint;
  if (abs(pitchError) < 0.5) pitchError = 0.0;
  pitchErrSum += pitchKi * pitchError;
  pitchOutput = (pitchKp * pitchError + pitchErrSum + pitchKd * (pitchError - lastPitchError));
  if (pitchOutput > 180.0) pitchOutput = 180.0;
  if (pitchOutput < -180.0) pitchOutput = -180.0;
  pitchOutput = map(pitchOutput, -180.0, 180.0, -PITCH_PID_MAX, PITCH_PID_MAX);
  if (pitchOutput > PITCH_PID_MAX) pitchOutput = PITCH_PID_MAX;
  if (pitchOutput < -PITCH_PID_MAX) pitchOutput = -PITCH_PID_MAX;
  lastPitchError = pitchError;

  // Yaw PID
  float error_tmp = sensorYaw - yawSetPoint;
  if (abs(yawError) < 0.5) yawError = 0.0;
  yawErrSum += yawKi * error_tmp;
  yawOutput = (yawKp * error_tmp + yawErrSum + yawKd * (error_tmp - lastYawError));
  if (yawOutput > 360.0) yawOutput = 360.0;
  if (yawOutput < -360.0) yawOutput = -360.0;
  yawOutput = map(yawOutput, -360.0, 360.0, -YAW_PID_MAX, YAW_PID_MAX);
  if (yawOutput > YAW_PID_MAX) yawOutput = YAW_PID_MAX;
  if (yawOutput < -YAW_PID_MAX) yawOutput = -YAW_PID_MAX;
  lastYawError = error_tmp;
  Serial.printf(F("R: %4.2f    RSP: %d    RE: %4.4f    RO: %d        P: %4.2f    PSP: %d    PE: %4.4f    PO: %d\n"), sensorRoll, rollSetPoint, rollError, rollOutput, sensorPitch, pitchSetPoint, pitchError, pitchOutput);
}

// Convert hex to int
long hex_To_Int(long Hex, char bits) {
  long hex_2_Int;
  char byte;
  hex_2_Int=0;

  for (byte = 0; byte < bits; byte++) {
     if (Hex&(0x0001 << byte)) hex_2_Int += 1 * (pow(2, byte));
      else hex_2_Int += 0 * (pow(2, byte));
  }

  return hex_2_Int;
}

// Map values from a source domain to a destination domain
float map(int value, int s_low, int s_high, float d_low, float d_high) {
  return d_low + (d_high - d_low) * ((float)value - (float)s_low) / ((float)s_high - (float)s_low);
}
int map(int value, int s_low, int s_high, int d_low, int d_high) {
  return d_low + (d_high - d_low) * (value - s_low) / (s_high - s_low);
}

