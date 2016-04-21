#include <Wire.h>
#include <L3G.h>
#include <ADXL345.h>
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
float rollErrSum,   rollKp,   rollKi,   rollKd,   rollOutput,   lastRollError;
float pitchErrSum,  pitchKp,  pitchKi,  pitchKd,  pitchOutput,  lastPitchError;
float yawErrSum,    yawKp,    yawKi,    yawKd,    yawOutput,    lastYawError;
int rollSetPoint = 0, pitchSetPoint = 0, yawSetPoint = 0;

// Hold sensor values and calibration
int calIdx = 0;
double filterAlpha = ALPHA;
L3G gyro;
ADXL345 accel;
double gyroRollCalibration, gyroPitchCalibration, gyroYawCalibration;
double gyroRollRaw, gyroPitchRaw, gyroYawRaw;
double gyroRoll, gyroPitch, gyroYaw;
double accelRollCalibration, accelPitchCalibration, accelYawCalibration;
double accelRoll, accelPitch, accelYaw;
double sensorRoll, sensorPitch, sensorYaw;

// Current flight state and loop timer
int state = 0;
int flightState = STATE_WAITING_FOR_INIT;
unsigned long loopTimer;
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
  delay(200);
  digitalWrite(LED, LOW);
  Serial.println(F("Setup Complete"));
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

  if (!gyro.init()) {
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
  delay(250);

  if (!accel.begin()) {
    Serial.println(F("Failed to detect accelerometer!"));
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16G);
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

  // Calibrate and get a baseline setpoint for all axis
  calibrateGyro();
  calibrateAccel();
}

void calibrateGyro() {
  Serial.println(F("Calibrating Gyro"));

  // Reset PID calculations.
  rollErrSum = 0;
  lastRollError = 0;
  pitchErrSum = 0;
  lastPitchError = 0;
  yawErrSum = 0;
  lastYawError = 0;

  // Zero out the calibrated values
  gyroRollCalibration  = 0;
  gyroPitchCalibration = 0;
  gyroYawCalibration   = 0;
  digitalWrite(LED, LOW);

  // Begin calibration
  int calibrationSteps = CALIBRATION_SAMPLE_SIZE;
  for (calIdx = 0; calIdx < calibrationSteps; calIdx++) {
    // Blink the LED
    if (calIdx % 15 == 0) digitalWrite(LED, !digitalRead(LED));

    // Read the gyro values
    readGyro();

    // Accumulate
    gyroRollCalibration  += gyroRollRaw;
    gyroPitchCalibration += gyroPitchRaw;
    gyroYawCalibration   += gyroYawRaw;
    delay(4);
  }

  // Get an average
  gyroRollCalibration  /= calibrationSteps;
  gyroPitchCalibration /= calibrationSteps;
  gyroYawCalibration   /= calibrationSteps;

  digitalWrite(LED, LOW);
}

void readGyro() {
  gyro.read();

  gyroRollRaw = gyro.g.y;
  if (calIdx == CALIBRATION_SAMPLE_SIZE) gyroRollRaw -= gyroRollCalibration;

  gyroPitchRaw = gyro.g.x * -1;
  if (calIdx == CALIBRATION_SAMPLE_SIZE) gyroPitchRaw -= gyroPitchCalibration;

  gyroYawRaw = gyro.g.z * -1;
  if (calIdx == CALIBRATION_SAMPLE_SIZE) gyroYawRaw -= gyroYawCalibration;
}

void calibrateAccel() {
  Serial.println(F("Calibrating Accel"));

  // Reset PID calculations.
  rollErrSum = 0;
  lastRollError = 0;
  pitchErrSum = 0;
  lastPitchError = 0;
  yawErrSum = 0;
  lastYawError = 0;

  // Zero out the calibrated values
  accelRollCalibration  = 0;
  accelPitchCalibration = 0;
  accelYawCalibration   = 0;
  digitalWrite(LED, LOW);

  // Begin calibration
  int calibrationSteps = CALIBRATION_SAMPLE_SIZE;
  for (calIdx = 0; calIdx < calibrationSteps; calIdx++) {
    // Blink the LED
    if (calIdx % 15 == 0) digitalWrite(LED, !digitalRead(LED));

    // Read the accel values
    readAccel();

    // Accumulate
    accelRollCalibration  += accelRoll;
    accelPitchCalibration += accelPitch;
    accelYawCalibration   += accelYaw;
    delay(4);
  }

  // Get an average
  accelRollCalibration  /= calibrationSteps;
  accelPitchCalibration /= calibrationSteps;
  accelYawCalibration   /= calibrationSteps;

  digitalWrite(LED, LOW);
}

void readAccel() {
  Vector norm = accel.readNormalize();
  Vector filtered = accel.lowPassFilter(norm, 0.5);
  float xVal = filtered.XAxis, yVal = filtered.YAxis, zVal = filtered.ZAxis;

  // Adjust with calibrated values.
  if (calIdx == CALIBRATION_SAMPLE_SIZE) {
    xVal -= accelRollCalibration;
    yVal -= accelPitchCalibration;
    zVal -= accelYawCalibration;
  }

  accelRoll  = -(atan2(xVal, sqrt(yVal * yVal + zVal * zVal)) * 180.0) / M_PI;
  accelPitch =  (atan2(-yVal, zVal) * 180.0) / M_PI;
  accelYaw   =  0;
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
  readGyro();
  readAccel();
#endif

  // Convert to dps and add in 4 ms of rate.
  gyroRoll  += (gyroRollRaw  / 57.14286) * 0.004;
  gyroPitch += (gyroPitchRaw / 57.14286) * 0.004;
  gyroYaw   += (gyroYawRaw   / 57.14286) * 0.004;

  // Complementary filter to combine gyro and accelerometer.
  sensorRoll  = (gyroRoll  * GYRO_TRUST) + (accelRoll  * (1.0 - GYRO_TRUST));
  sensorPitch = (gyroPitch * GYRO_TRUST) + (accelPitch * (1.0 - GYRO_TRUST));
  sensorYaw   = gyroYaw;

  // Check to see if there are new values
  handleRadioInput();

  // Check the heartbeat
  if (micros() - heartbeatTimer > HEARTBEAT_TIMEOUT) {
    // No heartbeat received so kill the motors
    flightState = STATE_WAITING_FOR_INIT;
  }

  // Get the setpoints.
  rollSetPoint = 0;
  if (roll_input > 1508) rollSetPoint = (roll_input - 1508) / STICK_SENSITIVITY;
  else if (roll_input < 1492) rollSetPoint = (roll_input - 1492) / STICK_SENSITIVITY;
  pitchSetPoint = 0;
  if (pitch_input > 1508) pitchSetPoint = (pitch_input - 1508) / STICK_SENSITIVITY;
  else if (pitch_input < 1492) pitchSetPoint = (pitch_input - 1492) / STICK_SENSITIVITY;
  yawSetPoint = 0;
  if (throttle_input > 1050) {
    if (yaw_input > 1508) yawSetPoint = (yaw_input - 1508) / STICK_SENSITIVITY;
    else if (yaw_input < 1492) yawSetPoint = (yaw_input - 1492) / STICK_SENSITIVITY;
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

  // Wait 4 millis per loop.
  while (micros() - loopTimer < 4000);
  loopTimer = micros();

  // Print the results.
  printResults();

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
          printResults();

          // Reset for the next command
          state = 0;
          messageReady = false;
          b = 0;
        }
      }
      else if (state == 3) {
          heartbeatTimer = micros();
          int count = radio->read();
          Serial.println(F("HEARTBEAT"));

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
              yaw_input = (int)map(rInputs[b], 0, 255, 1000, 2000);
              break;
            }
            case 2: /* Pitch */ {
              pitch_input = (int)map(rInputs[b], 0, 255, 1000, 2000);
              break;
            }
            case 3: /* Roll */ {
              roll_input = (int)map(rInputs[b], 0, 255, 1000, 2000);
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
                calibrateGyro();
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
              break;
            }
          }
        }

        // Print the results.
        printResults();

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
  else if (currentPIDAxis == 'p') {
    Serial.printf(F(", Pitch PID - Kp: %4.6f, Ki: %4.6f, Kd: %4.6f"), pitchKp, pitchKi, pitchKd);
  }
  else if (currentPIDAxis == 'y') {
    Serial.printf(F(", Yaw PID - Kp: %4.6f, Ki: %4.6f, Kd: %4.6f"), yawKp, yawKi, yawKd);
  }
*/
//  Serial.printf(F("    R: %s%4.4f    P: %s%4.4f    Y: %s%4.4f"), rollOutput < 0 ? "-" : "+", abs(rollOutput), pitchOutput < 0 ? "-" : "+", abs(pitchOutput), yawOutput < 0 ? "-" : "+", abs(yawOutput));
//  Serial.printf(F("    gRoll: %s%4.4f    gPitch: %s%4.4f    gYaw: %s%4.4f"), gyroRoll < 0 ? "-" : "+", abs(gyroRoll), gyroPitch < 0 ? "-" : "+", abs(gyroPitch), gyroYaw < 0 ? "-" : "+", abs(gyroYaw));
//  Serial.printf(F("    aRoll: %s%4.4f    aPitch: %s%4.4f"), accelRoll < 0 ? "-" : "+", abs(accelRoll), accelPitch < 0 ? "-" : "+", abs(accelPitch));
  Serial.printf(F("    sRoll: %s%4.4f    sPitch: %s%4.4f    sYaw: %s%4.4f"), (sensorRoll < 0 ? "-" : "+"), abs(sensorRoll), (sensorPitch < 0 ? "-" : "+"), abs(sensorPitch), (sensorYaw < 0 ? "-" : "+"), abs(sensorYaw));
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
  float error_tmp;

  // Roll PID
  error_tmp = sensorRoll - rollSetPoint;
  rollErrSum += rollKi * error_tmp;
  if (rollErrSum > ROLL_PID_MAX) rollErrSum = ROLL_PID_MAX;
  else if (rollErrSum < ROLL_PID_MAX * -1) rollErrSum = ROLL_PID_MAX * -1;
  rollOutput = rollKp * error_tmp + rollErrSum + rollKd * (error_tmp - lastRollError);
  if (rollOutput > ROLL_PID_MAX) rollOutput = ROLL_PID_MAX;
  else if (rollOutput < ROLL_PID_MAX * -1) rollOutput = ROLL_PID_MAX * -1;
  lastRollError = error_tmp;

  // Pitch PID
  error_tmp = sensorPitch - pitchSetPoint;
  pitchErrSum += pitchKi * error_tmp;
  if (pitchErrSum > PITCH_PID_MAX) pitchErrSum = PITCH_PID_MAX;
  else if (pitchErrSum < PITCH_PID_MAX * -1) pitchErrSum = PITCH_PID_MAX * -1;
  pitchOutput = pitchKp * error_tmp + pitchErrSum + pitchKd * (error_tmp - lastPitchError);
  if (pitchOutput > PITCH_PID_MAX) pitchOutput = PITCH_PID_MAX;
  else if (pitchOutput < PITCH_PID_MAX * -1) pitchOutput = PITCH_PID_MAX * -1;
  lastPitchError = error_tmp;

  // Yaw PID
  error_tmp = sensorYaw - yawSetPoint;
  yawErrSum += yawKi * error_tmp;
  if (yawErrSum > YAW_PID_MAX) yawErrSum = YAW_PID_MAX;
  else if (yawErrSum < YAW_PID_MAX * -1) yawErrSum = YAW_PID_MAX * -1;
  yawOutput = yawKp * error_tmp + yawErrSum + yawKd * (error_tmp - lastYawError);
  if (yawOutput > YAW_PID_MAX) yawOutput = YAW_PID_MAX;
  else if (yawOutput < YAW_PID_MAX * -1) yawOutput = YAW_PID_MAX * -1;
  lastYawError = error_tmp;
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

