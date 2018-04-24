//-------General Defs-------

#define RADIO_SERIAL_BAUD_RATE 115200

#define LED 13
#define CHANNELS 6
#define THROTTLE_CHANNEL 0
#define YAW_CHANNEL 1
#define PITCH_CHANNEL 2
#define ROLL_CHANNEL 3
#define TRIM_CHANNEL 4
#define ARM_CHANNEL 5

#define CALIBRATION_SAMPLE_SIZE 1000

// Heartbeat timeout in milliseconds
#define HEARTBEAT_TIMEOUT 500

//------Motor State Defs-----

#define STATE_WAITING_FOR_INIT    0
#define STATE_MOTORS_INITIALIZED  1
#define STATE_RUNNING             2

//---------Motor Defs--------

#define M1_PIN 23
#define M2_PIN 22
#define M3_PIN 21
#define M4_PIN 20

#define MIN_MOTOR_VALUE 1000
#define MAX_MOTOR_VALUE 2000

//----Sensor/PID Config-------

// If this value is true, only read the
// gyro sensor. Otherwise, run a combination
// filter on accel and gyro data.
#define GYRO_ONLY false
#define ABSOLUTE_ANGLE false

// +/-  250 dps =  8.75
// +/-  500 dps = 17.50
// +/- 2000 dps = 70.00
#define GYRO_SENSITIVITY 17.50 / 1000.0

// +/-  2 G =  3.9
// +/-  4 G =  7.8
// +/-  8 G = 15.6
// +/- 16 G = 31.2
#define ACCEL_SENSITIVITY 31.2 / 1000.0

// Calculate the loop rate and time delta
#define LOOP_DT     4000  // 4ms in microseconds
#define LOOP_RATE   1000000.0 / LOOP_DT // MHz or 4ms per loop

#define ALPHA       0.98F // Low pass constant.
#define GYRO_TRUST  0.95F // Percent Trust for gyro.
#define ACCEL_TRUST 0.95F // Percent Trust for accel.

// Controller Sensitivity.
#define CONTROLER_SENSITIVITY 3.0F

#define ROLL_PID_KP   0.000000F
#define ROLL_PID_KI   0.000000F
#define ROLL_PID_KD   0.100000F
//  #define ROLL_PID_KP   1.200000F
//  #define ROLL_PID_KI   0.020000F
//  #define ROLL_PID_KD   3.500000F
//  #define ROLL_PID_KP   1.030000F
//  #define ROLL_PID_KI   0.010000F
//  #define ROLL_PID_KD   0.400000F

#define PITCH_PID_KP  ROLL_PID_KP
#define PITCH_PID_KI  ROLL_PID_KI
#define PITCH_PID_KD  ROLL_PID_KD

#define YAW_PID_KP    4.000000F
#define YAW_PID_KI    0.020000F
#define YAW_PID_KD    0.000000F

#define ROLL_PID_MAX  400
#define PITCH_PID_MAX 400
#define YAW_PID_MAX   400

