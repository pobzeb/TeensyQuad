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

#define CALIBRATION_SAMPLE_SIZE 500

// Heartbeat timeout in milliseconds
#define HEARTBEAT_TIMEOUT 500

#define LOOP_DT 0.004F // Loop rate delta in ms.

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

#define ALPHA 0.8F  // Low pass constant.
#define GYRO_TRUST 0.98F // Percent Trust for gyro.

#define ROLL_PID_KP   1.700000F
#define ROLL_PID_KI   0.012000F
#define ROLL_PID_KD  11.000000F
#define ROLL_PID_MAX  400

#define PITCH_PID_KP  1.700000F
#define PITCH_PID_KI  0.012000F
#define PITCH_PID_KD 11.000000F
#define PITCH_PID_MAX 400

#define YAW_PID_KP    3.000000F
#define YAW_PID_KI    0.020000F
#define YAW_PID_KD    0.000000F
#define YAW_PID_MAX   400

