//-------General Defs-------

#define DEBUG false

#define RADIO_SERIAL_BAUD_RATE 115200

#define LED 13
#define CALIBRATION_SAMPLE_SIZE 2000

// Heartbeat timeout in microseconds
#define HEARTBEAT_TIMEOUT 2500000

// Status transmit timeout in microseconds
#define STATUS_TX_TIMEOUT 50000

#define LOOP_DT 4.0F // Loop rate delta in ms.

//------Motor State Defs-----

#define STATE_WAITING_FOR_INIT    0
#define STATE_MOTORS_INITIALIZED  1
#define STATE_RUNNING             2

//---------Motor Defs--------

#define MOTORS true

#define M1_PIN 23
#define M2_PIN 22
#define M3_PIN 21
#define M4_PIN 20

#define MIN_MOTOR_VALUE 1010
#define MAX_MOTOR_VALUE 1860

//----Sensor/PID Config-------

#define SENSORS true

#define ALPHA 0.8F  // Low pass constant.
#define GYRO_TRUST 0.98F // Percent Trust for gyro.
#define STICK_SENSITIVITY 3.0F // Higher value means less sensitive roll, pitch and yaw controll.

//#define ROLL_PID_KP   0.600000F
//#define ROLL_PID_KI   0.000500F
//#define ROLL_PID_KD  10.000000F
#define ROLL_PID_KP   2.500000F
#define ROLL_PID_KI   0.075000F
#define ROLL_PID_KD  10.000000F
#define ROLL_PID_MAX  400

//#define PITCH_PID_KP  0.300000F
//#define PITCH_PID_KI  0.000500F
//#define PITCH_PID_KD 10.000000F
#define PITCH_PID_KP  0.000000F
#define PITCH_PID_KI  0.000000F
#define PITCH_PID_KD  0.000000F
#define PITCH_PID_MAX 400

//#define YAW_PID_KP    6.600000F
//#define YAW_PID_KI    0.000000F
//#define YAW_PID_KD   15.000000F
#define YAW_PID_KP    0.000000F
#define YAW_PID_KI    0.000000F
#define YAW_PID_KD    0.000000F
#define YAW_PID_MAX   400
