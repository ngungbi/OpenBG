#define EEPROM_VERSION 1

#define USE_GYRO
#define USE_ACCEL

#define USE_FLOAT
//#undef  USE_FLOAT

#define IMU_FRONT X_pos
#define IMU_LEFT  Y_neg

#define rollKp 1400
#define rollKi 10
#define rollKd 5
#define rollMotorPWM 150
#define reverseRoll 0

#define pitchKp 1100
#define pitchKi 10
#define pitchKd 5
#define pitchMotorPWM 100
#define reversePitch 0

#define rollMotor  1
#define pitchMotor 0

#define RC_PITCH CH1
//#define RC_ROLL  CH0

#define PITCH_MIN -30
#define PITCH_MAX 90

#define DT_FLOAT 0.001
#define DT_ms    1
#define Fs       1000