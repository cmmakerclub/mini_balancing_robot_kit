#include <SPI.h>

#define slaveSelectPin     15
#define process_time_uS    18

#define none                      0x00
#define PD_Controller             0x02
#define Enable_Motor              0x04
#define PI_Steering               0x08
#define Car_mode                  0x10
#define acc_calibation_mode       0x20
#define gyro_calibation_mode      0x40
#define Manual_Motor_control      0x80

typedef struct __attribute((__packed__))Control_data {

  uint16_t mode;  // BIT define 7:   6:    5:    4:Car   3:ON/OFF PI Steering Controller    2:ON/OFF Motors     1:ON/OFF PD Angle Controller    0:Set Calibate Sensor Offset

  uint16_t AreRobotStanding;

  int16_t Pitch_Ref, Steering_Ref;

  int16_t Power_MotorL, Power_MotorR;

  int16_t roll, pitch, yaw;

  int16_t GyroX, GyroY, GyroZ;

  int16_t AccX, AccY, AccZ;

  int16_t OffsetGyroX, OffsetGyroY, OffsetGyroZ;

  int16_t OffsetAccX, OffsetAccY, OffsetAccZ;

  uint16_t VR1, VR2, VR3, VR4;

  int32_t Position_motorL, Position_motorR;

} Control_data;

extern Control_data State_data;

float Smooth_filter(float alfa, float new_data, float prev_data)
{
  float output = prev_data + (alfa * (new_data - prev_data));
  return output;
}
float limit(float x, float lower_b, float upper_b)
{
  if (x < lower_b) x = lower_b;
  if (x > upper_b) x = upper_b;
  return x;
}

void Init(void) {
  pinMode(slaveSelectPin, OUTPUT);
  // initialize SPI:
  SPI.begin();
  SPI.setFrequency(8000000);
  delay(1000);

  delayMicroseconds(process_time_uS);
  SPI.transfer(0);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0);
}

void Set_Mode(uint16_t tmp) {
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x01);
  delayMicroseconds(process_time_uS);
  SPI.transfer(tmp);
  SPI.transfer(tmp >> 8);
  digitalWrite(slaveSelectPin, HIGH);
}


void Set_Control(int16_t *tmp) {
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x02);
  delayMicroseconds(process_time_uS);
  SPI.transfer(tmp[0]); // Pitch_Ref
  SPI.transfer(tmp[0] >> 8); // Pitch_Ref

  SPI.transfer(tmp[1]); // Steering_Ref
  SPI.transfer(tmp[1] >> 8); // Steering_Ref
  digitalWrite(slaveSelectPin, HIGH);
}

void Get_Angles(int16_t *tmp) {
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x03);
  delayMicroseconds(process_time_uS);
  tmp[0] = (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // roll
  tmp[1] = (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // pitch
  tmp[2] = (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // yaw
  digitalWrite(slaveSelectPin, HIGH);
}

void Get_AngularVelocity(int16_t *tmp) {
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x04);
  delayMicroseconds(process_time_uS);
  tmp[0] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // GyroX
  tmp[1] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // GyroY
  tmp[2] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // GyroZ
  digitalWrite(slaveSelectPin, HIGH);
}

void Get_OffsetAngularVelocity(int16_t *tmp) {
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x05);
  delayMicroseconds(process_time_uS);
  tmp[0] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // OffsetGyroX
  tmp[1] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // OffsetGyroY
  tmp[2] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // OffsetGyroZ
  digitalWrite(slaveSelectPin, HIGH);
}

void Get_Accelerations(int16_t *tmp) {
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x06);
  delayMicroseconds(process_time_uS);
  tmp[0] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // AccX
  tmp[1] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // AccY
  tmp[2] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // AccZ
  digitalWrite(slaveSelectPin, HIGH);
}

void Get_OffsetAccelerations(int16_t *tmp) {
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x07);
  delayMicroseconds(process_time_uS);
  tmp[0] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // OffsetAccX
  tmp[1] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // OffsetAccY
  tmp[2] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // OffsetAccZ
  digitalWrite(slaveSelectPin, HIGH);
}

void Get_VRs(uint16_t *tmp) {
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x08);
  delayMicroseconds(process_time_uS);
  tmp[0] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // VR1
  tmp[1] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // VR2
  tmp[2] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // VR3
  tmp[3] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // VR4
  digitalWrite(slaveSelectPin, HIGH);
}

void Get_Positions(int32_t *tmp) {
  uint16_t* tmp1 = (uint16_t*)tmp;
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x09);
  delayMicroseconds(process_time_uS);
  tmp1[0] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // Position_motorL
  tmp1[1] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // Position_motorL
  tmp1[2] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // Position_motorR
  tmp1[3] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // Position_motorR
  digitalWrite(slaveSelectPin, HIGH);
}

void Set_P_Motors(int16_t* tmp) {

  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x0A);
  delayMicroseconds(process_time_uS);
  SPI.transfer(tmp[0]);
  SPI.transfer(tmp[0] >> 8);
  SPI.transfer(tmp[1]);
  SPI.transfer(tmp[1] >> 8);
  digitalWrite(slaveSelectPin, HIGH);
}
