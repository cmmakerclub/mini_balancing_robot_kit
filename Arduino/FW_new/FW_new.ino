
#include "CMMC_Receiver.h"
#include <SPI.h>

#define slaveSelectPin     15
#define process_time_uS    18

#define none                      0x01
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

Control_data State_data;


byte AreRobotStand = 0;


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

void setup() {

  // set the slaveSelectPin as an output:
  Serial.begin(115200);

  receriver_init();

  pinMode(slaveSelectPin, OUTPUT);
  // initialize SPI:
  SPI.begin();
  SPI.setFrequency(8000000);
  delay(2000);

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

//  while (1) {
//    State_data.mode = gyro_calibation_mode | acc_calibation_mode;
//    Set_Mode(State_data.mode);
//    delay(10000);
//    Get_OffsetAngularVelocity(&State_data.OffsetGyroX);
//    Serial.print(State_data.OffsetGyroX);  Serial.print("\t");
//    Serial.print(State_data.OffsetGyroY);  Serial.print("\t");
//    Serial.print(State_data.OffsetGyroZ);  Serial.print("\t");
//
//    Get_OffsetAccelerations(&State_data.OffsetAccX);
//    Serial.print(State_data.OffsetAccX);  Serial.print("\t");
//    Serial.print(State_data.OffsetAccY);  Serial.print("\t");
//    Serial.print(State_data.OffsetAccZ);  Serial.println("\t");
//  }

  State_data.mode = PI_Steering | PD_Controller | Enable_Motor;
  Set_Mode(State_data.mode);
}
float Kp_Speed = 0;
float Ki_Speed = 0;

float error_speed;
float error_speed_sum;
float Robot_Target_Angle;
float Robot_speed;
uint32_t timenow, t1, t2, t3;
uint32_t p1, p2, p3;
void loop() {


  receriver_loop();

  timenow = millis();

  if (timenow - t1 >= 10) {  // 100Hz Speed reading
    t1 = timenow;

    static int32_t last_L, last_R;

    Get_Positions(&State_data.Position_motorL);

    float dL =  ((int32_t)State_data.Position_motorL - last_L);
    float dR =  ((int32_t)State_data.Position_motorR - last_R);

    last_L = (int32_t)State_data.Position_motorL;
    last_R = (int32_t)State_data.Position_motorR;

    Robot_speed  = Smooth_filter(0.05f, (dR + dL) * 0.5f, Robot_speed); // 1hz lowpass
  }

  if (timenow - t2 >= 50) { // 50Hz velocity control
    t2 = timenow;
    Get_Angles(&State_data.roll);

    float kp = (float)tuningData[1].kp * 0.005f;
    float ki = (float)tuningData[1].ki * 0.0005f;

    float P_limit = map (abs(Robot_speed), 0, 50, 250, 150);

    error_speed = limit(kp * (-(float)Get_ChannelValue(2) * 0.4f + Robot_speed), -P_limit, P_limit);

    float i_limit = map (abs(Robot_speed), 0, 50, 150, 0);

    error_speed_sum = limit((error_speed_sum + (error_speed * 0.2f)), -i_limit / ki, i_limit / ki);

    State_data.Pitch_Ref = limit(ki * error_speed_sum + error_speed, -350, 350); // angle ref Control

    State_data.Steering_Ref = (float)Get_ChannelValue(1) * (float)tuningData[0].kp * 0.04f; // L-R Control



    if ( abs(State_data.pitch) > 450 | abs(State_data.roll) > 450) AreRobotStand = 0;
    if ( abs(State_data.pitch) <  50 & abs(State_data.roll) < 450) AreRobotStand = 1;

    if (AreRobotStand) {

      State_data.mode = State_data.mode | Enable_Motor;
      Set_Mode(State_data.mode);

    } else {
      State_data.mode = State_data.mode & (0xff - Enable_Motor);
      Set_Mode(State_data.mode);
      error_speed_sum = 0;
    }

    Set_Control(&State_data.Pitch_Ref);


  }

  if (timenow - t3 >= 100) { // 10Hz Serial print
    t3 = timenow;
    Serial.print("robot speed ");  Serial.print(Robot_speed);
    Serial.print("     Pitch_Ref");  Serial.print(State_data.Pitch_Ref);
    Serial.print("     Steering_Ref");  Serial.print(State_data.Steering_Ref);

    Serial.print("     Roll ");  Serial.print(State_data.roll);
    Serial.print("     Pitch ");  Serial.print(State_data.pitch);
    Serial.print("     Roll ");  Serial.print(State_data.yaw);

    Serial.print("   kp ");  Serial.print(tuningData[1].kp);
    Serial.print("   ki ");  Serial.print(tuningData[1].ki);

    Serial.print("\n");
  }

  //  Get_Positions(&State_data.Position_motorL);
  //  Serial.print(State_data.Position_motorL);  Serial.print("\t");
  //  Serial.print(State_data.Position_motorR);  Serial.print("\t");
  //
  //  Get_VRs(&State_data.VR1);
  //  Serial.print(State_data.VR1);  Serial.print("\t");
  //  Serial.print(State_data.VR2);  Serial.print("\t");
  //  Serial.print(State_data.VR3);  Serial.print("\t");
  //  Serial.print(State_data.VR4);  Serial.print("\t");
  //
  //  Get_Accelerations(&State_data.AccX);
  //  Serial.print(State_data.AccX);  Serial.print("\t");
  //  Serial.print(State_data.AccY);  Serial.print("\t");
  //  Serial.print(State_data.AccZ);  Serial.print("\t");
  //
  //  Get_Angles(&State_data.roll);
  //  Serial.print(State_data.roll);  Serial.print("\t");
  //  Serial.print(State_data.pitch);  Serial.print("\t");
  //  Serial.print(State_data.yaw);  Serial.print("\t");
  //
  //    Get_OffsetAngularVelocity(&State_data.OffsetGyroX);
  //    Serial.print(State_data.OffsetGyroX);  Serial.print("\t");
  //    Serial.print(State_data.OffsetGyroY);  Serial.print("\t");
  //    Serial.print(State_data.OffsetGyroZ);  Serial.print("\t");
  //  delay(100);

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

void Set_P_Motors(int32_t *tmp) {
  uint16_t* tmp1 = (uint16_t*)tmp;
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(process_time_uS);
  SPI.transfer(0x0A);
  delayMicroseconds(process_time_uS);
  tmp1[0] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // Power_MotorL
  tmp1[1] =  (uint16_t)SPI.transfer(0) | (uint16_t)SPI.transfer(0) << 8; // Power_MotorR
  digitalWrite(slaveSelectPin, HIGH);
}
