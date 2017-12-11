#include "CMMC_balancing.h"

Control_data State_data;

uint8_t max_Exmaple_mode = 10;
uint8_t Exmaple_mode = 0;

const uint8_t SerialBufferMax = 16;         // a string to hold incoming data
char SerialBuffer[SerialBufferMax] = {0};

int a, b, var;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n\n\n Start CMMC Balancing Robot demonstration\n\n");
  Serial.print("\n\n\t-Show Robot Angles");
  Serial.print("\n\t-Read Robot AngularVelocity");
  Serial.print("\n\t-Read Robot Acceleration");
  Serial.print("\n\t-Read Robot Wheels Position");
  Serial.print("\n\t-Read Variable Resistors");
  Serial.print("\n\t-Control Robot Wheel Speed Mode");
  Serial.print("\n\t-Control Disable/Enable Motor");
  Serial.print("\n\t-Control Robot Wheel Torque Mode\n\n");
  
  Init();

}

void loop() {

  Serial.print("\n...type: 1 2 3   for next section...\n\n");

  while (var == 0) {

    delay(100);
    serialEvent();
  }
  Serial.print("\n\n\n  Show Robot Angles \n\n");
  while (var == 1) {
    Get_Angles(&State_data.roll);
    Serial.print("Robot Angles\t\t Roll: "); Serial.print((float)State_data.roll * 0.1f);
    Serial.print("\t\tPitch: : "); Serial.print((float)State_data.pitch * 0.1f);
    Serial.print("\t\tYaw: "); Serial.print((float)State_data.yaw * 0.1f); Serial.println("\tDeg.");

    delay(100);
    serialEvent();
  }
  Serial.print("\n\n\n  Show Robot AngularVelocity\n\n");
  while (var == 2) {

    Get_AngularVelocity(&State_data.OffsetGyroX);
    Serial.print("Robot AngularVelocity \t\tWx: "); Serial.print((float)State_data.OffsetGyroX * 0.1f);
    Serial.print("\t\tWy: "); Serial.print((float)State_data.OffsetGyroY * 0.1f);
    Serial.print("\t\tWz: "); Serial.print((float)State_data.OffsetGyroZ * 0.1f); Serial.println("\tDeg/S.");

    delay(100);
    serialEvent();
  }
  Serial.print("\n\n\n Show Robot Acceleration\n\n");
  while (var == 3) {
    Get_Accelerations(&State_data.AccX);
    Serial.print("Robot Acceleration \t\tAx: "); Serial.print((float)State_data.AccX * 0.01f);
    Serial.print("\t\tAy: "); Serial.print((float)State_data.AccY * 0.01f);
    Serial.print("\t\tAz: "); Serial.print((float)State_data.AccZ * 0.01f); Serial.println("\tG.");
    delay(100);
    serialEvent();
  }
  Serial.print("\n\n\n Show Robot Wheel Position\n\n");
  while (var == 4) {
    Get_Positions(&State_data.Position_motorL);
    Serial.print("Robot Position L Wheel: "); Serial.print(State_data.Position_motorL);
    Serial.print("\tR Wheel: "); Serial.print(State_data.Position_motorR); Serial.println("\tPulse.");
    delay(100);
    serialEvent();
  }
  Serial.print("\n\n\n Show Variable Resistors\n\n");
  while (var == 5) {
    Get_VRs(&State_data.VR1); Serial.print("\tVR1: ");
    Serial.print(State_data.VR1);  Serial.print("\tVR2: ");
    Serial.print(State_data.VR2);  Serial.print("\tVR3: ");
    Serial.print(State_data.VR3);  Serial.print("\tVR4: ");
    Serial.print(State_data.VR4);  Serial.print("\n");
    delay(100);
    serialEvent();
  }

  Serial.print("\n\n\n Control Robot Wheel Speed Mode\n\n");
  a = 0;
  b = 0;
  State_data.mode = Manual_Motor_control | Enable_Motor;
  Set_Mode(State_data.mode);

  while (var == 6) {
    Serial.print("Speed Mode L motor(-100 to 100),R motor(-100 to 100)");
    Serial.print("\t\tL motor= "); Serial.print((int)a);
    Serial.print("\t\tR motor= "); Serial.println((int)b);
    State_data.mode = Manual_Motor_control | Enable_Motor;
    State_data.Power_MotorL = (int)a * 20;
    State_data.Power_MotorR = (int)b * 20;
    Set_P_Motors(&State_data.Power_MotorL);
    delay(100);
    serialEvent();
  }

  State_data.mode = none;
  Set_Mode(State_data.mode);
  delay(100);
  Serial.print("\n\n\n Control Disable/Enable Motor\n\n");
  State_data.mode = Manual_Motor_control ;
  Set_Mode(State_data.mode);
  State_data.Power_MotorL = 100;
  State_data.Power_MotorR = 100;
  Set_P_Motors(&State_data.Power_MotorL);

  while (var == 7) {
    Serial.print(" 0 0 0 = Disable,\t 1 1 0 = Enable Motor");
    if (a == 0) {
      Serial.println("\t Motor: Disable");
      State_data.mode = Manual_Motor_control;
      Set_Mode(State_data.mode);
    } else {
      Serial.println("\t Motor: Enable");
      State_data.mode = Manual_Motor_control | Enable_Motor ;
      Set_Mode(State_data.mode);
      State_data.Power_MotorL = 300;
      State_data.Power_MotorR = 300;
      Set_P_Motors(&State_data.Power_MotorL);
    }

    delay(100);
    serialEvent();
  }
  State_data.mode = none;
  Set_Mode(State_data.mode);
  delay(100);
  Serial.print("\n\n\n Control Robot Wheel Torque Mode\n\n");
  State_data.mode = Enable_Motor;
  Set_Mode(State_data.mode);
  a = 0;
  b = 0;
  while (var == 8) {
    Serial.print(" Torque Mode \tFoeward/Backward , Turn L/Turn R");
    Serial.print("\tF/B= "); Serial.print((int)a);
    Serial.print("\tL/R= "); Serial.println((int)b);
    State_data.Pitch_Ref = (int)a * 1;
    State_data.Steering_Ref = (int)b * 1;
    Set_Control(&State_data.Pitch_Ref);
    State_data.mode = Enable_Motor;
    delay(100);
    serialEvent();
  }
  State_data.mode = none;
  Set_Mode(State_data.mode);
  var = 0;
}

void serialEvent() {
  while (Serial.available() > 0) {

    int T1 = Serial.parseInt();
    int T3 = Serial.parseInt();
    int T2 = Serial.parseInt();
    if (Serial.read() == '\n') {
      Serial.print("\n\n\nGET Data");
      Serial.print("\n\n\n");
      a = T1;
      b = T3;
      if (T2 == 3 && T1 == 1 && T3 == 2) {
        var++;
        a = 0;
        b = 0;
      }
    }
  }
}
