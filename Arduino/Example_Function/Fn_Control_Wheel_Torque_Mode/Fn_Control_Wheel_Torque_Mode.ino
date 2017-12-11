#include "CMMC_balancing.h"

Control_data State_data;

int a, b;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n\n\n Start CMMC Balancing Robot demonstration\n\n");
  Serial.print("\n\t-Control Robot Wheel Torque Mode\n\n");

  Init(); // initialize

}

void loop() {

  State_data.mode = none;
  Set_Mode(State_data.mode);
  delay(100);
  Serial.print("\n\n\n Control Robot Wheel Torque Mode\n\n");
  State_data.mode = Enable_Motor;
  Set_Mode(State_data.mode);
  a = 0;
  b = 0;
  while (1) {
    Serial.print(" Torque Mode \tFoeward/Backward , Turn L/Turn R");
    Serial.print("\tF/B= "); Serial.print((int)a);
    Serial.print("\tL/R= "); Serial.println((int)b);
    State_data.Pitch_Ref =    (int)a * 1;
    State_data.Steering_Ref = (int)b * 1;
    Set_Control(&State_data.Pitch_Ref);
    State_data.mode = Enable_Motor;
    delay(100);
    serialEvent();
  }

}

void serialEvent() {
  while (Serial.available() > 0) {

    int T1 = Serial.parseInt();
    int T2 = Serial.parseInt();
    if (Serial.read() == '\n') {
      a = T1;
      b = T2;
    }
  }
}
