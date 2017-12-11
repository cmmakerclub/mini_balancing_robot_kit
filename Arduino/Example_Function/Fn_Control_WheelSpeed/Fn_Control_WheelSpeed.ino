#include "CMMC_balancing.h"

Control_data State_data;

int a, b;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n\n\n Start CMMC Balancing Robot demonstration\n\n");
  Serial.print("\n\t-Control Robot Wheel Speed Mode");

  Init(); // initialize
  a = 0;
  b = 0;
}

void loop() {

  State_data.mode = Manual_Motor_control | Enable_Motor;
  Set_Mode(State_data.mode);

  Serial.print("Speed Mode L motor(-100 to 100),R motor(-100 to 100)");
  Serial.print("\t\tL motor= "); Serial.print((int)a);
  Serial.print("\t\tR motor= "); Serial.println((int)b);

  State_data.Power_MotorL = (int)a * 20;  // -2999 t0 2999
  State_data.Power_MotorR = (int)b * 20;  // -2999 t0 2999
  Set_P_Motors(&State_data.Power_MotorL);
  delay(100);

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
