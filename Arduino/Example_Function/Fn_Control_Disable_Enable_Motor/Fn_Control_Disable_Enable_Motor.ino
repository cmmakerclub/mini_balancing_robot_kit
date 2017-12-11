#include "CMMC_balancing.h"

Control_data State_data;

int a;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n\n\n Start CMMC Balancing Robot demonstration\n\n");
  Serial.print("\n\t-Control Disable/Enable Motor");

  Init(); // initialize

}

void loop() {

  State_data.mode = none;
  Set_Mode(State_data.mode);
  delay(100);
  Serial.print("\n\n\n Control Disable/Enable Motor\n\n");
  State_data.mode = Manual_Motor_control ;
  Set_Mode(State_data.mode);
  State_data.Power_MotorL = 300;
  State_data.Power_MotorR = 300;
  Set_P_Motors(&State_data.Power_MotorL);

  while (1) {
    Serial.print(" 0 = Disable,\t 1 = Enable Motor");
    if (a == 0) {
      Serial.println("\t Motor: Disable");
      State_data.mode = Manual_Motor_controls;
      Set_Mode(State_data.mode);
    } else {
      Serial.println("\t Motor: Enable");
      State_data.mode = Manual_Motor_control | Enable_Motor ;
      Set_Mode(State_data.mode);
      State_data.Power_MotorL = 300; // -2999 to 2999
      State_data.Power_MotorR = 300; // -2999 to 2999
      Set_P_Motors(&State_data.Power_MotorL);
    }

    delay(100);

  }
}

void serialEvent() {
  while (Serial.available() > 0) {
    int T1 = Serial.parseInt();
    if (Serial.read() == '\n') {
      a = T1;
    }
  }
}
