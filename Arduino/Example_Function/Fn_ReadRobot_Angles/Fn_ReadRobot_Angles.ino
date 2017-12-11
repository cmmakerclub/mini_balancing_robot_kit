#include "CMMC_balancing.h"

Control_data State_data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n\n\n Start CMMC Balancing Robot demonstration\n\n");
  Serial.print("\n\t-Read Robot Angles");

  Init();// initialize Robot

}

void loop() {

  Get_Angles(&State_data.roll); // Read roll pitch yaw
  Serial.print("Robot Angles\t\t Roll: "); Serial.print((float)State_data.roll * 0.1f);
  Serial.print("\t\tPitch: : "); Serial.print((float)State_data.pitch * 0.1f);
  Serial.print("\t\tYaw: "); Serial.print((float)State_data.yaw * 0.1f); Serial.println("\tDeg.");
  delay(100);

}

