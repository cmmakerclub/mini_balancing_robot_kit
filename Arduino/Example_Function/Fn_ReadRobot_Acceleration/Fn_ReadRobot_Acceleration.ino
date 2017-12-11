#include "CMMC_balancing.h"

Control_data State_data;

void setup() {

  Serial.begin(115200);
  Serial.print("\n\n\n Start CMMC Balancing Robot demonstration\n\n");
  Serial.print("\n\t-Read Robot Acceleration");

  Init(); // initialize

}

void loop() {

  Get_Accelerations(&State_data.AccX);
  Serial.print("Robot Acceleration \t\tAx: "); Serial.print((float)State_data.AccX * 0.01f);
  Serial.print("\t\tAy: "); Serial.print((float)State_data.AccY * 0.01f);
  Serial.print("\t\tAz: "); Serial.print((float)State_data.AccZ * 0.01f); Serial.println("\tG.");
  delay(100);

}
