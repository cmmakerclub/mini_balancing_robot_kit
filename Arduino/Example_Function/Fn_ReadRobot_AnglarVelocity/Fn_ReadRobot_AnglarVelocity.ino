#include "CMMC_balancing.h"

Control_data State_data;

void setup() {

  Serial.begin(115200);
  Serial.print("\n\n\n Start CMMC Balancing Robot demonstration\n\n");
  Serial.print("\n\t-Read Robot AngularVelocity");

  Init(); // initialize

}

void loop() {
  
  Get_AngularVelocity(&State_data.OffsetGyroX);
  Serial.print("Robot AngularVelocity \t\tWx: "); Serial.print((float)State_data.OffsetGyroX * 0.1f);
  Serial.print("\t\tWy: "); Serial.print((float)State_data.OffsetGyroY * 0.1f);
  Serial.print("\t\tWz: "); Serial.print((float)State_data.OffsetGyroZ * 0.1f); Serial.println("\tDeg/S.");

  delay(100);

}
