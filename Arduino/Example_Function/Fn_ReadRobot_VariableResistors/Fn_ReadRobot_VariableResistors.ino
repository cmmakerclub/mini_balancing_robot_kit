#include "CMMC_balancing.h"

Control_data State_data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n\n\n Start CMMC Balancing Robot demonstration\n\n");
  Serial.print("\n\t-Read Variable Resistors");

  Init(); // initialize

}

void loop() {

  Get_VRs(&State_data.VR1); Serial.print("\tVR1: ");
  Serial.print(State_data.VR1);  Serial.print("\tVR2: ");
  Serial.print(State_data.VR2);  Serial.print("\tVR3: ");
  Serial.print(State_data.VR3);  Serial.print("\tVR4: ");
  Serial.print(State_data.VR4);  Serial.print("\n");
  delay(100);

}
