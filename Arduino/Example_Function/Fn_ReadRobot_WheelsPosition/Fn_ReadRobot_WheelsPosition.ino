#include "CMMC_balancing.h"

Control_data State_data;

void setup() {

  Serial.begin(115200);
  Serial.print("\n\n\n Start CMMC Balancing Robot demonstration\n\n");
  Serial.print("\n\t-Read Robot Wheels Position");

  Init(); // initialize

}

void loop() {

  Get_Positions(&State_data.Position_motorL);
  Serial.print("Robot Position L Wheel: "); Serial.print(State_data.Position_motorL);
  Serial.print("\tR Wheel: "); Serial.print(State_data.Position_motorR); Serial.println("\tPulse.");
  delay(100);

}
