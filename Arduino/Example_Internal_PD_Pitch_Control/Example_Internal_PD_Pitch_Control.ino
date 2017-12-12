#include "CMMC_Receiver.h"
#include "CMMC_balancing.h"
#include <SPI.h>

Control_data State_data;  // robot control struct

float error_pitch;
float error_pitch_dot;

uint32_t timenow, t1, t2, t3;
uint32_t p1, p2, p3;
byte AreRobotStand = 0;

void setup() {


  Serial.begin(115200);

  receriver_init(); //initialize reveiver mode

  Init(); // initialize Robot


  State_data.mode =  Enable_Motor;  //  Enable Motors Torque mode
  Set_Mode(State_data.mode); // sent command to robot
}

void loop() {

  receriver_loop();

  timenow = millis();


  if (timenow - t2 >= 5) { // 200Hz Pitch Angle control
    t2 = timenow;

    Get_Angles(&State_data.roll); // read robot angles

    float kp = (float)tuningData[1].kp * 0.03f;  // valus from APP setup page
    float kd = (float)tuningData[1].kd * 0.005f;  // valus from APP setup page

    float error_pitch_prev = error_pitch;

    error_pitch = - 0 + State_data.pitch;

    error_pitch_dot = (error_pitch - error_pitch_prev) * 200;

    State_data.Pitch_Ref = error_pitch * kp + error_pitch_dot * kd;

    if ( abs(State_data.pitch) > 450 | abs(State_data.roll) > 450) AreRobotStand = 0;  //fall
    if ( abs(State_data.pitch) <  50 & abs(State_data.roll) < 450) AreRobotStand = 1;  //stand

    if (AreRobotStand) {

      State_data.mode = State_data.mode | Enable_Motor; // sent command to robot
      Set_Mode(State_data.mode);

    } else {
      State_data.mode = State_data.mode & (0xff - Enable_Motor); // sent command to robot Disable motor
      Set_Mode(State_data.mode);
      
    }
    Set_Control(&State_data.Pitch_Ref);  // sent command to robot
  }


  if (timenow - t3 >= 100) { // 10Hz Serial print
    t3 = timenow;
    Serial.print("Output ");  Serial.print(State_data.Pitch_Ref);
    Serial.print("    error_pitch ");  Serial.print(error_pitch);


    Serial.print("\n");
  }
}

