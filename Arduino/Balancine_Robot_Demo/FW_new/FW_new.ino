#include "CMMC_Receiver.h"
#include "CMMC_balancing.h"
#include <SPI.h>

//#define calibrate_sensor

Control_data State_data;  // robot control struct

float Kp_Speed = 0;
float Ki_Speed = 0;
float error_speed;
float error_speed_sum;
float Robot_Target_Angle;
float Robot_speed;
uint32_t timenow, t1, t2, t3;
uint32_t p1, p2, p3;
byte AreRobotStand = 0;

void setup() {


  Serial.begin(115200);

  receriver_init(); //initialize reveiver mode

  Init(); // initialize Robot

#ifdef calibrate_sensor   //make sure the robot stand in perfect angle
  while (1) {
    State_data.mode = gyro_calibation_mode | acc_calibation_mode; // enable gyro_calibation and  acc_calibation
    Set_Mode(State_data.mode);// sent command to robot
    delay(10000);             // wait for calibating
    Get_OffsetAngularVelocity(&State_data.OffsetGyroX);
    Serial.print(State_data.OffsetGyroX);  Serial.print("\t");
    Serial.print(State_data.OffsetGyroY);  Serial.print("\t");
    Serial.print(State_data.OffsetGyroZ);  Serial.print("\t");

    Get_OffsetAccelerations(&State_data.OffsetAccX);
    Serial.print(State_data.OffsetAccX);  Serial.print("\t");
    Serial.print(State_data.OffsetAccY);  Serial.print("\t");
    Serial.print(State_data.OffsetAccZ);  Serial.println("\t");
  }
#endif

  State_data.mode = PI_Steering | PD_Controller | Enable_Motor;  // enable extanal PI_Steering turning control + enable extanal PD_Controller pitch angle control + Enable Motors
  Set_Mode(State_data.mode); // sent command to robot
}


void loop() {


  receriver_loop();

  timenow = millis();


  if (timenow - t1 >= 10) {  // 100Hz Robot Speed reading
    t1 = timenow;
    static int32_t last_L, last_R;

    Get_Positions(&State_data.Position_motorL);  // read wheels position

    float dL =  ((int32_t)State_data.Position_motorL - last_L);
    float dR =  ((int32_t)State_data.Position_motorR - last_R);

    last_L = (int32_t)State_data.Position_motorL;
    last_R = (int32_t)State_data.Position_motorR;

    Robot_speed  = Smooth_filter(0.05f, (dR + dL) * 0.5f, Robot_speed); // 0.83Hz lowpass filter , alpha = (2*PI*dT*Fc)/(2*PI*dT*Fc+1)
  }


  if (timenow - t2 >= 50) { // 20Hz PI velocity control
    t2 = timenow;
    
    Get_Angles(&State_data.roll); // read robot angles

    float Command_speed  = (float)Get_ChannelValue(2) * 0.4f; // read user command (-100 to 100 mapping ro -40 to 40 for robot speed)
    float Command_Turning  = (float)Get_ChannelValue(1);      // read user command


    float kp = (float)tuningData[1].kp * 0.005f;  // valus from APP setup page
    float ki = (float)tuningData[1].ki * 0.0005f;  // valus from APP setup page

    float P_limit = map (abs(Robot_speed), 0, 50, 250, 150);

    error_speed = limit(kp * (Command_speed - Robot_speed), -P_limit, P_limit);

    float i_limit = map (abs(Robot_speed), 0, 50, 150, 0);

    error_speed_sum = limit((error_speed_sum + (error_speed * 0.2f)), -i_limit / ki, i_limit / ki);

    State_data.Pitch_Ref = limit(ki * error_speed_sum + error_speed, -350, 350); // angle ref Control

    State_data.Steering_Ref = Command_Turning * (float)tuningData[0].kp * 0.04f; // L-R Control


    if ( abs(State_data.pitch) > 450 | abs(State_data.roll) > 450) AreRobotStand = 0;  //fall
    if ( abs(State_data.pitch) <  50 & abs(State_data.roll) < 450) AreRobotStand = 1;  //stand

    if (AreRobotStand) {

      State_data.mode = State_data.mode | Enable_Motor; // sent command to robot
      Set_Mode(State_data.mode);

    } else {
      State_data.mode = State_data.mode & (0xff - Enable_Motor); // sent command to robot Disable motor
      Set_Mode(State_data.mode);
      error_speed_sum = 0;
    }
    Set_Control(&State_data.Pitch_Ref);  // sent command to robot
  }


  if (timenow - t3 >= 100) { // 10Hz Serial print
    t3 = timenow;
    Serial.print("robot speed ");  Serial.print(Robot_speed);
    Serial.print("     Pitch_Ref");  Serial.print(State_data.Pitch_Ref);
    Serial.print("     Steering_Ref");  Serial.print(State_data.Steering_Ref);

    Serial.print("     Roll ");  Serial.print(State_data.roll);
    Serial.print("     Pitch ");  Serial.print(State_data.pitch);
    Serial.print("     Roll ");  Serial.print(State_data.yaw);

    Serial.print("   kp ");  Serial.print(tuningData[1].kp);
    Serial.print("   ki ");  Serial.print(tuningData[1].ki);

    Serial.print("\n");
  }
}

