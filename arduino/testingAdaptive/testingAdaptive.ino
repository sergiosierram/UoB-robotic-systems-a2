#include "motors.h"
#include "encoders.h"
#include "linesensor.h"
#include "kinematics.h"
#include "pid.h"
#include "waypoint.h"

// Declare global variables here
float eline = 0;
float last_eline = 0;
float d_eline = 0;
float i_eline = 0;

float control_action = 0;
float last_control_action = 0;

int16_t prev_x_global = 0; // We initialise the position as 0 in x
int16_t prev_y_global = 0; // We initialise the position as 0 in y
float prev_theta_global = 0;
int16_t prev_d = 0;


unsigned long tick;
unsigned long tick2;
unsigned long tock;
unsigned long tock2;

Motors_c motors;
LineSensor_c lsensors;
Kinematics_c kinematics;
Waypoint_c waypoint;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  //Initialise motors
  motors.initialise();

  //Initialise and calibrate light sensors
  lsensors.initialise();
  lsensors.calibrateSensors();  

  tick = millis();
  tick2 = millis();

}

void loop() {
  // put your main code here, to run repeatedly:

  tock = millis();

  // Basic manual controller without line-following algorithm
  unsigned long dt = tock - tick;
  if (dt < 7500)  {
    tock2 = millis();
    unsigned long dt2 = tock2 - tick2;
    if (dt2 > 50){
      kinematics.update(-1*count_e1, -1*count_e0, dt2/1000.0);
      prev_x_global = kinematics.x_global;
      prev_y_global = kinematics.y_global;
      prev_theta_global = kinematics.theta_global;
      if (waypoint.t == 0){
        waypoint.savePoint(kinematics.x_global - prev_x_global, kinematics.y_global - prev_y_global, kinematics.theta_global - prev_theta_global);
        Serial.print(waypoint.t);
        Serial.print(", ");
        Serial.print(kinematics.x_global - prev_x_global);
        Serial.print(", ");
        Serial.print(kinematics.y_global - prev_y_global);
        Serial.print(", ");
        Serial.print(kinematics.theta_global - prev_theta_global);
        Serial.print(", ");
        Serial.print(kinematics.d - prev_d);
        Serial.println(""); 
      }
    }
    waypoint.t = waypoint.t + int16_t(dt2/50);
    if (waypoint.t >= 10){
      waypoint.t = 0;
    }
    motors.setMotorPower(30,30);
    delay(1500);
    motors.setMotorPower(30,40);
    delay(4800);
    motors.setMotorPower(30,30);
    delay(1200);
  } else {
    motors.setMotorPower(0,0);
  }
  // waypoint.printPoints();
  // delay(25);
  Serial.println(waypoint.idx);
  

  


}
