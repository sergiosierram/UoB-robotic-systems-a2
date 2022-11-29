#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "waypoint.h"

#define LED_PIN 13  // Pin to activate the orange LED
boolean led_state;  // Variable to "remember" the state
                    // of the LED, and toggle it.

#define STATE_START 0
#define STATE_LINE_REACHED 1
#define STATE_JOINING 2
#define STATE_ONLINE 3
#define STATE_LINE_END 4
#define STATE_RETURNING 5
#define STATE_END 6

#define pinBuzzer  6
# define BUTTON_A_PIN  14

Motors_c motors;
LineSensor_c lsensors;
Kinematics_c kinematics;
Waypoint_c waypoint;

int state = 0;

float eline = 0;
float last_eline = 0;
float d_eline = 0;
float i_eline = 0;

float control_action = 0;
float last_control_action = 0;

unsigned long tick;
unsigned long tick2;
unsigned long tock;
unsigned long tock2;

bool white = false;
bool last_white = false;

int reverse = 0;

int16_t x_global_prev = 0;
int16_t y_global_prev = 0;
float theta_global_prev = 0; // Help to store the value of the previous value of theta_global
int16_t d_prev = 0;
float theta_reference = -10;

// put your setup code here, to run once:
void setup() {

  pinMode(pinBuzzer, OUTPUT);

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


// put your main code here, to run repeatedly:
void loop() {

  // First we need to compute the value of x_global, y_global and theta_global
  tock =  millis();
  unsigned long dt = tock - tick ;
  if (dt > 50){
    kinematics.update(-1*count_e1, -1*count_e0, (dt)/1000.0);
    if (true){
      motors.setMotorPower(0,0);
        delay(1000);
      if (theta_reference == -10){
        theta_reference = kinematics.theta_global- theta_global_prev;
        motors.setMotorPower(0,0);
        delay(1000);
      }
      if (abs(kinematics.theta_global - x_global_prev - theta_reference) > 0.05){
        waypoint.savePoint(kinematics.x_global - x_global_prev, kinematics.y_global - y_global_prev);
        theta_reference = kinematics.theta_global;
        Serial.print(waypoint.t);
        Serial.print(", ");
        Serial.print(kinematics.x_global - x_global_prev);
        Serial.print(", ");
        Serial.print(kinematics.y_global - y_global_prev);
        Serial.print(", ");
        Serial.print(kinematics.theta_global - theta_global_prev);
        Serial.print(", ");
        Serial.print(kinematics.d - d_prev);
        Serial.println("");
      }
    }
  }  

  // waypoint.t = waypoint.t + int16_t((tock-tick)/50);
  // if (waypoint.t >= 10){ // If the robot is turning (90 deg turns or rounded curved)
  //   waypoint.t = 0;
  // }

  tick = millis(); // We need to reset the timer

  // Store the value of count_e0 and count_e1 for creating an offset
  if (state == STATE_START){
    motors.setMotorPower(30,30);
    if (lsensors.onBlack()){
      motors.setMotorPower(0,0);
      state++;
    }  
  } 
  if (state == STATE_LINE_REACHED){
    motors.setMotorPower(0,30);
    state++;
  }
  if (state == STATE_JOINING){
    if (lsensors.onLine()){
      motors.setMotorPower(0,0);
      state++;
      x_global_prev = kinematics.x_global;
      y_global_prev = kinematics.y_global;
      theta_global_prev = kinematics.theta_global;
      d_prev = kinematics.d;
    }
  }
  if (state == STATE_ONLINE){
    eline = lsensors.getLineError(true);

    if (eline > -2 and eline < 2 ){    
      d_eline = eline - last_eline;
      i_eline = eline + last_eline;
      control_action = 1.5*eline + 0.5*d_eline + 0.1*i_eline;
      int vl = 30 - 30*control_action;
      int vr = 30 + 30*control_action;
      motors.setMotorPower(vl,vr);
    } else if (eline == 2){
      motors.setMotorPower(0,30);
      delay(50);
    } else if (eline == -2){
      motors.setMotorPower(30,0);
      delay(50);
    } else if (eline == 10){
      motors.setMotorPower(-30,-30);
      delay(50);
    } 
    
    last_eline = eline;

    //New
    white = lsensors.onWhite();
    if (white and !last_white){
      tock2 = millis() - tick2;
      //Beware of this timer, it is used to stop the robot at the end of each run
      if (tock2 > 13000){
        state++;
      }
    } 
    //   tick = millis();
    //   if (tick < 6000){
    //     reverse = 1;
    //     motors.setMotorPower(-30,30);
    //     delay(1200);
    //   } else {
    //     last_white = true;
    //   }
    // } else if (white and last_white){
    //   tock = millis() - tick;
    //   Serial.println(tock);
    //   if (tock > 1500){
    //     motors.setMotorPower(0,0);
    //     state++;
    //   }  
    // }
    // last_white = white;
  }
  //New
  if (state == STATE_LINE_END){
    motors.setMotorPower(0,0); // The robot will stop when it reaches the end of the line

    while (digitalRead(BUTTON_A_PIN) == LOW){
      waypoint.printPoints();

      Serial.println("..........");
      Serial.println(waypoint.idx);
      delay(50);
    }
  }



  //   motors.setMotorPower(-30,30);
  //   delay(1330);
  //   motors.setMotorPower(0,0);
  //   delay(50);
  //   motors.setMotorPower(30,30);
  //   delay(2000);
  //   motors.setMotorPower(0,0);
  //   delay(50);
  //   if (reverse == 0){
  //     motors.setMotorPower(-30,30);
  //   } else {
  //     motors.setMotorPower(30,-30);
  //   }
  //   delay(1330/2);
  //   motors.setMotorPower(0,0);
  //   delay(50);
  //   motors.setMotorPower(30,30);
  //   delay(12000);
  //   motors.setMotorPower(0,0);
  //   state++;
  // }
    
  // //lsensors.getLineError(true);
  lsensors.readLineSensorsCalibrated(false);
  
  delay(5);

}
