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

uint8_t state = 0;

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

uint8_t reverse = 0;

int16_t x_global_prev = 0;
int16_t y_global_prev = 0;
float theta_global_prev = 0;
int16_t d_prev = 0;


// put your setup code here, to run once:
void setup() {

  pinMode(pinBuzzer, OUTPUT);
  pinMode(BUTTON_A_PIN, INPUT);

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  //Initialise motors
  motors.initialise();

  //Initialise and calibrate light sensors
  lsensors.initialise();
  lsensors.calibrateSensors();

  //Initialise encoders
  setupEncoder0();
  setupEncoder1();

  tick = millis();
  tick2 = millis();

}


// put your main code here, to run repeatedly:
void loop() {  
  
  //First we read the odometry
  tock = millis();
  unsigned long dt = tock - tick ;
  if (dt > 50){
    kinematics.update(-1*count_e1, -1*count_e0, dt/1000.0);
    
    
      /*
       * We only save the path points if two conditions are met:
       * 1. the waypoint.t variable is zero. Please see below how this is set to zero
       * 2. the current STATE in on the line, meaning we are currently following the black line.
       *    This also prevents us from saving the odometry before joining the line.
      */
      if (waypoint.t == 0 and state == STATE_ONLINE){
        waypoint.savePoint(kinematics.x_global - x_global_prev, kinematics.y_global - y_global_prev, kinematics.theta_global - theta_global_prev);
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
    
    /*
     * The waypoint.t is zero in two cases:
     * 1. At the beginning of the code execution (It is initialised with 0). See waypoint.h
     * 2. Every 10 updates of the odometry. Why? The odometry updates every 50 ms, then if 
     * we want to save the path points every 500 ms, we need to wait for waypoint.t to reach 10
     * and then reset it to zero. This wil make the previous conditional to be true and save the
     * path point. If we wanted to save every 2 seconds, we would ne to wait for waypoint.t to 
     * reach 40 (expected_saving_period/50 = 2000/50 = 40)
    */
    waypoint.t = waypoint.t + int16_t(dt/50);
    if (waypoint.t >= 10){
      waypoint.t = 0;
    }
    
    /* 
    else {
      Serial.print(kinematics.x_global);
      Serial.print(", ");
      Serial.print(kinematics.y_global);
      Serial.print(", ");
      Serial.print(kinematics.theta_global);
      Serial.println("");
    }*/
    tick = millis();
  }
  
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
        delay(10);  
    } else if (eline == -2){
        motors.setMotorPower(30,0);
        delay(10);      
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
  }
  //New
  if (state == STATE_LINE_END){
    
    motors.setMotorPower(0,0);

    while (digitalRead(BUTTON_A_PIN) == LOW){
      
        waypoint.printPoints();
      
        Serial.println("..........");
        delay(50);
    }
    
   
  }
    
  //lsensors.getLineError(true);
  lsensors.readLineSensorsCalibrated(false);
  
  delay(5);
  
  
}
