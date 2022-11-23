#include "motors.h"
#include "linesensor.h"
//#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

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

Motors_c motors;
LineSensor_c lsensors;

int state = 0;

float eline = 0;
float last_eline = 0;
float d_eline = 0;
float i_eline = 0;

float control_action = 0;
float last_control_action = 0;

bool white = false;
bool last_white = false;

int reverse = 0;

unsigned long tick = 0;
unsigned long tock = 0;

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

  unsigned long tick = millis();

  
}


// put your main code here, to run repeatedly:
void loop() {
  
  
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
      tick = millis();
      if (tick < 6000){
        reverse = 1;
        motors.setMotorPower(-30,30);
        delay(1200);
      } else {
        last_white = true;
      }
    } else if (white and last_white){
      tock = millis() - tick;
      Serial.println(tock);
      if (tock > 1500){
        motors.setMotorPower(0,0);
        state++;
      }  
    }
    last_white = white;
  }
  //New
  if (state == STATE_LINE_END){
    motors.setMotorPower(-30,30);
    delay(1330);
    motors.setMotorPower(0,0);
    delay(50);
    motors.setMotorPower(30,30);
    delay(2000);
    motors.setMotorPower(0,0);
    delay(50);
    if (reverse == 0){
      motors.setMotorPower(-30,30);
    } else {
      motors.setMotorPower(30,-30);
    }
    delay(1330/2);
    motors.setMotorPower(0,0);
    delay(50);
    motors.setMotorPower(30,30);
    delay(12000);
    motors.setMotorPower(0,0);
    state++;
  }
    
  //lsensors.getLineError(true);
  lsensors.readLineSensorsCalibrated(true);
  
  delay(50);

}
