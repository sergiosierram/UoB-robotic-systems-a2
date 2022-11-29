#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
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
Kinematics_c kinematics;

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

  //Initialise encoders
  setupEncoder0();
  setupEncoder1();

  unsigned long tick = millis();

}


// put your main code here, to run repeatedly:
void loop() {
  unsigned long tock = millis();
  unsigned long dt = tock - tick ;
  if (dt > 50){
    kinematics.update(-1*count_e1, -1*count_e0, dt/1000.0);
    Serial.println(theta_local);
    tick = millis();
  }
//  Serial.print(-1*count_e1);
//  Serial.print(",");
//  Serial.println(-1*count_e0);
//  
  delay(1);
}
