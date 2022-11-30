#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "controllers.h"
#include "waypoint.h"

#define LED_PIN             13  // Pin to activate the orange LED

#define STATE_START         0   // This is the default recording state
#define STATE_LINE_REACHED  1   // This state is reached after white calibration
#define STATE_JOINING       2   // This state is reached when the robot attemps to join the line
#define STATE_ONLINE        3   // This state is reached and maintained while following the line with light sensors
#define STATE_LINE_END      4   
#define STATE_CONTROL       5
#define STATE_END           6

#define RUN_RECORD          0
#define RUN_CONTROL         1
#define RUN_DONE            2

#define FIXED_WAYPOINT      0
#define ADAPTIVE_WAYPOINT   1

#define PID_CONTROL         0
#define NONLINEAR_CONTROL   1

#define pinBuzzer  6

# define BUTTON_A_PIN  14

Motors_c motors;
LineSensor_c lsensors;
Kinematics_c kinematics;
Waypoint_c waypoint;
Controllers_c controller;

uint8_t state = 0;
uint8_t run_id = 0;

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
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

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
  if (run_id == RUN_RECORD){
    record(FIXED_WAYPOINT);
  } else if (run_id == RUN_CONTROL){
    control(PID_CONTROL);
  } else if (run_id == RUN_DONE){
    Serial.println("Job done, please see my results:");
    Serial.println("TO DO...");
  } else {
    Serial.println("This should not happen");  
  }
  
  lsensors.readLineSensorsCalibrated(true);
  delay(5);
}

void record(int algorithm){
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
        waypoint.savePoint(kinematics.x_global - x_global_prev, kinematics.y_global - y_global_prev);
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
      //motors.setMotorPower(0,0);
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
    while (digitalRead(BUTTON_A_PIN) == HIGH){
        waypoint.printPoints();
        Serial.println("..........");
        delay(50);
    }
    x_global_prev = kinematics.x_global-x_global_prev;
    y_global_prev = kinematics.y_global-y_global_prev;
    theta_global_prev = kinematics.theta_global-theta_global_prev;
    d_prev = kinematics.d-d_prev;
    run_id = RUN_CONTROL;
   
  }
}

void control(int controller){
  int16_t aux_idx = 0;
  while (aux_idx < waypoint.idx-1){
    //First we read the odometry
    tock = millis();
    unsigned long dt = tock - tick ;
    if (dt > 50){
      kinematics.update(-1*count_e1, -1*count_e0, dt/1000.0);
      tick = millis();
    
      int16_t error_x = 100;
      int16_t error_y = 100;
      while (abs(error_x) > 10 and abs(error_y) > 10){
        //1. Estimate the error between the current desired waypoint and the robot
        error_x = waypoint.x_path[aux_idx] - (kinematics.x_global - x_global_prev);
        error_y = waypoint.y_path[aux_idx] - (kinematics.y_global - y_global_prev);
  
        //2. Estimate the derivative between the next waypoint and the current waypoint
        int16_t diff_path_x = (waypoint.x_path[aux_idx+1] - waypoint.x_path[aux_idx])*dt;
        int16_t diff_path_y = (waypoint.y_path[aux_idx+1] - waypoint.x_path[aux_idx])*dt;

        //3. Create a Kinematic/Jacobian matrix with the model of the robot with the point of
        //interest in the front.
        float aux_theta = kinematics.theta_global - theta_global_prev;
        float J[2][2] = {{ cos(aux_theta), -0.0446*sin(aux_theta) },
                         { sin(aux_theta),  0.0446*cos(aux_theta) }};

        //4. Calculate the determinant of the Jacobian matrix
        float det = J[0][0]*J[1][1] -1*J[0][1]*J[1][0]; 

        //5. Calculate the inverse of the Jacobian matrix
        float inv_J[2][2] = {{J[1][1]/det, -1*J[0][1]/det},
                             {-1*J[1][0]/det, J[0][0]/det}};

        //6. Using the inv(J)*(diff + K*error) get the desired linear and angular velocities
        float aux[2] = { diff_path_x + 0.5*error_x,
                         diff_path_y + 0.5*error_y };        

        //6.1 Here we actually calculate u and w. The operation is inv(J)*(aux + k*e)
        float v[2] = {inv_J[0][0]*aux[0] + inv_J[0][1]*aux[1] ,
                      inv_J[1][0]*aux[0] + inv_J[1][1]*aux[1]};

        //7. Get indivual velocities for the wheels
        int vl = 30*(v[0] - 0.0446*v[1]);
        int vr = 30*(v[0] + 0.0446*v[1]);
        
        //8. Set the velocities to the motors.
        motors.setMotorPower(vl,vr);
        Serial.print(waypoint.x_path[aux_idx]);
        Serial.print(", ");
        Serial.print(waypoint.y_path[aux_idx]);
        Serial.print(", ");
        Serial.print(vl);
        Serial.print(", ");
        Serial.print(vr);
        Serial.print(", ");
        Serial.print(error_x);
        Serial.print(", ");
        Serial.print(error_y);
        Serial.println(", ");
        delay(10);     
      }
      aux_idx++; 
    }
  }
}
