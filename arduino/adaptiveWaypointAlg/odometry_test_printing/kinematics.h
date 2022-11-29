// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Define global variables here
volatile long count_e1_prev = 0;
volatile long count_e0_prev = 0;

float x_local = 0; // x coordinate of robot in the local frame
float y_local = 0; // y coordinate of robot in the local frame
float theta_local = 0; // angular position of robot in the global frame


float encoder_count = 358.3;

float l = 0.0446; // 
float r = 0.016; //

float phi_left = 0; // angular velocity of the left wheel;
float phi_right = 0; // angular velocity of the right wheel;

float x_global = 0; // x coordinate of robot in the global frame
float y_global = 0; // y coordinate of robot in the global frame
float theta_global = 0; // angular position of robot in the global frame

// Class to track robot position.
class Kinematics_c {
  public:
  
    // Constructor, must exist.
    Kinematics_c() {

    } 

    // Use this function to update
    // your kinematics
    void update(volatile long count_e1, volatile long count_e0, float dt) {
      
      angularVelocity(count_e1,count_e0, dt);

      float u = 0.5*(phi_left + phi_right);
      float w = (phi_right - phi_left)/(2*l);

      x_local = u*cos(theta_local);
      y_local = u*sin(theta_local);
      
      x_global = x_global + x_local*dt;
      y_global = y_global + y_local*dt;
      theta_local = theta_local + dt*w;
      
      

      //Serial.println(x_global);
    }

    void angularVelocity(volatile long count_e1, volatile long count_e0, float dt) {
      // This function calculates the angular velocity of each wheel
      phi_left = (r/dt) * (((count_e1-count_e1_prev)/358.3)*2*PI);
      phi_right = (r/dt) * (((count_e0-count_e0_prev)/358.3)*2*PI);
      count_e1_prev = count_e1;
      count_e0_prev = count_e0;
    }

   


   
};


//
//
//float x_local = 0; // x coordinate of robot in the local frame
//float y_local = 0; // y coordinate of robot in the local frame
//float theta_local = 0; // angular position of robot in the global frame
//
//
//

//
//void distanceGlobal() {
//  // This function helps the robot to move within the global frame
//  x_global = x_global + (x_local*cos(theta_global));
//  y_global = y_global + (x_local*sin(theta_global));
//  theta_global = theta_global + theta_local;
//}
//



#endif
