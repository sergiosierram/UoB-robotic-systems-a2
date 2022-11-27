// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Define global variables here
volatile long count_e1_prev = 0;
volatile long count_e0_prev = 0;

float phi_left = 0; // angular velocity of the left wheel;
float phi_right = 0; // angular velocity of the right wheel;

// Class to track robot position.
class Kinematics_c {
  public:
    float u = 0;
    float w = 0;
    
    int16_t x_global = 0; // x coordinate of robot in the global frame
    int16_t y_global = 0; // y coordinate of robot in the global frame
    float theta_global = 0; // angular position of robot in the global frame

    int16_t d = 0;
  
    // Constructor, must exist.
    Kinematics_c() {

    } 

    // Use this function to update
    // your kinematics
    void update(volatile long count_e1, volatile long count_e0, float dt) {
      
      angularVelocity(count_e1,count_e0, dt);

      u = 0.5*(phi_left + phi_right);
      w = (phi_right - phi_left)/(2*0.0446);
      
      x_global = int16_t(x_global + 1000*u*cos(theta_global)*dt);
      y_global = int16_t(y_global + 1000*u*sin(theta_global)*dt);
      theta_global = theta_global + dt*w;

      d = d + 1000*sqrt(pow(u*cos(theta_global)*dt, 2) + pow(u*sin(theta_global)*dt, 2));
    }

    void angularVelocity(volatile long count_e1, volatile long count_e0, float dt) {
      // This function calculates the angular velocity of each wheel
      phi_left = (0.016/dt) * (((count_e1-count_e1_prev)/358.3)*2*PI);
      phi_right = (0.016/dt) * (((count_e0-count_e0_prev)/358.3)*2*PI);
      count_e1_prev = count_e1;
      count_e0_prev = count_e0;
    }

};



#endif
