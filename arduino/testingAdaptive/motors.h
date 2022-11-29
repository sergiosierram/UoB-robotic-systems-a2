// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH

#define MAX_PWM 80

#define L_FACTOR 1  //This is used to temporarily overcome drift
#define R_FACTOR 1

// Class to operate the motor(s).
class Motors_c {
  public:

    // Constructor, must exist.
    Motors_c() {

    } 

    // Use this function to initialise the pins and state of your motor(s).
    void initialise() {
      // Set all the motor pins as outputs. There are 4 pins in total to set.
      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);
    
      // Set initial direction (HIGH/LOW) for the direction pins.
      digitalWrite(L_DIR_PIN, FWD);
      digitalWrite(R_DIR_PIN, REV);
    
      // Set initial power values for the PWM Pins.
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);

    }

    // Write a function to operate your motor(s)
    /*
     * Sets the power of the motors using analogWrite(). This function sets direction and PWM (power).
     * This function catches all errors of input PWM.
     * inputs: 
     *     pwm   accepts negative, 0 and positve values.  Sign of value used to set
     *           the direction of the motor.  Values are limited in range [ -100 : 100 ].
     *           Magnitude used to set analogWrite().
     */
    void setMotorPower( float left_pwm, float right_pwm ) {
      //First catch errors, such as values outside the expected range
      left_pwm = checkPwm(left_pwm);
      right_pwm = checkPwm(right_pwm);

      int left_reverse = 1;
      int right_reverse = 1;
      
      //Chek direction
      if (left_pwm < 0){
        digitalWrite(L_DIR_PIN, REV);
        left_reverse = -1;
      }
      if (left_pwm > 0){
        digitalWrite(L_DIR_PIN, FWD);
      }
      if (right_pwm < 0){
        digitalWrite(R_DIR_PIN, REV);
        right_reverse = -1;
      }
      if (right_pwm > 0){
        digitalWrite(R_DIR_PIN, FWD);
      }

      //Write actual speed to motors
      //Serial.println(int(left_pwm*MAX_PWM));
      //Serial.println(int(right_pwm*MAX_PWM));
      
      analogWrite(L_PWM_PIN, int(left_reverse*left_pwm*MAX_PWM*L_FACTOR));
      analogWrite(R_PWM_PIN, int(right_reverse*right_pwm*MAX_PWM*R_FACTOR));
    
    }

    float checkPwm(float pwm){
      if (pwm < -100){ 
        pwm = -100;
      }
      if (pwm > 100){
        pwm = 100;  
      }
      return pwm/100.0;
    }
    
};



#endif
