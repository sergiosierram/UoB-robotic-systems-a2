// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define LS_LEFT2_PIN A11
#define LS_LEFT1_PIN A0
#define LS_CENTER_PIN A2
#define LS_RIGHT1_PIN A3
#define LS_RIGHT2_PIN A4

#define EMIT  11
#define n_sensors 5

int pins_names[5] = { LS_LEFT2_PIN, LS_LEFT1_PIN, LS_CENTER_PIN, LS_RIGHT1_PIN, LS_RIGHT2_PIN };

int timeout = 5000;

// Class to operate the linesensor(s).
class LineSensor_c {
  public:

    unsigned long values[n_sensors] = {timeout, timeout, timeout, timeout, timeout};
    unsigned long values_calibrated[n_sensors] = {timeout, timeout,timeout, timeout, timeout};
    unsigned long calibration_min[n_sensors] = {timeout,timeout,timeout, timeout, timeout};
    unsigned long calibration_max[n_sensors] = {0,0,0, 0, 0};
    unsigned long calibration_min_black[n_sensors] = {timeout,timeout,timeout, timeout, timeout};
    unsigned long calibration_max_black[n_sensors] = {0,0,0, 0, 0};
    unsigned long calibration_offset[n_sensors] = {0,0,0, 0, 0};
    unsigned long calibration_range[n_sensors] = {timeout,timeout,timeout, timeout, timeout};
    bool calibration_done = false;
    float last_eline = 0;
    float eline = 0;
  
    // Constructor, must exist.
    LineSensor_c() {

    } 

    // Use this function to initialise the pins.
    void initialise() {
      // Declare pins as inputs for IRs
      for (int pin = 0; pin < n_sensors; pin ++){
        pinMode(pins_names[pin], INPUT);  
      }

      // Declare output mode for EMIT
      pinMode(EMIT, OUTPUT);

      // Set output value to work for line sensors
      digitalWrite(EMIT, HIGH);
    }

    bool onWhite(){
      if (calibration_done){
        readLineSensorsCalibrated(false);
        if (values_calibrated[0] < 500 and values_calibrated[1] < 500 and values_calibrated[2] < 500 and values_calibrated[3] < 500 and values_calibrated[4] < 500){
          return true;  
        } else {
          return false;  
        }
      } else {
        return true;
      }
      
    }

    bool onBlack(){
      if (calibration_done){
        readLineSensorsCalibrated(false);
        if (values_calibrated[1] > 500 and values_calibrated[2] > 500 and values_calibrated[3] > 500){
          return true;  
        } else {
          return false;  
        }
      } else {
        return true;
      }
    }

    bool onLine(){
      if (calibration_done){
        readLineSensorsCalibrated(false);
      } else {
        readLineSensors(false);
      }
      if (values_calibrated[1] < 800 and values_calibrated[2] > 800 and values_calibrated[3] < 800){
        return true;  
      } else {
        return false;  
      }
    }

    float getLineError(bool serial_verbose){
      if (onWhite()){
        return last_eline;  
      } else {
        readLineSensorsCalibrated(false);
        
        eline  = 0;
        int summation = values_calibrated[1] + values_calibrated[2] + values_calibrated[3];
        //int summation = 1000;
        float w_left = values_calibrated[1] + (values_calibrated[2]*0.5);
        float w_right = values_calibrated[3] + (values_calibrated[2]*0.5);
        if (summation != 0){
          eline = (w_left - w_right)/summation;
        } else {
          eline = (w_left - w_right);
        }

        if (eline < 0.1 and eline > -0.1){
          eline = 0;  
        }
        if (values_calibrated[0] > 1000 and values_calibrated[4] < 1000 ){
          eline = 2;  
        } else if (values_calibrated[4] > 1000 and values_calibrated[0] < 1000 ){
          eline = -2;  
        } else if (values_calibrated[0] > 1000 and values_calibrated[4] > 1000 ){
          eline = 10;  
        }
        
        if (serial_verbose){
          Serial.println(eline);  
        }
        last_eline = eline;
        return eline;
      }
    }

    void readLineSensorsCalibrated(bool serial_verbose){
      readLineSensors(false);  
      for (int s = 0; s < n_sensors; s++){
        int aux = (values[s] - (calibration_min[s] + calibration_max[s])/2);
        if (aux > 2000){ 
          aux = 2000;  
        } else if (aux < 0){
          aux = 0;  
        }
        values_calibrated[s] = aux;
      }
      
      if (serial_verbose){
        Serial.print(values_calibrated[0]); Serial.print(",");
        Serial.print(values_calibrated[1]); Serial.print(",");
        Serial.print(values_calibrated[2]); Serial.println(",");
      }
    }

    void readLineSensors(bool serial_verbose){
      values[0] = timeout;
      values[1] = timeout;
      values[2] = timeout;
      values[3] = timeout;
      values[4] = timeout;  

      for (int pin = 0; pin < n_sensors; pin ++){
        chargeCapacitors(pins_names[pin]);  
      }

      unsigned long tick = micros();
      unsigned long delta_time = 0;
      int remaining = n_sensors;
      while (true and remaining > 0){
        delta_time = micros() - tick;
        if (delta_time >= timeout){
          break;  
        }
        for (int pin = 0; pin < n_sensors; pin ++){
          if ( digitalRead(pins_names[pin]) == LOW and delta_time < values[pin]){
            values[pin] = delta_time;
            remaining--;
          }
        }
      }

      if (serial_verbose){
        Serial.print(values[0]); Serial.print(",");
        Serial.print(values[1]); Serial.print(",");
        Serial.print(values[2]); Serial.println(",");
      }
    }

    void chargeCapacitors(int pin){

      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
      delayMicroseconds(10);
      pinMode(pin, INPUT);
      
    }

    void calibrateSensors(){
      for (int i = 0; i < 10; i ++){
        readLineSensors(false);
        for (int s = 0; s < n_sensors; s++){
          if (values[s] < calibration_min[s]){
            calibration_min[s] = values[s];  
          }
          if (values[s] > calibration_max[s]){
            calibration_max[s] = values[s]; 
          }  
        }  
      }
      calibration_done = true;
    }

    void calibrateSensorsBlack(){
      for (int i = 0; i < 10; i ++){
        readLineSensors(false);
        for (int s = 0; s < n_sensors; s++){
          if (values[s] < calibration_min_black[s]){
            calibration_min_black[s] = values[s];  
          }
          if (values[s] > calibration_max_black[s]){
            calibration_max_black[s] = values[s]; 
          }  
        }  
      }
      updateCalibration();
      calibration_done = true;
    }

    void updateCalibration(){
      for (int s = 0; s < n_sensors; s++){
        calibration_offset[s] = (calibration_min[s] + calibration_max[s])/2;
        calibration_range[s] =  calibration_min_black[s] -  calibration_offset[s];
      }
    }

};



#endif
