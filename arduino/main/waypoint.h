// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _WAYPOINT_H
#define _WAYPOINT_H

// Class to contain generic waypoint algorithm.
class Waypoint_c {
  public:
    /*
     * First, declare the public parameters of this class.
     * idx is used for saving the points in the arrays, keeps tracks of the position
     * t is used for tracking the "time" and decide when to save one point
     * x_path and y_path are the arrays to save the points
    */
    int16_t idx = 0;
    int16_t idx2 = 0;
    int16_t t = 0;
    int16_t t2 = 0;
    int16_t aux_i = 0;
    int16_t x_path[150];
    int16_t y_path[150];
    int16_t x_control[150];
    int16_t y_control[150];
    int16_t x_path_adapt[100];
    int16_t y_path_adapt[100];
    int16_t theta_path[150];
  
    // Constructor, must exist.
    /*
     * We initialize the full path with 0 values, this allow us to measure the 
     * impact on the memory.
    */
    Waypoint_c() {

    }

    /*
     * This function saves the path points that were received as input.
     * To prevent memory overflow we limit the recording to 100 samples,
     * as this is the max size of each array.
     * Idx variable keeps track of the current position of the array, where the 
     * next point needs to be saved.
    */
    void savePoint(int16_t x, int16_t y, int16_t theta){
      if (idx < 100){
        x_path[idx] = x;
        y_path[idx] = y;
        theta_path[idx] = theta;
        idx++;
      } else {
        Serial.println("Skipping");
      }
    }

    void saveControlPoint(int16_t x, int16_t y){
      if (idx2 < 100){
        x_control[idx2] = x;
        y_control[idx2] = y;
        idx2++;
      } else {
        Serial.println("Skipping");
      }
    }

    
    /*
     * This function prints all the stored points
    */
    void printPoints(){
      for (int16_t i = 0; i < idx; i++ ){
        Serial.print(x_path[i]);
        Serial.print(", ");
        Serial.println(y_path[i]);
      }  
    }

    void printControlPoints(){
      for (int16_t i = 0; i < idx2; i++ ){
        Serial.print(x_control[i]);
        Serial.print(", ");
        Serial.println(y_control[i]);
      }  
    }

    void printPointsAdaptive(){
      for (int16_t i = 0; i < idx; i++){
        Serial.print(x_path_adapt[i]);
        Serial.print(", ");
        Serial.println(y_path_adapt[i]);
      }
    }

    void adaptiveWP(){
      for (int16_t i = 0; i < idx; i++){
        if (i == 0){
          x_path_adapt[aux_i] = x_path[i];
          y_path_adapt[aux_i] = y_path[i];
          aux_i++;
        } else if (i == idx - 1) {
          x_path_adapt[aux_i] = x_path[i];
          y_path_adapt[aux_i] = y_path[i];
          aux_i++;
        } else {
          if (abs(theta_path[i]/1000.0 - theta_path[i-1]/1000.0) >= 0.04) {
            x_path_adapt[aux_i] = x_path[i];
            y_path_adapt[aux_i] = y_path[i];
            aux_i++;
          }
        }
      }
    }

};



#endif
