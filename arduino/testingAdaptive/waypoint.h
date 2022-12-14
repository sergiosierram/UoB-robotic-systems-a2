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
    int16_t t = 0;
    int16_t x_path[100];
    int16_t y_path[100];
    float theta_path[100];
  
    // Constructor, must exist.
    /*
     * We initialize the full path with 0 values, this allow us to measure the 
     * impact on the memory.
    */
    Waypoint_c() {
      for (int16_t i = 0; i < 100; i ++){
        x_path[i] = 0;
        y_path[i] = 0;  
      }

    }

    /*
     * This function saves the path points that were received as input.
     * To prevent memory overflow we limit the recording to 100 samples,
     * as this is the max size of each array.
     * Idx variable keeps track of the current position of the array, where the 
     * next point needs to be saved.
    */
    void savePoint(int16_t x, int16_t y, float theta){
      if (idx < 100){
        x_path[idx] = x;
        y_path[idx] = y;
        theta_path[idx] = theta;
        idx++;
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
        Serial.print(y_path[i]);
        Serial.print(", ");
        Serial.println(theta_path[i]);
      }  
    }

};



#endif