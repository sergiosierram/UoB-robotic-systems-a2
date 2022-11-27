// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _WAYPOINT_H
#define _WAYPOINT_H

// Class to contain generic PID algorithm.
class Waypoint_c {
  public:
  
    int16_t idx = 0;
    int16_t t = 0;
    int16_t x_path[100];
    int16_t y_path[100];
  
    // Constructor, must exist.
    Waypoint_c() {
      for (int16_t i = 0; i < 100; i ++){
        x_path[i] = 0;
        y_path[i] = 0;  
      }

    }

    void savePoint(int16_t x, int16_t y){
      if (idx < 100){
        x_path[idx] = x;
        y_path[idx] = y;
        idx++;
      } else {
        Serial.println("Skipping");
      }
    }

    void printPoints(){
      for (int16_t i = 0; i < idx; i++ ){
        Serial.print(x_path[i]);
        Serial.print(", ");
        Serial.println(y_path[i]);
      }  
    }

};



#endif
