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
    int16_t idx_adapt_robot = 0;
    int16_t t = 0;
    int16_t aux_i = 0;
    int16_t size_filtAWP = 0;
    int16_t x_path[100];
    // int16_t d_x_path[100];
    int16_t x_path_adapt[100];
    int16_t x_path_filt[100];
    int16_t y_path[100];
    // int16_t d_y_path[100];
    int16_t y_path_adapt[100];
    int16_t y_path_filt[100];
    // int16_t d_wp[100];
    float theta_path[100];
    int16_t j = 0; // counter for generating the adaptive waypoint array
    // float theta_path_adapt[100];
  
    // Constructor, must exist.
    /*
     * We initialize the full path with 0 values, this allow us to measure the 
     * impact on the memory.
    */
    Waypoint_c() {
      for (int16_t i = 0; i < 100; i ++){
        x_path[i] = 0;
        //d_x_path[i] = 0;
        y_path[i] = 0;
        //d_y_path[i] = 0;
        //d_wp[i] = 0;
        theta_path[i] = 0;  
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

    void printPointsAdaptive(){
      for (int16_t i = 0; i < idx; i++){
        Serial.print(x_path_adapt[i]);
        Serial.print(", ");
        Serial.println(y_path_adapt[i]);
      }
    }

    void printPointsFiltered(){
      for (int16_t i = 0; i < idx-size_filtAWP; i++){
        Serial.print(x_path_filt[i]);
        Serial.print(", ");
        Serial.println(y_path_filt[i]);
      }
    }

    // void distanceWP(){
    //   if (idx_adapt_path < 100){
    //     for (int16_t i = 0; i < idx; i++ ){
    //       d_x_path[idx_adapt_path] = x_path[i+1] - x_path[i];
    //       d_y_path[idx_adapt_path] = y_path[i+1] - y_path[i];
    //       d_wp[idx_adapt_path] = sqrt(pow(d_x_path[idx_adapt_path],2)+pow(d_y_path[idx_adapt_path],2));
    //       idx_adapt_path++;
    //     }
    //   } 
    // }

    // void angleWaypoint(){
    //   if (idx_theta < 100){
    //     distanceWP();
    //     for (int16_t i = 0; i < idx_adapt_path; i++){
    //       theta_path_adapt[idx_theta] = float(atan2(double(d_y_path[i]),double(d_x_path[i])));
    //       idx_theta++;
    //     }
    //   }
    // }

    // This function reduces the number of waypoints in the path
    // void adaptiveWP(){
    //   // Store the first waypoint as point of reference
    //   if (idx_adapt_robot < idx){
    //     if (idx_adapt_robot < idx - 1){
    //       if (idx_adapt_robot == 0){
    //         x_path_adapt[0] = x_path[0];
    //         y_path_adapt[0] = y_path[0];
    //       } else {
    //         if (abs(theta_path[idx_adapt_robot] - theta_path[idx_adapt_robot - 1]) >= 0.04){
    //           x_path_adapt[idx_adapt_robot] = x_path[idx_adapt_robot];
    //           y_path_adapt[idx_adapt_robot] = y_path[idx_adapt_robot];
    //         }   
    //       }
    //     } else {
    //       x_path_adapt[idx_adapt_robot] = x_path[idx_adapt_robot];
    //       y_path_adapt[idx_adapt_robot] = y_path[idx_adapt_robot];
    //     }
    //     idx_adapt_robot++;
    //   }

      
    // }
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
          if (abs(theta_path[i] - theta_path[i-1]) >= 0.04) {
            x_path_adapt[aux_i] = x_path[i];
            y_path_adapt[aux_i] = y_path[i];
            aux_i++;
          }
        }
      }
    }

//     void removedWP(){
//       // function to get the number of removed waypoints
//       for (int16_t i = 0; i < idx; i++){
//         if (x_path_adapt[i] == 0 && y_path_adapt[i] == 0){
//           size_filtAWP++;
//         }
//       }
//     }

//     void filtered_adaptiveWP(){
//       // Initialise the first values of the filtered array in x and y
//       removedWP();
//       x_path_filt[0] = x_path_adapt[0];
//       x_path_filt[0] = x_path_adapt[0];
//       // Creation of the filtered array
//       for (int16_t i = 1; i < idx; i++){
//         // If the values seen in the adaptive waypoint arrays are null, they
//         // need to be disguarded, hence replace the value of the current row
//         // with the value of the following row.
//         if (x_path_adapt[i] == 0 && y_path_adapt[i] == 0){
//           if (j < i){
//             j = i + 1;
//           } else {
//             j++;
//           }
          
//           // This while loop was designed to continue looping until one of the values
//           // of the x and y arrays are non-zero
//           while (x_path_adapt[j] == 0 && y_path_adapt[j] == 0){
//             x_path_filt[i] = x_path_adapt[j];
//             y_path_filt[i] = y_path_adapt[j];
//             // If the value of x array or y array is non-zero, exit the while loop 
//             // and go to the next iteration in the for loop.
//             if (x_path_filt[i]!=0 || y_path_filt[i]!=0){
//               break;
//             // If the value of x array and y array are still null, remain in the 
//             // while loop until it finds non-zero values
//             } else {
//               j++;
//             }
//           }
//         } else {
//           if (j > i){
//             x_path_filt[i] = x_path_adapt[j+1];
//             y_path_filt[i] = y_path_adapt[j+1];
//           } else {
//             x_path_filt[i] = x_path_adapt[i];
//             y_path_filt[i] = y_path_adapt[i];
//           }
//         }
//         // It is expected that the size of the filtered array will be smaller than the 
//         // adapted waypoint arrays. If the value of j, the counter that goes through the
//         // AW arrays, exceeds the size of these arrays, exit the for loop and send the 
//         // results in the serial monitor (another function will make this possible.)
//         if (j > idx){
//           break; // limit the size of the filtered array
//         }
//       }
//     }

};



#endif