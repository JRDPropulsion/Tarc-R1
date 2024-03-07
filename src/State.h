/*
  State - Includes all functions responsible for transitions in main state machine
*/

#ifndef STATE_H
#define STATE_H

// Importing libraries
#include <Arduino.h>
#include <Sensors.h>
#include <Config.h>
#include <NeoPixel.h>

double acc_1, acc_2, acc_3, acc_4, 
       acc_5, acc_6, acc_7, acc_8, 
       acc_9, acc_avg_1, acc_avg_2, 
       acc_avg_3, acc_mov;

/*
  Detect that the vehicle is ascending using a moving average of acceleration
*/
void ascent_detect() 
{
  acc_1 = body_acc_z;
  delay(1);
  acc_2 = body_acc_z;
  delay(1);
  acc_3 = body_acc_z;

  acc_avg_1 = (acc_1 + acc_2 + acc_3) / 3; // average of 3 acceleration values
  delay(5);

  acc_4 = body_acc_z;
  delay(1);
  acc_5 = body_acc_z;
  delay(1);
  acc_6 = body_acc_z;

  acc_avg_2 = (acc_4 + acc_5 + acc_6) / 3; // average of 3 acceleration values
  delay(5);

  acc_7 = body_acc_z;
  delay(1);
  acc_8 = body_acc_z;
  delay(1);
  acc_9 = body_acc_z;

  acc_avg_3 = (acc_7 + acc_8 + acc_9) / 3; // average of 3 acceleration values
  delay(5);

  // Determine the moving average of the z-axis acceleration
  acc_mov = (acc_avg_1 + acc_avg_2 + acc_avg_3) / 3;

  if (abs(acc_mov) > ascent_threshold) 
  {
    // Indicate the state
    NeoPixel_blue();
    state++;
  }
}

#endif