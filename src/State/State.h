/*
  State - Includes all functions responsible for state transitions in main state machine
*/


#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

void ascent_detect();
void descent_detect(double dt);

#endif