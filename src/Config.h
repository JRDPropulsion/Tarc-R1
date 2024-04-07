/*
  Configurations - Holds global configuration values
*/

#ifndef CONFIG_H
#define CONFIG_H

// Importing libraries
#include <Arduino.h>

// Pins
int servo_z_pin = 1;
int chip_select = 2;

// Time
double mission_time;
double mission_time_liftoff;
double mission_time_descent;
double dt;

// Servo
int servo_z_offset = 100; //using this

// Setpoints
int state = 0; // Make sure that this value is 0 before launch
float alt_setpoint = 225; // In meters (240 for flight 1), set to 225 for flight 7

#endif