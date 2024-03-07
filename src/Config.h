/*
  Configurations - Holds global configuration values
*/

#ifndef CONFIG_H
#define CONFIG_H

// Importing libraries
#include <Arduino.h>

// Pins
int NeoPixel = 1; 
int servo_z_pin = 5;
int servo_y_pin = 2;
int servo_1_pin = 0;
int chip_select = 10;

// Time
double mission_time_liftoff;
double mission_time_descent;
double mission_time;
double dt = 0;

// Servo
int servo_z_offset = 150;
int servo_y_offset = 110;

// Setpoints
int state = 0; // Make sure that this value is 0 before launch
float ascent_threshold = 11.2; // DO NOT CHANGE
float alt_setpoint = 250; // In meters

#endif