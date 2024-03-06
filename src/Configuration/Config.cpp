/*
  Configurations - Holds global configuration values
*/


/*
  Importing libraries
*/
#include <Arduino.h>


// Pins
int NeoPixel = 1; 
int servo_z_pin = 5;
int servo_y_pin = 2;
int servo_1_pin = 0;
int pyro1_pin = 4;
int pyro2_pin = 3;
int pyro1_cont_pin = A0;
int pyro2_cont_pin = A2;
int chip_select = 10;

// Time
double mission_time_liftoff;
double mission_time_descent;
double mission_time;
double dt = 0;

// Servo
int servo_z_offset = 150; // 150 for main gimble
int servo_y_offset = 110; // 110 for main gimble
float servo_ratio_z = 2.9;
float servo_ratio_y = 2.9;

// Gains/values
int state = 0; // Make sure that this value is 0 before launch
float ascent_threshold = 11.2; // DO NOT CHANGE
float kp = 0.50;
float ki = 0.05;
float kd = 0.15;
float kp_landing = 0.90;
float ki_landing = 0.05;
float kd_landing = 0.20;

// Enable/disable
bool pyro1_enable = true;
bool pyro2_enable = true;

double vel_x_bias, pos_x_bias;
