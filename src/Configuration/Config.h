/*
  Configurations - Holds global configuration values
*/


#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

extern int servo_z_pin, servo_y_pin, servo_1_pin, 
           servo_z_offset, servo_y_offset, state, 
           pyro1_pin, pyro2_pin, pyro1_cont_pin, 
           pyro2_cont_pin, NeoPixel, chip_select; 

extern float kp, kd, ki, kp_landing, ki_landing, kd_landing, servo_ratio_z, servo_ratio_y, ascent_threshold;

extern double mission_time_liftoff, mission_time_descent, mission_time, dt, vel_x_bias, pos_x_bias;

extern bool pyro1_enable, pyro2_enable;

#endif