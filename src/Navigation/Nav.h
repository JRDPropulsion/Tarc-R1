/*
  Navigation - includes orientation processing and all other state calculations
*/


#ifndef NAV_H
#define NAV_H

#include <Arduino.h>

extern double global_ori_x, global_ori_y, global_ori_z, 
              pid_z, pid_y, servo_z_pwm, servo_y_pwm, 
              alt, alt_max, global_acc_z, vel_z, pos_z, 
              descent_time, estimate;

void nav_update(double dt);
void PID_compute(double dt);
void accel_position(double dt);
void inertialAccel(double body_acc_z, double body_acc_y, double body_acc_x, double global_ori_z, double global_ori_y, double global_ori_x, double dt);
void max_altitude();

#endif