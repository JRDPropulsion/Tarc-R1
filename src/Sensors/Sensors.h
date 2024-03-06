/*
  Sensors - All functions relating to sensor communication and data processing
*/


#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// Global variable definitions
extern double gyro_x, gyro_y, gyro_z, gyro_x_cal, gyro_y_cal, 
              gyro_z_cal, body_acc_x, body_acc_y, body_acc_z, 
              baro_alt_raw, baro_alt_filtered, baro_alt_cal, 
              baro_alt_filtered, 
              accel_x_cal, accel_y_cal, accel_z_cal, accel_bais_x, accel_bais_y, accel_bais_z, accel_x_offset, accel_y_offset, accel_z_offset, accel_z_cal_filt;

void sensors_init();
void sensors_update();
void sensors_bias();

#endif