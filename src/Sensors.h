/*
  Sensors - All functions to get sensor data
*/

#ifndef SENSORS_H
#define SENSORS_H

// Importing libraries
#include <Arduino.h>
#include <Config.h>
#include <Wire.h>
#include <BMI088.h>
#include <MS56XX.h>
#include <NeoPixel.h>
#include <SimpleKalmanFilter.h>

// Initialization and definitions
Bmi088Accel accel(Wire,0x18);
MS56XX MSXX(MS56XX_ADDR_LOW, MS5607);
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

double gyro_x, gyro_y, gyro_z, body_acc_x, body_acc_y, body_acc_z, 
       baro_alt_raw, baro_alt_filtered, baro_alt_bias, baro_alt_cal, 
       baro_offset, accel_z_cal, accel_bais_z, accel_z_offset;

/*
  Initialize all sensors
*/
void sensors_init() 
{
  Wire.begin();  
  accel.begin();
  MSXX.begin();

  // Initialize sensors
  accel.setOdr(Bmi088Accel::ODR_800HZ_BW_80HZ);
  accel.setRange(Bmi088Accel::RANGE_6G); 
  MSXX.configBaro(BARO_PRESS_D1_OSR_4096, BARO_TEMP_D2_OSR_4096);
}

/*
  Update all sensors
*/
void sensors_update() 
{
  accel.readSensor();
  MSXX.doBaro(true); //Calculate pressure and temperature, boolean for altitude estimation from sea level 

  // Body frame acceleration (relative to orientation of the sensor)
  body_acc_x = accel.getAccelX_mss();
  body_acc_y = accel.getAccelY_mss();
  body_acc_z = accel.getAccelZ_mss(); 

  // Calibrated z-axis acceleration
  accel_z_cal = (accel.getAccelZ_mss() - accel_bais_z) + accel_z_offset; // Calibrated z gyro measurement

  // Barometer measurements
  baro_alt_raw = MSXX.altitude;
  baro_alt_cal = (MSXX.altitude - baro_alt_bias) + baro_offset;
  baro_alt_filtered = pressureKalmanFilter.updateEstimate(baro_alt_cal);
}

/*
  Finds the bias by taking the average of a set of measurements
*/
void sensors_bias() 
{
  // Initialize local sums as 0
  float accel_z_sum = 0;
  float baro_sum = 0;
  int count_accel = 0;
  int count_baro = 0;
  int total_count_accel = 10000;
  int total_count_baro = 40;

  // Calibrate accel
  while (count_accel < total_count_accel)
  {
    // Intergrate the measurements
    accel_z_sum += body_acc_z;
    count_accel += 1;
  }

  // Calibrate barometer
  while (count_baro < total_count_baro)
  {
    // Intergrate the measurements
    bool dataready = MSXX.doBaro(true) ;

    if (dataready == 1) {
      baro_sum += MSXX.altitude;
      count_baro += 1;
    }
  }

  if ((count_baro == total_count_baro) && (count_accel == total_count_accel))
  {
    // Get average
    accel_bais_z = accel_z_sum / total_count_accel;
    baro_alt_bias = baro_sum / total_count_baro;

    // Calibrated measurements without offset
    accel_z_cal = (body_acc_z - accel_bais_z);
    baro_alt_cal = (MSXX.altitude - baro_alt_bias);

    // Set offsets
    accel_z_offset = -abs(accel_z_cal);
    baro_offset = -abs(baro_alt_cal);

    SerialUSB.println("Sensors calibrated!");
    NeoPixel_green();
    state++;
  }
}

#endif