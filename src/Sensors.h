/*
  Sensors - All functions to get sensor data
*/

#ifndef SENSORS_H
#define SENSORS_H

// Importing libraries
#include <Arduino.h>
#include <Config.h>
#include <Wire.h>
#include <MS56XX.h>
#include <SimpleKalmanFilter.h>

// Initialization and definitions
MS56XX MSXX(MS56XX_ADDR_LOW, MS5611);
SimpleKalmanFilter pressureKalmanFilter(0.1, 0.1, 0.01);

double baro_alt_raw, baro_alt_cal, baro_alt_bias, 
       baro_offset, baro_alt_filtered;

/*
  Initialize all sensors
*/
void sensors_init() 
{
  Wire.begin();  
  MSXX.begin();

  // Initialize sensors
  MSXX.configBaro(BARO_PRESS_D1_OSR_4096, BARO_TEMP_D2_OSR_4096);
}

/*
  Update all sensors
*/
void sensors_update() 
{
  MSXX.doBaro(true); //Calculate pressure and temperature, boolean for altitude estimation from sea level 

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
  float baro_sum = 0;
  int count_baro = 0;
  int total_count_baro = 200;

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

  if (count_baro == total_count_baro)
  {
    // Get average
    baro_alt_bias = baro_sum / total_count_baro;

    // Calibrated measurements without offset
    baro_alt_cal = (MSXX.altitude - baro_alt_bias);

    // Set offsets
    baro_offset = -abs(baro_alt_cal);

    Serial.println("Sensors calibrated!");
    digitalWrite(LED_BUILTIN, LOW);
    state++;
  }
}

#endif