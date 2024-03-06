/*
  Sensors - All functions to get sensor data
*/


/*
  Importing libraries
*/
#include <Arduino.h>
#include <Configuration/Config.h>
#include <Wire.h>
#include <BMI088.h>
#include <MS56XX.h>
#include <NeoPixel/NeoPixel.h>
#include <SimpleKalmanFilter.h>

/*
  Initialization and definitions
*/
Bmi088Accel accel(Wire,0x18);
Bmi088Gyro gyro(Wire,0x68); // Set to 0x69 or 0x68 depending on board used
MS56XX MSXX(MS56XX_ADDR_LOW, MS5607);
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

double gyro_x, gyro_y, gyro_z, gyro_x_cal, gyro_y_cal, 
       gyro_z_cal, gyro_bais_x, gyro_bais_y, gyro_bais_z, 
       gyro_x_offset, gyro_y_offset, gyro_z_offset, 
       body_acc_x, body_acc_y, body_acc_z, baro_alt_raw, 
       baro_alt_filtered, baro_alt_bias, baro_alt_cal, baro_offset,
       accel_x_cal, accel_y_cal, accel_z_cal, accel_bais_x, accel_bais_y, accel_bais_z,
       accel_x_offset, accel_y_offset, accel_z_offset, accel_z_cal_filt;


/*
  Initialize all sensors
*/
void sensors_init() 
{
  Wire.begin();  
  accel.begin();
  gyro.begin();
  MSXX.begin();

  // Initialize sensors
  accel.setOdr(Bmi088Accel::ODR_800HZ_BW_80HZ);
  accel.setRange(Bmi088Accel::RANGE_6G); 
  gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
  gyro.setRange(Bmi088Gyro::RANGE_1000DPS);
  MSXX.configBaro(BARO_PRESS_D1_OSR_4096, BARO_TEMP_D2_OSR_4096);
}

/*
  Update all sensors
*/
void sensors_update() 
{
  gyro.readSensor();
  accel.readSensor();
  MSXX.doBaro(true); //Calculate pressure and temperature, boolean for altitude estimation from sea level 

  gyro_x = gyro.getGyroX_rads();
  gyro_y = gyro.getGyroY_rads();
  gyro_z = gyro.getGyroZ_rads();

  body_acc_x = accel.getAccelX_mss();
  body_acc_y = accel.getAccelY_mss();
  body_acc_z = accel.getAccelZ_mss(); 

  // Calibrated gyro measurements
  gyro_x_cal = (gyro.getGyroX_rads() - gyro_bais_x) + gyro_x_offset; // Calibrated x gyro measurement
  gyro_y_cal = (gyro.getGyroY_rads() - gyro_bais_y) + gyro_y_offset; // Calibrated y gyro measurement
  gyro_z_cal = (gyro.getGyroZ_rads() - gyro_bais_z) + gyro_z_offset; // Calibrated z gyro measurement

  accel_x_cal = (accel.getAccelX_mss() - accel_bais_x) + accel_x_offset; // Calibrated x gyro measurement
  accel_y_cal = (accel.getAccelY_mss() - accel_bais_y) + accel_y_offset; // Calibrated y gyro measurement
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
  float gyro_x_sum = 0;
  float gyro_y_sum = 0;
  float gyro_z_sum = 0;
  float accel_x_sum = 0;
  float accel_y_sum = 0;
  float accel_z_sum = 0;
  float baro_sum = 0;

  int count_gyro = 0;
  int count_accel = 0;
  int count_baro = 0;
  int total_count_gyro = 1000;
  int total_count_accel = 10000;
  int total_count_baro = 40;

  // Calibrate gyros
  while (count_gyro < total_count_gyro)
  {
    // Intergrate the measurements
    gyro_x_sum += gyro_x;
    gyro_y_sum += gyro_y;
    gyro_z_sum += gyro_z;

    count_gyro += 1;
  }

  // Calibrate accel
  while (count_accel < total_count_accel)
  {
    // Intergrate the measurements
    accel_x_sum += body_acc_x;
    accel_y_sum += body_acc_y;
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

  if ((count_baro == total_count_baro) && (count_gyro == total_count_gyro) && (count_accel == total_count_accel))
  {
    // Get average
    gyro_bais_x = gyro_x_sum / total_count_gyro;
    gyro_bais_y = gyro_y_sum / total_count_gyro;
    gyro_bais_z = gyro_z_sum / total_count_gyro;
    accel_bais_x = accel_x_sum / total_count_accel;
    accel_bais_y = accel_y_sum / total_count_accel;
    accel_bais_z = accel_z_sum / total_count_accel;
    baro_alt_bias = baro_sum / total_count_baro;

    // Calibrated measurements without offset
    gyro_x_cal = (gyro_x - gyro_bais_x);
    gyro_y_cal = (gyro_y - gyro_bais_y);
    gyro_z_cal = (gyro_z - gyro_bais_z);
    accel_x_cal = (body_acc_x - accel_bais_x);
    accel_y_cal = (body_acc_y - accel_bais_y);
    accel_z_cal = (body_acc_z - accel_bais_z);
    baro_alt_cal = (MSXX.altitude - baro_alt_bias);

    gyro_x_offset = -(gyro_x_cal);
    gyro_y_offset = -(gyro_y_cal);
    gyro_z_offset = -(gyro_z_cal);
    accel_x_offset = -(accel_x_cal);
    accel_y_offset = -(accel_y_cal);
    accel_z_offset = -abs(accel_z_cal);
    baro_offset = -abs(baro_alt_cal);

    SerialUSB.println("Sensors calibrated!");
    NeoPixel_green();
    state++;
  }
}