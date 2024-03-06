/*
  Navigation - includes orientation processing and PID controller
*/


/*
  Importing libraries
*/
#include <Arduino.h>
#include <Sensors/Sensors.h>
#include <Orientation.h>
#include <Quaternion.h>
#include <Configuration/Config.h>

double global_ori_x, global_ori_y, global_ori_z, error_z, error_y, prev_error_z, prev_error_y,
       pid_z, pid_y, pid_z_p, pid_y_p, pid_z_i, pid_y_i, pid_z_d, pid_y_d, tvc_output_z, tvc_output_y, servo_z_pwm, servo_y_pwm, 
       alt, alt_max, global_acc_z, global_acc_z_bias, phi, theta, psi, pos_z, vel_z, descent_time;


unsigned long previousTime = 0;  // Variable to store the previous time
unsigned long delayTime = 10;  // Delay time in milliseconds

Orientation ori; // Main orientation measurement
EulerAngles oriMeasure; // Quaternion converted to Euler Angles for maths etc.

Quaternion orientation(1, 0, 0, 0);
Quaternion measuredGravity;
Quaternion worldGravity;
double aX, aY, aZ;
double oriZ, oriX, oriY;
double estimate;


/*
  PID controller
*/
void PID_compute(double dt) 
{
  // Set the orientation in the global frame as the error input
  error_z = global_ori_z;
  error_y = global_ori_y;
  
  // Proportional term
  pid_z_p = kp * error_z;
  pid_y_p = kp * error_y;

  // Intergral term
  pid_z_i = ki * (pid_z_i + error_z * dt);
  pid_y_i = ki * (pid_y_i + error_y * dt);

  // Derivative term
  pid_z_d = kd * ((error_z - prev_error_z) / dt);
  pid_y_d = kd * ((error_y - prev_error_y) / dt);

  // Sum of PID outputs
  pid_z = (pid_z_p + pid_z_i + pid_z_d);
  pid_y = (pid_y_p + pid_y_i + pid_y_d);

  // Deroll PID outputs
  double cos_roll = cos(-(global_ori_x * DEG_TO_RAD));
  double sin_roll = sin(-(global_ori_x * DEG_TO_RAD));

  tvc_output_z = pid_y * cos_roll - pid_z * sin_roll;
  tvc_output_y = pid_y * sin_roll + pid_z * cos_roll;

  servo_z_pwm = (tvc_output_z * servo_ratio_z) + servo_z_offset;
  servo_y_pwm = (tvc_output_y * servo_ratio_y) + servo_y_offset;

  // Set previous error as current error
  prev_error_z = error_z;
  prev_error_y = error_y;
}


/*
  Get position data by globalizing acceleration data and double integrating it
*/
void accel_position(double dt) {

  // Find the Z-axis acceleration in the global reference frame
  global_acc_z = -aZ;
  
  // Remove gravity component from global Z-axis acceleration
  global_acc_z -= global_acc_z_bias;

  // During sensor calibration, set the bias so we can later remove the gravity component
  if (state == 0) {
    global_acc_z_bias = global_acc_z;
  }

  // When pad idle, keep first measurements ready but do not integrate yet!
  if (state == 1) {
    //global_acc_z_bias = global_acc_z;
    vel_z = global_acc_z * dt;
    pos_z = vel_z * dt;
  }

  // Ok, start integrating, we have liftoff!
  if (state > 1) {
    vel_z += global_acc_z * dt;
    pos_z += vel_z * dt;
    
    estimate = (0.7 * pos_z) + (0.3 * baro_alt_cal);
  }

  // Test output
  SerialUSB.print("Global Z-axis acceleration: ");
  SerialUSB.print(global_acc_z);
  SerialUSB.print("     Position: ");
  SerialUSB.println(pos_z);
}



void inertialAccel(double body_acc_z, double body_acc_y, double body_acc_x, double global_ori_z, double global_ori_y, double global_ori_x, double dt) 
{
  oriX += gyro_x_cal;
  oriY += gyro_y_cal;
  oriZ += gyro_z_cal;

  double oriNorm = sqrt((oriX * oriX) + (oriY * oriY) + (oriZ * oriZ));
  oriNorm = max(abs(oriNorm), 1e-12);

  orientation = Quaternion::from_axis_angle(dt * oriNorm, oriX / oriNorm, oriY / oriNorm, oriZ / oriNorm);

  measuredGravity = Quaternion(0.0, accel_x_cal, accel_y_cal, body_acc_z);
  worldGravity = orientation.rotate(measuredGravity);

  aX = worldGravity.b;
  aY = worldGravity.c;
  aZ = worldGravity.d;
}


/*
  Detect apogee
*/
void max_altitude() {

  // alt_max can only increase but never decrease
  if (millis() - previousTime >= delayTime) {
    previousTime = millis();  // Update the previous time

    alt = pos_z;

    // Update alt_max to the current altitude if it's higher
    if (alt > alt_max) {
      alt_max = alt;
    }
  }
}


/*
  Update all navigation
*/
void nav_update(double dt)
{
    // ORDER MATTERS!!! I have no clue why this is the order but it works ¯\_(ツ)_/¯
    ori.update(gyro_y_cal, gyro_x_cal, gyro_z_cal, dt); // Input should be in rad/s
    oriMeasure = ori.toEuler();
    
    /*
      This is where the magic actually happens
        
      The order of your axis measurements (x, y, z) will depend on your sensor, your reference frame, and your IMU library of choice
      Swap & invert your gyro measurements so that .update() is called with (yaw, pitch, roll, dt) in that order
        
      All gyro measurements must be measured right-handed (positive = yaw left, pitch down, roll right) and coverted to radians/sec
    */

    // Set axis reversal here if needed
    global_ori_z = oriMeasure.yaw * RAD_TO_DEG;
    global_ori_y = oriMeasure.pitch * RAD_TO_DEG;
    global_ori_x = oriMeasure.roll * RAD_TO_DEG;

    // Updates the max alt function
    inertialAccel(body_acc_z, body_acc_y, body_acc_x, global_ori_z, global_ori_y, global_ori_x, dt);
    accel_position(dt);
    max_altitude();
}