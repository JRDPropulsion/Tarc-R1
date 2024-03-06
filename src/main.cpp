/*
  JRD Propulsion TVC landing software (flight version 1.4)
  By: Aryan Kapoor
  Last updated on: 3/5/24

  NOTES:
    - Z = yaw, Y = pitch, X = roll
*/


/*
  Importing libraries
*/
#include <Arduino.h>
#include <Configuration/Config.h>
#include <Sensors/Sensors.h>
#include <Navigation/Nav.h>
#include <State/State.h>
#include <Pyro/Pyro.h>
#include <NeoPixel/NeoPixel.h>
#include <Servo.h>
#include <SD.h>


/*
  Main file initializations
*/
Servo servo_z;
Servo servo_y; 
Servo servo_1;
uint64_t current_time = 0;
uint64_t previous_time = 0;
File dataFile; // Declare a global file object
double prevTime;


/*
  Zero out both of the servos to center the mount
*/
void servo_zero() 
{
  servo_z.write(servo_z_offset);
  servo_y.write(servo_y_offset);
}


/*
  Keep track of all time
*/
void time_keeper() 
{
  /*
    Calculate delta time (difference in time across every loop)
  */
  current_time = millis(); // Get new microsecond timestamp for this loop
  dt = (double)(current_time - previous_time) / 1000.0f; // Output is delta time in seconds
  previous_time = current_time; // We have updated, set the new timestamp

  mission_time += dt; // Calculate the elapsed time since startup

  // Calculate mission time only after liftoff
  if (state > 1) 
  {
    mission_time_liftoff += dt;
  }
}


/*
  Write data to the SD card reader (needs a redesign to work as csv file)
*/
void sd_write() 
{
  // make a string for assembling the data to log:
  String dataString = "";

  // if the file is available, write to it:
  if (dataFile)   
  {  
    // Log time data
    dataFile.print("Time_startup:");
    dataFile.print("\t");
    dataFile.print(String(mission_time, 8));
    dataFile.print("\t");
    dataFile.print("Time_liftoff:");
    dataFile.print("\t");
    dataFile.print(String(mission_time_liftoff, 8));
    dataFile.print("\t");

    // Log global orientation data
    dataFile.print("Global_ori_z(yaw):");
    dataFile.print("\t");
    dataFile.print(String(global_ori_z));
    dataFile.print("\t");
    dataFile.print("Global_ori_y(pitch):");
    dataFile.print("\t");
    dataFile.print(String(global_ori_y));
    dataFile.print("\t");
    dataFile.print("Global_ori_x(roll):");
    dataFile.print("\t");
    dataFile.print(String(global_ori_x));
    dataFile.print("\t");

      // PID output
    dataFile.print("PID_z:");
    dataFile.print("\t");
    dataFile.print(String(pid_z));
    dataFile.print("\t");
    dataFile.print("PID_y:");
    dataFile.print("\t");
    dataFile.print(String(pid_y));
    dataFile.print("\t");
    
    // Altitude
    dataFile.print("Baro_alt:");
    dataFile.print("\t");
    dataFile.print(String(baro_alt_cal));
    dataFile.print("\t");
    dataFile.print("pos_z");
    dataFile.print("\t");
    dataFile.print(String(pos_z));
    dataFile.print("\t");

    // State
    dataFile.print("state:");
    dataFile.print("\t");
    dataFile.print(String(state));
    dataFile.print("\t");

    // Raw acceleration
    dataFile.print("Estimate:");
    dataFile.print("\t");
    dataFile.print(String(estimate));
    dataFile.print("\t");
    dataFile.print("body_acc_z:");
    dataFile.print("\t");
    dataFile.print(String(body_acc_z));
    dataFile.print("\t");
    dataFile.print("Global_acc_z:");
    dataFile.print("\t");
    dataFile.print(String(global_acc_z));

    dataFile.println(); //create a new row to read data more clearly
  }  
}


/*
  Close the datalog file and save it
*/
void close_datalog()
{
  // Close the data log file when you're done logging
  if (dataFile) {
    dataFile.close();
    SerialUSB.println("File closed");
  }
}


/*
  Update PID
*/
void PID_update(double dt)
{
  // Update PID
  PID_compute(dt);

  // DO NOT set axis reversal here
  servo_z.write(servo_z_pwm);
  servo_y.write(servo_y_pwm);
}


/*
  Switch over to the PID gains optimized for landing
*/
void PID_landing() 
{
 kp = kp_landing;
 ki = ki_landing;
 kd = kd_landing;
}


/*
  Calculate landing motor ignition altitude
*/
void landing_motor_ignition()
{
  // Linear function that calculates the optimal altitude to ignite the landing motor
  float ignition_alt = (0.66 * alt_max);

  // If current altitude is less than the altitude from which we should fire the landing motor
  if (pos_z <= ignition_alt)
  {
    NeoPixel_purple();
    PID_landing(); // Switch over to the landing PID gains
    state++;
  }
}


/*
  Detect that the vehicle has landed
*/
void detect_landing(double dt) 
{
  // Time since landing motor is ignited
  mission_time_descent += dt;

  // Wait 5 seconds since descent is detected
  if (mission_time_descent >= 5) 
  {
    NeoPixel_white();
    state++;
  }
}


/*
  Setup function
*/
void setup() 
{
  // Do LED stuff first
  NeoPixel_init();
  NeoPixel_red();

  // Set outputs
  pinMode(chip_select, OUTPUT); // Have to do this
  servo_z.attach(servo_z_pin);
  servo_y.attach(servo_y_pin);
  servo_1.attach(servo_1_pin);

  // Start serial monitor (use SerialUSB)
  SerialUSB.begin(9600);
  delay(2000); // Wait for serial monitor (if connected)

  // Check to see if micro SD card is inserted
  if (!SD.begin(chip_select)) {
    SerialUSB.print("Card failed, or not present");
    return;
  }

  SerialUSB.println("card initialized.");

  // Check if data logging file exists and if it does, delete it
  if (SD.exists("datalog.txt")) 
  {
    SD.remove("datalog.txt");
  }

  // Initialization functions
  sensors_init();
  pyro_init();

  dataFile = SD.open("datalog.txt", FILE_WRITE);

  // Check if the file opened successfully
  if (dataFile) {
    SerialUSB.println("File opened successfully");
  } else {
    SerialUSB.println("Error opening file");
  }

  // Start the first part of the delta time calculation
  previous_time = current_time = millis();
}


/*
  Main Loop function
*/
void loop() 
{
  // Update functions that run no matter what:
  time_keeper();
  sensors_update();
  nav_update(dt);

  // Sensor calibrartion state 0
  if (state == 0) 
  {
    sensors_bias(); // manually calibrate the sensors by finding the bias and zero them out after
  }

  // Pad idle state 1
  if (state == 1) 
  {
    servo_zero();
    ascent_detect();
  }

  // Powered ascent state 2
  if (state == 2) 
  {
    PID_update(dt);
    descent_detect(dt);
    sd_write();
  }

  // Descent state 3
  if (state == 3) 
  {
    servo_zero();
    landing_motor_ignition();
    sd_write(); // Still data logging!
  }

  // Powered descent state 4
  if (state == 4)
  {
    pyro1_fire(dt); // Fire here so it doesnt delay the code
    PID_update(dt);
    detect_landing(dt);
    sd_write(); // Still data logging!
  }

  // Landed state 5
  if (state == 5) 
  {
    close_datalog(); // Save the file
    servo_zero();
  }
}