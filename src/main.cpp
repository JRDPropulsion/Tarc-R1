/*
  JRD Propulsion TARC software (flight version 0.1)
  By: Aryan Kapoor
  Last updated on: 3/6/24
*/

// Importing libraries
#include <Arduino.h>
#include <Config.h>
#include <Sensors.h>
#include <State.h>
#include <NeoPixel.h>
#include <Servo.h>
#include <SD.h>

// Main file initializations
Servo servo_z;
uint64_t current_time = 0;
uint64_t previous_time = 0;
File dataFile; // Declare a global file object
double prevTime;

/*
  Zero out the servo
*/
void servo_zero() 
{
  servo_z.write(servo_z_offset);
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
  Write data to the microSD card reader
*/
void sd_write() 
{
  // make a string for assembling the data to log
  String dataString = "";

  // if the file is available, write to it
  if (dataFile)   
  {  
    // Time
    dataFile.print("Time since startup:");
    dataFile.print("\t");
    dataFile.print(String(mission_time, 8));
    dataFile.print("\t");
    dataFile.print("Time since liftoff:");
    dataFile.print("\t");
    dataFile.print(String(mission_time_liftoff, 8));
    dataFile.print("\t");
    
    // Altitude
    dataFile.print("Raw altitude:");
    dataFile.print("\t");
    dataFile.print(String(baro_alt_cal));
    dataFile.print("\t");
    dataFile.print("Filtered altitude: ");
    dataFile.print("\t");
    dataFile.print(String(baro_alt_cal));
    dataFile.print("\t");

    // State
    dataFile.print("State:");
    dataFile.print("\t");
    dataFile.print(String(state));
    dataFile.print("\t");

    // Acceleration
    dataFile.print("local x-axis acceleration:");
    dataFile.print("\t");
    dataFile.print(String(body_acc_x));
    dataFile.print("\t");
    dataFile.print("local y-axis acceleration:");
    dataFile.print("\t");
    dataFile.print(String(body_acc_y));
    dataFile.print("\t");
    dataFile.print("local z-axis acceleration:");
    dataFile.print("\t");
    dataFile.print(String(body_acc_z));

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
  Deploy parachute
*/
void deploy_parachute(double baro_alt_filtered) 
{
  // subtract 5 meters from setpoint
  if ((baro_alt_filtered >= alt_setpoint) || (mission_time_liftoff >= 8)) 
  {
    servo_z.write((servo_z_offset + 70));
    NeoPixel_yellow();
    state++;
  } 
}

/*
  Detect that the vehicle has landed
*/
void detect_landing(double dt) 
{
  // Time since parachutes deployed
  mission_time_descent += dt;

  // Wait 60 seconds since descent is detected
  if (mission_time_descent >= 60) 
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
  pinMode(chip_select, OUTPUT);
  servo_z.attach(servo_z_pin);

  // Start serial monitor (use SerialUSB)
  SerialUSB.begin(9600);
  delay(2000); // Wait for serial monitor (if connected)

  // Check to see if micro SD card is inserted
  if (!SD.begin(chip_select)) {
    SerialUSB.print("Card failed, or not present");
    return;
  }

  SerialUSB.println("card initialized");

  // Check if data logging file exists and if it does, delete it
  if (SD.exists("datalog_tarc_r1.txt")) 
  {
    SD.remove("datalog_tarc_r1.txt");
  }

  // Initialization functions
  sensors_init();

  dataFile = SD.open("datalog_tarc_r1", FILE_WRITE);

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

  // Calibrartion state 0
  if (state == 0) 
  {
    sensors_bias();
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
    sd_write();
    deploy_parachute(baro_alt_filtered);
  }

  // Descent state 3
  if (state == 3) 
  {
    detect_landing(dt); // Timer
    sd_write(); // Still data logging!
  }

  // Landed state 4
  if (state == 4) 
  {
    close_datalog(); // Save the file
  }
}