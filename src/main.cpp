/*
  JRD Propulsion TARC software (flight version 0.2)
  By: Aryan Kapoor
  Last updated on: 4/3/24
*/

// Importing libraries
#include <Arduino.h>
#include <Config.h>
#include <Sensors.h>
#include <State.h>
#include <Servo.h>
#include <SD.h>

// Main file initializations
uint64_t current_time = 0;
uint64_t previous_time = 0;
File dataFile;

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

  // If current altitude is greater than 8 meters, we have liftoff!
  if (baro_alt_filtered >= 8) {
    mission_time_liftoff += dt;
  }
}

/*
  Initialize microSD card
*/
void sd_init() 
{
  Serial.print("Initializing SD card... ");

  if (!SD.begin(chip_select)) {
    Serial.println("initialization failed!");
    while (1) ;
  }

  Serial.println("card initialized");

  // Check if data logging file exists and if it does, delete it
  if (SD.exists("data.txt")) 
  {
    Serial.println("Deleting file");
    SD.remove("data.txt");
  }

  dataFile = SD.open("data.txt", FILE_WRITE);

  // Check if the file opened successfully
  if (dataFile) {
    Serial.println("File opened successfully");
  } else {
    Serial.println("Error opening file");
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
    
    // Altitude
    dataFile.print("Raw altitude:");
    dataFile.print("\t");
    dataFile.print(String(baro_alt_cal));
    dataFile.print("\t");
    dataFile.print("Filtered altitude: ");
    dataFile.print("\t");
    dataFile.print(String(baro_alt_filtered));
    dataFile.print("\t");

    // State
    dataFile.print("State:");
    dataFile.print("\t");
    dataFile.print(String(state));
    dataFile.print("\t");

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
    Serial.println("File closed");
  }
}

/*
  Setup function
*/
void setup() 
{
  // Start serial monitor
  Serial.begin(9600);
  delay(2000); // Wait for serial monitor (if connected)

  // Set outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);

  pinMode(chip_select, OUTPUT);
  servo_z.attach(servo_z_pin);

  // Initialization functions
  sensors_init();
  sd_init();

  // Start the first part of the delta time calculation
  previous_time = current_time = millis();
}

/*
  Main Loop function
*/
void loop() 
{
  max_altitude();

  // Update functions that run no matter what:
  time_keeper();
  sensors_update();

  // Calibration state 0
  if (state == 0) 
  {
    sensors_bias(); // Remove that bias!
    servo_zero();
  }

  // Pad idle state 1
  if (state == 1) 
  {
    sd_write();
    deploy_parachute(baro_alt_filtered); // Timer or max altitude
  }

  // Descent state 2
  if (state == 2) 
  {
    sd_write(); // Still data logging!
    detect_landing(dt); // Timer
  }

  // Landed state 3
  if (state == 3) 
  {
    close_datalog(); // Save the file
    state++;
  }

  if (state == 4) 
  {
    blinkLED(alt_max);
  }
}