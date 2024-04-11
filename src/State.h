/*
  State - Includes all functions responsible for transitions in main state machine
*/

#ifndef STATE_H
#define STATE_H

// Importing libraries
#include <Arduino.h>
#include <Servo.h>
#include <Sensors.h>
#include <Config.h>

float alt_max, alt, digitCount;
unsigned long previousTime = 0;  // Variable to store the previous time
unsigned long delayTime = 10;  // Delay time in milliseconds
Servo servo_z;

/*
  Deploy parachute
*/
void deploy_parachute(double baro_alt_filtered) 
{
  if ((baro_alt_filtered >= alt_setpoint) || (mission_time_liftoff >= 6)) 
  {
    servo_z.write((servo_z_offset + 70));
    digitalWrite(LED_BUILTIN, HIGH);
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

  // Wait 120 seconds since descent is detected
  if (mission_time_descent >= 120) 
  {
    state++;
  }
}

/*
  Detect apogee
*/
void max_altitude() {

  // alt_max can only increase but never decrease
  if (millis() - previousTime >= delayTime) {
    previousTime = millis();  // Update the previous time

    alt = baro_alt_filtered;

    // Update alt_max to the current altitude if it's higher
    if (alt > alt_max) {
      alt_max = alt;
    }
  }
}

/*
  Break up maximum altitude into 3 digits
*/
void printAltBlink(int x) {
  
  if (x >= 10) {
    printAltBlink(x / 10);
  }

  // Digit is remainder of max altitude when divided by 10
  int digit = x % 10;

  for (int i = 0; i < digit; i++) {
    digitalWrite(PIN_LED3, LOW); // Anode
    delay(300);
    digitalWrite(PIN_LED3, HIGH);
    delay(300);
  }

  delay(1000);
}

/*
  Display altitude
*/
void blinkLED(float x) {

  // Maximum altitude no decimal
  int digitAll = round(x); // get rid of decimal

  printAltBlink(digitAll);
  delay(2000); // Long pause
}

#endif