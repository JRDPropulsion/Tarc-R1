/*
  Pyro - All functions relating to pyrotechnics
*/


/*
  Importing libraries
*/
#include <Arduino.h>
#include <Navigation/Nav.h>
#include <Configuration/Config.h>
#include <NeoPixel/NeoPixel.h>

float pyro1_time, pyro2_time;


/*
  Initialize the pyro channels and check continuity across all channels
*/
void pyro_init() 
{
  pinMode (pyro1_pin, OUTPUT); 
  pinMode (pyro2_pin, OUTPUT); 
  pinMode (pyro1_cont_pin, INPUT); 
  pinMode (pyro2_cont_pin, INPUT);

  float pyro1_cont_analog = analogRead(pyro1_cont_pin) ;
  float pyro2_cont_analog = analogRead(pyro2_cont_pin) ;

  float pyro1_cont = 0;
  float pyro2_cont = 0;

  if (pyro1_cont_analog < 175)
  {
    pyro1_cont = 1;
    NeoPixel_purple();
    SerialUSB.println("Cont on pyro 1");
  } 

  delay (400);

  if (pyro2_cont_analog < 175) 
  {
    pyro2_cont = 1;
    NeoPixel_purple();
    SerialUSB.println("Cont on pyro 2");
  }

  delay(400);
}


/*
  Fire pyro channel 1
*/
void pyro1_fire(double dt) 
{
  digitalWrite(pyro1_pin, HIGH);
  pyro1_time += dt;

  // Turns on for 1 second
  if (pyro1_time >= 1) {
    digitalWrite(pyro1_pin, LOW);
  }
}


/*
  Fire pyro channel 2
*/
void pyro2_fire(double dt) 
{
  digitalWrite(pyro2_pin, HIGH);
  pyro2_time += dt;

  // Turns on for 1 second
  if (pyro2_time >= 1) {
    digitalWrite(pyro2_pin, LOW);
  }
}

