/*
  NeoPixel - Handles all NeoPixel related code
*/

#ifndef NEOPIXEL_H
#define NEOPIXEL_H

// Importing libraries
#include <Arduino.h>
#include <Config.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels(1, NeoPixel, NEO_GRB + NEO_KHZ800);

/*
  Initialize NeoPixel
*/
void NeoPixel_init()
{
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

/*
  Change NeoPixel color to red
*/
void NeoPixel_red()
{
    pixels.clear();
    pixels.setPixelColor(0, 255, 0, 0);
    pixels.show();
}

/*
  Change NeoPixel color to yellow
*/
void NeoPixel_yellow()
{
    pixels.clear();
    pixels.setPixelColor(0, 255, 255, 0);
    pixels.show();
}

/*
  Change NeoPixel color to green
*/
void NeoPixel_green()
{
    pixels.clear();
    pixels.setPixelColor(0, 0, 255, 0);
    pixels.show();
}

/*
  Change NeoPixel color to blue
*/
void NeoPixel_blue()
{
    pixels.clear();
    pixels.setPixelColor(0, 0, 0, 255);
    pixels.show();
}

/*
  Change NeoPixel color to purple
*/
void NeoPixel_purple()
{
    pixels.clear();
    pixels.setPixelColor(0, 255, 0, 255);
    pixels.show();
}

/*
  Change NeoPixel color to white
*/
void NeoPixel_white()
{
    pixels.clear();
    pixels.setPixelColor(0, 255, 255, 255);
    pixels.show();
}

#endif