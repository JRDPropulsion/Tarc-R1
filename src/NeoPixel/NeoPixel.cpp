/*
  NeoPixel - Handles all NeoPixel related code
*/


/*
  Importing libraries
*/
#include <Arduino.h>
#include <Configuration/Config.h>
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
  Cycle through NeoPixel colors to generate rainbow
*/
void NeoPixel_rainbow() {
    static int currentColorIndex = 0;
    static int nextColorIndex = 1;
    static uint32_t colors[] = {pixels.Color(100, 0, 0),   // Red
                                pixels.Color(0, 100, 0),   // Green
                                pixels.Color(0, 0, 100)};  // Blue

    // Calculate the transition step for each color component
    uint8_t stepR = (uint8_t)((colors[nextColorIndex] >> 16) - (colors[currentColorIndex] >> 16)) / 30;
    uint8_t stepG = (uint8_t)(((colors[nextColorIndex] >> 8) & 0xFF) - ((colors[currentColorIndex] >> 8) & 0xFF)) / 30;
    uint8_t stepB = (uint8_t)((colors[nextColorIndex] & 0xFF) - (colors[currentColorIndex] & 0xFF)) / 30;

    // Perform the smooth transition
    for (int i = 0; i < 30; i++) {
      uint8_t r = (colors[currentColorIndex] >> 16) + (stepR * i);
      uint8_t g = ((colors[currentColorIndex] >> 8) & 0xFF) + (stepG * i);
      uint8_t b = (colors[currentColorIndex] & 0xFF) + (stepB * i);

      for (int j = 0; j < 1; j++) {
        pixels.setPixelColor(j, pixels.Color(r, g, b));
      }

      pixels.show(); // Send the updated color to the NeoPixels
      delay(20); // Wait for 10ms (adjust this value for smoother or slower transitions)
    }

    // Switch to the next color
    currentColorIndex = (currentColorIndex + 1) % (sizeof(colors) / sizeof(colors[0]));
    nextColorIndex = (nextColorIndex + 1) % (sizeof(colors) / sizeof(colors[0]));
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
