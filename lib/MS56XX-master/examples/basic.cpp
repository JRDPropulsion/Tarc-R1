#include <Arduino.h>
#include <Wire.h>
#include "MS56XX.h"

//#define Serial SerialUSB //Tested on ATSAMD21 Arduino Zero environment
#define BAUD_RATE 115200

MS56XX MSXX(MS56XX_ADDR_HIGH, MS5607); //Barometer object, change the address depending if your CSB pin is pulled high or low, change MS5607/11 to your desired sensor

void setup(){
  Serial.begin(BAUD_RATE);

  while(!MSXX.begin()){ //Check if baro has initialized correctly
    Serial.println("BARO-FAIL");
    delay(1000);
  }

  MSXX.configBaro(BARO_PRESS_D1_OSR_4096, BARO_TEMP_D2_OSR_4096); //Configure oversampling rate for pressure and temperature respectively, default is 512 for both
}

void loop(){
  //RETURNS TRUE IF NEW DATA IS CALCULATED
  bool dataready = MSXX.doBaro(true); //Calculate pressure and temperature, boolean for altitude estimation from sea level
  
  Serial.print(uint8_t(dataready)); //Is data fresh?
  Serial.print(", ");
  Serial.print(MSXX.pressure); //Pascals
  Serial.print(", ");
  Serial.print(MSXX.temperature); //Degrees centigrade
  Serial.print(", ");
  Serial.println(MSXX.altitude); //Meters above sea level
}