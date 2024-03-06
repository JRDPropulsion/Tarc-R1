#include <Arduino.h>
#include <Wire.h>
#include "MS56XX.h"

//MS56XX

uint8_t MS56XX::commandBaro(uint8_t data){
    Wire.beginTransmission(device_address); 
    Wire.write(data); 
    return Wire.endTransmission();
}
uint8_t MS56XX::requestFromBaro(uint8_t reg, uint8_t count){
    Wire.beginTransmission(device_address); 
    Wire.write(reg); 
    uint8_t err = Wire.endTransmission(); 
    Wire.requestFrom(device_address, count);
    return err;
}

bool MS56XX::begin(){
    Wire.begin();

    delay(10);

    Wire.beginTransmission(device_address);
    uint8_t error = Wire.endTransmission(); //Check error code of a test transmission

    if(error != 0){ return false; }

    commandBaro(0x1E); //Reset the sensor
    
    delay(50);

    uint16_t PROM_BUFFER[8]; //Buffer for the PROM memory

    for(int i = 0; i < 8; i++){
        requestFromBaro(0xA0 + (i << 1), 2); //Read PROM
        PROM_BUFFER[i] = (Wire.read() << 8) | Wire.read();
    }

    c1_pres_sens = PROM_BUFFER[1];
    c2_pres_off = PROM_BUFFER[2];
    c3_temp_coef_pres_sens = PROM_BUFFER[3];
    c4_temp_coef_pres_off = PROM_BUFFER[4];
    c5_temp_ref = PROM_BUFFER[5];
    c6_temp_coef_sens = PROM_BUFFER[6];

    configBaro(BARO_PRESS_D1_OSR_512, BARO_TEMP_D2_OSR_512); //Default configuration

    return true;
}

void MS56XX::configBaro(uint8_t d1_anAddress, uint8_t d2_anAddress){
    d1_polling_address = d1_anAddress;
    d2_polling_address = d2_anAddress;
}

bool MS56XX::doBaro(bool doAltitude){ //RETURNS TRUE IF NEW DATA IS CALCULATED
    if(!d1_polled){
        prev_time = millis();
        d1_polled = true;
        switch(d1_polling_address){
            case BARO_PRESS_D1_OSR_4096:
                refresh_rate = 10;
                break;
            case BARO_PRESS_D1_OSR_2048:
                refresh_rate = 5;
                break;
            case BARO_PRESS_D1_OSR_1024:
                refresh_rate = 3;
                break;
            case BARO_PRESS_D1_OSR_512:
                refresh_rate = 2;
                break;
            case BARO_PRESS_D1_OSR_256:
                refresh_rate = 1;
                break;
        }
        commandBaro(d1_polling_address);
    }   

    dt = millis() - prev_time;
    if(dt > refresh_rate && !d1_read){
        requestFromBaro(BARO_ADC_READ, 3);

        d1_pressure = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();

        d1_read = true;
    }else if(!d1_read){
        return false;
    }
    
    if(!d2_polled){
        prev_time = millis();
        d2_polled = true;
        switch(d2_polling_address){
            case BARO_TEMP_D2_OSR_4096:
                refresh_rate = 10;
                break;
            case BARO_TEMP_D2_OSR_2048:
                refresh_rate = 5;
                break;
            case BARO_TEMP_D2_OSR_1024:
                refresh_rate = 3;
                break;
            case BARO_TEMP_D2_OSR_512:
                refresh_rate = 2;
                break;
            case BARO_TEMP_D2_OSR_256:
                refresh_rate = 1;
                break;
        }
        commandBaro(d2_polling_address);
    }

    dt = millis() - prev_time;
    if(dt > refresh_rate && !d2_read){
        requestFromBaro(BARO_ADC_READ, 3);

        d2_temperature = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();

        d2_read = true;
    }else if(!d2_read){
        return false;
    }

    calculateTemperature();
    calculateCompensatedPressure();
    if(doAltitude){
        altitude = 44330.0f * (1.0f - powf( (pressure/100) / 1013.25f, 0.1903f));
    }

    d1_polled = false; d2_polled = false;
    d1_read = false; d2_read = false;

    return true;
}

void MS56XX::calculateTemperature(){
    dT_temp_diff = float(d2_temperature) - float(c5_temp_ref) * float(1 << 8);
    float actual_temperature = 2000.0f + dT_temp_diff * float(c6_temp_coef_sens) / float(1 << 23);
    
    //Might add second order temperature compensation for less than 20C temperatures

    temperature = actual_temperature/100.0f; // Degrees Centigrade
}

void MS56XX::calculateCompensatedPressure(){
    float offset = float(c2_pres_off) * float(1 << (17 - var_const)) + float(c4_temp_coef_pres_off) * float(dT_temp_diff)/float(1 << (6 + var_const));
    float sens = float(c1_pres_sens) * float(1 << (16 - var_const)) + float(c3_temp_coef_pres_sens) * float(dT_temp_diff)/float(1 << (7 + var_const));
    float comp_pressure = ( (float(d1_pressure) * sens / float(1 << 21)) - offset)/float(1 << 15);

    pressure = comp_pressure; // Pascals
}