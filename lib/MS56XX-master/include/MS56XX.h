#ifndef MS56XX_H
#define MS56XX_H

#include <Arduino.h>
#include <Wire.h>

//ADDRESS
#define MS56XX_ADDR_LOW 0x77
#define MS56XX_ADDR_HIGH 0x76
//OSR D1
#define BARO_PRESS_D1_OSR_256 0x40
#define BARO_PRESS_D1_OSR_512 0x42
#define BARO_PRESS_D1_OSR_1024 0x44
#define BARO_PRESS_D1_OSR_2048 0x46
#define BARO_PRESS_D1_OSR_4096 0x48
//OSR D2
#define BARO_TEMP_D2_OSR_256 0x50
#define BARO_TEMP_D2_OSR_512 0x52
#define BARO_TEMP_D2_OSR_1024 0x54
#define BARO_TEMP_D2_OSR_2048 0x56
#define BARO_TEMP_D2_OSR_4096 0x58
//READ
#define BARO_ADC_READ 0x00
#define BARO_PROM_READ 0xA0
//BARO VARIANTS
#define MS5607 0
#define MS5611 1

class MS56XX{
    private:
        int prev_time, dt;
        uint16_t refresh_rate = 0;

        bool d1_polled, d2_polled;
        bool d1_read, d2_read;

        uint8_t var_const;

        uint16_t c1_pres_sens;
        uint16_t c2_pres_off;
        uint16_t c3_temp_coef_pres_sens;
        uint16_t c4_temp_coef_pres_off;
        uint16_t c5_temp_ref;
        uint16_t c6_temp_coef_sens;

        uint32_t d1_pressure;
        uint32_t d2_temperature;
        
        float dT_temp_diff;

        uint8_t d1_polling_address;
        uint8_t d2_polling_address;

        uint8_t commandBaro(uint8_t reg);
        uint8_t requestFromBaro(uint8_t reg, uint8_t count);

        void getCoefficients();
        void calculateTemperature();
        void calculateCompensatedPressure();
    public:
        uint8_t device_address;
        
        float pressure;
        float temperature;
        float altitude;

    MS56XX(uint8_t anAddress, uint8_t MSXX){
        device_address = anAddress;
        var_const = MSXX;
    }

    bool begin();
    void configBaro(uint8_t d1_anAddress, uint8_t d2_anAddress);
    bool doBaro(bool doAltitude);
};

#endif