#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>

#include "SoilMoisture/SoilMoisture.h"


enum Variable_Type
{
    ACCELERATION = 0,
    ROTATION_RATE,
    SOIL_MOISTURE
};

const int epoch_size = 1;
uint8_t variable_type = SOIL_MOISTURE;
uint8_t id_sensor = 3;
const uint8_t soil_m_pin = 35;
SoilMoisture soil_moisture_sensor(soil_m_pin);

// Lower and upper limits for % of soil moisture.
// Manually found.
int minADC = 192;
int maxADC = 3350;

void setup()
{
    Serial.begin(9600);
    
}

void loop()
{
    delay(500);
    uint16_t soil_moisture_adc = soil_moisture_sensor.get_soil_moisture_reading();
    Serial.print("Raw: "); Serial.println(soil_moisture_adc);
    int8_t soil_moisture_value = map(soil_moisture_adc, minADC, maxADC, 0, 100);
    Serial.print("Soil: ");
    Serial.println(soil_moisture_value);
}