#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Preferences.h>
#include <WiFi.h>
#include <MPU9255.h>
#include "esp_sleep.h"
#include "config.h"

#include "Message/Message.h"
#include "Message/MessageBuilder.h"
#include "Message/MessageBuilderManager.h"

#include "VariableType/VariableType.h"
#include "SoilMoisture/SoilMoisture.h"
// For storing data on non-volatile memory

struct Sensor
{
    VariableType variableType;
    uint8_t id_sensor;
};

Sensor soil_m1 = {SOIL_MOISTURE, 3};
Sensor soil_m2 = {SOIL_MOISTURE, 4};
const uint8_t soil_m1_pin = 35;
const uint8_t soil_m2_pin = 34;
SoilMoisture soil_m1_sensor(soil_m1_pin);
SoilMoisture soil_m2_sensor(soil_m2_pin);

const int number_of_variables = 2;
uint8_t current_variable = 0;

MessageBuilderManager msg_manager;
// Lower and upper limits for % of soil moisture.
// Manually found.
int minADC = 192;
int maxADC = 3350;

Preferences storage_manager;

void os_getArtEui(u1_t *buf)
{
}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

static osjob_t sendjob;
void do_send(osjob_t *j);
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
unsigned TX_INTERVAL = 900;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN, // was "14,"
    .dio = {26, 33, 32},
};

void enable_sleep()
{ //TODO implement  UBX-ACK
    do
    { //We cannot read UBX ack therefore try to sleep gps until it does not send data anymore
        Serial.println("try to sleep gps!");
        byte CFG_RST[12] = {0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x0F, 0x66};
        delay(600);                                                                                                                  //give some time to restart //TODO wait for ack
        const byte RXM_PMREQ[16] = {0xb5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4d, 0x3b}; //power off until wakeup call
        Serial1.write(RXM_PMREQ, sizeof(RXM_PMREQ));
        unsigned long startTime = millis();
        unsigned long offTime = 1;
        Serial.println(offTime);

        while (millis() - startTime < 1000)
        { //wait for the last command to finish
            int c = Serial1.read();
            if (offTime == 1 && c == -1)
            { //check  if empty
                offTime = millis();
            }
            else if (c != -1)
            {
                offTime = 1;
            }
            if (offTime != 1 && millis() - offTime > 100)
            { //if gps chip does not send any commands for .. seconds it is sleeping
                Serial.println("sleeping gps!");
                return;
            }
        }
    } while (1);
}

void onEvent(ev_t ev)
{
    switch (ev)
    {

    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        digitalWrite(BUILTIN_LED, LOW);
        // Schedule next transmission
        esp_sleep_enable_timer_wakeup(TX_INTERVAL * 1000000);
        esp_deep_sleep_start();

        //ESP.restart();
        //delay(TX_INTERVAL * 1000);
        do_send(&sendjob);
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}

void do_send(osjob_t *j)
{
    Serial.print("Variable type: ");
    Serial.println(current_variable);

    uint16_t soil_moisture_adc = 0;
    int8_t soil_moisture_value = 0;
    uint8_t id_sensor = 0;

    soil_moisture_adc = soil_m1_sensor.get_soil_moisture_reading();
    id_sensor = soil_m1.id_sensor;

    // Serial.print("Raw: ");
    // Serial.println(soil_moisture_adc);
    soil_moisture_value = map(soil_moisture_adc, minADC, maxADC, 0, 100);
    // Serial.print("Soil: ");
    // Serial.println(soil_moisture_value);
    soil_moisture_value = (soil_moisture_value < 0) ? 0 : soil_moisture_value;

    Message msg = msg_manager.createSoilMoistureMessage(id_sensor, SOIL_MOISTURE, soil_moisture_value);
    uint8_t *buffer = msg.getMessageArray();
    int buffer_size = msg.getMessageArraySize();

    // uint8_t buffer[] = {id_sensor, SOIL_MOISTURE, soil_moisture_value};

    // current_variable++;

    // if (current_variable >= number_of_variables)
    //     current_variable = 0;

    LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
    Serial.println(F("Packet queued"));
    // Serial.println("Borrado");
    delete[] buffer;
    // digitalWrite(BUILTIN_LED, HIGH);

    storage_manager.putUInt("sensor_index", current_variable);
    storage_manager.end();

    // if (current_variable == 0)
    //     TX_INTERVAL = 900;
    // else
    //     TX_INTERVAL = 1;
}

void setup()
{
    Serial.begin(9600);

    //Turn off WiFi and Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();
    
    // Puts GPS to sleep.
    Serial1.begin(9600, SERIAL_8N1, 12, 15);
    enable_sleep();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    LMIC_selectSubBand(1);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    // It should go before do_send
    storage_manager.begin("variables_state", false);
    current_variable = storage_manager.getUInt("sensor_index", 0);

    if (current_variable >= number_of_variables)
        current_variable = 0;

    do_send(&sendjob);
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);
}

void loop()
{
    os_runloop_once();
}