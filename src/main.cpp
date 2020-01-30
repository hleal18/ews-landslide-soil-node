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

#define LMIC_ENABLE_event_logging 1
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

void onEvent(ev_t ev)
{
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
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
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
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

    if (current_variable == 0)
    {
        soil_moisture_adc = soil_m1_sensor.get_soil_moisture_reading();
        id_sensor = soil_m1.id_sensor;
    }
    else if (current_variable == 1)
    {
        soil_moisture_adc = soil_m2_sensor.get_soil_moisture_reading();
        id_sensor = soil_m2.id_sensor;
    }

    // Serial.print("Raw: ");
    // Serial.println(soil_moisture_adc);
    soil_moisture_value = map(soil_moisture_adc, minADC, maxADC, 0, 100);
    // Serial.print("Soil: ");
    // Serial.println(soil_moisture_value);
    soil_moisture_value = (soil_moisture_value < 0) ? 0 : soil_moisture_value;

    // Message msg = msg_manager.createSoilMoistureMessage(id_sensor, SOIL_MOISTURE, soil_moisture_value);
    // uint8_t *buffer = msg.getMessageArray();
    // int buffer_size = msg.getMessageArraySize();

    uint8_t buffer[] = {id_sensor, SOIL_MOISTURE, soil_moisture_value};

    current_variable++;

    if (current_variable >= number_of_variables)
        current_variable = 0;

    LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
    Serial.println(F("Packet queued"));
    // Serial.println("Borrado");
    // delete[] buffer;
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

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    // LMIC_setupChannel(0, 903900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(1, 904100000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    // LMIC_setupChannel(2, 904300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(3, 904500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(4, 904700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(5, 904900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(6, 905100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(7, 905300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(8, 904600000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

    LMIC_selectSubBand(1);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF10, 14);

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