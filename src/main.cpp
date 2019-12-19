#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>

#include "esp_sleep.h"

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"
#include "SoilMoisture/SoilMoisture.h"

// For storing data on non-volatile memory
#include <Preferences.h>

// T-Beam specific hardwareVv
#undef BUILTIN_LED
#define BUILTIN_LED 21

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

Preferences storage_manager;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

static osjob_t sendjob;
void do_send(osjob_t *j);
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 1;

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
    }
}

void do_send(osjob_t *j)
{

    uint8_t arrBuff[epoch_size + 2];

    arrBuff[0] = id_sensor;
    arrBuff[1] = variable_type;

    uint16_t soil_moisture_adc = soil_moisture_sensor.get_soil_moisture_reading();
    int8_t soil_moisture_value = map(soil_moisture_adc, minADC, maxADC, 0, 100);
    soil_moisture_value = (soil_moisture_value < 0) ? 0 : soil_moisture_value;

    Serial.print("Soil: ");
    Serial.println(soil_moisture_value);
    arrBuff[2] = soil_moisture_value;

    LMIC_setTxData2(1, arrBuff, sizeof(arrBuff), 0);
    Serial.println(F("Packet queued"));
    digitalWrite(BUILTIN_LED, HIGH);
}

void setup()
{
    Serial.begin(9600);

    //Turn off WiFi and Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();

    // Initializing sensor

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    LMIC_setupChannel(0, 903900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 904100000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 904300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 904500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 904700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 904900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 905100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 905300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 904600000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF10, 14);

    do_send(&sendjob);
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);
}

void loop()
{
    os_runloop_once();
}