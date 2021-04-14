// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <ssd1306.h>

#define LEDPIN 2

unsigned int counter = 0;
char TTN_response[30];

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes.

// Copy the value from Device EUI from the TTN console in LSB mode.
static const u1_t PROGMEM DEVEUI[8]= { 0xBE, 0x44, 0xF0, 0xCB, 0xAA, 0x77, 0x42, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// Copy the value from Application EUI from the TTN console in LSB mode
static const u1_t PROGMEM APPEUI[8]= { 0xA5, 0xF4, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is. Anyway its in MSB mode.
static const u1_t PROGMEM APPKEY[16] = {0x86, 0x4A, 0xE4, 0xA4, 0x6A, 0xED, 0x40, 0xE3, 0x76, 0xBA, 0x14, 0xBF, 0xD6, 0x09, 0x70, 0x7E };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 120;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void do_send(osjob_t* j){
    // Payload to send (uplink)
    static uint8_t message[] = "Hello leo!";

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, message, sizeof(message)-1, 0);
        Serial.println(F("Sending uplink packet..."));
        digitalWrite(LEDPIN, HIGH);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
           case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));
            }

            if (LMIC.dataLen) {
              int i = 0;
              // data received in rx slot after tx
              Serial.print(F("Data Received: "));
              Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println();
              Serial.println(LMIC.rssi);

              for ( i = 0 ; i < LMIC.dataLen ; i++ )
                TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];
              TTN_response[i] = 0;
            }

            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            digitalWrite(LEDPIN, LOW);

            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING: -> Joining..."));
            break;
        case EV_JOINED: {
              Serial.println(F("EV_JOINED"));
              // Disable link check validation (automatically enabled
              // during join, but not supported by TTN at this time).
              LMIC_setLinkCheckMode(0);
            }
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

void setup() {
    Serial.begin(9600);
    Serial.println(F("Test..."));

    delay(2500);                      // Give time to the serial monitor to pick up
    Serial.println(F("Starting..."));

    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN,OUTPUT);

     
    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(DR_SF11,14);
    LMIC_setDrTxpow(DR_SF9,14);

    // Start job
    do_send(&sendjob);     // Will fire up also the join
    //LMIC_startJoining();
}

void loop() {
    os_runloop_once();
}
