/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Modified by Chip McClelland, See Insights LLC, for Simple Sense
 * 
 * v1.1
 *
 *******************************************************************************/
#include <Arduino_LoRaWAN_machineQ.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TimeLib.h>

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. 
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0xFF, 0xCA, 0xAF, 0x10, 0xB6, 0x76, 0x98 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). 
static const u1_t PROGMEM APPKEY[16] = { 0x70, 0xB3, 0xD5, 0x3E, 0x6F, 0xFF, 0xFF, 0xB1, 0x70, 0xB3, 0xD5, 0x3E, 0x6F, 0xFF, 0xFF, 0xB1 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static byte mydata[3] = {0x01,0x00,0x00};  // format us CayenneLPP - sensor, type, value
const int sensorCH = 1;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 15;
int sendAttempts = 0;
int ackedAttempts = 0;

// Pin mapping for Adafruit Feather M0 LoRa, etc.
const int PIRpin = A0;
const int LEDpin = 13;
volatile bool presenceState = false;
volatile bool presenceChange = false;
volatile unsigned long lastPIRevent = 0;
unsigned long timeout = 30000;
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

void setup() {
    delay(2000);
    Serial.begin(9600);
    Serial.println(F("Starting"));
    pinMode(PIRpin,INPUT);
    pinMode(LEDpin,OUTPUT);
    digitalWrite(LEDpin,LOW);
    
    os_init();                          // LMIC init
    LMIC_reset();                       // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_setAdrMode(1);                 // Enable Adaptive Rate Control - disable if device is mobile.
    //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    do_send(&sendjob);                  // Start job (sending automatically starts OTAA too)

    attachInterrupt(PIRpin,sensorISR,RISING);
}

void loop() {
    if ((millis() - lastPIRevent >= timeout) && presenceState) {
      digitalWrite(LEDpin,LOW);
      presenceState = false;
      presenceChange = true;
    }
    if (presenceChange) {                 // Schedule next transmission
      if (presenceState) {
        Serial.println("Detected");
        mydata[2] = 0xff;
      }
      else mydata[2] = 0x00;
      presenceChange = false;
    }
    os_runloop_once();
}


void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(sensorCH, mydata, sizeof(mydata), 1);
        Serial.print(F("Packet queued:"));
        Serial.print("Channel: ");
        Serial.print(sensorCH);
        Serial.print(" Sensor: 0x");
        Serial.print(mydata[0],HEX);
        Serial.print(" Type: 0x");
        Serial.print(mydata[1],HEX);
        Serial.print(" Value: 0x");
        Serial.println(mydata[2],HEX);
        sendAttempts++;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    digitalClockDisplay();
    Serial.print(": ");
    switch(ev) {
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
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.print(F("Received ack running at "));
              ackedAttempts++;
              float percentAcked = ((float)ackedAttempts / (float)sendAttempts) * 100.0;
              Serial.print(percentAcked);
              Serial.print("%");
              // presenceChange = false;
            }
            if (LMIC.dataLen) {
              Serial.print(F(" and "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            else Serial.println("");
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);  // Schedule the next event
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
        case EV_SCAN_FOUND:
            Serial.println(F("EV_SCAN_FOUND"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}


void sensorISR() {
  if (millis() - lastPIRevent >= timeout || !presenceState) {
    presenceState = true;
    presenceChange = true;
    digitalWrite(LEDpin,HIGH);  
  }
  lastPIRevent = millis();
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


