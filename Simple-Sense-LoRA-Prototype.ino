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
 * v1.1 - First Commit
 * v1.2 - Changed reporting to only on change in occupancy and required ack
 * v1.3 - Tried updating the clock error - still taking too long to connect
 *******************************************************************************/
#include <Arduino_LoRaWAN_machineQ.h>     // Tested with v0.5.2 of MCCI LoRAWan Library
#include <lmic.h>                         // Tested with v2.3.1 of MCCI LMIC Library
#include <hal/hal.h>
#include <SPI.h>
#include <TimeLib.h>

enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, LOW_BATTERY_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Low Battery", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;


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
const unsigned TX_INTERVAL = 10;
int maxSendAttempts = 5;
int sendAttempts = 0;
int ackedAttempts = 0;

// Pin mapping for Adafruit Feather M0 LoRa, etc.
const int PIRpin = A0;
const int LEDpin = 13;
const int RFMresetPin = 4;
const int RFMchipSelectPin = 8;
const int RFMgpioPin = 3;
volatile bool presenceState = false;
volatile bool presenceChange = false;
bool ackReceived = false;
bool joinedState = false;
volatile unsigned long lastPIRevent = 0;
unsigned long timeout = 180000;
const lmic_pinmap lmic_pins = {
    .nss = RFMchipSelectPin,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RFMresetPin,
    .dio = {RFMgpioPin, 6, LMIC_UNUSED_PIN},
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
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    do_send(&sendjob);                  // Start job (sending automatically starts OTAA too)
    while (!joinedState) {
      os_runloop_once();
      delay(1000);
    }
    
    attachInterrupt(PIRpin,sensorISR,RISING);

    state = IDLE_STATE;
}

void loop() {
  switch (state) {
    case IDLE_STATE:
      if(state != oldState) publishStateTransition();
      if ((millis() - lastPIRevent >= timeout) && presenceState) {
        digitalWrite(LEDpin,LOW);
        presenceState = false;
        presenceChange = true;
      }
      if (presenceChange) {
        if (presenceState) {
          Serial.println("Occupancy");
          mydata[2] = 0xff;
        }
        else {
          Serial.println("No occupancy");
          mydata[2] = 0x00;
        }
        state = REPORTING_STATE;
      }
      break;
    case REPORTING_STATE:
      publishStateTransition();
      os_runloop_once();
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);  // Schedule the next event
           //do_send(&sendjob);                  // Start job (sending automatically starts OTAA too)

       state = RESP_WAIT_STATE;
      break;
    case RESP_WAIT_STATE: 
      publishStateTransition();
      if (ackReceived) state = IDLE_STATE;
      else {
        delay(10000);                    // Don't retransmit too often
        state = REPORTING_STATE;         // Try again until we get ACK
      }
      break;
    case ERROR_STATE:
      publishStateTransition();
      digitalWrite(RFMresetPin,LOW);
      delay(100);
      digitalWrite(RFMresetPin,HIGH);
      os_init();                          // LMIC init
      LMIC_reset();                       // Reset the MAC state. Session and pending data transfers will be discarded.
      break;
  }
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
            joinedState = true;
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
              ackReceived = true;
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


void sensorISR() {                             // Interrupt handler for PIR sensor event
  if (!presenceState) {
    presenceState = true;
    presenceChange = true;
    digitalWrite(LEDpin,HIGH);  
  }
  lastPIRevent = millis();
}

void digitalClockDisplay() {                   // Displayed time since last reset for logging
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
}

void printDigits(int digits) {                 // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  Serial.println(stateTransitionString);
}
