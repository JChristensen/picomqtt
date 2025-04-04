// Raspberry Pi Pico W test sketch.
// Sends time and temperature via MQTT over wifi once a minute.
// J.Christensen 14Mar2025
// Developed using Arduino IDE 1.8.19 and Earle Philhower's Ardino-Pico core,
// https://github.com/earlephilhower/arduino-pico
// Copyright (C) 2025 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html
//
// TODO:
//  xxx 1. Investigate WiFiMulti vs ???
//  xxx 2. Recover wifi if connection drops.
//  ??? 3. Renew DHCP lease periodically.
//         - Have not found anything that indicates this is needed, or even if it's a thing for wifi.
//  4. Investigate GroveStreams MQTT API.
//  5. Investigate RP2040 RTC.
//  6. Investigate Pico NTP library.

#include <MCP79412RTC.h>    // https://github.com/JChristensen/DS3232RTC
#include <MCP9800.h>        // https://github.com/JChristensen/MCP9800
#include <PubSubClient.h>   // https://github.com/knolleary/pubsubclient
#include <Streaming.h>      // https://github.com/janelia-arduino/Streaming
#include <Timezone.h>       // https://github.com/JChristensen/Timezone
#include <WiFi.h>
#include "JC_MQTT.h"
#include "WifiManager.h"
#include "PicoConfig.h"
#include "Heartbeat.h"

// constant parameters
const char* mqBroker {"z1"};
constexpr uint32_t mqPort {1883};
const char* mqTopic {"picotest"};
constexpr int RTC_1HZ_PIN {22};
constexpr int wifiLED {9};
constexpr int sdaPin{20}, sclPin{21};       // I2C pins
constexpr int txdPin{4}, rxdPin{5};         // Serial2 pins
constexpr uint32_t blinkInterval {1000};    // LED blink interval

// object instantiations and globals
HardwareSerial& mySerial{Serial2};
PicoConfig cfg;
MCP79412RTC myRTC;
MCP9800 mySensor;
WifiManager wifi(mySerial);
WiFiClient picoClient;
JC_MQTT mq(picoClient, mySerial);
Heartbeat hb(LED_BUILTIN, blinkInterval);
volatile time_t isrUTC;     // ISR's copy of current time in UTC

void setup()
{
    hb.begin();
    pinMode(wifiLED, OUTPUT);
    Serial2.setTX(txdPin);
    Serial2.setRX(rxdPin);
    mySerial.begin(115200);
    delay(2000);
    mySerial.printf("\n%s\nCompiled %s %s %s @ %d MHz\n",
        __FILE__, __DATE__, __TIME__, BOARD_NAME, F_CPU/1000000);
    Wire.setSDA(sdaPin);
    Wire.setSCL(sclPin);
    checkI2C(sdaPin, sclPin);
    mySensor.begin();

    // rtc initialization
    pinMode(RTC_1HZ_PIN, INPUT_PULLUP);
    myRTC.begin();
    mySerial << millis() << " RTC SYNC... ";
    time_t lastUTC = myRTC.get();      // try to read the time from the RTC
    if ( lastUTC == 0 ) {       // couldn't read it, something wrong
        mySerial << "FAIL!\n";
        resetMCU(10);
    }
    else {
        mySerial << "OK.\n";
    }
    if (!myRTC.isRunning()) myRTC.set(lastUTC); // start the rtc if not running
    // check for signature indicating calibration value present
    if (myRTC.eepromRead(125) == 0xAA && myRTC.eepromRead(126) == 0x55) {
        myRTC.calibWrite( (int8_t)myRTC.eepromRead(127) );  // set calibration register from stored value
        mySerial << millis() << " RTC calibration set from EEPROM: " << myRTC.calibRead() << endl;
    }
    attachInterrupt(RTC_1HZ_PIN, incrementTime, FALLING);
    myRTC.squareWave(MCP79412RTC::SQWAVE_1_HZ);
    lastUTC = getUTC();
    // wait for the next interrupt
    while (lastUTC == getUTC()) delay(10);
    time_t utc = myRTC.get();
    noInterrupts();
    isrUTC = utc;
    interrupts();
    
    // connect to wifi
    cfg.begin();    // get credentials stored in eeprom
    cfg.read();
    wifi.begin(cfg.params.hostname, cfg.params.ssid, cfg.params.psk);
    mq.begin(mqBroker, mqPort, mqTopic, cfg.params.hostname);
}

void loop()
{
    constexpr int MSG_BUFFER_SIZE{80};
    static char msg[MSG_BUFFER_SIZE];   // must be static to pass to JC_MQTT::publish()
    static time_t pubLast{0};
    uint32_t ms = millis();
    static char picoStatus[16] {"starting"};
    constexpr TimeChangeRule edt = {"EDT", Second, Sun, Mar, 2, -240};  // Daylight time = UTC - 4 hours
    constexpr TimeChangeRule est = {"EST", First, Sun, Nov, 2, -300};   // Standard time = UTC - 5 hours
    static Timezone eastern(edt, est);

    bool wifiStatus = wifi.run();
    digitalWrite(wifiLED, wifiStatus);
    if (wifiStatus) {
        if (mq.run()) {
            time_t t = getUTC();
            if (minute(t) != minute(pubLast)) {
                TimeChangeRule* tcr;    // pointer to the time change rule, use to get TZ abbrev
                time_t l = eastern.toLocal(t, &tcr);    // convert to local time
                float F = mySensor.readTempF10(AMBIENT) / 10.0;
                sprintf(msg, "Pico %s %d-%.2d-%.2d %.2d:%.2d:%.2d %s %.1f°F %ld dBm %ld",
                    picoStatus, year(l), month(l), day(l), hour(l), minute(l), second(l),
                    tcr->abbrev, F, WiFi.RSSI(), ms);
                if (pubLast == 0) strcpy(picoStatus, "time");
                pubLast = t;
                mq.publish(msg);
            }
        }
    }
    hb.run();
}

// return current time
time_t getUTC()
{
    noInterrupts();
    time_t utc = isrUTC;
    interrupts();
    return utc;
}

// set the current time
void setUTC(time_t utc)
{
    noInterrupts();
    isrUTC = utc;
    interrupts();
}

// 1Hz RTC interrupt handler increments the current time
void incrementTime()
{
    ++isrUTC;
}

// returns true if the i2c bus was successfully reset or if it did not
// need to be reset. returns false if the reset attempt failed.
bool checkI2C(int sdaPin, int sclPin)
{
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, INPUT_PULLUP);
    bool sda = digitalRead(sdaPin);

    // if sda is pulled low, try to reset the bus
    if (!sda) {
        mySerial << millis() << " Resetting the I2C Bus... ";
        pinMode(sclPin, OUTPUT);
        for (int n=0; n<10; n++) {
            digitalWrite(sclPin, LOW);
            delay(1);
            digitalWrite(sclPin, HIGH);
            delay(1);
        }
        pinMode(sclPin, INPUT_PULLUP);
        sda = digitalRead(sdaPin);
        if (sda) {mySerial << "OK.\n";}
        else {mySerial << "FAIL!\n";}
    }    
    return sda;
}

void resetMCU(int seconds)
{
    mySerial << millis() << " Reboot in ";
    for (int i=seconds; i>=1; i--) {
        mySerial << i << ' ';
        delay(1000);
    }
    mySerial << "\n\n";
    rp2040.reboot();
}
