// Arduino wifi manager for Raspberry Pi Pico.
// https://github.com/JChristensen/
// Copyright (C) 2025 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

#ifndef WIFIMANAGER_H_INCLUDED
#define WIFIMANAGER_H_INCLUDED
#include <WiFi.h>
#include <Streaming.h>          // http://arduiniana.org/libraries/streaming/

class WifiManager
{
    enum m_states_t {CONNECT, CONNECT_WAIT, CONNECT_CHECK, RETRY_WAIT, MONITOR};
    public:
        WifiManager(HardwareSerial& hws=Serial)
            : m_Serial{hws} {}
        void begin(const char* hostname, const char* ssid, const char* psk);
        bool run();
        void resetMCU(int seconds);

    private:
        m_states_t m_state {CONNECT};
        HardwareSerial& m_Serial;       // alternate serial output
        int m_retryCount {0};
        uint m_lastTry;                 // last time we tried to connect
        static constexpr uint m_minRetryWait {15000};   // minimum wait between retries
        static constexpr uint m_connectWait {3000};
        static constexpr uint m_monitorWait {1000};
        static constexpr int m_maxRetries {10};
        uint m_waitTimer;
        const char* m_hostname;
        const char* m_ssid;
        const char* m_psk;
};
#endif

//#include <WifiManager.h>

void WifiManager::begin(const char* hostname, const char* ssid, const char* psk)
{
    m_hostname = hostname;
    m_ssid = ssid;
    m_psk = psk;
}

// run the state machine. returns true if connected to wifi.
bool WifiManager::run()
{
    static bool connected {false}; // this is the status we return to the caller
 
    switch(m_state) {

        // attempt to connect to wifi, reboot if we have tried too many times.
        case CONNECT:
            if (++m_retryCount > m_maxRetries) {
                m_Serial.printf("%d Too many retries.\n", millis());
                resetMCU(10);
            }
            m_state = CONNECT_WAIT;
            m_Serial.printf("%d Connecting to: %s\n", millis(), m_ssid);
            m_lastTry = millis();
            WiFi.setHostname(m_hostname);
            WiFi.begin(m_ssid, m_psk);
            m_waitTimer = millis();
            break;

        // wait a few seconds for the connection.
        case CONNECT_WAIT:
            if (millis() - m_waitTimer >= m_connectWait) {
                m_state = CONNECT_CHECK;
            }
            break;

        // check to see if we connected successfully.
        // if so, move to monitor mode. if not, try connecting again.
        case CONNECT_CHECK:
            if (WiFi.status() == WL_CONNECTED) {
                m_state = MONITOR;
                m_Serial.printf("%d WiFi connected %s %d dBm\n",
                    millis(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
                m_retryCount = 0;
                m_waitTimer = millis();
                connected = true;                
            }
            else {
                m_state = RETRY_WAIT;
            }
            break;

        // this state enforces a minimum wait between connection retries.
        case RETRY_WAIT:
            if (millis() - m_lastTry >= m_minRetryWait) {
                m_state = CONNECT;
            }
            break;

        // connection established. check it periodically (but no need to check too fast.)
        case MONITOR:
            if (millis() - m_waitTimer >= m_monitorWait) {
                if (WiFi.status() != WL_CONNECTED) {
                    m_state = RETRY_WAIT;
                    m_lastTry = millis();
                    WiFi.disconnect();
                    connected = false;
                    m_Serial.printf("%d WiFi connection lost.\n", millis());
                }
                else {
                    m_waitTimer = millis();
                }
            }
            break;
    }
    return connected;
}

void WifiManager::resetMCU(int seconds)
{
    m_Serial << millis() << " Reboot in ";
    for (int i=seconds; i>=1; i--) {
        m_Serial << i << ' ';
        delay(1000);
    }
    m_Serial << "\n\n";
    rp2040.reboot();
}
