// Arduino JC_MQTT Library
// A library to send messages to an MQTT broker.
// Derived from the PubSubClient class.
// https://github.com/JChristensen/
// Copyright (C) 2025 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

#ifndef JC_MQTT_H_INCLUDED
#define JC_MQTT_H_INCLUDED
#include <Arduino.h>
#include <PubSubClient.h>
#include <Streaming.h>          // http://arduiniana.org/libraries/streaming/

class JC_MQTT : public PubSubClient
{
    enum m_states_t {CONNECT, WAIT_CONNECT, WAIT, PUBLISH};
    public:
        JC_MQTT(Client& client, HardwareSerial& hws=Serial)
            : PubSubClient{client}, m_connectRetry{10}, m_Serial{hws} {}
        void begin(const char* server, const uint32_t port, const char* topic, const char* clientID);
        void setTopic(const char* topic) {m_pubTopic = topic;}
        void publish(const char* msg);
        bool run();

    private:
        m_states_t m_state {CONNECT};
        uint32_t m_connectRetry;        // connect retry interval, seconds
        int m_retryCount {0};
        static constexpr int m_maxRetries {10};
        uint32_t m_msLastConnect;       // time last connected to the broker
        const char* m_clientID;         // unique ID required for each client
        const char* m_pubTopic;         // the topic to publish to
        const char* m_msg;              // mqtt message text
        bool m_pubFlag;                 // ready to publish
        HardwareSerial& m_Serial;       // alternate serial output
};
#endif

// Arduino JC_MQTT Library
// A library to send messages to an MQTT broker.
// Derived from the PubSubClient class.
// https://github.com/JChristensen/???
// Copyright (C) 2025 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

//#include <JC_MQTT.h>

void JC_MQTT::begin(const char* mqttBroker, uint32_t port, const char* topic, const char* clientID)
{
    m_pubTopic = topic;
    m_clientID = clientID;
    setServer(mqttBroker, 1883);
}

void JC_MQTT::publish(const char* msg)
{
    m_msg = msg;
    m_pubFlag = true;
}

// run the state machine. returns true if connected to the broker.
bool JC_MQTT::run()
{
    switch(m_state) {
        case CONNECT:
            if (connect(m_clientID)) {
                m_state = WAIT;
                m_retryCount = 0;
                m_Serial << millis() << " Connected to MQTT broker\n";
            }
            else {
                m_state = WAIT_CONNECT;
                m_Serial << millis() << " Failed to connect to MQTT broker, rc=" << state() << endl;
                if (++m_retryCount > m_maxRetries) {
                    m_Serial << "Too many retries, rebooting in 5 seconds.\n";
                    delay(5000);
                    rp2040.reboot();
                }
                m_Serial << millis() << " Retry in " << m_connectRetry << " seconds.\n";
                m_msLastConnect = millis();
            }
            break;

        case WAIT_CONNECT:
            if (millis() - m_msLastConnect >= m_connectRetry * 1000) {
                m_state = CONNECT;
            }
            break;

        case WAIT:
            if (connected()) {
                loop();
                if (m_pubFlag) {
                    m_state = PUBLISH;
                }
            }
            else {
                m_state = CONNECT;
                m_Serial << millis() << " Lost connection to MQTT broker\n";
            }
            break;

        case PUBLISH:
            m_state = WAIT;
            m_pubFlag = false;
            m_Serial << millis() << " MQTT publish: " << m_msg << endl;
            PubSubClient::publish(m_pubTopic, m_msg);
            break;
    }
    return (m_state == WAIT || m_state == PUBLISH);
}
