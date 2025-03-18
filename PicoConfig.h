// Class for Raspberry Pi Pico to save and retrieve wifi credentials in EEPROM.
// https://github.com/JChristensen/
// Copyright (C) 2025 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

#ifndef PICO_CONFIG_H_INCLUDED
#define PICO_CONFIG_H_INCLUDED

#include <EEPROM.h>

class PicoConfig
{
    public:
        struct picoCfg {
            char hostname[16] {};   // hostname for the pico
            char ssid[64] {};       // wifi ssid
            char psk[64] {};        // wifi psk
        };
        PicoConfig() {};
        void begin();
        void write();
        void read();
        picoCfg config;

    private:
        static constexpr uint m_eepromSize {256};
        static constexpr uint m_startAddr {0};      // where our data is in eeprom
};
#endif

void PicoConfig::begin() {
    EEPROM.begin(m_eepromSize);
}

void PicoConfig::write() {
    EEPROM.put(m_startAddr, config);
    EEPROM.commit();
}

void PicoConfig::read() {
    EEPROM.get(m_startAddr, config);
}
