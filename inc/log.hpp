#pragma once

#include <Arduino.h>

// true: print status messages on Serial (115200 baud). false: the serial port is never started.
#define DEBUG_MODE false

inline void send_message(const String &msg) {
    if (DEBUG_MODE) {
        Serial.println(msg);
    }
}
