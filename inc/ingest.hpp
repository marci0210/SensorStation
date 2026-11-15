#pragma once

// Sends readings to the Persoboard ingest server: HTTPS with mutual TLS, every request signed with
// HMAC-SHA256 (see docs/esp32-firmware.md). Readings wait in a ring buffer until the server confirms them,
// and a background task does the sending, so the sensor loop is never blocked by the network.

#include <Arduino.h>

// API path on the ingest server (the same locally and on OpenShift)
#ifndef INGEST_PATH
    #define INGEST_PATH "/api/room-climate/readings"
#endif

namespace ingest {

// Checks the signing code against the test vector from docs/esp32-firmware.md. Can run before begin().
bool self_test();

// Starts NTP and the upload task. `address` is where to connect (host name or IP address). `certName` is the
// name the server certificate must contain; empty = the address itself. The TLS library can only match DNS
// names, not IP addresses, so when connecting by IP, set `certName` to a DNS name from the server certificate.
void begin(const char *address, uint16_t port, const char *certName);

// Queues a reading taken now. Never blocks on the network.
void enqueue(float temperature, float humidity, float iaq, uint8_t iaqAccuracy);

// Call right before a planned restart: gives the queued readings their final timestamps, so they
// survive the reboot (the buffer is kept in RTC memory across software resets).
void prepare_for_restart();

// True once NTP has set the clock
bool clock_valid();

// Number of readings waiting to be sent
size_t pending();

// Milliseconds since the server last answered (any HTTP status), or since begin() if it never did
uint32_t ms_since_server_response();

}  // namespace ingest
