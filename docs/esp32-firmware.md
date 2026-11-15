# ESP32 firmware: sending Room Climate readings to Persoboard

The ESP32 reads the BME680 through Bosch BSEC and, once a minute, POSTs the latest reading to the Persoboard **ingest server** over HTTPS with mutual TLS, signing every request with HMAC-SHA256. This document is everything the firmware repo needs; it does not assume access to the Persoboard code.

A working reference sketch lives in the Persoboard repo at `examples/esp32/persoboard_ingest/persoboard_ingest.ino`. The sections below are the contract that sketch follows.

## Checklist

- [ ] `secrets.h` from Persoboard included, and git-ignored in this repo
- [ ] Wi-Fi connect with automatic reconnect
- [ ] NTP time sync before the first send, and re-sync every few hours
- [ ] TLS that trusts only `PB_CA_CERT`, presents `PB_CLIENT_CERT` / `PB_CLIENT_KEY`, and verifies the server host name
- [ ] JSON body exactly as in [The request](#3-the-request), no extra fields
- [ ] HMAC-SHA256 signature as in [Signing](#4-signing); passes the test vector
- [ ] Response handling and a retry buffer as in [Responses and retries](#5-responses-and-retries)
- [ ] BSEC outputs mapped as in [Sensor values](#6-sensor-values-bme680--bsec)
- [ ] Production hardening as in [Device hardening](#7-device-hardening-production)
- [ ] All [acceptance tests](#8-acceptance-tests) pass

## 1. Credentials (`secrets.h`)

Each device has its own credentials, generated on the Persoboard side:

```powershell
python scripts/pki.py device bedroom-esp32
```

This writes `certs/devices/bedroom-esp32/secrets.h`. Copy it into the firmware project (next to the sketch, or into `include/` for PlatformIO). It defines:

| Symbol | What it is |
| --- | --- |
| `PB_DEVICE_ID` | Device id, sent in `X-PB-Device` (e.g. `bedroom-esp32`) |
| `PB_HMAC_KEY_HEX` | 32-byte signing key as 64 hex characters; decode to bytes once at boot |
| `PB_CA_CERT` | Persoboard CA certificate (PEM): the **only** CA the device trusts |
| `PB_CLIENT_CERT` | This device's certificate (PEM), for mutual TLS |
| `PB_CLIENT_KEY` | This device's private key (PEM, ECDSA P-256) |

Rules:

- Add `secrets.h` to this repo's `.gitignore` **before** copying it in. It is a secret: never commit, log or print any part of it.
- One `secrets.h` per physical device. Two devices must never share credentials.
- Keys are ECDSA P-256, supported by ESP32 mbedTLS out of the box.

## 2. Connection

| Setting | Local (development) | OpenShift (production) |
| --- | --- | --- |
| Host | The PC's LAN IP or name | The ingest Route host, e.g. `ingest-persoboard.apps.<cluster>` |
| Port | `8443` | `443` |
| Path | `/api/room-climate/readings` | `/api/room-climate/readings` |

Make host, port and path build-time settings (e.g. a `config.h` or PlatformIO `build_flags`), not hard-coded in several places.

- **The host must be one of the names in the server certificate.** On the Persoboard side, `python scripts/pki.py server --host <name> --host <ip> …` sets them; tell the server owner which host the device uses.
- **Time first.** Sync NTP (`configTime(0, 0, "pool.ntp.org", "time.google.com")`) and wait until `time(nullptr) > 1700000000` before any HTTPS call. TLS checks certificate dates, and the server rejects requests whose timestamp is more than 5 minutes off. Re-sync at least every 6 hours.
- **TLS setup** with `WiFiClientSecure`:

  ```cpp
  WiFiClientSecure tls;
  tls.setCACert(PB_CA_CERT);           // trust only the Persoboard CA
  tls.setCertificate(PB_CLIENT_CERT);  // mutual TLS
  tls.setPrivateKey(PB_CLIENT_KEY);
  ```

  Never call `setInsecure()` and never add public root CAs: pinning the Persoboard CA is what blocks man-in-the-middle attacks.
- Use `HTTPClient::begin(tls, host, port, path, true)` so the TLS host name (SNI) is the configured host.
- HTTP timeout: 10 s. A new TLS connection per request is fine (the server closes idle connections after 5 s).

## 3. The request

```
POST /api/room-climate/readings HTTP/1.1
Host: <host>
Content-Type: application/json
X-PB-Device: bedroom-esp32
X-PB-Timestamp: 1794757928
X-PB-Nonce: 0123456789abcdef0123456789abcdef
X-PB-Signature: 589630b1b0d6af955acc43a5724d0bb735ccc9135a5c4dd44bcf3d87ef373f16

{"timestamp":"2026-11-15T15:52:08Z","temperature":22.40,"humidity":45.10,"iaq":61.5,"iaq_accuracy":3}
```

### Headers

| Header | Value |
| --- | --- |
| `X-PB-Device` | `PB_DEVICE_ID` |
| `X-PB-Timestamp` | Current Unix time in seconds (UTC), decimal digits only. Time of **sending**, not of the reading. |
| `X-PB-Nonce` | 16 random bytes from the hardware RNG (`esp_fill_random`) as 32 **lowercase** hex characters. New for every request, including retries. |
| `X-PB-Signature` | 64 lowercase hex characters; see [Signing](#4-signing) |

### Body

JSON object with exactly these fields; any other field is rejected:

| Field | Type | Rule |
| --- | --- | --- |
| `timestamp` | string | Time the reading was **taken**, UTC, ISO 8601: `YYYY-MM-DDTHH:MM:SSZ`. Optional (the server then uses the time it receives the request), but always send it so buffered readings keep their real time. Must not be more than 5 minutes in the future; older is fine. |
| `temperature` | number | °C, −40 to 85 |
| `humidity` | number | % relative humidity, 0 to 100 |
| `iaq` | number | BSEC IAQ index, 0 to 500 |
| `iaq_accuracy` | integer | BSEC IAQ accuracy, 0 to 3 |

- Maximum body size 4 KB (a reading is about 100 bytes).
- The signature covers the body byte for byte: build the body string once, sign that exact string, and send that exact string.
- Readings are keyed by `timestamp`: sending the same timestamp again overwrites that reading. That makes retries safe.
- At most 30 requests per minute per device; the normal rate is 1 per minute.

## 4. Signing

```
string_to_sign = "PB1"            + "\n"
               + "POST"           + "\n"
               + path             + "\n"     // "/api/room-climate/readings"
               + X-PB-Timestamp   + "\n"
               + X-PB-Nonce       + "\n"
               + hex(SHA-256(body))          // lowercase hex

X-PB-Signature = hex(HMAC-SHA256(key = bytes of PB_HMAC_KEY_HEX, message = string_to_sign))
```

No trailing newline. All hex is lowercase.

### Test vector

Verify the implementation against this before talking to a server (e.g. a unit test, or a self-test in debug builds):

| Input | Value |
| --- | --- |
| Key (hex) | `000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f` |
| Path | `/api/room-climate/readings` |
| Timestamp | `1794757928` |
| Nonce | `0123456789abcdef0123456789abcdef` |
| Body | `{"timestamp":"2026-11-15T15:52:08Z","temperature":22.40,"humidity":45.10,"iaq":61.5,"iaq_accuracy":3}` |
| SHA-256(body) | `e1b8204326a983f9997b0772e445a83e31409ac9152d86b701fb20f14bd2f2d6` |
| **Signature** | `589630b1b0d6af955acc43a5724d0bb735ccc9135a5c4dd44bcf3d87ef373f16` |

These values come from the server's own signing code.

### Reference code (mbedTLS, bundled with the ESP32 Arduino core and ESP-IDF)

```cpp
#include <mbedtls/md.h>
#include <esp_random.h>

static String toHex(const uint8_t* data, size_t len) {
  static const char digits[] = "0123456789abcdef";
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    out += digits[data[i] >> 4];
    out += digits[data[i] & 0x0f];
  }
  return out;
}

static String sha256Hex(const String& data) {
  uint8_t digest[32];
  mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
             (const uint8_t*)data.c_str(), data.length(), digest);
  return toHex(digest, sizeof digest);
}

static String hmacSha256Hex(const uint8_t* key, size_t keyLen, const String& message) {
  uint8_t digest[32];
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), key, keyLen,
                  (const uint8_t*)message.c_str(), message.length(), digest);
  return toHex(digest, sizeof digest);
}

static String randomNonce() {
  uint8_t bytes[16];
  esp_fill_random(bytes, sizeof bytes);  // hardware RNG; Wi-Fi on gives true randomness
  return toHex(bytes, sizeof bytes);
}

// timestamp = String((unsigned long)time(nullptr)) at send time
static String signRequest(const uint8_t* key, size_t keyLen, const char* path,
                          const String& timestamp, const String& nonce, const String& body) {
  String toSign = String("PB1\nPOST\n") + path + "\n" + timestamp + "\n" + nonce + "\n" + sha256Hex(body);
  return hmacSha256Hex(key, keyLen, toSign);
}
```

## 5. Responses and retries

| Status | Meaning | Action |
| --- | --- | --- |
| `201` | Stored; body echoes the reading | Drop it from the buffer |
| `401` | Signature, device id, clock or nonce rejected (reason only in the server log) | Keep the reading. Re-sync NTP, then retry with backoff. If it persists, the device's credentials or the server's key list are wrong: log it, keep buffering. |
| `413` | Body over 4 KB | Bug: drop the reading and log it |
| `422` | Body invalid (field missing or unknown, value out of range, timestamp in the future) | Drop the reading and log the response body; retrying cannot succeed |
| `429` | Rate limit (30 per minute) | Keep; back off at least 60 s |
| `503` | Server up, database unreachable | Keep; retry with backoff |
| TLS or network error, timeout | Server unreachable, or certificate problem | Keep; retry with backoff. A certificate error that persists means a wrong host name, CA or clock. |

Retry rules:

- **Buffer unsent readings** in a ring buffer: at least 120 entries (2 hours at 1 per minute). When full, drop the oldest. Use RTC memory if the device deep-sleeps.
- Send the oldest reading first; send the rest of the buffer right after a success.
- Every attempt is a new request: new `X-PB-Timestamp`, new `X-PB-Nonce`, new signature. The **body stays the same**, so the reading keeps its own `timestamp` and a duplicate delivery just overwrites itself.
- Backoff: 5 s, doubling to a 5-minute maximum, with ±20 % random jitter. Reset after a `201`.
- Keep measuring while offline: sending must never block reading the sensor or feeding BSEC.

## 6. Sensor values (BME680 + BSEC)

| Body field | BSEC output |
| --- | --- |
| `temperature` | `BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE` |
| `humidity` | `BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY` |
| `iaq` | `BSEC_OUTPUT_IAQ`: the indoor IAQ, not `STATIC_IAQ` |
| `iaq_accuracy` | Accuracy reported with `BSEC_OUTPUT_IAQ` (0–3) |

- Run BSEC at its own sample rate (e.g. LP, every 3 s) and send the **latest** values once a minute, with the time they were produced.
- Save the BSEC state (`bsec_get_state`) to NVS every few hours and after accuracy first reaches 3, and restore it at boot. Without this, every reboot drops IAQ accuracy back to 0 for hours.
- Send readings while accuracy is still 0–2 as well; the dashboard shows the accuracy next to the IAQ.

## 7. Device hardening (production)

- **Flash encryption** and **Secure Boot v2**, so the key and certificate cannot be read off the flash chip. Both burn eFuses and cannot be undone: try them on a spare board first, and use a development key until the firmware is stable.
- If credentials are stored in NVS instead of compiled in, enable **NVS encryption**.
- Never log `secrets.h` values or the request headers; log status codes and the response body only.
- If OTA updates are added, accept only **signed** images.
- Disable JTAG and lower the log level in release builds.
- Run a task watchdog so a stuck TLS call reboots the device instead of silently stopping.

## 8. Acceptance tests

Run against a local ingest server (`.\start_ingest.ps1` on the Persoboard PC) with a server certificate that includes that PC's IP.

| # | Test | Expected |
| --- | --- | --- |
| 1 | Signing test vector | Exact match with [the expected signature](#test-vector) |
| 2 | Normal send | `201`; the reading appears on the Persoboard dashboard within a minute |
| 3 | Run 60 minutes | 60 readings stored, none missing |
| 4 | Stop the ingest server for 10 minutes, then start it | Readings buffered and delivered afterwards, with their original timestamps; no gaps |
| 5 | Set the device clock 10 minutes ahead (skip NTP once) | `401`, then NTP re-sync and success |
| 6 | Replace `PB_CA_CERT` with an unrelated CA | TLS fails: no request is sent |
| 7 | Connect to the server by a host name that is not in its certificate | TLS fails |
| 8 | Flip one character of `PB_HMAC_KEY_HEX` | `401`, reading kept in the buffer |
| 9 | Reboot after BSEC accuracy reached 3 | Accuracy comes back from the saved state, not from 0 |

## 9. Credential changes

| Situation | What changes in the firmware |
| --- | --- |
| Server moved or new host name (e.g. to OpenShift) | Only the host and port settings. The server certificate is re-issued from the same CA, so `secrets.h` stays the same. |
| Device credentials rotated (`scripts/pki.py device <id> --rotate`) | New `secrets.h`; reflash |
| Device revoked | Nothing to do: the server drops its key and every request gets `401` |
| CA replaced (rare; everything re-issued) | New `secrets.h` for every device |

## Appendix: ESP8266

The same protocol works on ESP8266 with BearSSL instead of mbedTLS:

- `BearSSL::WiFiClientSecure` with `setTrustAnchors(new BearSSL::X509List(PB_CA_CERT))` and `setClientECCert(new BearSSL::X509List(PB_CLIENT_CERT), new BearSSL::PrivateKey(PB_CLIENT_KEY), BR_KEYTYPE_KEYX | BR_KEYTYPE_SIGN, BR_KEYTYPE_EC)`.
- SHA-256 and HMAC come from `bearssl_hash.h` / `bearssl_hmac.h` (or the `Crypto` library).
- The ESP8266 has less RAM for TLS; call `setBufferSizes(1024, 1024)` if the handshake runs out of memory.
- No flash encryption on ESP8266: anyone with physical access can read the credentials, so prefer the ESP32 for this sensor.
