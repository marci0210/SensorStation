#include <WiFi.h>
#include <WiFiManager.h>
#include <Ticker.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

#include <bsec.h>
#include <EEPROM.h>

#include "log.hpp"
#include "ingest.hpp"

// Optional, git-ignored local settings: the portal's Wi-Fi password and the default ingest server.
//   #define PORTAL_PASSWORD "..."   #define INGEST_DEFAULT_HOST "..."   #define INGEST_DEFAULT_PORT 8443
#if __has_include("config.hpp")
    #include "config.hpp"
#endif

#ifdef PORTAL_PASSWORD
    static_assert(sizeof(PORTAL_PASSWORD) - 1 >= 8 && sizeof(PORTAL_PASSWORD) - 1 <= 63,
                  "PORTAL_PASSWORD must be 8..63 characters (WPA2)");
    static const char *portalPassword = PORTAL_PASSWORD;
#else
    #warning "PORTAL_PASSWORD not defined in config.hpp: the setup portal is an OPEN network"
    static const char *portalPassword = NULL;
#endif

/******************************************************************
 *     Defines
******************************************************************/

#define BME680_I2C_ADDR 0x76 // I2C address for your sensor (0x76 or 0x77)

#define SDA_PIN 15
#define SCL_PIN 0

#define BLUE_LED_PIN 22
#define GREEN_LED_PIN 5
#define WHITE_LED_PIN 23
#define YELLOW_LED_PIN 33
#define RED_LED_PIN 32

// Persoboard ingest server: defaults only. The real values are set in the Wi-Fi portal ("Setup" page) and
// kept in flash. Local: the PC's IP address or name, port 8443; OpenShift: the ingest Route host, port 443.
// The certificate name is a DNS name from the server certificate; it's needed when connecting by IP address,
// because the TLS library can't match IP addresses (empty = check against the server address).
// The API path is INGEST_PATH in ingest.hpp.
#ifndef INGEST_DEFAULT_HOST
    #define INGEST_DEFAULT_HOST     ""
#endif
#ifndef INGEST_DEFAULT_CERT_NAME
    #define INGEST_DEFAULT_CERT_NAME ""
#endif
#ifndef INGEST_DEFAULT_PORT
    #define INGEST_DEFAULT_PORT     8443
#endif
#define INGEST_HOST_MAX_LEN         128
#define INGEST_CFG_NAMESPACE        "ingest"
#define LEGACY_DB_CFG_NAMESPACE     "dbcfg"           // MySQL settings of older firmware; wiped at boot

// Sensor calibration: temperature offset = station reading - real temperature (self-heating of the board).
// Set in the portal ("Setup" page) and kept in flash; BSEC also corrects the humidity with it.
#define DEFAULT_TEMP_OFFSET         0.0f
#define TEMP_OFFSET_MIN             (-10.0f)
#define TEMP_OFFSET_MAX             10.0f
#define SENSOR_CFG_NAMESPACE        "sensor"

// Timing
#define READING_INTERVAL_MS         (60UL * 1000UL)                  // one reading per minute (BSEC samples every 3 s)
#define WIFI_RECONNECT_INTERVAL_MS  (30UL * 1000UL)                  // nudge Wi-Fi every 30 s while offline
#define WIFI_LOST_RESTART_MS        (15UL * 60UL * 1000UL)           // reboot if Wi-Fi stays down for 15 min
#define NO_RESPONSE_RESTART_MS      (60UL * 60UL * 1000UL)           // reboot if the server hasn't answered for 1 h
#define STATE_SAVE_PERIOD_MS        (6UL * 60UL * 60UL * 1000UL)     // re-save calibrated BSEC state every 6 h
#define BLINK_INTERVAL_MS           500u

// Wi-Fi provisioning (WiFiManager)
#define WIFI_HOSTNAME               "SensorStation"
#define WIFI_PORTAL_SSID            "ESP_SensorKit"
#define WIFI_CONNECT_TIMEOUT_S      30     // per attempt with the saved credentials
#define WIFI_CONNECT_RETRIES        3
#define CONFIG_PORTAL_TIMEOUT_S     180    // close the portal and reboot; saved credentials are kept

// Press RST twice within this window to open the config portal on demand
#define DRD_WINDOW_MS               3000u
#define DRD_NAMESPACE               "drd"
#define DRD_KEY                     "pending"

// Task watchdog: reboots if loop() or the upload task stops running (an HTTPS request takes at most ~30 s)
#define WDT_TIMEOUT_S               60

// EEPROM layout (v2): [0] magic, [1 .. BLOB] BSEC state, [BLOB + 1] IAQ accuracy of the stored state,
// [BLOB + 2 .. BLOB + 5] fingerprint of the BSEC configuration + library version the state belongs to
#define STATE_MAGIC                 0x78    // v2 (with fingerprint)
#define STATE_MAGIC_V1              0x77    // v1 (no fingerprint): ignored, forces one new calibration
#define STATE_ACCURACY_ADDR         (BSEC_MAX_STATE_BLOB_SIZE + 1)
#define STATE_FINGERPRINT_ADDR      (BSEC_MAX_STATE_BLOB_SIZE + 2)
#define EEPROM_SIZE                 (BSEC_MAX_STATE_BLOB_SIZE + 6)

/******************************************************************
 *     Variables
******************************************************************/

Bsec iaqSensor;
// BSEC configuration matching this hardware/firmware: 3.3 V supply, 3 s sample interval (LP mode),
// 4-day calibration history. Must match BSEC_SAMPLE_RATE_LP used in updateSubscription().
const uint8_t bsecConfig[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint32_t bsecFingerprint = 0u;   // identifies bsecConfig + BSEC library version (see compute_bsec_fingerprint)
float tempOffset = DEFAULT_TEMP_OFFSET;
uint8_t savedAccuracy = 0u;      // IAQ accuracy of the state currently stored in flash
uint32_t lastStateSave = 0u;

WiFiManager wifiManager;
Ticker blueBlinker;
volatile bool blueLedState = false;

char ingestHost[INGEST_HOST_MAX_LEN + 1] = INGEST_DEFAULT_HOST;            // where to connect: IP or host name
uint16_t ingestPort = INGEST_DEFAULT_PORT;
char ingestCertName[INGEST_HOST_MAX_LEN + 1] = INGEST_DEFAULT_CERT_NAME;   // empty = same as ingestHost

// Portal "Setup" page fields
WiFiManagerParameter ingestHostParam("pb_host", "Ingest server (IP address or host name)", "", INGEST_HOST_MAX_LEN);
WiFiManagerParameter ingestPortParam("pb_port", "Ingest server port (8443 local, 443 OpenShift)", "", 5,
                                     "type='number' min='1' max='65535'");
WiFiManagerParameter ingestCertNameParam("pb_cert_name",
                                         "Certificate name (a DNS name in the server certificate; required when the "
                                         "server is an IP address, otherwise leave empty)", "", INGEST_HOST_MAX_LEN);
WiFiManagerParameter tempOffsetParam("temp_offset", "Temperature offset in °C (station reading minus real temperature)", "", 6,
                                     "type='number' step='0.1' min='-10' max='10'");

uint32_t sampleCounter = 0u;
uint32_t nextReadingMs = 0u;
uint32_t lastWifiAttempt = 0u;
uint32_t wifiLostSince = 0u;
bool wifiWasConnected = false;

float temp; // °C
float hum;  // %
float iaq;   // BSEC IAQ (BSEC_OUTPUT_IAQ), as the Persoboard contract requires
uint8_t iaq_accuracy; // accuracy of that IAQ value
uint8_t iaq_color_state = 100u;

/******************************************************************
 *     LEDs
******************************************************************/

void set_blue_led(bool on) {
    blueLedState = on;
    digitalWrite(BLUE_LED_PIN, on ? HIGH : LOW);
}

void toggle_blue_led() {
    set_blue_led(!blueLedState);
}

// Blink the blue LED in the background (Ticker), independent of what loop() is doing.
void blink_blue_led(bool enable) {
    if (enable) {
        if (!blueBlinker.active()) {
            blueBlinker.attach_ms(BLINK_INTERVAL_MS, toggle_blue_led);
        }
    }
    else {
        blueBlinker.detach();
    }
}

void set_iaq_leds(bool green, bool white, bool yellow, bool red) {
    digitalWrite(GREEN_LED_PIN, green ? HIGH : LOW);
    digitalWrite(WHITE_LED_PIN, white ? HIGH : LOW);
    digitalWrite(YELLOW_LED_PIN, yellow ? HIGH : LOW);
    digitalWrite(RED_LED_PIN, red ? HIGH : LOW);
}

uint8_t get_iaq_color(uint16_t iaq_value) {
    if (iaq_value <= 50) {
        return 0; // Green
    }
    else if (iaq_value <= 100) {
        return 1; // White
    }
    else if (iaq_value <= 150) {
        return 2; // Yellow
    }
    else {
        return 3; // Red
    }
}

void update_iaq_leds() {
    uint8_t color = get_iaq_color(iaq);
    if (color != iaq_color_state) {
        iaq_color_state = color;
        set_iaq_leds(color == 0, color == 1, color == 2, color == 3);
    }
}

/******************************************************************
 *     BSEC state (calibration) persistence
******************************************************************/

// FNV-1a hash of the BSEC configuration blob and the BSEC library version. A saved state is only valid for
// the configuration and library it was created with; BSEC itself may accept a mismatched state and then
// produce nonsense (e.g. static IAQ far above 500 with accuracy 2), so it is checked here.
uint32_t compute_bsec_fingerprint() {
    uint32_t hash = 2166136261u;
    auto mix = [&hash](uint8_t b) {
        hash ^= b;
        hash *= 16777619u;
    };
    for (size_t i = 0; i < sizeof(bsecConfig); i++) {
        mix(bsecConfig[i]);
    }
    mix(iaqSensor.version.major);
    mix(iaqSensor.version.minor);
    mix(iaqSensor.version.major_bugfix);
    mix(iaqSensor.version.minor_bugfix);
    return hash;
}

void loadState() {
    EEPROM.begin(EEPROM_SIZE);
    uint8_t magic = EEPROM.read(0);
    uint32_t storedFingerprint = 0u;
    EEPROM.get(STATE_FINGERPRINT_ADDR, storedFingerprint);

    if (magic == STATE_MAGIC_V1 || (magic == STATE_MAGIC && storedFingerprint != bsecFingerprint)) {
        send_message("Saved BSEC state was made with another BSEC configuration/version. Starting a new calibration.");
    }
    else if (magic == STATE_MAGIC) {
        for (uint16_t i = 0u; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
            bsecState[i] = EEPROM.read(i + 1);
        }
        iaqSensor.setState(bsecState);

        if (iaqSensor.bsecStatus == BSEC_OK) {
            uint8_t acc = EEPROM.read(STATE_ACCURACY_ADDR);
            savedAccuracy = (acc <= 3u) ? acc : 0u;   // older firmware didn't store this byte
            send_message("BSEC state loaded from EEPROM. Stored accuracy: " + String(savedAccuracy));
        }
        else {
            send_message("Stored BSEC state rejected: " + String((int)iaqSensor.bsecStatus));
        }
    }
    EEPROM.end();
}

void saveState() {
    iaqSensor.getState(bsecState);
    if (iaqSensor.bsecStatus != BSEC_OK) {
        send_message("BSEC getState failed: " + String((int)iaqSensor.bsecStatus));
        return;
    }

    EEPROM.begin(EEPROM_SIZE);
    EEPROM.write(0, STATE_MAGIC);
    for (uint16_t i = 0u; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
        EEPROM.write(i + 1, bsecState[i]);
    }
    EEPROM.write(STATE_ACCURACY_ADDR, iaqSensor.iaqAccuracy);
    EEPROM.put(STATE_FINGERPRINT_ADDR, bsecFingerprint);
    EEPROM.commit();
    EEPROM.end();

    savedAccuracy = iaqSensor.iaqAccuracy;
    lastStateSave = millis();
    send_message("BSEC state saved to EEPROM. Accuracy: " + String(savedAccuracy));
}

// Save when the accuracy improved, and every STATE_SAVE_PERIOD_MS once fully calibrated (Bosch recommendation).
void maybe_save_state() {
    uint8_t acc = iaqSensor.iaqAccuracy;
    bool improved = acc > savedAccuracy;
    bool periodic = (acc >= 3u) && (millis() - lastStateSave >= STATE_SAVE_PERIOD_MS);
    if (improved || periodic) {
        saveState();
    }
}

// Reboot, but keep the latest calibration if it is at least as good as the stored one.
void safe_restart(const char* reason) {
    send_message(reason);
    ingest::prepare_for_restart();   // unsent readings get their timestamps and survive the reboot
    if (iaqSensor.iaqAccuracy > 0u && iaqSensor.iaqAccuracy >= savedAccuracy) {
        saveState();
    }
    delay(200);
    ESP.restart();
}

/******************************************************************
 *     Sensor
******************************************************************/

void initialize_sensor() {
    Wire.begin(SDA_PIN , SCL_PIN );

    iaqSensor.begin(BME680_I2C_ADDR, Wire);
    send_message("BSEC version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." +
                 String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix));

    // Order matters: configuration first, then the saved state (it belongs to this configuration),
    // then the subscription.
    iaqSensor.setConfig(bsecConfig);
    if (iaqSensor.bsecStatus < BSEC_OK) {
        send_message("BSEC setConfig failed: " + String((int)iaqSensor.bsecStatus));
    }
    bsecFingerprint = compute_bsec_fingerprint();   // needs the version, which begin() reads

    // Compensate the self-heating of the board; BSEC applies it to temperature and humidity
    iaqSensor.setTemperatureOffset(tempOffset);
    send_message("Temperature offset: " + String(tempOffset, 1) + " C");

    loadState();

    bsec_virtual_sensor_t sensorList[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
    };

    iaqSensor.updateSubscription(sensorList, sizeof(sensorList) / sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);
}

/******************************************************************
 *     Settings (ingest server + sensor calibration, configurable in the portal)
******************************************************************/

// Copy the current values into the portal form fields
void update_setup_params() {
    char portStr[6];
    snprintf(portStr, sizeof(portStr), "%u", (unsigned)ingestPort);
    char offsetStr[8];
    snprintf(offsetStr, sizeof(offsetStr), "%.1f", tempOffset);

    ingestHostParam.setValue(ingestHost, INGEST_HOST_MAX_LEN);
    ingestPortParam.setValue(portStr, 5);
    ingestCertNameParam.setValue(ingestCertName, INGEST_HOST_MAX_LEN);
    tempOffsetParam.setValue(offsetStr, 6);
}

void load_settings() {
    Preferences prefs;
    prefs.begin(INGEST_CFG_NAMESPACE, false);
    prefs.getString("host", INGEST_DEFAULT_HOST).toCharArray(ingestHost, sizeof(ingestHost));
    ingestPort = prefs.getUShort("port", INGEST_DEFAULT_PORT);
    prefs.getString("certname", INGEST_DEFAULT_CERT_NAME).toCharArray(ingestCertName, sizeof(ingestCertName));
    prefs.end();

    prefs.begin(SENSOR_CFG_NAMESPACE, false);
    tempOffset = prefs.getFloat("toffset", DEFAULT_TEMP_OFFSET);
    prefs.end();

    update_setup_params();
    send_message("Ingest server: " + String(ingestHost[0] ? ingestHost : "(not set)") + ":" + String(ingestPort) +
                 (ingestCertName[0] ? ", certificate name " + String(ingestCertName) : String("")));
}

// Older firmware kept the MySQL settings, including the password in plain text, in flash. They are no
// longer used, so remove them.
void forget_legacy_db_settings() {
    Preferences prefs;
    if (!prefs.begin(LEGACY_DB_CFG_NAMESPACE, true)) {
        return;   // namespace doesn't exist
    }
    bool present = prefs.isKey("host") || prefs.isKey("user") || prefs.isKey("pass");
    prefs.end();
    if (present && prefs.begin(LEGACY_DB_CFG_NAMESPACE, false)) {
        prefs.clear();
        prefs.end();
        send_message("Old database settings removed from flash.");
    }
}

// Temperature offset in °C, e.g. "2.3" or "-0.5" (a decimal comma is accepted too)
bool parse_temp_offset(const String &text, float &out) {
    String s = text;
    s.trim();
    s.replace(',', '.');
    if (s.length() == 0) {
        return false;
    }
    char *end = NULL;
    float value = strtof(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || isnan(value) || value < TEMP_OFFSET_MIN || value > TEMP_OFFSET_MAX) {
        return false;
    }
    out = value;
    return true;
}

// Host name or IPv4 address: letters, digits, '.', '-'
bool is_valid_host(const String &host) {
    if (host.length() == 0 || host.length() > INGEST_HOST_MAX_LEN) {
        return false;
    }
    for (size_t i = 0; i < host.length(); i++) {
        char c = host[i];
        if (!isalnum(c) && c != '.' && c != '-') {
            return false;
        }
    }
    return true;
}

// Called by WiFiManager when the "Setup" page is saved. All fields are validated first; if anything is
// invalid nothing is changed.
void on_setup_saved() {
    String host = String(ingestHostParam.getValue());
    host.trim();
    long port = String(ingestPortParam.getValue()).toInt();
    String certName = String(ingestCertNameParam.getValue());   // optional
    certName.trim();
    float offset = 0.0f;

    if (!is_valid_host(host) || port < 1 || port > 65535 ||
        (certName.length() > 0 && !is_valid_host(certName)) ||
        !parse_temp_offset(String(tempOffsetParam.getValue()), offset)) {
        send_message("Invalid settings, nothing changed.");
        update_setup_params();   // show the stored values again
        return;
    }

    host.toCharArray(ingestHost, sizeof(ingestHost));
    ingestPort = (uint16_t)port;
    certName.toCharArray(ingestCertName, sizeof(ingestCertName));

    Preferences prefs;
    prefs.begin(INGEST_CFG_NAMESPACE, false);
    prefs.putString("host", ingestHost);
    prefs.putUShort("port", ingestPort);
    prefs.putString("certname", ingestCertName);
    prefs.end();

    // The sensor is initialised after the portal closes, so the new offset takes effect right away
    tempOffset = offset;
    prefs.begin(SENSOR_CFG_NAMESPACE, false);
    prefs.putFloat("toffset", tempOffset);
    prefs.end();

    update_setup_params();
    send_message("Settings saved: ingest server " + String(ingestHost) + ":" + String(ingestPort) +
                 (ingestCertName[0] ? ", certificate name " + String(ingestCertName) : String("")) +
                 ", temperature offset " + String(tempOffset, 1) + " C");
}

/******************************************************************
 *     Wi-Fi
******************************************************************/

void refuse_portal_ota() {
    send_message("Firmware upload through the portal is disabled. Restarting.");
    delay(200);
    ESP.restart();
}

void on_config_portal_started(WiFiManager *wm) {
    // Saved network not reachable (or none saved): portal is open -> blue solid, IAQ LEDs off
    blink_blue_led(false);
    set_blue_led(true);
    set_iaq_leds(false, false, false, false);
    send_message("Config portal started: " + String(WIFI_PORTAL_SSID));
}

// Double reset detection. A flag in flash means "the previous boot happened less than DRD_WINDOW_MS ago".
// RTC memory can't be used for this: on the ESP32 the RST/EN button power-cycles the whole chip.
// Costs DRD_WINDOW_MS of boot time (the IAQ LEDs are lit during it = "press RST again now").
bool check_double_reset() {
    Preferences prefs;
    prefs.begin(DRD_NAMESPACE, false);

    if (prefs.getBool(DRD_KEY, false)) {
        prefs.putBool(DRD_KEY, false);
        prefs.end();
        return true;
    }

    prefs.putBool(DRD_KEY, true);
    delay(DRD_WINDOW_MS);   // a reset during this delay boots with the flag still set
    prefs.putBool(DRD_KEY, false);
    prefs.end();
    return false;
}

void connect_wifi(bool forcePortal) {
    WiFi.mode(WIFI_STA);

    wifiManager.setDebugOutput(DEBUG_MODE);
    wifiManager.setHostname(WIFI_HOSTNAME);
    wifiManager.setWiFiAutoReconnect(true);
    // Try the saved network several times before opening the portal (the router may still be booting
    // after a power cut), and never let the portal block forever: on timeout we reboot and retry the
    // saved credentials, which are never erased.
    wifiManager.setConnectTimeout(WIFI_CONNECT_TIMEOUT_S);
    wifiManager.setConnectRetries(WIFI_CONNECT_RETRIES);
    wifiManager.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT_S);
    wifiManager.setAPClientCheck(true);   // keep the portal open while a phone is connected to it
    wifiManager.setAPCallback(on_config_portal_started);

    // Ingest server fields on a separate "Setup" page, so the address can be changed without
    // re-entering the Wi-Fi password
    wifiManager.addParameter(&ingestHostParam);
    wifiManager.addParameter(&ingestPortParam);
    wifiManager.addParameter(&ingestCertNameParam);
    wifiManager.addParameter(&tempOffsetParam);
    wifiManager.setSaveParamsCallback(on_setup_saved);
    // No firmware upload from the portal: it would accept unsigned images, and a modified firmware could read
    // the device credentials. The menu entry is hidden, and an upload sent to the URL directly is refused.
    std::vector<const char *> menu = {"wifi", "param", "info", "exit"};
    wifiManager.setMenu(menu);
    wifiManager.setPreOtaUpdateCallback(refuse_portal_ota);

    bool connected = false;
    if (forcePortal) {
        // On-demand reconfiguration (double reset, or no ingest server set yet): portal first, saved network
        // untouched until new credentials are saved. On timeout fall through to the normal saved-network connect.
        send_message("Opening config portal.");
        connected = wifiManager.startConfigPortal(WIFI_PORTAL_SSID, portalPassword);
        if (!connected) {
            send_message("Portal closed without new settings. Using the saved network.");
            set_blue_led(false);
            set_iaq_leds(true, true, true, true);
        }
    }

    if (!connected) {
        blink_blue_led(true);
        connected = wifiManager.autoConnect(WIFI_PORTAL_SSID, portalPassword);
    }
    if (!connected) {
        send_message("No Wi-Fi and portal timed out. Restarting...");
        delay(1000);
        ESP.restart();
    }
    blink_blue_led(false);
    set_blue_led(false);

    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);   // modem power-save causes disconnects with many routers
    wifiWasConnected = true;
    send_message("WiFi connected, IP: " + WiFi.localIP().toString());
}

void handle_wifi() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiWasConnected) {
            wifiWasConnected = true;
            blink_blue_led(false);
            set_blue_led(false);
            send_message("WiFi reconnected, IP: " + WiFi.localIP().toString());
        }
        return;
    }

    if (wifiWasConnected) {
        wifiWasConnected = false;
        lastWifiAttempt = millis();
        wifiLostSince = millis();
        blink_blue_led(true);
        send_message("WiFi lost.");
    }

    // Auto-reconnect normally handles it; nudge periodically in case the driver gave up.
    // (reconnect() reuses the stored credentials and never erases them.)
    if (millis() - lastWifiAttempt >= WIFI_RECONNECT_INTERVAL_MS) {
        lastWifiAttempt = millis();
        send_message("WiFi reconnecting...");
        WiFi.reconnect();
    }
}

/******************************************************************
 *     Arduino entry points
******************************************************************/

void setup() {
    if (DEBUG_MODE) {
        Serial.begin(115200);
        delay(1000);
    }
    send_message("Reset reason: " + String((int)esp_reset_reason()));

    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(WHITE_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    set_blue_led(false);

    // Turn on all IAQ LEDs to indicate startup
    set_iaq_leds(true, true, true, true);

    load_settings();
    forget_legacy_db_settings();
    ingest::self_test();

    // Open the portal on a double reset, and also while no ingest server is set (nothing could be sent)
    bool doubleReset = check_double_reset();
    connect_wifi(doubleReset || ingestHost[0] == '\0');
    initialize_sensor();

    // turn off all LEDs after successful startup
    set_iaq_leds(false, false, false, false);
    iaq_color_state = 100u;

    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    ingest::begin(ingestHost, ingestPort, ingestCertName);   // after the watchdog: the upload task registers with it
    nextReadingMs = millis();
}

void loop() {
    esp_task_wdt_reset();

    handle_wifi();

    if (iaqSensor.run()) {
        sampleCounter++;

        // Read sensor datas
        temp = iaqSensor.temperature;   // heat-compensated, temperature offset applied
        hum = iaqSensor.humidity;       // heat-compensated
        iaq = iaqSensor.iaq;            // BSEC_OUTPUT_IAQ (not the static IAQ)
        iaq_accuracy = iaqSensor.iaqAccuracy;

        maybe_save_state();
        send_message("#" + String(sampleCounter) + " T=" + String(temp, 2) + " H=" + String(hum, 2) +
                     " IAQ=" + String(iaq, 1) + " (acc " + String(iaq_accuracy) + ")" +
                     " sIAQ=" + String(iaqSensor.staticIaq, 1) + " (acc " + String(iaqSensor.staticIaqAccuracy) + ")" +
                     " gas=" + String(iaqSensor.gasResistance / 1000.0f, 1) + " kOhm");

        // Once a minute: queue the latest values with the time they were produced. The upload task sends
        // them (and anything buffered while offline); this never waits for the network.
        if ((int32_t)(millis() - nextReadingMs) >= 0) {
            nextReadingMs += READING_INTERVAL_MS;
            if ((int32_t)(millis() - nextReadingMs) >= 0) {
                nextReadingMs = millis() + READING_INTERVAL_MS;   // fell behind (sensor stalled): restart the schedule
            }
            update_iaq_leds();
            ingest::enqueue(temp, hum, iaq, iaq_accuracy);
        }
    }

    // Last-resort recovery. Buffered readings, Wi-Fi credentials and the BSEC calibration survive the reboot.
    if (!wifiWasConnected && millis() - wifiLostSince >= WIFI_LOST_RESTART_MS) {
        safe_restart("Wi-Fi down for 15 minutes. Restarting...");
    }
    if (ingestHost[0] != '\0' && ingest::ms_since_server_response() >= NO_RESPONSE_RESTART_MS) {
        safe_restart("No answer from the ingest server for 1 hour. Restarting...");
    }
}
