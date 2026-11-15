#include <WiFi.h>
#include <WiFiManager.h>
#include <Ticker.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

#include <MySQL_Generic.h>

#include <bsec.h>
#include <EEPROM.h>

// Optional, git-ignored local settings: default DB credentials and the portal's Wi-Fi password.
//   char db_user[] = "...";  char db_password[] = "...";  #define PORTAL_PASSWORD "..."
#if __has_include("config.hpp")
    #include "config.hpp"
    #define DEFAULT_DB_USER         db_user
    #define DEFAULT_DB_PASSWORD     db_password
#else
    #define DEFAULT_DB_USER         ""
    #define DEFAULT_DB_PASSWORD     ""
#endif

#ifdef PORTAL_PASSWORD
    static_assert(sizeof(PORTAL_PASSWORD) - 1 >= 8 && sizeof(PORTAL_PASSWORD) - 1 <= 63,
                  "PORTAL_PASSWORD must be 8..63 characters (WPA2)");
    static const char *portalPassword = PORTAL_PASSWORD;
#else
    #warning "PORTAL_PASSWORD not defined in config.hpp: the setup portal is an OPEN network and DB credentials typed into it are sent unencrypted"
    static const char *portalPassword = NULL;
#endif

/******************************************************************
 *     Defines
******************************************************************/

#define DEBUG_MODE false
#define BME680_I2C_ADDR 0x76 // I2C address for your sensor (0x76 or 0x77)

#define SDA_PIN 15
#define SCL_PIN 0

#define BLUE_LED_PIN 22
#define GREEN_LED_PIN 5
#define WHITE_LED_PIN 23
#define YELLOW_LED_PIN 33
#define RED_LED_PIN 32

// Database: defaults only. The real values are set in the Wi-Fi portal ("Setup" page) and kept in flash.
#define DEFAULT_DB_HOST             "192.168.0.151"   // NAS
#define DEFAULT_DB_PORT             3307
#define DEFAULT_DB_NAME             "myroom"
#define DEFAULT_DB_TABLE            "sensor_data"
#define DB_HOST_MAX_LEN             64
#define DB_USER_MAX_LEN             32
#define DB_PASS_MAX_LEN             64
#define DB_IDENT_MAX_LEN            64                // database / table name (MySQL limit)
#define DB_CFG_NAMESPACE            "dbcfg"

// Timing
#define SEND_EVERY_N_SAMPLES        20u                              // 20 x 3 s (BSEC LP rate) ~= 60 s
#define WIFI_RECONNECT_INTERVAL_MS  (30UL * 1000UL)                  // nudge Wi-Fi every 30 s while offline
#define DB_RECONNECT_INTERVAL_MS    (30UL * 1000UL)                  // min. time between DB connect attempts
#define NO_DATA_RESTART_MS          (15UL * 60UL * 1000UL)           // reboot if nothing was stored for 15 min
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

// Hardware watchdog: reboots if loop() stops running (a DB connect can block for ~50 s)
#define WDT_TIMEOUT_S               120

// EEPROM layout: [0] magic, [1 .. BLOB] BSEC state, [BLOB + 1] IAQ accuracy of the stored state
#define STATE_MAGIC                 0x77
#define STATE_ACCURACY_ADDR         (BSEC_MAX_STATE_BLOB_SIZE + 1)
#define EEPROM_SIZE                 (BSEC_MAX_STATE_BLOB_SIZE + 2)

/******************************************************************
 *     Variables
******************************************************************/

Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint8_t savedAccuracy = 0u;      // IAQ accuracy of the state currently stored in flash
uint32_t lastStateSave = 0u;

WiFiManager wifiManager;
Ticker blueBlinker;
volatile bool blueLedState = false;

char dbHost[DB_HOST_MAX_LEN + 1] = DEFAULT_DB_HOST;   // IP address or host name
uint16_t dbPort = DEFAULT_DB_PORT;
char dbName[DB_IDENT_MAX_LEN + 1] = DEFAULT_DB_NAME;
char dbTable[DB_IDENT_MAX_LEN + 1] = DEFAULT_DB_TABLE;
char dbUser[DB_USER_MAX_LEN + 1] = "";
char dbPassword[DB_PASS_MAX_LEN + 1] = "";

// Portal "Setup" page fields. The password field is never pre-filled: empty = keep the current password.
WiFiManagerParameter dbHostParam("db_host", "Database server (IP address or host name)", "", DB_HOST_MAX_LEN);
WiFiManagerParameter dbPortParam("db_port", "Database port", "", 5, "type='number' min='1' max='65535'");
WiFiManagerParameter dbNameParam("db_name", "Database name", "", DB_IDENT_MAX_LEN);
WiFiManagerParameter dbTableParam("db_table", "Table name", "", DB_IDENT_MAX_LEN);
WiFiManagerParameter dbUserParam("db_user", "Database user", "", DB_USER_MAX_LEN, "autocomplete='off'");
WiFiManagerParameter dbPassParam("db_pass", "Database password (leave empty to keep the current one)", "", DB_PASS_MAX_LEN,
                                 "type='password' autocomplete='new-password'");

MySQL_Connection conn((Client *)&client);   // `client` is the WiFiClient declared by MySQL_Generic

uint32_t sampleCounter = 0u;
uint32_t lastSuccessfulSend = 0u;
uint32_t lastWifiAttempt = 0u;
uint32_t lastDbAttempt = 0u;
bool dbAttempted = false;
bool wifiWasConnected = false;

float temp; // °C
float hum;  // %
float iaq;   // IAQ
uint8_t iaq_accuracy; // IAQ Accuracy of the current sample
uint8_t iaq_color_state = 100u;

/******************************************************************
 *     Helpers
******************************************************************/

void send_message(const char* msg) {
    if (DEBUG_MODE) {
        Serial.println(msg);
    }
}

void send_message(String msg) {
    if (DEBUG_MODE) {
        Serial.println(msg);
    }
}

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

void loadState() {
    EEPROM.begin(EEPROM_SIZE);
    if (EEPROM.read(0) == STATE_MAGIC) {
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
 *     Database settings (configurable in the portal)
******************************************************************/

// Copy the current values into the portal form fields (never the password)
void update_db_params() {
    char portStr[6];
    snprintf(portStr, sizeof(portStr), "%u", (unsigned)dbPort);
    dbHostParam.setValue(dbHost, DB_HOST_MAX_LEN);
    dbPortParam.setValue(portStr, 5);
    dbNameParam.setValue(dbName, DB_IDENT_MAX_LEN);
    dbTableParam.setValue(dbTable, DB_IDENT_MAX_LEN);
    dbUserParam.setValue(dbUser, DB_USER_MAX_LEN);
    dbPassParam.setValue("", DB_PASS_MAX_LEN);
}

void load_db_config() {
    Preferences prefs;
    prefs.begin(DB_CFG_NAMESPACE, false);
    prefs.getString("host", DEFAULT_DB_HOST).toCharArray(dbHost, sizeof(dbHost));
    dbPort = prefs.getUShort("port", DEFAULT_DB_PORT);
    prefs.getString("db", DEFAULT_DB_NAME).toCharArray(dbName, sizeof(dbName));
    prefs.getString("table", DEFAULT_DB_TABLE).toCharArray(dbTable, sizeof(dbTable));
    prefs.getString("user", DEFAULT_DB_USER).toCharArray(dbUser, sizeof(dbUser));
    prefs.getString("pass", DEFAULT_DB_PASSWORD).toCharArray(dbPassword, sizeof(dbPassword));
    prefs.end();

    update_db_params();
    send_message("Database: " + String(dbUser) + "@" + String(dbHost) + ":" + String(dbPort) + " -> " +
                 String(dbName) + "." + String(dbTable));
}

// Host name or IPv4 address: letters, digits, '.', '-'
bool is_valid_host(const String &host) {
    if (host.length() == 0 || host.length() > DB_HOST_MAX_LEN) {
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

// Database / table name. Restricted to letters, digits and '_' because it is inserted into the SQL text.
bool is_valid_identifier(const String &name) {
    if (name.length() == 0 || name.length() > DB_IDENT_MAX_LEN) {
        return false;
    }
    for (size_t i = 0; i < name.length(); i++) {
        char c = name[i];
        if (!isalnum(c) && c != '_') {
            return false;
        }
    }
    return true;
}

// Called by WiFiManager when the "Setup" page is saved. All fields are validated first; if anything is
// invalid nothing is changed.
void on_db_params_saved() {
    String host = String(dbHostParam.getValue());
    String name = String(dbNameParam.getValue());
    String table = String(dbTableParam.getValue());
    String user = String(dbUserParam.getValue());
    String pass = String(dbPassParam.getValue());   // empty = keep the current password
    host.trim();
    name.trim();
    table.trim();
    user.trim();
    long port = String(dbPortParam.getValue()).toInt();

    if (!is_valid_host(host) || port < 1 || port > 65535 ||
        !is_valid_identifier(name) || !is_valid_identifier(table) ||
        user.length() == 0 || user.length() > DB_USER_MAX_LEN || pass.length() > DB_PASS_MAX_LEN) {
        send_message("Invalid database settings, nothing changed.");
        update_db_params();   // show the stored values again
        return;
    }

    host.toCharArray(dbHost, sizeof(dbHost));
    dbPort = (uint16_t)port;
    name.toCharArray(dbName, sizeof(dbName));
    table.toCharArray(dbTable, sizeof(dbTable));
    user.toCharArray(dbUser, sizeof(dbUser));
    if (pass.length() > 0) {
        pass.toCharArray(dbPassword, sizeof(dbPassword));
    }

    Preferences prefs;
    prefs.begin(DB_CFG_NAMESPACE, false);
    prefs.putString("host", dbHost);
    prefs.putUShort("port", dbPort);
    prefs.putString("db", dbName);
    prefs.putString("table", dbTable);
    prefs.putString("user", dbUser);
    prefs.putString("pass", dbPassword);
    prefs.end();

    update_db_params();
    send_message("Database settings saved: " + String(dbUser) + "@" + String(dbHost) + ":" + String(dbPort) +
                 " -> " + String(dbName) + "." + String(dbTable));
}

/******************************************************************
 *     Wi-Fi
******************************************************************/

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

    // Database server fields on a separate "Setup" page, so the address can be changed without
    // re-entering the Wi-Fi password
    wifiManager.addParameter(&dbHostParam);
    wifiManager.addParameter(&dbPortParam);
    wifiManager.addParameter(&dbNameParam);
    wifiManager.addParameter(&dbTableParam);
    wifiManager.addParameter(&dbUserParam);
    wifiManager.addParameter(&dbPassParam);
    wifiManager.setSaveParamsCallback(on_db_params_saved);
    std::vector<const char *> menu = {"wifi", "param", "info", "exit", "sep", "update"};
    wifiManager.setMenu(menu);

    bool connected = false;
    if (forcePortal) {
        // On-demand reconfiguration (double reset): portal first, saved network untouched until new
        // credentials are saved. On timeout fall through to the normal saved-network connect.
        send_message("Double reset detected. Opening config portal.");
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

void db_disconnect();

void handle_wifi() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiWasConnected) {
            wifiWasConnected = true;
            blink_blue_led(false);
            set_blue_led(false);
            db_disconnect();   // the old TCP socket did not survive the Wi-Fi drop
            send_message("WiFi reconnected, IP: " + WiFi.localIP().toString());
        }
        return;
    }

    if (wifiWasConnected) {
        wifiWasConnected = false;
        lastWifiAttempt = millis();
        blink_blue_led(true);
        db_disconnect();
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
 *     Database
******************************************************************/

// Always close the socket explicitly: the MySQL library leaves it open when the login or a query times
// out, so conn.connected() keeps returning true and the firmware would never reconnect.
void db_disconnect() {
    conn.close();
    client.stop();
}

bool ensure_db_connected() {
    if (conn.connected()) {
        return true;
    }
    if (dbAttempted && (millis() - lastDbAttempt < DB_RECONNECT_INTERVAL_MS)) {
        return false;
    }
    dbAttempted = true;
    lastDbAttempt = millis();

    send_message("Database connecting to " + String(dbHost) + ":" + String(dbPort) + "...");
    if (conn.connect(dbHost, dbPort, dbUser, dbPassword)) {
        send_message("Database connected.");
        return true;
    }

    send_message("Database connection failed.");
    db_disconnect();
    return false;
}

bool send_to_db() {
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }
    if (!ensure_db_connected()) {
        return false;
    }

    // dbName / dbTable are validated to [A-Za-z0-9_] when saved, so they are safe to put into the SQL text
    char query[320];
    snprintf(query, sizeof(query),
             "INSERT INTO `%s`.`%s` (temperature, humidity, iaq, iaq_accuracy) VALUES (%.2f, %.2f, %.2f, %u)",
             dbName, dbTable, temp, hum, iaq, (unsigned)iaq_accuracy);

    MySQL_Query query_executor(&conn);
    if (query_executor.execute(query)) {
        send_message("Data successfully sent!");
        return true;
    }

    send_message("Insert failed. Dropping the DB connection.");
    db_disconnect();
    return false;
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

    load_db_config();

    bool doubleReset = check_double_reset();
    connect_wifi(doubleReset);
    initialize_sensor();

    // turn off all LEDs after successful startup
    set_iaq_leds(false, false, false, false);
    iaq_color_state = 100u;

    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    lastSuccessfulSend = millis();
}

void loop() {
    esp_task_wdt_reset();

    handle_wifi();

    if (iaqSensor.run()) {
        sampleCounter++;

        // Read sensor datas
        temp = iaqSensor.temperature;
        hum = iaqSensor.humidity;
        iaq = iaqSensor.iaq;
        iaq_accuracy = iaqSensor.iaqAccuracy;

        maybe_save_state();
        send_message("Sensor data read. Counter state: " + String(sampleCounter));

        // Send data every SEND_EVERY_N_SAMPLES samples; on failure retry with the next sample
        if (sampleCounter >= SEND_EVERY_N_SAMPLES) {
            update_iaq_leds();

            if (send_to_db()) {
                sampleCounter = 0u;
                lastSuccessfulSend = millis();
            }
        }
    }

    // Last-resort recovery: if nothing reached the database for a while (Wi-Fi, DB or sensor stuck),
    // start from a clean state. Wi-Fi credentials and BSEC calibration survive the reboot.
    if (millis() - lastSuccessfulSend >= NO_DATA_RESTART_MS) {
        safe_restart("No data stored for 15 minutes. Restarting...");
    }
}
