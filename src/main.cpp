#include <Arduino.h>

#include <ESP8266WiFi.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#include <MySQL_Generic.h>

#include "config.hpp"

/******************************************************************
 *     Defines
******************************************************************/

#define DEBUG_MODE false
#define BME680_I2C_ADDR 0x76 // I2C address for your sensor (0x76 or 0x77)

/******************************************************************
 *     Variables
******************************************************************/

Adafruit_BME680 bme; // I2C

IPAddress server_addr(192,168,0,151); // NAS IP
MySQL_Connection conn((Client *)&client);

float temp; // °C
float hum;  // %
float pres; // hPa
float gas;  // KOhms

/******************************************************************
 *     Functions
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

void connect_to_network(const char* ssid, const char* password) {
    send_message("Connecting to SSID: " + String(ssid));

    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int max_attempts = 20; // Max attempts to connect
    int attempt = 0;

    while (WiFi.status() != WL_CONNECTED && attempt < max_attempts) {
        delay(500);
        send_message(".");
        attempt++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        send_message("\nConnected successfully!");
        send_message("IP Address: " + WiFi.localIP().toString());
    } else {
        send_message("\nFailed to connect.");
    }
}

void initialize_sensor() {
    Wire.begin(D2, D1); // SDA, SCL

    if (!bme.begin(BME680_I2C_ADDR)) {
        send_message("Could not find a valid BME680 sensor, check wiring!");
        while (1);
    }

    // Set up oversampling and filter
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320°C for 150 ms
}

void setup() {
    if (DEBUG_MODE) {
        Serial.begin(115200);
        delay(1000);
    }

    initialize_sensor();

    delay(60000); // Wait for sensor to stabilize
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        send_message("WiFi lost. Reconnecting...");
        connect_to_network(ssid, password);
    }
    if (WiFi.status() == WL_CONNECTED && !conn.connected()) {
        send_message("Database disconnected. Reconnecting...");
        conn.close();

        if (conn.connect(server_addr, 3307, db_user, db_password)) {
            send_message("Database connected.");
        } 
        else {
            send_message("Database reconnection failed.");
        }
    }

    if (WiFi.status() == WL_CONNECTED && conn.connected()) {
        if (!bme.performReading()) {
            send_message("Sensor reading failed.");
        }
        else {
            temp = bme.temperature;
            hum = bme.humidity;
            pres = bme.pressure / 100.0; // hPa
            gas = bme.gas_resistance / 1000.0; // KOhms

            char query[256];
            sprintf(query, "INSERT INTO myroom.sensor_data (temperature, humidity, pressure, gas) VALUES (%.2f, %.2f, %.2f, %.2f)", temp, hum, pres, gas);

            send_message("Temp: " + String(temp) + " °C, Humidity: " + String(hum) + " %, Pressure: " + String(pres) + " hPa, Gas: " + String(gas) + " KOhms\n");
            
            MySQL_Query query_executor(&conn);
            if (query_executor.execute(query)) {
                send_message("Data successfully sent to MySQL!");
            } 
            else {
                send_message("Insert failed. Check connection or table permissions.");
            }
        }
    } 

    delay(300000); // Wait for 5 minutes before next reading
}
