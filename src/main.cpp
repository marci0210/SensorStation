#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiManager.h>

#include <Wire.h>
#include <MySQL_Generic.h>

#include <bsec.h>
#include <EEPROM.h>

#include "config.hpp"

/******************************************************************
 *     Defines
******************************************************************/

#define DEBUG_MODE false
#define BME680_I2C_ADDR 0x76 // I2C address for your sensor (0x76 or 0x77)

/******************************************************************
 *     Variables
******************************************************************/

Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};

WiFiManager wifiManager;

IPAddress server_addr(192,168,0,151); // NAS IP
MySQL_Connection conn((Client *)&client);
uint16_t sendCounter = 0u;

float temp; // °C
float hum;  // %
float pres; // hPa
float gas;  // KOhms
float iaq;   // IAQ
uint8_t iaq_accuracy; // IAQ Accuracy

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

void loadState(Bsec &iaqSensor) {
    EEPROM.begin(512);
    if (EEPROM.read(0) == 0x77) { 
        for (uint16_t i = 0u; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
            bsecState[i] = EEPROM.read(i + 1);
        }
        iaqSensor.setState(bsecState);
        send_message("BSEC state loaded from EEPROM.");
    }
}

void saveState(Bsec &iaqSensor) {
    iaqSensor.getState(bsecState);
    EEPROM.write(0, 0x77); 

    for (uint16_t i = 0u; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
        EEPROM.write(i + 1, bsecState[i]);
    }

    EEPROM.commit();
    send_message("BSEC state saved to EEPROM.");
}

void initialize_sensor() {
    Wire.begin(D2, D1); // SDA, SCL

    iaqSensor.begin(BME680_I2C_ADDR, Wire);

    loadState(iaqSensor);

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

void setup() {
    if (DEBUG_MODE) {
        Serial.begin(115200);
        delay(1000);
    }

    // connect to WiFi
    if (!wifiManager.autoConnect("ESP_SensorKit")) {
        delay(3000);
        ESP.restart();
    }

    initialize_sensor();
}

void loop() {
    if (iaqSensor.run()) {
        sendCounter++;

        // Read sensor datas
        temp = iaqSensor.temperature;
        hum = iaqSensor.humidity;
        pres = iaqSensor.pressure / 100.0; // hPa
        gas = iaqSensor.gasResistance / 1000.0; // KOhms
        iaq = iaqSensor.iaq;

        // Save state if accuracy improved
        if (iaqSensor.iaqAccuracy > iaq_accuracy) {
            saveState(iaqSensor);
            iaq_accuracy = iaqSensor.iaqAccuracy;
        }

        // send data every 20th times
        if (sendCounter >= 20) {
            // Check WiFi and DB connection
            if (WiFi.status() != WL_CONNECTED) {
                send_message("WiFi lost. Reconnecting...");
                if (wifiManager.autoConnect("ESP_SensorKit")) {
                    send_message("WiFi reconnected.");
                } 
                else {
                    send_message("WiFi reconnection failed.");
                }
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

            // Insert data into database
            if (WiFi.status() == WL_CONNECTED && conn.connected()) {
                char query[256];
                sprintf(query, "INSERT INTO myroom.sensor_data (temperature, humidity, pressure, gas, iaq, iaq_accuracy) VALUES (%.2f, %.2f, %.2f, %.2f, %.2f, %d)", temp, hum, pres, gas, iaq, iaq_accuracy);

                send_message("Temp: " + String(temp) + " °C, Humidity: " + String(hum) + " %, Pressure: " + String(pres) + " hPa, Gas: " + String(gas) + " KOhms, IAQ: " + String(iaq) + ", IAQ Accuracy: " + String(iaq_accuracy) + "\n");
                
                MySQL_Query query_executor(&conn);
                if (query_executor.execute(query)) {
                    send_message("Data successfully sent to MySQL!");
                    sendCounter = 0;
                } 
                else {
                    send_message("Insert failed. Check connection or table permissions.");
                }
            }
            else {
                send_message("WiFi or Database connection lost...");
            }
        }
    }
}
