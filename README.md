# SensorStation – ESP32 + Bosch BME680 Indoor Air Quality Monitor

An ESP32 firmware (PlatformIO / Arduino framework) that reads a **Bosch BME680** environmental sensor through the
**Bosch BSEC** library and does three things with the readings:

- computes an **Indoor Air Quality (IAQ)** index plus temperature and humidity,
- shows the current air quality on a set of **status LEDs** (green / white / yellow / red),
- logs the readings to a **MySQL / MariaDB** database on the local network (for example a NAS) about once a minute.

Wi-Fi credentials are not hard-coded. On first boot the device opens a **WiFiManager** captive portal where you
pick the network.

---

## Table of contents

- [Features](#features)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [LED indicators](#led-indicators)
- [Software and dependencies](#software-and-dependencies)
- [Configuration](#configuration)
- [Database setup](#database-setup)
- [Build and flash](#build-and-flash)
- [First boot and Wi-Fi setup](#first-boot-and-wi-fi-setup)
- [How it works](#how-it-works)
- [Project structure](#project-structure)
- [Troubleshooting](#troubleshooting)
- [Known limitations / notes](#known-limitations--notes)
- [License](#license)

---

## Features

- **BSEC-based IAQ**: IAQ index (0–500), IAQ accuracy (0–3), heat-compensated temperature and humidity.
- **Persistent calibration**: the BSEC calibration state is saved to flash when the IAQ accuracy improves, every 6 h
  once fully calibrated, and before any self-restart. It is restored on boot.
- **Status LEDs**: one LED shows Wi-Fi status and four LEDs show the air-quality level.
- **Captive-portal Wi-Fi setup** through [WiFiManager](https://github.com/tzapu/WiFiManager). The portal closes
  after 3 minutes so the device never gets stuck in setup mode.
- **Self-healing connectivity**: Wi-Fi reconnect with periodic retries, the database socket closed on every failure,
  a reboot when nothing has been stored for 15 minutes, and a hardware task watchdog for hangs.
- **Database logging**: rows are inserted into a MySQL/MariaDB table through
  [MySQL_MariaDB_Generic](https://github.com/khoih-prog/MySQL_MariaDB_Generic). A PostgreSQL schema is also included
  (see [Database setup](#database-setup)).

---

## Hardware

| Component | Notes |
|---|---|
| **ESP32 board** | **WEMOS LOLIN32** (`board = lolin32`). Other ESP32 dev boards work too if you change `board` in `platformio.ini` and check the pins. |
| **Bosch BME680 breakout** | I²C mode, address **`0x76`** (set to `0x77` if your board's SDO pin is pulled high). |
| **5 × LEDs** | Blue, green, white, yellow, red (3 mm or 5 mm). |
| **5 × resistors** | 220 Ω for green, yellow and red; 100 Ω for blue and white (see the note below). |
| Power | USB 5 V to the ESP32 board. Power the BME680 from **3.3 V**. |
| Database server | Any MySQL/MariaDB server reachable on the LAN (originally a NAS at `192.168.0.151:3307`). |

> **Resistor values:** the GPIOs drive 3.3 V. Red, yellow and green LEDs drop about 2 V, so 220 Ω gives roughly
> 5–6 mA. Blue and white LEDs drop about 2.8–3.1 V, which leaves very little headroom at 3.3 V; with 220 Ω they would
> be barely visible, so they get 100 Ω. If an LED is too bright or too dim, adjust its resistor.

---

## Wiring

![SensorStation wiring schematic](docs/schematic.svg)

All pins are defined at the top of [src/main.cpp](src/main.cpp). The schematic source is
[docs/schematic.svg](docs/schematic.svg).

### BME680 (I²C)

| BME680 pin | Connect to | Notes |
|---|---|---|
| VCC / VIN | ESP32 **3V3** | Not 5 V |
| GND | ESP32 **GND** | |
| SDA (sometimes labelled SDI) | **GPIO 15** | |
| SCL (sometimes labelled SCK) | **GPIO 0** | |
| SDO | **GND** | Selects address `0x76` (tie to 3V3 for `0x77`) |
| CS | **3V3** | Selects I²C mode (often already done on the breakout) |

> ⚠️ GPIO 0 and GPIO 15 are ESP32 **strapping pins**. The I²C pull-up resistors on the breakout keep them high, which
> is the normal boot state, so this works. Still, nothing should pull these lines low while the board resets. If you
> run into boot problems, move I²C to the ESP32 defaults (SDA 21 / SCL 22). GPIO 22 is currently taken by the blue
> LED, so you would need to move that LED too.

### LEDs

Every LED is wired the same way: **GPIO → resistor → LED anode (long leg)**, and **LED cathode (short leg, flat
side) → GND**. They are active-HIGH.

| LED | ESP32 GPIO | Resistor | Meaning |
|---|---|---|---|
| Blue | **GPIO 22** | 100 Ω | Wi-Fi status |
| Green | **GPIO 5** | 220 Ω | IAQ 0–50 |
| White | **GPIO 23** | 100 Ω | IAQ 51–100 |
| Yellow | **GPIO 33** | 220 Ω | IAQ 101–150 |
| Red | **GPIO 32** | 220 Ω | IAQ > 150 |

> The LOLIN32's **onboard LED is also on GPIO 5** but is active-LOW, so it is lit whenever the green LED is off.
> This is expected.

---

## LED indicators

### Startup

| LED state | Meaning |
|---|---|
| Green, white, yellow and red **on**, blue **blinking** | Booting; connecting to the saved Wi-Fi (up to 3 × 30 s) |
| Blue **solid on**, all others off | Saved Wi-Fi not reachable (or none saved); the **configuration portal** is open for 3 minutes, then the board reboots and tries again |
| All LEDs **off** | Startup finished; waiting for the first IAQ result |

### Normal operation

| LED state | Meaning |
|---|---|
| Blue **blinking** (0.5 s) | Wi-Fi lost; reconnecting |
| Blue **off** | Wi-Fi connected |
| One of green / white / yellow / red on | Current air-quality level (see below) |

The IAQ LED updates at each database send cycle (about once a minute), not on every sample.

### IAQ colour scale

The thresholds follow the Bosch BSEC IAQ scale:

| IAQ index | Bosch classification | LED |
|---|---|---|
| 0 – 50 | Excellent | 🟢 Green |
| 51 – 100 | Good | ⚪ White |
| 101 – 150 | Lightly polluted | 🟡 Yellow |
| 151 – 500 | Moderately to extremely polluted | 🔴 Red |

---

## Software and dependencies

- [PlatformIO](https://platformio.org/) (VS Code extension `platformio.platformio-ide` recommended)
- Platform: `espressif32 @ ^6.12.0` (Arduino-ESP32 core 2.0.x), framework: `arduino`

Libraries (PlatformIO downloads them automatically from `lib_deps`):

| Library | Version | Purpose |
|---|---|---|
| [`boschsensortec/BSEC Software Library`](https://github.com/boschsensortec/BSEC-Arduino-library) | `^1.8.1492` | BME680 driver + BSEC IAQ algorithm (includes a precompiled `libalgobsec.a` for ESP32) |
| [`khoih-prog/MySQL_MariaDB_Generic`](https://github.com/khoih-prog/MySQL_MariaDB_Generic) | `^1.7.2` | MySQL/MariaDB client |
| [`tzapu/WiFiManager`](https://github.com/tzapu/WiFiManager) | latest from GitHub | Wi-Fi captive portal |

The platform is pinned to the 6.x line because the task-watchdog call (`esp_task_wdt_init(timeout, panic)`) uses the
core 2.x API. Core 3.x changed that signature.

`platformio.ini` also sets a `lib_ignore` list (WiFiNINA, WiFi101, UIPEthernet, …). These are optional transports
that MySQL_MariaDB_Generic would otherwise pull in, and they don't compile for the ESP32. `lib_ldf_mode = chain+`
is needed so the library dependency finder evaluates the `#define`s correctly.

---

## Configuration

### 1. Database credentials: `inc/config.hpp` (required, not in git)

The database user and password are stored in `inc/config.hpp`. This file is **ignored by git** (`**/config.hpp` in
`.gitignore`), so after a fresh clone you have to create it:

```cpp
// inc/config.hpp
#pragma once

char db_user[]     = "your_db_user";
char db_password[] = "your_db_password";
```

### 2. Settings in `src/main.cpp`

**Hardware and network**

| Setting | Default | Description |
|---|---|---|
| `DEBUG_MODE` | `false` | Set to `true` for Serial output (115200 baud). When `false`, the serial port is never started. |
| `BME680_I2C_ADDR` | `0x76` | Sensor I²C address (`0x76` / `0x77`) |
| `SDA_PIN` / `SCL_PIN` | `15` / `0` | I²C pins |
| `*_LED_PIN` | see [Wiring](#leds) | LED pins |
| `server_addr` | `192.168.0.151` | IP address of the MySQL/MariaDB server |
| `DB_PORT` | `3307` | Database port (3307 is the MariaDB 10 default on Synology NAS; standard MySQL uses 3306) |
| Target table | `myroom.sensor_data` | In the `INSERT` query in `send_to_db()` |
| `WIFI_PORTAL_SSID` | `ESP_SensorKit` | Name of the configuration access point |
| `WIFI_HOSTNAME` | `SensorStation` | Hostname the device uses on your network (shown in the router's client list) |

**Timing and recovery**

| Setting | Default | Description |
|---|---|---|
| `SEND_EVERY_N_SAMPLES` | `20` | Samples per database insert. With the 3 s BSEC LP rate this is about 60 s. |
| `WIFI_CONNECT_TIMEOUT_S` / `WIFI_CONNECT_RETRIES` | `30` / `3` | How long the saved network is tried at boot before the portal opens |
| `CONFIG_PORTAL_TIMEOUT_S` | `180` | Portal closes after this long and the board reboots (saved credentials are kept) |
| `WIFI_RECONNECT_INTERVAL_MS` | 30 s | How often `WiFi.reconnect()` is called while offline |
| `DB_RECONNECT_INTERVAL_MS` | 30 s | Minimum time between DB connection attempts |
| `NO_DATA_RESTART_MS` | 15 min | Reboot if no row has been stored for this long |
| `STATE_SAVE_PERIOD_MS` | 6 h | Re-save the calibrated BSEC state this often |
| `WDT_TIMEOUT_S` | `120` | Hardware watchdog: reboot if `loop()` stops running for this long |

---

## Database setup

Ready-made setup scripts are in [db/](db/):

| Script | For | Status |
|---|---|---|
| [db/mysql/setup.sql](db/mysql/setup.sql) | MySQL / MariaDB | **Used by the current firmware** |
| [db/postgresql/setup.sql](db/postgresql/setup.sql) | PostgreSQL | Schema only. The firmware can't talk to PostgreSQL yet (see below). |

### MySQL / MariaDB (current firmware)

```bash
mysql -h <nas-ip> -P 3307 -u root -p < db/mysql/setup.sql
```

The script creates the `myroom` database, the `myroom.sensor_data` table (with an auto `id` and a server-side
`created_at` timestamp) and an insert-only user `sensor@192.168.0.%`. Change the password in the script and put the
same credentials in `inc/config.hpp`. The server must accept **TCP connections from the LAN** (not just `localhost`),
and the user must use `mysql_native_password`, which is the only authentication MySQL_MariaDB_Generic supports.

### PostgreSQL

```bash
psql -h <host> -U postgres -v writer_pw='...' -v reader_pw='...' -f db/postgresql/setup.sql
```

The script is idempotent (safe to re-run) and must be run with `psql`. It creates:

- database `sensorstation` with schema `myroom` and table `myroom.sensor_data`, using the same column names as the
  MySQL version, so the firmware's `INSERT` statement works unchanged,
- range checks on humidity, IAQ and accuracy, and an index on `created_at`,
- the views `myroom.latest_reading` and `myroom.hourly_summary`,
- the roles `sensor_writer` (INSERT only, for the device) and `sensor_reader` (SELECT only, e.g. for Grafana).

> ⚠️ The firmware currently speaks only the **MySQL protocol**. To log into PostgreSQL, the database code in
> `main.cpp` has to be ported to a PostgreSQL client library. Embedded clients often support only `md5` or
> cleartext passwords, while PostgreSQL 14+ defaults to `scram-sha-256`, so check the library before choosing it.

---

## Build and flash

Using the PlatformIO CLI (or the matching buttons in the VS Code PlatformIO toolbar):

```bash
# Build
pio run

# Build and upload (921600 baud)
pio run -t upload

# Serial monitor (115200 baud; set DEBUG_MODE to true first)
pio device monitor
```

The only environment is `lolin32`.

---

## First boot and Wi-Fi setup

1. Power the board. All four IAQ LEDs light up and the blue LED blinks while the ESP32 tries the saved network
   (3 attempts × 30 s).
2. If that fails, or no network has been saved yet, the blue LED stays on and the device starts an open access point
   named **`ESP_SensorKit`**.
3. Connect to it from a phone or laptop. The captive portal should open on its own; if it doesn't, go to
   **http://192.168.4.1**.
4. Choose your Wi-Fi network, enter the password and save. The ESP32 stores the credentials and connects.
5. If nobody finishes the portal within **3 minutes**, the board reboots and tries the saved network again. The
   credentials are **never erased**, so after a power cut the device keeps cycling (about 4.5 min per round) until the
   router is back, then reconnects on its own.

To switch networks later, make the saved network unavailable (the portal opens on the next boot), or erase the flash
with `pio run -t erase` and re-flash.

---

## How it works

```
setup()
 ├─ configure LED pins, all IAQ LEDs on
 ├─ WiFiManager.autoConnect("ESP_SensorKit")
 │    ├─ try saved network: 3 × 30 s (blue LED blinking)
 │    └─ otherwise open portal (blue solid) for 180 s → reboot on timeout
 ├─ initialize_sensor()
 │    ├─ Wire.begin(SDA=15, SCL=0), Bsec.begin(0x76)
 │    ├─ loadState() → restore BSEC calibration from flash
 │    └─ subscribe to IAQ, static IAQ, CO₂-eq, bVOC-eq, raw T/H/gas,
 │       heat-compensated T/H at BSEC_SAMPLE_RATE_LP (one sample every 3 s)
 └─ start the 120 s task watchdog

loop()
 ├─ feed watchdog
 ├─ handle_wifi(): on loss → blink blue, drop DB socket, WiFi.reconnect() every 30 s
 ├─ when iaqSensor.run() returns a new sample (≈ every 3 s):
 │    ├─ read temperature, humidity, IAQ, accuracy
 │    ├─ save BSEC state if accuracy improved (or every 6 h when calibrated)
 │    └─ every 20th sample (≈ 60 s):
 │         ├─ update the IAQ colour LED
 │         └─ send_to_db(): connect if needed (max. every 30 s), INSERT;
 │            on any failure close the socket and retry with the next sample
 └─ nothing stored for 15 min → save state, ESP.restart()
```

### Connection recovery

Earlier versions would forget the Wi-Fi network, or stop sending data after a while. These were the causes and the
fixes:

| Problem | Cause | Fix |
|---|---|---|
| Device "forgets" the Wi-Fi after a power cut | The ESP32 boots faster than the router. The saved network wasn't found, the WiFiManager portal opened, and it had **no timeout**, so the device sat in AP mode forever. (The 20 s boot wait loop also never called `WiFi.begin()`, so it couldn't connect.) | Saved network tried 3 × 30 s, portal timeout of 3 min, then reboot. Credentials are kept. |
| Stops sending data and never recovers | MySQL_MariaDB_Generic leaves the TCP socket open when the login or a query times out. `conn.connected()` then kept returning `true`, so the firmware never reconnected and every INSERT failed. | `db_disconnect()` closes the socket after every failed connect or query, and after every Wi-Fi drop. |
| Flaky reconnects | `WiFi.begin()` was called every 3 s while offline, interrupting the driver's own reconnect. Wi-Fi modem sleep also causes drops on many routers. | Driver auto-reconnect plus `WiFi.reconnect()` every 30 s; `WiFi.setSleep(false)`. |
| Any other stuck state (sensor, network stack, NAS) | — | Reboot after 15 min without a stored row, plus a 120 s hardware watchdog. BSEC state is saved first. |

### IAQ accuracy and calibration

BSEC calibrates itself over time. The `iaq_accuracy` value means:

| Value | Meaning |
|---|---|
| 0 | Stabilising / run-in. IAQ values are not meaningful yet |
| 1 | Low accuracy. Background history is uncertain |
| 2 | Medium accuracy. Calibrating |
| 3 | High accuracy. Calibrated |

A brand-new sensor usually needs a few hours to a couple of days of continuous operation, with exposure to both clean
and polluted air, before it reaches accuracy 3. The saved BSEC state, together with its accuracy, lets the device
keep that calibration across reboots.

Flash layout (emulated EEPROM, 157 bytes): byte 0 is the magic `0x77`, bytes 1–155 hold the BSEC state, and byte 156
holds the accuracy of the stored state.

### Measured values

The BSEC subscription also requests static IAQ, CO₂ equivalent, breath-VOC equivalent and raw gas resistance, but only
**temperature, humidity, IAQ and IAQ accuracy** are stored. Pressure isn't logged. To log more values, read them
from the `Bsec` object (e.g. `iaqSensor.co2Equivalent`, `iaqSensor.breathVocEquivalent`, `iaqSensor.staticIaq`,
`iaqSensor.pressure`) and add matching columns to the table and the `INSERT` query.

---

## Project structure

```
.
├── platformio.ini            # PlatformIO environment (lolin32), build flags, library dependencies
├── src/
│   └── main.cpp              # The whole firmware
├── inc/
│   └── config.hpp            # DB credentials (git-ignored, create it yourself)
├── db/
│   ├── mysql/setup.sql       # MySQL/MariaDB schema + device user (used by the firmware)
│   └── postgresql/setup.sql  # PostgreSQL schema, views and roles
├── docs/
│   └── schematic.svg         # Wiring schematic
└── .vscode/                  # VS Code / PlatformIO IDE settings
```

---

## Troubleshooting

| Symptom | Possible cause / fix |
|---|---|
| Build fails with `config.hpp: No such file or directory` | Create `inc/config.hpp` (see [Configuration](#1-database-credentials-incconfighpp-required-not-in-git)). |
| Nothing on the serial monitor | `DEBUG_MODE` is `false` by default. Set it to `true` and re-flash. The first line printed is the reset reason, which helps tell a watchdog reboot from a power cut. |
| IAQ stays at 25 and accuracy stays at 0 | Normal during the first few minutes or hours (BSEC run-in). Leave it running. |
| No sensor data at all | Check the wiring and I²C address (`0x76` vs `0x77`). Make sure the BME680 runs on 3.3 V. |
| Board doesn't boot / hangs at reset | Something may be pulling GPIO 0 or GPIO 15 low (strapping pins). Disconnect the sensor and check, or move I²C to other pins. |
| Blue LED blinking | Wi-Fi lost. The device reconnects on its own and reboots after 15 min if that doesn't work. |
| Blue LED solid, `ESP_SensorKit` network visible | Saved network not reachable. Wait for the next reboot cycle (3 min), or set up a new network in the portal. |
| Board reboots about every 15 min | No row is reaching the database: check the server IP/port, that the DB accepts remote connections, the user's host/privileges, and that the `myroom.sensor_data` table exists. |
| Captive portal doesn't show up | Connect to `ESP_SensorKit` and browse to `http://192.168.4.1` manually. |
| Onboard LED lights up | Expected: it's on GPIO 5 (active-LOW) and is lit whenever the green LED is off. |

---

## Known limitations / notes

- **Hard-coded network settings**: the server IP, port and database/table names are compiled into the firmware.
- **MySQL only**: the firmware uses the MySQL protocol. The PostgreSQL script is ready, but the firmware's database
  code has to be ported before it can use it.
- **No clock on the device**: there is no RTC/NTP, so timestamps come from the database (`created_at`).
- **Blocking DB connect**: a connection attempt to an unreachable server can block `loop()` for up to about 50 s
  (library retries). BSEC then reports a timing warning for that cycle, which is harmless. The watchdog timeout
  (120 s) allows for this.
- **Flash wear**: the BSEC state (157 bytes) is written when accuracy improves, every 6 h, and before self-restarts.
  NVS wear-levels these writes, so wear is negligible even if the NAS is offline for days.
- **Bosch standalone BSEC package**: `inc/bsec_*.h` and `obj/` from an earlier linking attempt are git-ignored and not
  used by the build. The BSEC Arduino library from `lib_deps` brings its own headers and precompiled library.

---

## License

The project code is the author's own work. No license has been chosen yet.

The **Bosch BSEC** library (downloaded by PlatformIO) is © Bosch Sensortec GmbH and covered by the
[Bosch BSEC license](https://www.bosch-sensortec.com/software-tools/software/bsec/). Its terms restrict
redistribution, which is why none of its files are committed to this repository.
