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

- **BSEC-based IAQ**: static IAQ index with accuracy (0–3), plus heat-compensated temperature and humidity
  with a configurable temperature offset for self-heating.
- **Persistent calibration**: the BSEC calibration state is saved to flash when the IAQ accuracy improves, every 6 h
  once fully calibrated, and before any self-restart. It is restored on boot.
- **Status LEDs**: one LED shows Wi-Fi status and four LEDs show the air-quality level.
- **Captive-portal Wi-Fi setup** through [WiFiManager](https://github.com/tzapu/WiFiManager). It opens
  automatically when the saved network can't be reached, or on demand by **pressing RST twice within 3 seconds**.
  It closes after 3 minutes so the device never gets stuck in setup mode.
- **Self-healing connectivity**: Wi-Fi reconnect with periodic retries, the database socket closed on every failure,
  a reboot when nothing has been stored for 15 minutes, and a hardware task watchdog for hangs.
- **Database logging**: rows are inserted into a MySQL/MariaDB table through
  [MySQL_MariaDB_Generic](https://github.com/khoih-prog/MySQL_MariaDB_Generic). Server address (IP or host name), port,
  database, table, user and password are all set in the Wi-Fi portal, no re-flashing needed. A PostgreSQL schema is also included
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
| Green, white, yellow and red **on**, blue off (first 3 s) | Double-reset window: press RST again now to open the configuration portal |
| Green, white, yellow and red **on**, blue **blinking** | Connecting to the saved Wi-Fi (up to 3 × 30 s) |
| Blue **solid on**, all others off | The **configuration portal** is open: either RST was pressed twice, or the saved Wi-Fi isn't reachable (or none is saved) |
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

### 1. Local secrets: `inc/config.hpp` (optional, not in git)

`inc/config.hpp` is **ignored by git** (`**/config.hpp` in `.gitignore`). It holds the **portal password** and the
**default** database credentials, which are used until others are saved in the portal:

```cpp
// inc/config.hpp
#pragma once

#define PORTAL_PASSWORD "at-least-8-characters"   // WPA2 password of the ESP_SensorKit setup network

char db_user[]     = "your_db_user";               // defaults; can be changed in the portal
char db_password[] = "your_db_password";
```

- **`PORTAL_PASSWORD` is strongly recommended.** Without it, `ESP_SensorKit` is an **open** network, so anyone nearby
  could watch the database password being typed in, or use the portal while it's open (including its firmware-update
  page). The build prints a warning if it's missing. WPA2 requires 8–63 characters, and the build fails otherwise.
- Without `config.hpp` the firmware still builds. The default credentials are then empty, and you set them in the
  portal.

### 2. Settings in `src/main.cpp`

**Hardware and network**

| Setting | Default | Description |
|---|---|---|
| `DEBUG_MODE` | `false` | Set to `true` for Serial output (115200 baud). When `false`, the serial port is never started. |
| `BME680_I2C_ADDR` | `0x76` | Sensor I²C address (`0x76` / `0x77`) |
| `SDA_PIN` / `SCL_PIN` | `15` / `0` | I²C pins |
| `*_LED_PIN` | see [Wiring](#leds) | LED pins |
| `DEFAULT_DB_HOST` | `192.168.0.151` | Default database server (see [Database settings](#3-database-settings-in-the-portal)) |
| `DEFAULT_DB_PORT` | `3307` | Default database port (3307 is the MariaDB 10 default on Synology NAS; standard MySQL uses 3306) |
| `DEFAULT_DB_NAME` / `DEFAULT_DB_TABLE` | `myroom` / `sensor_data` | Default target table |
| `WIFI_PORTAL_SSID` | `ESP_SensorKit` | Name of the configuration access point |
| `WIFI_HOSTNAME` | `SensorStation` | Hostname the device uses on your network (shown in the router's client list) |

**Timing and recovery**

| Setting | Default | Description |
|---|---|---|
| `SEND_EVERY_N_SAMPLES` | `20` | Samples per database insert. With the 3 s BSEC LP rate this is about 60 s. |
| `WIFI_CONNECT_TIMEOUT_S` / `WIFI_CONNECT_RETRIES` | `30` / `3` | How long the saved network is tried at boot before the portal opens |
| `CONFIG_PORTAL_TIMEOUT_S` | `180` | Portal closes after this long without page activity, unless a device is connected to it (saved credentials are kept) |
| `DRD_WINDOW_MS` | 3000 | Double-reset window: a second RST press within this time after boot opens the portal. Also adds this much to every boot. |
| `WIFI_RECONNECT_INTERVAL_MS` | 30 s | How often `WiFi.reconnect()` is called while offline |
| `DB_RECONNECT_INTERVAL_MS` | 30 s | Minimum time between DB connection attempts |
| `NO_DATA_RESTART_MS` | 15 min | Reboot if no row has been stored for this long |
| `STATE_SAVE_PERIOD_MS` | 6 h | Re-save the calibrated BSEC state this often |
| `DEFAULT_TEMP_OFFSET` | `0.0` | Temperature offset until one is set in the portal |
| `WDT_TIMEOUT_S` | `120` | Hardware watchdog: reboot if `loop()` stops running for this long |

### 3. Database settings (in the portal)

Everything about where and how readings are stored can be changed **without re-flashing**:

1. Open the portal: press **RST twice within 3 s** (see [Changing the Wi-Fi network later](#changing-the-wi-fi-network-later)).
2. Connect to `ESP_SensorKit` (using `PORTAL_PASSWORD`) and tap **Setup**.
3. Fill in the fields and tap **Save**:

   | Field | Example | Rules |
   |---|---|---|
   | Database server | `192.168.0.151` or `db.example.com` | IP address or host name: letters, digits, `.`, `-` |
   | Database port | `3307` | 1–65535 |
   | Database name | `myroom` | Letters, digits, `_` |
   | Table name | `sensor_data` | Letters, digits, `_` |
   | Database user | `sensor` | Required, max. 32 characters |
   | Database password | | **Leave empty to keep the current password.** Max. 64 characters. |
   | Temperature offset | `2.3` | °C, −10 to +10. See [Temperature offset](#temperature-offset-calibration). |

4. Tap **Exit**. The device connects to the saved Wi-Fi and uses the new settings.

- All values are stored in flash (namespace `dbcfg`), so they survive reboots and firmware updates.
  `.\flash.ps1 -Erase` resets them to the defaults.
- **Everything is validated first.** If any field is invalid, nothing is changed and the stored values stay. The portal
  still says "Setup saved", so open **Setup** again to check what's stored.
- **The password field is never pre-filled**, so the stored password doesn't appear in the page. Because empty means
  "keep the current one", an empty database password can't be set from the portal.
- The database and table names are inserted into the SQL statement, which is why only letters, digits and `_` are
  allowed. The table must have the columns `temperature`, `humidity`, `iaq` and `iaq_accuracy` (see
  [db/](db/)).
- Wi-Fi and database settings are on separate pages, so changing one doesn't require re-entering the other.

---

## Database setup

Ready-made setup scripts are in [db/](db/):

| Script | For | Status |
|---|---|---|
| [db/mysql/setup.sql](db/mysql/setup.sql) | MySQL / MariaDB | **Used by the current firmware**. New installation. |
| [db/mysql/migrate_v2.sql](db/mysql/migrate_v2.sql) | MySQL / MariaDB | Upgrade an existing table to the current layout |
| [db/postgresql/setup.sql](db/postgresql/setup.sql) | PostgreSQL | Schema only. The firmware can't talk to PostgreSQL yet (see below). |

### Table layout

| Column | Type (MySQL / PostgreSQL) | Content |
|---|---|---|
| `timestamp` | `TIMESTAMP` / `TIMESTAMPTZ`, **primary key** | Measurement time, set by the database server (the device has no clock) |
| `temperature` | `FLOAT` / `REAL` | °C, heat-compensated, temperature offset applied |
| `humidity` | `FLOAT` / `REAL` | % relative humidity |
| `iaq` | `FLOAT` / `REAL` | BSEC static IAQ (unscaled, so it can exceed 500) |
| `iaq_accuracy` | `TINYINT UNSIGNED` / `SMALLINT` | 0–3 |

The timestamp is the primary key, and there's no separate `id` column:

- **Less storage:** this saves the `id` column and a separate index on `timestamp`.
- **Faster time-range queries:** InnoDB stores rows ordered by the primary key.
- **Uniqueness:** the device writes about one row per minute, so timestamps are unique.
- **Self-healing on collision:** if two inserts ever land in the same second, the second one is rejected and simply
  retried with the next sample.
- **More devices later:** a single timestamp key won't work for several stations writing to one table. You would then
  need a `device` column and a combined key `(device, timestamp)`.
- **Quoting:** `timestamp` is also a type name, so the scripts quote it (`` `timestamp` `` in MySQL, `"timestamp"` in
  PostgreSQL). Your own queries work without quotes in both, but quoting is the safe habit.
- **MariaDB 2038 limit:** MariaDB before 11.5 stores `TIMESTAMP` only up to January 2038. Switching to `DATETIME` is
  a simple change before then.

### MySQL / MariaDB (current firmware)

**New installation:**

```bash
mysql -h <nas-ip> -P 3307 -u root -p < db/mysql/setup.sql
```

The script creates the `myroom` database, the `myroom.sensor_data` table and an insert-only user
`sensor@192.168.0.%`. Change the password in the script and put the same credentials in the portal's **Setup** page (or
as defaults in `inc/config.hpp`). The server must accept **TCP connections from the LAN** (not just `localhost`), and
the user must use `mysql_native_password`, which is the only authentication MySQL_MariaDB_Generic supports.

**Existing table** (older layout with `id`, `pressure` and `gas` columns): use
[db/mysql/migrate_v2.sql](db/mysql/migrate_v2.sql). **Back up first**, then run it **step by step**. It checks for rows
without a timestamp and for duplicate timestamps, which both have to be resolved before `timestamp` can become the
primary key. Then it removes `id`, `pressure` and `gas` and rebuilds the table to free the space. It's written for
MariaDB (Synology); on MySQL, remove the `IF EXISTS` parts.

### PostgreSQL

```bash
psql -h <host> -U postgres -v writer_pw='...' -v reader_pw='...' -f db/postgresql/setup.sql
```

The script is idempotent (safe to re-run) and must be run with `psql`. It creates:

- database `sensorstation` with schema `myroom` and table `myroom.sensor_data`, using the same column names as the
  MySQL version, so the firmware's `INSERT` statement works unchanged,
- `timestamp` as the primary key, and range checks on humidity, IAQ (≥ 0) and accuracy,
- the views `myroom.latest_reading` and `myroom.hourly_summary`,
- the roles `sensor_writer` (INSERT only, for the device) and `sensor_reader` (SELECT only, e.g. for Grafana).

> ⚠️ The firmware currently speaks only the **MySQL protocol**. To log into PostgreSQL, the database code in
> `main.cpp` has to be ported to a PostgreSQL client library. Embedded clients often support only `md5` or
> cleartext passwords, while PostgreSQL 14+ defaults to `scram-sha-256`, so check the library before choosing it.

---

## Build and flash

### Windows: `flash.ps1` (recommended)

The script builds the firmware, **finds the board's COM port automatically** and uploads:

```powershell
.\flash.ps1              # build + flash, auto-detect the port
.\flash.ps1 -Monitor     # ...and open the serial monitor afterwards
.\flash.ps1 -Port COM5   # use a specific port
.\flash.ps1 -Erase       # wipe the flash first (forgets Wi-Fi + BSEC calibration)
.\flash.ps1 -List        # only show the detected ports, don't flash
```

Port detection looks for the USB-serial chips used on ESP32 boards (CH340/CH9102, CP210x, FTDI, Espressif native
USB) and ignores other COM ports, such as the motherboard's COM1. If more than one board is plugged in, it asks which
one to use. It warns if `inc/config.hpp` or its `PORTAL_PASSWORD` is missing, and finds `pio` even when it isn't on
`PATH`.

If PowerShell refuses to run the script (execution policy), use
`powershell -ExecutionPolicy Bypass -File .\flash.ps1`.

### PlatformIO CLI

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

1. Power the board. All four IAQ LEDs light up. After 3 s the blue LED starts blinking while the ESP32 tries the
   saved network (3 attempts × 30 s).
2. If that fails, or no network has been saved yet, the blue LED stays on and the device starts an access point
   named **`ESP_SensorKit`**. Its password is `PORTAL_PASSWORD` from `inc/config.hpp`; without one, it's an open
   network.
3. Connect to it from a phone or laptop. The captive portal should open on its own; if it doesn't, go to
   **http://192.168.4.1**.
4. Choose your Wi-Fi network, enter the password and save. The ESP32 stores the credentials and connects.
5. The portal stays open while your phone is connected to `ESP_SensorKit`. Once nothing is connected and no page has
   been loaded for **3 minutes**, the board reboots and tries the saved network again. The credentials are **never
   erased**, so after a power cut the device keeps retrying until the router is back, then reconnects on its own.

### Changing the Wi-Fi network later

**Press the RST button twice, within 3 seconds** (the second press while the four IAQ LEDs are lit after the first
one). The blue LED turns solid and the `ESP_SensorKit` portal opens. Then continue from step 3 above.

- The old credentials stay saved until you save new ones. If you close the portal without saving, the device goes
  back to the old network after 3 minutes.
- The BSEC calibration is **not** affected.
- If you changed your router's name or password, you don't need to do anything: the saved network fails and the
  portal opens by itself after about 1.5 minutes.
- `.\flash.ps1 -Erase` also resets Wi-Fi, but it **deletes the BSEC calibration** too, because both are stored in the
  same flash area.

The double reset is detected with a flag in flash, not in RTC memory, because on the ESP32 the RST (EN) button cuts
power to the whole chip. The BOOT button can't be used for this: on this board it's wired to GPIO 0, the sensor's SCL
line, and holding it during reset puts the ESP32 into flashing mode.

---

## How it works

```
setup()
 ├─ configure LED pins, all IAQ LEDs on
 ├─ load_db_config(): database host/port from flash (defaults 192.168.0.151:3307)
 ├─ check_double_reset(): flag in flash set → this is the 2nd reset → open portal on demand;
 │    otherwise set the flag, wait 3 s, clear it
 ├─ (on demand) WiFiManager.startConfigPortal() → on timeout continue with the saved network
 ├─ WiFiManager.autoConnect("ESP_SensorKit")
 │    ├─ try saved network: 3 × 30 s (blue LED blinking)
 │    └─ otherwise open portal (blue solid) for 180 s → reboot on timeout
 ├─ initialize_sensor()
 │    ├─ Wire.begin(SDA=15, SCL=0), Bsec.begin(0x76)
 │    ├─ setConfig(generic_33v_3s_4d), setTemperatureOffset(<from flash>)
 │    ├─ loadState() → restore BSEC calibration from flash
 │    └─ subscribe to IAQ, static IAQ, CO₂-eq, bVOC-eq, raw T/H/gas,
 │       heat-compensated T/H at BSEC_SAMPLE_RATE_LP (one sample every 3 s)
 └─ start the 120 s task watchdog

loop()
 ├─ feed watchdog
 ├─ handle_wifi(): on loss → blink blue, drop DB socket, WiFi.reconnect() every 30 s
 ├─ when iaqSensor.run() returns a new sample (≈ every 3 s):
 │    ├─ read temperature, humidity, static IAQ + accuracy
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
| No way to switch networks while the old one is still up | The portal only opened when the saved network failed. | Press RST twice within 3 s to open the portal on demand. |
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

A saved state only fits the **BSEC configuration and library version** it was created with. BSEC may still accept a
mismatched state, and then produces nonsense, such as a static IAQ far above 500 at accuracy 2. So each save also
stores a **fingerprint** (FNV-1a hash of the configuration and the library version). At boot, a state with a different
fingerprint is discarded and calibration starts over. This happens automatically after a BSEC library update or a
configuration change.

Flash layout (emulated EEPROM, 161 bytes):

| Bytes | Content |
|---|---|
| 0 | Magic `0x78` (`0x77` is the older layout without a fingerprint; it's ignored) |
| 1–155 | BSEC state |
| 156 | Accuracy of the stored state |
| 157–160 | Fingerprint |

### Measured values

| Stored column | Source | Notes |
|---|---|---|
| `temperature` | BSEC heat-compensated temperature | Corrected by the **temperature offset** (see below) |
| `humidity` | BSEC heat-compensated relative humidity | Recalculated by BSEC at the corrected temperature |
| `iaq` | BSEC **static IAQ** (`iaqSensor.staticIaq`) | Bosch's recommended output for stationary devices |
| `iaq_accuracy` | Accuracy of the static IAQ (`staticIaqAccuracy`) | 0–3, see above |

**BSEC configuration.** The firmware loads `generic_33v_3s_4d` from the BSEC library: 3.3 V supply, 3 s sample
interval (LP mode, matching `BSEC_SAMPLE_RATE_LP`) and a 4-day calibration history. The configuration is loaded before
the saved state is restored, because a state only fits the configuration it was created with. If you change it
(e.g. to `generic_33v_3s_28d` for a steadier but slower-adapting IAQ), BSEC rejects the old state and recalibrates.

**Static IAQ vs. IAQ.** The plain IAQ output keeps rescaling itself to recent conditions, which suits a device carried
around. For a monitor that stays in one room, a room that stays stuffy for days would slowly drift back toward
"good". Static IAQ doesn't drift like that. Both are calculated; the IAQ LEDs and the database use static IAQ.

> ⚠️ Rows stored before this change hold the **plain** IAQ in the `iaq` column. Keep that in mind when comparing old
> and new data.

The BSEC subscription also provides the plain IAQ, CO₂ equivalent, breath-VOC equivalent and raw gas resistance, which
aren't stored. Pressure isn't logged either. To log more values, read them from the `Bsec` object (e.g.
`iaqSensor.co2Equivalent`, `iaqSensor.breathVocEquivalent`, `iaqSensor.iaq`, `iaqSensor.pressure`) and add matching
columns to the table and the `INSERT` query.

### Temperature offset (calibration)

The BME680 is warmed by the ESP32 next to it and by its own electronics, so it reads **too warm**, typically by about
1–4 °C. Because relative humidity depends on temperature, it then also reads **too low**. BSEC corrects both if it
knows the offset:

1. Run the station in its final place, with its final firmware, for **at least 1 hour** so it reaches a steady
   temperature.
2. Put a trusted thermometer right next to it and compare the readings once both are stable.
3. **Offset = station reading − real temperature.** For example, the station shows 24.6 °C and the thermometer
   shows 22.3 °C, so the offset is `2.3`.
4. Open the portal (RST twice), go to **Setup**, enter the offset in **Temperature offset** and save. Allowed range:
   −10 to +10 °C.

The offset is stored in flash (namespace `sensor`) and applied from the next sensor start, which is right after the
portal closes. Humidity corrects itself. Keeping the sensor 5–10 cm away from the ESP32 (on a short cable) makes the
offset smaller and more stable. Re-check the offset if you move the station or change its enclosure.

---

## Project structure

```
.
├── platformio.ini            # PlatformIO environment (lolin32), build flags, library dependencies
├── flash.ps1                 # Build + flash with automatic COM port detection (Windows)
├── src/
│   └── main.cpp              # The whole firmware
├── inc/
│   └── config.hpp            # Portal password + default DB credentials (optional, git-ignored)
├── db/
│   ├── mysql/setup.sql       # MySQL/MariaDB schema + device user (used by the firmware)
│   ├── mysql/migrate_v2.sql  # Upgrade an existing table (drop id/pressure/gas, timestamp as key)
│   └── postgresql/setup.sql  # PostgreSQL schema, views and roles
├── docs/
│   └── schematic.svg         # Wiring schematic
└── .vscode/                  # VS Code / PlatformIO IDE settings
```

---

## Troubleshooting

| Symptom | Possible cause / fix |
|---|---|
| Build warning `PORTAL_PASSWORD not defined` | Add `#define PORTAL_PASSWORD "..."` to `inc/config.hpp` (see [Configuration](#1-local-secrets-incconfighpp-optional-not-in-git)). |
| Can't connect to `ESP_SensorKit` | Since `PORTAL_PASSWORD` was added, the network asks for that password. |
| Nothing on the serial monitor | `DEBUG_MODE` is `false` by default. Set it to `true` and re-flash. The first line printed is the reset reason, which helps tell a watchdog reboot from a power cut. |
| Static IAQ far above 500 although accuracy is 2–3 | The restored calibration doesn't fit the current BSEC configuration. Firmware from this version on detects this and recalibrates automatically. If it still happens after a fresh calibration, check the raw gas resistance with `DEBUG_MODE` (typically tens to hundreds of kΩ; very low values mean solvents or alcohol near the sensor). |
| IAQ stays at 25 and accuracy stays at 0 | Normal during the first few minutes or hours (BSEC run-in). Leave it running. |
| No sensor data at all | Check the wiring and I²C address (`0x76` vs `0x77`). Make sure the BME680 runs on 3.3 V. |
| Board doesn't boot / hangs at reset | Something may be pulling GPIO 0 or GPIO 15 low (strapping pins). Disconnect the sensor and check, or move I²C to other pins. |
| Blue LED blinking | Wi-Fi lost. The device reconnects on its own and reboots after 15 min if that doesn't work. |
| Blue LED solid, `ESP_SensorKit` network visible | Portal is open: RST was pressed twice, or the saved network isn't reachable. Set up a network, or wait 3 min. |
| Portal opens right after flashing or opening the serial monitor | The monitor reset the board within 3 s of the post-upload reset. `platformio.ini` sets `monitor_rts = 0` / `monitor_dtr = 0` to prevent this; check that your monitor tool doesn't toggle DTR/RTS. |
| Portal opened after a quick power flicker | Expected: a power cut within 3 s of booting counts as a double reset. The portal closes after 3 min and the device reconnects. |
| Board reboots about every 15 min | No row is reaching the database: check the settings in the portal's **Setup** page (a typo in the host name, user, password or table name fails silently), that the DB accepts remote connections, the user's host/privileges, and that the table exists. |
| Captive portal doesn't show up | Connect to `ESP_SensorKit` and browse to `http://192.168.4.1` manually. |
| Onboard LED lights up | Expected: it's on GPIO 5 (active-LOW) and is lit whenever the green LED is off. |

---

## Known limitations / notes

- **Credentials stored unencrypted in flash**: the database password is kept in NVS in plain text, so anyone with
  physical access and a USB cable can read it out. Give the device's DB user only `INSERT` rights on its table, as
  the setup scripts in [db/](db/) do.
- **Plain MySQL over TCP, no TLS**: fine on a home LAN. A database in the cloud (e.g. OpenShift) needs an encrypted
  path, either a TLS-capable client or an HTTPS ingest service in front of the database.
- **MySQL only**: the firmware uses the MySQL protocol. The PostgreSQL script is ready, but the firmware's database
  code has to be ported before it can use it.
- **No clock on the device**: there is no RTC/NTP, so timestamps come from the database (`timestamp` column).
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
