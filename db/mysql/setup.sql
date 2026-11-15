-- =====================================================================
--  SensorStation – MySQL / MariaDB setup (used by the current firmware)
-- =====================================================================
--  Run as an admin user, e.g.:
--
--      mysql -h <host> -P 3307 -u root -p < db/mysql/setup.sql
--
--  Change the password below (and in the portal's Setup page) before running.
--  The script is idempotent: re-running it does not drop data.
--  Upgrading an existing table instead? See migrate_v2.sql.
-- =====================================================================

CREATE DATABASE IF NOT EXISTS myroom CHARACTER SET utf8mb4 COLLATE utf8mb4_general_ci;

-- The measurement time is the primary key: one device writes about one row per minute, so it is unique,
-- and InnoDB stores rows ordered by it, which makes time-range queries fast without an extra index.
-- Note: TIMESTAMP is stored in UTC with 1 s resolution and (before MariaDB 11.5) ends in January 2038.
CREATE TABLE IF NOT EXISTS myroom.sensor_data (
    `timestamp`   TIMESTAMP        NOT NULL DEFAULT CURRENT_TIMESTAMP,  -- the device has no clock
    temperature   FLOAT            NOT NULL,   -- °C, heat-compensated, temperature offset applied
    humidity      FLOAT            NOT NULL,   -- % relative humidity, heat-compensated
    iaq           FLOAT            NOT NULL,   -- BSEC static IAQ index
    iaq_accuracy  TINYINT UNSIGNED NOT NULL,   -- 0 = stabilising .. 3 = calibrated
    PRIMARY KEY (`timestamp`)
) ENGINE = InnoDB;

-- Device account: insert-only, limited to the local network.
-- mysql_native_password is required by the MySQL_MariaDB_Generic library.
-- (MariaDB syntax; on MySQL 8 use: IDENTIFIED WITH mysql_native_password BY '...')
CREATE USER IF NOT EXISTS 'sensor'@'192.168.0.%' IDENTIFIED BY 'change-me';
GRANT INSERT ON myroom.sensor_data TO 'sensor'@'192.168.0.%';
FLUSH PRIVILEGES;
