-- =====================================================================
--  SensorStation – MySQL / MariaDB setup (used by the current firmware)
-- =====================================================================
--  Run as an admin user, e.g.:
--
--      mysql -h <host> -P 3307 -u root -p < db/mysql/setup.sql
--
--  Change the password below (and in inc/config.hpp) before running.
--  The script is idempotent: re-running it does not drop data.
-- =====================================================================

CREATE DATABASE IF NOT EXISTS myroom CHARACTER SET utf8mb4 COLLATE utf8mb4_general_ci;

CREATE TABLE IF NOT EXISTS myroom.sensor_data (
    id            INT UNSIGNED     NOT NULL AUTO_INCREMENT PRIMARY KEY,
    created_at    TIMESTAMP        NOT NULL DEFAULT CURRENT_TIMESTAMP,  -- the device has no clock
    temperature   FLOAT            NOT NULL,   -- °C, heat-compensated
    humidity      FLOAT            NOT NULL,   -- % relative humidity, heat-compensated
    iaq           FLOAT            NOT NULL,   -- BSEC IAQ index, 0..500
    iaq_accuracy  TINYINT UNSIGNED NOT NULL,   -- 0 = stabilising .. 3 = calibrated
    INDEX sensor_data_created_at_idx (created_at)
) ENGINE = InnoDB;

-- Device account: insert-only, limited to the local network.
-- mysql_native_password is required by the MySQL_MariaDB_Generic library.
-- (MariaDB syntax; on MySQL 8 use: IDENTIFIED WITH mysql_native_password BY '...')
CREATE USER IF NOT EXISTS 'sensor'@'192.168.0.%' IDENTIFIED BY 'change-me';
GRANT INSERT ON myroom.sensor_data TO 'sensor'@'192.168.0.%';
FLUSH PRIVILEGES;
