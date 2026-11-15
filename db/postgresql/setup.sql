-- =====================================================================
--  SensorStation – PostgreSQL setup
-- =====================================================================
--  Creates the database, the `myroom` schema, the `sensor_data` table and
--  two roles (a writer for the device, a read-only one for dashboards).
--
--  Run as a superuser with psql (it uses psql meta-commands, so it will not
--  work in a plain SQL GUI):
--
--      psql -h <host> -U postgres -v writer_pw='...' -v reader_pw='...' -f db/postgresql/setup.sql
--
--  The script is idempotent: re-running it does not drop data.
--
--  NOTE: the current firmware uses the MySQL wire protocol
--  (MySQL_MariaDB_Generic) and cannot talk to PostgreSQL. This schema is
--  laid out so that the same INSERT statement works unchanged once the
--  firmware uses a PostgreSQL client:
--      INSERT INTO myroom.sensor_data (temperature, humidity, iaq, iaq_accuracy) VALUES (...);
-- =====================================================================

\set ON_ERROR_STOP on

-- Default passwords if none were passed with -v (CHANGE THEM!)
\if :{?writer_pw}
\else
  \set writer_pw 'change-me-writer'
\endif
\if :{?reader_pw}
\else
  \set reader_pw 'change-me-reader'
\endif

-- ---------------------------------------------------------------------
-- 1. Roles
-- ---------------------------------------------------------------------
SELECT format('CREATE ROLE sensor_writer LOGIN PASSWORD %L', :'writer_pw')
WHERE NOT EXISTS (SELECT 1 FROM pg_roles WHERE rolname = 'sensor_writer')
\gexec

SELECT format('CREATE ROLE sensor_reader LOGIN PASSWORD %L', :'reader_pw')
WHERE NOT EXISTS (SELECT 1 FROM pg_roles WHERE rolname = 'sensor_reader')
\gexec

-- ---------------------------------------------------------------------
-- 2. Database
-- ---------------------------------------------------------------------
SELECT 'CREATE DATABASE sensorstation ENCODING ''UTF8'' TEMPLATE template0'
WHERE NOT EXISTS (SELECT 1 FROM pg_database WHERE datname = 'sensorstation')
\gexec

REVOKE ALL ON DATABASE sensorstation FROM PUBLIC;
GRANT CONNECT ON DATABASE sensorstation TO sensor_writer, sensor_reader;

\connect sensorstation

-- ---------------------------------------------------------------------
-- 3. Schema and table
-- ---------------------------------------------------------------------
CREATE SCHEMA IF NOT EXISTS myroom;

CREATE TABLE IF NOT EXISTS myroom.sensor_data (
    id            BIGINT      GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
    created_at    TIMESTAMPTZ NOT NULL DEFAULT now(),   -- the device has no clock; the server stamps each row
    temperature   REAL        NOT NULL,                 -- °C, heat-compensated
    humidity      REAL        NOT NULL,                 -- % relative humidity, heat-compensated
    iaq           REAL        NOT NULL,                 -- BSEC IAQ index
    iaq_accuracy  SMALLINT    NOT NULL,                 -- 0 = stabilising .. 3 = calibrated

    CONSTRAINT sensor_data_humidity_range CHECK (humidity BETWEEN 0 AND 100),
    CONSTRAINT sensor_data_iaq_range      CHECK (iaq BETWEEN 0 AND 500),
    CONSTRAINT sensor_data_accuracy_range CHECK (iaq_accuracy BETWEEN 0 AND 3)
);

COMMENT ON TABLE  myroom.sensor_data              IS 'BME680/BSEC readings from the SensorStation ESP32, about one row per minute';
COMMENT ON COLUMN myroom.sensor_data.iaq          IS 'BSEC IAQ: 0-50 excellent, 51-100 good, 101-150 lightly polluted, 151-200 moderately, 201-250 heavily, 251-350 severely, >350 extremely polluted';
COMMENT ON COLUMN myroom.sensor_data.iaq_accuracy IS '0 stabilising, 1 low, 2 medium, 3 high (calibrated)';

-- Time-range queries (dashboards, "last 24 h") are by far the most common
CREATE INDEX IF NOT EXISTS sensor_data_created_at_idx ON myroom.sensor_data (created_at);

-- ---------------------------------------------------------------------
-- 4. Convenience views
-- ---------------------------------------------------------------------
CREATE OR REPLACE VIEW myroom.latest_reading AS
SELECT *
FROM myroom.sensor_data
ORDER BY created_at DESC
LIMIT 1;

CREATE OR REPLACE VIEW myroom.hourly_summary AS
SELECT date_trunc('hour', created_at)   AS hour,
       count(*)                         AS samples,
       round(avg(temperature)::numeric, 2) AS avg_temperature,
       round(avg(humidity)::numeric, 2)    AS avg_humidity,
       round(avg(iaq)::numeric, 1)         AS avg_iaq,
       round(max(iaq)::numeric, 1)         AS max_iaq,
       min(iaq_accuracy)                AS min_accuracy
FROM myroom.sensor_data
GROUP BY 1;

-- ---------------------------------------------------------------------
-- 5. Privileges
-- ---------------------------------------------------------------------
-- Device: may only insert new rows (identity columns need no extra sequence grant)
GRANT USAGE  ON SCHEMA myroom             TO sensor_writer;
GRANT INSERT ON myroom.sensor_data        TO sensor_writer;

-- Dashboards (Grafana etc.): read-only
GRANT USAGE  ON SCHEMA myroom             TO sensor_reader;
GRANT SELECT ON ALL TABLES IN SCHEMA myroom TO sensor_reader;
ALTER DEFAULT PRIVILEGES IN SCHEMA myroom GRANT SELECT ON TABLES TO sensor_reader;

-- ---------------------------------------------------------------------
-- 6. Optional: data retention (uncomment and schedule, e.g. with cron/pg_cron)
-- ---------------------------------------------------------------------
-- DELETE FROM myroom.sensor_data WHERE created_at < now() - interval '2 years';

\echo 'SensorStation PostgreSQL setup complete.'
