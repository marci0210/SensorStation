-- =====================================================================
--  SensorStation – migrate an existing MySQL / MariaDB table to the v2 layout
-- =====================================================================
--  v2 layout: `timestamp` is the primary key; the id, pressure and gas columns are removed.
--
--  !! BACK UP FIRST !!   mysqldump -h <host> -P 3307 -u root -p myroom sensor_data > sensor_data_backup.sql
--
--  Written for MariaDB (Synology "MariaDB 10"): uses IF EXISTS, which MySQL does not support
--  for DROP COLUMN / DROP INDEX. On MySQL, remove the IF EXISTS parts for the columns you have.
--
--  Check your current layout before running:   SHOW CREATE TABLE myroom.sensor_data;
--  The table must have a `timestamp` column. If it does not, stop here: old rows have no time
--  that could become the key. If there is another index on `timestamp` besides the one dropped
--  in step 3, drop it as well (the primary key replaces it).
--
--  Run it step by step (e.g. in phpMyAdmin or the mysql client), not blindly in one go.
-- =====================================================================

-- ---------------------------------------------------------------------
-- 1. Rows without a timestamp cannot be kept (a primary key cannot be NULL). Count them first:
-- ---------------------------------------------------------------------
SELECT COUNT(*) AS rows_without_timestamp FROM myroom.sensor_data WHERE `timestamp` IS NULL;
-- If not 0 and you agree to drop them:
-- DELETE FROM myroom.sensor_data WHERE `timestamp` IS NULL;

-- ---------------------------------------------------------------------
-- 2. Duplicate timestamps would block the new primary key. List them:
-- ---------------------------------------------------------------------
SELECT `timestamp`, COUNT(*) AS n
FROM myroom.sensor_data
GROUP BY `timestamp`
HAVING COUNT(*) > 1;

-- If any are listed, keep the first row of each second (lowest id) and delete the rest:
-- DELETE t1 FROM myroom.sensor_data t1
-- JOIN myroom.sensor_data t2 ON t1.`timestamp` = t2.`timestamp` AND t1.id > t2.id;

-- ---------------------------------------------------------------------
-- 3. Restructure: drop id (and with it the old primary key), pressure, gas and the now
--    redundant timestamp index, then make `timestamp` the primary key.
--    MODIFY also removes a possible "ON UPDATE CURRENT_TIMESTAMP" (older MariaDB adds it
--    automatically to the first TIMESTAMP column), which must not touch a primary key.
-- ---------------------------------------------------------------------
ALTER TABLE myroom.sensor_data
    DROP INDEX  IF EXISTS sensor_data_timestamp_idx,
    DROP COLUMN IF EXISTS id,
    DROP COLUMN IF EXISTS pressure,
    DROP COLUMN IF EXISTS gas,
    MODIFY `timestamp` TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    ADD PRIMARY KEY (`timestamp`);

-- ---------------------------------------------------------------------
-- 4. Reclaim the freed space (rebuilds the table) and check the result
-- ---------------------------------------------------------------------
OPTIMIZE TABLE myroom.sensor_data;
SHOW CREATE TABLE myroom.sensor_data;
