//! Used to log in the db
//! We probably need to revisit once we are more mature
//! TODO: we probably want to autogenerate this at one point
mod reservoir;
mod schema;
pub mod simulation;

use diesel::Connection as ConnectionTrait;
use diesel::SqliteConnection;
pub use reservoir::{DBRcData, NewDBRcData};
use rusqlite::Connection;
use std::sync::Mutex;

const ENSURE_FLIGHT_LOGS_QUERY: &str = "
    CREATE TABLE IF NOT EXISTS flight_log (
        id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
        simulation_id TEXT NOT NULL,
        start_seconds REAL NOT NULL,
        end_seconds REAL NOT NULL,
        motor_input_1 REAL NOT NULL,
        motor_input_2 REAL NOT NULL,
        motor_input_3 REAL NOT NULL,
        motor_input_4 REAL NOT NULL,
        battery_voltage_sag REAL NOT NULL,
        battery_voltage REAL NOT NULL,
        amperage REAL NOT NULL,
        mah_drawn REAL NOT NULL,
        cell_count INTEGER NOT NULL,
        rot_quat_x REAL NOT NULL,
        rot_quat_y REAL NOT NULL,
        rot_quat_z REAL NOT NULL,
        rot_quat_w REAL NOT NULL,
        linear_acceleration_x REAL NOT NULL,
        linear_acceleration_y REAL NOT NULL,
        linear_acceleration_z REAL NOT NULL,
        angular_velocity_x REAL NOT NULL,
        angular_velocity_y REAL NOT NULL,
        angular_velocity_z REAL NOT NULL,
        throttle REAL NOT NULL,
        roll REAL NOT NULL,
        pitch REAL NOT NULL,
        yaw REAL NOT NULL
    );

    CREATE TABLE IF NOT EXISTS rc_model (
        id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
        rc_id TEXT NOT NULL,
        n_internal_units INTEGER NOT NULL,
        input_scaling REAL NOT NULL,
        internal_weights TEXT NOT NULL,
        input_weights TEXT,
        alpha REAL NOT NULL,
        readout_coeff TEXT,
        readout_intercept TEXT
    );
";

pub struct AscentDb2 {
    // Diesel conn for type safty
    diesel_conn: Mutex<SqliteConnection>,
    // Rusqlte conn for speed
    rusqlite_conn: Mutex<Connection>,
}

impl AscentDb2 {
    pub fn new(db_file: &str) -> Self {
        let diesel_conn = SqliteConnection::establish(db_file).unwrap();
        let rusqlite_conn = Connection::open(db_file).unwrap();
        rusqlite_conn.execute(ENSURE_FLIGHT_LOGS_QUERY, []).unwrap();
        // TODO: ensure flight logs
        Self {
            diesel_conn: Mutex::new(diesel_conn),
            rusqlite_conn: Mutex::new(rusqlite_conn),
        }
    }
}
