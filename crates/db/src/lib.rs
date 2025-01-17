//! Used to log in the db
//! We probably need to revisit once we are more mature
//! TODO: we probably want to autogenerate this at one point
mod reservoir;
mod schema;
pub mod simulation;
pub mod simulation_frame;

use diesel::Connection as ConnectionTrait;
use diesel::SqliteConnection;
pub use reservoir::{DBRcData, NewDBRcData};
use rusqlite::Connection;
use std::sync::Arc;
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

    CREATE TABLE IF NOT EXISTS low_pass_filter (
        id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
        output DOUBLE NOT NULL,
        e_pow DOUBLE NOT NULL
    );

    CREATE TABLE IF NOT EXISTS rotor_state (
        id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
        current DOUBLE NOT NULL,
        rpm DOUBLE NOT NULL,
        motor_torque DOUBLE NOT NULL,
        effective_thrust DOUBLE NOT NULL,
        pwm DOUBLE NOT NULL,
        rotor_dir DOUBLE NOT NULL,
        motor_pos_x DOUBLE NOT NULL,
        motor_pos_y DOUBLE NOT NULL,
        motor_pos_z DOUBLE NOT NULL,
        pwm_low_pass_filter INTEGER NOT NULL references low_pass_filter (id)
    );

    CREATE TABLE IF NOT EXISTS simulation_frame (
        id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,

        capacity DOUBLE NOT NULL,
        bat_voltage DOUBLE NOT NULL,
        bat_voltage_sag DOUBLE NOT NULL,
        amperage DOUBLE NOT NULL,
        m_ah_drawn DOUBLE NOT NULL,
        rotor_1_state INTEGER NOT NULL references rotor_state (id),
        rotor_2_state INTEGER NOT NULL references rotor_state (id),
        rotor_3_state INTEGER NOT NULL references rotor_state (id),
        rotor_4_state INTEGER NOT NULL references rotor_state (id),

        position_x DOUBLE NOT NULL,
        position_y DOUBLE NOT NULL,
        position_z DOUBLE NOT NULL,

        rotation_x DOUBLE NOT NULL,
        rotation_y DOUBLE NOT NULL,
        rotation_z DOUBLE NOT NULL,
        rotation_w DOUBLE NOT NULL,

        linear_velocity_x DOUBLE NOT NULL,
        linear_velocity_y DOUBLE NOT NULL,
        linear_velocity_z DOUBLE NOT NULL,

        angular_velocity_x DOUBLE NOT NULL,
        angular_velocity_y DOUBLE NOT NULL,
        angular_velocity_z DOUBLE NOT NULL,

        acceleration_x DOUBLE NOT NULL,
        acceleration_y DOUBLE NOT NULL,
        acceleration_z DOUBLE NOT NULL,

        gyro_rotation_x DOUBLE NOT NULL,
        gyro_rotation_y DOUBLE NOT NULL,
        gyro_rotation_z DOUBLE NOT NULL,
        gyro_rotation_w DOUBLE NOT NULL,

        gyro_acceleration_x DOUBLE NOT NULL,
        gyro_acceleration_y DOUBLE NOT NULL,
        gyro_acceleration_z DOUBLE NOT NULL,

        gyro_angular_velocity_x DOUBLE NOT NULL,
        gyro_angular_velocity_y DOUBLE NOT NULL,
        gyro_angular_velocity_z DOUBLE NOT NULL,

        gyro_low_pass_filter_1 INTEGER NOT NULL references low_pass_filter (id),
        gyro_low_pass_filter_2 INTEGER NOT NULL references low_pass_filter (id),
        gyro_low_pass_filter_3 INTEGER NOT NULL references low_pass_filter (id)
    );
";

pub struct AscentDb {
    // Diesel conn for type safty
    diesel_conn: Arc<Mutex<SqliteConnection>>,
    // Rusqlte conn for speed
    rusqlite_conn: Arc<Mutex<Connection>>,
}

impl AscentDb {
    pub fn new(db_file: &str) -> Self {
        let diesel_conn = SqliteConnection::establish(db_file).unwrap();
        let rusqlite_conn = Connection::open(db_file).unwrap();
        rusqlite_conn.execute(ENSURE_FLIGHT_LOGS_QUERY, []).unwrap();
        // TODO: ensure flight logs
        Self {
            diesel_conn: Arc::new(Mutex::new(diesel_conn)),
            rusqlite_conn: Arc::new(Mutex::new(rusqlite_conn)),
        }
    }
}
