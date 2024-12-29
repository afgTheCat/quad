//! Used to log in the db
//! We probably need to revisit once we are more mature
//! TODO: we probably want to autogenerate this at one point

use derive_more::derive::Deref;
use flight_controller::{BatteryUpdate, Channels, GyroUpdate, MotorInput};
use nalgebra::DVector;
use rusqlite::{params, Connection};
use std::{ops::Range, sync::Mutex, time::Duration};

const ENSURE_FLIGHT_LOGS_QUERY: &str = "CREATE TABLE IF NOT EXISTS flight_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    simulation_id TEXT,
    start_seconds REAL,
    end_seconds REAL,
    motor_input_1 REAL,
    motor_input_2 REAL,
    motor_input_3 REAL,
    motor_input_4 REAL,
    battery_voltage_sag REAL,
    battery_voltage REAL,
    amperage REAL,
    mah_drawn REAL,
    cell_count INTEGER,
    rot_quat_x REAL,
    rot_quat_y REAL,
    rot_quat_z REAL,
    rot_quat_w REAL,
    linear_acceleration_x REAL,
    linear_acceleration_y REAL,
    linear_acceleration_z REAL,
    angular_velocity_x REAL,
    angular_velocity_y REAL,
    angular_velocity_z REAL,
    throttle REAL,
    roll REAL,
    pitch REAL,
    yaw REAL
);";

const INSERT_FLIGHT_LOGS_QUERY: &str = "
INSERT INTO flight_log (
    simulation_id,
    start_seconds,
    end_seconds,
    motor_input_1,
    motor_input_2,
    motor_input_3,
    motor_input_4,
    battery_voltage_sag,
    battery_voltage,
    amperage,
    mah_drawn,
    cell_count,
    rot_quat_x,
    rot_quat_y,
    rot_quat_z,
    rot_quat_w,
    linear_acceleration_x,
    linear_acceleration_y,
    linear_acceleration_z,
    angular_velocity_x,
    angular_velocity_y,
    angular_velocity_z,
    throttle,
    roll,
    pitch,
    yaw
)
VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13, ?14, ?15, ?16, ?17, ?18, ?19, ?20, ?21, ?22, ?23, ?24, ?25, ?26)";

const SELECT_FLIGHT_LOGS_QUERY: &str = "
SELECT
    start_seconds,
    end_seconds,
    motor_input_1,
    motor_input_2,
    motor_input_3,
    motor_input_4,
    battery_voltage_sag,
    battery_voltage,
    amperage,
    mah_drawn,
    cell_count,
    rot_quat_x,
    rot_quat_y,
    rot_quat_z,
    rot_quat_w,
    linear_acceleration_x,
    linear_acceleration_y,
    linear_acceleration_z,
    angular_velocity_x,
    angular_velocity_y,
    angular_velocity_z,
    throttle,
    roll,
    pitch,
    yaw
from flight_log where simulation_id = ?1 ORDER BY start_seconds
";

const SELECT_SIMULATION_IDS: &str = "SELECT DISTINCT simulation_id FROM flight_log";

#[derive(Debug, Clone)]
pub struct FlightLogEvent {
    pub range: Range<Duration>,
    pub motor_input: MotorInput,
    pub battery_update: BatteryUpdate,
    pub gyro_update: GyroUpdate,
    pub channels: Channels,
}

impl FlightLogEvent {
    pub fn to_rc_input(&self) -> DVector<f64> {
        let FlightLogEvent {
            battery_update:
                BatteryUpdate {
                    bat_voltage_sag,
                    bat_voltage,
                    amperage,
                    m_ah_drawn,
                    ..
                },
            gyro_update:
                GyroUpdate {
                    rotation,
                    linear_acc,
                    angular_velocity,
                },
            channels:
                Channels {
                    throttle,
                    roll,
                    pitch,
                    yaw,
                },
            ..
        } = self;
        DVector::from_row_slice(&[
            *bat_voltage_sag,
            *bat_voltage,
            *amperage,
            *m_ah_drawn,
            rotation[0],
            rotation[1],
            rotation[2],
            rotation[3],
            linear_acc[0],
            linear_acc[1],
            linear_acc[2],
            angular_velocity[0],
            angular_velocity[1],
            angular_velocity[2],
            *throttle,
            *roll,
            *yaw,
            *pitch,
        ])
    }

    pub fn to_rc_output(&self) -> DVector<f64> {
        let FlightLogEvent {
            motor_input: MotorInput { input },
            ..
        } = self;
        DVector::from_row_slice(input)
    }
}

#[derive(Debug, Deref, Default, Clone)]
pub struct FlightLog(Vec<FlightLogEvent>);

impl FlightLog {
    pub fn len(&self) -> usize {
        self.0.len()
    }
}

pub struct AscentDb {
    conn: Mutex<Connection>,
}

impl AscentDb {
    pub fn new(db_file: &str) -> Self {
        let conn = Connection::open(db_file).unwrap();
        conn.execute(ENSURE_FLIGHT_LOGS_QUERY, []).unwrap();
        Self {
            conn: Mutex::new(conn),
        }
    }

    pub fn write_flight_logs(&self, simulation_id: &str, flight_logs: &[FlightLogEvent]) {
        let mut conn = self.conn.lock().unwrap();
        // should be fast enough, but in the future we can do batch insert
        let tx = conn.transaction().unwrap();
        {
            let mut statement = tx.prepare(INSERT_FLIGHT_LOGS_QUERY).unwrap();
            for flight_log in flight_logs {
                statement
                    .execute(params![
                        &simulation_id,
                        flight_log.range.start.as_secs_f64(),
                        flight_log.range.end.as_secs_f64(),
                        flight_log.motor_input[0],
                        flight_log.motor_input[1],
                        flight_log.motor_input[2],
                        flight_log.motor_input[3],
                        flight_log.battery_update.bat_voltage_sag,
                        flight_log.battery_update.bat_voltage,
                        flight_log.battery_update.amperage,
                        flight_log.battery_update.m_ah_drawn,
                        flight_log.battery_update.cell_count,
                        flight_log.gyro_update.rotation[0],
                        flight_log.gyro_update.rotation[1],
                        flight_log.gyro_update.rotation[2],
                        flight_log.gyro_update.rotation[3],
                        flight_log.gyro_update.linear_acc[0],
                        flight_log.gyro_update.linear_acc[1],
                        flight_log.gyro_update.linear_acc[2],
                        flight_log.gyro_update.angular_velocity[0],
                        flight_log.gyro_update.angular_velocity[1],
                        flight_log.gyro_update.angular_velocity[2],
                        flight_log.channels.throttle,
                        flight_log.channels.roll,
                        flight_log.channels.pitch,
                        flight_log.channels.yaw,
                    ])
                    .unwrap();
            }
        }
        tx.commit().unwrap();
    }

    // just retrieves a simulation id, no guarantees
    fn get_simulation_id(&self) -> Option<String> {
        let guard = self.conn.lock().ok()?;
        let mut statement = guard
            .prepare("SELECT simulation_id FROM flight_log limit 1")
            .ok()?;
        // WTF xd this is supa retarded
        let id = statement.query([]).ok()?.next().ok()??.get(0).ok()?;
        id
    }

    // TODO: add the simulation id here
    pub fn get_simuation_data(&self, simulation_id: &str) -> FlightLog {
        let conn = self.conn.lock().unwrap();
        let mut statement = conn.prepare(SELECT_FLIGHT_LOGS_QUERY).unwrap();
        let events = statement
            .query_map([simulation_id], |r| {
                let start_secs = Duration::from_secs_f64(r.get(0)?);
                let end_secs = Duration::from_secs_f64(r.get(1)?);
                Ok(FlightLogEvent {
                    range: start_secs..end_secs,
                    motor_input: MotorInput {
                        input: [r.get(2)?, r.get(3)?, r.get(4)?, r.get(5)?],
                    },
                    battery_update: BatteryUpdate {
                        bat_voltage_sag: r.get(6)?,
                        bat_voltage: r.get(7)?,
                        amperage: r.get(8)?,
                        m_ah_drawn: r.get(9)?,
                        cell_count: r.get(10)?,
                    },
                    gyro_update: GyroUpdate {
                        rotation: [r.get(11)?, r.get(12)?, r.get(13)?, r.get(14)?],
                        linear_acc: [r.get(15)?, r.get(16)?, r.get(17)?],
                        angular_velocity: [r.get(18)?, r.get(19)?, r.get(20)?],
                    },
                    channels: Channels {
                        throttle: r.get(21)?,
                        roll: r.get(22)?,
                        pitch: r.get(23)?,
                        yaw: r.get(24)?,
                    },
                })
            })
            .unwrap()
            .map(Result::unwrap)
            .collect::<Vec<_>>();
        FlightLog(events)
    }

    pub fn get_all_simulation_ids(&self) -> Vec<String> {
        let conn = self.conn.lock().unwrap();
        let mut statement = conn.prepare(SELECT_SIMULATION_IDS).unwrap();
        statement
            .query_map([], |r| Ok(r.get(0)?))
            .unwrap()
            .map(Result::unwrap)
            .collect::<Vec<_>>()
    }

    pub fn get_first_simulation(&self) -> FlightLog {
        let simulation_ids = self.get_all_simulation_ids();
        self.get_simuation_data(&simulation_ids[0])
    }
}

#[cfg(test)]
mod test {
    use crate::{AscentDb, FlightLogEvent};
    use flight_controller::{BatteryUpdate, Channels, GyroUpdate, MotorInput};
    use std::time::Duration;

    #[test]
    fn insert_flight_log() {
        let db = AscentDb::new("data.db");
        let flight_log = FlightLogEvent {
            range: Duration::new(0, 0)..Duration::new(0, 0),
            motor_input: MotorInput::default(),
            battery_update: BatteryUpdate::default(),
            gyro_update: GyroUpdate::default(),
            channels: Channels::default(),
        };
        db.write_flight_logs("02855edd-0e00-467e-b352-36b9e8b323b4", &vec![flight_log])
    }
}
