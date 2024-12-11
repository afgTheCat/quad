//! Used to log in the db
//! We probably need to revisit once we are more mature

use flight_controller::MotorInput;
use rusqlite::Connection;
use std::{ops::Range, sync::Mutex, time::Duration};

const ENSURE_FLIGHT_LOGS_QUERY: &str = "CREATE TABLE IF NOT EXISTS flight_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    simulation_id TEXT,
    start_seconds REAL,
    end_seconds REAL,
    motor_input_1 REAL,
    motor_input_2 REAL,
    motor_input_3 REAL,
    motor_input_4 REAL
);";

const INSERT_FLIGHT_LOGS_QUERY: &str = "
INSERT INTO flight_log (simulation_id, start_seconds, end_seconds, motor_input_1, motor_input_2, motor_input_3, motor_input_4)
VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)";

const SELECT_FLIGHT_LOGS_QUERY: &str = "
SELECT start_seconds, end_seconds, motor_input_1, motor_input_2, motor_input_3, motor_input_4 from flight_log 
where simulation_id = ?1 ORDER BY start_seconds
";

#[derive(Debug, Clone)]

pub struct FlightLog {
    pub range: Range<Duration>,
    pub motor_input: MotorInput,
}

pub struct AscentDb {
    conn: Mutex<Connection>,
}

impl AscentDb {
    pub fn new() -> Self {
        let conn = Connection::open("data.db").unwrap();
        conn.execute(ENSURE_FLIGHT_LOGS_QUERY, []).unwrap();
        Self {
            conn: Mutex::new(conn),
        }
    }

    pub fn write_flight_log(&self, simulation_id: &str, data: &[[f64; 6]]) {
        let mut conn = self.conn.lock().unwrap();
        // should be fast enough, but in the future we can do batch insert
        let tx = conn.transaction().unwrap();
        {
            let mut statement = tx.prepare(INSERT_FLIGHT_LOGS_QUERY).unwrap();
            for data in data {
                // super slow
                statement
                    .execute((
                        &simulation_id,
                        data[0],
                        data[1],
                        data[2],
                        data[3],
                        data[4],
                        data[5],
                    ))
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
    pub fn get_simuation_data(&self) -> Vec<FlightLog> {
        let Some(simulation_id) = self.get_simulation_id() else {
            return vec![];
        };
        let conn = self.conn.lock().unwrap();
        let mut statement = conn.prepare(SELECT_FLIGHT_LOGS_QUERY).unwrap();
        // let simulation_id = "c12ab1de-d0ba-4b4e-b096-00f5ad712c31";
        statement
            .query_map([simulation_id], |r| {
                let start_secs = Duration::from_secs_f64(r.get(0)?);
                let end_secs = Duration::from_secs_f64(r.get(1)?);
                Ok(FlightLog {
                    range: start_secs..end_secs,
                    motor_input: MotorInput {
                        input: [r.get(2)?, r.get(3)?, r.get(4)?, r.get(5)?],
                    },
                })
            })
            .unwrap()
            .map(Result::unwrap)
            .collect::<Vec<_>>()
    }
}

#[cfg(test)]
mod test {
    use crate::AscentDb;

    #[test]
    fn thing() {
        let db = AscentDb::new();
        let data = db.get_simuation_data();
        println!("{data:?}");
    }
}
