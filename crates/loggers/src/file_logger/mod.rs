use crate::{FlightLog, Logger, SnapShot};
use std::{fs, path::PathBuf};

const LOG_PATH: &str = "/home/gabor/.local/share/quad/replays/";

pub struct FileLogger {
    simulation_id: String,
    snapshots: Vec<SnapShot>,
}

impl Logger for FileLogger {
    fn log_time_stamp(&mut self, snapshot: SnapShot) {
        self.snapshots.push(snapshot);
    }

    fn flush(&mut self) {
        let flight_log = FlightLog {
            simulation_id: self.simulation_id.clone(),
            steps: self.snapshots.clone(),
        };
        if self.snapshots.len() > 0 {
            let mut log_path = PathBuf::from(LOG_PATH);
            log_path.push(self.simulation_id.clone());
            let contents = serde_json::to_string(&flight_log).unwrap();
            fs::write(log_path, contents).unwrap();
        }
    }

    // fn set_simulation_id(&mut self, smulation_id: &str) {
    //     self.simulation_id = smulation_id.to_string()
    // }
}

impl FileLogger {
    pub fn new(simulation_id: String) -> Self {
        Self {
            simulation_id,
            snapshots: vec![],
        }
    }
}

impl Drop for FileLogger {
    fn drop(&mut self) {
        self.flush();
    }
}
