pub mod db_logger;
pub mod empty_logger;
pub mod file_logger;
pub mod rerun_logger;

use flight_controller::{BatteryUpdate, Channels, GyroUpdate, MotorInput};
use serde::{Deserialize, Serialize};
use std::{any::Any, time::Duration};

#[derive(Debug, Serialize, Deserialize)]
pub struct SnapShot {
    pub duration: Duration,
    pub motor_input: MotorInput,
    pub battery_update: BatteryUpdate,
    pub gyro_update: GyroUpdate,
    pub channels: Channels,
    // // TODO: this has a lot, I think this should be enough
    // pub current_frame: SimulationFrame,
}

// This is what we need to save
#[derive(Debug, Serialize, Deserialize)]
pub struct FlightLog {
    pub simulation_id: String,
    pub steps: Vec<SnapShot>,
}

pub trait Logger: Sync + Send + Any {
    fn log_time_stamp(&mut self, duration: Duration, snapshot: SnapShot);
}
